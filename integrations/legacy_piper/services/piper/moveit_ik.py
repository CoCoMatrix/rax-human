"""Persistent pinned MoveIt2 process. Pure IK; no SDK, dispatch, or fallback."""
import atexit
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import threading
import time
import uuid
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from .calibration_cartesian import CartesianJog

CONFIG=Path(__file__).with_name('moveit-runtime.json')
_client=None
_creation=threading.Lock()

class MoveItClient:
    def __init__(self, config=CONFIG):
        self.lock=threading.Lock();self.process=None
        self.manifest=json.loads(Path(config).read_text())
        for path,digest in self.manifest['files'].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:
                raise ValueError('MOVEIT_SOURCE_IDENTITY_CHANGED:'+path)
        self.identity=hashlib.sha256(Path(config).read_bytes()).hexdigest()
        base=Path(self.manifest['root']);prefix=base/'env'
        self.log=(base/'live-ik.log').open('a')
        env={**os.environ,'AMENT_PREFIX_PATH':str(prefix),'LD_LIBRARY_PATH':str(prefix/'lib'),
             'ROS_DOMAIN_ID':'232','ROS_LOCALHOST_ONLY':'1','RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp',
             'ROS_LOG_DIR':str(base/'ros-logs'),'TMPDIR':'/home/ubuntu/code_junqi/tmp'}
        self.process=subprocess.Popen([self.manifest['binary'],self.manifest['urdf'],self.manifest['srdf']],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,env=env,bufsize=0)
        try:
            ready=self._read(15)
            if ready.get('ready') is not True or ready.get('joint_names') != ['left_joint'+str(i) for i in range(1,7)]:
                raise ValueError('MOVEIT_HANDSHAKE')
        except BaseException:self.close();raise
        self.kinematics=CartesianJog();atexit.register(self.close)

    def _read(self, timeout):
        deadline=time.monotonic()+timeout;data=b''
        while not data.endswith(b'\n'):
            remaining=deadline-time.monotonic()
            if remaining<=0 or not select.select([self.process.stdout],[],[],remaining)[0]:
                self.close();raise TimeoutError('MOVEIT_PROCESS_TIMEOUT_NO_MOTION_SUBMITTED')
            byte=os.read(self.process.stdout.fileno(),1)
            if not byte:raise RuntimeError('MOVEIT_PROCESS_EXITED')
            data+=byte
            if len(data)>16384:raise ValueError('MOVEIT_RESPONSE_SIZE')
        result=yaml.safe_load(data)
        if not isinstance(result,dict):raise ValueError('MOVEIT_RESPONSE_TYPE')
        return result

    def solve(self, position, rotation, seed, max_joint_delta=None):
        p=np.asarray(position,float);R=np.asarray(rotation,float);q=np.asarray(seed,float)
        if p.shape!=(3,) or R.shape!=(3,3) or q.shape!=(6,) or not all(np.isfinite(x).all() for x in (p,R,q)):
            raise ValueError('MOVEIT_FINITE_POSE_REQUIRED')
        if not np.allclose(R.T@R,np.eye(3),atol=1e-7) or abs(np.linalg.det(R)-1)>1e-7:raise ValueError('MOVEIT_ROTATION')
        k=self.kinematics;k.vector(q)
        req={'id':uuid.uuid4().hex,'position':p.tolist(),'quaternion_xyzw':Rotation.from_matrix(R).as_quat().tolist(),'seed':q.tolist(),'timeout_s':.1}
        if max_joint_delta is not None:
            if not np.isfinite(max_joint_delta) or not 0<max_joint_delta<=np.deg2rad(60):raise ValueError('MOVEIT_JOINT_CAP')
            req['consistency_limits']=[float(max_joint_delta)]*6
        with self.lock:
            if self.process is None or self.process.poll() is not None:raise RuntimeError('MOVEIT_UNAVAILABLE_NO_LEGACY_FALLBACK')
            self.process.stdin.write((json.dumps(req,allow_nan=False)+'\n').encode());self.process.stdin.flush()
            result=self._read(1.5)
        if result.get('id')!=req['id'] or result.get('hardware_writes')!=0:raise ValueError('MOVEIT_REPLY_IDENTITY')
        if result.get('ok') is not True:raise ValueError(str(result.get('error'))+'; no acceptable full-pose IK within unchanged joint limits')
        target=k.vector(result['q']);T=k.fk(target)
        pe=float(np.linalg.norm(T[:3,3]-p));re=float(Rotation.from_matrix(R@T[:3,:3].T).magnitude())
        if pe>.0002 or re>.002:raise ValueError('MOVEIT_INDEPENDENT_FK_DISAGREEMENT')
        if max_joint_delta is not None and np.max(abs(target-q))>max_joint_delta+1e-8:raise ValueError('MOVEIT_JOINT_CAP_VIOLATION')
        margins=np.minimum(target-k.limits[:,0],k.limits[:,1]-target)
        return {**result,'solver_converged':True,'position_error_m':pe,'orientation_error_rad':re,
                'error':float(np.hypot(pe*5,re)),'flange':T.tolist(),'manifest_sha256':self.identity,
                'active_joint_limits':['left_joint'+str(i+1) for i,v in enumerate(margins) if v<1e-5],
                'joint_limit_margin_rad':margins.tolist(),'executed':False,'physical_clearance_verified':False}

    def close(self):
        p=self.process;self.process=None
        if p is not None:
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:p.kill();p.wait(timeout=3)
            if p.stdin:p.stdin.close()
            if p.stdout:p.stdout.close()
        if hasattr(self,'log'):self.log.close()

def client():
    global _client
    with _creation:
        if _client is None:_client=MoveItClient()
    return _client
