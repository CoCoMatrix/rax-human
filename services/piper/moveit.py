"""Persistent existing MoveIt KDL binary. Pure IK, no ROS motion or hardware writer."""
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
from scripts.site_paths import path as site_path
import yaml
from scipy.spatial.transform import Rotation
from .geometry import ASSETS, Kinematics, vector
from .bridge import RejectedBeforeDispatch


class IKNoSolution(RejectedBeforeDispatch):
    """A verified pure solver refusal; permits planning, never automatic dispatch."""
    recovery_action = 'replan_read_only'


class MoveIt:
    def __init__(self, log_root, manifest=None):
        self.manifest = json.loads(Path(manifest or site_path('moveit_manifest', ASSETS / 'moveit-runtime.json')).read_text())
        self.log_root = Path(log_root); self.process = None; self.lock = threading.Lock()
        self.kinematics = Kinematics(); self.log = None

    def start(self):
        if self.process is not None:
            if self.process.poll() is not None: raise RuntimeError('MoveIt exited; no automatic fallback')
            return
        for path, sha in self.manifest['files'].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest() != sha:
                raise ValueError('MoveIt dependency identity changed: ' + path)
        self.log_root.mkdir(parents=True, exist_ok=True)
        prefix = Path(self.manifest['root']) / 'env'
        self.log = (self.log_root / 'moveit.log').open('ab')
        env = {**os.environ, 'AMENT_PREFIX_PATH': str(prefix), 'LD_LIBRARY_PATH': str(prefix / 'lib'),
               'ROS_DOMAIN_ID': '232', 'ROS_LOCALHOST_ONLY': '1', 'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
               'ROS_LOG_DIR': str(self.log_root / 'ros'), 'TMPDIR': str(self.log_root)}
        self.process = subprocess.Popen([self.manifest['binary'], self.manifest['urdf'], self.manifest['srdf']],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log, env=env, bufsize=0)
        try:
            ready = self._read(15)
            if ready.get('ready') is not True or ready.get('joint_names') != [f'left_joint{i}' for i in range(1, 7)]:
                raise ValueError('MoveIt handshake mismatch')
        except BaseException:
            self.close(); raise

    def _read(self, timeout):
        deadline = time.monotonic() + timeout; data = b''
        while not data.endswith(b'\n'):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                self.close(); raise TimeoutError('MoveIt response timeout; no motion submitted')
            byte = os.read(self.process.stdout.fileno(), 1)
            if not byte: raise RuntimeError('MoveIt pipe closed')
            data += byte
            if len(data) > 16384: raise ValueError('MoveIt response too large')
        result = yaml.safe_load(data)
        if not isinstance(result, dict): raise ValueError('Invalid MoveIt response')
        return result

    def solve(self, position, quaternion_xyzw, seed):
        p = vector(position, 3); quat = vector(quaternion_xyzw, 4); q = self.kinematics.check(seed)
        if abs(np.linalg.norm(quat) - 1) > 1e-6: raise ValueError('Unit quaternion required')
        req = {'id': uuid.uuid4().hex, 'position': p.tolist(), 'quaternion_xyzw': quat.tolist(),
               'seed': q.tolist(), 'timeout_s': .1}
        with self.lock:
            self.start()
            self.process.stdin.write((json.dumps(req, allow_nan=False) + '\n').encode()); self.process.stdin.flush()
            result = self._read(1.5)
        if result.get('id') != req['id'] or result.get('hardware_writes') != 0: raise ValueError('MoveIt reply identity')
        if result.get('ok') is not True:
            if result.get('error') == 'MOVEIT_NO_IK_SOLUTION':
                raise IKNoSolution('MOVEIT_NO_IK_SOLUTION')
            raise ValueError(result.get('error', 'No full-pose IK'))
        target = self.kinematics.check(result['q']); t = self.kinematics.fk(target.tolist())
        pe = float(np.linalg.norm(t[:3, 3] - p))
        re = float(Rotation.from_matrix(Rotation.from_quat(quat).as_matrix() @ t[:3, :3].T).magnitude())
        if pe > .0002 or re > .002: raise ValueError('Independent FK verification failed')
        return {**result, 'position_error_m': pe, 'orientation_error_rad': re, 'executed': False,
                'scene_collision_checked': False, 'controlled_link': 'left_link6', 'frame': 'left_base_link'}

    def close(self):
        p = self.process; self.process = None
        if p is not None:
            if p.poll() is None:
                p.terminate()
                try: p.wait(timeout=3)
                except subprocess.TimeoutExpired: p.kill(); p.wait(timeout=3)
            if p.stdin: p.stdin.close()
            if p.stdout: p.stdout.close()
        if self.log: self.log.close(); self.log = None
