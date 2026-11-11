"""RAX joint moves through the formal 30 Hz ActionChunk/review gate, no SDK.

Both arms are addressable. Jaw targets remain unchanged until independently
qualified. Motion success requires exact owner receipts AND measured arrival.
"""
import copy
import math
import threading
import time
import uuid
import numpy as np
from services.json_io import write_private_json
from .controlled_transport import require,vector
from .dispatch import action_schema,digest
from .installed_settling import state_matches, JOINT_STATE_TOLERANCE_RAD
from .motion_profiles import LEFT_JAW_COMMAND_MAX_M,LEFT_JAW_ARRIVAL_TOLERANCE_M

JOINTS=[0,1,2,3,4,5,7,8,9,10,11,12]
ACTIVE=('STAGED','RUNNING','SETTLING')


class MotionTransport:
    general_joint_motion=True
    def __init__(self,geometry,collision,*,arm_check,stop_client,lease_file,
                 max_displacement_rad=.02,joint_tolerance_rad=.001,allow_left_gripper=False,obstacles=(),clock_ns=time.time_ns,
                 max_velocity_rad_s=.02,max_acceleration_rad_s2=.05,terminal_dwell_s=2,tracking_velocity_ratio=1.):
        require(type(max_displacement_rad) in (int,float) and math.isfinite(max_displacement_rad)
                and 0<max_displacement_rad<=math.radians(60),'joint displacement cap must be positive and <=60 degrees')
        require(type(max_velocity_rad_s) in (int,float) and math.isfinite(max_velocity_rad_s) and 0<max_velocity_rad_s<=math.radians(20),'velocity cap must be positive and <=20 degrees/s')
        require(type(max_acceleration_rad_s2) in (int,float) and math.isfinite(max_acceleration_rad_s2) and 0<max_acceleration_rad_s2<=math.radians(40),'acceleration cap must be positive and <=40 degrees/s²')
        require(type(terminal_dwell_s) in (int,float) and math.isfinite(terminal_dwell_s) and .2<=terminal_dwell_s<=2,'terminal dwell must be .2..2 seconds')
        require(type(tracking_velocity_ratio) in (int,float) and math.isfinite(tracking_velocity_ratio) and .5<=tracking_velocity_ratio<=1,'invalid velocity tracking reserve')
        self.velocity=max_velocity_rad_s*tracking_velocity_ratio;self.acceleration=max_acceleration_rad_s2
        self.dwell=math.ceil(terminal_dwell_s*30)/30
        require(type(joint_tolerance_rad) in (int,float) and math.isfinite(joint_tolerance_rad)
                and .001<=joint_tolerance_rad<=JOINT_STATE_TOLERANCE_RAD,'joint tolerance must be 0.001..0.01 rad')
        self.joint_tolerance=joint_tolerance_rad
        require(type(allow_left_gripper) is bool,'left gripper enable must be boolean')
        self.allow_left_gripper=allow_left_gripper
        self.geometry=geometry;self.collision=collision;self.arm_check=arm_check;self.stop_client=stop_client
        self.lease_file=lease_file;self.limit=max_displacement_rad;self.obstacles=copy.deepcopy(list(obstacles));self.clock=clock_ns
        self.lock=threading.RLock();self.state='IDLE';self.binding=None;self.plan=None;self.release=None
        self.receipts=[];self.events=[];self.result=None;self.last_observed_ns=None;self.stable_since=None
        self.stop_report={};self.approved_previews=set();self.deadline=None;self.last_q=None

    def preview(self,start,target,duration_s):
        a=self.geometry._vector(start);b=self.geometry._vector(target)
        require(np.float32(a[13])==np.float32(b[13]),'right gripper must stay unchanged on the float32 wire')
        if not self.allow_left_gripper:
            require(np.float32(a[6])==np.float32(b[6]),'left gripper dispatch is not enabled')
        # A measured saturated gap may exceed the command cap by encoder/calibration
        # tolerance. Preserve it for arm-only moves, or close back inside range;
        # never treat that tolerance as extra commanded opening.
        unchanged=np.float32(a[6])==np.float32(b[6])
        require((unchanged and 0<=b[6]<=LEFT_JAW_COMMAND_MAX_M+LEFT_JAW_ARRIVAL_TOLERANCE_M)
                or 0<=b[6]<=LEFT_JAW_COMMAND_MAX_M+1e-8,'left jaw command exceeds physical 75 mm limit')
        require(abs(b[6]-a[6])<=.01,'left jaw step exceeds 10 mm')
        delta=abs(b-a)
        if duration_s is None:
            from .motion_profiles import movement_duration
            duration_s=movement_duration(delta,self.velocity,self.acceleration)
        require(type(duration_s) in (int,float) and math.isfinite(duration_s) and 1<=duration_s<=18,'duration 1..18 seconds required')
        duration_s=math.ceil(duration_s*30)/30
        require(np.max(delta[JOINTS])<=self.limit+1e-9,'joint displacement exceeds operator scope')
        require(1.875*delta[6]/duration_s<=.01,'left jaw speed exceeds 10 mm/s')
        require(np.max(delta)>1e-7,'target already equals current state')
        require(np.max(1.875*delta[JOINTS]/duration_s)<=self.velocity+1e-12,'joint speed cap exceeded; increase duration')
        require(np.max((10/np.sqrt(3))*delta[JOINTS]/duration_s**2)<=self.acceleration+1e-12,'joint acceleration cap exceeded; increase duration')
        # All coordinates follow the same monotonic quintic parameter. Its
        # geometric path is this straight segment; collision.path subdivides it.
        clear=self.collision.path([a.tolist(),b.tolist()],obstacles=self.obstacles)
        require(clear['cad_clear'],'whole-path CAD collision refusal')
        n=round(duration_s*30);u=np.arange(1,n+1)/n
        wire=(a+(10*u**3-15*u**4+6*u**5)[:,None]*(b-a)).astype('<f4')
        wire=np.concatenate((wire,np.repeat(wire[-1:],round(self.dwell*30),axis=0)))
        for q in wire:self.geometry._vector(q.astype(float).tolist())
        p=dict(schema='rax.piper.joint_motion.v1',anchor14=a.tolist(),target14=wire[-1].astype(float).tolist(),
               targets14=wire.tolist(),duration_s=len(wire)/30,movement_duration_s=n/30,terminal_dwell_s=self.dwell,
               urdf_sha256=self.geometry.status()['urdf_sha256'],
               scene_sha256=digest(self.obstacles),collision=clear,physical_scene_accepted=False)
        p['sha256']=digest(p);self.approved_previews.add(p['sha256'])
        return p

    def check_anchor_workspace(self,start,target):
        import json
        from pathlib import Path
        from _rax_installed_piper.geometry import Kinematics
        opts=json.loads((Path(self.lease_file).parent.parent/'backend-options.json').read_text())
        low=np.asarray(opts['workspace_min'],float);high=np.asarray(opts['workspace_max'],float)
        require(low.shape==high.shape==(3,) and np.isfinite([low,high]).all() and np.all(low<high),'invalid anchor workspace')
        kin=Kinematics();a=vector(start);b=vector(target)
        for u in np.linspace(0.,1.,41):
            point=kin.fk((a[:6]+u*(b[:6]-a[:6])).tolist())[:3,3]
            require(np.all(point>=low) and np.all(point<=high),'execution anchor connector leaves workspace')

    def _validate_obs(self,o):
        vector(o['q']);stamp=o['timestamp_ns']
        require(type(stamp) is int and 0<=self.clock()-stamp<=300_000_000,'stale/future feedback')
        require(type(o['generation']) is int and o['generation']>=0 and isinstance(o['policy_session_id'],str) and o['policy_session_id'],'owner identity missing')
        if self.binding is not None:require((o['policy_session_id'],o['generation'])==self.binding,'owner generation/session changed')

    def stage(self,plan,observation):
        with self.lock:
            require(self.state in ('IDLE','DONE'),'motion active or fault latched; operator recovery required')
            body=copy.deepcopy(plan);sha=body.pop('sha256')
            require(digest(body)==sha and sha in self.approved_previews,'unreviewed or modified motion plan')
            require(self.arm_check().get('armed') is True,'operator disarmed')
            self._validate_obs(observation)
            require(state_matches(observation['q'],plan['anchor14']),'planning anchor drift')
            self.plan=copy.deepcopy(plan);self.binding=(observation['policy_session_id'],observation['generation'])
            self.approved_previews.remove(sha);self.receipts=[];self.release=None;self.result=None
            self.stable_since=None;self.state='STAGED';self.last_observed_ns=observation['timestamp_ns']
            self.deadline=self.clock()+int((plan['duration_s']+4)*1e9);self.last_q=vector(observation['q'])
            self._lease('active')

    def _lease(self,phase):
        if self.plan is None:return
        write_private_json(self.lease_file,dict(schema='rax.piper.motion_lease.v1',timestamp_ns=self.clock(),
            phase=phase,request_id=self.plan['sha256'],policy_session_id=self.binding[0],generation=self.binding[1],target14=self.plan['target14'],joint_tolerance_rad=self.joint_tolerance),durable=False)

    def _guards(self):
        require(self.arm_check().get('armed') is True,'operator disarmed/expired')
        require(self.clock()<self.deadline,'motion arrival timeout')
        require(self.last_observed_ns is not None and 0<=self.clock()-self.last_observed_ns<=300_000_000,'feedback interrupted')

    def tick(self,o):
        with self.lock:
            if self.state not in ACTIVE:return None
            try:
                self._validate_obs(o);self.last_observed_ns=o['timestamp_ns'];self._guards()
                if self.state!='STAGED':return None
                require(state_matches(o['q'],self.plan['anchor14']),'execution anchor drift')
                targets=self.plan['targets14'];stamp=o['timestamp_ns']
                if np.max(abs(vector(o['q'])-vector(self.plan['anchor14'])))>1e-7:
                    clear=self.collision.path([o['q'],targets[0]],obstacles=self.obstacles)
                    require(clear['cad_clear'],'execution anchor connector collision refusal')
                    self.check_anchor_workspace(o['q'],targets[0])
                    self._validate_obs(o);self._guards()
                delta=abs(vector(o['q'])-vector(self.plan['anchor14']))
                self.anchor_validation=dict(joint_tolerance_rad=JOINT_STATE_TOLERANCE_RAD,jaw_tolerance_m=.0005,
                    max_joint_delta_rad=float(np.max(delta[JOINTS])),max_jaw_delta_m=float(np.max(delta[[6,13]])),
                    measured14=o['q'],planned_anchor14=self.plan['anchor14'],zero_filter_applied=False)
                r=dict(schema='junqi.piper.trajectory_release.v1',release_id=uuid.uuid4().hex,
                    teacher_request_id=self.plan['sha256'],execution_session_id=self.binding[0],generation=self.binding[1],
                    steps=len(targets),review_observation_timestamp_ns=stamp,execution_observation_timestamp_ns=stamp,
                    expires_ns=stamp+300_000_000+int((len(targets)-1)*1e9/30),anchor14=o['q'],targets14=targets,
                    scene_check='fresh_three_cameras_and_state_match_image_comparison_disabled_by_operator',
                    anchor_validation=self.anchor_validation)
                self.release=r;self.state='RUNNING';self._lease('active');return copy.deepcopy(r)
            except Exception as exc:self.stop(str(exc));return None

    def receipt(self,r):
        with self.lock:
            if self.state not in ACTIVE:return
            try:
                require(r.get('event')!='release_rejected','owner rejected release')
                if r.get('event')!='sdk_dispatch_completed':return
                i=len(self.receipts);release=self.release
                require(release is not None and i<release['steps'],'unexpected dispatch receipt')
                require(r.get('review_release_id')==release['release_id'] and r.get('teacher_request_id')==self.plan['sha256']
                        and r.get('generation')==self.binding[1] and r.get('model_chunk_step_index')==i,'receipt identity/order mismatch')
                require(r.get('hardware_dispatch_confirmed') is True,'SDK completion unconfirmed')
                require(np.array_equal(np.asarray(vector(r['applied14']),dtype='<f4'),np.asarray(release['targets14'][i],dtype='<f4')),'owner changed target')
                self.receipts.append(copy.deepcopy(r))
                if len(self.receipts)==release['steps']:self.state='SETTLING'
            except Exception as exc:self.stop(str(exc))

    def observe_feedback(self,o):
        with self.lock:
            if self.state not in ACTIVE:return
            try:
                self._validate_obs(o);q=vector(o['q']);a=vector(self.plan['anchor14']);b=vector(self.plan['target14'])
                require(abs(q[13]-a[13])<=.001,'right gripper moved unexpectedly')
                require(min(a[6],b[6])-.001<=q[6]<=max(a[6],b[6])+.001,'left gripper left checked envelope')
                require(np.all(q[JOINTS]>=np.minimum(a,b)[JOINTS]-.015)
                        and np.all(q[JOINTS]<=np.maximum(a,b)[JOINTS]+.015),'motion left checked joint envelope')
                self.last_observed_ns=o['timestamp_ns'];self._guards()
                err=float(np.max(abs(q[JOINTS]-b[JOINTS])))
                stationary=self.last_q is not None and state_matches(q,self.last_q,jaw_tolerance=.0003)
                self.last_q=q
                jaw_error=float(abs(q[6]-b[6]))
                if self.state=='SETTLING' and err<=self.joint_tolerance and jaw_error<=.001 and stationary:
                    if self.stable_since is None:self.stable_since=self.clock()
                    if self.clock()-self.stable_since>=300_000_000:
                        self.state='DONE';self.result=dict(executed=True,measured_arrival=True,joint_error_max_rad=err,joint_tolerance_rad=self.joint_tolerance,
                            sdk_completed_steps=len(self.receipts),final14=q.tolist(),left_jaw_error_m=jaw_error,object_success=None)
                        self._lease('done')
                else:self.stable_since=None
            except Exception as exc:self.stop(str(exc))

    def watchdog(self):
        with self.lock:
            if self.state in ACTIVE:
                try:self._guards();self._lease('active')
                except Exception as exc:self.stop(str(exc))
            elif self.state=='DONE':self._lease('done')

    def stop(self,reason):
        with self.lock:
            if self.state=='FAULT':return self.stop_report
            was_active=self.state in ACTIVE
            self.state='FAULT';self._lease('cancelled')
            self.stop_report=dict(reason=reason,physical_stop_confirmed=False)
            if was_active:
                try:self.stop_report.update(self.stop_client())
                except Exception as exc:self.stop_report['transport_error']=type(exc).__name__
            return copy.deepcopy(self.stop_report)

    def observe_stop(self,state):return False  # measured stop is assessed separately by the operator test

    def wait_payload(self):
        return dict(result_kind='no_action',actions=np.empty((0,14),np.float32),action_schema=action_schema(),action_frequency_hz=30,
            policy_metadata=dict(execute_prefix_steps=0,inference_executed=False,hardware_execution_confirmed=False,
                transport_receipt=dict(mode='continuous_review_wait',request_id='rax-motion',status=self.state,dispatch_authorized=False)))
