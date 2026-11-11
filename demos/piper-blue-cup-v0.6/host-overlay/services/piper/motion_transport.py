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

    def preview(self,start,target,duration_s,*,full_closure=False):
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
        if full_closure:
            require(self.allow_left_gripper and 0<=b[6]<a[6]<=.020,'Full closure must decrease gap within20mm')
            require(np.array_equal(a[JOINTS],b[JOINTS]),'Arm must stay fixed during full closure')
        else:require(abs(b[6]-a[6])<=.01,'left jaw step exceeds 10 mm')
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

    def preview_waypoints(self, start, waypoints, obstacles, blend_indices=()):
        from _rax_installed_piper.waypoint_plan import compile_arm_path
        require(isinstance(obstacles,list) and bool(obstacles),'Measured environment required')
        a=self.geometry._vector(start)
        require(isinstance(waypoints,list) and 1<=len(waypoints)<=16,'Expected 1..16 LEFT waypoints')
        targets=[]
        for left in waypoints:
            left=np.asarray(left,float)
            require(left.shape==(6,) and np.isfinite(left).all(),'Invalid LEFT waypoint')
            q=a.copy();q[:6]=left
            previous=a if not targets else np.asarray(targets[-1])
            require(np.max(abs(q[:6]-previous[:6]))<=self.limit+1e-9,'joint displacement exceeds operator scope')
            targets.append(q.tolist())
        def path(points):
            # Each quintic follows its reviewed straight joint-space leg. Check
            # the actual float32 wire as well, including inter-leg connectors.
            clear=self.collision.path(points,obstacles=obstacles)
            if blend_indices:self.check_wire_workspace(points)
            else:
                for x,y in zip([a.tolist(),*targets],targets):self.check_anchor_workspace(x,y)
            return {**clear,'workspace_clear':True}
        p=compile_arm_path(a.tolist(),targets,check_state=self.geometry._vector,
            check_path=path,dwell_s=self.dwell,velocity=self.velocity,acceleration=self.acceleration,blend_indices=blend_indices)
        p.pop('hardware_execution_enabled');p.pop('validation_scope')
        p.update(schema='rax.piper.multi_waypoint.v1',urdf_sha256=self.geometry.status()['urdf_sha256'],
            scene_sha256=digest(obstacles),obstacles=copy.deepcopy(obstacles),
            collision=p.pop('review'),physical_scene_accepted=False)
        p['sha256']=digest(p);self.approved_previews.add(p['sha256']);return p

    def progress(self):
        n=len(self.plan['targets14']) if self.plan else 0
        i=len(self.receipts)
        return dict(dispatched_samples=i,total_samples=n,fraction=i/n if n else 0,
            waypoint_index=sum(i>=v['sample_end'] for v in (self.plan or {}).get('waypoints',[])),
            feedback=getattr(self,'tracking_evidence',None),phase=self.state,
            recovery='Stop, inspect actual held-object state, explicit Prepare, fresh localization; no trajectory replay')

    def check_feedback_path(self,q,stamp):
        a=vector(self.plan['anchor14']);b=vector(self.plan['target14'])
        require(abs(q[13]-a[13])<=.001,'right gripper moved unexpectedly')
        require(min(a[6],b[6])-.001<=q[6]<=max(a[6],b[6])+.001,'left gripper left checked envelope')
        if self.plan.get('schema')!='rax.piper.multi_waypoint.v1':
            require(np.all(q[JOINTS]>=np.minimum(a,b)[JOINTS]-.015)
                and np.all(q[JOINTS]<=np.maximum(a,b)[JOINTS]+.015),'motion left checked joint envelope')
            return
        require(np.max(abs(q[7:13]-a[7:13]))<=self.joint_tolerance,'right arm moved unexpectedly')
        # Feedback acquisition precedes journal publication. Restrict candidates
        # by receipt time and a bounded 350 ms tracking lag, never the whole path.
        receipts=[x for x in self.receipts if x['recorded_ns']<=stamp+50_000_000]
        if self.state=='STAGED' or not receipts:
            points=np.asarray([a[:6]]);lo=hi=-1
        else:
            hi=receipts[-1]['model_chunk_step_index']
            recent=[x for x in receipts if x['recorded_ns']>=stamp-350_000_000]
            if not recent:
                require(self.state=='SETTLING' and hi==len(self.plan['targets14'])-1,'trajectory receipt progress interrupted')
                lo=hi
            else:lo=max(-1,recent[0]['model_chunk_step_index']-1)
            points=np.asarray([a[:6] if j<0 else self.plan['targets14'][j][:6] for j in range(lo,hi+1)])
        error=float(np.min(np.max(abs(points-q[:6]),axis=1)))
        for x,y in zip(points,points[1:]):
            d=y-x;u=np.clip(np.dot(q[:6]-x,d)/max(float(np.dot(d,d)),1e-20),0,1)
            error=min(error,float(np.max(abs(q[:6]-(x+u*d)))))
        self.tracking_evidence=dict(sample_min=lo,sample_max=hi,max_error_rad=error,
            corridor_rad=.015,max_tracking_lag_s=.35,feedback_timestamp_ns=stamp)
        require(error<=.015,'feedback outside dispatched trajectory progress corridor')

    def check_wire_workspace(self,points):
        import json
        from pathlib import Path
        from _rax_installed_piper.geometry import Kinematics
        opts=json.loads((Path(self.lease_file).parent.parent/'backend-options.json').read_text())
        low=np.asarray(opts['workspace_min']);high=np.asarray(opts['workspace_max']);kin=Kinematics()
        # Check endpoints and interpolated midpoints of every changed wire segment.
        for a,b in zip(points,points[1:]):
            for u in (0.,.5,1.):
                q=np.asarray(a[:6])*(1-u)+np.asarray(b[:6])*u
                p=kin.fk(q.tolist())[:3,3]
                require(np.all(p>=low) and np.all(p<=high),'Blended wire path leaves workspace')

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
            self.plan=copy.deepcopy(plan);self.tracking_evidence=None;self.binding=(observation['policy_session_id'],observation['generation'])
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
                    clear=self.collision.path([o['q'],targets[0]],obstacles=self.plan.get('obstacles',self.obstacles))
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
                if self.plan.get('schema')=='rax.piper.multi_waypoint.v1':
                    require(type(r.get('recorded_ns')) is int and r['recorded_ns']<=self.clock()+50_000_000,'invalid receipt timestamp')
                    if self.receipts:require(r['recorded_ns']>=self.receipts[-1]['recorded_ns'],'receipt timestamp reversed')
                self.receipts.append(copy.deepcopy(r))
                if len(self.receipts)==release['steps']:self.state='SETTLING'
            except Exception as exc:self.stop(str(exc))

    def observe_feedback(self,o):
        with self.lock:
            if self.state not in ACTIVE:return
            try:
                self._validate_obs(o);q=vector(o['q']);a=vector(self.plan['anchor14']);b=vector(self.plan['target14'])
                self.check_feedback_path(q,o['timestamp_ns'])
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
