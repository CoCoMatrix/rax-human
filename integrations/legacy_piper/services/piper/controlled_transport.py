"""Operator-side bounded commissioning transport. No SDK, CAN, auto-arm or start.

Prepared code only: importing it performs no I/O. Existing RAX NO_SEND backend
is unchanged. A future owner must use bounded review and gripper writes OFF.
"""
import copy
import hashlib
import json
import math
import threading
import time
import uuid
from pathlib import Path
import urllib.request
import numpy as np
from .dispatch import action_schema, digest

FROZEN = [6,7,8,9,10,11,12,13]

def require(ok, reason):
    if not ok: raise ValueError(reason)

def vector(q):
    a=np.asarray(q)
    require(a.shape==(14,) and a.dtype.kind in 'fi' and np.isfinite(a).all(), 'finite measured 14-vector required')
    return a.astype(float)

class StopOnlyHTTP:
    """One allowlisted mutation: Stop. Never Prepare, Start, enable or reconnect.
    A 200 response is queue acknowledgement, NOT a physical-stop receipt.
    Token is supplied privately by operator; no discovery or logging.
    """
    def __init__(self, token, *, port=8091):
        require(type(port) is int and port==8091,'pinned owner port required')
        require(isinstance(token,str) and 32<=len(token)<=256 and token.isascii() and not any(c.isspace() for c in token),'invalid private operator token')
        self.token=token;self.session=hashlib.sha256(token.encode()).hexdigest()
        self.base=f'http://127.0.0.1:{port}'
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self,*args,**kwargs): raise ValueError('owner redirect refused')
        self.http=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect())
    def _request(self,path,payload=None):
        request=urllib.request.Request(self.base+path,method='GET' if payload is None else 'POST',
            data=None if payload is None else json.dumps(payload).encode(),
            headers={'Content-Type':'application/json','X-XPolicy-Operator-Token':self.token})
        with self.http.open(request,timeout=.3) as response:
            require(response.headers.get('X-XPolicy-Operator-Session')==self.session,'owner session changed')
            raw=response.read(1_000_001);require(len(raw)<=1_000_000,'owner response too large')
        return json.loads(raw)
    def state(self): return self._request('/api/state')
    def __call__(self):
        self.state()  # Session identity before mutation, not after alone.
        r=self._request('/api/command',{'command':'stop','source':'keyboard_ui'})
        require(r.get('ok') is True and r.get('command')=='stop','Stop not acknowledged')
        return {'stop_request_acknowledged':True,'physical_stop_confirmed':False}

class ReceiptTail:
    """Read once, preserve partial lines; refuse truncation/replacement."""
    def __init__(self,path,*,max_line_bytes=65536):
        require(type(max_line_bytes) is int and 65536<=max_line_bytes<=524288,'invalid receipt line bound')
        self.max_line_bytes=max_line_bytes
        self.path=Path(path);self.offset=0;self.identity=None
        if self.path.exists():
            s=self.path.stat();self.identity=(s.st_dev,s.st_ino);self.offset=s.st_size
    def read(self):
        require(self.path.exists() or self.identity is None,'receipt journal disappeared')
        if not self.path.exists():return []
        out=[]
        with self.path.open('rb') as f:
            import os
            s=os.fstat(f.fileno());ident=(s.st_dev,s.st_ino)
            require(self.identity in (None,ident) and s.st_size>=self.offset,'receipt journal replaced/truncated')
            self.identity=ident;f.seek(self.offset)
            for _ in range(1000):
                line=f.readline(self.max_line_bytes+1);require(len(line)<=self.max_line_bytes,'oversize receipt')
                if not line.endswith(b'\n'):break
                value=json.loads(line);require(isinstance(value,dict),'invalid receipt');out.append(value);self.offset=f.tell()
            else:raise ValueError('receipt backlog budget exceeded')
        return out

class ControlledTransport:
    """1 reviewed left-joint probe, max 3s/.01rad; right and jaws unchanged.
    Exact wire observation validation is supplied by CommissioningPolicy below.
    arm_check must read the official operator arming file on every call.
    The caller independently polls watchdog every <=50ms; Stop HTTP is bounded.
    """
    def __init__(self,geometry,collision,*,arm_check,stop_client,clock_ns=time.time_ns):
        self.geometry=geometry;self.collision=collision;self.arm_check=arm_check
        self.stop_client=stop_client;self.clock=clock_ns;self.lock=threading.RLock()
        self.state='IDLE';self.plan=None;self.binding=None;self.active=None
        self.next_index=0;self.receipts=[];self.used=set();self.last_observed_ns=None
        self.stop_report={};self.result=None;self.stop_requested_ns=None
        self.events=[];self.settle_since=None;self.preview_hashes=set();self.chunk_arrival=None
    def _event(self,kind,**kw):self.events.append(dict(time_ns=self.clock(),event=kind,**kw))
    def preview(self,start,target,duration_s=3.0,*,obstacles=(),native_single_target=False):
        start=vector(start);target=vector(target)
        require(type(duration_s) in (int,float) and math.isfinite(duration_s) and 1<=duration_s<=3,'duration 1..3 seconds')
        require(np.array_equal(start[FROZEN],target[FROZEN]),'right arm and both jaws must be unchanged')
        changed=np.flatnonzero(abs(target[:6]-start[:6])>1e-12)
        require(len(changed)==1 and np.max(abs(target-start))<=.01,'exactly one left joint, max .01 rad')
        for q in (start,target):self.geometry._vector(q.tolist())
        n=round(duration_s*30);u=np.arange(1,n+1)/n;s=10*u**3-15*u**4+6*u**5
        wire=(start+s[:,None]*(target-start)).astype('<f4')
        for q in wire:self.geometry._vector(q.astype(float).tolist())
        delta=np.diff(np.vstack([start,wire.astype(float)]),axis=0)
        require(np.max(abs(delta[:,:6]))*30<=.02,'probe speed exceeded')
        require(np.max(abs(np.diff(np.vstack([np.zeros(14),delta,np.zeros(14)]),axis=0)[:,:6]))*900<=.05,'probe acceleration exceeded')
        c=self.collision.path(np.vstack([start,wire.astype(float)]),obstacles=obstacles)
        require(c['cad_clear'],'CAD collision refusal')
        if native_single_target:
            require(np.max(abs(target-start))<=.005000001,'native single target max .005 rad')
            # Same checked spatial path, one native position target. Hardware
            # speed is separately pinned to 1%; no host trajectory timing claim.
            wire=target.astype('<f4')[None,:]
        value=dict(schema='rax.piper.left_probe.v1',anchor14=start.tolist(),targets14=wire.tolist(),duration_s=None if native_single_target else n/30,
                   dispatch_mode='native_single_target' if native_single_target else 'quintic_stream',
                   urdf_sha256=self.geometry.status()['urdf_sha256'],scene_sha256=digest(list(obstacles)),
                   collision_report=c,physical_calibration_validated=False,dispatch_authorized=False)
        value['sha256']=digest(value)
        require(len(self.preview_hashes)<16,'preview budget exhausted')
        self.preview_hashes.add(value['sha256'])
        return value
    def stage(self,plan,approval,observation):
        with self.lock:
            require(self.state=='IDLE','one probe per transport; new instance after stop/completion')
            body=copy.deepcopy(plan);sha=body.pop('sha256');require(digest(body)==sha,'plan hash mismatch')
            # Only this process's exact previously checked path is releasable.
            # CAD work must finish BEFORE acquiring the <300 ms live anchor;
            # repeating it here makes a fresh observation stale during staging.
            require(sha in self.preview_hashes,'plan was not previewed by this transport')
            targets=np.asarray(body['targets14'],dtype=float);anchor=vector(body['anchor14'])
            require(np.array_equal(targets[:,FROZEN].astype('<f4'),np.tile(anchor[FROZEN].astype('<f4'),(len(targets),1))),'frozen axes altered')
            require(body['scene_sha256']==digest(list(approval.get('obstacles',()))) and body['urdf_sha256']==self.geometry.status()['urdf_sha256'],'asset/scene identity mismatch')
            require(approval.get('plan_sha256')==sha and approval.get('kind')=='left_joint_identification','approval not bound to probe')
            for key in ('operator_present','workspace_clear_for_probe','controller_stop_procedure_reviewed','owner_bounded_review_enabled','owner_gripper_writes_disabled','owner_model_filter_disabled','owner_rtc_disabled'):
                require(approval.get(key) is True,'missing operator prerequisite: '+key)
            now=self.clock();expiry=approval.get('expires_ns')
            require(type(expiry) is int and now<expiry<=now+15_000_000_000,'approval expiry invalid')
            require(self.arm_check().get('armed') is True,'operator disarmed')
            self._validate_obs(observation)
            require(approval.get('generation')==observation['generation'] and approval.get('policy_session_id')==observation['policy_session_id'],'approval owner mismatch')
            require(np.max(abs(vector(observation['q'])-anchor))<=.0005,'planning anchor drift')
            self.plan=copy.deepcopy(plan);self.approval=copy.deepcopy(approval)
            self.binding=(observation['policy_session_id'],observation['generation'])
            self.last_observed_ns=observation['timestamp_ns'];self.state='STAGED';self._event('staged',plan_sha256=sha)
    def _validate_obs(self,o):
        require(isinstance(o,dict),'observation required');vector(o['q'])
        stamp=o['timestamp_ns'];now=self.clock()
        require(type(stamp) is int and 0<=now-stamp<=300_000_000,'stale/future feedback')
        require(type(o['generation']) is int and o['generation']>=0 and isinstance(o['policy_session_id'],str) and o['policy_session_id'],'owner identity missing')
        if self.binding is not None:require((o['policy_session_id'],o['generation'])==self.binding,'owner generation/session changed')
    def _guards(self):
        require(self.arm_check().get('armed') is True,'operator disarmed/expired')
        require(self.clock()<self.approval['expires_ns'],'probe approval expired')
        require(self.last_observed_ns is not None and 0<=self.clock()-self.last_observed_ns<=300_000_000,'feedback stream interrupted')
    def observe_feedback(self,observation):
        """Passive owner telemetry continues while a reviewed prefix executes."""
        with self.lock:
            if self.state not in ('STAGED','RUNNING','SETTLING'):return
            try:
                self._validate_obs(observation)
                q=vector(observation['q']);anchor=np.asarray(self.plan['anchor14'])
                require(np.max(abs(q[7:13]-anchor[7:13]))<=.001,'right arm drift')
                require(np.max(abs(q[[6,13]]-anchor[[6,13]]))<=.001,'gripper drift')
                require(np.max(abs(q[:6]-anchor[:6]))<=.015,'probe left-arm envelope exceeded')
                self.last_observed_ns=max(self.last_observed_ns or 0,observation['timestamp_ns'])
            except Exception as exc:self.stop('passive feedback: '+str(exc))
    def tick(self,observation):
        with self.lock:
            if self.state not in ('STAGED','RUNNING','SETTLING'):return None
            try:
                self._validate_obs(observation);self.last_observed_ns=observation['timestamp_ns'];self._guards()
                q=vector(observation['q']);anchor=np.asarray(self.plan['anchor14']);targets=np.asarray(self.plan['targets14'])
                require(np.max(abs(q[7:13]-anchor[7:13]))<=.001,'right arm drift')
                require(np.max(abs(q[[6,13]]-anchor[[6,13]]))<=.001,'gripper drift')
                if self.active is not None:
                    require(self.clock()<self.active['expires_ns']+300_000_000,'missing dispatch receipt timeout');return None
                if self.chunk_arrival is not None:
                    arrival=self.chunk_arrival
                    require(self.clock()-arrival['started_ns']<=2_000_000_000,'chunk measured arrival timeout')
                    error=float(np.max(abs(q[:6]-np.asarray(arrival['target'])[:6])))
                    if error>.0003:
                        arrival['stable_ns']=None
                        return None
                    if arrival['stable_ns'] is None:arrival['stable_ns']=self.clock()
                    if self.clock()-arrival['stable_ns']<100_000_000:return None
                    self._event('chunk_measured_arrival',joint_error_max_rad=error)
                    self.chunk_arrival=None
                if self.next_index==len(targets):
                    self.state='SETTLING'
                    if self.settle_since is None:self.settle_since=self.clock()
                    err=float(np.max(abs(q[:6]-targets[-1,:6])))
                    if err<=.001:
                        if not hasattr(self,'stable_since'):self.stable_since=self.clock()
                        if self.clock()-self.stable_since>=200_000_000:
                            self.result=dict(sdk_completed_steps=len(self.receipts),measured_arrival_within_tolerance=True,joint_error_max_rad=err,object_success=False)
                            self.stop('probe complete; do not automatically return')
                    else:
                        if hasattr(self,'stable_since'):del self.stable_since
                    require(self.clock()-self.settle_since<=2_000_000_000,'arrival timeout')
                    return None
                # No observation arrives from PolicyServer while its prefix is
                # executing. Keep it to 3 frames (<100 ms), below the unchanged
                # 300 ms feedback watchdog. Longer motion requires a separate
                # fresh owner feedback channel, not a relaxed watchdog.
                chunk=targets[self.next_index:self.next_index+3]
                require(np.max(abs(chunk[0,:6]-q[:6]))<=.005,'tracking error before release')
                r=dict(schema='junqi.piper.bounded_review_release.v1',release_id=uuid.uuid4().hex,teacher_request_id=self.plan['sha256'],
                       execution_session_id=self.binding[0],generation=self.binding[1],steps=len(chunk),
                       review_observation_timestamp_ns=observation['timestamp_ns'],execution_observation_timestamp_ns=observation['timestamp_ns'],
                       expires_ns=observation['timestamp_ns']+300_000_000+int((len(chunk)-1)*1e9/30),
                       anchor14=q.tolist(),targets14=chunk.tolist(),scene_check='fresh_three_cameras_and_state_match_image_comparison_disabled_by_operator')
                # Explicit operator reviewed this one small joint path; no claimed photometric collision check.
                require(self.approval.get('scene_review_mode')=='operator_manual_static_probe','scene review not explicitly selected')
                self.active=r;self.active_receipts=0;self.next_index+=len(chunk);self.state='RUNNING';self._event('release',release_id=r['release_id'],steps=len(chunk))
                return copy.deepcopy(r)
            except Exception as exc:self.stop(str(exc));return None
    def receipt(self,r):
        with self.lock:
            if self.state not in ('RUNNING','SETTLING'):return
            try:
                require(r.get('event')!='release_rejected','owner rejected release')
                if r.get('event')!='sdk_dispatch_completed':return
                a=self.active;require(a is not None and r.get('review_release_id')==a['release_id'],'unexpected receipt identity')
                i=self.active_receipts
                require(r.get('teacher_request_id')==self.plan['sha256'] and r.get('generation')==self.binding[1] and r.get('model_chunk_step_index')==i,'receipt identity/order mismatch')
                require(r.get('hardware_dispatch_confirmed') is True,'SDK dispatch not confirmed')
                require(np.array_equal(np.asarray(vector(r['applied14']),dtype='<f4'),np.asarray(a['targets14'][i],dtype='<f4')),'owner changed reviewed target')
                self.receipts.append(copy.deepcopy(r));self.active_receipts+=1
                if self.active_receipts==a['steps']:
                    self.used.add(a['release_id'])
                    # SDK acceptance is not physical arrival. Do not anchor the
                    # next release while this small joint move is still catching
                    # up; retain the owner's existing 0.001 rad anchor gate.
                    if self.plan.get('dispatch_mode')!='native_single_target':
                        self.chunk_arrival=dict(target=a['targets14'][-1],started_ns=self.clock(),stable_ns=None)
                    self.active=None
            except Exception as exc:self.stop(str(exc))
    def stop(self,reason):
        with self.lock:
            if self.state in ('STOP_REQUESTED','STOP_OBSERVED','STOP_UNCONFIRMED'):return self.stop_report
            self.state='STOP_REQUESTED';self.active=None;self.stop_requested_ns=self.clock();self._event('cancel_all',reason=reason)
            self.stop_report=dict(reason=reason,pending_cleared=True,stop_request_acknowledged=False,physical_stop_confirmed=False)
        # Release lock before bounded network call; no future tick can release an action.
        try:self.stop_report.update(self.stop_client())
        except Exception as exc:self.stop_report['transport_error']=type(exc).__name__;self.state='STOP_UNCONFIRMED'
        return copy.deepcopy(self.stop_report)
    def observe_stop(self,state):
        """Requires fresh owner state AFTER Stop and a generation/lifecycle transition.
        This establishes owner queue invalidation only, NOT mechanical rest.
        """
        with self.lock:
            if self.state not in ('STOP_REQUESTED','STOP_UNCONFIRMED'):return False
            try:
                stamp=state['timestamp_ns'];now=self.clock()
                require(type(stamp) is int and self.stop_requested_ns<stamp<=now and now-stamp<=300_000_000,'stale stop state')
                require(self.binding is not None and state['generation']>self.binding[1] and state['lifecycle_state'] in ('idle','paused','stopped'),'owner has not invalidated generation')
                self.state='STOP_OBSERVED';self.stop_report['owner_queue_stop_observed']=True;return True
            except (KeyError,ValueError,TypeError):return False
    def watchdog(self):
        with self.lock:
            if self.state in ('STAGED','RUNNING','SETTLING'):
                try:
                    self._guards()
                    if self.active is not None:require(self.clock()<self.active['expires_ns']+300_000_000,'receipt timeout')
                except Exception as exc:self.stop(str(exc))
            elif self.state=='STOP_REQUESTED' and self.clock()-self.stop_requested_ns>1_000_000_000:
                self.state='STOP_UNCONFIRMED';self._event('stop_unconfirmed',operator_action='use physical emergency stop; do not restart')
    def wait_payload(self):
        return dict(result_kind='no_action',actions=np.empty((0,14),np.float32),action_schema=action_schema(),action_frequency_hz=30,
          policy_metadata=dict(execute_prefix_steps=0,inference_executed=False,hardware_execution_confirmed=False,transport_receipt=dict(mode='continuous_review_wait',request_id='rax-controlled',status=self.state,dispatch_authorized=False)))
