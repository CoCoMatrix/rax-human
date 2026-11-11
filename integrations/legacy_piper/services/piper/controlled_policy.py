"""Formal PolicyServer adapter for one operator-reviewed identification probe.

No GPT/SAM/FFS/task-planner dependency. contract is the existing pure PiperContract.
This module does not start a server or owner. Default output is NO_ACTION.
"""
import asyncio
import math
import time
import numpy as np
from .controlled_transport import require
from .owner import SERIALS,ORDER
from .dispatch import action_schema

class ControlledPolicy:
    def __init__(self,contract,transport,*,task,receipt_tail,operator_arm,owner_state=None,monotonic_ns=time.monotonic_ns):
        self.contract=contract;self.transport=transport;self.task=task;self.tail=receipt_tail
        self.operator_arm=operator_arm;self.owner_state=owner_state;self.previous_sequence=None
        self.latest=None;self.closed=False;self.last_owner_stop_poll=0;self.last_error=None
        self.feedback=None;self.pending_probe=None
        self.monotonic_ns=monotonic_ns;self.clock_sample=None
        self.timing_recovery=None;self.last_timing_event=None
    def _begin_timing_recovery(self,reason):
        # Only completed general-motion calls can wait for valid observations.
        # Commissioning and any active/staged motion retain their hard stop.
        t=self.transport
        if not getattr(t,'general_joint_motion',False) or t.state not in ('IDLE','DONE'):
            t.stop('observation/contract failure: '+reason);self.latest=None;return
        if self.timing_recovery is None:
            anchor=(t.result or {}).get('final14') if t.state=='DONE' else (self.latest or {}).get('q')
            self.timing_recovery=dict(started_mono_ns=self.monotonic_ns(),reason=reason,
                anchor=anchor,generation=None if t.binding is None else t.binding[1],
                stable_since=None,samples=0,last_feedback_stamp=None)
        else:
            self.timing_recovery.update(stable_since=None,samples=0,last_feedback_stamp=None)
        self.last_timing_event=dict(time_ns=t.clock(),reason=reason,state=t.state,recovered=False)
        self.latest=None
    def _sample_clock(self):
        before=self.monotonic_ns();wall=self.transport.clock();after=self.monotonic_ns()
        # A descheduled sampling thread is not evidence of a clock step.
        if after-before>5_000_000:
            self.clock_sample=None;return
        mono=(before+after)//2
        if self.clock_sample is not None:
            w,m=self.clock_sample;step=(wall-w)-(mono-m)
            if abs(step)>50_000_000:
                self._begin_timing_recovery('host wall-clock discontinuity: '+str(step)+' ns')
        self.clock_sample=(wall,mono)
    def _recovery_timeout(self):
        if self.timing_recovery is not None and self.monotonic_ns()-self.timing_recovery['started_mono_ns']>3_000_000_000:
            self.latest=None;self.transport.stop('observation freshness recovery timeout')
    def _recovery_feedback(self):
        r=self.timing_recovery;f=self.feedback
        if r is None or f is None:return
        stamp=f['timestamp_ns'];now=self.transport.clock()
        require(0<=now-stamp<=300_000_000,'recovery CAN feedback stale/future')
        require(f['lifecycle_state']=='running','recovery owner is not running')
        require(r['generation'] in (None,f['generation']),'recovery owner generation changed')
        r['generation']=f['generation']
        q=np.asarray(f['q'],dtype=float)
        require(q.shape==(14,) and np.isfinite(q).all(),'recovery state invalid')
        if r['anchor'] is None:r['anchor']=q.tolist()
        require(np.max(abs(q-np.asarray(r['anchor'])))<=.0003,'recovery measured drift')
        if r['stable_since'] is None:r['stable_since']=self.monotonic_ns()
        if stamp!=r['last_feedback_stamp']:r['samples']+=1;r['last_feedback_stamp']=stamp
    def _recovery_ready(self,normal):
        r=self.timing_recovery
        if r is None:return True
        self._recovery_timeout()
        if self.transport.state not in ('IDLE','DONE'):return False
        f=self.feedback
        if not f or r['stable_since'] is None or r['samples']<4:return False
        if self.monotonic_ns()-r['stable_since']<300_000_000:return False
        require(0<=self.transport.clock()-f['timestamp_ns']<=300_000_000,'recovery CAN feedback stale/future')
        require(normal['generation']==r['generation'],'recovery observation generation changed')
        require(np.max(abs(np.asarray(normal['q'])-np.asarray(r['anchor'])))<=.0003,'recovery observation drift')
        self.timing_recovery=None
        self.last_timing_event.update(recovered=True,recovered_ns=self.transport.clock())
        return True
    def get_capabilities(self):
        if getattr(self.transport,'general_joint_motion',False):
            return dict(mode='rax_dual_joint_motion',action_frequency_hz=30,horizon=600,
                        automatic_arming=False,automatic_owner_start=False,task_exploration_enabled=False)
        return dict(mode='rax_left_probe_prepared',action_frequency_hz=30,horizon=15,
                    automatic_arming=False,automatic_owner_start=False,task_exploration_enabled=False)
    def reset(self,**kwargs):
        # Initial handshake before any probe is allowed. Later reset never rearms.
        if self.transport.state!='IDLE':self.transport.stop('protocol RESET')
        self.latest=None;self.previous_sequence=None
        return dict(physical_reset=False,automatic_rearm=False)
    def infer(self,wire):
        t=self.transport
        try:
            self._sample_clock();self._recovery_timeout()
            if t.state=='FAULT':return t.wait_payload()
            now=t.clock();o=self.contract.validate_observation(wire,max_age_ms=300,now_ns=now,previous_sequence=self.previous_sequence)
            require(o.instruction==self.task,'wrong task')
            frames=o.metadata.get('camera_frames',{})
            require(set(frames)=={'camera_'+k for k in SERIALS},'three camera metadata required')
            for k,serial in SERIALS.items():
                f=frames['camera_'+k];stamp=f.get('source_timestamp_ns')
                require(f.get('camera_serial')==serial,'wrong camera identity')
                require(type(stamp) is int,'camera timestamp invalid')
                require(0<=now-stamp<=300_000_000,'camera timestamp stale/future')
            normal=dict(q=o.state14.tolist(),timestamp_ns=o.timestamp_ns,generation=o.metadata['owner_generation'],policy_session_id=o.metadata['policy_session_id'])
            t._validate_obs(normal);self.previous_sequence=o.sequence
            if not self._recovery_ready(normal):return t.wait_payload()
            self.latest=normal;self.last_error=None
            if self.pending_probe is not None:
                plan,approval=self.pending_probe;self.pending_probe=None
                t.stage(plan,approval,normal)
            for row in self.tail.read():t.receipt(row)
            r=t.tick(normal)
            if r is None:return t.wait_payload()
            actions=np.asarray(r['targets14'],dtype=np.float32)
            self.contract.validate_candidate(actions,schema=action_schema(),frequency_hz=30,measured_state14=o.state14,prefix_steps=len(actions))
            return dict(actions=actions,action_schema=action_schema(),action_frequency_hz=30,
                        policy_metadata=dict(execute_prefix_steps=len(actions),review_release=r,hardware_execution_confirmed=False))
        except Exception as exc:
            self.last_error=type(exc).__name__+': '+str(exc)
            timing=str(exc) in ('observation is stale or timestamp is in the future',
                'camera timestamp stale/future','stale/future feedback')
            if timing and getattr(t,'general_joint_motion',False):self._begin_timing_recovery(str(exc))
            elif t.state!='IDLE' or getattr(t,'general_joint_motion',False):t.stop('observation/contract failure: '+str(exc))
            self.latest=None
            return t.wait_payload()
    def submit_operator_probe(self,plan,approval):
        # This method must be invoked only from operator's protected filesystem/CLI,
        # never exposed as an unapproved agent MCP mutation.
        require(self.latest is not None,'fresh observation needed before operator submission')
        require(self.pending_probe is None and self.transport.state=='IDLE','probe already submitted')
        self.pending_probe=(plan,approval)
    async def watchdog(self):
        while not self.closed:
            t=self.transport
            try:
                self._sample_clock();self._recovery_timeout()
                for row in self.tail.read():t.receipt(row)
                if self.owner_state:
                    raw=await asyncio.to_thread(self.owner_state)
                    age=raw.get('robot_status_age_seconds');status=raw.get('robot_status',{})
                    require(type(age) in (int,float) and math.isfinite(age) and 0<=age<=1.5,'stale owner status heartbeat')
                    state=raw.get('robot_state',{});meta=state.get('feedback_metadata',{})
                    require(state.get('available') is True and state.get('names')==ORDER and state.get('units')==['rad']*6+['m']+['rad']*6+['m'],'passive state schema mismatch')
                    stamp=meta.get('source_timestamp_ns',meta.get('timestamp_ns'));can_age=meta.get('maximum_age_ms')
                    require(type(stamp) is int and type(can_age) in (int,float) and math.isfinite(can_age) and 0<=can_age<=100,'feedback timing unavailable')
                    stamp-=math.ceil(can_age*1e6)
                    require(0<=t.clock()-stamp<=300_000_000,'stale CAN feedback')
                    self.feedback=dict(timestamp_ns=stamp,q=state.get('values'),generation=status.get('generation'),
                        lifecycle_state=status.get('lifecycle_state'),observed_ns=t.clock())
                    self._recovery_feedback()
                    if t.binding is not None:
                        t.observe_feedback({**self.feedback,'policy_session_id':t.binding[0]})
                        t.observe_stop(self.feedback)
                t.watchdog()
            except Exception as exc:
                self.feedback=None
                if self.timing_recovery is not None:
                    self.timing_recovery.update(stable_since=None,samples=0,last_feedback_stamp=None)
                    if str(exc) in ('recovery measured drift','recovery owner generation changed','recovery owner is not running','recovery state invalid'):
                        t.stop('watchdog: '+str(exc))
                if t.state in ('STAGED','RUNNING','SETTLING'):t.stop('watchdog: '+str(exc))
            await asyncio.sleep(.05)
    def close(self):
        self.closed=True
        if self.transport.state!='IDLE':self.transport.stop('service closing')
