"""Piper measured settling; no hardware access, no changes to installed SDK.

Operator-selected 0.3 s window, matching the Franka setting. Offline tested;
not yet live-qualified for Piper and not proof against delayed filtered feedback.
"""
import time
import numpy as np

SETTLING_WINDOW_S=.3
SETTLING_MIN_SAMPLES=4

JOINTS=[0,1,2,3,4,5,7,8,9,10,11,12]
# Operator-approved joint STATE tolerance across arrival, anchor, rest and recovery.
# Not a joint limit, velocity/acceleration bound, jaw tolerance or firmware repair.
JOINT_STATE_TOLERANCE_RAD=.01
JOINT_ARRIVAL_TOLERANCE_RAD=JOINT_STATE_TOLERANCE_RAD
# Separately approved observation binding; never apply radians to jaw metres.
OBSERVATION_JOINT_TOLERANCE_RAD=JOINT_STATE_TOLERANCE_RAD
OBSERVATION_JAW_TOLERANCE_M=.0005



def state_matches(a,b,*,jaw_tolerance=.0005):
    a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
    if a.shape!=(14,) or b.shape!=(14,) or not np.isfinite([a,b]).all():return False
    d=abs(a-b)
    return bool(np.max(d[JOINTS])<=JOINT_STATE_TOLERANCE_RAD and np.max(d[[6,13]])<=jaw_tolerance)


def settling_valid(e):
    # Explicit units; historical scalar evidence is accepted only at its OLD,
    # stricter bound. Never treat a mixed-unit max as .01 metres of jaw drift.
    if 'max_joint_range_rad' not in e:
        return 0<=e.get('max_range',float('inf'))<=.0003
    return (e.get('joint_range_limit_rad')==JOINT_STATE_TOLERANCE_RAD
            and e.get('jaw_range_limit_m')==.0003
            and 0<=e.get('max_joint_range_rad',float('inf'))<=JOINT_STATE_TOLERANCE_RAD
            and 0<=e.get('max_jaw_range_m',float('inf'))<=.0003)


class RestWindow:
    def __init__(self):
        self.first=None;self.last_stamp=None;self.last_mono=None
        self.low=None;self.high=None;self.samples=0

    def add(self,q,stamp,mono):
        q=np.asarray(q,dtype=float)
        if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('Invalid rest feedback')
        if type(stamp) is not int:raise ValueError('Missing rest feedback timestamp')
        if self.last_stamp is not None and stamp<self.last_stamp:raise ValueError('Rest feedback timestamp reversed')
        if self.last_mono is not None and mono<self.last_mono:raise ValueError('Rest monotonic clock reversed')
        if stamp==self.last_stamp:return False
        if (self.first is None or mono-self.last_mono>300_000_000
                or not state_matches(np.maximum(self.high,q),np.minimum(self.low,q),jaw_tolerance=.0003)):
            self.first=mono;self.low=q.copy();self.high=q.copy();self.samples=0
        self.low=np.minimum(self.low,q);self.high=np.maximum(self.high,q)
        self.last_stamp=stamp;self.last_mono=mono;self.samples+=1
        return mono-self.first>=int(SETTLING_WINDOW_S*1e9) and self.samples>=SETTLING_MIN_SAMPLES

    def evidence(self):
        return {'window_s':(self.last_mono-self.first)/1e9,'samples':self.samples,
                'max_range':float(np.max(self.high-self.low)),
                'max_joint_range_rad':float(np.max((self.high-self.low)[JOINTS])),
                'max_jaw_range_m':float(np.max((self.high-self.low)[[6,13]])),
                'joint_range_limit_rad':JOINT_STATE_TOLERANCE_RAD,'jaw_range_limit_m':.0003,
                'source_timestamp_ns':self.last_stamp,
                'clock':'monotonic','physical_rest_independently_verified':False}


def transport_class(Base):
    """Keep inherited planning/receipt/envelope guards, replace only settling."""
    class SettledMotionTransport(Base):
        def __init__(self,*args,monotonic_ns=time.monotonic_ns,**kwargs):
            super().__init__(*args,**kwargs)
            # Scope the new tolerance to this RAX adapter after legacy setup;
            # the shared XPolicy/legacy constructor and its defaults stay intact.
            self.joint_tolerance=JOINT_ARRIVAL_TOLERANCE_RAD
            self.monotonic_ns=monotonic_ns;self.rest_window=RestWindow();self.mono_deadline=None

        def stage(self,plan,observation):
            with self.lock:
                super().stage(plan,observation)
                self.rest_window=RestWindow()
                self.mono_deadline=self.monotonic_ns()+int((plan['duration_s']+4)*1e9)

        def _guards(self):
            super()._guards()
            if self.mono_deadline is not None and self.monotonic_ns()>=self.mono_deadline:
                raise ValueError('motion arrival timeout (monotonic)')

        def observe_feedback(self,o):
            with self.lock:
                if self.state!='SETTLING':return super().observe_feedback(o)
                try:
                    self._validate_obs(o)
                    q=np.asarray(o['q']);a=np.asarray(self.plan['anchor14']);b=np.asarray(self.plan['target14'])
                    self.check_feedback_path(q,o['timestamp_ns'])
                    self.last_observed_ns=o['timestamp_ns'];self._guards()
                    err=float(np.max(abs(q[JOINTS]-b[JOINTS])));jaw_error=float(abs(q[6]-b[6]));self.last_q=q
                    if err>self.joint_tolerance or jaw_error>.001:
                        self.rest_window=RestWindow();return
                    if self.rest_window.add(q,o['timestamp_ns'],self.monotonic_ns()):
                        self.state='DONE'
                        self.result=dict(completed_at_ns=self.clock(),executed=True,measured_arrival=True,measured_settled=True,
                            joint_error_max_rad=err,joint_tolerance_rad=self.joint_tolerance,
                            sdk_completed_steps=len(self.receipts),final14=q.tolist(),target14=b.tolist(),
                            owner_generation=self.binding[1],settling=self.rest_window.evidence(),
                            left_jaw_error_m=jaw_error,object_success=None,
                            anchor_validation=getattr(self,'anchor_validation',None),
                            trajectory_progress=self.progress())
                        self._lease('done')
                except Exception as exc:self.stop(str(exc))
    return SettledMotionTransport
