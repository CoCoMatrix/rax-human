"""Private, short-lived RAX write lease checked by the existing formal owner.

No driver or device access. Copy this module into an isolated formal runtime.
An expired lease cancels MODEL through its normal Stop/generation boundary.
"""
import json
import math
import os
from pathlib import Path
import stat
import time


def read_private(path):
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    with os.fdopen(fd) as f:
        s=os.fstat(f.fileno())
        if not stat.S_ISREG(s.st_mode) or s.st_uid!=os.getuid() or s.st_mode & 0o077:
            raise ValueError('private operator-owned regular file required')
        raw=f.read(65537)
    if len(raw)>65536:raise ValueError('oversize lease')
    return json.loads(raw)


class MotionLease:
    def __init__(self,path,arming_file,clock_ns=time.time_ns):
        self.path=Path(path);self.arming_file=Path(arming_file);self.clock=clock_ns
        self.max_joint_tolerance=float(os.environ.get('RAX_MAX_JOINT_ARRIVAL_TOLERANCE_RAD','.003'))
        if self.max_joint_tolerance not in (.003,.006,.01):
            raise ValueError('Unsupported owner joint arrival tolerance policy')
        self.active=None;self.last_stop=None

    def read(self):
        try:v=read_private(self.path);a=read_private(self.arming_file)
        except OSError as exc:raise ValueError('operator lease/arming file unavailable') from exc
        now=self.clock()
        if v.get('schema')!='rax.piper.motion_lease.v1':raise ValueError('lease schema')
        stamp=v.get('timestamp_ns')
        if type(stamp) is not int or not 0<=now-stamp<=250_000_000:raise ValueError('motion heartbeat expired')
        if a.get('armed') is not True:raise ValueError('operator disarmed')
        until=a.get('until')
        if until is not None and (type(until) not in (float,int) or not math.isfinite(until) or now/1e9>=until):
            raise ValueError('operator arming expired')
        return v

    def authorize(self,release):
        v=self.read()
        if (v.get('phase')!='active' or v.get('request_id')!=release['teacher_request_id']
                or v.get('generation')!=release['generation']
                or v.get('policy_session_id')!=release['execution_session_id']):
            raise ValueError('lease is not bound to this release')
        target=v.get('target14')
        if not isinstance(target,list) or len(target)!=14 or any(type(x) not in (float,int) or not math.isfinite(x) for x in target):
            raise ValueError('lease target malformed')
        tolerance=v.get('joint_tolerance_rad',.001)
        if type(tolerance) not in (float,int) or not math.isfinite(tolerance) or not .001<=tolerance<=self.max_joint_tolerance:
            raise ValueError('lease joint tolerance outside commissioning scope')
        if self.active is not None and self.active['request_id']!=v['request_id']:
            raise ValueError('previous motion has not completed')
        self.active=v

    def stop_reason(self,measured):
        if self.active is None:return None
        try:
            v=self.read()
            if any(v.get(k)!=self.active[k] for k in ('request_id','generation','policy_session_id','target14')):
                raise ValueError('active lease identity changed')
            if v.get('joint_tolerance_rad',.001)!=self.active.get('joint_tolerance_rad',.001):
                raise ValueError('active lease tolerance changed')
            if v.get('phase')=='done':
                errors=[abs(x-y) for x,y in zip(measured,v['target14'],strict=True)]
                if max(errors[i] for i in (0,1,2,3,4,5,7,8,9,10,11,12))>v.get('joint_tolerance_rad',.001) or max(errors[6],errors[13])>.001:
                    raise ValueError('completion does not match measured position')
                self.active=None
            elif v.get('phase')!='active':raise ValueError('motion cancelled')
            return None
        except Exception as exc:
            self.last_stop=str(exc);self.active=None
            return self.last_stop

    def status(self):
        return dict(enabled=True,active_request=None if self.active is None else self.active['request_id'],
                    heartbeat_budget_ms=250,last_stop=self.last_stop,
                    max_joint_arrival_tolerance_rad=self.max_joint_tolerance)
