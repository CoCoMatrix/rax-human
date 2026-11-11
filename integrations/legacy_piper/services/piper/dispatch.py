"""Dual-arm command contract, dry-run review and no-action PolicyServer adapter.

There is deliberately no SDK, network client or live arm switch in this module.
A checked candidate is not a dispatch authorization. Hardware release remains
unavailable until measured calibration and controller characterization exist.
"""
import hashlib
import json
import time
import uuid
import numpy as np
from .geometry import ORDER

JOINTS=[0,1,2,3,4,5,7,8,9,10,11,12]
JAWS=[6,13]

def action_schema():
    return dict(schema_id='xpolicy.piper_x_dual.joint_position.v1',fields=list(ORDER),
                units=['rad']*6+['m']+['rad']*6+['m'],
                semantics=['absolute_joint']*6+['absolute_gripper_width']+['absolute_joint']*6+['absolute_gripper_width'])

def digest(value): return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()

class NoSendDispatch:
    def __init__(self,geometry,collision):
        self.geometry,self.collision=geometry,collision
        self.pending={};self.used=set();self.cancelled=False

    def prepare(self,start,samples,*,generation,observed_ns,now_ns=None,obstacles=()):
        now=time.time_ns() if now_ns is None else now_ns
        if self.cancelled: raise ValueError('dispatcher cancelled; explicit new session required')
        if type(generation) is not int or generation<0 or type(observed_ns) is not int or not 0<=now-observed_ns<=300_000_000: raise ValueError('stale/future feedback or invalid generation')
        anchor=self.geometry._vector(start)
        a=np.asarray(samples)
        if a.ndim!=2 or a.shape[1]!=14 or not 1<=len(a)<=600 or a.dtype.kind not in 'fi' or not np.isfinite(a).all(): raise ValueError('finite 1..600 x14 targets required')
        for row in a: self.geometry._vector(row.tolist())
        wire=a.astype('<f4')
        for row in wire: self.geometry._vector(row.astype(float).tolist())
        d=np.diff(np.vstack([anchor,wire.astype(float)]),axis=0)
        if np.max(abs(d[:,JOINTS]))>.2/30+1e-8 or np.max(abs(d[:,JAWS]))>.02/30+1e-8: raise ValueError('conservative command speed exceeded')
        # Include rest at both endpoints: a path must not acquire/discard velocity abruptly.
        acc=np.diff(np.vstack([np.zeros(14),d,np.zeros(14)]),axis=0)*900
        if np.max(abs(acc[:,JOINTS]))>.5+1e-5 or np.max(abs(acc[:,JAWS]))>.05+1e-5: raise ValueError('conservative command acceleration exceeded')
        report=self.collision.path(np.vstack([anchor,wire.astype(float)]),obstacles=obstacles)
        if not report['cad_clear']: raise ValueError('collision/clearance refusal: '+json.dumps(report['contacts']))
        # No scene assertion: the formal owner must reject this dry-run schema.
        packet=dict(schema='rax.piper.no_send_candidate.v1',candidate_id=uuid.uuid4().hex,
                    generation=generation,observed_ns=observed_ns,expires_ns=observed_ns+300_000_000,
                    action_schema=action_schema(),frequency_hz=30,anchor14=anchor.tolist(),targets14=wire.astype(float).tolist(),
                    urdf_sha256=self.geometry.status()['urdf_sha256'],collision_report=report,
                    collision_scene_sha256=digest(list(obstacles)),physical_calibration_validated=False,
                    dispatch_authorized=False,hardware_writes=0)
        packet['sha256']=digest(packet)
        self.pending[packet['candidate_id']]=json.loads(json.dumps(packet))
        return packet

    def consume_dry_run(self,packet,*,generation,measured,now_ns=None,armed=False):
        now=time.time_ns() if now_ns is None else now_ns
        cid=packet.get('candidate_id')
        if self.cancelled or cid in self.used or cid not in self.pending: raise ValueError('unknown/consumed/cancelled candidate')
        expected=self.pending[cid]
        if packet!=expected: raise ValueError('candidate changed after review')
        if generation!=packet['generation'] or not packet['observed_ns']<=now<packet['expires_ns']: raise ValueError('generation or freshness invalidated')
        q=self.geometry._vector(measured)
        if np.max(abs(q-np.asarray(packet['anchor14'])))>.001: raise ValueError('measured anchor drift')
        self.used.add(cid);del self.pending[cid]
        return dict(dry_run_consumed=True,armed=bool(armed),hardware_writes=0,dispatch_authorized=False,
                    encoded_targets=len(packet['targets14']),reason='NO_SEND; arming cannot enable this adapter')

    def cancel(self):
        self.pending.clear();self.cancelled=True
        return dict(hardware_stop_sent=False,pending_cleared=True)

    def dispatch(self,*args,**kwargs):
        raise PermissionError('NO_SEND: physical dispatch unavailable; requires measured commissioning and explicit operator release')

class NoSendPolicy:
    """Implements the existing owner PolicyServer protocol; always zero actions.
    Use only with an isolated test client or an explicitly no-write owner graph.
    No forwarder to hardware or prior GPTPolicy agent is imported.
    """
    def get_capabilities(self):
        return dict(mode='rax_piper_no_send',hardware_write_allowed=False,owner_dispatch_ready=False)
    def reset(self,**kwargs): return dict(physical_reset=False,automatic_rearm=False)
    def infer(self,observation):
        return dict(result_kind='no_action',actions=np.empty((0,14),np.float32),action_schema=action_schema(),action_frequency_hz=30,
                    policy_metadata=dict(execute_prefix_steps=0,hardware_execution_confirmed=False,inference_executed=False,
                    transport_receipt=dict(mode='continuous_review_wait',request_id='rax-no-send',status='NO_SEND',dispatch_authorized=False,hardware_execution_confirmed=False)))
