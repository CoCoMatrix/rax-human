import copy,time
from types import SimpleNamespace
import pytest
import numpy as np
import assembly_batch as b
import controller as c
from assembly_controller import latest_control_feedback


def state():
 q=[0.]*14;q[6]=.0512
 return {'measured_state':{'available':True,'values':q,'feedback_metadata':{'source_timestamp_ns':1_000_000_000}},'owner_status':{'generation':1},'bridge':{'feedback':{'q':[.002]*14,'timestamp_ns':999_000_000}}}


def test_authoritative_feedback_ignores_delayed_bridge():
 s=state();r=latest_control_feedback(s,now_ns=1_001_000_000)
 assert r['q']==s['measured_state']['values'] and r['source']=='owner_measured_state'


@pytest.mark.parametrize('change',[lambda s:s['measured_state'].update(available=False),lambda s:s['measured_state']['feedback_metadata'].update(source_timestamp_ns=0),lambda s:s['measured_state']['values'].__setitem__(0,float('nan')),lambda s:s['owner_status'].update(generation=None)])
def test_invalid_owner_never_falls_back(change):
 s=state();change(s)
 with pytest.raises((RuntimeError,ValueError)):latest_control_feedback(s,now_ns=1_001_000_000)


def error(**overrides):
 d={'type':'RejectedBeforeDispatch','stage':'validation','operation':'piper_move_waypoints','episode_ended':False,'control_uncertain':False,'no_action_executed':True,'executed':False,'dispatch_started':False,'reason':'Initial connector exceeds acceleration limit; repreview before dispatch'}
 d.update(overrides);return c.MotionRejectedBeforeDispatch(d)


@pytest.mark.parametrize('change',[{'executed':True},{'dispatch_started':True},{'control_uncertain':True},{'no_action_executed':False},{'episode_ended':True},{'operation':'piper_gripper_stroke'},{'reason':'whole-path CAD collision refusal'},{'reason':'Owner identity changed'},{'stage':'execution'}])
def test_retry_allowlist_requires_proven_no_send(change):
 assert not b.retryable_anchor_refusal(error(**change))
 assert not b.retryable_anchor_refusal(RuntimeError(str(error())))


def proposal(q):return {'arguments':{'start14':q.copy(),'waypoints':[[.1]*6]},'target14':[.1]*6+q[6:],'duration_s':1.2,'samples':36,'stages':['orange-lower']}


def setup(monkeypatch):
 q=[0.]*14;q[6]=.0512
 e=SimpleNamespace(ctx=object(),guard=lambda:({},q.copy()))
 calls=[]
 def rebuild(e,actions,start,report,*,soft_start=False):
  assert soft_start
  calls.append(start.copy());return proposal(start),start
 monkeypatch.setattr(b,'prepare_preview',rebuild)
 return e,q,calls,{'receipts':[],'executed':False}


def test_nosend_retry_regenerates_and_sends_once(monkeypatch):
 e,q,rebuilds,r=setup(monkeypatch);attempts=[]
 def move(*args,**kw):
  attempts.append(1)
  if len(attempts)==1:raise error()
  r['receipts'].append({'executed':True});return 'ok'
 monkeypatch.setattr(c,'_move',move)
 out,_,p=b.dispatch_reviewed_curve(e,[{}],proposal(q),q,r)
 assert out=='ok' and len(attempts)==2 and len(rebuilds)==1 and len(r['receipts'])==1
 assert r['no_dispatch_anchor_retries'][0]['replayed_motion'] is False
 assert r['pre_dispatch_rejected'] is False


def test_retry_is_bounded(monkeypatch):
 e,q,rebuilds,r=setup(monkeypatch);attempts=[]
 def move(*args,**kw):attempts.append(1);raise error()
 monkeypatch.setattr(c,'_move',move)
 with pytest.raises(c.MotionRejectedBeforeDispatch):b.dispatch_reviewed_curve(e,[{}],proposal(q),q,r)
 assert len(attempts)==3 and len(rebuilds)==2 and not r['receipts']


@pytest.mark.parametrize('exc',[RuntimeError('TLS/reset with unknown outcome'),error(executed=True),error(reason='CAD collision refusal'),error(episode_ended=True)])
def test_ambiguous_or_unrelated_errors_never_replay(monkeypatch,exc):
 e,q,rebuilds,r=setup(monkeypatch);attempts=[]
 def move(*args,**kw):attempts.append(1);raise exc
 monkeypatch.setattr(c,'_move',move)
 with pytest.raises(RuntimeError):b.dispatch_reviewed_curve(e,[{}],proposal(q),q,r)
 assert len(attempts)==1 and not rebuilds


def test_receipt_prevents_retry_even_if_error_claims_no_send(monkeypatch):
 e,q,rebuilds,r=setup(monkeypatch)
 def move(*args,**kw):r['receipts'].append({'executed':True});raise error()
 monkeypatch.setattr(c,'_move',move)
 with pytest.raises(c.MotionRejectedBeforeDispatch):b.dispatch_reviewed_curve(e,[{}],proposal(q),q,r)
 assert not rebuilds


def test_fresh_feedback_guard_failure_blocks_retry(monkeypatch):
 e,q,rebuilds,r=setup(monkeypatch)
 def guard():raise RuntimeError('owner or previous request changed')
 def move(*args,**kw):e.guard=guard;raise error()
 monkeypatch.setattr(c,'_move',move)
 with pytest.raises(RuntimeError,match='owner'):b.dispatch_reviewed_curve(e,[{}],proposal(q),q,r)
 assert not rebuilds

