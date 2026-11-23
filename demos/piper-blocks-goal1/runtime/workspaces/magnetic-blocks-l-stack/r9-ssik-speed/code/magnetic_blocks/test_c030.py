from unittest.mock import patch
import pytest
import assembly_plan as a
from test_c034 import planner

def test_endpoint_cache_is_independent_of_incoming_seed_and_tail():
 p=planner();calls=[]
 def solve(pos,q,seed):calls.append(seed);return [0.1]*6+seed[6:]
 p._solve_single=solve
 x=p.endpoint([.3,-.1,.2],a.quat(180),[0.]*14)
 y=p.endpoint([.3,-.1,.2],a.quat(180),[.2]*14)
 assert len(calls)==1 and x[:6]==y[:6] and y[6:]==[.2]*8

def test_short_refusal_is_retried_full_but_success_reused():
 p=planner();p.search_active=True;p.ik_mode='fast';calls=[]
 def solve(pos,q,seed):
  calls.append(p.ik_mode)
  if p.ik_mode=='fast':raise a.IKUnavailable('short budget')
  return seed
 p._solve_single=solve
 with pytest.raises(a.IKUnavailable):p.endpoint([.3,-.1,.2],a.quat(180),[0.]*14)
 p.ik_mode='full';p.endpoint([.3,-.1,.2],a.quat(180),[0.]*14)
 assert calls==['fast']*3+['full']

def test_family_timeout_rotates_to_low_grasp_and_restores_deadline():
 p=planner();p.progress=lambda **kw:None;seen=[]
 def family(start,scene,color,target,candidate):
  seen.append(candidate)
  if candidate is None:
   yield {'phase':'source_endpoint','resume':True}
   raise AssertionError('Low candidate should finish first')
  return {'ok':True}
 p._family_steps=family
 with patch.object(a,'pickup_source_conflicts',return_value=[]):
  assert p.transfer([0.]*14,p.scene,'orange',[.3,-.1,.08])['ok']
 assert seen[0] is None and seen[1]['height_delta_m']==-.006
 assert p.grasp_deadline is None

def test_transport_error_not_converted_to_endpoint_failure():
 p=planner();p._solve_single=lambda *x:(_ for _ in ()).throw(ConnectionError('closed'))
 with pytest.raises(ConnectionError):p.endpoint([.3,-.1,.2],a.quat(180),[0.]*14)
 assert p.endpoint_cache=={}
