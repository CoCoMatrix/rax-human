import pytest
import numpy as np
from unittest.mock import patch
import assembly_plan as a
from test_composition import SceneContext

def planner():
 ctx=SceneContext();return a.Planner(ctx,ctx.scene,ctx.robot(),np.zeros((16,3)),np.zeros((16,3)))

def test_fast_success_never_runs_full_or_changes_execution_solver():
 p=planner();calls=[]
 p._plan=lambda start:(calls.append(p.ik_mode) or {'ok':True})
 r=p.plan([0.]*14)
 assert calls==['fast'] and not r['ik_search']['fallback_used']
 assert p.ik_mode=='full' and not p.search_active

def test_long_pass_only_rechecks_unresolved_ik_preserves_all_pure_evidence():
 p=planner();calls=[]
 def value(name,**kw):
  calls.append((name,kw['position']))
  return {'ok':kw['position'][0]==.2 or name=='piper_solve_ik','q':[0.]*6}
 p.ctx.value=value;p.prefix_cache={'prefix':'checked'};p.prefix_failures={}
 def run(start):
  p.solve([.2,0,.25],[0,1,0,0],start)
  if p.ik_mode=='fast':
   p.route_cache['route']='checked';p.route_failures['dependent']='unresolved IK';p.prefix_failures['dependent']='unresolved IK'
   p.fk_cache['fk']='pure';p.native_cache['native']='pure';p.path_cache['passed']='pure';p.failed_checks['blocked']='pure'
  else:
   assert not p.route_failures and not p.prefix_failures
   assert p.prefix_cache['prefix']=='checked' and p.route_cache['route']=='checked'
   assert p.fk_cache['fk']=='pure' and p.native_cache['native']=='pure'
   assert p.path_cache['passed']=='pure' and p.failed_checks['blocked']=='pure'
  p.solve([.21,0,.25],[0,1,0,0],start);return {'ok':True}
 p._plan=run;r=p.plan([0.]*14)
 assert calls.count(('piper_solve_ik_fast',[.2,0.,.25]))==1
 assert calls.count(('piper_solve_ik_fast',[.21,0.,.25]))==3
 assert calls.count(('piper_solve_ik',[.21,0.,.25]))==1
 assert r['ik_search']['attempts'][1]['validated_ik_reused']==1
 assert r['ik_search']['attempts'][1]['unresolved_ik']==3

@pytest.mark.parametrize('change',('pose','seed'))
def test_changed_ik_input_never_uses_promoted_success(change):
 p=planner();p.search_active=True;p.ik_mode='fast';calls=[]
 p.ctx.value=lambda name,**kw:(calls.append(name) or {'ok':True,'q':[0.]*6})
 pose=[.2,0.,.25];seed=[0.]*14;p.solve(pose,[0,1,0,0],seed);p.prepare_full_recheck();p.ik_mode='full'
 if change=='pose':pose[0]+=1e-11
 else:seed[0]+=1e-11
 p.solve(pose,[0,1,0,0],seed)
 assert calls==['piper_solve_ik_fast','piper_solve_ik']

@pytest.mark.parametrize('error',[RuntimeError('pipe closed'),ValueError('wrong model')])
def test_non_planning_fault_never_retries(error):
 p=planner();calls=[]
 def fake(start):calls.append(p.ik_mode);raise error
 p._plan=fake
 with pytest.raises(type(error),match=str(error)):p.plan([0.]*14)
 assert calls==['fast'] and not p.search_active
 assert p.ik_attempts[0]['outcome']=='aborted'

def test_geometry_only_failure_does_not_request_full_budget():
 p=planner();calls=[]
 def fake(start):calls.append(p.ik_mode);raise a.NoPlan('blocked geometry')
 p._plan=fake
 with pytest.raises(a.NoPlan):p.plan([0.]*14)
 assert calls==['fast']

def test_fast_timeout_retains_evidence_but_total_deadline_is_90_seconds():
 p=planner();clock=[0.];calls=[]
 def fake(start):
  calls.append(p.ik_mode)
  if p.ik_mode=='fast':p.ik_cache[('fast',(1,))]=None;clock[0]=31.;p.progress()
  clock[0]=89.;p.progress();return {'ok':True}
 p._plan=fake
 with patch.object(a.time,'monotonic',lambda:clock[0]):r=p.plan([0.]*14)
 assert calls==['fast','full'] and r['elapsed_s']==89.
 p=planner();clock[0]=0.
 def expired(start):clock[0]=91.;p.progress()
 p._plan=expired
 with patch.object(a.time,'monotonic',lambda:clock[0]),pytest.raises(a.PlanningTimeout):p.plan([0.]*14)
 assert p.ik_attempts[0]['outcome']=='aborted'

def test_sample_fk_deduplicates_and_preserves_original_sample_order():
 p=planner();calls=[]
 def many(ctx,points):
  calls.append(list(points));return [{'T_base_flange':np.eye(4).tolist(),'position':[q[0],0,0]} for q in points]
 with patch.object(a,'fk_many',many):
  assert [r['position'][0] for r in p.sample_fks([[0]*6,[1]*6,[0]*6])]==[0,1,0]
  assert [r['position'][0] for r in p.sample_fks([[1]*6,[2]*6])]==[1,2]
 assert [len(x) for x in calls]==[2,1]

def test_fk_cache_is_local_and_bounded():
 p=planner()
 with patch.object(a,'fk_many',lambda ctx,pts:({'position':[q[0],0,0]} for q in pts)):
  assert len(p.sample_fks([[i,0,0,0,0,0] for i in range(8200)]))==8200
 assert len(p.sample_fk_cache)==8192 and not planner().sample_fk_cache

@pytest.mark.parametrize('gap,allowed',[(.002996,True),(.0025,True),(.00249,True),(.00246,True),(.00199,True),(.00189,True),(.00149,True),(.00049,True),(0,False),(-.001,False)])
def test_three_mm_nominal_rule_has_explicit_one_mm_tolerance(gap,allowed):
 assert (gap>0 and gap>=a.finger_gap_minimum(.003))==allowed

@pytest.mark.parametrize('nominal',[.004,.020])
def test_nominal_object_buffers_receive_same_bounded_tolerance(nominal):
 assert a.finger_gap_minimum(nominal)==nominal-.0015
