import copy
from unittest.mock import patch
import pytest
import numpy as np
import assembly_plan as a
from test_c034 import planner

def consume(g):
 while True:
  try:next(g)
  except StopIteration as x:return x.value

def test_stage_continuation_does_not_repeat_completed_stage():
 p=planner();p.progress=lambda **kw:None;calls=[];attempt=[0]
 def expensive():
  attempt[0]+=1
  if attempt[0]==1:raise a.GraspBudgetExceeded('slice')
  return 7
 def task():
  calls.append('prefix')
  value=yield from p.search_step('retreat',expensive)
  calls.append('end');return value
 g=task();assert next(g)['resume'];assert consume(g)==7
 assert calls==['prefix','end'] and attempt[0]==2

@pytest.mark.parametrize('exc',[a.CandidateBudgetExceeded,a.GraspBudgetExceeded])
def test_timeout_is_unresolved_not_permanent_stage_failure(exc):
 p=planner();p.progress=lambda **kw:None;calls=[0]
 def f():
  calls[0]+=1
  if calls[0]==1:raise exc('budget')
  return 'checked'
 g=p.search_step('release',f);event=next(g)
 assert event['resume'] and consume(g)=='checked'

@pytest.mark.parametrize('exc',[ConnectionError,a.NoPlan])
def test_real_error_is_not_hidden_by_resume(exc):
 p=planner();p.progress=lambda **kw:None
 with pytest.raises(exc):next(p.search_step('retreat',lambda:(_ for _ in ()).throw(exc('failure'))))

def test_finish_checked_before_pickup_and_known_bad_finish_never_routes_pickup():
 p=planner();p.progress=lambda **kw:None;p.endpoint=lambda pos,q,seed:list(pos)+[0.]*3+list(seed[6:])
 p.fk=lambda q:{'T_base_flange':np.eye(4).tolist(),'position':q[:3]}
 p.pickup_prefix=lambda *x,**kw:pytest.fail('Known bad finish must be rejected first')
 def fail(*x,**kw):
  if False:yield
  raise a.NoPlan('withdrawal blocked')
 p.finish_steps=fail
 with patch.object(a,'pickup_source_conflicts',return_value=[]),pytest.raises(a.NoPlan):
  consume(p._transfer_steps([0.]*14,p.scene,'orange',[.3,-.1,.08]))

def test_scheduler_resumes_nearly_finished_family():
 p=planner();p.progress=lambda **kw:None;calls=[]
 def fake(start,scene,color,target,candidate):
  calls.append(('begin',candidate))
  yield {'phase':'retreat','resume':True}
  calls.append(('resume',candidate));return {'ok':True}
 p._family_steps=fake
 with patch.object(a,'pickup_source_conflicts',return_value=[]):
  assert p.transfer([0.]*14,p.scene,'orange',[.3,-.1,.08])['ok']
 assert calls==[('begin',None),('resume',None)]
 assert p.search_resume_counts['family']==1

def test_finish_cache_requires_exact_start_and_scene():
 p=planner();p.progress=lambda **kw:None;p.check=lambda *x,**kw:{};calls=[]
 p.fk=lambda q:{'position':q[:3]}
 def route(start,dest,q,scene,**kw):
  calls.append(1);target=list(start);target[:3]=dest
  return [{'start14':list(start),'target14':target,'check':{}}]
 p.route=route;start=[.3,-.1,.2]+[0.]*11;start[6]=a.CLOSED
 args=[start,p.scene,'orange',[.3,-.1,.08],{},a.quat(180),None,None]
 consume(p.finish_steps(*args));consume(p.finish_steps(*args));assert len(calls)==1
 args[0]=start.copy();args[0][0]+=.001;consume(p.finish_steps(*args));assert len(calls)==2
 args[1]=copy.deepcopy(p.scene);args[1]['table']['z_ax_by_c'][2]+=.001
 consume(p.finish_steps(*args));assert len(calls)==3

def test_route_resumes_at_first_unfinished_leg():
 from test_b04_clearance import route_planner
 p=route_planner();start=[.3,0.,.25]+[0.]*11;dest=[.3,.1,.3];calls=[];interrupted=[False]
 def edge(q,point,rot,scene,**kw):
  calls.append((tuple(q[:3]),tuple(point)))
  if q[:3]==start[:3] and point==dest:raise a.NoPlan('direct blocked')
  if q[:3]!=start[:3] and not interrupted[0]:
   interrupted[0]=True;raise a.GraspBudgetExceeded('slice')
  target=list(point)+q[3:]
  return [{'start14':list(q),'target14':target,'check':{}}],target
 p.connect_edge=edge
 with pytest.raises(a.GraspBudgetExceeded):p._route(start,dest,[1,0,0,0],{},contact=True)
 rows=p._route(start,dest,[1,0,0,0],{},contact=True)
 assert len(rows)==2 and rows[-1]['target14'][:3]==dest
 assert sum(q==tuple(start[:3]) for q,point in calls)==2  # failed direct plus one completed half; neither repeated

def test_valid_route_survives_timeout_while_seeking_preferred_clearance():
 from test_b04_clearance import route_planner
 p=route_planner();start=[.3,0.,.25]+[0.]*11;dest=[.3,.1,.3];calls=[0]
 def edge(q,point,rot,scene,**kw):
  calls[0]+=1
  if calls[0]>1:raise a.GraspBudgetExceeded('slice')
  target=list(point)+q[3:]
  return [{'start14':list(q),'target14':target,'check':{'min_payload_neighbor_m':.021}}],target
 p.connect_edge=edge
 rows=p._route(start,dest,[1,0,0,0],{},active='orange',payload={'color':'orange'})
 assert rows[-1]['target14'][:3]==dest and calls[0]==2
