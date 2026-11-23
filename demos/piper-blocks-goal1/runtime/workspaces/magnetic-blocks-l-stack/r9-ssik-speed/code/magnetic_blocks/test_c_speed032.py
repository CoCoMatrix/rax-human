import copy
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
import assembly_plan as a
from test_c034 import planner
from native_fk.geometry import URDF_SHA


def test_outer_queue_revisits_suspended_layout_without_restarting_it():
 p=planner();p.check=lambda *x,**kw:{};begins=[];resumes=[]
 def layout(start,anchor,side):
  begins.append(anchor[0])
  if anchor[0]!=.2:raise a.NoPlan('layout geometrically blocked')
  for i in range(3):
   yield {'resume':True,'completed':1}
   resumes.append(i)
  return {'ok':True}
 p._layout_steps=layout
 with patch.object(a,'layout_candidates',return_value=[([.2+i*.01,-.2],-1) for i in range(12)]):
  assert p._plan([0.]*14)=={'ok':True}
 assert begins.count(.2)==1 and resumes==[0,1,2]


def test_full_pass_resumes_same_generator_instead_of_restarting_layout():
 p=planner();p.check=lambda *x,**kw:{};begins=[];resumes=[]
 def layout(*args):
  begins.append(1)
  yield {'upgrade':True,'completed':2}
  resumes.append(p.ik_mode)
  return {'ok':True,'moves':[]}
 p._layout_steps=layout
 with patch.object(a,'layout_candidates',return_value=[([.2,-.2],-1)]):
  out=p.plan([0.]*14)
 assert begins==[1] and resumes==['full'] and out['ik_search']['fallback_used']


def test_completed_color_and_order_survive_layout_yield_and_full_upgrade():
 p=planner();p.progress=lambda **kw:None;p.ik_mode='fast';p.check=lambda *x,**kw:{}
 p.solve=lambda pos,q,seed:list(seed);p.fk=lambda q:{'position':[.2,-.2,.25]}
 p.certify_complete_plan=lambda moves:None;p.layout_resume_count=0
 p.opening_check={};p.layout_candidates=[([.3,-.2],-1)];calls=[];orange=[0]
 def transfer(start,scene,color,target):
  calls.append(color)
  if color=='orange':
   orange[0]+=1
   if orange[0]==1:raise a.CandidateBudgetExceeded('slice')
   if orange[0]==2:raise a.NoPlan('unresolved fast IK')
  updated=copy.deepcopy(scene);updated['blocks'][color]['top_center']['surface_position_m']=target
  return {'color':color,'scene_after':updated,'end14':list(start),'actions':[]}
 p.transfer=transfer
 with patch.object(a,'pickup_source_conflicts',return_value=[]),patch.object(a,'block_bounds',side_effect=lambda scene,c:(np.array([100]*3),np.array([100]*3)) if c=='green' else (np.zeros(3),np.zeros(3))),patch.object(a,'assert_goal'),patch.object(a,'ranked_orders',return_value=[{'order':['red','orange','green'],'blocked_pickups':0}]):
  g=p._layout_steps([0.]*14,[.3,-.2],-1)
  assert next(g)['resume'] and calls==['red','orange']
  assert next(g)['needs_full']
  p.ik_mode='full'
  with pytest.raises(StopIteration) as stop:next(g)
 assert stop.value.value['order']==['red','orange','green']
 assert calls.count('red')==1 and calls.count('green')==1


def test_upgrade_keeps_partial_routes_and_reopens_only_exhausted_routes():
 p=planner();partial={'index':2,'leg':1,'rows':['checked']}
 p.route_resume={'partial':partial,'exhausted':{'exhausted':True}}
 p.failed_checks={'exact_geometry':'blocked'};p.prepare_full_recheck()
 assert p.route_resume=={'partial':partial} and p.failed_checks=={'exact_geometry':'blocked'}


def batch_planner():
 p=planner();p.ctx._blocks_batch_ik=True;p.search_active=True;p.started=a.time.monotonic();p.ik_mode='fast'
 return p


def test_batch_cache_rechecks_only_failed_seeds_and_stops_on_valid_solution():
 p=batch_planner();calls=[]
 def value(name,**kw):
  calls.append(kw)
  rows=[{'ok':False,'reason':'MOVEIT_NO_IK_SOLUTION','seed':seed} for seed in kw['seeds']]
  return {'ok':False,'attempts':rows,'hardware_writes':0,'executed':False,'frame':'left_base_link','controlled_link':'left_link6','urdf_sha256':URDF_SHA}
 p.ctx.value=value;seeds=[[0.]*6,[.01]*6]
 with pytest.raises(a.IKUnavailable):p._solve_candidates([.2,-.1,.25],a.quat(180),seeds,[0.]*8)
 with pytest.raises(a.IKUnavailable):p._solve_candidates([.2,-.1,.25],a.quat(180),seeds,[0.]*8)
 assert len(calls)==1
 p.prepare_full_recheck();p.ik_mode='full'
 with pytest.raises(a.IKUnavailable):p._solve_candidates([.2,-.1,.25],a.quat(180),seeds,[0.]*8)
 assert len(calls)==2 and calls[-1]['mode']=='full'


@pytest.mark.parametrize('fault',['seed','model','truncated','transport'])
def test_bad_batch_reply_is_not_a_no_solution(fault):
 p=batch_planner()
 def bad(name,**kw):
  if fault=='transport':raise ConnectionError('backend disconnected')
  rows=[] if fault=='truncated' else [{'ok':False,'reason':'MOVEIT_NO_IK_SOLUTION','seed':[1.]*6 if fault=='seed' else kw['seeds'][0]}]
  return {'ok':False,'attempts':rows,'hardware_writes':0,'executed':False,'frame':'left_base_link','controlled_link':'left_link6','urdf_sha256':'bad' if fault=='model' else URDF_SHA}
 p.ctx.value=bad
 with pytest.raises((ValueError,ConnectionError)):
  p._solve_candidates([.2,-.1,.25],a.quat(180),[[0.]*6],[0.]*8)


def test_new_pose_in_long_pass_gets_short_screen_before_long_query():
 p=batch_planner();p.ik_mode='full';calls=[]
 def value(name,**kw):
  calls.append(kw['mode'])
  return {'ok':False,'attempts':[{'ok':False,'reason':'MOVEIT_NO_IK_SOLUTION','seed':s} for s in kw['seeds']], 'hardware_writes':0,'executed':False,'frame':'left_base_link','controlled_link':'left_link6','urdf_sha256':URDF_SHA}
 p.ctx.value=value
 with pytest.raises(a.IKUnavailable):p._solve_candidates([.2,-.1,.25],a.quat(180),[[0.]*6],[0.]*8)
 assert calls==['fast','full']


def test_short_success_never_receives_long_budget_query():
 p=batch_planner();p.ik_mode='full';calls=[]
 def value(name,**kw):
  calls.append(kw['mode'])
  return {'ok':True,'attempts':[{'ok':True,'q':[0.]*6,'seed':kw['seeds'][0]}], 'hardware_writes':0,'executed':False,'frame':'left_base_link','controlled_link':'left_link6','urdf_sha256':URDF_SHA}
 p.ctx.value=value
 assert p._solve_candidates([.2,-.1,.25],a.quat(180),[[0.]*6],[0.]*8)==[0.]*14
 assert calls==['fast']


def test_active_grasp_continues_before_hundreds_of_untried_variants():
 p=planner();p.progress=lambda **kw:None;p.ik_mode='fast';begins=[]
 def family(start,scene,color,target,candidate):
  begins.append(candidate)
  if candidate is not None:raise a.NoPlan('not this family')
  for _ in range(4):yield {'phase':'hover','resume':True}
  return {'ok':True}
 p._family_steps=family
 with patch.object(a,'pickup_source_conflicts',return_value=[]):assert p.transfer([0.]*14,p.scene,'orange',[.3,-.1,.08])['ok']
 assert len(begins)<8 and begins.count(None)==1


def test_long_pass_does_not_escalate_unfinished_grasp_generator():
 p=planner();p.ik_mode='full';p.progress=lambda **kw:None;modes=[]
 def family(*args):
  modes.append(p.query_mode)
  yield {'phase':'retreat','resume':True}
  modes.append(p.query_mode)
  return {'ok':True}
 p._family_steps=family
 with patch.object(a,'pickup_source_conflicts',return_value=[]):assert p.transfer([0.]*14,p.scene,'orange',[.3,-.1,.08])['ok']
 assert modes==['fast','fast']


def test_only_exhausted_grasp_is_reopened_with_long_ik():
 p=planner();p.ik_mode='full';p.progress=lambda **kw:None;modes=[]
 def family(start,scene,color,target,candidate):
  if False:yield
  modes.append((candidate,p.query_mode))
  if p.query_mode=='fast':raise a.NoPlan('unresolved fast endpoints')
  return {'ok':True}
 p._family_steps=family
 with patch.object(a,'pickup_source_conflicts',return_value=[]):assert p.transfer([0.]*14,p.scene,'orange',[.3,-.1,.08])['ok']
 assert [m for c,m in modes if c is None]==['fast','full']
