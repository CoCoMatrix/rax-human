"""Four-stage search must keep completed prefixes across scheduler/IK pauses."""
import copy
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
import assembly_plan as a
from task_spec import COLORS,PREFLIGHT_BUDGET_S


def planner():
 scene={'table':{'z_ax_by_c':[0,0,.018]},'blocks':{c:{'top_center':{'surface_position_m':[.2,-.36+i*.08,.078]},'height_m':.06} for i,c in enumerate(COLORS)}}
 return a.Planner(SimpleNamespace(),scene,{'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36]},np.zeros((16,3)),np.zeros((16,3)))


def test_three_block_prefix_survives_yellow_pause_and_ik_upgrade():
 p=planner();p.progress=lambda **kw:None;p.ik_mode='fast';p.check=lambda *args,**kw:{}
 p.solve=lambda pos,q,seed:list(seed)
 p.fk=lambda q:{'position':[.2,-.2,.25],'T_base_flange':np.eye(4).tolist()}
 def finish(*args,**kw):
  if False:yield
  return {'actions':[]}
 p.finish_steps=finish;p.certify_complete_plan=lambda moves:None;p.layout_resume_count=0
 p.opening_check={};p.layout_candidates=[([.3,-.2],1)];calls=[];yellow=[0]
 def transfer(start,scene,color,target):
  calls.append(color)
  if color=='yellow':
   yellow[0]+=1
   if yellow[0]==1:raise a.CandidateBudgetExceeded('slice')
   if yellow[0]==2:raise a.NoPlan('unresolved fast IK')
  updated=copy.deepcopy(scene);updated['blocks'][color]['top_center']['surface_position_m']=target
  return {'color':color,'scene_after':updated,'end14':list(start),'actions':[]}
 p.transfer=transfer
 with patch.object(a,'pickup_source_conflicts',return_value=[]),patch.object(a,'ranked_orders',return_value=[{'order':list(COLORS),'blocked_pickups':0}]):
  g=p._layout_steps([0.]*14,[.3,-.2],1)
  assert next(g)['resume'] and calls==list(COLORS)
  assert next(g)['needs_full']
  p.ik_mode='full'
  with pytest.raises(StopIteration) as stop:next(g)
 assert stop.value.value['order']==list(COLORS)
 assert all(calls.count(c)==1 for c in COLORS[:-1])
 assert calls.count('yellow')==3


def test_native_certification_can_finish_after_slice_but_not_global_budget():
 p=planner();p.search_active=True;p.started=0.;p.ik_mode='fast';p.candidate_deadline=2.5;p.candidate_yielded=True
 clock=[2.];visited=[]
 def certify(moves):
  for t in (2.,3.,13.):clock[0]=t;p.progress();visited.append(t)
 with patch.object(a.time,'monotonic',side_effect=lambda:clock[0]),patch.object(p,'certify_moves',side_effect=certify):
  p.certify_complete_plan([])
 assert visited==[2.,3.,13.] and p.candidate_deadline==2.5
 with patch.object(a.time,'monotonic',return_value=PREFLIGHT_BUDGET_S+1),patch.object(p,'certify_moves',side_effect=lambda _:p.progress()):
  with pytest.raises(a.PlanningTimeout):p.certify_complete_plan([])
 assert not p.certifying_complete_plan
