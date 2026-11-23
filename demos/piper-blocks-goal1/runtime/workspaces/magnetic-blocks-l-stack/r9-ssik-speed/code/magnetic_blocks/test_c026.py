import copy
from types import SimpleNamespace
import numpy as np
import pytest
import assembly_plan as ap
from numeric_policy import clearance_minimum
from payload_replan import bind_pickup
from test_composition import SceneContext

def fixture():
 p=ap.Planner.__new__(ap.Planner);p.ik_mode='fast';p.search_active=True;p.defer_native=True
 p.fk=lambda a:{'position':a[:3],'quaternion_xyzw':[1.,0.,0.,0.]}
 p.check=lambda *a,**k:{}
 p.solve=lambda d,q,a:list(d)+a[3:]
 p.joint_route=lambda a,b,scene,**k:[{'start14':a,'target14':b,'check':{}}]
 return p

def test_exact_edge_reuse_and_scene_invalidation():
 p=fixture();s=ap.initial_scene(SceneContext().scene);a=[0.]*14;calls=[]
 old=p.solve;p.solve=lambda *args:calls.append(args) or old(*args)
 r=p.connect_edge(a,[.2,0,.3],[1,0,0,0],s)
 r[0][0]['target14'][0]=99
 assert p.connect_edge(a,[.2,0,.3],[1,0,0,0],s)[1][0]==.2
 assert len(calls)==1
 changed=copy.deepcopy(s);changed['blocks']['red']['top_center']['surface_position_m'][0]+=.001
 p.connect_edge(a,[.2,0,.3],[1,0,0,0],changed);assert len(calls)==2

def test_fast_failure_does_not_poison_long_budget():
 p=fixture();s=ap.initial_scene(SceneContext().scene);a=[0.]*14;calls=[]
 def solve(*args):calls.append(1);raise ap.IKUnavailable('No full-pose IK')
 p.solve=solve
 for _ in range(2):
  with pytest.raises(ap.NoPlan):p.connect_edge(a,[.2,0,.3],[1,0,0,0],s)
 assert len(calls)==1
 p.ik_mode='full'
 with pytest.raises(ap.NoPlan):p.connect_edge(a,[.2,0,.3],[1,0,0,0],s)
 assert len(calls)==2

def test_explicit_one_mm_added_to_old_geometry_allowance():
 assert clearance_minimum(.020)==pytest.approx(.0185)
 assert ap.finger_gap_minimum(.003)==pytest.approx(0.)
 assert .019481>clearance_minimum(.020) and .019269>clearance_minimum(.020)
 assert 0<=ap.finger_gap_minimum(.003)  # Touch/overlap is separately refused in task and native checks.

@pytest.mark.parametrize('color',['red','orange','green'])
def test_small_measured_pickup_uses_preview_without_route_search(color):
 p=fixture();p.fk=lambda q:{'position':q[:3],'quaternion_xyzw':[1,0,0,0]}
 p.route=lambda *a,**k:(_ for _ in ()).throw(AssertionError('not needed before actual wire preview'))
 a=[.3,-.2,.2]+[0.]*11;b=a.copy();b[2]+=.03
 action={'start14':a,'target14':b,'pickup':True,'color':color}
 actual=a.copy();actual[0]+=.0003
 e=SimpleNamespace(planner=p,payload={'color':color},held=color,scene={})
 out,fact=bind_pickup(e,[action],actual,prefer_preview=True)
 assert fact['actual_wire_check_required'] and not fact['rebound']
 assert out[0]['start14']==actual and action['start14']==a

def test_far_corridor_changes_ranking_without_discarding_original_layout():
 s=SceneContext()
 for color,position in zip(('red','orange','green'),((.244056,-.280094,.081),(.266535,-.125930,.078),(.411988,-.214910,.079))):
  s.scene['blocks'][color]['top_center']['surface_position_m']=list(position)
 layouts=ap.layout_candidates(s.scene,s.robot())
 assert layouts[0][0]!=[.244056,-.280094]
 assert any(a==[.244056,-.280094] for a,side in layouts)
