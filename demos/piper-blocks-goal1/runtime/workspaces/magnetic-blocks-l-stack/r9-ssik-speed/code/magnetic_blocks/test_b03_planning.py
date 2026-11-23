import copy
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np,pytest
import assembly_plan as a
from test_composition import SceneContext

def scene():return a.initial_scene(SceneContext().scene)

def put(s,c,x,y,z=None):
 p=s['blocks'][c]['top_center']['surface_position_m'];p[:2]=[x,y]
 if z is not None:p[2]=z

def test_outer_layout_bounds_and_table_cover_same_extended_area():
 assert a.LAYOUT_MIN.tolist()==[.09,-.42] and a.LAYOUT_MAX.tolist()==[.52,.065]
 t=a.boxes(scene())[0];center=np.array(t['center_m'][:2])-[0,.30];size=np.array(t['size_m'][:2])
 assert center-size/2==pytest.approx([.11,-.42]);assert center+size/2==pytest.approx(a.LAYOUT_MAX)
 # Reported red outer edge55.77mm now fits, beyond65mm still excluded.
 assert .0257724+.03<=a.LAYOUT_MAX[1];assert .036+.03>a.LAYOUT_MAX[1]

def test_order_changes_with_predicted_blocking_not_color_priority():
 s=scene();put(s,'red',.3,-.2);put(s,'orange',.3,-.05);put(s,'green',.45,-.35)
 target=a.goal_scene(s,[.3,-.28],1)
 assert a.ranked_orders(s,target,[.2,-.1])[0]['order'][0]=='red'
 target=a.goal_scene(s,[.3,-.08],1)
 assert a.ranked_orders(s,target,[.2,-.1])[0]['order'][0]=='orange'

def test_blocked_pickup_rejects_before_any_ik_or_path_enumeration():
 s=scene();put(s,'red',.3,-.1);put(s,'orange',.3,-.175)
 p=a.Planner.__new__(a.Planner)
 p.solve=lambda *args:pytest.fail('impossible start must be pruned first')
 with pytest.raises(a.NoPlan,match='Pickup source blocked'):p.transfer([0.]*14,s,'orange',[.3,-.25,.078])

def test_prefix_cache_exact_binding_deepcopy_and_invalidation():
 s=scene();p=a.Planner.__new__(a.Planner);p.progress=lambda **kw:None;p.check=lambda *args,**kw:{};calls=[]
 def route(start,dest,q,scene,**kw):
  calls.append((start,dest));return [{'start14':list(start),'target14':list(dest)+[0.]*3+start[6:],'check':{}}]
 def fk(q):
  T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=q[:3];return {'T_base_flange':T.tolist()}
 p.route=route;p.fk=fk;start=[0.]*14;start[6]=a.OPEN
 first=p.pickup_prefix(start,s,'orange',180.,.195,None);second=p.pickup_prefix(start,s,'orange',180.,.195,None)
 assert len(calls)==2 and first==second
 second['actions'][0]['target14'][0]=99
 assert p.pickup_prefix(start,s,'orange',180.,.195,None)==first
 moved=copy.deepcopy(s);moved['blocks']['green']['top_center']['surface_position_m'][0]+=.01
 p.pickup_prefix(start,moved,'orange',180.,.195,None);assert len(calls)==4

def test_flange_workspace_still_rejects_outside_actual_tool_contract():
 p=a.Planner.__new__(a.Planner);p.progress=lambda:None;p.robot={'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36]}
 with pytest.raises(a.NoPlan,match='workspace'):p.solve([.51,0,.25],[1.,0,0,0],[0.]*14)

def test_loaded_invalid_start_is_checked_before_detours():
 p=a.Planner.__new__(a.Planner);seen=[]
 def check(*args,**kw):
  seen.append(kw);raise a.NoPlan('occupied actual start')
 p.check=check;p.fk=lambda *args:pytest.fail('No route construction after invalid start')
 with pytest.raises(a.NoPlan,match='occupied actual start'):
  p._route([0.]*14,[.3,0,.3],[1.,0,0,0],scene(),active='orange',payload={'color':'orange'})
 assert seen[0]['geometry_only'] and seen[0]['reserve_m']==.003
