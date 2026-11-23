import copy,time
import numpy as np
import pytest
import assembly_plan as ap
from test_composition import SceneContext

def scene():return ap.initial_scene(SceneContext().scene)
def test_loaded_destination_screen_rejects_low_cruise_near_neighbor():
 s=scene();goal=ap.goal_scene(s,[.372,-.062],-1);goal['blocks'].pop('green')
 values=ap.destination_heights(goal,'orange',ap.xyz(goal,'orange'))
 required=ap.xyz(goal,'red')[2]+ap.height(goal,'orange')+.128+ap.clearance_minimum(ap.PAYLOAD_MIN_CLEARANCE_M)
 assert min(values)>=required-1e-9 and min(values)>ap.xyz(goal,'orange')[2]+.158

def test_no_future_green_phantom_obstacle_for_base_screen():
 s=scene();g=ap.goal_scene(s,[.30,-.10],-1);g['blocks'].pop('green')
 assert min(ap.destination_heights(g,'orange',ap.xyz(g,'orange')))<.34

def test_scattered_sources_no_line_assumption_and_extended_goal_grid():
 s=scene();s['blocks']['green']['top_center']['surface_position_m'][0]-=.09
 robot={'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36]}
 layouts=ap.layout_candidates(s,robot)
 assert any(a[0]>.4 for a,side in layouts)
 assert any(a[1]<-.25 for a,side in layouts)
 for a,side in layouts:ap.assert_goal(ap.goal_scene(s,a,side))
 assert len(layouts)==len({(tuple(a),side) for a,side in layouts})

def test_candidate_budget_yields_without_bypassing_checks():
 p=ap.Planner.__new__(ap.Planner);p.search_active=True;p.searching={};p.started=time.monotonic();p.ik_mode='full';p.candidate_deadline=time.monotonic()-1
 with pytest.raises(ap.CandidateBudgetExceeded):p.progress(phase='transfer')
 assert not issubclass(ap.CandidateBudgetExceeded,ap.NoPlan)

def test_symmetric_grasp_alternatives_cover_all_four_faces():
 assert {0.,90.,180.,-90.}.issubset(ap.YAW_DEGREES)
 assert all(min(abs((y-a+180)%360-180) for a in (0,90,180,-90))<=3.000001 for y in ap.YAW_DEGREES)

def test_preferred_clearance_is_not_a_hard_lower_bound():
 s=scene();g=ap.goal_scene(s,[.30,-.10],-1);g['blocks'].pop('green')
 values=ap.destination_heights(g,'orange',ap.xyz(g,'orange'))
 preferred=ap.xyz(g,'red')[2]+ap.height(g,'orange')+.128+ap.PAYLOAD_PREFERRED_CLEARANCE_M
 assert min(values)<preferred and max(values)>=preferred

def test_collision_at_release_endpoint_is_rejected_before_transport_search():
 from types import SimpleNamespace
 from unittest.mock import patch
 s=scene();p=ap.Planner.__new__(ap.Planner);p.ctx=SimpleNamespace();p.progress=lambda **kw:None
 corners=np.zeros((16,3));corners[:8,1]=-.04;corners[8:,1]=.04
 p.local={ap.OPEN:corners};p.prefix_phase='test';p.solve=lambda pos,q,seed:list(pos)+[0.]*3+seed[6:];p.endpoint=p.solve;p.ik_mode='full'
 p.fk=lambda q:{'T_base_flange':np.eye(4).tolist(),'position':q[:3]}
 p.pickup_prefix=lambda *a,**kw:{'start14':[0.]*14,'payload':{'test':True},'actions':[]}
 def refuse(*a,**kw):raise ap.NoPlan('placement endpoint collision')
 p.check=refuse
 p.route=lambda *a,**kw:pytest.fail('Do not sample a transit to a known occupied endpoint')
 with patch.object(ap,'YAW_DEGREES',(180.,)),pytest.raises(ap.NoPlan,match='screen failed|families exhausted'):
  p.transfer([0.]*14,s,'orange',(ap.xyz(s,'orange')+[-.02,0,0]).tolist())
