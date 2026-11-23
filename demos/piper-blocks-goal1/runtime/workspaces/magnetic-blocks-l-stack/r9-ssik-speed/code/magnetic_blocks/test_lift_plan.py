"""Read-only lift/path regressions with synthetic geometry, no hardware/network."""
import copy,json,unittest
from unittest.mock import patch
import numpy as np
from test_composition import SceneContext
from scene_checks import PerceptionUnknown
from lift_plan import select_height
from manipulation_checks import loaded_segment

class KinematicContext(SceneContext):
 def __init__(self):
  super().__init__();self.ceiling=.295;self.fail_transfer=False;self.transport_error=False
 def value(self,tool,**kw):
  if tool=='piper_solve_ik' and self.transport_error:raise RuntimeError('connection lost')
  if tool=='piper_solve_ik' and kw['position'][2]>self.ceiling:
   raise RuntimeError(json.dumps({'error':{'recovery_action':'replan_read_only','control_uncertain':False,'no_action_executed':True,'dispatch_started':False}}))
  if tool=='piper_check_path' and self.fail_transfer:
   a=self.fk(kw['start14'][:6])['position'];b=self.fk(kw['target14'][:6])['position']
   if np.linalg.norm(np.array(a[:2])-b[:2])>.002:return {'cad_clear':False,'workspace_clear':True}
  if tool not in ('piper_fk','piper_solve_ik','piper_check_path'):raise AssertionError('Planning must be numerical only: '+tool)
  return super().value(tool,**kw)

class LiftPlan(unittest.TestCase):
 def case(self):
  ctx=KinematicContext();p=[.303,-.155,.216];q=[1.,0.,0.,0.]
  start=ctx.encode(p,q)+[.050]+[0.]*7
  origin={'color':'orange','grasp_flange':[.303,-.155,.206],'flange_to_bottom_m':.188,'half_xy_m':.035}
  local=[np.array([[x,y,z] for x in (-.01,.01) for y in (-.01,.01) for z in (.135,.145)])]
  return ctx,start,p,q,copy.deepcopy(ctx.scene),origin,local,ctx.robot()
 def plan(self,args):return select_height(*args,'orange_raise')
 def test_selects_reachable_height_and_all_following_legs(self):
  a=self.case();r=self.plan(a);z=r['selected']['position'][2]
  self.assertAlmostEqual(z,.291);self.assertNotAlmostEqual(r['selected']['rise_m'],.085)
  self.assertEqual([p['leg'] for p in r['following_path']['selected']['preflights']],['transfer','descent_1','descent_2','retreat'])
  self.assertGreaterEqual(r['following_path']['transfer_check']['min_neighbor_separation_m'],.020)
 def test_layout_and_payload_height_change_selected_height(self):
  a=self.case();a[0].ceiling=.4;first=self.plan(a)
  a[4]['blocks']['red']['top_center']['surface_position_m'][2]+=.008
  second=self.plan(a);self.assertAlmostEqual(second['selected']['position'][2]-first['selected']['position'][2],.008)
  a[5]['flange_to_bottom_m']+=.002
  third=self.plan(a);self.assertAlmostEqual(third['selected']['position'][2]-second['selected']['position'][2],.002)
 def test_lift_ik_alone_does_not_accept_blocked_transfer(self):
  a=self.case();a[0].fail_transfer=True
  with self.assertRaises(PerceptionUnknown) as caught:self.plan(a)
  self.assertTrue(caught.exception.lift_diagnostics['candidates'])
  self.assertTrue(all(not r['ok'] for r in caught.exception.lift_diagnostics['candidates']))
 def test_transport_failure_is_not_retried_as_candidate_refusal(self):
  a=self.case();a[0].transport_error=True
  with self.assertRaisesRegex(RuntimeError,'connection lost'):self.plan(a)
 def test_neighbor_envelope_still_rejects_collision(self):
  a=self.case();ctx,start,p,q,scene,origin,local,_=a
  scene['blocks']['red']['top_center']['surface_position_m']=[p[0],p[1],.078]
  with self.assertRaisesRegex(PerceptionUnknown,'neighbor'):loaded_segment(ctx,start,[*p[:2],p[2]+.03],q,scene,origin,local)
 def test_exhausted_workspace_no_unchecked_fallback(self):
  a=self.case();a[7]['workspace_max'][2]=.28
  with self.assertRaises(PerceptionUnknown) as caught:self.plan(a)
  self.assertEqual(caught.exception.lift_diagnostics['candidates'],[])
 def test_green_second_height_checks_stack_descent_and_reverse(self):
  ctx,start,p,q,scene,origin,local,robot=self.case();ctx.ceiling=.4
  scene['blocks']['orange']['top_center']['surface_position_m']=[.307,-.082,.078]
  origin.update(color='green',grasp_flange=[.304,-.281,.206]);p=[.304,-.187,.275];start=ctx.encode(p,q)+start[6:]
  r=select_height(ctx,start,p,q,scene,origin,local,robot,'green_raise_2')
  self.assertIn('descent',r['following_path']);self.assertTrue(r['following_path']['retreat']['cad_clear'])
 def test_green_first_checks_following_lift_and_final_placement(self):
  ctx,start,p,q,scene,origin,local,robot=self.case();ctx.ceiling=.4
  scene['blocks']['orange']['top_center']['surface_position_m']=[.307,-.082,.078]
  origin.update(color='green',grasp_flange=[.304,-.281,.206]);p=[.304,-.281,.216];start=ctx.encode(p,q)+start[6:]
  r=select_height(ctx,start,p,q,scene,origin,local,robot,'green_raise_1')
  next_lift=r['following_path']['next_lift']
  self.assertTrue(next_lift['ok']);self.assertIn('descent',next_lift['following_path'])
  self.assertGreater(next_lift['selected']['position'][2],r['selected']['position'][2])
 def test_no_fixed_rise_in_public_input_generation(self):
  import composition
  for name in ('ORANGE_RAISE','GREEN_FIRST_RAISE','GREEN_SECOND_RAISE','ORANGE_RETREAT','ORANGE_FINAL_LOWER'):
   self.assertFalse(hasattr(composition,name))
