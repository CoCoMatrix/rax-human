import copy,json,unittest
from types import SimpleNamespace
from unittest.mock import patch
import composition as g
from scene_checks import PerceptionUnknown

def ik_refusal():return RuntimeError('MCP tool failed: '+json.dumps({'error':{'recovery_action':'replan_read_only','control_uncertain':False,'no_action_executed':True,'dispatch_started':False}}))
class Adaptive(unittest.TestCase):
 def setup_case(self):
  robot={'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36]};obs={'frame_id':'o','measured14':[0.]*14};fk={'position':[.3,-.1,.3],'quaternion_xyzw':[.707,-.707,0,0]};scene={'blocks':{'green':{'top_center':{'surface_position_m':[.3,-.25,.078]}}}}
  return robot,obs,fk,scene
 def test_green_raise_routes_to_shared_path_planner(self):
  r,o,f,s=self.setup_case();f['position']=[.3,-.25,.216]
  with patch('composition._current',return_value=(r,o,{},f)),patch('lift_plan.choose_lift',return_value={'selected':{'position':[.3,-.25,.2685]}}) as planner:
   result=g.pose_inputs(SimpleNamespace(),previous_report='trial',mode='green_raise_1',color='green',support_report='support')
  self.assertTrue(result['ok']);self.assertAlmostEqual(result['inputs']['waypoints'][0][2],.2685)
  self.assertEqual(planner.call_args.args[-1],'support')
 def test_failed_lift_planning_cannot_be_replaced_by_fixed_height(self):
  r,o,f,s=self.setup_case()
  with patch('composition._current',return_value=(r,o,{},f)),patch('lift_plan.choose_lift',side_effect=PerceptionUnknown('Missing pickup history')):
   result=g.pose_inputs(SimpleNamespace(),previous_report='trial',mode='green_raise_1',color='green')
  self.assertFalse(result['ok']);self.assertFalse(result['dispatch_attempted'])
