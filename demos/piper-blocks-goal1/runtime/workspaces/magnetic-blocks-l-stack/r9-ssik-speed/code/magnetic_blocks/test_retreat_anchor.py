import unittest,copy
from types import SimpleNamespace
from unittest.mock import patch
import task_geometry as g
import composition as c
from scene_checks import PerceptionUnknown
class RetreatAnchor(unittest.TestCase):
 def test_small_shortfall_reanchors_upward_not_accepts_lower_exit(self):
  current=[.335,-.064,.299863537]
  result=g.retreat_anchor([.335,-.064,.299700796],current)
  self.assertEqual(result['position'],current);self.assertTrue(result['reanchored'])
 def test_generous_five_mm_binding_keeps_higher_clearance(self):
  result=g.retreat_anchor([.303,.004,.35],[.3,0,.3])
  self.assertEqual(result['position'],[.3,0,.35])
  result=g.retreat_anchor([.3,0,.2951],[.3,0,.3]);self.assertEqual(result['position'][2],.3)
 def test_larger_stale_target_and_nonfinite_rejected(self):
  for p in ([.306,0,.3],[.3,0,.294],[.3,0,float('nan')]):
   with self.subTest(p=p),self.assertRaises(PerceptionUnknown):g.retreat_anchor(p,[.3,0,.3])
 def test_stack_reversal_uses_current_start_within_allowance(self):
  r=g.retreat_anchor([.3,0,.303],[.3,0,.3],reverse_to_start=True);self.assertEqual(r['position'],[.3,0,.3])
  with self.assertRaises(PerceptionUnknown):g.retreat_anchor([.3,0,.306],[.3,0,.3],reverse_to_start=True)
 def pose(self,mode,previous,records=None):
  p=[.335,-.064,.299863537]
  with patch('composition._current',return_value=({}, {},previous,{'position':p,'quaternion_xyzw':[1,0,0,0]})),patch('composition.read_report',side_effect=lambda name:records[name] if records else (_ for _ in ()).throw(AssertionError('Do not reread old nominal plan'))):
   return c.pose_inputs(SimpleNamespace(),previous_report='prev',mode=mode,placement_plan_report='obsolete')
 def fixture(self):return {'facts':[{'scene':{'table':{'z_ax_by_c':[0,0,.02]}},'payload_origin':{'flange_to_bottom_m':.185}}]}
 def test_first_lower_uses_actual_height_not_previous_nominal_plan(self):
  r=self.pose('orange_lower_1',self.fixture());self.assertTrue(r['ok'],r['why']);self.assertEqual(r['inputs']['retreat'],[[.335,-.064,.299863537]])
 def test_second_lower_retains_validated_higher_clearance(self):
  prev=self.fixture();prev['facts'][0]['retreat_position']=[.33,-.06,.32]
  r=self.pose('orange_lower_2',prev);self.assertTrue(r['ok']);self.assertEqual(r['inputs']['retreat'],[[.335,-.064,.32]])
 def test_green_exit_consumes_effective_reanchored_retreat(self):
  records={'prev':{'inputs':{'previous_report':'lower'}},'lower':{'inputs':{'retreat':[[.3,0,.301]]},'result':{'facts':[{'retreat_position':[.3,0,.304]}]}}}
  r=self.pose('green_retreat',self.fixture(),records);self.assertTrue(r['ok']);self.assertEqual(r['inputs']['waypoints'][0][2],.304)
