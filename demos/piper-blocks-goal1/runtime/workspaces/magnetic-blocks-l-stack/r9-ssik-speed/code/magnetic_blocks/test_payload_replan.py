import unittest
from unittest.mock import patch
import test_assembly_aplus as ta
from assembly_plan import NoPlan

class CorrectionDispatch(unittest.TestCase):
 def setUp(self):
  self.h=ta.Execution();self.h.setUp();self.addCleanup(self.h.doCleanups)
  self.e=self.h.execution;self.e.held='orange';self.e.payload={'color':'orange'}
  self.action=self.h.action();self.action.update(stage='orange-transfer',payload=self.e.payload)
 def test_adjust_unexecuted_leg_then_rejoin(self):
  calls=[]
  def check(*args,**kw):
   calls.append(1)
   if len(calls)==1:raise NoPlan('Payload/red path conflict')
   return {'min_payload_neighbor_m':.03}
  self.h.planner.check=check
  a=self.action['start14'];b=self.action['target14'];m=a.copy();m[0]=.05
  fix={'parts':[{'start14':a,'target14':m},{'start14':m,'target14':b}],'rise_m':.005,'rejoins_original_endpoint':True}
  with patch('payload_replan.corrected_leg',return_value=fix) as repair:r=self.e.run(self.action)
  self.assertTrue(r['ok'],r.get('why'));self.assertEqual(repair.call_count,1)
  self.assertEqual(sum(t=='piper_move_joints' for t,a in self.h.f.calls),2)
  self.assertEqual(self.e.expected[:6],b[:6])
 def test_no_feasible_correction_sends_no_action(self):
  self.h.planner.check=lambda *a,**kw:(_ for _ in ()).throw(NoPlan('blocked'))
  with patch('payload_replan.corrected_leg',side_effect=NoPlan('no correction')):r=self.e.run(self.action)
  self.assertFalse(r['ok']);self.assertFalse(any(t=='piper_move_joints' for t,a in self.h.f.calls))
 def test_contact_failure_is_not_automatically_adjusted(self):
  self.action.update(stage='orange-lower',contact=True)
  self.h.planner.check=lambda *a,**kw:(_ for _ in ()).throw(NoPlan('contact blocked'))
  with patch('payload_replan.corrected_leg') as repair:r=self.e.run(self.action)
  self.assertFalse(r['ok']);repair.assert_not_called()

 def test_preferred_reserve_shortfall_does_not_force_vertical_repair(self):
  self.h.planner.check=lambda *a,**kw:{'min_payload_neighbor_m':.02263678}
  with patch('payload_replan.corrected_leg',side_effect=AssertionError('must use passed direct path')) as repair:r=self.e.run(self.action)
  self.assertTrue(r['ok'],r.get('why'));repair.assert_not_called()
  self.assertFalse(r['payload_clearance']['preferred_met'])
  self.assertEqual(sum(t=='piper_move_joints' for t,a in self.h.f.calls),1)
