import copy,unittest
from unittest.mock import patch
from types import SimpleNamespace
import assembly_batch as ab
import test_assembly_aplus as ta
from assembly_plan import NoPlan

class Boundaries(unittest.TestCase):
 def test_only_same_free_stage_grouped(self):
  def arm(stage,**opts):return dict(kind='arm',stage=stage,**opts)
  actions=[arm('orange-hover'),arm('orange-hover'),arm('orange-entry',contact=True),dict(kind='jaw',stage='orange-close'),arm('orange-trial',contact=True),arm('orange-raise'),arm('orange-transfer'),arm('orange-transfer'),arm('orange-lower',contact=True)]
  self.assertEqual([len(g) for g in ab.groups(actions)],[2,1,1,1,1,2,1])
 def test_eight_point_bound(self):
  self.assertEqual([len(g) for g in ab.groups([dict(kind='arm',stage='green-hover')]*9)],[8,1])

class Execution(unittest.TestCase):
 def setUp(self):
  self.h=ta.Execution();self.h.setUp();self.addCleanup(self.h.doCleanups);self.e=self.h.execution
  a=self.h.action();b=copy.deepcopy(a);b['start14']=a['target14'];b['target14']=b['target14'].copy();b['target14'][0]=.2;self.actions=[a,b]
 def test_invalid_preview_falls_back_before_any_batch_dispatch(self):
  with patch('assembly_batch.preview',side_effect=NoPlan('curve conflicts')):r=ab.run(self.e,self.actions)
  self.assertTrue(r['ok'],r['why']);self.assertEqual(sum(t=='piper_move_joints' for t,a in self.h.f.calls),2)
  self.assertFalse(any(t=='piper_move_waypoints' for t,a in self.h.f.calls))
 def test_dispatch_exception_never_replays_original_legs(self):
  proposal=dict(arguments={},target14=self.actions[-1]['target14'],check={},samples=3,duration_s=2,waypoint_boundaries=[])
  proposal['arguments']['blend_indices']=[0]
  with patch('assembly_batch.preview',return_value=proposal),patch('assembly_batch.c._move',side_effect=RuntimeError('ambiguous wire')) as move:r=ab.run(self.e,self.actions)
  self.assertFalse(r['ok']);self.assertEqual(move.call_count,1)
  self.assertFalse(any(t=='piper_move_joints' for t,a in self.h.f.calls))
 def test_success_one_native_dispatch(self):
  proposal=dict(arguments={'blend_indices':[0]},target14=self.actions[-1]['target14'],check={},samples=3,duration_s=2,waypoint_boundaries=[])
  def move(ctx,tool,args,report,**kw):
   self.assertEqual(tool,'piper_move_waypoints');self.assertFalse(kw['observe_after']);report['receipts']=[{'result':{}}];return {'control_feedback_only':True}
  with patch('assembly_batch.preview',return_value=proposal),patch('assembly_batch.c._move',side_effect=move) as m,patch.object(self.e,'receipt') as receipt:r=ab.run(self.e,self.actions)
  self.assertTrue(r['ok'],r['why']);self.assertEqual(m.call_count,1);self.assertEqual(receipt.call_count,1)
