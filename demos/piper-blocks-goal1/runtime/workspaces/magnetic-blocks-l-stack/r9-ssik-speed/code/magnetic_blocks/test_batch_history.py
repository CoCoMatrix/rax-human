"""Regression for native waypoint receipts through real retained-scene validation."""
import copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import scene_checks as s
from test_occlusion_repair import H,record

class BatchHistory(unittest.TestCase):
 def setUp(self):
  self.raw=record('r4-approach-mid-01')
  self.obs=self.raw['result']['measured']
  self.receipt=self.raw['result']['receipts'][0]
  self.receipt['tool']='piper_move_waypoints'
  self.receipt['arguments']={'waypoints':[[0.]*6 for _ in range(3)]}
  self.receipt['result'].update(sdk_completed_steps=429,trajectory_progress={
   'phase':'DONE','fraction':1.,'total_samples':429,'dispatched_samples':429,'waypoint_index':3})
 def retain(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'batch.json';p.write_text(json.dumps(self.raw))
   with patch('scene_checks.time.time_ns',return_value=self.obs['captured_at_ns']):
    return s.retain_full_scene(H[0],H[:-1]+[str(p)],self.obs)
 def test_completed_batch_preserves_scene(self):
  out=self.retain()
  self.assertEqual(out['blocks'],record('r4-transition-01')['result']['facts'][0]['scene']['blocks'])
 def test_partial_progress_rejected(self):
  a=self.receipt['result'];saved=copy.deepcopy(a)
  for key,value in [('phase','RUNNING'),('fraction',.9),('total_samples',0),('dispatched_samples',428),('waypoint_index',2)]:
   with self.subTest(key=key):
    a.clear();a.update(copy.deepcopy(saved));a['trajectory_progress'][key]=value
    with self.assertRaisesRegex(s.PerceptionUnknown,'Incomplete waypoint'):self.retain()
 def test_incomplete_sdk_rejected(self):
  self.receipt['result']['sdk_completed_steps']=428
  with self.assertRaisesRegex(s.PerceptionUnknown,'Incomplete waypoint'):self.retain()
 def test_missing_progress_rejected(self):
  del self.receipt['result']['trajectory_progress']
  with self.assertRaisesRegex(s.PerceptionUnknown,'Incomplete waypoint'):self.retain()
 def test_unsettled_batch_rejected(self):
  self.receipt['result']['measured_settled']=False
  with self.assertRaisesRegex(s.PerceptionUnknown,'incomplete native'):self.retain()
 def test_closure_tool_still_rejected(self):
  self.receipt['tool']='piper_set_gripper'
  with self.assertRaisesRegex(s.PerceptionUnknown,'closure/contact'):self.retain()
 def test_wrong_owner_still_rejected(self):
  self.receipt['result']['owner_generation']=-1
  with self.assertRaisesRegex(s.PerceptionUnknown,'Gap or intervening'):self.retain()
 def test_closed_jaw_still_rejected(self):
  self.receipt['result']['final14'][6]=.04
  with self.assertRaisesRegex(s.PerceptionUnknown,'Jaw closure'):self.retain()
 def test_loaded_stage_still_rejected(self):
  self.raw['result']['stage']='transport'
  with self.assertRaisesRegex(s.PerceptionUnknown,'nonempty'):self.retain()
if __name__=='__main__':unittest.main()
