import unittest,itertools,copy
import numpy as np
from unittest.mock import patch
import scene_checks as s
import controller as c
from types import SimpleNamespace

def box(lo,hi):return np.array(list(itertools.product(*zip(lo,hi))))
def scene(pos=(.30,-.04,.08)):
 return {'table':{'z_ax_by_c':[0.,0.,.02]},'blocks':{'red':{'top_center':{'surface_position_m':list(pos)}}}}
def obs():
 return {'measured14':[0.]*6+[.0003]+[0.]*6+[.002],
  'finger_geometry':{'geometry_identity':{'representation':'link6_axis_aligned_finger_envelopes_v1'},
   'measured_full_gap_m':.0003,'jaw_motion_model':'linear_prismatic_full_gap_v1',
   'fingers':[{'finger':i,'cad_corners_base_m':box([.20,-.01,.195],[.27,.01,.26]).tolist(),
   'translation_per_full_gap_base':[0.,sgn*.5,0.]} for i,sgn in [(1,1),(2,-1)]]}}
class SpatialGates(unittest.TestCase):
 def test_old_startup_xy_false_rejection(self):
  from task_geometry import opening_sweep
  r=opening_sweep(obs(),scene(),.069)
  self.assertGreater(r['min_block_separation_lower_m'],.09)
 def test_mid_sweep_obstacle_not_just_endpoints(self):
  from task_geometry import opening_sweep
  o=obs();o['finger_geometry']['fingers'][0]['cad_corners_base_m']=box([.2,0,.12],[.22,.002,.14]).tolist()
  with self.assertRaises(s.PerceptionUnknown):opening_sweep(o,scene((.21,.02,.14)),.069)
 def test_below_table_rejected(self):
  from task_geometry import envelope_clearance
  with self.assertRaises(s.PerceptionUnknown):envelope_clearance([box([.1,0,0],[.2,.03,.03])],scene())
 def test_far_lateral_object_does_not_require_flying_over_it(self):
  from task_geometry import envelope_clearance
  r=envelope_clearance([box([.1,0,.07],[.15,.03,.10])],scene((.4,0,.08)))
  self.assertGreater(r['min_block_separation_lower_m'],.1)
 def test_overlap_never_passes(self):
  from task_geometry import envelope_clearance
  with self.assertRaises(s.PerceptionUnknown):envelope_clearance([box([.28,-.06,.04],[.32,-.02,.07])],scene())
 def test_missing_axis_or_bad_model_rejected(self):
  from task_geometry import opening_sweep
  for key in ('geometry_identity','jaw_motion_model'):
   o=obs();o['finger_geometry'].pop(key)
   with self.assertRaises(s.PerceptionUnknown):opening_sweep(o,scene(),.069)
  o=obs();o['finger_geometry']['fingers'][0]['translation_per_full_gap_base']=[float('nan'),0,0]
  with self.assertRaises(s.PerceptionUnknown):opening_sweep(o,scene(),.069)
 def test_opening_native_refusal_and_pad_diagnostics(self):
  o=obs();o.update(frame_id='f',cameras={'left':{'inspection_image':'dummy'}})
  for clear in (False,True):
   ctx=SimpleNamespace(value=lambda name,**args:{'cad_clear':clear,'workspace_clear':clear})
   with patch('scene_checks.measure_scene',return_value=scene()),patch('scene_checks.cv2.imread',return_value=np.zeros((720,1280,3),np.uint8)):
    r=s.empty_opening(ctx,o,.069)
   self.assertEqual(r['ok'],True if clear else None)
   if clear:self.assertEqual(r['wrist_pad_pixels'],[0,0])
   else:self.assertIn('CAD/workspace',r['why'])
 def test_specific_reason_survives_summary(self):
  r=c._run(SimpleNamespace(),'initial_state',lambda r:c._fact(lambda **k:{'ok':None,'why':'Obstacle red: gap -0.003m'},'opening',{}, {'frame_id':'f'},{}),'open')
  self.assertIn('gap -0.003m',r['why'])
if __name__=='__main__':unittest.main()
