"""Four-block task contracts; synthetic checks are not physical qualification."""
import copy,json
from pathlib import Path
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
import assembly_plan as a
import ordinary_checks as ordinary
from task_spec import COLORS,SUPPORTS

class FourTask(unittest.TestCase):
 def setUp(self):
  self.scene={'table':{'z_ax_by_c':[0,0,.018]},'blocks':{c:{'top_center':{'surface_position_m':[.2+i*.075,-.2,.078]},'height_m':.06} for i,c in enumerate(COLORS)}}
 def test_goal_has_three_level_support_chain_and_adjacent_base(self):
  for side in (-1,1):
   g=a.goal_scene(self.scene,[.24,-.15],side);a.assert_goal(g)
   self.assertAlmostEqual(a.xyz(g,'yellow')[2],.198)
   np.testing.assert_allclose(a.xyz(g,'yellow')-a.xyz(g,'green'),[0,0,.06])
   np.testing.assert_allclose(a.xyz(g,'green')-a.xyz(g,'red'),[0,0,.06])
   self.assertEqual(set(g['blocks']),set(COLORS))
 def test_missing_yellow_or_wrong_support_not_silent_three_block_success(self):
  del self.scene['blocks']['yellow']
  with self.assertRaises(a.NoPlan):a.initial_scene(self.scene)
  self.setUp();g=a.goal_scene(self.scene,[.24,-.15]);g['blocks']['yellow']['top_center']['surface_position_m'][0]+=.065
  with self.assertRaises(a.NoPlan):a.assert_goal(g)
 def test_yellow_is_in_native_obstacles_before_any_pick(self):
  boxes=a.boxes(self.scene);self.assertIn('yellow',[x['name'] for x in boxes])
  self.assertNotIn('finger_clearance_comparison_error_m',boxes[0])
 def test_dependency_order_and_recorded_current_scene(self):
  rows=a.ranked_orders(self.scene,a.goal_scene(self.scene,[.24,-.15]),[.2,0])
  for r in rows:self.assertEqual(r['order'][-2:],('green','yellow'))
  source=Path(__file__).parent.parent/'INITIAL.json'
  if source.exists():
   s=json.loads(source.read_text())['facts'][0]['scene'];a.initial_scene(s)
   self.assertEqual(len(s['blocks']),4)
 def test_yellow_support_uses_green_top_not_table_or_red(self):
  scene=a.goal_scene(self.scene,[.24,-.15]);origin={'color':'yellow','grasp_flange':[.24,-.15,.206],'flange_to_bottom_m':.188}
  T=np.eye(4);T[:3,3]=[.24,-.15,.326]
  origin.update(object_top_flange_m=[0,0,-.128],corners_flange_m=[[x,y,z] for x in (-.03,.03) for y in (-.03,.03) for z in (-.188,-.128)])
  ctx=SimpleNamespace(value=lambda *args,**kw:{'position':T[:3,3].tolist(),'T_base_flange':T.tolist()})
  with patch('ordinary_checks.placed_region_evidence',return_value={'visible':True,'head':{},'wrist':{},'wrist_pair':{}}),patch('history_scene.reject_visible_displacement',return_value={}):
   r=ordinary.placement(ctx,'support',{'frame_id':'offline','measured14':[0.]*14},scene,origin)
  self.assertTrue(r['ok'],r);self.assertAlmostEqual(r['estimated_support_height_error_m'],0.)
 def test_tilted_grasp_uses_tcp_and_preserves_opening_axis(self):
  from scipy.spatial.transform import Rotation
  variants=[g for g in a.grasp_variants(self.scene,'yellow') if g.get('tilt_axis')=='jaw_opening']
  self.assertTrue(variants)
  for g in variants:
   R=Rotation.from_quat(g['quaternion']);nominal=Rotation.from_quat(a.quat(g['yaw']))
   np.testing.assert_allclose(R.apply([1,0,0]),nominal.apply([1,0,0]),atol=1e-12)
   tcp=np.array(g['position'])+R.apply([0,0,.140])
   np.testing.assert_allclose(tcp,a.xyz(self.scene,'yellow')+[0,0,-.012+g['height_delta_m']],atol=1e-12)
   self.assertLessEqual(abs(g['tilt_degrees']),25.)
 def test_other_colors_do_not_get_new_tilt_families(self):
  for c in ('red','orange','green'):
   self.assertTrue(all(abs(g['tilt_degrees'])<=5 for g in a.grasp_variants(self.scene,c)))
 def test_yellow_orange_ranges_do_not_overlap(self):
  from perception import HSV_RANGES,component
  import cv2
  lo=HSV_RANGES['yellow'][0][0][0];hi=HSV_RANGES['orange'][0][1][0];self.assertGreater(lo,hi)
  im=np.zeros((720,1280,3),np.uint8)
  im[150:250,200:300]=[14,210,220];im[400:500,500:600]=[22,210,220]
  bgr=cv2.cvtColor(im,cv2.COLOR_HSV2BGR)
  self.assertEqual(component(bgr,'orange')[0]['bbox_xywh'],[200,150,100,100])
  self.assertEqual(component(bgr,'yellow')[0]['bbox_xywh'],[500,400,100,100])

if __name__=='__main__':unittest.main()
