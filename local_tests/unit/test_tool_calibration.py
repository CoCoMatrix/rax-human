import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from services.piper.geometry import Kinematics
from services.piper.tool_calibration import load_calibration,tool_pose
from services.piper.finger_geometry import inspection_geometry
class ToolCalibration(unittest.TestCase):
 def test_operator_measurements_not_claimed_independently_validated(self):
  c=load_calibration();self.assertEqual(c['C1_axial_m'],.14);self.assertEqual(c['C2_axial_m'],.14)
  self.assertFalse(c['independent_physical_validation']);self.assertFalse(c['right_tcp_available'])
 def test_rotated_tcp_and_inverse_roundtrip(self):
  for e in ([0,0,0],[180,0,0],[20,40,60]):
   t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',e,degrees=True).as_matrix();t[:3,3]=[.23,-.1,.3]
   r=tool_pose(t);a=np.array(r['T_base_tcp']);back=a@np.linalg.inv(np.array(load_calibration()['T_flange_tcp']))
   np.testing.assert_allclose(back,t,atol=1e-12)
   self.assertAlmostEqual(np.linalg.norm(a[:3,3]-t[:3,3]),.14)
   self.assertAlmostEqual(np.linalg.norm(np.array(r['tip_axis_point_base_m'])-a[:3,3]),.03)
 def test_downward_tip_is_below_tcp(self):
  t=np.eye(4);t[:3,:3]=Rotation.from_euler('x',180,degrees=True).as_matrix();t[:3,3]=[.23,-.1,.3]
  r=tool_pose(t);self.assertAlmostEqual(r['position'][2],.16);self.assertAlmostEqual(r['tip_axis_point_base_m'][2],.13)
 def test_envelope_reaches170_in_flange_frame(self):
  q=[0.]*14;q[6]=.0184;t=Kinematics().fk(q[:6]);inv=np.linalg.inv(t)
  head={'metric_depth':True,'K':[[900,0,640],[0,900,360],[0,0,1]],'T_base_camera':np.eye(4).tolist()}
  r=inspection_geometry(q,head)
  for f in r['fingers']:
   a=np.array(f['cad_corners_base_m']);b=(inv@np.c_[a,np.ones(8)].T).T
   self.assertAlmostEqual(b[:,2].max(),.17)
  self.assertFalse(r['physical_clearance_authority'])
 def test_invalid_transform(self):
  with self.assertRaises(ValueError):tool_pose(np.full((4,4),np.nan))
if __name__=='__main__':unittest.main()
