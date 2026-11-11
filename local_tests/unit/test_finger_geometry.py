import unittest
import numpy as np
from services.piper.finger_geometry import inspection_geometry
from services.piper.geometry import Kinematics
class Geometry(unittest.TestCase):
    def head(self):return {'metric_depth':True,'K':[[900,0,640],[0,900,360],[0,0,1]],'T_base_camera':np.eye(4).tolist()}
    def test_projection_and_width_are_consistent(self):
        q=[0.]*14; q[6]=.02;r=inspection_geometry(q,self.head())
        self.assertFalse(r['physical_clearance_authority']);self.assertFalse(r['installed_tcp_validated'])
        for f in r['fingers']:
            for xyz,uv in zip(f['cad_corners_base_m'],f['head_pixels_native']):
                self.assertAlmostEqual(uv[0],900*xyz[0]/xyz[2]+640)
        q[6]=.04;s=inspection_geometry(q,self.head());rotation=Kinematics().fk([0.]*6)[:3,:3]
        deltas=[np.array(b['cad_corners_base_m'])-np.array(a['cad_corners_base_m']) for a,b in zip(r['fingers'],s['fingers'])]
        self.assertTrue(np.allclose(deltas[0],-deltas[1],atol=1e-8))
        self.assertAlmostEqual(float(np.linalg.norm(deltas[0][0])),.01)
    def test_wrong_frame_or_gap_rejected(self):
        q=[0.]*14;q[6]=.08
        with self.assertRaises(ValueError):inspection_geometry(q,self.head())
        with self.assertRaises(ValueError):inspection_geometry([0.]*14,{'metric_depth':False})
if __name__=='__main__':unittest.main()
