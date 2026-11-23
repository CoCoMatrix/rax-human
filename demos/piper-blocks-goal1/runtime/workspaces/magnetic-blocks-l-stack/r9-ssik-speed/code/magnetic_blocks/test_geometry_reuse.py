"""Offline regression for RGB-only observation geometry, no robot calls."""
import unittest
from unittest.mock import patch
import numpy as np
import scene_checks as s

class GeometryReuse(unittest.TestCase):
 def setUp(self):
  self.source={'measured14':[0.]*6+[.0685]+[0.]*7,'finger_geometry':{'fingers':[{'cad_corners_base_m':[[1.,2.,3.],[2.,2.,3.]]}]}}
  self.obs={'measured14':self.source['measured14'][:],'finger_geometry':{'projection_available':False}}
  class C:
   def value(self,*a,**k):return {'T_base_flange':np.eye(4).tolist()}
  self.ctx=C()
 def test_rigid_transform_preserves_source_model(self):
  t=np.array([[0.,-1.,0.,.4],[1.,0.,0.,.5],[0.,0.,1.,.6],[0.,0.,0.,1.]])
  with patch.object(s,'scene_geometry',return_value=self.source):
   out=s.installed_finger_corners(self.ctx,self.obs,{}, {'T_base_flange':t.tolist()})
  np.testing.assert_allclose(out,[[-1.6,1.5,3.6],[-1.6,2.5,3.6]])
 def test_changed_jaw_refuses(self):
  self.obs['measured14'][6]=.05
  with patch.object(s,'scene_geometry',return_value=self.source):
   with self.assertRaises(s.PerceptionUnknown):s.installed_finger_corners(self.ctx,self.obs,{})
 def test_missing_source_refuses(self):
  self.source['finger_geometry']={}
  with patch.object(s,'scene_geometry',return_value=self.source):
   with self.assertRaises(s.PerceptionUnknown):s.installed_finger_corners(self.ctx,self.obs,{})
if __name__=='__main__':unittest.main()
