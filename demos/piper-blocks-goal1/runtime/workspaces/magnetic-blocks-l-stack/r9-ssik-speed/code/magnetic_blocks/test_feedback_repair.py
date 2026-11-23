import unittest,copy
from unittest.mock import patch
from types import SimpleNamespace
import assembly_batch as ab
from assembly_plan import NoPlan
from history_scene import reject_visible_displacement
from scene_checks import PerceptionUnknown
import numpy as np

class BatchBinding(unittest.TestCase):
 def make(self,gap,nominal,payload):
  start=[0.]*14;start[6]=gap;start[13]=.002
  a=start.copy();a[6]=nominal;b=a.copy();b[0]=.1;c=b.copy();c[0]=.2
  actions=[dict(kind='arm',stage='green-transfer' if payload else 'green-hover',color='green',payload=payload,start14=a,target14=b),dict(kind='arm',stage='green-transfer' if payload else 'green-hover',color='green',payload=payload,start14=b,target14=c)]
  def value(name,**kw):
   self.assertEqual(name,'piper_preview_waypoints');self.assertEqual(kw['start14'][6],gap)
   return dict(targets14=[q+start[6:] for q in kw['waypoints']],review={'cad_clear':True,'workspace_clear':True},duration_s=2,waypoints=[{"sample_end":1},{"sample_end":2}])
  e=SimpleNamespace(ctx=SimpleNamespace(value=value),planner=SimpleNamespace(check=lambda *a,**k:{},_check=lambda *a,**k:{}),scene={},payload=payload,held='green' if payload else None)
  return e,actions,start
 def test_actual_jaw_does_not_have_to_equal_nominal_command(self):
  for actual,planned,payload in [(.0725,.073,None),(.0511,.05,{'color':'green'}),(.0512,.05,{'color':'green'})]:
   with self.subTest(actual=actual):
    e,a,s=self.make(actual,planned,payload)
    with patch('assembly_batch.boxes',return_value=[{'synthetic':True}]):r=ab.preview(e,a,s)
    self.assertEqual(r['arguments']['start14'][6],actual)
 def test_out_of_range_grasp_and_joint_drift_still_refused(self):
  for gap,joint in [(.056,0.),(.051,.02)]:
   e,a,s=self.make(gap,.05,{'color':'green'});s[0]=joint
   with self.assertRaises(NoPlan):ab.preview(e,a,s)

class TargetAssociation(unittest.TestCase):
 def test_local_target_beats_unrelated_global_patch(self):
  scene={'blocks':{'green':{'top_center':{'surface_position_m':[0.,0.,1.]}}}}
  obs={'cameras':{'head':{'T_base_camera':np.eye(4).tolist(),'K':np.eye(3).tolist()}}}
  with patch('history_scene.current_image',return_value=np.zeros((720,1280,3),np.uint8)),patch('ordinary_checks.visible_near_point',return_value={'visible':True}),patch('perception.component',side_effect=AssertionError('background search must not veto local target')):
   reject_visible_displacement(scene,obs)
 def test_unassociated_color_is_not_proof_of_target_displacement(self):
  scene={'blocks':{'green':{'top_center':{'surface_position_m':[0.,0.,1.]}}}}
  obs={'cameras':{'head':{'T_base_camera':np.eye(4).tolist(),'K':np.eye(3).tolist()}}}
  with patch('history_scene.current_image',return_value=np.zeros((720,1280,3),np.uint8)),patch('ordinary_checks.visible_near_point',return_value={'visible':False}),patch('perception.component',return_value=({'bbox_xywh':[400,400,50,50]},None)):
   self.assertEqual(reject_visible_displacement(scene,obs)['unassociated_candidates'][0]['color'],'green')
