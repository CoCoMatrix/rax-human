"""Regression: receipt reuse is not a before/after trial comparison."""
import copy,time,unittest
from unittest.mock import patch
import qualitative as q
from scene_checks import PerceptionUnknown

class EndpointRetention(unittest.TestCase):
 def fixture(self,stage='trial_lift',color='orange'):
  obs={'frame_id':'a'*32,'captured_at_ns':time.time_ns(),'measured14':[0.]*14,
       'state_binding':{'owner_generation':1},'calibration_sha256':'cal',
       'finger_geometry':{'geometry_identity':'tool'}}
  previous={'ok':True,'stage':stage,'held_block':color,'measured':obs,
   'facts':[{'payload_origin':{'color':color}},
            {'ok':True,'frame_id':obs['frame_id'],'wrist_match':{'ok':True}}]}
  return previous,copy.deepcopy(obs)
 def test_successful_trial_endpoint_reuses_positive_result_not_same_image_comparison(self):
  previous,obs=self.fixture()
  with patch('qualitative.current_image'),patch('qualitative.retention_evidence',side_effect=AssertionError('duplicate comparison')):
   r=q.retention_at_endpoint(None,previous,obs,'orange')
  self.assertTrue(r['ok']);self.assertTrue(r['reused_retention'])
 def test_all_loaded_boundary_stages_and_both_colors(self):
  for stage in ('trial_lift','transport','lower_to_support','green_stack_lower'):
   for color in ('orange','green'):
    p,o=self.fixture(stage,color)
    if stage=='green_stack_lower':p['facts'][-1]={'ok':True,'frame_id':o['frame_id'],'match':{}}
    with patch('qualitative.current_image'):
     self.assertTrue(q.retention_at_endpoint(None,p,o,color)['reused_retention'])
 def test_new_image_keeps_original_comparison(self):
  p,o=self.fixture();o['frame_id']='b'*32;o['captured_at_ns']+=1
  with patch('qualitative.retention_evidence',return_value={'ok':True}) as compare:
   q.retention_at_endpoint(None,p,o,'orange');compare.assert_called_once_with(None,p['measured'],o,'orange')
 def test_same_frame_cannot_invent_success_or_relabel_metadata(self):
  for change in ('failed','no_fact','negative','wrong_color','stage','time','pose','owner','calibration','geometry','old'):
   p,o=self.fixture()
   if change=='failed':p['ok']=False
   if change=='no_fact':p['facts']=[]
   if change=='negative':p['facts'][-1]['ok']=False
   if change=='wrong_color':p['held_block']='green'
   if change=='stage':p['stage']='close_grasp'
   if change=='time':o['captured_at_ns']+=1
   if change=='pose':o['measured14'][0]=.001
   if change=='owner':o['state_binding']['owner_generation']=2
   if change=='calibration':o['calibration_sha256']='new'
   if change=='geometry':o['finger_geometry']['geometry_identity']='new'
   if change=='old':p['measured']['captured_at_ns']=o['captured_at_ns']=time.time_ns()-2_000_000_000
   with self.subTest(change=change),patch('qualitative.current_image'),self.assertRaises(PerceptionUnknown):
    q.retention_at_endpoint(None,p,o,'orange')
 def test_missing_current_image_does_not_reuse(self):
  p,o=self.fixture()
  with patch('qualitative.current_image',side_effect=PerceptionUnknown('missing')),self.assertRaises(PerceptionUnknown):
   q.retention_at_endpoint(None,p,o,'orange')
 def test_actual_motion_comparison_still_rejects_same_frame(self):
  p,o=self.fixture()
  with self.assertRaises(PerceptionUnknown):q.retention_evidence(None,p['measured'],o,'orange')
