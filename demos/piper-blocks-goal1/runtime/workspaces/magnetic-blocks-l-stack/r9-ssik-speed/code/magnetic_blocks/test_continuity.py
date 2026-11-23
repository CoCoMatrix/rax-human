"""New default main plumbing/failure contracts; geometry tested by native replay."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import controller as c
import continuous_approach as ca
import test_composition as tc
from perception import PerceptionUnknown

class Continuity(unittest.TestCase):
 def setUp(self):
  self.h=tc.Composition();self.h.setUp();self.addCleanup(self.h.doCleanups)
  def plan(ctx,robot,before,scene,color,points,q):
   return {'parts':[{'joints':ctx.encode(points[-1],q),'target':points[-1],'check':{'synthetic_geometry':True}}],
           'original_segments':len(points),'candidates':[{'segments':1,'ok':True}]},np.zeros((16,3))
  self.p=patch('continuous_approach.plan',plan);self.p.start();self.addCleanup(self.p.stop)
  self.g=patch('continuous_approach.geometry',return_value={'synthetic_geometry':True});self.g.start();self.addCleanup(self.g.stop)
 def run_main(self):return c.main(self.h.f,output_dir=Path(self.h.temp.name)/'new-main',group_empty_entry=False)
 def test_default_full_main_uses_single_strokes_and_descents(self):
  r=self.run_main();self.assertTrue(r['ok'],r.get('why'));self.assertEqual(r['auxiliary_visual_calls'],6)  # inherited fake visual checker deliberately calls six times
  rows=self.h.records(r)
  for name in ('open-empty','orange-close','green-close','orange-release','green-stack-release'):
   self.assertEqual(rows[name]['tool_counts'].get('piper_gripper_stroke'),1)
   self.assertEqual(rows[name]['motion_dispatch_count'],1)
  for name in ('orange-approach','green-approach'):
   self.assertEqual(rows[name]['tool_counts'].get('piper_move_joints'),1)
   self.assertEqual(rows[name]['motion_dispatch_count'],1)
  self.assertTrue(r['trajectory_optimization']);self.assertEqual(r['candidate_version'],'v0.3.1-blend-batchfk')
 def test_rejected_grasp_precondition_never_closes(self):
  with patch('scene_checks.supported_entry',return_value={'ok':False,'frame_id':'rejected','why':'injected misalignment'}):r=self.run_main()
  self.assertFalse(r['ok']);self.assertEqual(r['failed_step'],'orange-close')
  self.assertFalse(any(t=='piper_gripper_stroke' and a['direction']=='close' for t,a in self.h.f.calls))
 def test_ambiguous_whole_close_stops_without_trial_or_replay(self):
  self.h.f.fail_motion=lambda t,a,f:t=='piper_gripper_stroke' and a['direction']=='close'
  r=self.run_main();self.assertFalse(r['ok']);self.assertEqual(r['failed_step'],'orange-close')
  self.assertEqual(sum(t=='piper_gripper_stroke' and a['direction']=='close' for t,a in self.h.f.calls),1)
  self.assertNotIn('orange-trial',[s['label'] for s in r['steps']])
 def test_completed_stroke_missing_observation_not_repeated(self):
  self.h.f.omit_observation=lambda t,a,f:t=='piper_gripper_stroke'
  r=self.run_main();self.assertFalse(r['ok']);self.assertEqual(r['failed_step'],'open-empty')
  self.assertEqual(sum(t=='piper_gripper_stroke' for t,a in self.h.f.calls),1)
 def test_no_insertion_plan_never_dispatches_descent_or_close(self):
  with patch('continuous_approach.plan',side_effect=PerceptionUnknown('no current path')):r=self.run_main()
  self.assertFalse(r['ok']);self.assertEqual(r['failed_step'],'orange-approach')
  self.assertEqual(self.h.records(r)['orange-approach']['motion_dispatch_count'],0)
  self.assertFalse(any(t=='piper_gripper_stroke' and a['direction']=='close' for t,a in self.h.f.calls))

class NumericChecks(unittest.TestCase):
 def test_midpath_excursion_rejected_even_if_endpoints_align(self):
  class Ctx:
   def value(self,name,**args):
    if name=='piper_check_path':return {'cad_clear':True,'workspace_clear':True}
    u=args['joints'][0];T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=[.3+.02*np.sin(np.pi*u),-.1,.28-.08*u];return {'T_base_flange':T.tolist()}
  scene={'table':{'z_ax_by_c':[0.,0.,0.]},'blocks':{'orange':{'top_center':{'surface_position_m':[.3,-.1,.06]}}}}
  with self.assertRaisesRegex(PerceptionUnknown,'12mm'):
   ca.geometry(Ctx(),[0.]*14,[1.,0,0,0,0,0],scene,'orange',np.zeros((16,3)),robot={'workspace_min':[.05,-.4,.17],'workspace_max':[.5,.05,.36]})
 def test_bad_path_does_not_request_fresh_image_or_model(self):
  from test_composition import SceneContext
  f=SceneContext();calls=[]
  with patch('scene_checks.installed_finger_corners',return_value=np.zeros((16,3))),patch('continuous_approach.geometry',side_effect=PerceptionUnknown('collision')):
   with self.assertRaises(PerceptionUnknown):ca.plan(f,f.robot(),f.obs(),f.scene,'orange',[[.3,-.1,.25],[.3,-.1,.23]],[1.,0,0,0])
  self.assertTrue(all(t in ('piper_fk','piper_solve_ik') for t,a in f.calls))
