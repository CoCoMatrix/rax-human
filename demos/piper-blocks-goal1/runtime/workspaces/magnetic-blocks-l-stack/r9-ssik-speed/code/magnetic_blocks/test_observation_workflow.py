import copy,json,time,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import observations,ordinary_checks,qualitative,timing,controller,rehearse_step,scene_checks

class Observations(unittest.TestCase):
 def setUp(self):
  now=time.time_ns();self.obs={'frame_id':'f','captured_at_ns':now,'measured14':[0.]*14,
   'cameras':{c:{'metric_depth':False} for c in ('head','left','right')}}
  self.robot={'urdf_sha256':'u','camera_calibration':{'sha256':'c'},'owner_status':{'generation':1},
   'bridge':{'instance':'b','feedback':{'timestamp_ns':now,'generation':1,'q':[0.]*14}},'last_motion':{'owner_request_id':'req'}}
  class Context:
   def __init__(s,obs):s.calls=[];s.obs=obs
   def value(s,t,**kw):s.calls.append(t);return s.obs
  self.ctx=Context(self.obs);observations.remember(self.ctx,self.obs,self.robot)
  p=patch('observations.current_image');p.start();self.addCleanup(p.stop)
 def test_receipt_image_reused_without_relabeling_time(self):
  x,source=observations.observe(self.ctx,self.robot)
  self.assertEqual(source,'recent_receipt_or_observation');self.assertEqual(x,self.obs);self.assertEqual(self.ctx.calls,[])
 def test_force_depth_old_feedback_motion_owner_and_geometry_require_new_capture(self):
  for kind in ('force','depth','old','feedback','motion','owner','geometry','pose'):
   r=copy.deepcopy(self.robot);o=copy.deepcopy(self.obs);kw={}
   if kind=='force':kw['force']=True
   if kind=='depth':kw['require_depth']=True
   if kind=='old':o['captured_at_ns']-=2_000_000_000
   observations.remember(self.ctx,o,self.robot)
   if kind=='feedback':r['bridge']['feedback']['timestamp_ns']-=1_000_000_000
   if kind=='motion':r['last_motion']['owner_request_id']='new'
   if kind=='owner':r['bridge']['instance']='new'
   if kind=='geometry':r['left_tool_calibration']={'new':'geometry'}
   if kind=='pose':r['bridge']['feedback']['q'][0]=.1
   with self.subTest(kind=kind):self.assertEqual(observations.observe(self.ctx,r,**kw)[1],'fresh_capture')

class Ordinary(unittest.TestCase):
 def fixture(self):
  obs={'frame_id':'f','measured14':[0.]*14,'cameras':{'head':{'K':[[500,0,640],[0,500,360],[0,0,1]],
   'T_base_camera':[[1,0,0,0],[0,1,0,0],[0,0,1,-1],[0,0,0,1]]}}}
  scene={'blocks':{c:{'top_center':{'surface_position_m':p}} for c,p in [('red',[.3,0,.08]),('orange',[.3,-.067,.08]),('green',[.3,-.2,.08])]},'table':{'z_ax_by_c':[0,0,.02]}}
  origin={'color':'green','grasp_flange':[.3,-.2,.205],'flange_to_bottom_m':.185}
  class Context:
   def value(s,t,**kw):
    if t!='piper_fk':raise AssertionError('No model/depth/motion allowed')
    return {'position':[.3,0,.265]}
  return Context(),obs,scene,origin
 def test_partial_head_surface_with_checked_height_passes_without_model(self):
  ctx,obs,scene,origin=self.fixture();im=np.zeros((720,1280,3),np.uint8);im[350:375,760:785]=[0,180,0]
  with patch('ordinary_checks.current_image',return_value=im),patch('history_scene.reject_visible_displacement',return_value={}):
   r=ordinary_checks.placement(ctx,'supported',obs,scene,origin)
  self.assertTrue(r['ok']);self.assertEqual(r['method'],'ordinary_rgb_measured_placement');json.dumps(r,allow_nan=False)
 def test_wrist_can_resolve_head_occlusion_but_blank_views_do_not_pass(self):
  ctx,obs,scene,origin=self.fixture();head=np.zeros((720,1280,3),np.uint8);wrist=head.copy();wrist[300:350,500:600]=[0,180,0]
  with patch('ordinary_checks.current_image',side_effect=lambda o,c:head if c=='head' else wrist),patch('history_scene.reject_visible_displacement',return_value={}):
   self.assertTrue(ordinary_checks.placement(ctx,'supported',obs,scene,origin)['ok'])
   wrist[:]=0;self.assertIsNone(ordinary_checks.placement(ctx,'supported',obs,scene,origin)['ok'])
 def test_large_support_height_contradiction_is_not_ignored(self):
  ctx,obs,scene,origin=self.fixture();origin['flange_to_bottom_m']=.1
  self.assertFalse(ordinary_checks.placement(ctx,'supported',obs,scene,origin)['ok'])
 def test_auxiliary_model_is_opt_in_and_unknown_is_explicit(self):
  class Context:
   def value(s,*a,**kw):raise AssertionError('Unexpected model call')
  r=qualitative.assess_images(Context(),'support',None,{'frame_id':'f'},'question')
  self.assertIsNone(r['ok']);self.assertFalse(r['model_called'])

class Summaries(unittest.TestCase):
 def test_full_evidence_preserved_and_summary_is_small(self):
  from test_workflow import Fake
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'result.json'
   r=rehearse_step.run_one(Fake(),'large',{},p,operation=lambda ctx:{'ok':True,'facts':[{'ok':True,'why':'broad relation','raw':'x'*450000}],'why':'done'})
   full=json.loads(p.read_text());summary=json.loads(p.with_suffix('.summary.json').read_text())
   self.assertEqual(len(full['result']['facts'][0]['raw']),450000)
   self.assertLess(p.with_suffix('.summary.json').stat().st_size,8000)
   self.assertEqual(summary['report'],str(p));self.assertIn('timing',r)
 def test_timing_does_not_invent_missing_motor_time(self):
  r=timing.motion_timing({'elapsed_s':5,'result':{'settling':{'window_s':.3}}})
  self.assertIsNone(r['sdk_receipt_stream_s']);self.assertEqual(r['settling_window_s'],.3)
 def test_timing_uses_same_clock_and_documents_overlap(self):
  r=timing.motion_timing({'result':{'execution_timing':{'first_receipt_seen_monotonic_ns':1_000_000_000,'last_receipt_seen_monotonic_ns':3_000_000_000,'settled_monotonic_ns':3_400_000_000}}})
  self.assertEqual(r['sdk_receipt_stream_s'],2);self.assertAlmostEqual(r['last_receipt_to_settled_s'],.4)

class GreenRecovery(unittest.TestCase):
 def test_completed_lowering_recovers_readonly_and_next_release_accepts_report(self):
  from test_composition import Composition
  h=Composition();h.setUp();self.addCleanup(h.doCleanups)
  h.f.negative_phase='stack_supported';r=h.run_main();self.assertEqual(r['failed_step'],'green-stack-lower')
  source=r['steps'][-1]['report'];original=Path(source).read_bytes();before=len(h.f.calls)
  recovered=Path(h.temp.name)/'recovered.json'
  with patch('recovery.scene_geometry'),patch('recovery.placement',side_effect=lambda ctx,phase,obs,*a,**kw:{'ok':True,'frame_id':obs['frame_id'],'why':'new endpoint evidence','supported':True}):
   rr=rehearse_step.run_one(h.f,'verify_completed_action',{'previous_report':source},recovered)
  self.assertTrue(rr['result']['ok'],rr['result'].get('why'))
  self.assertFalse(any(t in rehearse_step.MOTIONS for t,a in h.f.calls[before:]));self.assertEqual(Path(source).read_bytes(),original)
  resolved=scene_checks.read_report(recovered);self.assertEqual(resolved['result']['stage'],'green_stack_lower')
  support=resolved['inputs']['support_report'];out=Path(h.temp.name)/'release.json'
  with patch('ordinary_checks.placement',side_effect=lambda ctx,phase,obs,*a,**kw:{'ok':True,'frame_id':obs['frame_id'],'why':'new ordinary release evidence','supported':True}):
   released=rehearse_step.run_one(h.f,'green_stack_release',{'previous_report':str(recovered),'support_report':support},out)
  self.assertTrue(released['result']['ok'],released['result'].get('why'))
  Path(source).write_bytes(original+b'\n')
  with self.assertRaises(scene_checks.PerceptionUnknown):scene_checks.read_report(recovered)
 def test_completed_release_and_retreat_missing_images_recover_without_repeating_motion(self):
  from test_composition import Composition
  import composition
  h=Composition();h.setUp();self.addCleanup(h.doCleanups)
  h.f.negative_phase='green_supported_before_release';r=h.run_main();rows=h.records(r)
  lower=rows['green-stack-lower']['report'];support=rows['green-stack-lower']['inputs']['support_report']
  h.f.negative_phase=None;h.f.omit_observation=lambda tool,args,ctx:tool=='piper_gripper' and args['width_m']>.068
  def yes(ctx,phase,obs,*a,**kw):return {'ok':True,'frame_id':obs['frame_id'],'why':'synthetic fresh endpoint fact','supported':True}
  release=Path(h.temp.name)/'release-missing-image.json'
  with patch('ordinary_checks.placement',side_effect=yes):
   x=rehearse_step.run_one(h.f,'green_stack_release',{'previous_report':lower,'support_report':support},release)
  self.assertFalse(x['result']['ok']);self.assertTrue(x['result']['action_completed'])
  h.f.omit_observation=None;before=len(h.f.calls);verified=Path(h.temp.name)/'verified-release.json'
  with patch('recovery.scene_geometry'),patch('recovery.placement',side_effect=yes):
   x=rehearse_step.run_one(h.f,'verify_completed_action',{'previous_report':str(release)},verified)
  self.assertTrue(x['result']['ok'],x['result'].get('why'))
  self.assertFalse(any(t in rehearse_step.MOTIONS for t,a in h.f.calls[before:]))
  inputs=composition.pose_inputs(h.f,previous_report=str(verified),mode='green_retreat',color='green')['inputs']
  h.f.omit_observation=lambda tool,args,ctx:tool=='piper_move_pose'
  retreat=Path(h.temp.name)/'retreat-missing-image.json'
  x=rehearse_step.run_one(h.f,'green_stack_retreat',inputs,retreat)
  self.assertFalse(x['result']['ok']);self.assertTrue(x['result']['action_completed'])
  h.f.omit_observation=None;before=len(h.f.calls)
  with patch('recovery.after_release',side_effect=yes):
   x=rehearse_step.run_one(h.f,'verify_completed_action',{'previous_report':str(retreat)},Path(h.temp.name)/'verified-retreat.json')
  self.assertTrue(x['result']['ok'],x['result'].get('why'))
  self.assertFalse(any(t in rehearse_step.MOTIONS for t,a in h.f.calls[before:]))
 def test_unconfirmed_green_action_does_not_get_replayed_or_promoted(self):
  from test_workflow import Fake
  f=Fake()
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'ambiguous.json';p.write_text(json.dumps({'result':{'stage':'green_stack_lower','executed':True,'action_completed':False,'receipts':[]}}))
   r=controller.verify_completed_action(f,previous_report=str(p))
  self.assertFalse(r['ok']);self.assertFalse(any(t in rehearse_step.MOTIONS for t,a in f.calls))
