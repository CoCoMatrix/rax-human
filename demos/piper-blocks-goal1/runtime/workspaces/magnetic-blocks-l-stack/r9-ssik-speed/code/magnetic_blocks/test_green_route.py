import copy,json,unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import composition as g
import green_route as route
from scene_checks import PerceptionUnknown
from test_adaptive_inputs import ik_refusal

class Route(unittest.TestCase):
 def case(self):
  robot={'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36],
         'bridge':{'instance':'same'}}
  obs={'frame_id':'old','measured14':[0.]*14,'state_binding':{'owner_generation':3}}
  fk={'position':[.3,-.15,.3],'quaternion_xyzw':[1.,0.,0.,0.]}
  scene={'blocks':{'green':{'top_center':{'surface_position_m':[.3,-.25,.078]}}}}
  return robot,obs,{'start':obs['measured14'],'fk':fk,'scene':scene,'tcp_offset_flange':[0.,0.,.14],'local':np.zeros((8,3)),'frame_id':'old'}
 def fact(self):return {'ok':True,'end_joints':[0.]*6,'min_block_allowance_m':.01}
 def test_numeric_search_never_reads_images_or_status(self):
  r,o,e=self.case();ctx=SimpleNamespace(value=lambda *a,**k:(_ for _ in ()).throw(AssertionError('No live call in numerical test')));report={};calls=[]
  def path(ctx,start,scene,local,args):
   calls.append(args)
   if args['quaternion']==e['fk']['quaternion_xyzw']:raise ik_refusal()
   return self.fact()
  with patch('green_route.planned_green_path',side_effect=path),patch('green_route.insertion_preview',return_value={}):
   args,fact=route.choose(ctx,e,r,report)
  self.assertNotEqual(args['quaternion'],e['fk']['quaternion_xyzw']);np.testing.assert_allclose(np.asarray(args['waypoints'][0][:2])+(route.rotation_matrix(args['quaternion'])@np.asarray(e['tcp_offset_flange']))[:2],[.3,-.25]);self.assertTrue(report['route_plan']['height_neighborhood'])
 def test_knife_edge_candidate_is_rejected_not_accepted(self):
  r,o,e=self.case();report={}
  with patch('green_route.planned_green_path',return_value=dict(self.fact(),min_block_allowance_m=.00007)),patch('green_route.insertion_preview') as onward:
   with self.assertRaisesRegex(ValueError,'No green approach'):route.choose(SimpleNamespace(),e,r,report)
  onward.assert_not_called();self.assertLessEqual(len(report['candidates']),117)
 def test_following_insertion_is_required(self):
  r,o,e=self.case();report={}
  with patch('green_route.planned_green_path',return_value=self.fact()),patch('green_route.insertion_preview',side_effect=PerceptionUnknown('Insertion blocked')):
   with self.assertRaisesRegex(ValueError,'No green approach'):route.choose(SimpleNamespace(),e,r,report)
  self.assertTrue(any(row.get('why')=='Insertion blocked' for row in report['candidates']))
 def test_unknown_network_error_is_not_retried(self):
  r,o,e=self.case();report={}
  with patch('green_route.planned_green_path',side_effect=RuntimeError('TLS lost')) as path:
   with self.assertRaisesRegex(RuntimeError,'TLS lost'):route.choose(SimpleNamespace(),e,r,report)
  self.assertEqual(path.call_count,1)
 def test_neighborhood_failure_rejects_candidate(self):
  r,o,e=self.case();report={};calls=[]
  def path(*args):
   calls.append(args[-1]);
   if len(calls)%2==0:raise ik_refusal()
   return self.fact()
  with patch('green_route.planned_green_path',side_effect=path),patch('green_route.insertion_preview') as onward:
   with self.assertRaises(ValueError):route.choose(SimpleNamespace(),e,r,report)
  onward.assert_not_called()
 def run_rebind(self,change=None):
  r,o,e=self.case();fresh=copy.deepcopy(o);fresh['frame_id']='new';r2=copy.deepcopy(r)
  if change:change(r2,fresh)
  prepared=[]
  def prepare(self,obs):prepared.append(obs['frame_id']);return dict(e,start=obs['measured14'],frame_id=obs['frame_id'])
  def choose(ctx,e,r,report):
   report['route_plan']={'selected':{'height_above_top_m':.195}}
   return {'waypoints':[[.3,-.25,.273]],'quaternion':[1,0,0,0]},self.fact()
  with patch('composition._current',return_value=(r,o,{},e['fk'])),patch('green_checks.GreenPregrasp.prepare',prepare),patch('green_route.choose',side_effect=choose),patch('controller._start',return_value=(r2,fresh)) as start,patch('green_checks.planned_green_path',return_value=self.fact()),patch('green_route.insertion_preview',return_value={}):
   result=g.green_hover_inputs(SimpleNamespace(),previous_report='prev',stationary_scene_report='original')
  return result,start,prepared
 def test_rebind_gets_real_new_observation_before_return(self):
  result,start,prepared=self.run_rebind();self.assertTrue(result['ok'],result['why']);self.assertEqual(prepared,['old','new'])
  self.assertTrue(start.call_args.kwargs['force_observe']);self.assertFalse(start.call_args.kwargs['require_depth']);self.assertEqual(result['measured']['frame_id'],'new');self.assertFalse(result['dispatch_attempted'])
 def test_owner_changed_during_search_stops(self):
  result,_,prepared=self.run_rebind(lambda r,o:r['bridge'].update(instance='different'));self.assertFalse(result['ok']);self.assertIn('Owner changed',result['why']);self.assertEqual(prepared,['old'])
 def test_robot_moved_during_search_stops(self):
  result,_,_=self.run_rebind(lambda r,o:o['measured14'].__setitem__(0,.02));self.assertFalse(result['ok']);self.assertIn('Robot moved',result['why'])
 def test_small_feedback_change_recomputes_path_from_actual_state(self):
  result,_,prepared=self.run_rebind(lambda r,o:o['measured14'].__setitem__(0,.001));self.assertTrue(result['ok']);self.assertEqual(prepared,['old','new'])

 def test_long_search_refreshes_instead_of_rechecking_old_frame(self):
  r,o,e=self.case();o['captured_at_ns']=1_000_000_000;now=[1_000_000_000];seen=[]
  def prepare(checker,obs):
   seen.append(obs['frame_id'])
   if (now[0]-obs['captured_at_ns'])/1e9>10:raise PerceptionUnknown('Stale observation')
   return dict(e,frame_id=obs['frame_id'])
  def choose(ctx,e,r,report):
   now[0]+=12_000_000_000;report['route_plan']={'selected':{'height_above_top_m':.195}}
   return {'waypoints':[[.3,-.25,.273]],'quaternion':[1,0,0,0]},self.fact()
  def fresh(*a,**kw):return r,dict(o,frame_id='new',captured_at_ns=now[0])
  with patch('composition._current',return_value=(r,o,{},e['fk'])),patch('green_checks.GreenPregrasp.prepare',prepare),patch('green_route.choose',side_effect=choose),patch('controller._start',side_effect=fresh),patch('green_checks.planned_green_path',return_value=self.fact()),patch('green_route.insertion_preview',return_value={}):
   result=g.green_hover_inputs(SimpleNamespace(),previous_report='prev',stationary_scene_report='source')
  self.assertTrue(result['ok'],result['why']);self.assertEqual(seen,['old','new'])
