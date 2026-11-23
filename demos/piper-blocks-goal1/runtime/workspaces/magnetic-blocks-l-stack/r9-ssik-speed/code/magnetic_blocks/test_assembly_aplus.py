"""A+ sequencing/failure tests. Synthetic feedback is NOT physics qualification.
The six actual-position cases separately use installed FK/IK/FCL, no mocks.
"""
import copy,itertools,json,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
import assembly_plan as ap
import assembly_controller as ac
import test_composition as tc
from test_empty_transition import instrument

class Goals(unittest.TestCase):
 def test_six_initial_permutations_two_base_orders_and_translation(self):
  original=tc.SceneContext().scene;pos=[ap.xyz(original,c).tolist() for c in ('red','orange','green')]
  for colors in itertools.permutations(('red','orange','green')):
   s=copy.deepcopy(original)
   for c,p in zip(colors,pos):s['blocks'][c]['top_center']['surface_position_m']=p
   s=ap.initial_scene(s)
   for sign in (-1,1):
    goal=ap.goal_scene(s,[.31,-.09],sign);ap.assert_goal(goal)
    self.assertAlmostEqual(ap.xyz(goal,'orange')[1]-ap.xyz(goal,'red')[1],sign*ap.SPACING)
    self.assertTrue(np.allclose(ap.xyz(goal,'red')[:2],ap.xyz(goal,'green')[:2]))
    self.assertAlmostEqual(ap.xyz(goal,'green')[2]-ap.xyz(goal,'red')[2],ap.height(s,'green'))
   self.assertEqual(s['blocks'][colors[0]]['top_center']['surface_position_m'],pos[0])
 def test_wrong_stack_or_rotated_axis_rejected(self):
  s=ap.goal_scene(ap.initial_scene(tc.SceneContext().scene),[.3,-.1]);s['blocks']['green']['top_center']['surface_position_m'][0]+=.07
  with self.assertRaises(ap.NoPlan):ap.assert_goal(s)
 def test_environment_contains_table_and_nonactive_blocks(self):
  boxes=ap.boxes(ap.initial_scene(tc.SceneContext().scene),('orange',))
  self.assertEqual({x['name'] for x in boxes},{'measured_task_table','red','green'})

class Execution(unittest.TestCase):
 def setUp(self):
  self.h=tc.Composition();self.h.setUp();self.addCleanup(self.h.doCleanups);self.f=self.h.f;instrument(self.f)
  self.patches=[]
  def p(name,*a,**k):
   obj=patch(name,*a,**k);self.patches.append(obj);return obj.start()
  self.addCleanup(lambda:[x.stop() for x in reversed(self.patches)])
  self.patch=p
  raw_obs=self.f.obs
  def obs():
   value=raw_obs();value['captured_at_ns']=time.time_ns();return value
  self.f.obs=obs
  p('assembly_controller.scene_geometry',lambda *a:None)
  self.planner=SimpleNamespace(check=lambda *a,**k:{'offline_test_double':True})
  self.plan={'source_scene':ap.initial_scene(self.f.scene),'source_digest':'synthetic'}
  self.execution=ac.Execution(self.f,self.planner,self.plan,self.f.robot(),self.f.obs())
 def action(self):
  a=self.f.q.copy();b=a.copy();b[0]=.1
  return dict(kind='arm',stage='orange-hover',color='orange',start14=a,target14=b,payload=None,contact=False)
 def test_no_image_between_normal_free_legs(self):
  r=self.execution.run(self.action());self.assertTrue(r['ok'],r['why'])
  self.assertFalse(any(t in ('piper_observe','gemini_generate') for t,a in self.f.calls))
  self.assertFalse(next(a for t,a in self.f.calls if t=='piper_move_joints')['observe_after'])
 def test_stale_owner_or_changed_state_refuses_before_motion(self):
  changes=[lambda r:r['bridge']['feedback'].__setitem__('timestamp_ns',0),lambda r:r['bridge'].__setitem__('instance','changed'),lambda r:r['bridge']['feedback']['q'].__setitem__(8,.02)]
  for change in changes:
   self.f.calls=[];self.f.mutate_feedback=change
   r=self.execution.run(self.action());self.assertFalse(r['ok']);self.assertFalse(any(t.startswith('piper_move') for t,a in self.f.calls))
 def test_ambiguous_dispatch_holds_and_never_repeats(self):
  self.f.fail_motion=lambda *a:True
  r=self.execution.run(self.action());self.assertFalse(r['ok']);self.assertEqual(sum(t=='piper_move_joints' for t,a in self.f.calls),1)
  self.assertEqual(sum(t=='piper_hold' for t,a in self.f.calls),1)
 def test_remaining_path_failure_does_not_move(self):
  def refuse(*a,**k):raise ap.NoPlan('changed remaining path')
  self.planner.check=refuse
  r=self.execution.run(self.action());self.assertFalse(r['ok']);self.assertFalse(any(t=='piper_move_joints' for t,a in self.f.calls))
 def test_release_requires_previous_support_result(self):
  a=self.action();a.update(kind='jaw',stage='orange-release',width_m=ap.OPEN);self.f.q[6]=.05;self.execution.expected=self.f.q.copy();a['start14']=self.f.q.copy()
  r=self.execution.run(a);self.assertFalse(r['ok']);self.assertFalse(any(t=='piper_gripper_stroke' for t,a in self.f.calls))
 def test_complete_plan_required_before_opening_or_grasping(self):
  class Refusal:
   def __init__(self,*a):pass
   def plan(self,*a):raise ap.NoPlan('future placement impossible')
  self.patch('assembly_controller.Planner',Refusal);self.patch('assembly_controller.finger_models',return_value=[[],[]])
  result=ac.main(self.f,output_dir=Path(self.h.temp.name)/'aplus-refusal')
  self.assertFalse(result['ok']);self.assertEqual(result['failed_step'],'whole-task-preflight')
  self.assertFalse(any(t in tc.rehearse_step.MOTIONS for t,a in self.f.calls));self.assertEqual(result['primary_model_calls'],0)
  with self.assertRaises(RuntimeError):ac.main(self.f,output_dir=Path(self.h.temp.name)/'replay')
 def test_full_plan_runner_and_records_with_explicit_mock_geometry(self):
  scene=ap.initial_scene(self.f.scene);a=self.f.q.copy();a[6]=ap.OPEN;b=a.copy();b[0]=.1
  plan={'source_scene':scene,'source_digest':ap.digest(scene),'goal_anchor':[.3,-.1],'order':['red','orange','green'],'moves':[{'actions':[dict(kind='arm',stage='orange-hover',color='orange',start14=a,target14=b,payload=None,contact=False)]}]}
  class MockPlanner:
   def __init__(self,*args):pass
   def plan(self,*args):return plan
   def check(self,*args,**kw):return {'synthetic':True}
  self.patch('assembly_controller.Planner',MockPlanner);self.patch('assembly_controller.finger_models',return_value=[[],[]])
  r=ac.main(self.f,output_dir=Path(self.h.temp.name)/'synthetic-chain')
  self.assertTrue(r['ok'],r.get('why'));self.assertEqual(r['motion_dispatch_count'],2)
  self.assertEqual(r['auxiliary_visual_calls'],0);self.assertTrue((Path(r['report']).parent/'WHOLE_TASK_PLAN.json').exists())
  # This tests sequencing/reporting only; these mocked actions are not a claim
  # that a partial task constitutes physical success.

if __name__=='__main__':unittest.main()
