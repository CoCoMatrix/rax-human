import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
import assembly_plan as a


class ReleaseFeedback(unittest.TestCase):
 def test_small_positive_gap_allowed_overlap_not_allowed(self):
  self.assertGreaterEqual(.000499726474909673,a.finger_gap_minimum(.003))
  self.assertLess(-.000001,a.finger_gap_minimum(.003))
  self.assertEqual(a.finger_gap_minimum(.020),a.clearance_minimum(.020))
  self.assertEqual(a.FINGER_GAP_COMPARISON_ERROR_M,.001)

 def test_full_paths_cover_both_jaw_bounds_for_each_retreat_kind(self):
  for withdraw in (False,True):
   with self.subTest(withdraw=withdraw):
    calls=[];start=[0.]*14;start[6]=a.CLOSED
    scene={'blocks':{'green':{'top_center':{'surface_position_m':[.2,-.1,.12]}}}}
    def search(phase,fn):
     yield {'phase':phase}
     return fn()
    def check(x,y,scene,**kw):
     calls.append((list(x),list(y),kw))
     return {'native':{'cad_clear':True,'workspace_clear':True}}
    def route(opened,dest,q,scene,**kw):
     if kw['withdraw']!=withdraw:raise a.NoPlan('test candidate selection')
     target=list(opened);target[0]+=.1
     return [{'start14':list(opened),'target14':target,'check':check(opened,target,scene,**kw)}]
    fake=SimpleNamespace(search_step=search,check=check,route=route,connect_edge=lambda *args,**kw:(route(*args,**kw),None),fk=lambda q:{'position':[.2,-.1,.2]})
    gen=a.Planner.finish_steps(fake,start,scene,'green',[.2,-.1,.12],{},[0,0,0,1],'red',None)
    while True:
     try:next(gen)
     except StopIteration as done:result=done.value;break
    feedback=result['actions'][-1]['release_feedback_preflight']
    np.testing.assert_allclose(feedback['gap_range_m'],[.072,.074])
    self.assertEqual(len(feedback['checks']),2)
    for gap in (.072,.074):
     self.assertTrue(any(abs(x[6]-gap)<1e-12 and abs(y[6]-gap)<1e-12 and x[0]!=y[0] for x,y,k in calls))

 def test_feedback_native_refusal_rejects_whole_candidate(self):
  for clear in (True,False):
   with self.subTest(clear=clear):
    request={'key':'feedback','arguments':{'start14':[0.]*14}}
    feedback={'native':{'deferred':True},'_pending_native':request}
    moves=[{'actions':[{'check':{'native':{'cad_clear':True,'workspace_clear':True}},'release_feedback_preflight':{'checks':[feedback]}}]}]
    calls=[]
    def value(name,**kw):
     calls.append(name);return {'cad_clear':clear,'workspace_clear':True}
    fake=SimpleNamespace(lazy_stats={'native_candidates':0,'native_rejections':0,'native_certified_actions':0},native_cache={},progress=lambda **kw:None,ctx=SimpleNamespace(value=value))
    if clear:
     a.Planner.certify_moves(fake,moves)
     self.assertNotIn('_pending_native',feedback)
     self.assertEqual(fake.lazy_stats['native_certified_feedback_paths'],1)
    else:
     with self.assertRaises(a.NativePathRefusal):a.Planner.certify_moves(fake,moves)
    self.assertEqual(calls,['piper_check_path'])


class NativeComparison(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  import fcl
  cls.fcl=fcl
  path=Path(__file__).resolve().parent.parent/'installed_collision_math.py'
  spec=importlib.util.spec_from_file_location('retreat_candidate_math',path)
  cls.math=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.math)

 def check(self,gap,extra=.001,link='left_gripper_link1',declared=.0025):
  corners=np.array([[x,y,z,1.] for x in (-.01,.01) for y in (-.01,.01) for z in (-.01,.01)])
  obj=self.fcl.CollisionObject(self.fcl.Box(.02,.02,.02),self.fcl.Transform(np.zeros(3)))
  fake=SimpleNamespace(_state=lambda q:q,model=SimpleNamespace(update_cfg=lambda c:None),scene=SimpleNamespace(graph={'finger':(np.eye(4),None)}),objects={'finger':obj},local_bounds={'finger':corners},pairs=[],links={'finger':link})
  obstacle={'name':'green','center_m':[.02+gap,0.,0.],'size_m':[.02,.02,.02],'finger_clearance_tolerance_m':declared,'finger_clearance_comparison_error_m':extra}
  return self.math.sampled_check(fake,[0.]*14,obstacles=[obstacle])

 def test_new_allowance_and_old_default(self):
  self.assertTrue(self.check(.000497)['cad_clear'])
  self.assertFalse(self.check(.000497,extra=0.)['cad_clear'])

 def test_overlap_touch_and_nonfinger_remain_rejected(self):
  for gap,link in ((-.0001,'left_gripper_link1'),(0.,'left_gripper_link1'),(.000497,'right_gripper_link1'),(.000497,'left_link6')):
   with self.subTest(gap=gap,link=link):self.assertFalse(self.check(gap,link=link)['cad_clear'])
  self.assertFalse(self.check(.000497,declared=0.)['cad_clear'])

 def test_invalid_error_is_not_silently_accepted(self):
  for error in (-.001,.00101,float('nan'),True):
   with self.subTest(error=error),self.assertRaises(ValueError):self.check(.000497,extra=error)


if __name__=='__main__':unittest.main()
