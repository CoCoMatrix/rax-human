import time,unittest
from unittest.mock import patch
from types import SimpleNamespace
import assembly_plan as ap

class SearchLifecycle(unittest.TestCase):
 def planner(self):
  p=object.__new__(ap.Planner)
  p.started=time.monotonic()-100;p.search_active=False;p.ik_mode="full";p.ik_query_counts={"fast":0,"full":0};p.ik_reused_successes=0;p.searching={};p.progress_at=0
  p.fk_cache={};p.ik_cache={};p.failed_checks={};p.path_cache={};p.ctx=SimpleNamespace()
  return p
 def test_budget_is_only_for_search_not_later_execution(self):
  p=self.planner();p.progress(phase='executing')
  p.search_active=True
  with self.assertRaises(ap.PlanningTimeout):p.progress()
 def test_search_lifecycle_finishes_after_success_or_failure(self):
  p=self.planner()
  with patch.object(p,'_plan',return_value={'ok':True}):self.assertTrue(p.plan([])['ok'])
  self.assertFalse(p.search_active)
  with patch.object(p,'_plan',side_effect=ap.NoPlan('unreachable')):
   with self.assertRaises(ap.NoPlan):p.plan([])
  self.assertFalse(p.search_active)
 def test_pure_fk_cache_uses_exact_arm_joints_only(self):
  p=self.planner();calls=[]
  p.ctx.value=lambda name,**args:calls.append(args) or {'position':args['joints'][:3]}
  a=[0.]*14;p.fk(a);a[6]=.073;p.fk(a)
  self.assertEqual(len(calls),1)
  a[0]=1e-12;p.fk(a);self.assertEqual(len(calls),2)
 def test_deadline_is_not_caught_as_another_pose_candidate(self):
  self.assertFalse(issubclass(ap.PlanningTimeout,ap.NoPlan))
 def test_progress_has_computation_counts(self):
  p=self.planner();events=[];p.ctx._blocks_preflight_event=events.append;p.search_active=True;p.started=time.monotonic();p.progress(color='green',phase='hover')
  self.assertEqual(len(events),1);self.assertEqual(events[0]['color'],'green');self.assertIn('checked_paths',events[0])
if __name__=='__main__':unittest.main()
