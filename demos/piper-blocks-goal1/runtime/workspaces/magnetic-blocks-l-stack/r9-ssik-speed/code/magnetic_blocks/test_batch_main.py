"""Main/history/counting plumbing. Curve proof is separate native replay."""
import copy
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import controller as c
import test_composition as tc
from test_empty_transition import instrument

class BatchMain(unittest.TestCase):
 def setUp(self):
  self.h=tc.Composition();self.h.setUp();self.addCleanup(self.h.doCleanups);self.f=self.h.f;instrument(self.f)
  original=self.f.value
  def value(name,**kw):
   if name=='piper_move_waypoints':
    if getattr(self.f,'fail_batch',False):raise RuntimeError('injected uncertain batch')
    return original('piper_move_joints',joints=kw['waypoints'][-1],observe_after=kw['observe_after'])
   return original(name,**kw)
  self.f.value=value
  def batch(ctx,points,report):
   robot,obs=c._start(ctx,report=report,require_depth=True)
   report['planned_motion_count']=1
   report['facts']=[{'ok':True,'frame_id':obs['frame_id'],'scene':copy.deepcopy(self.f.scene),'why':'explicit synthetic batch scene'}]
   return c._move(ctx,'piper_move_waypoints',{'start14':obs['measured14'],'waypoints':points,'obstacles':[{'synthetic':True}],'blend_indices':[0,1]},report)
  def plan(ctx,robot,before,scene,color,points,q):
   return {'parts':[{'joints':ctx.encode(points[-1],q),'target':points[-1],'check':{}}],'original_segments':len(points),'candidates':[]},np.zeros((16,3))
  for target,fn in [('batched_entry.execute',batch),('continuous_approach.plan',plan),('continuous_approach.geometry',lambda *a,**k:{})]:
   p=patch(target,fn);p.start();self.addCleanup(p.stop)
 def test_original_default_main_stays_separate(self):
  r=c.main(self.f,output_dir=Path(self.h.temp.name)/'default');self.assertTrue(r['ok'],r.get('why'))
  labels=[x['label'] for x in r['steps']]
  self.assertFalse(r['group_empty_entry']);self.assertFalse(r['blend_empty_entry'])
  for i in range(1,4):self.assertIn('orange-transition-'+str(i),labels)
 def test_independent_blended_main_complete_receipt_history(self):
  r=c.main_blended(self.f,output_dir=Path(self.h.temp.name)/'blend');self.assertTrue(r['ok'],r.get('why'))
  rows=self.h.records(r);batch=rows['orange-entry-blended'];self.assertEqual(batch['motion_dispatch_count'],1)
  self.assertEqual(batch['tool_counts']['piper_move_waypoints'],1)
  self.assertIn('orange-close',rows);self.assertIn('orange-trial',rows);self.assertIn('green-stack-release',rows)
 def test_ambiguous_batch_never_falls_back_or_starts_grasp(self):
  self.f.fail_batch=True
  r=c.main_blended(self.f,output_dir=Path(self.h.temp.name)/'fail');self.assertFalse(r['ok']);self.assertEqual(r['failed_step'],'orange-entry-blended')
  labels=[x['label'] for x in r['steps']];self.assertNotIn('orange-approach',labels);self.assertNotIn('orange-transition-1',labels)
