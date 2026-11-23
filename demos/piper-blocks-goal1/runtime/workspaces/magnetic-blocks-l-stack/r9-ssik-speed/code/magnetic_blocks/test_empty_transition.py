"""Grouped entry contracts; native geometry is separately replayed offline."""
import copy
import json
from pathlib import Path
import time
import unittest
from unittest.mock import patch
import numpy as np
import controller as c
import empty_transition as et
import scene_checks as sc
import test_composition as tc


def instrument(f):
    original=f.robot
    def robot():
        r=original();r.update(camera_calibration={'sha256':'a'*64},urdf_sha256='b'*64,
                             left_tool_calibration={'id':'offline-fixture'})
        r['bridge']['feedback']={'q':list(f.q),'generation':1,'timestamp_ns':time.time_ns()}
        mutate=getattr(f,'mutate_feedback',None)
        if mutate:mutate(r)
        return r
    f.robot=robot


class Entry(unittest.TestCase):
    def setUp(self):
        self.h=tc.Composition();self.h.setUp();self.addCleanup(self.h.doCleanups)
        self.f=self.h.f;instrument(self.f);self.f.q[6]=.069
        self.points=[[.1,0.,0.,0.,0.,0.],[.2,0.,0.,0.,0.,0.],[.3,0.,0.,0.,0.,0.]]
        def geo(ctx,start,end,scene,local,robot):
            ctx.value('piper_check_path',start14=start,target14=end+start[6:])
            return {'start14':list(start),'synthetic_geometry':True}
        self.g=patch('empty_transition.geometry',side_effect=geo).start();self.addCleanup(patch.stopall)
    def run_entry(self):return c.empty_joint_transition(self.f,joint_waypoints=self.points)
    def motions(self):return [(t,a) for t,a in self.f.calls if t=='piper_move_joints']
    def test_same_waypoints_one_initial_capture_and_only_final_receipt_image(self):
        r=self.run_entry();self.assertTrue(r['ok'],r['why'])
        self.assertEqual([a['joints'] for t,a in self.motions()],self.points)
        self.assertEqual([a['observe_after'] for t,a in self.motions()],[False,False,True])
        self.assertEqual(sum(t=='piper_observe' for t,a in self.f.calls),1)
        self.assertEqual(sum(t=='piper_check_path' for t,a in self.f.calls),3)
        self.assertFalse(any(t=='gemini_generate' for t,a in self.f.calls))
        self.assertTrue(r['measured'].get('frame_id'))
        self.assertEqual(r['empty_transition']['completed_segments'],3)
        # Downstream retained-scene/history accepts the real multi-receipt chain.
        self.assertIs(sc.completed_action_state({'result':r},r['measured'],self.f.robot()),r)
    def test_current_feedback_used_instead_of_commanded_waypoint(self):
        def drift(r):
            if self.f.q[0]==.1:r['bridge']['feedback']['q'][0]+=.001
        self.f.mutate_feedback=drift
        r=self.run_entry();self.assertTrue(r['ok'],r['why'])
        self.assertAlmostEqual(self.g.call_args_list[1].args[1][0],.101)
    def test_control_changes_stop_before_second_leg(self):
        changes={
          'drift':lambda r:r['bridge']['feedback']['q'].__setitem__(0,.2),
          'jaw':lambda r:r['bridge']['feedback']['q'].__setitem__(6,.06),
          'right':lambda r:r['bridge']['feedback']['q'].__setitem__(8,.02),
          'stale':lambda r:r['bridge']['feedback'].__setitem__('timestamp_ns',time.time_ns()-1_000_000_000),
          'owner':lambda r:r['bridge'].__setitem__('instance','other'),
          'calibration':lambda r:r['camera_calibration'].__setitem__('sha256','other'),
          'intervening':lambda r:r.__setitem__('last_motion',{'owner_request_id':'other'}),
          'busy':lambda r:r['bridge'].__setitem__('active_request','other')}
        for name,change in changes.items():
            with self.subTest(name=name):
                self.f.q=[0.]*14;self.f.q[6]=.069;self.f.q[13]=.002;self.f.calls=[];self.f.last=None
                self.f.mutate_feedback=lambda r:change(r) if self.f.q[0]==.1 else None
                r=self.run_entry();self.assertFalse(r['ok']);self.assertEqual(len(self.motions()),1)
                self.assertIn('hold',r)
    def test_path_refusal_after_first_does_not_dispatch_or_replay(self):
        from perception import PerceptionUnknown
        self.g.side_effect=[{'ok':True},PerceptionUnknown('new remaining path refused')]
        r=self.run_entry();self.assertFalse(r['ok']);self.assertEqual(len(self.motions()),1)
        self.assertEqual(r['empty_transition']['completed_segments'],1)
        with self.assertRaises(PerceptionUnknown):
            sc.completed_action_state({'result':r},self.f.obs(),self.f.robot())
    def test_first_path_refusal_dispatches_nothing(self):
        self.g.side_effect=RuntimeError('path refused')
        r=self.run_entry();self.assertFalse(r['ok']);self.assertEqual(len(self.motions()),0)
    def test_ambiguous_second_dispatch_holds_and_preserves_first(self):
        self.f.fail_motion=lambda t,a,f:t=='piper_move_joints' and a['joints']==self.points[1]
        r=self.run_entry();self.assertFalse(r['ok']);self.assertEqual(len(self.motions()),2)
        self.assertEqual(len(r['receipts']),1);self.assertIn('hold',r)
    def test_final_image_missing_does_not_repeat_completed_motions(self):
        self.f.omit_observation=lambda *a:True
        r=self.run_entry();self.assertFalse(r['ok']);self.assertTrue(r['needs_review'])
        self.assertEqual(len(self.motions()),3);self.assertTrue(r['action_completed'])
        self.assertNotIn('hold',r)
    def test_full_main_grouped_entry_keeps_scene_and_reaches_final_stage(self):
        # Other primitives run as in inherited composition tests; fake insertion
        # geometry is explicit, never evidence of collision qualification.
        def plan(ctx,robot,before,scene,color,points,q):
            return {'parts':[{'joints':ctx.encode(points[-1],q),'target':points[-1],'check':{}}],
                    'original_segments':len(points),'candidates':[]},np.zeros((16,3))
        with patch('continuous_approach.plan',plan),patch('continuous_approach.geometry',return_value={}):
            self.f.q[6]=.0004
            r=c.main(self.f,output_dir=Path(self.h.temp.name)/'grouped-main',group_empty_entry=True)
        self.assertTrue(r['ok'],r.get('why'));self.assertTrue(r['group_empty_entry'])
        rows=self.h.records(r);self.assertIn('orange-entry',rows)
        self.assertNotIn('orange-transition-1',rows)
        self.assertEqual(rows['orange-entry']['motion_dispatch_count'],3)
        self.assertEqual(rows['orange-entry']['tool_counts']['piper_observe'],1)
        self.assertEqual(r['candidate_version'],'v0.3.1-blend-batchfk')


if __name__=='__main__':unittest.main()
