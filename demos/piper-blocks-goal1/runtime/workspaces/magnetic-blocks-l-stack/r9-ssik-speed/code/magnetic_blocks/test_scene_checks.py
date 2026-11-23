"""Offline recorded-frame checks only; no runtime Context construction."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import scene_checks as s

class SceneChecks(unittest.TestCase):
    def setUp(self):
        self.o=json.loads(Path('runtime/frames/2da08a44d99742be9d86614763e37449/meta.json').read_text())
    def test_recorded_table_and_all_blocks(self):
        with patch('scene_checks.time.time_ns',return_value=self.o['captured_at_ns']):
            blocks,xyz,valid,rgb=s.current_arrays(self.o)
            table=s.tabletop(self.o,xyz,valid,rgb)
        self.assertEqual(set(blocks),{'red','orange','green'})
        self.assertGreater(table['points'],5000)
        self.assertLess(table['residual_p95_m'],.006)
        self.assertTrue(-.02<table['z_ax_by_c'][2]<.06)
    def test_stale_and_unbound_fail(self):
        with self.assertRaises(s.PerceptionUnknown):s.current_arrays(self.o)
        bad=copy.deepcopy(self.o);bad['state_binding']['owner_unchanged']=False
        with patch('scene_checks.time.time_ns',return_value=bad['captured_at_ns']):
            with self.assertRaises(s.PerceptionUnknown):s.current_arrays(bad)
    def test_opening_rejects_loaded_or_nonstartup_without_tools(self):
        for i,v in [(6,.05),(2,.1)]:
            bad=copy.deepcopy(self.o);bad['measured14'][i]=v
            r=s.empty_opening(None,bad,.069)
            self.assertIsNone(r['ok'])
    def test_missing_depth_plane_refuses(self):
        with self.assertRaises(s.PerceptionUnknown):
            s.tabletop(self.o,np.zeros((720,1280,3)),np.zeros((720,1280),bool),np.zeros((720,1280,3),np.uint8))
    def test_recorded_partial_red_and_border_green(self):
        o=json.loads(Path('references/rehearsal-r4/r4-hover-02.json').read_text())['result']['review']['after']
        ref=json.loads(Path('references/rehearsal-r4/r4-transition-02.json').read_text())['result']['facts'][0]['scene']
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']):
            selected,_,_,_=s.current_arrays(o,ref)
        self.assertTrue(selected['red']['partial_visible'])
        self.assertFalse(selected['green'].get('partial_visible',False))
        self.assertGreater(selected['red']['visible_color_pixels'],500)
    def test_full_xyz_retention_and_invalidations(self):
        paths=['references/rehearsal-r4/r4-transition-01.json','references/rehearsal-r4/r4-transition-02.json',
               'references/rehearsal-r4/r4-hover-05.json','references/rehearsal-r4/r4-approach-hover-02.json','references/rehearsal-r4/r4-approach-mid-01.json']
        o=json.loads(Path(paths[-1]).read_text())['result']['measured']
        source=json.loads(Path(paths[0]).read_text())['result']['facts'][0]['scene']
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']):
            retained=s.retain_full_scene(paths[0],paths,o)
            self.assertEqual(retained['blocks'],source['blocks'])
            self.assertEqual(retained['retention']['source_frame_id'],source['frame_id'])
            s.verified_previous_scene(paths[-1],o)
            for bad in [dict(o,calibration_sha256='changed'),dict(o,measured14=o['measured14'][:6]+[.05]+o['measured14'][7:])]:
                with self.assertRaises(s.PerceptionUnknown):s.retain_full_scene(paths[0],paths,bad)
            with self.assertRaises(s.PerceptionUnknown):s.retain_full_scene(paths[0],paths,o,invalidated=True)
            with self.assertRaises(s.PerceptionUnknown):s.retain_full_scene(paths[0],[paths[0],*paths[2:]],o)
    def test_retention_rejects_visible_disappearance(self):
        paths=['references/rehearsal-r4/r4-transition-01.json','references/rehearsal-r4/r4-transition-02.json',
               'references/rehearsal-r4/r4-hover-05.json','references/rehearsal-r4/r4-approach-hover-02.json','references/rehearsal-r4/r4-approach-mid-01.json']
        o=json.loads(Path(paths[-1]).read_text())['result']['measured']
        original=s.cv2.imread;im=original(o['cameras']['head']['inspection_image']);im[240:350,620:730]=160
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']),patch('scene_checks.cv2.imread',return_value=im):
            with self.assertRaises(s.PerceptionUnknown):s.retain_full_scene(paths[0],paths,o)

if __name__=='__main__':unittest.main()
