"""R7 saved-evidence regressions. All contexts are local fakes; no live runtime."""
import copy
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import controller
import green_checks
import manipulation_checks as m
import perception
import scene_checks as s

REPORTS = Path('output/rehearsal')
FAILED_FRAME = 'ed11dd3c4e224bda89363ae91324d679'


def report(name):
    return json.loads((REPORTS / (name + '.json')).read_text())


class SavedDepth:
    """Offline median optical-depth projection, NOT a native tool validation."""
    def __init__(self, obs):
        self.obs = obs
        with np.load(Path('runtime/frames') / obs['frame_id'] / 'head-depth.npz') as d:
            self.xyz = d['xyz_rgb_m'].copy()
            self.valid = d['valid'].astype(bool) & np.isfinite(self.xyz).all(axis=2)
        self.calls = []

    def value(self, tool, **kw):
        if tool != 'piper_pixel_to_position':
            raise AssertionError('Offline depth adapter forbids: ' + tool)
        assert kw['frame_id'] == self.obs['frame_id'] and kw['camera'] == 'head'
        assert kw['radius'] == 3
        self.calls.append(kw)
        head = self.obs['cameras']['head']
        T, K = np.array(head['T_base_camera']), np.array(head['K'])
        points = []
        for u, v in kw['pixels']:
            valid = self.valid[v-3:v+4, u-3:u+4]
            depth = self.xyz[v-3:v+4, u-3:u+4, 2][valid]
            if len(depth) < 9:
                raise AssertionError('Insufficient saved native depth')
            z = float(np.median(depth))
            pos = T[:3, :3] @ (np.linalg.solve(K, [u, v, 1]) * z) + T[:3, 3]
            points.append({'pixel': [u, v], 'valid_samples': len(depth),
                           'depth_m': z, 'position': pos.tolist()})
        return {'frame_id': self.obs['frame_id'], 'camera': 'head',
                'frame': 'left_base_link', 'base_transform_applied': True,
                'robot_target_usable': True, 'calibration_provisional': True,
                'calibration_sha256': self.obs['calibration_sha256'], 'points': points}


class ReadOnlyReplay:
    """Consume recorded read-only path replies; reject dispatch and new calls."""
    ALLOWED = {'piper_status', 'piper_fk', 'piper_solve_ik', 'piper_check_path'}

    def __init__(self, name, mutate=None):
        rows = [json.loads(l) for l in (REPORTS / (name + '.tools.jsonl')).read_text().splitlines()]
        self.rows = [r for r in rows if r['tool'] in self.ALLOWED]
        self.calls = []
        self.mutate = mutate
        self.fk_count = 0

    def value(self, tool, **kw):
        assert tool in self.ALLOWED, 'Offline replay forbids ' + tool
        row = self.rows[len(self.calls)]
        assert row['tool'] == tool, (row['tool'], tool)
        assert row['arguments'].keys() == kw.keys()
        for key in kw:
            np.testing.assert_allclose(kw[key], row['arguments'][key], atol=1e-10)
        self.calls.append(tool)
        result = copy.deepcopy(row['result'])
        if tool == 'piper_fk':
            self.fk_count += 1
            if self.mutate:
                self.mutate(result, self.fk_count)
        return result


class R7Depth(unittest.TestCase):
    def setUp(self):
        self.obs = json.loads((Path('runtime/frames') / FAILED_FRAME / 'meta.json').read_text())

    def test_failed_windows_empty_replacement_uses_real_samples(self):
        ctx = SavedDepth(self.obs)
        for u, v in [[382,279],[376,279],[388,279],[382,273],[382,285]]:
            self.assertEqual(ctx.valid[v-3:v+4, u-3:u+4].sum(), 0)
        with patch('scene_checks.time.time_ns', return_value=self.obs['captured_at_ns']):
            scene = s.measure_scene(ctx, self.obs, colors=['red', 'orange'])
        self.assertEqual(set(scene['blocks']), {'red', 'orange'})
        for color in ['red', 'orange']:
            b = scene['blocks'][color]
            self.assertTrue(.065 < b['top_center']['surface_position_m'][2] < .09)
            self.assertEqual(b['top_center']['frame_id'], FAILED_FRAME)
            self.assertGreaterEqual(min(b['selection']['depth_selection']['valid_samples']), 9)
        self.assertTrue(scene['blocks']['orange']['selection']['depth_selection']['reselected'])
        self.assertEqual(len(ctx.calls), 3)
        orange=scene['blocks']['orange']['top_center']
        self.assertFalse(orange['center_depth_measured'])
        self.assertIn('center depth unavailable', orange['height_source'])

    def test_missing_depth_returns_unknown_without_query(self):
        with patch('scene_checks.time.time_ns', return_value=self.obs['captured_at_ns']):
            rgb, xyz, valid = s.depth_arrays(self.obs)
            # Keep the table but remove the whole orange rectangle's depth.
            valid[258:349, 304:412] = False
            ctx = SavedDepth(self.obs)
            with patch.object(s, 'depth_arrays', return_value=(rgb, xyz, valid)):
                with self.assertRaises(s.PerceptionUnknown):
                    s.measure_scene(ctx, self.obs, colors=['orange'])
            self.assertEqual(ctx.calls, [])

    def test_center_hole_with_nonflat_patch_remains_unknown(self):
        ctx=SavedDepth(self.obs); original=ctx.value
        def sloped_patch(tool, **kw):
            reply=original(tool, **kw)
            for i, point in enumerate(reply['points']):
                point['position'][2] += i*.0015
            return reply
        ctx.value=sloped_patch
        with patch('scene_checks.time.time_ns', return_value=self.obs['captured_at_ns']):
            with self.assertRaisesRegex(s.PerceptionUnknown, 'flat, spatially spread'):
                s.measure_scene(ctx, self.obs, colors=['orange'])

    def test_invalid_pixels_and_color_boundary_cannot_supply_support(self):
        distance = np.zeros((720, 1280), np.float32)
        distance[100:130, 100:130] = 10
        valid = np.zeros((720, 1280), bool)
        valid[200:230, 200:230] = True
        with self.assertRaisesRegex(s.PerceptionUnknown, 'Insufficient real depth'):
            perception.supported_depth_pixels(distance, valid, [[110, 110], [116, 110], [110, 116]])

    def test_green_without_action_history_is_not_promoted_from_old_frame(self):
        with self.assertRaisesRegex(s.PerceptionUnknown, 'history'):
            green_checks.refreshed_base_scene(None,self.obs,{},'output/rehearsal/r7-transition-01.json')

    def test_green_is_one_empty_function_without_forced_depth(self):
        with patch.object(controller, 'approach', return_value={'ok': False}) as approach:
            controller.green_pregrasp(None, position=[.3,-.2,.265], quaternion=[1,0,0,0],
                previous_report='previous', stationary_scene_report='source', scene_invalidated=True)
        self.assertFalse(approach.call_args.kwargs['require_depth'])
        self.assertFalse(approach.call_args.kwargs['engage'])
        self.assertTrue(approach.call_args.kwargs['assess'].invalidated)


class R7Recovery(unittest.TestCase):
    def recovery(self, mutate=None, target_offset=0):
        r = report('r7-raise-recovery-02')
        obs, inputs = r['result']['initial_observation'], copy.deepcopy(r['inputs'])
        inputs['waypoints'][0][2] += target_offset
        ctx = ReadOnlyReplay('r7-raise-recovery-02', mutate)
        with patch('scene_checks.time.time_ns', return_value=obs['captured_at_ns']):
            result = m.LoadedChecks(ctx, inputs['previous_report']).path(obs, inputs)
        return result, ctx

    def test_recorded_recovery_passes_readonly_path_replay(self):
        result, ctx = self.recovery()
        self.assertTrue(result['ok'])
        self.assertTrue(result['prechecked_placement_retreat'])
        self.assertEqual(len(result['samples']), 41)
        self.assertEqual(len(ctx.calls), len(ctx.rows))

    def test_different_retreat_target_refuses(self):
        with self.assertRaisesRegex(s.PerceptionUnknown, 'prechecked retreat target'):
            self.recovery(target_offset=.005)

    def test_lateral_bow_above_two_mm_refuses(self):
        def bow(f, n):
            if n == 3:
                f['position'][0] += .003
                f['T_base_flange'][0][3] += .003
        with self.assertRaisesRegex(s.PerceptionUnknown, 'lateral/downward'):
            self.recovery(bow)

    def test_neighbor_facing_drift_and_downward_sample_refuse(self):
        for axis, delta in [(1, .001), (2, -.01)]:
            def mutate(f, n):
                if n == 3:
                    f['position'][axis] += delta
                    f['T_base_flange'][axis][3] += delta
            with self.subTest(axis=axis), self.assertRaises(s.PerceptionUnknown):
                self.recovery(mutate)

    def test_original_sixty_mm_placement_still_refuses_finger_overlap(self):
        r = report('r7-lower-02'); obs = r['result']['initial_observation']
        ctx = ReadOnlyReplay('r7-lower-02')
        with patch('scene_checks.time.time_ns', return_value=obs['captured_at_ns']):
            with self.assertRaisesRegex(s.PerceptionUnknown, 'intersects neighboring red'):
                m.LowerChecks(ctx, r['inputs']['previous_report']).path(obs, r['inputs'])

    def test_supported_endpoint_uses_broad_assessment_not_retention_template(self):
        r = report('r7-lower-04'); before = r['result']['initial_observation']; after = r['result']['measured']
        for verdict in [True, False, None]:
            fact = {'ok': verdict, 'frame_id': after['frame_id'], 'why': 'offline supplied verdict'}
            with patch('scene_checks.time.time_ns', return_value=after['captured_at_ns']), \
                 patch('scene_checks.wrist_trial_match', side_effect=AssertionError('Wrong endpoint gate')), \
                 patch('qualitative.assess_images', return_value=fact):
                result = m.LowerChecks(None, 'unused', True)(phase='supported_before_release',
                    before=before, after=after, inputs={})
            self.assertIs(result['ok'], verdict)
            self.assertEqual(result['supported'], verdict is True)

    def test_recovered_support_does_not_rewrite_original_outcome(self):
        p = REPORTS / 'r7-lower-04.json'; original = p.read_bytes()
        recovered = s.read_report(REPORTS / 'r7-support-verify.json')['result']
        self.assertTrue(recovered['ok'])
        self.assertFalse(json.loads(original)['result']['ok'])
        self.assertTrue(json.loads(original)['result']['executed'])
        self.assertEqual(hashlib.sha256(p.read_bytes()).digest(), hashlib.sha256(original).digest())

    def test_green_payload_cannot_enter_orange_placement(self):
        r=report('r7-lower-02');obs=r['result']['initial_observation']
        previous=s.read_report(r['inputs']['previous_report'])
        previous['result']['facts'][0]['payload_origin']['color']='green'
        with patch('scene_checks.time.time_ns', return_value=obs['captured_at_ns']), \
             patch.object(m, 'read_report', return_value=previous):
            with self.assertRaisesRegex(s.PerceptionUnknown, 'orange only'):
                m.LowerChecks(ReadOnlyReplay('r7-lower-02'), 'unused').path(obs, r['inputs'])

    def test_retired_width_gate_cannot_authorize_recovery(self):
        self.assertFalse(hasattr(m,'released_body_recedes'))


if __name__ == '__main__':
    unittest.main()
