"""Offline provenance and refusal regressions; no physical qualification.

Receipt/geometry validators are stubbed only in the provenance unit fixture;
existing suites exercise those validators independently.
"""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import controller
import history_scene
from scene_checks import PerceptionUnknown
from stack_checks import support_scene
from test_workflow import Fake, evidence


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class SupportHistory(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'pregrasp.json'
        self.tip = Path(self.temp.name) / 'transport.json'
        self.refresh = Path(self.temp.name) / 'refresh.json'
        self.robot = {'bridge': {'instance': 'test', 'phase': 'DONE'},
                      'owner_status': {'generation': 1}}
        self.obs = {'measured14': [0.] * 14, 'captured_at_ns': 6, 'frame_id': 'current'}
        self.scene = {'frame_id': 'source', 'blocks': {'red': {}, 'orange': {}, 'green': {}},
                      'control_identity': {'bridge_instance': 'test', 'owner_generation': 1}}
        self.write(self.root, {'inputs': {'color': 'green'}, 'result': {
            'ok': True, 'stage': 'approach', 'engage': False,
            'measured': dict(self.obs, captured_at_ns=1)}})
        self.write(self.tip, {'inputs': {'color': 'green', 'previous_report': str(self.root)},
            'result': {'ok': True, 'stage': 'transport',
                       'initial_observation': dict(self.obs, captured_at_ns=3)}})
        self.ref = {'inputs': {'previous_report': str(self.root)}, 'result': {
            'ok': True, 'stage': 'refresh_stack_support', 'executed': False,
            'support_scene': self.scene, 'start_status': {'robot_status': self.robot},
            'measured': dict(self.obs, captured_at_ns=2),
            'support_source': {'report': str(self.root), 'sha256': digest(self.root)}}}
        self.write(self.refresh, self.ref)
        for target, kwargs in (
            ('stack_checks.completed_action_state', {'side_effect': lambda r, *a, **kw: r['result']}),
            ('stack_checks.scene_geometry', {'return_value': None}),
            ('history_scene.reject_visible_displacement', {'return_value': None}),
        ):
            p = patch(target, **kwargs); p.start(); self.addCleanup(p.stop)

    def write(self, path, value):
        path.write_text(json.dumps(value))

    def check(self, scene=None):
        return support_scene(None, str(self.tip), self.obs, self.robot,
                             str(self.refresh), scene or self.scene)

    def test_bound_history_preserves_payload_scene(self):
        result = self.check()
        self.assertEqual(result['blocks']['green'], self.scene['blocks']['green'])
        self.assertEqual(len(result['stack_reference']['history']), 2)
        self.assertNotIn('stack_reference', self.scene)
        self.check(result)

    def test_legacy_unbound_refresh_refused(self):
        del self.ref['result']['support_source']; self.write(self.refresh, self.ref)
        with self.assertRaisesRegex(PerceptionUnknown, 'Missing or changed support pregrasp'):
            self.check()

    def test_changed_pregrasp_refused(self):
        self.root.write_text(self.root.read_text() + '\n')
        with self.assertRaisesRegex(PerceptionUnknown, 'support pregrasp'):
            self.check()

    def test_changed_refresh_refused(self):
        pinned = self.check(); self.refresh.write_text(self.refresh.read_text() + '\n')
        with self.assertRaisesRegex(PerceptionUnknown, 'support source changed'):
            self.check(pinned)

    def test_changed_retained_action_refused(self):
        pinned = self.check(); self.tip.write_text(self.tip.read_text() + '\n')
        with self.assertRaisesRegex(PerceptionUnknown, 'action history changed'):
            self.check(pinned)

    def test_bypassing_retained_action_refused(self):
        pinned = self.check()
        self.tip = Path(self.temp.name) / 'replacement.json'
        self.write(self.tip, {'inputs': {'color': 'green', 'previous_report': str(self.root)},
            'result': {'ok': True, 'stage': 'transport',
                       'initial_observation': dict(self.obs, captured_at_ns=3)}})
        with self.assertRaisesRegex(PerceptionUnknown, 'bypasses retained predecessor'):
            self.check(pinned)

    def test_wrong_payload_in_history_refused(self):
        r = json.loads(self.tip.read_text()); r['inputs']['color'] = 'orange'; self.write(self.tip, r)
        with self.assertRaisesRegex(PerceptionUnknown, 'another payload'):
            self.check()

    def test_root_must_be_empty_green_pregrasp(self):
        r = json.loads(self.root.read_text()); r['result']['engage'] = True; self.write(self.root, r)
        self.ref['result']['support_source']['sha256'] = digest(self.root); self.write(self.refresh, self.ref)
        with self.assertRaisesRegex(PerceptionUnknown, 'empty green pregrasp'):
            self.check()

    def test_refresh_after_pickup_boundary_refused(self):
        self.ref['result']['measured']['captured_at_ns'] = 4; self.write(self.refresh, self.ref)
        with self.assertRaisesRegex(PerceptionUnknown, 'between green hover and pickup'):
            self.check()

    def test_owner_restart_refused(self):
        self.robot = copy.deepcopy(self.robot); self.robot['bridge']['instance'] = 'restart'
        with self.assertRaisesRegex(PerceptionUnknown, 'another owner'):
            self.check()


class Boundaries(unittest.TestCase):
    def run_refresh(self, *, color='green', mutate=False):
        f = Fake()
        with tempfile.TemporaryDirectory() as d:
            source = Path(d) / 'pregrasp.json'
            record = {'inputs': {'color': color}, 'result': {
                'ok': True, 'stage': 'approach', 'engage': False}}
            source.write_text(json.dumps(record)); expected = digest(source)
            def measure(*a, **kw):
                if mutate:
                    source.write_text(source.read_text() + '\n')
                return {'blocks': {c: {'top_center': {'surface_position_m': p}} for c, p in
                        [('red', [.3, 0, .08]), ('orange', [.3, -.067, .08])]},
                        'table': {'z_ax_by_c': [0, 0, .02]}}
            with patch('scene_checks.completed_action_state', return_value=record['result']), \
                 patch('scene_checks.measure_scene', side_effect=measure), patch('scene_checks.scene_control'):
                result = controller.refresh_stack_support(f, previous_report=str(source))
            self.assertFalse(result['executed'])
            self.assertFalse(any(t in ('piper_gripper','piper_move_pose','piper_move_joints') for t, _ in f.calls))
            return result, {'report': str(source), 'sha256': expected}

    def test_refresh_pins_pregrasp_bytes(self):
        result, expected = self.run_refresh()
        self.assertTrue(result['ok'], result)
        self.assertEqual(result['support_source'], expected)

    def test_refresh_rejects_non_green_root(self):
        result, _ = self.run_refresh(color='orange')
        self.assertFalse(result['ok'])
        self.assertNotIn('support_source', result)

    def test_refresh_rejects_source_changed_during_measurement(self):
        result, _ = self.run_refresh(mutate=True)
        self.assertFalse(result['ok'])
        self.assertIn('changed during support refresh', result['why'])
        self.assertNotIn('support_source', result)

    def test_rejected_green_support_never_opens(self):
        f = Fake(); f.q[6] = .051
        prev = {'stage': 'green_stack_lower', 'support_required': True,
                'facts': [{'supported': True}], 'measured': f.obs()}
        negative = {'ok': False, 'frame_id': 'negative', 'why': 'forward overhang'}
        with patch('stack_checks.loaded_state', return_value=(prev, {}, {'color': 'green'})), \
             patch('stack_checks.assess_images', return_value=negative):
            result = controller.green_stack_release(f, previous_report='unused', support_report='unused')
        self.assertFalse(result['executed'])
        self.assertEqual(result['status'], 'rejected_scene_fact')
        self.assertEqual(result['review']['assessment']['why'], 'forward overhang')
        self.assertFalse(any(t in ('piper_gripper','piper_move_pose','piper_move_joints') for t, _ in f.calls))

    def test_generic_recovery_cannot_promote_green_stack_lower(self):
        f = Fake(); f.q[6] = .051
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'lower.json'; p.write_text('{}')
            with patch('scene_checks.read_report', return_value={'result': {}}), \
                 patch('scene_checks.completed_action_state', return_value={'stage': 'green_stack_lower'}):
                result = controller.verify_completed_action(f, previous_report=str(p), assess=evidence)
        self.assertFalse(result['ok']); self.assertFalse(result['executed'])
        self.assertIn('Unsupported recovery boundary', result['why'])
        self.assertFalse(any(t in ('piper_gripper','piper_move_pose','piper_move_joints') for t, _ in f.calls))

    def test_placement_ends_old_empty_history_without_mutating_source(self):
        scene = {'blocks': {c: {'top_center': {'surface_position_m': [.3, y, .08]}}
                           for c, y in [('red', 0), ('orange', -.2), ('green', -.3)]},
                 'retention': {'history': ['old-orange-pregrasp']}}
        original = copy.deepcopy(scene)
        release = {'ok': True, 'stage': 'release', 'start_status': {'robot_status': {'bridge': {'instance': 'test'}}},
            'receipts': [{'result': {'owner_request_id': 'release', 'final14': [0.] * 14}}],
            'measured': {'frame_id': 'released'}, 'facts': [{'scene': scene,
                'payload_origin': {'color': 'orange', 'grasp_flange': [.3, -.2, .205]}}]}
        retreat = dict(release, stage='retreat', initial_observation={})
        f = Fake(); f.pos = [.3, -.067, .205]
        with tempfile.TemporaryDirectory() as d:
            rel = Path(d) / 'release.json'; ret = Path(d) / 'retreat.json'
            rel.write_text(json.dumps({'result': release}))
            ret.write_text(json.dumps({'result': retreat, 'inputs': {'previous_report': str(rel)}}))
            with patch('history_scene.current_image'), patch('history_scene.scene_control'), \
                 patch('history_scene.scene_geometry'), patch('history_scene.reject_visible_displacement'), \
                 patch('history_scene.completed_action_state', side_effect=lambda r, *a, **kw: r['result']):
                out = history_scene.released_scene(f, {'frame_id': 'now'}, {'bridge': {'instance': 'test'}}, str(ret))
        self.assertNotIn('retention', out)
        self.assertEqual(out['prior_empty_retention'], original['retention'])
        self.assertEqual(scene, original)
        self.assertAlmostEqual(out['blocks']['orange']['top_center']['surface_position_m'][1], -.067)


if __name__ == '__main__':
    unittest.main()
