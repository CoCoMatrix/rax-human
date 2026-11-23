"""R7 engineering regression: saved frames and read-only replies, no hardware."""
import copy
import json
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
import controller
import green_checks
import scene_checks as s
import manipulation_checks as m
from test_r7_closeout import report, ReadOnlyReplay


class RecordedFK:
    def __init__(self):
        self.rows=[];self.calls=[]
        for path in Path('output/rehearsal').glob('r7-*.tools.jsonl'):
            self.rows.extend(r for r in map(json.loads,path.read_text().splitlines()) if r['tool']=='piper_fk' and 'result' in r)
    def value(self,tool,**kw):
        self.calls.append(tool)
        if tool!='piper_fk':raise AssertionError('Offline FK evidence forbids '+tool)
        for r in self.rows:
            if np.allclose(kw['joints'],r['arguments']['joints'],atol=1e-10,rtol=0):return copy.deepcopy(r['result'])
        raise AssertionError('No recorded FK for '+str(kw))


class Repair(unittest.TestCase):
    def test_saved_wrist_depth_uses_its_own_pixels_and_stays_optical(self):
        import qualitative as q
        path=Path('runtime/frames/e8872eefcea64bd0aa5607058bd4bf24')
        obs=json.loads((path/'meta.json').read_text())
        with np.load(path/'left-depth.npz') as d:
            xyz=d['xyz_rgb_m'].copy();valid=d['valid'].astype(bool)
        calls=[]
        class SavedOptical:
            def value(self,tool,**kw):
                calls.append(tool)
                assert tool=='piper_pixel_to_position'
                assert kw['frame_id']==obs['frame_id'] and kw['camera']=='left' and kw['output_frame']=='camera_optical'
                K=np.asarray(obs['cameras']['left']['K']);points=[]
                for u,v in kw['pixels']:
                    mask=valid[v-3:v+4,u-3:u+4];values=xyz[v-3:v+4,u-3:u+4,2][mask]
                    assert len(values)>=9
                    z=float(np.median(values));points.append({'pixel':[u,v],'position':(np.linalg.solve(K,[u,v,1])*z).tolist(),'depth_m':z,'valid_samples':len(values)})
                return {'camera':'left','frame_id':obs['frame_id'],'frame':'left_d435_color_optical_frame','robot_target_usable':False,'base_transform_applied':False,'points':points}
        with patch('scene_checks.time.time_ns',return_value=obs['captured_at_ns']):
            result=q.wrist_depth_evidence(SavedOptical(),obs,'orange')
        self.assertTrue(result['available']);self.assertFalse(result['robot_target_usable'])
        self.assertEqual(calls,['piper_pixel_to_position']);self.assertGreaterEqual(len(result['points']),3)

    def test_optional_wrist_refresh_cannot_hide_state_change(self):
        import qualitative as q
        obs=report('r7-green-pregrasp-02')['result']['initial_observation']
        new=copy.deepcopy(obs);new['measured14'][0]+=.02;new['frame_id']='f'*32
        class Changed:
            def value(self,tool,**kw):
                assert tool=='piper_observe' and kw['depth_cameras']==['left'] and not kw['require_depth']
                return new
        with patch('scene_checks.time.time_ns',return_value=obs['captured_at_ns']):
            with self.assertRaises(q.EvidenceBindingError):q.wrist_depth_evidence(Changed(),obs,'green',request_if_missing=True)

    def test_unassociated_background_color_does_not_invalidate_reuse(self):
        from history_scene import reject_visible_displacement
        scene,_=self.green_scene();obs=report('r7-green-pregrasp-02')['result']['initial_observation']
        with patch('scene_checks.time.time_ns',return_value=obs['captured_at_ns']),patch('perception.component',return_value=({'bbox_xywh':[1,1,80,80]},None)):
            result=reject_visible_displacement(scene,obs)
            self.assertTrue(result['unassociated_candidates'])

    def test_new_empty_approach_can_start_after_placement_not_at_original_scene_pose(self):
        # Synthetic function wrapper around a recorded EMPTY move; this only
        # tests report/anchor continuity, not an unexecuted green grasp.
        from history_scene import released_scene
        r=report('r7-green-transition-02');before=r['result']['initial_observation'];after=r['result']['measured']
        robot=r['result']['start_status']['robot_status']
        with patch('scene_checks.time.time_ns',return_value=before['captured_at_ns']):
            scene=released_scene(RecordedFK(),before,robot,r['inputs']['previous_report'])
        synthetic=copy.deepcopy(r);synthetic['result']['stage']='empty_joint_waypoint'
        synthetic['result']['facts'][0]['scene']=scene
        with tempfile.TemporaryDirectory() as d:
            path=str(Path(d)/'synthetic-empty-move.json');Path(path).write_text(json.dumps(synthetic))
            with patch('scene_checks.time.time_ns',return_value=after['captured_at_ns']):
                retained=s.retain_full_scene(path,[path],after)
                self.assertEqual(retained['blocks'],scene['blocks'])
                bad=copy.deepcopy(synthetic);bad['result']['facts'][0]['scene']['blocks']['green']['top_center']['surface_position_m'][0]+=.04
                Path(path).write_text(json.dumps(bad))
                with self.assertRaises(s.PerceptionUnknown):s.retain_full_scene(path,[path],after)

    def green_scene(self,mutate=None,invalidated=False):
        r=report('r7-green-pregrasp-02');obs=r['result']['initial_observation']
        robot=r['result']['start_status']['robot_status']
        if mutate:mutate(obs,robot)
        ctx=RecordedFK()
        with patch('scene_checks.time.time_ns',return_value=obs['captured_at_ns']),patch('scene_checks.measure_scene',side_effect=AssertionError('Must not re-localize')):
            scene=green_checks.refreshed_base_scene(ctx,obs,robot,r['inputs']['stationary_scene_report'],previous_report=r['inputs']['previous_report'],invalidated=invalidated)
        return scene,ctx

    def test_occluded_green_reuses_red_green_and_updates_orange_without_new_depth(self):
        scene,ctx=self.green_scene()
        original=report('r7-transition-01')['result']['facts'][0]['scene']
        for color in ('red','green'):
            self.assertEqual(scene['blocks'][color]['top_center'],original['blocks'][color]['top_center'])
        self.assertEqual(scene['blocks']['orange']['top_center']['measurement_kind'],'action_derived')
        self.assertNotEqual(scene['blocks']['orange']['top_center']['surface_position_m'],original['blocks']['orange']['top_center']['surface_position_m'])
        self.assertEqual(ctx.calls,['piper_fk'])
        self.assertEqual(scene['reuse']['history'][-1]['stage'],'release')

    def test_retained_scene_refuses_reset_owner_calibration_and_intervening_motion(self):
        mutations=[lambda o,r:r['bridge'].update(instance='new'),
            lambda o,r:o.update(calibration_sha256='changed'),
            lambda o,r:r.update(last_motion={'owner_request_id':'new'}),
            lambda o,r:o['finger_geometry'].update(geometry_identity={'changed':True})]
        for mutate in mutations:
            with self.subTest(mutate=mutate),self.assertRaises(s.PerceptionUnknown):self.green_scene(mutate)
        with self.assertRaises(s.PerceptionUnknown):self.green_scene(invalidated=True)

    def test_missing_body_does_not_pass_low_template_score(self):
        import qualitative as q
        r=report('r7-lower-04');before=r['result']['initial_observation'];after=r['result']['measured']
        with patch('scene_checks.time.time_ns',return_value=after['captured_at_ns']),patch('scene_checks.wrist_between_pads',side_effect=s.PerceptionUnknown('missing body')),patch.object(q,'assess_images',return_value={'ok':None,'why':'missing actual evidence'}):
            with self.assertRaises(s.PerceptionUnknown):q.retention_evidence(None,before,after,'orange')

    def test_green_entry_does_not_force_head_depth(self):
        with patch.object(controller, 'approach', return_value={}) as fn:
            controller.green_pregrasp(None, position=[.3,-.3,.265], quaternion=[1,0,0,0],
                previous_report='previous', stationary_scene_report='source')
        self.assertFalse(fn.call_args.kwargs['require_depth'])

    def test_template_change_is_diagnostic_not_failure(self):
        r=report('r7-lower-04'); before=r['result']['initial_observation']; after=r['result']['measured']
        with patch('scene_checks.time.time_ns',return_value=after['captured_at_ns']):
            d=s.wrist_trial_match(before,after)
        self.assertLess(d['correlation'],.75)
        self.assertFalse(d['motion_authority'])

    def test_retreat_recovery_uses_current_qualitative_check(self):
        r=report('r7-retreat-01'); before=r['result']['initial_observation']; after=r['result']['measured']
        answer={'ok':True,'frame_id':after['frame_id'],'why':'offline qualitative verdict'}
        with patch('qualitative.assess_images',return_value=answer) as fn:
            result=m.inspect_retreat_endpoint(phase='final_structure',before=before,after=after,inputs={},ctx=object())
        self.assertTrue(result['ok']);fn.assert_called_once()

    def test_support_no_longer_requires_old_wrist_body_gate(self):
        r=report('r7-lower-04'); before=r['result']['initial_observation']; after=r['result']['measured']
        answer={'ok':True,'frame_id':after['frame_id'],'why':'offline qualitative verdict'}
        with patch.object(m,'wrist_between_pads',side_effect=s.PerceptionUnknown('old gate')),patch('qualitative.assess_images',return_value=answer):
            result=m.LowerChecks(None,'unused')(phase='supported_before_release',before=before,after=after,inputs={})
        self.assertTrue(result['ok'],result)


if __name__=='__main__':unittest.main()
