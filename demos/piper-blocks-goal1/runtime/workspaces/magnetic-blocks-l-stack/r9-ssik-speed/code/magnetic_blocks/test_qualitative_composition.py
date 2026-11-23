"""Offline regression cases; no synthetic verdict counts as physical support."""
import copy
import unittest
from unittest.mock import patch
import numpy as np
import qualitative
from scene_checks import PerceptionUnknown
from manipulation_checks import LoadedChecks
from test_r8_stack import Geometry, fixture


class SupportReuse(unittest.TestCase):
    def setUp(self):
        self.obs={'frame_id':'old','captured_at_ns':1_000_000_000,
                  'measured14':[0.]*14,'calibration_sha256':'cal',
                  'finger_geometry':{'geometry_identity':'fingers'},
                  'state_binding':{'owner_generation':1}}
        self.after=copy.deepcopy(self.obs)
        self.after.update(frame_id='new',captured_at_ns=5_000_000_000)
        self.prev={'ok':True,'stage':'green_stack_lower','measured':self.obs,'facts':[{'ok':True,'supported':True,'frame_id':'old'}]}
        self.image=np.zeros((720,1280,3),dtype=np.uint8)
        self.image[200:350,500:650]=[0,0,210]
        self.images={('old',c):self.image.copy() for c in ('head','left')}
        self.images.update({('new',c):self.image.copy() for c in ('head','left')})
        p=patch('qualitative.current_image',side_effect=lambda o,c,**kw:self.images[o['frame_id'],c])
        p.start();self.addCleanup(p.stop)

    def reuse(self):
        return qualitative.reuse_support_endpoint(self.prev,self.after,'release')

    def test_unchanged_views_reuse_positive_relation(self):
        r=self.reuse();self.assertTrue(r['ok']);self.assertTrue(r['reused_support'])
        self.assertEqual(r['frame_id'],'new');self.assertEqual(r['source_frame_id'],'old')

    def test_small_sensor_noise_is_not_a_new_support_question(self):
        self.images['new','head']=np.clip(self.image.astype(int)+3,0,255).astype('uint8')
        self.assertIsNotNone(self.reuse())

    def test_moved_block_in_either_view_needs_new_assessment(self):
        for camera in ('head','left'):
            with self.subTest(camera=camera):
                self.images['new',camera]=np.roll(self.image,80,axis=1)
                self.assertIsNone(self.reuse())
                self.images['new',camera]=self.image.copy()

    def test_old_negative_unknown_missing_or_wrong_frame_never_promoted(self):
        for fact in ({'ok':False,'supported':True,'frame_id':'old'},
                     {'ok':None,'supported':True,'frame_id':'old'},
                     {'ok':True,'supported':True,'frame_id':'other'},{}):
            self.prev['facts']=[fact];self.assertIsNone(self.reuse())

    def test_pause_owner_calibration_geometry_and_pose_changes_disable_reuse(self):
        original=copy.deepcopy(self.after)
        for key,value in [('captured_at_ns',100_000_000_000),('calibration_sha256','changed'),
                          ('state_binding',{'owner_generation':2}),
                          ('finger_geometry',{'geometry_identity':'changed'}),('measured14',[.1]*14)]:
            self.after=copy.deepcopy(original);self.after[key]=value;self.assertIsNone(self.reuse())

    def test_missing_camera_does_not_silently_reuse(self):
        with patch('qualitative.current_image',side_effect=PerceptionUnknown('missing')):
            self.assertIsNone(self.reuse())

    def test_green_release_consumes_reuse_without_another_model_verdict(self):
        from stack_checks import StackReleaseChecks
        self.prev['support_required']=True
        self.obs['measured14'][6]=self.after['measured14'][6]=.051
        with patch('stack_checks.loaded_state',return_value=(self.prev,{},{})), \
             patch('stack_checks.assess_images',side_effect=AssertionError('unnecessary model call')):
            r=StackReleaseChecks(None,'old','support')(phase='green_supported_before_release',before=self.after,after=self.after,inputs={})
        self.assertTrue(r['ok']);self.assertTrue(r['reused_support'])

    def test_changed_view_fallback_negative_is_not_overridden_by_previous_yes(self):
        from stack_checks import StackReleaseChecks
        self.prev['support_required']=True
        self.obs['measured14'][6]=self.after['measured14'][6]=.051
        self.images['new','left']=np.roll(self.image,80,axis=1)
        with patch('stack_checks.loaded_state',return_value=(self.prev,{},{})), \
             patch('stack_checks.assess_images',return_value={'ok':False,'frame_id':'new','why':'visible tilt'}) as model:
            r=StackReleaseChecks(None,'old','support')(phase='green_supported_before_release',before=self.after,after=self.after,inputs={})
        self.assertFalse(r['ok']);self.assertFalse(r['supported']);self.assertEqual(model.call_count,1)


class MergedPath(unittest.TestCase):
    def check(self,*,blocked=False,bow=False,distance=.2):
        scene,origin,obs=fixture();obs.update(frame_id='f');origin['half_xy_m']=.035
        obs['measured14'][:3]=[.3,-.2,.3]
        for f in obs['finger_geometry']['fingers']:
            for p in f['cad_corners_base_m']:p[1]-=.203
        previous={'ok':True,'stage':'transport','measured':obs,
                  'facts':[{'scene':scene,'payload_origin':origin}]}
        class Context(Geometry):
            def value(self,tool,**kw):
                if tool=='piper_status':return {}
                result=super().value(tool,**kw)
                if bow and tool=='piper_fk' and -.12<kw['joints'][1]<-.08:
                    result['position'][2]-=.15;result['T_base_flange'][2][3]-=.15
                return result
        ctx=Context();ctx.block=blocked
        with patch('manipulation_checks.read_report',return_value={'result':previous}), \
             patch('manipulation_checks.completed_action_state',return_value=previous), \
             patch('manipulation_checks.scene_control'),patch('manipulation_checks.scene_geometry'), \
             patch('manipulation_checks.retention_at_endpoint',return_value={'ok':True}):
            return LoadedChecks(ctx,'previous','green').path(obs,{'waypoints':[[.3,-.2+distance,.3]],'quaternion':[1,0,0,0]})

    def test_longer_level_path_checked_at_at_least_old_sampling_density(self):
        r=self.check();self.assertGreaterEqual(len(r['samples']),68)

    def test_merged_path_cannot_bypass_native_collision_or_interior_payload_check(self):
        for kwargs in ({'blocked':True},{'bow':True},{'distance':.401}):
            with self.assertRaises(PerceptionUnknown):self.check(**kwargs)
