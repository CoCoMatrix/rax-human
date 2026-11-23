"""Offline regressions using immutable R4 evidence; never opens a Context."""
import copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import scene_checks as s
import controller as c
R=Path('references/rehearsal-r4')
H=[str(R/(n+'.json')) for n in ('r4-transition-01','r4-transition-02','r4-hover-05','r4-approach-hover-02','r4-approach-mid-01')]
def record(name):return json.loads((R/(name+'.json')).read_text())
class Offline:
    def __init__(self,obs,position=None,last_motion=None):self.obs=obs;self.position=position;self.last_motion=last_motion;self.calls=[]
    def value(self,name,**kw):
        self.calls.append((name,kw))
        if name=='piper_fk':return {'position':self.position,'T_base_flange':np.diag([1,-1,-1,1]).tolist()}
        if name=='piper_observe':return self.obs
        if name=='status':return {'control_uncertain':False,'wall_budget':{'remaining_seconds':500,'finish_reserve_seconds':60,'phase':'active'},'robot_status':{'controlled_link':'left_link6','frame':'left_base_link','motion_ready':True,'bridge':{'phase':'IDLE'},'last_motion':self.last_motion}}
        raise AssertionError('Unexpected tool/motion '+name)
class OcclusionRepair(unittest.TestCase):
    def test_full_xyz_survives_no_new_depth(self):
        o=record('r4-approach-mid-01')['result']['measured'];o['cameras']['head']['metric_depth']=False
        source=record('r4-transition-01')['result']['facts'][0]['scene']
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']),patch('scene_checks.np.load',side_effect=AssertionError('No depth read')):
            retained=s.retain_full_scene(H[0],H,o)
        self.assertEqual(retained['blocks'],source['blocks'])
        self.assertNotEqual(retained['frame_id'],o['frame_id'])
    def test_geometry_change_invalidates_retained_target(self):
        o=record('r4-approach-mid-01')['result']['measured']
        for key in ('geometry','K','T'):
            bad=copy.deepcopy(o)
            if key=='geometry':bad['finger_geometry']['geometry_identity']['tool_calibration_sha256']='changed'
            elif key=='K':bad['cameras']['head']['K'][0][0]+=1
            else:bad['cameras']['head']['T_base_camera'][0][3]+=.01
            with patch('scene_checks.time.time_ns',return_value=bad['captured_at_ns']):
                with self.assertRaises(s.PerceptionUnknown):s.retain_full_scene(H[0],H,bad)
    def test_same_generation_after_restart_not_same_owner(self):
        scene={};robot={'bridge':{'phase':'IDLE','instance':'first'},'owner_status':{'generation':1}}
        s.scene_control(scene,robot,establish=True);s.scene_control(scene,robot)
        robot['bridge']['instance']='new'
        with self.assertRaises(s.PerceptionUnknown):s.scene_control(scene,robot)
    def test_insert_clipped_merged_body_no_head_depth_needed(self):
        r=record('r4-insert-01');o=r['result']['measured'];o['cameras']['head']['metric_depth']=False
        scene=r['result']['facts'][0]['scene'];ctx=Offline(o,r['inputs']['waypoints'][-1])
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']),patch('scene_checks.np.load',side_effect=AssertionError('No new depth')):
            answer=s.supported_entry(ctx,o,scene)
        self.assertTrue(answer['ok'],answer)
        self.assertEqual([x[0] for x in ctx.calls],['piper_fk'])
        self.assertNotIn('current_surface',answer)
    def test_one_occluded_pad_allowed_but_missing_body_not_pass(self):
        o=record('r4-insert-01')['result']['measured'];im=s.cv2.imread(o['cameras']['left']['inspection_image'])
        one=im.copy();one[500:720,950:1180]=140
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']),patch('scene_checks.cv2.imread',return_value=one):s.wrist_between_pads(o,'orange')
        none=im.copy();none[230:520,420:950]=140
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']),patch('scene_checks.cv2.imread',return_value=none):
            with self.assertRaises(s.PerceptionUnknown):s.wrist_between_pads(o,'orange')
    def test_readonly_recovery_no_action(self):
        raw=record('r4-close-01');o=raw['result']['measured'];raw['result']['measured']={'measured14':o['measured14'],'control_feedback_only':True}
        raw['result'].update(ok=False,needs_review=True,review={'phase':'observe_completed_action'})
        ctx=Offline(o,last_motion=raw['result']['receipts'][-1]['result'])
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'action.json';p.write_text(json.dumps(raw))
            with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']):
                result=c.verify_completed_action(ctx,previous_report=str(p),assess=lambda **kw:{'ok':True,'frame_id':kw['after']['frame_id'],'why':'offline supplied endpoint fact'})
            self.assertTrue(result['ok'],result);self.assertFalse(result['executed'])
            self.assertEqual([x[0] for x in ctx.calls],['status','piper_observe'])
    def test_out_and_back_cannot_reuse_same_pose(self):
        raw=record('r4-close-01');o=raw['result']['measured']
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']):
            with self.assertRaises(s.PerceptionUnknown):s.completed_action_state(raw,o,{'last_motion':{'owner_request_id':'different'}})
    def test_partial_sequence_cannot_be_recovered_as_complete(self):
        raw=record('r4-insert-01');o=raw['result']['measured'];raw['result']['planned_motion_count']=2
        with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']):
            with self.assertRaises(s.PerceptionUnknown):s.completed_action_state(raw,o)
    def test_wrist_comparison_marks_shift_as_inconclusive_diagnostic(self):
        before=record('r4-close-01')['result']['measured'];after=copy.deepcopy(before)
        after['frame_id']='f'*32;after['captured_at_ns']+=1000000000
        image=s.cv2.imread(before['cameras']['left']['inspection_image'])
        shifted=s.cv2.warpAffine(image,np.float32([[1,0,0],[0,1,45]]),(1280,720))
        with patch('scene_checks.current_image',side_effect=[image,image]):
            self.assertGreater(s.wrist_trial_match(before,after)['correlation'],.9)
        with patch('scene_checks.current_image',side_effect=[image,shifted]):
            self.assertFalse(s.wrist_trial_match(before,after)['reliable_match'])
    def test_close_entry_requests_rgb_only_and_no_relocalization(self):
        raw=record('r4-insert-01');o=raw['result']['measured'];o['cameras']['head']['metric_depth']=False
        ctx=Offline(o,raw['inputs']['waypoints'][-1],raw['result']['receipts'][-1]['result'])
        base=ctx.value
        def value(name,**kw):
            answer=base(name,**kw)
            if name=='status':answer['robot_status'].update(bridge={'phase':'DONE','instance':'fixture-bridge'},owner_status={'generation':1})
            return answer
        ctx.value=value
        raw['result']['facts'][0]['scene']['control_identity']={'bridge_instance':'fixture-bridge','owner_generation':1}
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'previous.json';path.write_text(json.dumps(raw))
            with patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']),patch('controller._jaw',return_value=o) as jaw:
                result=c.close_grasp(ctx,previous_report=str(path))
            self.assertTrue(result['ok'],result);jaw.assert_called_once()
        obs=[kw for name,kw in ctx.calls if name=='piper_observe'];self.assertEqual(len(obs),1)
        self.assertIs(obs[0]['require_depth'],False)
        self.assertNotIn('piper_pixel_to_position',[name for name,_ in ctx.calls])
    def test_real_recorded_near_views_are_qualitatively_usable(self):
        for name in ('r4-approach-mid-01','r4-insert-01','r4-close-01'):
            o=record(name)['result']['measured']
            with self.subTest(name=name),patch('scene_checks.time.time_ns',return_value=o['captured_at_ns']):s.wrist_between_pads(o,'orange')
if __name__=='__main__':unittest.main()
