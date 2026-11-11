import copy
import json
import pathlib
import tempfile
import unittest
from unittest.mock import patch
import controller as c
import rehearse_step as runner

SCHEMAS={r['name']:r['inputSchema'] for r in json.loads(pathlib.Path(__file__).with_name('tool-schemas.json').read_text())}


class Fake:
    def __init__(self):
        self.calls=[];self.q=[0.0]*14;self.q[6]=.0004;self.pos=[.314,-.274,.205]
        self.n=0;self.missing_image=False;self.fail=None;self.uncertain=False;self.remaining=1000
    def obs(self):
        self.n+=1;return {'frame_id':str(self.n),'measured14':list(self.q)}
    def value(self,tool,**kw):
        if tool in SCHEMAS:
            schema=SCHEMAS[tool]
            assert not set(kw)-set(schema['properties']), (tool,kw,'not live schema')
            assert set(schema.get('required',[])) <= set(kw)
        self.calls.append((tool,copy.deepcopy(kw)))
        if tool==self.fail:raise RuntimeError('injected ambiguous transport failure')
        if tool=='status':return {'control_uncertain':self.uncertain,'wall_budget':{'remaining_seconds':self.remaining,'finish_reserve_seconds':60,'phase':'active'},
            'robot_status':{'controlled_link':'left_link6','frame':'left_base_link',
                            'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36]}}
        if tool=='piper_observe':return self.obs()
        if tool=='piper_fk':return {'position':list(self.pos)}
        if tool=='piper_check_path':return {'cad_clear':True,'workspace_clear':True}
        if tool=='piper_hold':return {'held':True}
        if tool=='piper_gripper':self.q[6]=kw['width_m']
        elif tool=='piper_move_pose':self.pos=list(kw['position'])
        elif tool=='piper_move_joints':self.q[:6]=kw['joints']
        elif tool=='piper_pixel_to_position':return {'frame_id':kw['frame_id'],'points':[]}
        else:raise AssertionError(tool)
        r={'executed':True,'measured_arrival':True,'measured_settled':True,'final14':list(self.q)}
        if kw.get('observe_after'):
            if self.missing_image:r['observation_error']='injected image failure after successful motion'
            else:r['observation']=self.obs()
        return r


def evidence(**kw):
    return {'ok':True,'frame_id':kw['after']['frame_id'],'why':'synthetic test evidence, never hardware'}


class Workflow(unittest.TestCase):
    def test_actual_r1_feedback_replay_preserves_eight_moves_no_review_wait(self):
        records=json.loads(pathlib.Path(__file__).with_name('recorded-opening.json').read_text())
        f=Fake();f.q=list(records[0]['anchor_validation']['measured14']);original=f.value;index=[0]
        def replay(tool,**kw):
            if tool!='piper_gripper':return original(tool,**kw)
            r=copy.deepcopy(records[index[0]]);index[0]+=1
            self.assertAlmostEqual(kw['width_m'],r['target14'][6],places=7)
            f.calls.append((tool,kw));f.q=list(r['final14'])
            if not kw['observe_after']:r.pop('observation')
            return r
        f.value=replay
        phases=[]
        def assess(**kw):phases.append(kw['phase']);return evidence(**kw)
        r=c.open_empty_gripper(f,assess=assess)
        self.assertTrue(r['ok']);self.assertEqual(index[0],8)
        self.assertAlmostEqual(f.q[6],.0687);self.assertEqual(phases,['empty_opening_sweep'])
        self.assertEqual(sum(kw['observe_after'] for t,kw in f.calls if t=='piper_gripper'),1)
    def test_unknown_precondition_returns_without_wait_or_motion(self):
        f=Fake();r=c.open_empty_gripper(f)
        self.assertTrue(r['needs_review']);self.assertFalse(r['executed'])
        self.assertFalse(any(t in runner.MOTIONS for t,a in f.calls))
    def test_opening_eight_moves_one_fact_and_one_final_image(self):
        f=Fake();phases=[]
        def assess(**kw):phases.append(kw['phase']);return evidence(**kw)
        r=c.open_empty_gripper(f,assess=assess)
        self.assertTrue(r['ok']);self.assertEqual(phases,['empty_opening_sweep'])
        jaw=[a for t,a in f.calls if t=='piper_gripper']
        self.assertEqual(len(jaw),8);self.assertEqual(sum(a['observe_after'] for a in jaw),1)
        self.assertAlmostEqual(jaw[-1]['width_m'],.069)
        for a,b in zip([.0004]+[x['width_m'] for x in jaw],[x['width_m'] for x in jaw]):self.assertLessEqual(abs(a-b),.010)
    def test_closed_target_sent_and_no_lift(self):
        f=Fake();f.q[6]=.069;r=c.close_grasp(f,assess=evidence)
        self.assertTrue(r['ok']);self.assertAlmostEqual(f.q[6],.050)
        self.assertFalse(any(t=='piper_move_pose' for t,a in f.calls));self.assertEqual(r['next_function'],'trial_lift')
    def test_trial_unknown_keeps_execution_and_next_boundary(self):
        f=Fake();f.q[6]=.051
        def assess(**kw):return {'ok':None} if kw['phase']=='retained_after_trial' else evidence(**kw)
        r=c.trial_lift(f,quaternion=[1,0,0,0],assess=assess)
        self.assertTrue(r['needs_review']);self.assertTrue(r['executed']);self.assertTrue(r['action_completed'])
        self.assertEqual(r['next_function'],'transport');self.assertAlmostEqual(f.pos[2],.215)
        self.assertFalse(any(t in ('piper_hold','piper_gripper') for t,a in f.calls))
    def test_image_failure_does_not_erase_motion(self):
        f=Fake();f.missing_image=True
        r=c.approach(f,waypoints=[[.31,-.2,.27]],quaternion=[1,0,0,0],assess=evidence)
        self.assertTrue(r['needs_review']);self.assertTrue(r['executed']);self.assertTrue(r['action_completed'])
        self.assertEqual(r['review']['phase'],'observe_completed_action')
        self.assertFalse(any(t=='piper_hold' for t,a in f.calls))
    def test_real_failure_holds_once_and_no_retry(self):
        f=Fake();f.fail='piper_move_pose'
        r=c.transport(f,waypoints=[[.31,-.2,.27],[.31,-.1,.27]],quaternion=[1,0,0,0],assess=evidence)
        self.assertFalse(r['ok']);self.assertFalse(r['action_completed'])
        self.assertEqual(sum(t=='piper_move_pose' for t,a in f.calls),1)
        self.assertEqual(sum(t=='piper_hold' for t,a in f.calls),1)
    def test_known_contradiction_holds_without_release(self):
        f=Fake();f.q[6]=.051
        def assess(**kw):return {'ok':False,'why':'visible slipped block'} if kw['phase']=='retained_after_trial' else evidence(**kw)
        r=c.trial_lift(f,quaternion=[1,0,0,0],assess=assess)
        self.assertFalse(r['ok']);self.assertEqual(f.calls[-1][0],'piper_hold');self.assertAlmostEqual(f.q[6],.051)
    def test_descent_never_releases_when_support_unknown(self):
        f=Fake();f.q[6]=.051
        def assess(**kw):return {'ok':None} if kw['phase']=='supported_before_release' else evidence(**kw)
        r=c.lower_to_support(f,descent=[[.304,-.049,.265]],retreat=[[.304,-.049,.3]],quaternion=[1,0,0,0],assess=assess)
        self.assertTrue(r['needs_review']);self.assertEqual(r['next_function'],'release')
        self.assertFalse(any(t=='piper_gripper' for t,a in f.calls))
    def test_old_combined_entries_never_replay(self):
        f=Fake();self.assertFalse(c.grasp_and_trial_lift(f)['executed'])
        self.assertFalse(c.place_release_retreat(f)['executed']);self.assertEqual(f.calls,[])
    def test_release_then_separate_retreat(self):
        f=Fake();f.q[6]=.051
        self.assertTrue(c.release(f,assess=evidence)['ok']);self.assertAlmostEqual(f.q[6],.069)
        self.assertFalse(any(t=='piper_move_pose' for t,a in f.calls))
        self.assertTrue(c.retreat(f,waypoints=[[.304,-.049,.3]],quaternion=[1,0,0,0],assess=evidence)['ok'])
    def test_unknown_control_and_budget_do_not_dispatch(self):
        for attr,v in [('uncertain',True),('remaining',100)]:
            f=Fake();setattr(f,attr,v);r=c.open_empty_gripper(f,assess=evidence)
            self.assertFalse(r['ok']);self.assertFalse(r['dispatch_attempted'])
    def test_workspace_failure_before_action(self):
        f=Fake();r=c.approach(f,waypoints=[[.6,0,.2]],quaternion=[1,0,0,0],assess=evidence)
        self.assertFalse(r['ok']);self.assertFalse(r['dispatch_attempted'])
    def test_live_pose_schema_rejects_old_quaternion_keyword(self):
        with self.assertRaises(AssertionError):Fake().value('piper_move_pose',position=[.3,0,.2],quaternion=[1,0,0,0])
    def test_changed_frame_evidence_cannot_pass(self):
        f=Fake();r=c.open_empty_gripper(f,assess=lambda **kw:{'ok':True,'frame_id':'old','why':'old'})
        self.assertFalse(r['ok']);self.assertFalse(r['dispatch_attempted'])
    def test_empty_joint_function_uses_real_schema(self):
        f=Fake();r=c.empty_joint_waypoint(f,joints=[0,.1,0,0,0,0],assess=evidence)
        self.assertTrue(r['ok']);self.assertTrue(any(t=='piper_check_path' for t,a in f.calls))
    def test_runner_saves_callable_separately_and_does_not_mutate_inputs(self):
        with tempfile.TemporaryDirectory() as d:
            args={'gap_m':.069};p=pathlib.Path(d)/'output/rehearsal/one.json';f=Fake()
            r=runner.run_one(f,'open_empty_gripper',args,p,assess=evidence)
            self.assertEqual(args,{'gap_m':.069});self.assertTrue(r['result']['ok'])
            json.loads(p.read_text());state=json.loads((pathlib.Path(d)/'memory/REHEARSAL_STATE.json').read_text())
            self.assertTrue(state['executed']);self.assertEqual(state['next_function'],'approach')
            self.assertEqual(r['agent_wait_inside_function_s'],0)
            self.assertFalse(list(pathlib.Path(d).rglob('pending.json')))
    def test_runner_refuses_duplicate_report_before_tools(self):
        with tempfile.TemporaryDirectory() as d:
            p=pathlib.Path(d)/'x.json';p.write_text('{}');f=Fake()
            with self.assertRaises(FileExistsError):runner.run_one(f,'open_empty_gripper',{},p)
            self.assertEqual(f.calls,[])
    def test_callable_in_json_inputs_fails_before_tools(self):
        with tempfile.TemporaryDirectory() as d:
            f=Fake()
            with self.assertRaises(TypeError):runner.run_one(f,'open_empty_gripper',{'assess':evidence},pathlib.Path(d)/'x.json')
            self.assertEqual(f.calls,[])
    def test_runner_partial_report_retains_receipt_and_resume(self):
        with tempfile.TemporaryDirectory() as d:
            f=Fake();f.q[6]=.051
            def assess(**kw):return {'ok':None} if kw['phase']=='retained_after_trial' else evidence(**kw)
            r=runner.run_one(f,'trial_lift',{'quaternion':[1,0,0,0]},pathlib.Path(d)/'x.json',assess=assess)
            self.assertTrue(r['result']['executed']);self.assertTrue(r['result']['needs_review'])
            self.assertEqual(sum(t=='piper_move_pose' for t,a in f.calls),1)
    def test_all_runtime_tools_match_exported_contract(self):
        f=Fake();f.q[6]=.051
        # Offline historical RGB exercises semantic selection; the fake depth
        # reply is deliberately incomplete and must still fail closed.
        obs=json.loads(pathlib.Path('runtime/frames/3416ddeb272944778da7dfa780d52dcc/meta.json').read_text())
        with patch('perception.time.time_ns',return_value=obs['captured_at_ns']):
            result=c.locate_block(f,observation=obs,color='red')
        self.assertTrue(result['needs_review'])
        c.transport(f,waypoints=[[.31,-.15,.28]],quaternion=[1,0,0,0],assess=evidence)
        self.assertTrue(any(t=='piper_pixel_to_position' for t,a in f.calls))


if __name__=='__main__':unittest.main()
