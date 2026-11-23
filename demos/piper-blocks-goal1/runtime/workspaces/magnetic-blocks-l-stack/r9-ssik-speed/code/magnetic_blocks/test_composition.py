"""Offline composition contracts, not a simulation or physical qualification.

Real main, runner, primitives, receipt/owner continuity, support history and target
calculations run on a synthetic context. Vision/scene collision checkers are
explicit test doubles; inherited tests cover their saved-evidence contracts.
"""
import copy
from collections import Counter
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
import controller as c
import composition as glue
import rehearse_step
import scene_checks
import stack_checks


class SceneContext:
    def __init__(self, shift=(0.,0.,0.), pickup_error=0.):
        self.calls=[]; self.q=[0.]*14; self.q[6]=.0004; self.q[13]=.002
        self.poses={}; self.n=0; self.last=None; self.fail_motion=None
        self.negative_phase=None; self.model_delay=0.; self.pickup_error=pickup_error
        self.omit_observation=None
        self.scene={'frame_id':'source','table':{'z_ax_by_c':[0.,0.,.018+shift[2]]},
            'blocks':{color:{'top_center':{'surface_position_m':(np.array(p)+shift).tolist(),'frame_id':'source'},
                             'selection':{}} for color,p in {
                'red':[.307,-.017,.078], 'orange':[.303,-.155,.078],
                'green':[.304,-.281,.078]}.items()},
            'control_identity':{'bridge_instance':'offline','owner_generation':1}}
        self.placed_orange=None

    def robot(self):
        return {'controlled_link':'left_link6','frame':'left_base_link','motion_ready':True,
            'workspace_min':[-1.,-1.,.10],'workspace_max':[1.,1.,.60],
            'bridge':{'phase':'DONE','instance':'offline'},'owner_status':{'generation':1},
            'last_motion':self.last}

    def obs(self):
        self.n+=1
        return {'frame_id':str(self.n),'captured_at_ns':self.n,'measured14':list(self.q),
            'calibration_sha256':'a'*64,
            'state_binding':{'owner_generation':1,'rest_consistent':True,'owner_unchanged':True},
            'cameras':{c:{'metric_depth':False} for c in ('head','left','right')}}

    def encode(self, position, quaternion):
        quaternion=[-x for x in quaternion] if quaternion[0]<0 else quaternion
        q=[position[0]*2,position[1]*2,(position[2]-.21)*2,*quaternion[:3]]
        self.poses[tuple(q)]=(list(position),list(quaternion))
        return q

    def fk(self, joints):
        default=([joints[0]/2,joints[1]/2,.21+joints[2]/2],
                 [1.,0.,0.,0.] if any(joints) else [-.478,.478,-.521,.521])
        p,q=self.poses.get(tuple(joints),default)
        q=(np.array(q)/np.linalg.norm(q)).tolist()
        x,y,z,w=q
        R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=p
        return {'position':list(p),'quaternion_xyzw':q,'T_base_flange':T.tolist()}

    def value(self, tool, **kw):
        self.calls.append((tool,copy.deepcopy(kw)))
        if tool=='status':return {'control_uncertain':False,'robot_status':self.robot(),
            'wall_budget':{'phase':'active','remaining_seconds':10000,'finish_reserve_seconds':60}}
        if tool=='piper_status':return self.robot()
        if tool=='piper_observe':return self.obs()
        if tool=='piper_fk_batch':return {'executed':False,'frame':'left_base_link','T_base_flange':[self.fk(q)['T_base_flange'] for q in kw['joints_batch']]}
        if tool=='piper_fk':return self.fk(kw['joints'])
        if tool=='piper_solve_ik':return {'ok':True,'q':self.encode(kw['position'],kw['quaternion_xyzw'])}
        if tool=='piper_check_path':return {'cad_clear':True,'workspace_clear':True}
        if tool=='piper_pixel_to_position':return {'offline_test_only':True}
        if tool=='gemini_generate':
            if self.model_delay:time.sleep(self.model_delay)
            return {'offline_test_only':True}
        if tool=='piper_hold':return {'held':True}
        if tool not in rehearse_step.MOTIONS:raise AssertionError(tool)
        anchor=list(self.q)
        if self.fail_motion and self.fail_motion(tool,kw,self):
            raise RuntimeError('injected ambiguous dispatch; receipt lost')
        if tool=='piper_move_joints':self.q[:6]=kw['joints']
        elif tool=='piper_move_pose':
            p=list(kw['position'])
            # Measured green insertion differs from commanded X, within ordinary
            # synthetic tolerance. Later stack XY must use actual pickup relation.
            green=self.scene['blocks']['green']['top_center']['surface_position_m']
            if abs(p[1]-green[1])<.01 and p[2]<green[2]+.135:p[0]+=self.pickup_error
            self.q[:6]=self.encode(p,kw['quaternion_xyzw'])
        else:self.q[6]=kw['width_m']
        r={'executed':True,'measured_arrival':True,'measured_settled':True,
           'final14':list(self.q),'anchor_validation':{'measured14':anchor},
           'owner_generation':1,'owner_request_id':str(len(self.calls))}
        self.last=copy.deepcopy(r)
        if kw.get('observe_after'):
            if self.omit_observation and self.omit_observation(tool,kw,self):
                r['observation_error']='synthetic image failure after completed motion'
            else:r['observation']=self.obs()
        return r


def fact(obs, **fields):
    return {'ok':True,'frame_id':obs['frame_id'],'why':'synthetic offline scene evidence',**fields}


class Composition(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.f=SceneContext()
        self.seen_contexts=[]
        def watch(fn):
            def call(ctx,*args,**kwargs):
                self.seen_contexts.append(ctx)
                return fn(ctx,*args,**kwargs)
            return call
        # Keep every rehearsed function real and assert identical context identity.
        for name in rehearse_step.FUNCTIONS:
            self.patch('controller.'+name,watch(getattr(c,name)))
        self.patch('scene_checks.current_image',lambda *a,**k:None)
        self.patch('stack_checks.current_image',lambda *a,**k:None)
        self.patch('scene_checks.scene_geometry',lambda *a,**k:None)
        self.patch('stack_checks.scene_geometry',lambda *a,**k:None)
        def measure(ctx,obs,reference_scene=None,colors=None):
            scene=copy.deepcopy(ctx.scene);scene['frame_id']=obs['frame_id']
            if ctx.placed_orange is not None:
                scene['blocks']['orange']['top_center']['surface_position_m']=ctx.placed_orange
            if colors:scene['blocks']={x:scene['blocks'][x] for x in colors}
            for color in scene['blocks']:
                ctx.value('piper_pixel_to_position',camera='head',frame_id=obs['frame_id'],pixels=[[1,1]],radius=3)
            return scene
        self.patch('composition.measure_scene',measure)
        self.patch('scene_checks.measure_scene',measure)
        self.patch('scene_checks.empty_opening',lambda ctx,obs,gap:fact(obs,scene=measure(ctx,obs)))
        self.patch('scene_checks.empty_joint_path',lambda ctx,obs,joints,retained_scene=None:
                   fact(obs,scene=copy.deepcopy(retained_scene) if retained_scene else measure(ctx,obs)))
        def resolved(previous,obs,**kwargs):
            return copy.deepcopy(scene_checks.read_report(previous)['result']['facts'][0]['scene'])
        self.patch('scene_checks.resolve_empty_scene',resolved)
        self.patch('composition.resolve_empty_scene',resolved)
        self.patch('scene_checks.approach_scene',lambda ctx,obs,scene,color,target,**k:
            fact(obs,scene=scene,flange_above_block_top_m=target[2]-scene['blocks'][color]['top_center']['surface_position_m'][2]))
        self.patch('scene_checks.supported_entry',lambda ctx,obs,scene,color:fact(obs,scene=scene))
        self.patch('qualitative.alignment_evidence',lambda ctx,obs,color:fact(obs))
        self.patch('qualitative.retention_evidence',lambda ctx,before,after,color:fact(after))
        self.patch('stack_checks.retention_evidence',lambda ctx,before,after,color:fact(after))
        self.patch('history_scene.reject_visible_displacement',lambda *a:None)
        def released(ctx,obs,robot,previous_report,**kwargs):
            rec=scene_checks.read_report(previous_report)
            while rec['result']['stage']!='release':rec=scene_checks.read_report(rec['inputs']['previous_report'])
            f=rec['result']['facts'][0];scene=copy.deepcopy(f['scene'])
            origin=f['payload_origin'];p=ctx.fk(rec['result']['measured']['measured14'][:6])['position']
            old=scene['blocks']['orange']['top_center']['surface_position_m']
            ctx.placed_orange=(np.array(old)+p-np.array(origin['grasp_flange'])).tolist()
            scene['blocks']['orange']['top_center']['surface_position_m']=ctx.placed_orange
            return scene
        self.patch('history_scene.released_scene',released)
        self.patch('scene_checks.installed_finger_corners',lambda ctx,obs,scene,current_fk=None:
            np.array([(current_fk or ctx.fk(obs['measured14'][:6]))['position']]))
        def green_path(checker,obs,args):
            return fact(obs,scene=released(checker.ctx,obs,checker.ctx.robot(),checker.previous))
        self.patch('green_checks.GreenPregrasp.path',green_path)
        def green_prepare(checker,obs):
            return {'start':obs['measured14'],'fk':checker.ctx.fk(obs['measured14'][:6]),
                'scene':released(checker.ctx,obs,checker.ctx.robot(),checker.previous),
                'tcp_offset_flange':[0.,0.,.14],'local':np.zeros((8,3)),'frame_id':obs['frame_id']}
        self.patch('green_checks.GreenPregrasp.prepare',green_prepare)
        def planned_green(ctx,start,scene,local,args):
            return {'ok':True,'end_joints':ctx.encode(args['waypoints'][0],args['quaternion']),
                    'min_block_allowance_m':.01}
        self.patch('green_checks.planned_green_path',planned_green)
        self.patch('green_route.planned_green_path',planned_green)
        self.patch('green_route.insertion_preview',lambda *a:{'segments':[]})


        def payload(ctx,previous,obs,color,support=None):
            rec=scene_checks.read_report(previous);r=rec['result'];f=copy.deepcopy(r['facts'][0])
            scene=f['scene'];origin=f.get('payload_origin')
            if origin is None:
                close=scene_checks.read_report(rec['inputs']['previous_report'])['result']
                p=ctx.fk(close['measured']['measured14'][:6])['position'];plane=scene['table']['z_ax_by_c']
                origin={'color':color,'grasp_flange':p,'half_xy_m':.035,
                        'flange_to_bottom_m':p[2]-(p[0]*plane[0]+p[1]*plane[1]+plane[2])}
            if support:
                scene=stack_checks.support_scene(ctx,previous,obs,ctx.robot(),support,scene)
            return fact(obs,scene=scene,payload_origin=origin)

        def loaded(ctx,previous,color='orange',support_report=None):
            def check(*,phase,before,after,inputs):
                return payload(ctx,previous,after,color,support_report) if phase=='retained_and_loaded_path' else fact(after)
            return check
        self.patch('manipulation_checks.LoadedChecks',loaded)
        def visual(ctx,phase,before,after,question='',**kwargs):
            ctx.value('gemini_generate',messages=[],reasoning={'effort':'low'},max_tokens=1,response_format={'type':'json_object'})
            return fact(after)|({'ok':False,'why':'synthetic conflicting support'} if ctx.negative_phase==phase else {})
        self.patch('stack_checks.assess_images',visual)
        def lower(ctx,previous,support_required=True):
            def check(*,phase,before,after,inputs):
                if phase=='placement_and_retreat_path':return payload(ctx,previous,after,'orange')
                return visual(ctx,phase,before,after)|{'supported':True} if support_required else fact(after,supported=False)
            return check
        self.patch('manipulation_checks.LowerChecks',lower)
        def release(ctx,previous):
            def check(*,phase,before,after,inputs):
                return payload(ctx,previous,after,'orange')|visual(ctx,phase,before,after)|{'supported':True}
            return check
        self.patch('manipulation_checks.VisualReleaseChecks',release)
        class Retreat:
            def __init__(s,ctx,previous,**kwargs):s.ctx=ctx
            def path(s,obs,inputs):return fact(obs)
            def __call__(s,*,phase,before,after,inputs):
                return s.path(after,inputs) if phase=='released_and_retreat_path' else visual(s.ctx,phase,before,after)
        self.patch('manipulation_checks.RetreatChecks',Retreat)
        def stack_geometry(ctx,obs,scene,origin,target,quaternion,**kwargs):
            # Keep the center/height calculation real; collision geometry is stubbed.
            expected=stack_checks.stack_target(scene,origin)
            self.assertLess(np.linalg.norm(np.array(target)-expected),.003)
            return {'samples':[{'flange':ctx.fk(obs['measured14'][:6])['position']}],
                    'end_joints':ctx.encode(target,quaternion)}
        self.patch('stack_checks.check_stack_geometry',stack_geometry)
        def base_plan(ctx,obs,scene,origin,quaternion,**kwargs):
            red=scene['blocks']['red']['top_center']['surface_position_m'];z=ctx.fk(obs['measured14'][:6])['position'][2]
            high=[red[0],red[1]-.065,z];plane=scene['table']['z_ax_by_c']
            low=[*high[:2],high[0]*plane[0]+high[1]*plane[1]+plane[2]+origin['flange_to_bottom_m']]
            return {'ok':True,'selected':{'transfer':high,'retreat':high,'descent':low}}
        self.patch('placement_plan.side_by_side',base_plan)
        self.patch('lift_plan.scene_geometry',lambda *a:None)
        self.patch('lift_plan.finger_locals',lambda *a: [np.zeros((8,3))])
        def planned_base(ctx,start,scene,origin,quaternion,local,**kwargs):
            return base_plan(ctx,{'measured14':start},scene,origin,quaternion,**kwargs)
        self.patch('placement_plan.plan_side_by_side',planned_base)
        def planned_loaded(ctx,start,target,q,scene,origin,local,**kwargs):
            return {'ok':True,'end_joints':ctx.encode(target,q)}
        self.patch('lift_plan.loaded_segment',planned_loaded)
        self.patch('stack_checks.planned_stack_geometry',lambda ctx,start,scene,origin,target,q,local,**kwargs:
            {'end_joints':ctx.encode(target,q)})


    def patch(self,target,replacement):
        p=patch(target,replacement);p.start();self.addCleanup(p.stop)

    def run_main(self,ctx=None,name='run'):
        return c.main(ctx or self.f,output_dir=Path(self.temp.name)/name,optimize_trajectory=False,group_empty_entry=False)

    def records(self,report):
        return {s['label']:json.loads(Path(s['report']).read_text()) for s in report['steps']}

    def test_complete_real_primitive_chain_same_ctx_and_provenance(self):
        r=self.run_main();self.assertTrue(r['ok'],r.get('why'))
        self.assertEqual(r['status'],'complete');self.assertFalse(r['held_state']['uncertain'])
        self.assertTrue(self.seen_contexts);self.assertTrue(all(ctx is self.f for ctx in self.seen_contexts))
        rows=self.records(r)
        motions=[s['label'] for s in r['steps'] if s['motion_dispatch_count']]
        self.assertEqual(motions,['open-empty','orange-transition-1','orange-transition-2','orange-transition-3',
            'orange-approach','orange-close','orange-trial','orange-raise','orange-transfer','orange-lower-1',
            'orange-lower-2','orange-release','orange-retreat','inspection-1','inspection-2','green-pregrasp',
            'green-approach','green-close','green-trial','green-raise-1','green-transfer-1','green-raise-2',
            'green-transfer-3','green-stack-lower','green-stack-release','green-stack-retreat'])
        hover=rows['green-pregrasp']
        self.assertEqual(hover['inputs']['color'],'green');self.assertNotIn('color',hover['invocation_inputs'])
        support=rows['refresh-stack-support']['result']['support_source']
        self.assertEqual(support['report'],hover['report'])
        self.assertEqual(rows['green-approach']['inputs']['empty_history'],[hover['report']])
        self.assertEqual(json.loads(Path(r['report']).read_text()),r)

    def test_first_failed_release_stops_no_retry_or_retreat(self):
        self.f.negative_phase='green_supported_before_release'
        r=self.run_main();self.assertFalse(r['ok']);self.assertEqual(r['failed_step'],'green-stack-release')
        self.assertEqual(r['steps'][-1]['motion_dispatch_count'],0)
        self.assertNotIn('green-stack-retreat',self.records(r));self.assertEqual(self.f.q[6],c.GRASP_GAP_M)
        self.assertTrue(r['held_state']['uncertain']);self.assertIn('preserve the dispute',r['next_recovery_boundary'])
        before=len(self.f.calls)
        with self.assertRaisesRegex(RuntimeError,'main already entered'):self.run_main(name='not-a-retry')
        self.assertEqual(len(self.f.calls),before)

    def test_ambiguous_dispatch_saved_once_and_no_replay(self):
        self.f.fail_motion=lambda tool,args,ctx:tool=='piper_move_joints'
        r=self.run_main();self.assertFalse(r['ok']);self.assertEqual(r['failed_step'],'orange-transition-1')
        self.assertEqual(r['ambiguous_dispatch_count'],1)
        self.assertEqual(sum(t=='piper_move_joints' for t,a in self.f.calls),1)
        self.assertEqual(sum(t=='piper_hold' for t,a in self.f.calls),1)
        self.assertIn('Resolve dispatch',r['next_recovery_boundary']);self.assertTrue(r['held_state']['uncertain'])
        self.assertNotIn('orange-transition-2',self.records(r))

    def test_changed_layout_and_actual_pickup_relation_drive_targets(self):
        a=self.run_main();self.assertTrue(a['ok'],a.get('why'));ar=self.records(a)
        other=SceneContext(shift=(.025,.012,.006),pickup_error=.001)
        b=self.run_main(other,'changed-layout');self.assertTrue(b['ok'],b.get('why'));br=self.records(b)
        ao=ar['orange-transfer']['inputs']['waypoints'][0];bo=br['orange-transfer']['inputs']['waypoints'][0]
        np.testing.assert_allclose(np.array(bo)-ao,[.025,.012,.006],atol=1e-10)
        end=br['green-transfer-3']['inputs']['waypoints'][0]
        source=br['green-raise-1']['result']['facts'][0]
        expected=stack_checks.stack_target(source['scene'],source['payload_origin'])
        np.testing.assert_allclose(end[:2],expected[:2],atol=1e-10)
        self.assertNotAlmostEqual(end[0],other.scene['blocks']['red']['top_center']['surface_position_m'][0])
        self.assertEqual(br['green-approach']['inputs']['waypoints'][-1][2],other.scene['blocks']['green']['top_center']['surface_position_m'][2]+glue.INSERTION_HEIGHT)

    def test_first_green_transfer_preserves_rehearsed_orange_relative_detour(self):
        for name, f in [('base', self.f), ('shifted', SceneContext(shift=(.025,.012,.006)))]:
            r=self.run_main(f,name);self.assertTrue(r['ok'],r.get('why'))
            rows=self.records(r)
            fact=rows['green-raise-1']['result']['facts'][0]
            orange_y=fact['scene']['blocks']['orange']['top_center']['surface_position_m'][1]
            raised=rows['green-raise-1']['inputs']['waypoints'][-1]
            first=rows['green-transfer-1']['inputs']['waypoints'][0]
            self.assertAlmostEqual(first[0],raised[0])
            self.assertAlmostEqual(first[2],raised[2])
            self.assertAlmostEqual(first[1],orange_y-.105)

    def test_initial_held_jaw_or_non_neutral_refuses_before_opening(self):
        for name,index,value in [('held',6,.0513),('not-neutral',0,.20),('open-prior-task',6,.069)]:
            f=SceneContext();f.q[index]=value;r=self.run_main(f,name)
            self.assertFalse(r['ok']);self.assertEqual(r['failed_step'],'initial-state')
            self.assertFalse(any(t in rehearse_step.MOTIONS for t,a in f.calls))

    def test_counter_totals_match_actual_logged_calls_and_elapsed_time(self):
        self.f.model_delay=.001
        start=time.monotonic();r=self.run_main();duration=time.monotonic()-start
        self.assertTrue(r['ok'],r.get('why'))
        counts=Counter(t for t,a in self.f.calls);self.assertEqual(r['tool_counts'],dict(counts))
        self.assertEqual(r['primary_model_calls'],0)
        self.assertEqual(r['calls'],len(self.f.calls));self.assertEqual(r['auxiliary_visual_calls'],counts['gemini_generate'])
        self.assertEqual(r['auxiliary_visual_calls'],6) # synthetic full-chain visual boundaries
        self.assertEqual(r['motion_dispatch_count'],sum(counts[t] for t in rehearse_step.MOTIONS))
        self.assertEqual(r['motion_receipt_count'],r['motion_dispatch_count'])
        self.assertEqual(r['ambiguous_dispatch_count'],0)
        self.assertGreater(r['elapsed_s'],.006);self.assertLessEqual(r['elapsed_s'],duration)
        self.assertGreater(r['tool_s'],.006)
        logged=Counter()
        for s in r['steps']:
            for line in Path(s['report']).with_suffix('.tools.jsonl').read_text().splitlines():logged[json.loads(line)['tool']]+=1
        self.assertEqual(logged,counts)

    def test_existing_run_directory_never_replays(self):
        target=Path(self.temp.name)/'interrupted';target.mkdir();(target/'REPORT.json').write_text('{"status":"in_progress"}')
        with self.assertRaises(FileExistsError):self.run_main(name='interrupted')
        self.assertEqual(self.f.calls,[])

    def test_planning_transport_error_does_not_try_more_candidates(self):
        original=self.f.value
        def value(tool,**kw):
            if tool=='piper_solve_ik':
                self.f.calls.append((tool,kw));raise RuntimeError('unknown read-only transport error')
            return original(tool,**kw)
        self.f.value=value
        r=self.run_main();self.assertEqual(r['failed_step'],'plan-orange-entry')
        self.assertEqual(sum(t=='piper_solve_ik' for t,a in self.f.calls),1)
        self.assertFalse(any(t=='piper_move_joints' for t,a in self.f.calls))

    def test_completed_trial_with_missing_image_stops_without_reclosure(self):
        self.f.omit_observation=lambda tool,args,ctx:tool=='piper_move_pose' and ctx.q[6]==c.GRASP_GAP_M
        r=self.run_main();self.assertEqual(r['failed_step'],'orange-trial')
        row=r['steps'][-1];self.assertTrue(row['action_completed'])
        self.assertEqual(row['motion_receipt_count'],1);self.assertEqual(r['ambiguous_dispatch_count'],0)
        self.assertFalse(any(t=='piper_hold' for t,a in self.f.calls))
        self.assertNotIn('orange-raise',self.records(r));self.assertTrue(r['held_state']['uncertain'])
        self.assertIn('never replay',r['next_recovery_boundary'])

    def test_first_read_only_ik_refusal_can_select_distinct_bounded_candidate(self):
        original=self.f.value;attempts=[]
        def value(tool,**kw):
            if tool=='piper_solve_ik':
                attempts.append(kw)
                if len(attempts)==1:
                    self.f.calls.append((tool,kw))
                    return {'ok':False,'why':'synthetic planning-only refusal'}
            return original(tool,**kw)
        self.f.value=value
        r=self.run_main();self.assertTrue(r['ok'],r.get('why'))
        rows=self.records(r)['plan-orange-entry']['result']['candidates']
        self.assertEqual(len(rows),2);self.assertFalse(rows[0]['ok']);self.assertTrue(rows[1]['ok'])
        self.assertNotEqual(attempts[0]['quaternion_xyzw'],attempts[1]['quaternion_xyzw'])

    def test_dispatch_interruption_records_partial_jaw_and_never_releases(self):
        original=self.f.value;closures=[]
        def value(tool,**kw):
            if tool=='piper_gripper' and kw['width_m']<self.f.q[6]:
                closures.append(kw)
                if len(closures)==2:
                    self.f.calls.append((tool,kw));raise KeyboardInterrupt('synthetic interrupt after dispatch')
            return original(tool,**kw)
        self.f.value=value
        r=self.run_main();self.assertEqual(r['failed_step'],'orange-close')
        self.assertEqual(r['ambiguous_dispatch_count'],1);self.assertTrue(r['held_state']['uncertain'])
        self.assertTrue(r['control_uncertain']);self.assertEqual(len(closures),2)
        self.assertNotIn('orange-trial',self.records(r));self.assertLess(self.f.q[6],c.OPEN_GAP_M)

    def test_current_opening_evidence_failure_cannot_be_overridden_by_main(self):
        with patch('scene_checks.empty_opening',return_value={'ok':None,'why':'empty state ambiguous'}):
            r=self.run_main()
        self.assertEqual(r['failed_step'],'initial-state');self.assertTrue(r['held_state']['uncertain'])
        self.assertFalse(any(t in rehearse_step.MOTIONS for t,a in self.f.calls))

    def test_malformed_native_receipt_still_records_ambiguity(self):
        original=self.f.value
        def value(tool,**kw):
            if tool=='piper_move_joints':
                self.f.calls.append((tool,kw));return None
            return original(tool,**kw)
        self.f.value=value
        r=self.run_main();self.assertEqual(r['failed_step'],'orange-transition-1')
        self.assertEqual(r['ambiguous_dispatch_count'],1)
        self.assertTrue(r['counters_complete']);self.assertTrue(r['control_uncertain'])
        self.assertEqual(self.records(r)['orange-transition-1']['result']['receipts'][0]['result'],None)


if __name__=='__main__':unittest.main()
