from numeric_policy import OPEN_JAW_ERROR_M, LOCALIZATION_ERROR_M, DOWN_COS
"""Independent A+ main: whole-task preflight followed by bounded functions.

No Agent, no motion at import, no replay of sent/uncertain actions. A's main remains
unchanged. Planning predicts geometry; only real receipts/images verify outcomes.
"""
import copy,time,uuid
from pathlib import Path
import numpy as np
import controller as c
from assembly_plan import Planner,OPEN,CLOSED,VERSION,xyz,table,box_corners,block_bounds,digest,NoPlan,measured_limits
from scene_checks import same_state,scene_control,scene_geometry,PerceptionUnknown
from observations import identity
import task_spec as TS
from rehearse_step import save,run_one


def finger_models(ctx,obs):
    T=np.asarray(ctx.value('piper_fk',joints=obs['measured14'][:6])['T_base_flange'])
    g=obs['finger_geometry']
    if not g.get('geometry_identity') or g.get('jaw_motion_model')!='linear_prismatic_full_gap_v1':
        raise PerceptionUnknown('Missing installed finger geometry identity')
    if abs(g['measured_full_gap_m']-obs['measured14'][6])>.0005:raise PerceptionUnknown('Finger gap binding differs')
    models=[]
    for gap in (OPEN,CLOSED):
        rows=[]
        for row in g['fingers']:
            p=np.asarray(row['cad_corners_base_m'])+np.asarray(row['translation_per_full_gap_base'])*(gap-obs['measured14'][6])
            rows.append((p-T[:3,3])@T[:3,:3])
        if len(rows)!=2:raise PerceptionUnknown('Both installed fingers required')
        models.append(np.concatenate(rows))
    return models


def latest_control_feedback(robot, *, now_ns=None):
    """Use the owner's current CAN state, not the delayed bridge status cache."""
    measured=robot.get('measured_state',{})
    stamp=measured.get('feedback_metadata',{}).get('source_timestamp_ns')
    now=time.time_ns() if now_ns is None else now_ns
    if measured.get('available') is not True or type(stamp) is not int or not 0<=now-stamp<=300_000_000:
        raise RuntimeError('Current owner feedback unavailable/stale; do not fall back to cached bridge state')
    q=measured.get('values');c._vector(q,14)
    generation=robot.get('owner_status',{}).get('generation')
    if type(generation) is not int or generation<0:raise RuntimeError('Current owner feedback identity missing')
    return {'q':list(q),'timestamp_ns':stamp,'generation':generation,'source':'owner_measured_state'}


class Execution:
    def __init__(self,ctx,planner,plan,robot,obs):
        self.ctx=ctx;self.planner=planner;self.plan=plan;self.robot=robot
        self.expected=list(obs['measured14']);self.initial=list(self.expected)
        self.scene=copy.deepcopy(plan['source_scene']);self.previous=None;self.obs=obs
        self.payload=None;self.origin=None;self.close_obs=None;self.support=None;self.held=None
        self.completed=[];self.release=None
    def guard(self):
        state=self.ctx.value('status');c._budget(state,c.DISPATCH_ALLOWANCE_S)
        now=state['robot_status'];b=now.get('bridge',{});f=latest_control_feedback(now)
        if (state.get('control_uncertain') is not False or now.get('motion_ready') is False
            or b.get('active_request') or b.get('phase') not in ('IDLE','DONE')):raise RuntimeError('A+ control not idle/ready/certain')
        if identity(now)!=identity(self.robot) or identity(now) is None:raise RuntimeError('A+ owner/calibration/geometry changed')
        stamp=f.get('timestamp_ns')
        if type(stamp) is not int or not 0<=time.time_ns()-stamp<=300_000_000 or f.get('generation')!=now['owner_status']['generation']:
            raise RuntimeError('A+ feedback not current')
        q=f.get('q')
        measured_limits(q)
        if not same_state(q,self.expected):raise RuntimeError('A+ state differs from completed endpoint')
        fixed=list(q);fixed[:7]=self.initial[:7]
        if not same_state(fixed,self.initial):raise RuntimeError('A+ fixed right-side state changed')
        if self.previous and (now.get('last_motion') or {}).get('owner_request_id')!=self.previous['owner_request_id']:
            raise RuntimeError('A+ intervening motion invalidated plan')
        scene_control(self.scene,now)
        return now,list(q)
    def observe(self):
        robot,obs=c._start(self.ctx,motion=False,force_observe=True)
        if identity(robot)!=identity(self.robot) or not same_state(obs['measured14'],self.expected):raise RuntimeError('State changed during observation')
        scene_geometry(self.scene,obs);self.obs=obs;return obs
    def receipt(self,report,start,target):
        r=report['receipts'][-1]['result']
        if r.get('owner_generation')!=self.robot['owner_status']['generation'] or not r.get('owner_request_id'):
            raise RuntimeError('Receipt owner identity missing/changed')
        if not same_state(r.get('anchor_validation',{}).get('measured14'),start):raise RuntimeError('Receipt anchor differs from checked path')
        actual=c._vector(r.get('final14'),14)
        # Jaw contact feedback has its established48..54mm grasp interval;
        # do not equate the50mm command with measured pad compression.
        if max(abs(a-b) for a,b in zip(actual[:6],target[:6]))>.01:raise RuntimeError('Arrival differs from target')
        fixed=actual.copy();fixed[:7]=start[:7]
        if not same_state(fixed,start):raise RuntimeError('Right side changed during action')
        self.previous=r;self.expected=actual
        if isinstance(r.get('observation'),dict):self.obs=r['observation']
        else:self.obs=None
        return actual
    def run(self,action,*,last_segment=True):
        color=action.get('color');stage=action['stage'];kind=action['kind']
        def operation(report):
            now,start=self.guard();report['start_status']={'robot_status':now}
            if max(abs(a-b) for a,b in zip(start[:6],action['start14'][:6]))>.01:raise NoPlan('Plan start joints differ; replan read-only before another action')
            report['planned_motion_count']=1
            report['plan_reference']={'version':VERSION,'source_digest':self.plan['source_digest'],'stage':stage}
            if kind=='jaw':
                target=start.copy();target[6]=action['width_m']
                opening=color is None;closing=action['width_m']<start[6]
                if closing:
                    # Entry's qualitative evidence may be reused only at the same
                    # completed endpoint; no duplicate model call before closure.
                    from qualitative import alignment_evidence
                    obs=self.obs if (self.obs and 0<=time.time_ns()-self.obs['captured_at_ns']<2_000_000_000
                        and same_state(self.obs.get('measured14'),start)
                        and self.obs.get('state_binding',{}).get('owner_generation')==now['owner_status']['generation']) else self.observe()
                    report['facts'].append(alignment_evidence(self.ctx,obs,color,scene=self.scene,reference14=action['start14']))
                    from history_scene import reject_visible_displacement
                    reject_visible_displacement(self.scene,obs,associated_colors=(color,))
                elif not opening:
                    if not self.support or self.support.get('ok') is not True or self.held!=color:raise PerceptionUnknown('Release lacks verified lowering/support endpoint')
                    binding=self.support.get('completed_endpoint',{})
                    reused=(0<=time.monotonic()-self.support['verified_monotonic']<=30
                        and binding.get('color')==color
                        and binding.get('owner_identity')==identity(now)
                        and binding.get('request_id')==(now.get('last_motion') or {}).get('owner_request_id')
                        and binding.get('request_id') is not None
                        and same_state(binding.get('measured14'),start))
                    if not reused:
                        obs=self.observe();self.support=self.placement(obs,color)
                    report['facts'].append({**self.support,'reused_support':reused})
                numeric_scene=self.scene
                if not closing and not opening:
                    numeric_scene=copy.deepcopy(self.scene)
                    numeric_scene['blocks'][color]['top_center']['surface_position_m']=self.support['estimated_object_top_m']
                checked_target=target.copy()
                if closing:checked_target[6]=.048
                report['path_check']=self.planner.check(start,checked_target,numeric_scene,active=color,contact=not opening,orientation_reference14=action['start14'] if not opening else None)
                report['jaw_profile']='speed25'
                args={'width_m':target[6],'direction':'close' if closing else 'open','profile':'speed25'}
                if closing:args['contact_range_m']=[.048,.054]
                after=c._move(self.ctx,'piper_gripper_stroke',args,report,observe_after=closing or (opening and stage!='empty-open'))
                actual=self.receipt(report,start,target)
                if closing:
                    if report['receipts'][-1]['result'].get('contact_range_m')!=[.048,.054]:raise RuntimeError('Contact completion contract missing from native receipt')
                    if not .048<=actual[6]<=.054:raise PerceptionUnknown('Grasp feedback outside rehearsed48..54mm interval')
                    T=np.asarray(self.ctx.value('piper_fk',joints=actual[:6])['T_base_flange']);lo,hi=block_bounds(self.scene,color)
                    self.payload={'corners_flange':((box_corners(lo,hi)-T[:3,3])@T[:3,:3]).tolist(),'color':color}
                    self.origin={'color':color,'grasp_flange':T[:3,3].tolist(),'flange_to_bottom_m':float(T[2,3]-table(self.scene,xyz(self.scene,color))),'half_xy_m':.035,'object_top_flange_m':((xyz(self.scene,color)-T[:3,3])@T[:3,:3]).tolist(),'corners_flange_m':self.payload['corners_flange']}
                    self.close_obs=after;self.held=color;self.support=None
                else:
                    if actual[6]<.065-OPEN_JAW_ERROR_M:raise PerceptionUnknown('Release/open feedback is not open')
                    if not opening:
                        self.scene=copy.deepcopy(numeric_scene)
                        point=self.scene['blocks'][color]['top_center']
                        point.update(measurement_kind='action_derived',action_frame=after.get('frame_id'),
                            release_request_id=self.previous['owner_request_id'],support_frame=self.support['frame_id'],
                            meaning='Measured supported placement plus completed release receipt; not new depth or a release image')
                        self.release={'color':color,'receipt':report['receipts'][-1]['result'],'support':self.support}
                        self.held=None;self.payload=None
                        if action.get('terminal_release_hold'):
                            if color!=TS.FINAL_COLOR:raise NoPlan('Only the block placed last may finish without retreat')
                            # Check the actual released pose and gap, not just
                            # the nominal offline endpoint. This is pure checking.
                            report['terminal_hold_check']=self.planner.check(actual,actual,self.scene,active=color,contact=True)
                            report['facts'].append(dict(action['terminal_release_hold'],release_request_id=self.previous['owner_request_id'],verified_release=True))
                return after
            if stage.endswith('-retreat'):
                release=self.release or {};receipt=release.get('receipt',{})
                follows_release=(self.previous or {}).get('owner_request_id')==receipt.get('owner_request_id')
                follows_retreat=bool(self.completed and self.completed[-1]==stage)
                if (release.get('color')!=color or self.held is not None or self.payload is not None
                    or not all(receipt.get(k) is True for k in ('executed','measured_arrival','measured_settled'))
                    or receipt.get('owner_generation')!=now['owner_status']['generation']
                    or start[6]<.065-OPEN_JAW_ERROR_M or receipt.get('final14',[0.]*14)[6]<.065-OPEN_JAW_ERROR_M
                    or not (follows_release or follows_retreat)):
                    raise NoPlan('Retreat requires this object completed open release and contiguous checked withdrawal')
            target=list(action['target14']);target[6:]=start[6:]
            if self.held!= (color if action.get('payload') else None):raise RuntimeError('A+ payload state does not match planned phase')
            if action.get('pickup'):
                from payload_replan import bind_pickup
                bound,binding=bind_pickup(self,[action],start)
                report['pickup_binding']=binding;report['planned_motion_count']=len(bound)
                for i,part in enumerate(bound):
                    actual_start=start if i==0 else self.guard()[1]
                    target=list(part['target14']);target[6:]=actual_start[6:]
                    report['path_check']=self.planner.check(actual_start,target,self.scene,active=color,payload=self.payload,pickup=True,support=action.get('support'))
                    after=c._move(self.ctx,'piper_move_joints',{'joints':target[:6]},report,observe_after=False)
                    self.receipt(report,actual_start,target)
                return after
            options=dict(active=color,payload=self.payload,contact=action.get('contact',False),support=action.get('support'),withdraw=action.get('withdraw',False),assembly_neighbor=action.get('assembly_neighbor'),pickup=action.get('pickup',False))
            can_adjust=self.payload is not None and not options['contact'] and not options['pickup'] and stage.endswith(('-raise','-transfer'))
            can_retreat_adjust=stage.endswith('-retreat') and options['withdraw'] and self.payload is None
            correction=None;proposal=None
            try:
                from assembly_batch import eligible,prepare_preview,dispatch_reviewed_curve
                if action['kind']=='arm':
                    proposal,start=prepare_preview(self,[action],start,report)
                    target=proposal['target14'];check=proposal['check']
                    report['single_certified_curve']={'duration_s':proposal['duration_s'],'samples':proposal['samples']}
                else:check=self.planner.check(start,target,self.scene,**options)
                gap=check.get('min_payload_neighbor_m')
                # Full path already passed the common20mm minimum. A
                # missing preferred3mm reserve is not a reason to force a lift.
                from assembly_plan import payload_clearance_fact
                if can_adjust:report['payload_clearance']=payload_clearance_fact(gap)
                needs_adjustment=False
            except NoPlan as refusal:
                if can_retreat_adjust:report['retreat_original_refusal']=str(refusal)
                if not (can_adjust or can_retreat_adjust):raise
                needs_adjustment=True;check=None
            if needs_adjustment:
                if can_retreat_adjust:
                    # Only an unexecuted withdrawal after the verified release.
                    # Existing bounded route search tries direct Cartesian
                    # subdivisions; every leg is checked before any dispatch.
                    goal=self.planner.fk(target)
                    parts=copy.deepcopy(self.planner.route(start,goal['position'],goal['quaternion_xyzw'],self.scene,**options))
                    parts[-1]['target14']=list(target)
                    parts[-1]['check']=self.planner.check(parts[-1]['start14'],target,self.scene,**options)
                    correction={'parts':parts,'strategy':'rebind_released_retreat',
                                'rejoins_original_endpoint':True,'native_collision_unchanged':True}
                else:
                    from payload_replan import corrected_leg
                    correction=corrected_leg(self.planner,start,target,self.scene,active=color,payload=self.payload)
                correction_key='retreat_correction' if can_retreat_adjust else 'payload_correction'
                report[correction_key]={k:v for k,v in correction.items() if k!='parts'}
                report[correction_key]['waypoints14']=[x['target14'] for x in correction['parts']]
                parts=correction['parts']
            else:parts=[{'start14':start,'target14':target,'check':check}]
            report['planned_motion_count']=len(parts)
            observe=last_segment and stage.endswith(('-trial','-lower','-retreat'))
            for i,part in enumerate(parts):
                if i==0 and correction is None:actual_start=start
                else:now,actual_start=self.guard()
                if not same_state(actual_start,part['start14']):raise NoPlan('Correction start changed before dispatch')
                target=list(part['target14']);target[6:]=actual_start[6:]
                # Recheck actual endpoints after every receipt. Never catch or
                # retry an ambiguous/dispatched action, nor replay a completed leg.
                report['path_check']=check if i==0 and correction is None else self.planner.check(actual_start,target,self.scene,**options)
                report.setdefault('executed_leg_checks',[]).append({'start14':actual_start,'target14':target,'min_payload_neighbor_m':report['path_check'].get('min_payload_neighbor_m')})
                if proposal is not None and correction is None:
                    after,actual_start,proposal=dispatch_reviewed_curve(self,[action],proposal,actual_start,report,observe_after=observe and i==len(parts)-1)
                    target=proposal['target14']
                    report['single_certified_curve']={'duration_s':proposal['duration_s'],'samples':proposal['samples']}
                else:
                    after=c._move(self.ctx,'piper_move_joints',{'joints':target[:6]},report,observe_after=observe and i==len(parts)-1)
                self.receipt(report,actual_start,target)
            if observe:
                if stage.endswith('-trial'):
                    from qualitative import retention_evidence
                    fact=retention_evidence(self.ctx,self.close_obs,after,color)
                elif stage.endswith('-lower'):
                    fact=self.placement(after,color);self.support=fact
                else:
                    from ordinary_checks import released_endpoint
                    fact=released_endpoint(self.ctx,after,self.scene,color,self.release,report['receipts'][-1]['result'])
                report['facts'].append(fact)
            return after
        result=c._run(self.ctx,stage,operation)
        if result['ok']:self.completed.append(stage)
        return result
    def placement(self,obs,color):
        from ordinary_checks import placement
        fact=placement(self.ctx,'aplus_support',obs,self.scene,self.origin)
        if fact.get('ok') is not True:raise PerceptionUnknown(fact.get('why','Support outcome unresolved'))
        return dict(fact,verified_monotonic=time.monotonic(),completed_endpoint={
            'color':color,'measured14':list(obs['measured14']),
            'owner_identity':identity(self.robot),
            'request_id':(self.previous or {}).get('owner_request_id')})


def main(ctx,*,output_dir=None,allow_auxiliary_vision=False):
    """A+ entry, one Context, explicit phase reports, no fallback to A on failure."""
    if getattr(ctx,'_magnetic_blocks_composition_run',None):raise RuntimeError('Never replay partial main on same Context')
    ctx._blocks_continuity=True;ctx._blocks_batch_fk=True;ctx._blocks_local_fk=True;ctx._blocks_batch_ik=True;ctx.allow_auxiliary_vision=allow_auxiliary_vision
    root=(Path(output_dir) if output_dir else Path('output/composition/runs')/uuid.uuid4().hex).resolve();root.mkdir(parents=True,exist_ok=False)
    path=root/'REPORT.json';ctx._magnetic_blocks_composition_run=str(path);start=time.monotonic()
    report=dict(ok=False,status='in_progress',candidate_version=VERSION,entrypoint='assembly_controller.main',report=str(path),started_unix=time.time(),steps=[],active_step=None,failed_step=None,primary_model_calls=0,auxiliary_visual_calls=0,never_blindly_replay=True,automatic_recovery=False,held_state={'object':None,'uncertain':True},offline_qualified_only=True,counters_complete=True,control_uncertain=False)
    def persist():
        report.update(elapsed_s=time.monotonic()-start,auxiliary_visual_calls=sum(s['auxiliary_visual_calls'] for s in report['steps']),motion_dispatch_count=sum(s['motion_dispatch_count'] for s in report['steps']),motion_receipt_count=sum(s['motion_receipt_count'] for s in report['steps']))
        from timing import inter_motion_gaps
        report['inter_motion_gaps']=inter_motion_gaps(report['steps'])
        save(path,report)
    def step(label,fn):
        report['active_step']=label;out=root/f'{len(report["steps"])+1:03d}-{label}.json';report['active_step_report']=str(out);persist()
        rec=run_one(ctx,label,{},out,operation=fn);result=rec['result']
        row={k:rec[k] for k in ('wall_s','tool_s','motion_rpc_s','calls','tool_counts','auxiliary_visual_calls','motion_dispatch_count','motion_receipt_count','ambiguous_dispatch_count','timing','summary_report')}
        row.update(label=label,function=label,report=str(out),ok=result.get('ok') is True,why=result.get('why'),executed=result.get('executed'),action_completed=result.get('action_completed'));report['steps'].append(row)
        if result.get('ok') is not True or rec['ambiguous_dispatch_count']:
            report.update(status='partial',failed_step=label,why=result.get('why'),failure_category=result.get('failure_category'),control_uncertain=bool(rec['ambiguous_dispatch_count'] or result.get('control_uncertain')),next_recovery_boundary='Inspect last receipts and fresh endpoint; no automatic retry or reset');report['held_state']['uncertain']=bool(report['control_uncertain'] or result.get('executed') and not result.get('action_completed'));persist();raise NoPlan('Step stopped: '+label)
        persist();return result
    persist();execution=None
    try:
        import composition
        initial=step('initial-state',composition.initial_state);robot=initial['start_status']['robot_status'];obs=initial['measured'];scene=initial['facts'][0]['scene']
        def planning_progress(detail):
            report['preflight_progress']=detail;persist()
        ctx._blocks_preflight_event=planning_progress
        planner=Planner(ctx,scene,robot,*finger_models(ctx,obs));plan=None
        def preflight(ctx):
            def op(r):
                nonlocal plan
                try:
                    from assembly_parallel import plan_parallel
                    plan=plan_parallel(planner,obs['measured14'])
                finally:r['ik_search_attempts']=getattr(planner,'ik_attempts',[])
                save(root/'WHOLE_TASK_PLAN.json',plan)
                r['plan_file']=str(root/'WHOLE_TASK_PLAN.json');r['plan_sha256']=digest(plan)
                r['facts'].append({'ok':True,'why':'All picks, loaded paths, placements, releases and planned retreats preflighted; final yellow may use an explicitly checked release-in-place endpoint','goal_anchor':plan['goal_anchor'],'order':plan['order'],'predicted_only':True})
                return obs
            return c._run(ctx,'whole_task_preflight',op)
        step('whole-task-preflight',preflight)
        execution=Execution(ctx,planner,plan,robot,obs)
        # Planning time cannot silently turn its source image into current evidence.
        def rebound(ctx):
            def op(r):
                execution.guard();fresh=execution.observe()
                from history_scene import reject_visible_displacement
                reject_visible_displacement(execution.scene,fresh)
                r['facts'].append({'ok':True,'frame_id':fresh['frame_id'],'why':'Same owner/pose and no obvious scene displacement since initial3D measurement'})
                return fresh
            return c._run(ctx,'rebind_plan',op)
        step('rebind-plan',rebound)
        actions=[dict(kind='jaw',stage='empty-open',width_m=OPEN,start14=obs['measured14'])]
        actions += [a for m in plan['moves'] for a in m['actions']]
        from assembly_batch import groups,run as run_batch,green_yellow_transition
        grouped=groups(actions)
        for i,group in enumerate(grouped):
            action=group[0]
            last=i==len(grouped)-1 or grouped[i+1][0]['stage']!=action['stage']
            report['held_state']={'object':execution.held,'uncertain':False};persist()
            label='green-to-yellow-empty-transition' if green_yellow_transition(group) else ((action['color']+'-lift-transfer') if len({a['stage'] for a in group})>1 else action['stage'])
            step(label+'-'+str(i+1),lambda ctx,g=group,last=last:run_batch(execution,g) if len(g)>1 else execution.run(g[0],last_segment=last))
            report['held_state']={'object':execution.held,'uncertain':False}
        terminal=actions[-1].get('terminal_release_hold')
        if terminal and (execution.held is not None or (execution.release or {}).get('color')!=TS.FINAL_COLOR):
            raise NoPlan('Final release-in-place lacks successful release of the last block')
        report.update(ok=True,status='completed',active_step=None,
            completion_mode='released_in_place' if terminal else 'released_and_retreated',
            terminal_release_hold=terminal,
            why=TS.COLOR_NAMES_ZH[TS.FINAL_COLOR]+'已放置并释放；退开候选无解，保持最终张爪位置，未退开。' if terminal else 'A+ task functions completed with current receipts and qualitative endpoint evidence',final_scene=execution.scene)
    except NoPlan:
        pass
    except BaseException as exc:
        report.update(status='error',why=f'{type(exc).__name__}: {exc}',failed_step=report['active_step'],next_recovery_boundary='Inspect recorded active step; never replay main blindly')
    persist();return report
