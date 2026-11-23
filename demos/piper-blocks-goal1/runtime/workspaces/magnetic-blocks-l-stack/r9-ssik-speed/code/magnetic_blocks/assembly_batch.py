from numeric_policy import OPEN_JAW_ERROR_M, LOCALIZATION_ERROR_M, DOWN_COS
import task_spec as TS
"""Bounded cross-stage lift/transfer batching; existing native wire/progress/Stop contract.
No contact/closure/trial/release blending. Only explicit no-dispatch anchor refusals allow bounded repreview.
"""
import time,json
import numpy as np
from assembly_plan import NoPlan,boxes
from scene_checks import same_state
import controller as c


class PreviewRejected(NoPlan):
    """Explicit read-only tool refusal, never a dispatched command failure."""


class NativeBlendRefusal(PreviewRejected):
    """Native collision review of an undispatched blended curve failed."""


def is_horizon_refusal(exc):
    text=str(exc)
    return ('TRAJECTORY_HORIZON_EXCEEDED' in text
            or 'native600-sample horizon' in text.replace('native 600','native600'))


def preview_tool(ctx,**arguments):
    try:return ctx.value('piper_preview_waypoints',**arguments)
    except RuntimeError as exc:
        prefix='MCP tool failed: '
        if not str(exc).startswith(prefix):raise
        try:error=json.loads(str(exc)[len(prefix):])['error']
        except (ValueError,KeyError,TypeError):raise exc
        if (not isinstance(error,dict) or error.get('operation')!='piper_preview_waypoints'
            or error.get('type') not in ('RejectedBeforeDispatch','ValueError','TrajectoryHorizonExceeded')
            or error.get('stage')!='validation' or error.get('request_no_action_executed') is not True
            or error.get('episode_ended') is not False
            or error.get('executed') is not False or error.get('dispatch_started') is not False
            or error.get('no_action_executed') is not True or error.get('control_uncertain') is not False):raise
        reason=error.get('reason')
        if arguments.get('blend_indices') and (reason in ('Blended wire path rejected','Blended wire path leaves workspace','Blended wire leaves workspace')
                or (arguments.get('blend_scale')==0. and reason=='Whole wire path lacks collision/workspace clearance')):
            raise NativeBlendRefusal(reason) from exc
        if isinstance(reason,str) and reason.startswith('TRAJECTORY_HORIZON_EXCEEDED: Merged path exceeds native600-sample horizon'):
            raise PreviewRejected(reason) from exc
        raise


def check_small_anchor_connector(e,actions,proposal,start,current):
    """Retain the exact reviewed wire; certify only the changed initial connector.

    The bridge compiles from requested start14, then independently repeats native
    connector/state/dynamics checks. No regenerated unreviewed loaded curve.
    """
    if not same_state(current,start):raise NoPlan('Anchor changed outside original binding')
    # Preserve the reviewed fixed command wire; existing feedback tolerance is not command drift.
    if abs(current[6]-start[6])>.0005+1e-12 or not np.array_equal(np.asarray(current[7:],dtype='<f4'),np.asarray(start[7:],dtype='<f4')):
        raise NoPlan('Fixed jaw/right wire changed; full repreview required')
    first=proposal.get('connector_targets14',[])
    if len(first)!=2:raise NoPlan('Preview lacks connector samples')
    limits=e.robot['bridge']['limits'];v=limits['max_joint_velocity_rad_s']*limits['tracking_velocity_ratio'];a=limits['max_joint_acceleration_rad_s2']
    qs=np.asarray([current,current,*first],float)[:,:6]
    if np.max(abs(np.diff(qs,axis=0)))*30>v+1e-5 or np.max(abs(np.diff(qs,n=2,axis=0)))*900>a+1e-4:
        raise NoPlan('Changed initial connector dynamics require full repreview')
    action=actions[0]
    command_start=list(current);command_start[6]=first[0][6]
    check=e.planner.check(command_start,first[0],e.scene,active=action['color'],payload=e.payload,
                          support=action.get('support'),pickup=action.get('pickup',False),
                          contact=action.get('contact',False),withdraw=action.get('withdraw',False),
                          assembly_neighbor=action.get('assembly_neighbor'),fixed_jaw_feedback_m=current[6])
    return {'reused_exact_wire':True,'connector_checked':True,'max_joint_drift_rad':float(max(abs(np.asarray(current[:6])-start[:6]))),'check':check}


def empty_handoff(action):
    if not TS.EMPTY_HANDOFF:return False
    return (action.get('kind')=='arm' and action.get('payload') is None and not action.get('pickup')
            and action.get('stage') in ('green-retreat','yellow-hover')
            and action.get('color')==('green' if action.get('stage')=='green-retreat' else 'yellow')
            and (action.get('stage')=='green-retreat' or not (action.get('contact') or action.get('withdraw'))))


def green_yellow_transition(actions):
    return bool(TS.EMPTY_HANDOFF) and any(a.get('stage')=='green-retreat' for a in actions) and any(a.get('stage')=='yellow-hover' for a in actions)


def eligible(action):
    if empty_handoff(action) and action['stage']=='green-retreat':return True
    return (action.get('kind')=='arm' and not action.get('contact') and not action.get('withdraw')
            and (action.get('stage','').endswith(('-hover','-transfer')) or (action.get('pickup') and action.get('stage','').endswith('-raise'))))


def compatible(a,b):
    if empty_handoff(a) and empty_handoff(b) and a['stage']=='green-retreat':
        return (b['stage'] in ('green-retreat','yellow-hover')
                and same_state(a.get('target14'),b.get('start14'))
                and a.get('scene')==b.get('scene'))
    return (eligible(a) and eligible(b) and a.get('color')==b.get('color') and bool(a.get('payload'))==bool(b.get('payload'))
            and (a['stage']==b['stage'] or a['stage'].endswith('-raise') and b['stage'].endswith('-transfer')))

def groups(actions):
    out=[]
    for action in actions:
        if (out and compatible(out[-1][-1],action) and len(out[-1])<8):out[-1].append(action)
        else:out.append([action])
    return out


def preview(execution,actions,start,*,blend_scale=1.,soft_start=False):
    """Full explicit geometry on actual wire and midpoints, including payload."""
    e=execution;first=actions[0];color=first['color']
    if not 1<=len(actions)<=8 or first.get('kind')!='arm' or (len(actions)>1 and (not eligible(first) or any(not compatible(a,b) for a,b in zip(actions,actions[1:])))):raise NoPlan('Not a compatible arm path')
    if e.held!=(color if first.get('payload') else None):raise RuntimeError('Batch held-object state differs')
    reference=list(first['start14']);reference[6:]=start[6:]
    if not same_state(start,reference):raise NoPlan('Batch left joints differ from plan')
    # Jaws/right are bound to the verified current endpoint by guard(), not to
    # the pre-grasp command. Contact compression is not encoder drift.
    gap=start[6]
    if e.payload and not .048<=gap<=.054:raise NoPlan('Actual grasp gap outside rehearsed range')
    if not e.payload and not .065-OPEN_JAW_ERROR_M<=gap<=.0755:raise NoPlan('Empty batch requires confirmed open jaw')
    from payload_replan import bind_pickup
    actions,binding=bind_pickup(e,actions,start,prefer_preview=True)
    if len(actions)>8:raise NoPlan('Rebound pickup exceeds bounded batch; use checked individual legs')
    opts=dict(active=color,payload=e.payload,support=first.get('support'))
    handoff=bool(TS.EMPTY_HANDOFF) and any(a.get('stage')=='green-retreat' for a in actions)
    if len(actions)==1:opts.update(contact=first.get('contact',False),withdraw=first.get('withdraw',False),assembly_neighbor=first.get('assembly_neighbor'),single_leg_wire=True)
    points=[];cur=start
    for action in actions:
        goal=list(action['target14']);goal[6:]=start[6:]
        # The actual blended wire and midpoints are checked below. Checking a
        # different straight joint leg here duplicated native work; standalone
        # fallback still checks its own complete path before dispatch.
        points.append(goal);cur=goal
    env=boxes(e.scene,(color,) if color and (e.payload or first.get('contact')) else (),table_allowance=0. if e.payload or first.get('contact') or first.get('withdraw') else .032)
    if handoff:
        # One native certificate covers the entire actual curve, with all
        # placed objects present and the stricter free-flight table envelope.
        # Task contact/withdrawal semantics are still applied per phase below.
        env=boxes(e.scene,(),table_allowance=.032)
    # Only after a proven pre-dispatch refusal: an in-line lead-in with
    # the native minimum leg duration, starting at measured zero velocity.
    delta=np.asarray(points[0][:6])-np.asarray(start[:6])
    lead_in=soft_start and np.max(np.abs(delta))>1e-6
    if lead_in:
        fraction=min(.1,.005/float(np.max(np.abs(delta))))
        points.insert(0,[*(np.asarray(start[:6])+fraction*delta),*start[6:]])
    indices=list(range(len(points)-1))
    plan=preview_tool(e.ctx,start14=start,waypoints=[p[:6] for p in points],obstacles=env,blend_indices=indices,blend_scale=blend_scale)
    if not isinstance(plan,dict) or any(plan.get(key,False) is not False for key in ('dispatch_started','control_uncertain','episode_ended')):
        raise RuntimeError('Preview returned ambiguous control status')
    if plan.get('executed') is not False and not ('executed' not in plan and plan.get('hardware_execution_enabled') is False and plan.get('validation_scope')=='offline_only; native owner integration required'):
        raise RuntimeError('Preview lacks explicit no-execution evidence')
    wire=plan.get('targets14');native=plan.get('review',{})
    if not wire or not same_state(wire[-1],points[-1]):raise NoPlan('Preview endpoint missing/differs')
    boundaries=plan.get('waypoints',[])
    if lead_in:
        if len(boundaries)!=len(actions)+1:raise NoPlan('Recovery lead-in missing')
        boundaries=boundaries[1:]  # first task phase includes every lead-in sample
    if len(boundaries)!=len(actions):raise NoPlan('Wire lacks phase progress boundaries')
    checks=[];offset=0;anchor=list(start)
    for i,action in enumerate(actions):
        end=boundaries[i]['sample_end'] if i<len(actions)-1 else len(wire)
        if not offset<end<=len(wire):raise NoPlan('Invalid wire phase boundary')
        section=wire[offset:end];samples=[anchor]
        for a,b in zip([anchor,*section],section):
            samples.extend([((np.asarray(a)+np.asarray(b))*.5).tolist(),list(b)])
        if not np.allclose(np.asarray(samples)[:,6:],np.asarray(start)[6:],rtol=0,atol=1e-7):raise NoPlan('Preview changes jaw/right side')
        for sample in samples:sample[6:]=start[6:]
        phase_opts=opts
        if handoff:
            phase_opts=dict(active=action['color'],payload=None,support=action.get('support'),
                contact=action.get('contact',False),withdraw=action.get('withdraw',False),assembly_neighbor=action.get('assembly_neighbor'),
                released_empty_curve=action.get('stage')=='green-retreat' and len(actions)>1)
        try:
            checks.append(e.planner._check(anchor,samples[-1],e.scene,**phase_opts,pickup=action.get('pickup',False),reserve_m=.003 if e.payload else 0.,curve14=samples,native_result=native))
        except NoPlan as exc:
            raise BlendGeometryRefusal(str(exc)) from exc
        anchor=samples[-1];offset=end
    gaps=[x['min_payload_neighbor_m'] for x in checks if x.get('min_payload_neighbor_m') is not None]
    check={'segments':checks,'min_payload_neighbor_m':min(gaps) if gaps else None,'actual_wire_checked':True}

    return dict(connector_targets14=wire[:2],arguments={'start14':start,'waypoints':[p[:6] for p in points],'obstacles':env,'blend_indices':indices,'blend_scale':blend_scale},
                empty_transition=green_yellow_transition(actions),
                junction_speeds_rad_s=[{'after_stage':actions[i]['stage'],'before_stage':actions[i+1]['stage'],
                    'max_joint_speed_rad_s':float(np.max(np.abs((np.asarray(wire[min(len(wire)-1,b['sample_end'])][:6])-np.asarray(wire[max(0,b['sample_end']-2)][:6]))*15.)))}
                    for i,b in enumerate(boundaries[:-1])],
                check=check,pickup_binding=binding,duration_s=plan['duration_s'],samples=len(wire),waypoint_boundaries=plan['waypoints'],stages=[a['stage'] for a in actions],target14=points[-1])


class BlendGeometryRefusal(NoPlan):
    """Task envelope rejects a generated curve before any command is sent."""


MAX_ANCHOR_REPREVIEWS=2


def prepare_preview(e,actions,start,report,*,soft_start=False):
    """Recompile from fresh feedback at most twice; check the actual new wire."""
    def checked(anchor):
        attempts=report.setdefault('curve_attempts',[])
        for scale in ((1.,.35,0.) if len(actions)>1 else (1.,)):
            t=time.monotonic()
            try:proposal=preview(e,actions,anchor,blend_scale=scale,**({'soft_start':True} if soft_start else {}))
            except (PreviewRejected,BlendGeometryRefusal) as exc:
                attempts.append({'scale':scale,'ok':False,'reason':str(exc),'elapsed_s':time.monotonic()-t})
                if scale==0. or len(actions)==1:raise
            else:
                attempts.append({'scale':scale,'ok':True,'elapsed_s':time.monotonic()-t})
                return proposal
    for attempt in range(MAX_ANCHOR_REPREVIEWS+1):
        proposal=checked(start)
        _,current=e.guard()
        if np.array_equal(np.asarray(current,dtype='<f4'),np.asarray(start,dtype='<f4')):return proposal,start
        try:
            report['batch_anchor_connector']=check_small_anchor_connector(e,actions,proposal,start,current)
            return proposal,start
        except (NoPlan,ValueError,KeyError) as exc:
            report.setdefault('anchor_repreview_reasons',[]).append(str(exc))
            if attempt==MAX_ANCHOR_REPREVIEWS:raise
            start=current


def retryable_anchor_refusal(exc):
    error=c.no_dispatch_error(exc)
    return bool(error and error.get('operation')=='piper_move_waypoints'
        and error.get('episode_ended') is False
        and error.get('reason') in (
            'Initial connector exceeds acceleration limit; repreview before dispatch',
            'Initial connector exceeds velocity limit; repreview before dispatch',
            'Initial connector feedback exceeds anchor tolerance',
            'Multi-waypoint planning anchor drift','Multi-waypoint anchor changed','planning anchor drift'))


def dispatch_reviewed_curve(e,actions,proposal,start,report,*,observe_after=False):
    """Never retry a dispatched/ambiguous motion. Only repreview proven no-send."""
    for attempt in range(MAX_ANCHOR_REPREVIEWS+1):
        _,current=e.guard()
        if not np.array_equal(np.asarray(current,dtype='<f4'),np.asarray(start,dtype='<f4')):
            try:report['dispatch_anchor_connector']=check_small_anchor_connector(e,actions,proposal,start,current)
            except (NoPlan,ValueError,KeyError):
                proposal,start=prepare_preview(e,actions,current,report,**({'soft_start':True} if attempt else {}))
        report['dispatch_anchor_source']='owner_measured_state'
        before=len(report.get('receipts',[]))
        # A previous proven rejection must not mask uncertainty in this attempt.
        report['pre_dispatch_rejected']=False
        report.pop('rejected_motion',None)
        try:
            after=c._move(e.ctx,'piper_move_waypoints',proposal['arguments'],report,observe_after=observe_after)
        except c.MotionRejectedBeforeDispatch as exc:
            if not retryable_anchor_refusal(exc) or len(report.get('receipts',[]))!=before or attempt==MAX_ANCHOR_REPREVIEWS:raise
            report.setdefault('no_dispatch_anchor_retries',[]).append({'attempt':attempt+1,'error':dict(exc.error),'replayed_motion':False})
            # guard checks original completed receipt, unchanged owner, current
            # state and existing tolerances before rebuilding the entire path.
            _,start=e.guard()
            proposal,start=prepare_preview(e,actions,start,report,soft_start=True)
            continue
        report['pre_dispatch_rejected']=False
        report['final_curve']={k:proposal[k] for k in ('duration_s','samples','stages')}
        return after,start,proposal


def checked_partition(execution,actions,start,*,known_oversized=False):
    """Pure pre-dispatch partition; retain the largest recursively checked batches.
    Never raises the native horizon, skips a sample or dispatches a command.
    """
    e=execution
    if len(actions)==1:
        action=actions[0];goal=list(action['target14']);goal[6:]=start[6:]
        e.planner.check(start,goal,e.scene,active=action['color'],payload=e.payload,
                        support=action.get('support'),pickup=action.get('pickup',False),contact=action.get('contact',False),withdraw=action.get('withdraw',False),assembly_neighbor=action.get('assembly_neighbor'))
        return [actions],goal
    if not known_oversized:
        try:
            proposal=preview(e,actions,start)
            return [actions],proposal['target14']
        except PreviewRejected as exc:
            if not is_horizon_refusal(exc):raise
    middle=(len(actions)+1)//2
    left,anchor=checked_partition(e,actions[:middle],start)
    right,end=checked_partition(e,actions[middle:],anchor)
    return left+right,end


def check_loaded_partition(e,chunks,start):
    """Verify every proposed loaded chunk before allowing the first dispatch."""
    from payload_replan import bind_pickup
    anchor=list(start)
    checks=[]
    for chunk in chunks:
        if len(chunk)==1 and chunk[0].get('pickup'):
            bound,_=bind_pickup(e,chunk,anchor)
            groups_to_check=[[part] for part in bound]
        else:
            groups_to_check=[chunk]
        for group in groups_to_check:
            proposal=preview(e,group,anchor)
            checks.append({'stages':proposal['stages'],'min_payload_neighbor_m':proposal['check']['min_payload_neighbor_m']})
            anchor=proposal['target14']
    return checks


def require_released_green(execution,actions):
    if not TS.EMPTY_HANDOFF or actions[0].get('stage')!='green-retreat':return
    e=execution;release=e.release or {};receipt=release.get('receipt',{})
    if (release.get('color')!='green' or e.held is not None or e.payload is not None
        or not receipt.get('owner_request_id')
        or not ((e.previous or {}).get('owner_request_id')==receipt['owner_request_id']
                or bool(e.completed and e.completed[-1]=='green-retreat'))):
        raise NoPlan('Empty transition requires completed green opening and contiguous withdrawal')


def run(execution,actions):
    e=execution;stage=actions[0]['stage']
    def op(report):
        now,start=e.guard();report['start_status']={'robot_status':now}
        require_released_green(e,actions)
        t=time.monotonic()
        # Failure before dispatch may fall back to separately checked original
        # functions. A dispatch exception NEVER enters this fallback branch.
        try:
            proposal,start=prepare_preview(e,actions,start,report)
            report['batch_anchor_binding']={'planned14':actions[0]['start14'],'measured14':start,'jaw_from_completed_receipt':True,'joint_tolerance_rad':.01,'native_fresh_anchor_recheck':True}
        except (PreviewRejected,BlendGeometryRefusal) as exc:
            report['batch_fallback']=str(exc);report['planned_motion_count']=len(actions)
            chunks=[[a] for a in actions]
            if is_horizon_refusal(exc) and len(actions)>2:
                try:
                    chunks,_=checked_partition(e,actions,start,known_oversized=True)
                    report['batch_partition']={'chunk_lengths':[len(g) for g in chunks],
                        'all_chunks_checked_before_dispatch':True,'native_horizon_unchanged':True}
                except (PreviewRejected,BlendGeometryRefusal) as partition_error:
                    report['batch_partition_refusal']=str(partition_error)
            if e.payload is not None:
                report['loaded_split_checks']=check_loaded_partition(e,chunks,start)
            report['planned_motion_count']=len(chunks)
            report['batch_partition_preparation_s']=time.monotonic()-t
            report['batch_chunks']=[]
            # This loop is outside the preview/retry try block. An ambiguous
            # dispatch or failed chunk stops here and never replays earlier legs.
            for chunk in chunks:
                result=run(e,chunk) if len(chunk)>1 else e.run(chunk[0],last_segment=False)
                report['batch_chunks'].append({'segments':len(chunk),'ok':result.get('ok'),
                    'batch':result.get('batch'),'fallback':result.get('batch_fallback')})
                report['receipts'].extend(result.get('receipts',[]));report['executed']|=result.get('executed',False)
                report['dispatch_attempted']|=result.get('dispatch_attempted',False)
                report['action_completed']=result.get('action_completed',False)
                if not result['ok']:
                    report['control_uncertain']=result.get('control_uncertain',False)
                    raise NoPlan('Single-leg fallback stopped: '+result['why'])
            return result['measured']
        report['batch']={k:v for k,v in proposal.items() if k not in ('arguments','target14','check','connector_targets14')}
        report['batch'].update(preparation_s=time.monotonic()-t,blend_indices=proposal['arguments']['blend_indices'],original_segments=len(actions))
        report['path_check']=proposal['check'];report['planned_motion_count']=1
        after,start,proposal=dispatch_reviewed_curve(e,actions,proposal,start,report,observe_after=False)
        report['batch'].update({k:v for k,v in proposal.items() if k not in ('arguments','target14','check','connector_targets14')})
        report['path_check']=proposal['check']
        e.receipt(report,start,proposal['target14'])
        return after
    result=c._run(e.ctx,stage,op)
    if result['ok']:e.completed.append(stage)
    return result
