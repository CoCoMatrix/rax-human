"""R8 composed candidate; rehearsed primitives and an offline-checked main(ctx).

Units: metres/radians; poses control left_link6 in left_base_link.
An optional synchronous perception function supplies facts for the actual frames.
Missing facts return needs_review immediately. The Agent inspects reports between
calls; no function waits for an Agent approval file. Import never connects.
"""
import math
import json
import time
from perception import PerceptionUnknown

GRASP_GAP_M = 0.050  # R3 command; observed feedback ~0.0509--0.0513, not pad spacing.
OPEN_GAP_M = 0.069   # R3 green/base release; initial orange opening was 0.073.
JAW_STEP_M = 0.009   # <= live 0.010 tool limit, calculated from measured feedback.
TRIAL_LIFT_M = 0.010 # This block run only; never inherit the cup's 40 mm setting.
FUNCTION_ALLOWANCE_S = 120
DISPATCH_ALLOWANCE_S = 60
TRIAL_ALLOWANCE_S = 15  # candidate estimate, never replaces native finish reserve


class NeedsReview(Exception):
    def __init__(self, phase, before, after, inputs, answer=None):
        super().__init__(phase)
        self.review = {'phase': phase, 'before': before, 'after': after,
                       'inputs': inputs, 'assessment': answer}


class RejectedFact(RuntimeError):
    """Preserve a negative assessment without embedding entire frames in why."""
    def __init__(self, phase, before, after, inputs, answer):
        super().__init__(f'{phase}: {answer.get("why", "Scene check rejected")}')
        self.review = {'phase': phase, 'before': before, 'after': after,
                       'inputs': inputs, 'assessment': answer}


class MotionRejectedBeforeDispatch(RuntimeError):
    def __init__(self,error):
        self.error=error
        super().__init__(error.get('reason','Rejected before dispatch'))


def no_dispatch_error(exc):
    """Trust only explicit native no-dispatch evidence, never message keywords.

    Context currently wraps the MCP JSON in RuntimeError. Preserve its structured
    contract here without mistaking a timeout or a missing receipt for rejection.
    """
    payload=getattr(exc,'error',None)
    if not isinstance(payload,dict):
        text=str(exc);prefix='MCP tool failed: '
        if not text.startswith(prefix):return None
        try:payload=json.loads(text[len(prefix):]).get('error')
        except (ValueError,TypeError,AttributeError):return None
    if not isinstance(payload,dict):return None
    if (payload.get('type')!='RejectedBeforeDispatch' or payload.get('stage')!='validation'
        or payload.get('control_uncertain') is not False or payload.get('no_action_executed') is not True
        or payload.get('executed') is not False or payload.get('dispatch_started') is not False):return None
    return dict(payload)


def _vector(v, n):
    if len(v) != n or not all(type(x) in (int, float) and math.isfinite(x) for x in v):
        raise ValueError(f'Expected {n} finite values')
    return list(v)


def _fact(assess, phase, before, after, inputs):
    if assess is None:
        raise NeedsReview(phase, before, after, inputs)
    answer = assess(phase=phase, before=before, after=after, inputs=inputs)
    if not isinstance(answer, dict) or answer.get('ok') is None:
        raise NeedsReview(phase, before, after, inputs, answer)
    if answer.get('ok') is not True:
        raise RejectedFact(phase, before, after, inputs, answer)
    if answer.get('frame_id') != after.get('frame_id') or not answer.get('why'):
        raise RuntimeError(f'{phase}: assessment must cite the supplied frame and reason')
    return answer


def _budget(state, allowance_s=0):
    budget = state.get('wall_budget', {})
    remaining = budget.get('remaining_seconds')
    reserve = budget.get('finish_reserve_seconds')
    if (type(remaining) not in (int, float) or not math.isfinite(remaining)
        or type(reserve) not in (int, float) or not math.isfinite(reserve)
        or remaining < 0 or reserve < 0):
        raise RuntimeError('Missing or malformed measured wall budget; no dispatch')
    if budget.get('phase') != 'active' or remaining <= reserve + allowance_s:
        raise RuntimeError('Native finish reserve: stop starting task functions/motions')


def _start(ctx, *, require_depth=False, motion=True, allowance_s=FUNCTION_ALLOWANCE_S, report=None, force_observe=False):
    state = ctx.value('status')
    if report is not None:report['start_status']=state
    _budget(state, allowance_s if motion else 0)
    if state.get('control_uncertain') is not False:
        raise RuntimeError('Unknown control state; resolve with native status/hold before resuming')
    robot = state['robot_status']
    if robot.get('motion_ready') is False:
        raise RuntimeError('Robot is not motion ready')
    if robot.get('controlled_link') != 'left_link6' or robot.get('frame') != 'left_base_link':
        raise RuntimeError('Unexpected pose contract')
    bridge = robot.get('bridge', {})
    if bridge.get('active_request') or bridge.get('phase', 'IDLE') not in ('IDLE','DONE'):
        raise RuntimeError('Inspect native receipt/state for non-idle bridge before another function')
    from observations import observe
    obs, source = observe(ctx,robot,require_depth=require_depth,force=force_observe)
    if report is not None:report['observation_source']=source
    if report is not None:
        report['measured']=obs
        report['initial_observation']=obs
    _vector(obs['measured14'], 14)
    return robot, obs


def _poses(robot, positions, quaternion):
    q = _vector(quaternion, 4)
    if abs(sum(x*x for x in q)-1) > 1e-5:
        raise ValueError('Quaternion must be normalized (xyzw)')
    points = [_vector(p, 3) for p in positions]
    if not points:
        raise ValueError('At least one waypoint is required')
    for p in points:
        if not all(a <= x <= b for x, a, b in zip(p, robot['workspace_min'], robot['workspace_max'])):
            raise ValueError('Waypoint outside live flange workspace')
    return points, q


def _move(ctx, tool, args, report, observe_after=True, allowance_s=DISPATCH_ALLOWANCE_S):
    state = ctx.value('status')
    report['last_dispatch_status']=state
    _budget(state, allowance_s)
    if state.get('control_uncertain') is not False:
        raise RuntimeError('Uncertain control before dispatch; inspect native receipt/state')
    robot=state.get('robot_status',{});bridge=robot.get('bridge',{})
    if robot.get('motion_ready') is False or bridge.get('active_request') or bridge.get('phase','IDLE') not in ('IDLE','DONE'):
        raise RuntimeError('Non-idle/not-ready bridge before dispatch; inspect native receipt/state')
    # Record attempted dispatch before calling: a transport error is ambiguous.
    report['dispatch_attempted'] = True
    report['action_completed'] = False
    t = time.monotonic()
    ctx._rax_observation=None
    try:
        r = ctx.value(tool, **args, observe_after=observe_after)
    except Exception as exc:
        error=no_dispatch_error(exc)
        if error is None:raise
        report['pre_dispatch_rejected']=True
        report['rejected_motion']={'tool':tool,'arguments':args,'error':error}
        raise MotionRejectedBeforeDispatch(error) from exc
    report['receipts'].append({'tool': tool, 'arguments': args,
                               'elapsed_s': time.monotonic()-t, 'result': r})
    report['executed'] |= r.get('executed') is True
    if not all(r.get(k) is True for k in ('executed', 'measured_arrival', 'measured_settled')):
        raise RuntimeError('Motion receipt did not confirm execution, arrival and settling')
    report['action_completed'] = True
    report['measured'] = {'measured14': r['final14'], 'control_feedback_only': True}
    if not observe_after:
        return report['measured']
    if not isinstance(r.get('observation'), dict):
        raise NeedsReview('observe_completed_action', None, report['measured'],
                          {'observation_error': r.get('observation_error'), 'never_replay_motion': True})
    obs = r['observation']
    _vector(obs['measured14'], 14)
    report['measured'] = obs
    from observations import remember
    remember(ctx,obs,robot,request_id=r.get('owner_request_id'))
    return obs


def _jaw(ctx, obs, target, report):
    if getattr(ctx, '_blocks_continuity', False):
        current = obs['measured14'][6]
        report['jaw_stroke'] = {'start_m':current, 'target_m':target,
                               'fixed_arms':True, 'speed_unchanged':True}
        return _move(ctx, 'piper_gripper_stroke',
                     {'width_m':target, 'direction':'open' if target>current else 'close'}, report)
    # Bounded feedback-based stepping; cannot silently spin on a stale jaw value.
    for _ in range(12):
        current = obs['measured14'][6]
        command = current + max(-JAW_STEP_M, min(JAW_STEP_M, target-current))
        final_sent = abs(command-target) < 1e-10
        after = _move(ctx, 'piper_gripper', {'width_m': command}, report, observe_after=final_sent)
        # Native measured_arrival/settled are authoritative; no extra jaw tolerance.
        if final_sent:
            return after
        if abs(after['measured14'][6]-current) < 0.0001:
            raise RuntimeError('Jaw did not progress; do not repeat the closure blindly')
        obs = after
    raise RuntimeError('Jaw step budget exhausted')


def _run(ctx, stage, operation, next_function=None):
    t = time.monotonic()
    report = {'ok': False, 'stage': stage, 'why': '', 'executed': False,
              'dispatch_attempted': False, 'receipts': [], 'facts': [],
              'action_completed': False, 'needs_review': False,
              'next_function': next_function,
              'measured': None,
              'code_status': 'R8 supervised function candidate; qualification is per report, no unattended whole-task success'}
    try:
        report['measured'] = operation(report)
        report['ok'] = True
        report['why'] = 'This function completed; this does not imply whole-task success'
    except MotionRejectedBeforeDispatch as exc:
        report.update(status='planning_refused',failure_category='pre_dispatch_rejection',
                      why='Rejected before dispatch: '+str(exc),control_uncertain=False,
                      resume='Correct the rejected request and inspect current state; no automatic replay')
    except OSError as exc:
        # ConnectionError/TimeoutError are OSError subclasses, not missing RGBD.
        report['failure_category']='service_timeout' if isinstance(exc,TimeoutError) else 'service_connection' if isinstance(exc,ConnectionError) else 'io_error'
        report['status']='service_error'
        report['why']=f'{report["failure_category"]}: {type(exc).__name__}: {exc}'
        report['resume']='inspect service/Stop status; reconnect explicitly; never replay a dispatched action'
    except (NeedsReview, PerceptionUnknown, KeyError, TypeError) as exc:
        report['needs_review'] = True
        report['status'] = 'needs_review'
        detail=(exc.review.get('assessment') or {}).get('why') if isinstance(exc,NeedsReview) else None
        report['failure_category']=getattr(exc,'reason_category','scene_evidence')
        labels={'planning_or_geometry':'Planning/geometry refusal: ','native_path_collision':'Native collision/workspace refusal: ','ik_no_solution':'IK no solution: '}
        report['why'] = labels.get(report['failure_category'],'Missing scene evidence: ') + str(exc) + (': '+detail if detail else '')
        report['review'] = (exc.review if isinstance(exc,NeedsReview) else
                            {'phase':stage,'after':report.get('measured'),'reason':str(exc)})
        report['resume'] = ('resolve missing endpoint facts before next_function; never repeat completed motion'
                            if report['executed'] else 'supply a real synchronous check before calling this function')
        if report['failure_category'] in labels:
            report['needs_review']=False
            report['status']='planning_refused'
            report['resume']='inspect the reported IK/geometry constraint and replan before motion; extra image confirmation is not the remedy'
        # Known completed/settled movement is not a control fault. Do not add Hold
        # or re-execute it merely because the Agent must inspect the saved images.
    except Exception as exc:
        report['why'] = f'{type(exc).__name__}: {exc}'
        report['failure_category']=getattr(exc,'reason_category','cancelled' if type(exc).__name__ in ('Cancelled','CancelledError') else 'execution_error' if report['dispatch_attempted'] else 'program_error')
        if report['failure_category']=='planning_budget':report['status']='planning_timeout'
        if report['failure_category']=='cancelled':report['status']='cancelled'
        if isinstance(exc, RejectedFact):
            report['review'] = exc.review
            report['status'] = 'rejected_scene_fact'
        # Never release a potentially held block or replay a failed action.
        if report['dispatch_attempted'] and not (isinstance(exc,RejectedFact) and report['action_completed'] and stage in ('green_stack_lower','green_stack_release','green_stack_retreat')):
            try:
                report['hold'] = ctx.value('piper_hold')
            except Exception as stop_error:
                report['hold_error'] = str(stop_error)
                report['control_uncertain'] = True
    if not report['ok'] and report['dispatch_attempted'] and not report['action_completed'] and not report.get('pre_dispatch_rejected'):
        # Missing/malformed receipt after an attempted dispatch is control-uncertain,
        # unlike a missing image after confirmed arrival/settling.
        report['control_uncertain']=True
        if 'hold' not in report and 'hold_error' not in report:
            try:report['hold']=ctx.value('piper_hold')
            except Exception as exc:report['hold_error']=str(exc)
    if not report['ok']:ctx._rax_observation=None
    report['elapsed_s'] = time.monotonic()-t
    return report


def locate_block(ctx, observation=None, interior_pixels=None, *, color=None):
    """Read-only synchronous color selection and native depth on exact head RGB.

    Default captures fresh RGB/depth itself. Supplied observations must be recent
    and exactly bound. Optional pixels must pass the same semantic interior gate.
    Returns visible surface estimates, never a grasp pose or a motion permit.
    """
    from perception import PerceptionUnknown, select_block, measure_block
    def operation(report):
        if observation is None:
            _, obs = _start(ctx, report=report, require_depth=True, motion=False)
        else:
            state = ctx.value('status'); _budget(state)
            if state.get('control_uncertain') is not False:
                raise RuntimeError('Unknown control state')
            obs = observation
        report['measured'] = obs
        try:
            selected = select_block(obs, color, interior_pixels=interior_pixels)
            report['selection'] = selected
            depth = ctx.value('piper_pixel_to_position', camera='head',
                             frame_id=obs['frame_id'], pixels=selected['pixels'], radius=3)
            report['depth'] = depth
            report['localization'] = measure_block(obs, selected, depth)
        except PerceptionUnknown as exc:
            raise NeedsReview('localize_colored_surface', obs, obs,
                              {'color': color}, {'ok': None, 'why': str(exc)}) from exc
        report['facts'].append({'ok': True, 'frame_id': obs['frame_id'],
             'why': 'One compact color component with interior pixels and consistent native depth',
             'scope': 'visible surface localization only; not support/grasp/clearance'})
        return obs
    return _run(ctx, 'locate_block', operation)


def verify_completed_action(ctx, *, previous_report, assess=None):
    """Inspect a completed endpoint without issuing any motion or replaying a step.

    The callback supplies the missing endpoint fact, never the pre-action fact.
    The returned recovery report can replace previous_report at the next boundary.
    """
    import hashlib
    from pathlib import Path
    from scene_checks import read_report, completed_action_state
    def operation(report):
        robot, obs = _start(ctx, report=report, motion=False,force_observe=True)
        raw = Path(previous_report).read_bytes()
        record = read_report(previous_report)
        if record['result'].get('recovered_endpoint') is not None:
            raise PerceptionUnknown('Use the recovered report directly; do not nest recoveries')
        previous = completed_action_state(record, obs, robot)
        phase = {'approach': 'grasp_alignment' if previous.get('engage') else 'hover_alignment',
                 'empty_joint_waypoint': 'empty_joint_endpoint',
                 'close_grasp': 'closed_grasp_before_trial',
                 'trial_lift': 'retained_after_trial',
                 'open_empty_gripper': 'empty_open_endpoint',
                 'transport': 'retained_after_transport',
                 'lower_to_support': 'supported_before_release',
                 'release': 'released_on_support', 'retreat': 'final_structure'}.get(previous['stage'])
        if phase is None and previous['stage'] not in ('green_stack_lower','green_stack_release','green_stack_retreat'):
            raise PerceptionUnknown('Unsupported recovery boundary')
        if phase is None:phase='green_endpoint_recovery'
        # Keep the true pre-action frame for comparisons, including trial lift.
        before = previous.get('initial_observation') or previous.get('review', {}).get('before')
        checker=assess
        if previous['stage'] in ('green_stack_lower','green_stack_release','green_stack_retreat'):
            from recovery import green_endpoint
            checker=lambda **kw:green_endpoint(ctx,record,previous_report,obs,robot)
        if checker is None and previous['stage']=='lower_to_support' and previous.get('support_required'):
            from manipulation_checks import LowerChecks
            checker=LowerChecks(ctx,previous_report,True)
        if checker is None and previous['stage']=='retreat':
            from manipulation_checks import inspect_retreat_endpoint
            checker=lambda **kw:inspect_retreat_endpoint(ctx=ctx,**kw)
        report['facts'].append(_fact(checker, phase, before, obs,
                                    {'completed_report': previous, 'never_replay_motion': True}))
        report['recovery'] = {'report': str(previous_report),
                              'sha256': hashlib.sha256(raw).hexdigest()}
        report['next_function'] = previous.get('next_function')
        return obs
    return _run(ctx, 'verify_completed_action', operation)


def verify_approach_boundary(ctx, *, previous_report, initial_scene_report, empty_history,
                             color='orange', scene_invalidated=False):
    """Recover a completed EMPTY approach using RGB and retained XYZ only."""
    from scene_checks import resolve_empty_scene, approach_scene
    def assess(**kw):
        obs = kw['after']
        scene = resolve_empty_scene(previous_report, obs, initial_scene_report=initial_scene_report,
                                    empty_history=empty_history, invalidated=scene_invalidated, color=color, robot=ctx.value('piper_status'))
        fk = ctx.value('piper_fk', joints=obs['measured14'][:6])
        return approach_scene(ctx, obs, scene, color, fk['position'])
    result = verify_completed_action(ctx, previous_report=previous_report, assess=assess)
    result['stage'] = 'verify_approach_boundary'
    return result


def approach(ctx, *, waypoints, quaternion, assess=None, previous_report=None, color='orange', engage=True, initial_scene_report=None, empty_history=None, scene_invalidated=False, insertion_fact_request_id=None, require_depth=False):
    """Empty-arm approach to explicit scene-relative targets chosen for this run.

    assess('empty_approach_path') covers current obstacles, not just CAD self checks.
    Backend performs live IK/path/state checks; refusals return without blind retry.
    """
    def operation(report):
        robot, before = _start(ctx, report=report, require_depth=require_depth)
        points, q = _poses(robot, waypoints, quaternion)
        report['planned_motion_count'] = len(points)
        report['engage'] = engage
        inputs = {'waypoints': points, 'quaternion': q}
        if assess is None and previous_report is not None:
            from scene_checks import resolve_empty_scene, approach_scene, validate_insertion_fact
            try:scene=resolve_empty_scene(previous_report,before,initial_scene_report=initial_scene_report,
                         empty_history=empty_history,invalidated=scene_invalidated,color=color,robot=robot)
            except PerceptionUnknown as e:
                raise NeedsReview('approach_reference',before,before,inputs,{'ok':None,'why':str(e)}) from e
            after=before
            report['engage']=engage
            max_down=.021
            if insertion_fact_request_id is not None:
                if len(points)!=1:raise PerceptionUnknown('Onsite insertion fact covers one segment only')
                fact=ctx.value('request_operator',request_id=insertion_fact_request_id,timeout_s=0)
                history=[h['report'] for h in scene['retention']['history']]
                report['assistance']=validate_insertion_fact(fact,insertion_fact_request_id,before,robot,history)
                max_down=.040
            # Preplan return to the known empty hover before any descent.
            fk=ctx.value('piper_fk',joints=before['measured14'][:6])
            report['retreat_ik']=ctx.value('piper_solve_ik',position=fk['position'],quaternion_xyzw=fk['quaternion_xyzw'],seed=before['measured14'][:6])
            if report['retreat_ik'].get('ok') is not True:
                raise NeedsReview('retreat_ik_unavailable',before,before,inputs,report['retreat_ik'])
            if len(points)>6:raise ValueError('At most six bounded approach segments')
            if getattr(ctx, '_blocks_continuity', False) and insertion_fact_request_id is None:
                from continuous_approach import execute
                return execute(ctx, robot, before, scene, color, points, q, engage, report)
            for p in points:
                checker=lambda **kw:approach_scene(ctx,kw['after'],scene,color,p,engage=engage,max_down_m=max_down)
                report['facts'].append(_fact(checker,'vertical_approach_segment',after,after,inputs))
                after=_move(ctx,'piper_move_pose',{'position':p,'quaternion_xyzw':q},report,True)
            checker=lambda **kw:approach_scene(ctx,kw['after'],scene,color,points[-1],engage=engage)
            report['facts'].append(_fact(checker,'grasp_alignment' if engage else 'hover_alignment',before,after,inputs))
            report['engage']=engage
            return after
        report['facts'].append(_fact(assess, 'empty_approach_path', before, before, inputs))
        after = before
        for i, p in enumerate(points):
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion_xyzw': q}, report, i==len(points)-1)
        report['facts'].append(_fact(assess, 'grasp_alignment', before, after, inputs))
        return after
    return _run(ctx, 'approach', operation, 'close_grasp' if engage else 'approach')


def open_empty_gripper(ctx, *, assess=None, gap_m=OPEN_GAP_M):
    """Only after current evidence establishes empty fingers and clear opening sweep."""
    def operation(report):
        _, before = _start(ctx, report=report, require_depth=True)
        if not 0 < gap_m <= 0.075:
            raise ValueError('Gap outside this task tested/opening range')
        if assess is None:
            from scene_checks import empty_opening
            checker=lambda **kw: empty_opening(ctx,kw['after'],kw['inputs']['gap_m'])
        else:
            checker=assess
        report['facts'].append(_fact(checker, 'empty_opening_sweep', before, before, {'gap_m': gap_m}))
        return _jaw(ctx, before, gap_m, report)
    return _run(ctx, 'open_empty_gripper', operation, 'approach')


def close_grasp(ctx, *, assess=None, previous_report=None, color='orange'):
    """Close a freshly verified engaged block; return before the trial-lift boundary."""
    def operation(report):
        robot, before = _start(ctx, report=report,require_depth=False)
        checker=assess
        if assess is None and previous_report is not None:
            from scene_checks import supported_entry, read_report, completed_action_state, scene_control
            prev=completed_action_state(read_report(previous_report), before, robot)
            if prev.get('stage') != 'approach':
                raise PerceptionUnknown('Closure requires the completed insertion report')
            scene=prev['facts'][0]['scene']
            scene_control(scene,robot)
            checker=lambda **kw:supported_entry(ctx,kw['after'],scene,color)
        report['facts'].append(_fact(checker, 'engaged_supported_block', before, before,
                                    {'command_gap_m': GRASP_GAP_M}))
        return _jaw(ctx, before, GRASP_GAP_M, report)
    return _run(ctx, 'close_grasp', operation, 'trial_lift')


def trial_lift(ctx, *, quaternion, assess=None, previous_report=None, color='orange'):
    """Lift from the CURRENT measured pose; no closure or grasp replay inside."""
    def operation(report):
        robot, before = _start(ctx, report=report,require_depth=False,allowance_s=TRIAL_ALLOWANCE_S)
        fk = ctx.value('piper_fk', joints=before['measured14'][:6])
        report['planned_motion_count'] = 1
        target = list(fk['position']); target[2] += TRIAL_LIFT_M
        _, q = _poses(robot, [target], quaternion)
        current_q=_vector(fk['quaternion_xyzw'],4)
        if abs(sum(x*y for x,y in zip(q,current_q)))<.99999:
            raise ValueError('Trial lift must preserve the current orientation')
        if assess is None and previous_report is not None:
            from scene_checks import trial_surface,wrist_trial_match,wrist_between_pads,read_report,completed_action_state,scene_control,scene_geometry
            prev=completed_action_state(read_report(previous_report), before, robot)
            if prev.get('stage') != 'close_grasp' or not prev.get('ok'):
                raise PerceptionUnknown('Trial requires a verified completed closure')
            scene=prev['facts'][0]['scene']
            scene_control(scene,robot);scene_geometry(scene,before)
            baseline_z=scene['blocks'][color]['top_center']['surface_position_m'][2]
            from qualitative import alignment_evidence
            wrist_before=alignment_evidence(ctx,before,color)
            report['facts'].append({'ok':True,'frame_id':before['frame_id'],
                'why':'Completed closure, unchanged control state and recognizable block at the grasp; retention remains to be tested',
                'wrist':wrist_before,'scene':scene})
            report['retreat_ik']=ctx.value('piper_solve_ik',position=fk['position'],quaternion_xyzw=current_q,seed=before['measured14'][:6])
            if report['retreat_ik'].get('ok') is not True:
                raise NeedsReview('trial_return_ik_unavailable',before,before,{},report['retreat_ik'])
            lifted=_move(ctx,'piper_move_pose',{'position':target,'quaternion_xyzw':q},report,allowance_s=TRIAL_ALLOWANCE_S)
            try:
                after_fk=ctx.value('piper_fk',joints=lifted['measured14'][:6])
                actual_lift=after_fk['position'][2]-fk['position'][2]
                if not .5*TRIAL_LIFT_M<=actual_lift<=2*TRIAL_LIFT_M+.005:
                    raise PerceptionUnknown('Measured trial displacement does not match the intended lift')
                if abs(lifted['measured14'][6]-before['measured14'][6])>.003:
                    raise PerceptionUnknown('Jaw opening changed during trial')
                from qualitative import retention_evidence
                match=retention_evidence(ctx,before,lifted,color)
                # Depth is optional independent corroboration, never requested here.
                # If supplied, reliable contradictory height change must not be ignored.
                depth_rise=None
                if all(o['cameras']['head'].get('metric_depth') is True for o in (before,lifted)):
                    try:
                        surface=trial_surface(ctx,before,scene,baseline_z,color)
                        after_surface=trial_surface(ctx,lifted,scene,baseline_z,color)
                        depth_rise=after_surface['surface_position_m'][2]-surface['surface_position_m'][2]
                    except PerceptionUnknown:
                        pass  # Missing depth is not a negative grasp result.
                    if depth_rise is not None and not .003<=depth_rise<=.030:
                        raise PerceptionUnknown('Reliable head depth contradicts block retention')
            except PerceptionUnknown as e:raise NeedsReview('retained_after_trial',before,lifted,{}, {'ok':None,'why':str(e)}) from e
            report['facts'].append({'ok':True,'frame_id':lifted['frame_id'],'why':'Measured upward motion, steady jaw and coarse wrist-relative object appearance support retention','measured_lift_m':actual_lift,'optional_head_rise_m':depth_rise,'wrist_match':match,'scope':'Qualitative candidate, not measured force; no physical trial in R4'})
            report['held_block']=color;report['retained_scene_invalidated_by']='Moved held block requires update; stationary neighbors retain provenance unless displacement is observed'
            return lifted
        report['facts'].append(_fact(assess, 'closed_grasp_before_trial', before, before,
                                    {'target': target, 'command_gap_m': GRASP_GAP_M}))
        lifted = _move(ctx, 'piper_move_pose', {'position': target, 'quaternion_xyzw': q}, report)
        report['facts'].append(_fact(assess, 'retained_after_trial', before, lifted,
                                    {'lift_m': TRIAL_LIFT_M, 'command_gap_m': GRASP_GAP_M}))
        return lifted
    return _run(ctx, 'trial_lift', operation, 'transport')


def grasp_and_trial_lift(ctx, **kwargs):
    """Old combined entry deliberately performs no action; use the two boundaries."""
    return {'ok': False, 'executed': False, 'why': 'Use close_grasp, inspect result, then trial_lift',
            'next_function': 'close_grasp', 'status': 'split_function_entry'}


def transport(ctx, *, waypoints, quaternion, assess=None, previous_report=None, color='orange', support_report=None):
    """Only with a retained block; path review includes payload and neighboring blocks."""
    def operation(report):
        robot, before = _start(ctx, report=report, require_depth=False)
        points, q = _poses(robot, waypoints, quaternion)
        inputs = {'waypoints': points, 'quaternion': q}
        checker=assess
        if checker is None and previous_report is not None:
            from manipulation_checks import LoadedChecks
            checker=LoadedChecks(ctx,previous_report,color,support_report=support_report)
        report['planned_motion_count']=len(points)
        report['facts'].append(_fact(checker, 'retained_and_loaded_path', before, before, inputs))
        after = before
        for i, p in enumerate(points):
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion_xyzw': q}, report, i==len(points)-1)
        report['facts'].append(_fact(checker, 'retained_at_destination', before, after, inputs))
        return after
    return _run(ctx, 'transport', operation, 'lower_to_support')


def plan_base_placement(ctx, *, previous_report, quaternion, spacing_min_m=.060,
                        spacing_max_m=.075, preferred_spacing_m=.065, finger_margin_m=.003):
    """Read-only candidates for this run; Agent selects/executes the next function."""
    from scene_checks import read_report,completed_action_state
    from placement_plan import side_by_side
    def operation(report):
        robot,obs=_start(ctx,report=report,motion=False,require_depth=False)
        prev=completed_action_state(read_report(previous_report),obs,robot)
        if not prev.get('ok') or prev['stage']!='transport':
            raise PerceptionUnknown('Plan placement after a verified loaded raise/transport')
        fact=prev['facts'][0]
        result=side_by_side(ctx,obs,fact['scene'],fact['payload_origin'],quaternion,
            spacing_min_m=spacing_min_m,spacing_max_m=spacing_max_m,
            preferred_spacing_m=preferred_spacing_m,finger_margin_m=finger_margin_m)
        report['placement_plan']=result
        if not result['ok']:raise PerceptionUnknown(result['why'])
        report['facts'].append({'ok':True,'frame_id':obs['frame_id'],'why':'Read-only bounded candidate plan; no motion dispatched'})
        return obs
    return _run(ctx,'plan_base_placement',operation,'transport')


def lower_to_support(ctx, *, descent, retreat, quaternion, assess=None, previous_report=None, support_required=True):
    """Stop at support boundary; never open the jaws in this function."""
    def operation(report):
        robot, before = _start(ctx, report=report, require_depth=False)
        down, q = _poses(robot, descent, quaternion)
        up, _ = _poses(robot, retreat, quaternion)
        inputs = {'descent': down, 'retreat': up, 'quaternion': q}
        checker=assess
        if checker is None and previous_report is not None:
            from manipulation_checks import LowerChecks
            checker=LowerChecks(ctx,previous_report,support_required)
        report['planned_motion_count']=len(down)
        report['support_required']=support_required
        report['facts'].append(_fact(checker, 'placement_and_retreat_path', before, before, inputs))
        after = before
        for i, p in enumerate(down):
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion_xyzw': q}, report, i==len(down)-1)
        report['facts'].append(_fact(checker, 'supported_before_release', before, after, inputs))
        return after
    return _run(ctx, 'lower_to_support', operation, 'release' if support_required else 'lower_to_support')


def release(ctx, *, assess=None, previous_report=None, support_fact_request_id=None):
    """Only a supported block; no automatic re-descent or retreat."""
    def operation(report):
        _, before = _start(ctx, report=report,force_observe=True)
        checker=assess
        if checker is None and previous_report is not None and support_fact_request_id is not None:
            from manipulation_checks import SupportedReleaseChecks
            checker=SupportedReleaseChecks(ctx,previous_report,support_fact_request_id)
        elif checker is None and previous_report is not None:
            from manipulation_checks import VisualReleaseChecks
            checker=VisualReleaseChecks(ctx,previous_report)
        report['facts'].append(_fact(checker, 'supported_before_release', before, before, {}))
        return _jaw(ctx, before, OPEN_GAP_M, report)
    return _run(ctx, 'release', operation, 'retreat')


def retreat(ctx, *, waypoints, quaternion, assess=None, previous_report=None):
    def operation(report):
        robot, before = _start(ctx, report=report, require_depth=False)
        points, q = _poses(robot, waypoints, quaternion)
        inputs = {'waypoints': points, 'quaternion': q}
        checker=assess
        if checker is None and previous_report is not None:
            from manipulation_checks import RetreatChecks
            checker=RetreatChecks(ctx,previous_report)
        report['planned_motion_count']=len(points)
        report['facts'].append(_fact(checker, 'released_and_retreat_path', before, before, inputs))
        after = before
        for i,p in enumerate(points):
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion_xyzw': q}, report, i==len(points)-1)
        report['facts'].append(_fact(checker, 'released_stable_and_clear', before, after, inputs))
        return after
    return _run(ctx, 'retreat', operation, 'inspect_task_result')


def place_release_retreat(ctx, **kwargs):
    return {'ok': False, 'executed': False, 'why': 'Use lower_to_support, release, retreat separately',
            'next_function': 'lower_to_support', 'status': 'split_function_entry'}


def empty_joint_waypoint(ctx, *, joints, assess=None, previous_report=None,
                         initial_scene_report=None, empty_history=None, scene_invalidated=False):
    """One supervised empty transition, selected by seeded IK before this call.

    Fresh measured state anchors the native CAD/workspace check. The assessor must
    also establish empty fingers and scene clearance for the complete swept path.
    """
    def operation(report):
        robot, before = _start(ctx, report=report, require_depth=previous_report is None)
        report['planned_motion_count'] = 1
        q = _vector(joints, 6)
        target = q + before['measured14'][6:]
        check = ctx.value('piper_check_path', start14=before['measured14'], target14=target)
        report['preflight'] = check
        if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:
            raise RuntimeError('Native path preflight did not establish CAD and workspace clearance')
        inputs = {'joints': q, 'path_check': check}
        if assess is None:
            from scene_checks import empty_joint_path,resolve_empty_scene,scene_control
            reference_scene=None
            if scene_invalidated:raise PerceptionUnknown('Scene invalidated; acquire a new complete scene/history')
            if previous_report is not None:
                reference_scene=resolve_empty_scene(previous_report,before,
                         initial_scene_report=initial_scene_report,empty_history=empty_history,
                         invalidated=scene_invalidated,robot=robot)
            elif initial_scene_report is not None or empty_history is not None:
                raise PerceptionUnknown('Retained scene requires the last completed previous_report')
            checker=lambda **kw: empty_joint_path(ctx,kw['after'],kw['inputs']['joints'],reference_scene)
        else:
            checker=assess
        fact=_fact(checker, 'empty_joint_path', before, before, inputs)
        if assess is None and previous_report is None:scene_control(fact['scene'],robot,establish=True)
        report['facts'].append(fact)
        after = _move(ctx, 'piper_move_joints', {'joints': q}, report)
        return after
    return _run(ctx, 'empty_joint_waypoint', operation, 'approach')


def empty_joint_transition(ctx, *, joint_waypoints):
    """Execute the planned empty entry in one function, observing only at ends."""
    from empty_transition import execute
    # Keep the existing empty-flight history semantic, with all real receipts.
    return _run(ctx, 'empty_joint_waypoint',
                lambda report: execute(ctx, joint_waypoints, report), 'approach')


def empty_inspection(ctx, *, position, quaternion, previous_report):
    """One empty high move to reveal the table after a released base placement."""
    from scene_checks import completed_action_state,read_report,installed_finger_corners
    from history_scene import released_scene,scene_height_bounds
    import numpy as np
    def operation(report):
        robot,obs=_start(ctx,report=report,require_depth=False)
        prev=completed_action_state(read_report(previous_report),obs,robot)
        if not prev.get('ok') or prev['stage'] not in ('retreat','empty_inspection'):
            raise PerceptionUnknown('Need completed verified empty retreat/inspection')
        if not .065<=obs['measured14'][6]<=.074:
            raise PerceptionUnknown('Expected open empty jaw')
        points,q=_poses(robot,[position],quaternion)
        scene=released_scene(ctx,obs,robot,previous_report)
        table=scene['table'];plane=np.array(table['z_ax_by_c']);blocks=scene_height_bounds(scene)
        start=obs['measured14'];fk=ctx.value('piper_fk',joints=start[:6]);T=np.array(fk['T_base_flange'])
        if abs(np.dot(q,fk['quaternion_xyzw']))<np.cos(np.deg2rad(60)/2):
            raise PerceptionUnknown('Empty inspection orientation transition exceeds60deg; compare nearer seeded candidates')
        sol=ctx.value('piper_solve_ik',position=position,quaternion_xyzw=q,seed=start[:6]);end=sol['q'];check=ctx.value('piper_check_path',start14=start,target14=end+start[6:])
        if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:raise PerceptionUnknown('Inspection CAD/workspace failed')
        corners=installed_finger_corners(ctx,obs,scene,fk);local=(corners-T[:3,3])@T[:3,:3];top=max(b['height_p95_m'] for b in blocks.values());minimum=1.
        for t in np.linspace(0,1,41):
            f=ctx.value('piper_fk',joints=(np.array(start[:6])+(np.array(end)-start[:6])*t).tolist());F=np.array(f['T_base_flange']);corn=local@F[:3,:3].T+F[:3,3]
            clearance=float(np.min(corn[:,2]-(corn[:,:2]@plane[:2]+plane[2]))-top);minimum=min(minimum,clearance)
        if minimum<.025:raise PerceptionUnknown('Empty inspection lacks25mm full finger clearance over observed table-level blocks')
        report['planned_motion_count']=1;report['facts'].append({'ok':True,'frame_id':obs['frame_id'],'why':'Verified empty release/retreat; retained stationary and receipt-updated block heights; sampled fingers above all table-level blocks','scene':scene,'table':table,'blocks':blocks,'min_clearance_m':minimum,'preflight':check})
        return _move(ctx,'piper_move_joints',{'joints':end},report)
    return _run(ctx,'empty_inspection',operation,'locate_block')


def green_pregrasp(ctx, *, position, quaternion, previous_report, stationary_scene_report,
                   scene_invalidated=False):
    """Candidate only: one empty green-relative hover, never grasp/stack.

    Caller must invalidate retained green after reset/contact/displacement;
    wrist identity alone cannot establish that a hidden block stayed still.
    """
    from green_checks import GreenPregrasp
    checker=GreenPregrasp(ctx,previous_report,stationary_scene_report,scene_invalidated)
    return approach(ctx,waypoints=[position],quaternion=quaternion,assess=checker,engage=False,color='green',require_depth=False)


def refresh_stack_support(ctx, *, previous_report):
    """Read-only: measure the base pair after placement, before green pickup."""
    import hashlib
    from pathlib import Path
    from scene_checks import completed_action_state, read_report, measure_scene, scene_control
    import numpy as np
    def operation(report):
        robot, obs = _start(ctx, report=report, motion=False, require_depth=True)
        source_bytes = Path(previous_report).read_bytes()
        record = read_report(previous_report)
        prev = completed_action_state(record, obs, robot)
        if (not prev.get('ok') or prev['stage'] != 'approach' or prev.get('engage')
                or record['inputs'].get('color') != 'green'):
            raise PerceptionUnknown('Refresh support at a verified empty pregrasp endpoint')
        scene = measure_scene(ctx, obs, colors=['red', 'orange'])
        scene_control(scene, robot, establish=True)
        red = np.array(scene['blocks']['red']['top_center']['surface_position_m'])
        orange = np.array(scene['blocks']['orange']['top_center']['surface_position_m'])
        plane = np.array(scene['table']['z_ax_by_c'])
        heights = {c: float(np.array(b['top_center']['surface_position_m'])[2] -
            (np.array(b['top_center']['surface_position_m'])[:2] @ plane[:2] + plane[2]))
            for c, b in scene['blocks'].items()}
        if (abs(red[0]-orange[0]) > .015 or not .050 <= red[1]-orange[1] <= .080
                or abs(red[2]-orange[2]) > .008
                or any(not .045 <= h <= .085 for h in heights.values())):
            raise PerceptionUnknown('Fresh base pair is outside the upright side-by-side stack model')
        report['facts'].append({'ok': True, 'frame_id': obs['frame_id'], 'scene': scene,
            'heights_m': heights, 'center_spacing_m': float(np.linalg.norm(red[:2]-orange[:2])),
            'why': 'Fresh native head depth of both base blocks after placement; approximately level table-supported pair',
            'scope': 'Numerical support anchor only; visual support/alignment still required at placement'})
        report['support_scene'] = scene
        if Path(previous_report).read_bytes() != source_bytes:
            raise PerceptionUnknown('Pregrasp report changed during support refresh')
        report['support_source'] = {'report': str(previous_report),
                                    'sha256': hashlib.sha256(source_bytes).hexdigest()}
        return obs
    return _run(ctx, 'refresh_stack_support', operation, 'approach')


def green_stack_lower(ctx, *, descent, retreat, quaternion, previous_report, support_report, support_required=True):
    """R8 candidate: one green-on-red lowering segment; never release here."""
    from stack_checks import StackLowerChecks
    def operation(report):
        robot,before=_start(ctx,report=report)
        down,q=_poses(robot,descent,quaternion);up,_=_poses(robot,retreat,quaternion)
        inputs={'descent':down,'retreat':up,'quaternion':q}
        checker=StackLowerChecks(ctx,previous_report,support_report,support_required)
        report['planned_motion_count']=len(down);report['support_required']=support_required
        report['facts'].append(_fact(checker,'stack_path',before,before,inputs))
        after=_move(ctx,'piper_move_pose',{'position':down[0],'quaternion_xyzw':q},report)
        report['facts'].append(_fact(checker,'stack_supported',before,after,inputs))
        return after
    return _run(ctx,'green_stack_lower',operation,'green_stack_release' if support_required else 'green_stack_lower')


def green_stack_release(ctx, *, previous_report, support_report):
    """R8 candidate: open only at a verified green-on-red support endpoint."""
    from stack_checks import StackReleaseChecks
    def operation(report):
        _,before=_start(ctx,report=report,force_observe=True)
        checker=StackReleaseChecks(ctx,previous_report,support_report)
        report['facts'].append(_fact(checker,'green_supported_before_release',before,before,{}))
        return _jaw(ctx,before,OPEN_GAP_M,report)
    return _run(ctx,'green_stack_release',operation,'green_stack_retreat')


def green_stack_retreat(ctx, *, waypoints, quaternion, previous_report):
    """R8 candidate: open vertical withdrawal; verify the complete L structure."""
    from stack_checks import StackRetreatChecks
    def operation(report):
        robot,before=_start(ctx,report=report)
        points,q=_poses(robot,waypoints,quaternion);inputs={'waypoints':points,'quaternion':q}
        checker=StackRetreatChecks(ctx,previous_report)
        report['planned_motion_count']=len(points)
        report['facts'].append(_fact(checker,'released_and_retreat_path',before,before,inputs))
        after=_move(ctx,'piper_move_pose',{'position':points[0],'quaternion_xyzw':q},report)
        report['facts'].append(_fact(checker,'released_stable_and_clear',before,after,inputs))
        return after
    return _run(ctx,'green_stack_retreat',operation,'inspect_task_result')


def main(ctx, *, output_dir=None, allow_auxiliary_vision=False, optimize_trajectory=True, group_empty_entry=False, blend_empty_entry=False):
    """Compose R8 boundaries on this SAME supplied context; first failure stops.

    Caller owns authorization, native Prepare/lifecycle and sole controller use.
    No resume index, historical scene input, recovery dispatch or finally motion.
    A context that entered main cannot enter it again: inspect its saved boundary
    and use separately checked individual functions for same-session recovery.
    """
    from collections import Counter
    from pathlib import Path
    import uuid
    import composition as inputs
    from rehearse_step import run_one, save

    if type(allow_auxiliary_vision) is not bool:raise TypeError('allow_auxiliary_vision must be boolean')
    if type(optimize_trajectory) is not bool:raise TypeError('optimize_trajectory must be boolean')
    if type(blend_empty_entry) is not bool:raise TypeError('blend_empty_entry must be boolean')
    if type(group_empty_entry) is not bool:raise TypeError('group_empty_entry must be boolean')
    ctx._blocks_continuity=optimize_trajectory
    ctx.allow_auxiliary_vision=allow_auxiliary_vision
    previous_run = getattr(ctx, '_magnetic_blocks_composition_run', None)
    if previous_run is not None:
        raise RuntimeError('main already entered on this ctx; inspect '+str(previous_run)+'; never replay a partial run')
    root = Path(output_dir) if output_dir is not None else Path('output/composition/runs')/uuid.uuid4().hex
    root = root.resolve()
    root.mkdir(parents=True, exist_ok=False)  # reserve a new run, including interrupted runs
    report_path = root/'REPORT.json'
    ctx._magnetic_blocks_composition_run = str(report_path)
    started = time.monotonic()
    report = {'ok':False, 'status':'in_progress', 'code_status':'COMPOSED CANDIDATE',
              'started_unix':time.time(), 'report':str(report_path), 'steps':[],
              'active_step':None, 'active_step_report':None, 'failed_step':None,
              'counters_complete':True, 'control_uncertain':False,
              'primary_model_calls':0,
              'held_state':{'object':None,'uncertain':True},
              'next_recovery_boundary':'Verify native neutral/Prepare and empty jaw',
              'never_blindly_replay':True, 'automatic_recovery':False,
              'optimization_analysis_only':False, 'candidate_version':'v0.3.1-blend-batchfk', 'group_empty_entry':group_empty_entry,'blend_empty_entry':blend_empty_entry, 'trajectory_optimization':optimize_trajectory, 'allow_auxiliary_vision':allow_auxiliary_vision, 'scene_scope':'current run only'}
    save(report_path, report)

    class StopChain(Exception): pass

    def persist():
        counts = Counter()
        for step in report['steps']: counts.update(step.get('tool_counts', {}))
        report.update(elapsed_s=time.monotonic()-started, tool_counts=dict(counts),
            auxiliary_visual_calls=counts['gemini_generate'], calls=sum(counts.values()),
            motion_dispatch_count=sum(s.get('motion_dispatch_count',0) for s in report['steps']),
            motion_receipt_count=sum(s.get('motion_receipt_count',0) for s in report['steps']),
            ambiguous_dispatch_count=sum(s.get('ambiguous_dispatch_count',0) for s in report['steps']),
            function_wall_s=sum(s.get('wall_s',0) for s in report['steps']),
            tool_s=sum(s.get('tool_s',0) for s in report['steps']))
        from timing import inter_motion_gaps
        report['inter_motion_gaps']=inter_motion_gaps(report['steps'])
        save(report_path, report)

    def step(label, function, args=None, operation=None):
        report['active_step'] = label
        path = root/f'{len(report["steps"])+1:03d}-{label}.json'
        report['active_step_report'] = str(path)
        report['next_recovery_boundary'] = ('Inspect active step report and native status/receipts before any further action: '+label)
        # Persist the uncertainty of a potential dispatched action before entry.
        prior_held = dict(report['held_state'])
        if function in ('close_grasp','trial_lift','transport','lower_to_support',
                        'release','green_stack_lower','green_stack_release','green_stack_retreat','retreat'):
            report['held_state'] = {'object':(args or {}).get('color',prior_held['object']),
                                    'uncertain':True, 'reason':'active boundary not yet verified'}
        persist()
        record = run_one(ctx, function, args or {}, path, operation=operation)
        result = record['result']
        summary = {k:record[k] for k in ('function','wall_s','tool_s','motion_rpc_s','calls',
            'tool_counts','auxiliary_visual_calls','motion_dispatch_count','motion_receipt_count','ambiguous_dispatch_count')}
        summary.update(label=label, report=str(path), summary_report=record['summary_report'], timing=record['timing'], ok=result.get('ok') is True,
                       stage=result.get('stage'), elapsed_s=result.get('elapsed_s'),
                       dispatch_attempted=result.get('dispatch_attempted',False),
                       action_completed=result.get('action_completed',False), why=result.get('why'))
        report['steps'].append(summary)
        report['held_state'] = prior_held
        if result.get('ok') is not True or record['ambiguous_dispatch_count']:
            report.update(status='partial', failed_step=label,
                          control_uncertain=bool(result.get('control_uncertain') or record['ambiguous_dispatch_count']),
                          why=result.get('why','Unconfirmed action receipt'))
            if record['motion_dispatch_count'] or prior_held['object'] is not None:
                report['held_state'] = {'object':(args or {}).get('color',prior_held['object']),
                    'uncertain':True, 'reason':'failed/ambiguous boundary; jaw or payload outcome needs inspection'}
            if record['ambiguous_dispatch_count'] or (result.get('dispatch_attempted') and not result.get('action_completed')):
                boundary = 'Resolve dispatch with native status, receipt and fresh observation; do not replay this action or main'
            elif result.get('dispatch_attempted'):
                boundary = 'Inspect completed/partial receipts and endpoint facts; never replay the action; individually verify a supported recovery boundary'
            else:
                boundary = 'Resolve refused precondition at the unchanged measured pose; no automatic retry'
            if function in ('green_stack_lower','green_stack_release','green_stack_retreat'):
                boundary += '; preserve the dispute; verify_completed_action can recheck a confirmed complete green endpoint without replay'
            report['next_recovery_boundary'] = boundary
            report['suggested_next_function_after_review'] = result.get('next_function')
            persist()
            raise StopChain()
        if function == 'initial_state':
            report['held_state'] = {'object':None,'uncertain':False,'reason':'fresh startup empty-opening guard passed'}
        elif function == 'close_grasp':
            report['held_state'] = {'object':args['color'],'uncertain':True,'reason':'closed; retention requires separate trial'}
        elif function == 'trial_lift':
            report['held_state'] = {'object':args['color'],'uncertain':False,'reason':'qualitative trial retention passed; no force measurement'}
        elif function in ('release','green_stack_release'):
            report['held_state'] = {'object':prior_held['object'],'uncertain':True,'reason':'opened at checked support; withdrawal still required'}
        elif function in ('retreat','green_stack_retreat'):
            report['held_state'] = {'object':None,'uncertain':False,'reason':'open withdrawal and final visual check passed'}
        report['active_step'] = None
        report['active_step_report'] = None
        persist()
        return str(path), result

    def calc(label, fn, **args):
        return step(label, fn.__name__, args, fn)[1]

    def pose(label, previous, mode, **args):
        return calc(label, inputs.pose_inputs, previous_report=previous, mode=mode, **args)['inputs']

    try:
        step('initial-state', 'initial_state', operation=inputs.initial_state)
        prev, _ = step('open-empty', 'open_empty_gripper')
        entry = calc('plan-orange-entry', inputs.orange_entry, previous_report=prev)
        if blend_empty_entry:
            prev, _ = step('orange-entry-blended','empty_joint_batch',{'joint_waypoints':entry['joint_waypoints']})
            stationary = prev
        elif group_empty_entry:
            prev, _ = step('orange-entry', 'empty_joint_transition',
                           {'joint_waypoints':entry['joint_waypoints']})
            stationary = prev
        else:
            stationary = None
            for i, joints in enumerate(entry['joint_waypoints']):
                args = {'joints':joints}
                if i: args['previous_report'] = prev
                prev, _ = step(f'orange-transition-{i+1}', 'empty_joint_waypoint', args)
                if stationary is None: stationary = prev
        approach_args = calc('orange-approach-inputs', inputs.approach_inputs,
                             previous_report=prev, color='orange')['inputs']
        prev, _ = step('orange-approach','approach',approach_args)
        prev, _ = step('orange-close','close_grasp',{'previous_report':prev,'color':'orange'})
        prev, _ = step('orange-trial','trial_lift',pose('orange-trial-inputs',prev,'trial'))
        prev, _ = step('orange-raise','transport',pose('orange-raise-inputs',prev,'orange_raise'))
        plan, _ = step('orange-base-plan','plan_base_placement',pose('orange-plan-inputs',prev,'base_plan'))
        prev, _ = step('orange-transfer','transport',pose('orange-transfer-inputs',prev,'orange_transfer',placement_plan_report=plan))
        for i in (1,2):
            args = pose(f'orange-lower-{i}-inputs',prev,f'orange_lower_{i}',placement_plan_report=plan)
            prev, _ = step(f'orange-lower-{i}','lower_to_support',args)
        prev, _ = step('orange-release','release',{'previous_report':prev})
        prev, _ = step('orange-retreat','retreat',pose('orange-retreat-inputs',prev,'orange_retreat'))
        for i in (1,2):
            args = calc(f'inspection-{i}-inputs',inputs.inspection_inputs,previous_report=prev,turn=i)['inputs']
            prev, _ = step(f'inspection-{i}','empty_inspection',args)
        args = calc('green-hover-inputs',inputs.green_hover_inputs,previous_report=prev,
                    stationary_scene_report=stationary)['inputs']
        prev, _ = step('green-pregrasp','green_pregrasp',args)
        support, _ = step('refresh-stack-support','refresh_stack_support',{'previous_report':prev})
        args = calc('green-approach-inputs',inputs.approach_inputs,previous_report=prev,color='green')['inputs']
        prev, _ = step('green-approach','approach',args)
        prev, _ = step('green-close','close_grasp',{'previous_report':prev,'color':'green'})
        prev, _ = step('green-trial','trial_lift',pose('green-trial-inputs',prev,'trial',color='green'))
        prev, _ = step('green-raise-1','transport',pose('green-raise-1-inputs',prev,'green_raise_1',color='green',support_report=support))
        for i in (1,3):  # preserve low detour/raise; merge the two high level legs
            args = calc(f'green-transfer-{i}-inputs',inputs.green_transfer_inputs,
                        previous_report=prev,support_report=support,remaining_segments=4-i)['inputs']
            prev, _ = step(f'green-transfer-{i}','transport',args)
            if i == 1:
                prev, _ = step('green-raise-2','transport',pose('green-raise-2-inputs',prev,'green_raise_2',color='green',support_report=support))
        args = calc('green-lower-inputs',inputs.green_lower_inputs,previous_report=prev,support_report=support)['inputs']
        prev, _ = step('green-stack-lower','green_stack_lower',args)
        prev, _ = step('green-stack-release','green_stack_release',{'previous_report':prev,'support_report':support})
        prev, _ = step('green-stack-retreat','green_stack_retreat',pose('green-retreat-inputs',prev,'green_retreat',color='green'))
        report.update(ok=True,status='complete',active_step=None,
            next_recovery_boundary='Task checks completed; any native reset/new layout requires separate authorization',
            why='All composed function checks passed in this invocation; qualification depends on the actual run evidence')
    except StopChain:
        pass
    except BaseException as exc:
        report.update(status='partial',failed_step=report['active_step'],
                      counters_complete=False, control_uncertain=True,
                      why=f'{type(exc).__name__}: {exc}',
                      next_recovery_boundary='Inspect active step, pending dispatch log and native status; never replay after interruption')
        report['held_state']['uncertain'] = True
    persist()
    return report


def _main_cli():
    """Preview by default. Runtime owns the connection only after --execute."""
    import argparse
    import json
    from pathlib import Path
    import sys
    parser = argparse.ArgumentParser(description='R8 composed candidate; offline preview unless explicitly executed')
    parser.add_argument('--execute', action='store_true')
    entry_group=parser.add_mutually_exclusive_group()
    entry_group.add_argument('--blend', action='store_true', help='Use preserved versionA main_blended')
    entry_group.add_argument('--aplus', action='store_true', help='Use independent A+ whole-task-preflight main')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--allow-auxiliary-vision',action='store_true')
    args = parser.parse_args()
    if not args.execute and args.aplus:
        print(json.dumps({'plan_only':True,'entry':'assembly_controller.main(ctx)','version':'Aplus-0.1','hardware_calls':0,'model_calls':0,'goal':'red/orange either side alongY; green over red; whole translation permitted','order':'locate -> whole-task preflight -> rebound -> function execution'}));return 0
    if not args.execute:
        print(json.dumps({'plan_only':True,'entry':'controller.main_blended(ctx)' if args.blend else 'controller.main(ctx)',
            'status':'COMPOSED CANDIDATE','output_dir':str(args.output_dir) if args.output_dir else 'output/composition/runs/<new-id>',
            'start':'separately authorized native neutral/Prepare, closed empty jaw, fresh three-block scene',
            'ordering':['empty opening','three empty transitions','orange approach/close/trial/raise',
                'base plan/transfer/two lowerings/release/retreat','two empty inspections',
                'green pregrasp/support refresh/approach/close/trial','raise/transfer/raise/merged high transfer',
                'green lower/release/retreat'], 'hardware_calls':0,'model_calls':0},indent=2))
        return 0
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
    from api.runtime import Context
    with Context() as ctx:
        from assembly_controller import main as main_aplus
        entry = main_aplus if args.aplus else main_blended if args.blend else main
        result = entry(ctx, output_dir=args.output_dir, allow_auxiliary_vision=args.allow_auxiliary_vision)
    print(json.dumps({k:result[k] for k in ('ok','status','report','elapsed_s','failed_step')},indent=2))
    return 0 if result['ok'] else 1




def empty_joint_batch(ctx, *, joint_waypoints):
    from batched_entry import execute
    return _run(ctx,'empty_joint_waypoint',lambda report:execute(ctx,joint_waypoints,report),'approach')


def main_blended(ctx, *, output_dir=None, allow_auxiliary_vision=False):
    """Independent experimental entry; ordinary main keeps separate functions."""
    ctx._blocks_batch_fk=True
    return main(ctx,output_dir=output_dir,allow_auxiliary_vision=allow_auxiliary_vision,
                group_empty_entry=False,blend_empty_entry=True)


if __name__ == '__main__':
    # Avoid importing a second controller module when the recorder resolves steps.
    import sys
    sys.modules['controller'] = sys.modules[__name__]
    raise SystemExit(_main_cli())
