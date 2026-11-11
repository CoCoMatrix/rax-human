"""R3 manual strategy distilled into RAX rehearsal functions, not a qualified main.

Units: metres/radians; poses control left_link6 in left_base_link.
An optional synchronous perception function supplies facts for the actual frames.
Missing facts return needs_review immediately. The Agent inspects reports between
calls; no function waits for an Agent approval file. Import never connects.
"""
import math
import time

GRASP_GAP_M = 0.050  # R3 command; observed feedback ~0.0509--0.0513, not pad spacing.
OPEN_GAP_M = 0.069   # R3 green/base release; initial orange opening was 0.073.
JAW_STEP_M = 0.009   # <= live 0.010 tool limit, calculated from measured feedback.
TRIAL_LIFT_M = 0.010 # This block run only; never inherit the cup's 40 mm setting.
FUNCTION_ALLOWANCE_S = 120
DISPATCH_ALLOWANCE_S = 60


class NeedsReview(Exception):
    def __init__(self, phase, before, after, inputs, answer=None):
        super().__init__(phase)
        self.review = {'phase': phase, 'before': before, 'after': after,
                       'inputs': inputs, 'assessment': answer}


def _vector(v, n):
    if len(v) != n or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in v):
        raise ValueError(f'Expected {n} finite values')
    return list(v)


def _fact(assess, phase, before, after, inputs):
    if assess is None:
        raise NeedsReview(phase, before, after, inputs)
    answer = assess(phase=phase, before=before, after=after, inputs=inputs)
    if not isinstance(answer, dict) or answer.get('ok') is None:
        raise NeedsReview(phase, before, after, inputs, answer)
    if answer.get('ok') is not True:
        raise RuntimeError(f'{phase}: {answer}')
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


def _start(ctx, *, require_depth=False, motion=True):
    state = ctx.value('status')
    _budget(state, FUNCTION_ALLOWANCE_S if motion else 0)
    if state.get('control_uncertain') is not False:
        raise RuntimeError('Unknown control state; resolve with native status/hold before resuming')
    robot = state['robot_status']
    if robot.get('motion_ready') is False:
        raise RuntimeError('Robot is not motion ready')
    if robot.get('controlled_link') != 'left_link6' or robot.get('frame') != 'left_base_link':
        raise RuntimeError('Unexpected pose contract')
    bridge = robot.get('bridge', {})
    if bridge.get('active_request') or bridge.get('phase', 'IDLE') != 'IDLE':
        raise RuntimeError('Inspect native receipt/state for non-idle bridge before another function')
    obs = ctx.value('piper_observe', cameras=['head', 'left', 'right'], require_depth=require_depth)
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


def _move(ctx, tool, args, report, observe_after=True):
    state = ctx.value('status')
    _budget(state, DISPATCH_ALLOWANCE_S)
    if state.get('control_uncertain') is not False:
        raise RuntimeError('Uncertain control before dispatch; inspect native receipt/state')
    # Record attempted dispatch before calling: a transport error is ambiguous.
    report['dispatch_attempted'] = True
    report['action_completed'] = False
    t = time.monotonic()
    r = ctx.value(tool, **args, observe_after=observe_after)
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
    return obs


def _jaw(ctx, obs, target, report):
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
              'code_status': 'R3 writable candidate; only individually reported functions rehearsed'}
    try:
        report['measured'] = operation(report)
        report['ok'] = True
        report['why'] = 'This function completed; this does not imply whole-task success'
    except NeedsReview as exc:
        report['needs_review'] = True
        report['status'] = 'needs_review'
        report['why'] = 'Missing scene evidence: ' + str(exc)
        report['review'] = exc.review
        report['resume'] = ('inspect completed result, then call next_function; never repeat this motion'
                            if report['executed'] else 'supply a real synchronous check before calling this function')
        # Known completed/settled movement is not a control fault. Do not add Hold
        # or re-execute it merely because the Agent must inspect the saved images.
    except Exception as exc:
        report['why'] = f'{type(exc).__name__}: {exc}'
        # Never release a potentially held block or replay a failed action.
        if report['dispatch_attempted']:
            try:
                report['hold'] = ctx.value('piper_hold')
            except Exception as stop_error:
                report['hold_error'] = str(stop_error)
                report['control_uncertain'] = True
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
            _, obs = _start(ctx, require_depth=True, motion=False)
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


def approach(ctx, *, waypoints, quaternion, assess=None):
    """Empty-arm approach to explicit scene-relative targets chosen for this run.

    assess('empty_approach_path') covers current obstacles, not just CAD self checks.
    Backend performs live IK/path/state checks; refusals return without blind retry.
    """
    def operation(report):
        robot, before = _start(ctx)
        points, q = _poses(robot, waypoints, quaternion)
        inputs = {'waypoints': points, 'quaternion': q}
        report['facts'].append(_fact(assess, 'empty_approach_path', before, before, inputs))
        after = before
        for i, p in enumerate(points):
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion_xyzw': q}, report, i==len(points)-1)
        report['facts'].append(_fact(assess, 'grasp_alignment', before, after, inputs))
        return after
    return _run(ctx, 'approach', operation, 'close_grasp')


def open_empty_gripper(ctx, *, assess=None, gap_m=OPEN_GAP_M):
    """Only after current evidence establishes empty fingers and clear opening sweep."""
    def operation(report):
        _, before = _start(ctx)
        if not 0 < gap_m <= 0.075:
            raise ValueError('Gap outside this task tested/opening range')
        report['facts'].append(_fact(assess, 'empty_opening_sweep', before, before, {'gap_m': gap_m}))
        return _jaw(ctx, before, gap_m, report)
    return _run(ctx, 'open_empty_gripper', operation, 'approach')


def close_grasp(ctx, *, assess=None):
    """Close a freshly verified engaged block; return before the trial-lift boundary."""
    def operation(report):
        _, before = _start(ctx)
        report['facts'].append(_fact(assess, 'engaged_supported_block', before, before,
                                    {'command_gap_m': GRASP_GAP_M}))
        return _jaw(ctx, before, GRASP_GAP_M, report)
    return _run(ctx, 'close_grasp', operation, 'trial_lift')


def trial_lift(ctx, *, quaternion, assess=None):
    """Lift from the CURRENT measured pose; no closure or grasp replay inside."""
    def operation(report):
        robot, before = _start(ctx)
        fk = ctx.value('piper_fk', joints=before['measured14'][:6])
        target = list(fk['position']); target[2] += TRIAL_LIFT_M
        _, q = _poses(robot, [target], quaternion)
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


def transport(ctx, *, waypoints, quaternion, assess=None):
    """Only with a retained block; path review includes payload and neighboring blocks."""
    def operation(report):
        robot, before = _start(ctx)
        points, q = _poses(robot, waypoints, quaternion)
        inputs = {'waypoints': points, 'quaternion': q}
        report['facts'].append(_fact(assess, 'retained_and_loaded_path', before, before, inputs))
        after = before
        for i, p in enumerate(points):
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion_xyzw': q}, report, i==len(points)-1)
        report['facts'].append(_fact(assess, 'retained_at_destination', before, after, inputs))
        return after
    return _run(ctx, 'transport', operation, 'lower_to_support')


def lower_to_support(ctx, *, descent, retreat, quaternion, assess=None):
    """Stop at support boundary; never open the jaws in this function."""
    def operation(report):
        robot, before = _start(ctx)
        down, q = _poses(robot, descent, quaternion)
        up, _ = _poses(robot, retreat, quaternion)
        inputs = {'descent': down, 'retreat': up, 'quaternion': q}
        report['facts'].append(_fact(assess, 'placement_and_retreat_path', before, before, inputs))
        after = before
        for i, p in enumerate(down):
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion_xyzw': q}, report, i==len(down)-1)
        report['facts'].append(_fact(assess, 'supported_before_release', before, after, inputs))
        return after
    return _run(ctx, 'lower_to_support', operation, 'release')


def release(ctx, *, assess=None):
    """Only a supported block; no automatic re-descent or retreat."""
    def operation(report):
        _, before = _start(ctx)
        report['facts'].append(_fact(assess, 'supported_before_release', before, before, {}))
        return _jaw(ctx, before, OPEN_GAP_M, report)
    return _run(ctx, 'release', operation, 'retreat')


def retreat(ctx, *, waypoints, quaternion, assess=None):
    def operation(report):
        robot, before = _start(ctx)
        points, q = _poses(robot, waypoints, quaternion)
        inputs = {'waypoints': points, 'quaternion': q}
        report['facts'].append(_fact(assess, 'released_and_retreat_path', before, before, inputs))
        after = before
        for i,p in enumerate(points):
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion_xyzw': q}, report, i==len(points)-1)
        report['facts'].append(_fact(assess, 'released_stable_and_clear', before, after, inputs))
        return after
    return _run(ctx, 'retreat', operation, 'inspect_task_result')


def place_release_retreat(ctx, **kwargs):
    return {'ok': False, 'executed': False, 'why': 'Use lower_to_support, release, retreat separately',
            'next_function': 'lower_to_support', 'status': 'split_function_entry'}


def empty_joint_waypoint(ctx, *, joints, assess=None):
    """One supervised empty transition, selected by seeded IK before this call.

    Fresh measured state anchors the native CAD/workspace check. The assessor must
    also establish empty fingers and scene clearance for the complete swept path.
    """
    def operation(report):
        _, before = _start(ctx)
        q = _vector(joints, 6)
        target = q + before['measured14'][6:]
        check = ctx.value('piper_check_path', start14=before['measured14'], target14=target)
        report['preflight'] = check
        if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:
            raise RuntimeError('Native path preflight did not establish CAD and workspace clearance')
        inputs = {'joints': q, 'path_check': check}
        report['facts'].append(_fact(assess, 'empty_joint_path', before, before, inputs))
        after = _move(ctx, 'piper_move_joints', {'joints': q}, report)
        return after
    return _run(ctx, 'empty_joint_waypoint', operation, 'approach')
