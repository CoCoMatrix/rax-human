"""R3 manual strategy distilled into RAX rehearsal functions, not a qualified main.

Units: metres/radians; poses control left_link6 in left_base_link.
An assessor supplies scene facts from the actual supplied observations. It may be
the rehearsing Agent or a later ordinary perception function; no Gemini is called
here. Robot tools retain their live checks. Importing this file never connects.
"""
import math
import time

GRASP_GAP_M = 0.050  # R3 command; observed feedback ~0.0509--0.0513, not pad spacing.
OPEN_GAP_M = 0.069   # R3 green/base release; initial orange opening was 0.073.
JAW_STEP_M = 0.009   # <= live 0.010 tool limit, calculated from measured feedback.
TRIAL_LIFT_M = 0.010 # This block run only; never inherit the cup's 40 mm setting.


def _vector(v, n):
    if len(v) != n or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in v):
        raise ValueError(f'Expected {n} finite values')
    return list(v)


def _fact(assess, phase, before, after, inputs):
    answer = assess(phase=phase, before=before, after=after, inputs=inputs)
    if not isinstance(answer, dict) or answer.get('ok') is not True:
        raise RuntimeError(f'{phase}: {answer}')
    if answer.get('frame_id') != after.get('frame_id') or not answer.get('why'):
        raise RuntimeError(f'{phase}: assessment must cite the supplied frame and reason')
    return answer


def _start(ctx):
    state = ctx.value('status')
    if state.get('control_uncertain') is not False:
        raise RuntimeError('Unknown control state; resolve with native status/hold before resuming')
    robot = state['robot_status']
    if robot.get('controlled_link') != 'left_link6' or robot.get('frame') != 'left_base_link':
        raise RuntimeError('Unexpected pose contract')
    obs = ctx.value('piper_observe', cameras=['head', 'left', 'right'], require_depth=False)
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


def _move(ctx, tool, args, report):
    # Record attempted dispatch before calling: a transport error is ambiguous.
    report['dispatch_attempted'] = True
    t = time.monotonic()
    r = ctx.value(tool, **args, observe_after=True)
    report['receipts'].append({'tool': tool, 'arguments': args,
                               'elapsed_s': time.monotonic()-t, 'result': r})
    report['executed'] |= r.get('executed') is True
    if not all(r.get(k) is True for k in ('executed', 'measured_arrival', 'measured_settled')):
        raise RuntimeError('Motion receipt did not confirm execution, arrival and settling')
    obs = r['observation']
    _vector(obs['measured14'], 14)
    return obs


def _jaw(ctx, obs, target, report):
    # Bounded feedback-based stepping; cannot silently spin on a stale jaw value.
    final_sent = False
    for _ in range(12):
        current = obs['measured14'][6]
        if final_sent and abs(target-current) <= 0.0015:
            return obs
        command = current + max(-JAW_STEP_M, min(JAW_STEP_M, target-current))
        after = _move(ctx, 'piper_gripper', {'width_m': command}, report)
        final_sent = abs(command-target) < 1e-10
        if final_sent and abs(target-after['measured14'][6]) <= 0.0015:
            return after
        if abs(after['measured14'][6]-current) < 0.0001:
            raise RuntimeError('Jaw did not progress; do not repeat the closure blindly')
        obs = after
    raise RuntimeError('Jaw step budget exhausted')


def _run(ctx, stage, operation):
    t = time.monotonic()
    report = {'ok': False, 'stage': stage, 'why': '', 'executed': False,
              'dispatch_attempted': False, 'receipts': [], 'facts': [],
              'code_status': 'offline-validated; physical function rehearsal pending'}
    try:
        report['measured'] = operation(report)
        report['ok'] = True
        report['why'] = 'This function completed; this does not imply whole-task success'
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


def locate_block(ctx, observation, interior_pixels):
    """Pixels must have been selected on this exact fresh observation, not old RGB.

    Semantic selection stays with the rehearsing Agent at this stage. Native depth
    windows reject invalid samples; wrist optical points never become base targets.
    """
    return ctx.value('piper_pixel_to_position', camera='head',
                     frame_id=observation['frame_id'], pixels=interior_pixels, radius=3)


def approach(ctx, *, waypoints, quaternion, assess):
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
        for p in points:
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion': q}, report)
        report['facts'].append(_fact(assess, 'grasp_alignment', before, after, inputs))
        return after
    return _run(ctx, 'approach', operation)


def open_empty_gripper(ctx, *, assess, gap_m=OPEN_GAP_M):
    """Only after current evidence establishes empty fingers and clear opening sweep."""
    def operation(report):
        _, before = _start(ctx)
        if not 0 < gap_m <= 0.075:
            raise ValueError('Gap outside this task tested/opening range')
        report['facts'].append(_fact(assess, 'empty_opening_sweep', before, before, {'gap_m': gap_m}))
        return _jaw(ctx, before, gap_m, report)
    return _run(ctx, 'open_empty_gripper', operation)


def grasp_and_trial_lift(ctx, *, quaternion, assess):
    """Start at freshly verified engagement height. Close to 50 mm, trial lift 10 mm.

    The 205 mm flange height was for R3's particular table; it is NOT embedded.
    The caller obtains the current object/table relation before approach.
    """
    def operation(report):
        robot, before = _start(ctx)
        fk = ctx.value('piper_fk', joints=before['measured14'][:6])
        target = list(fk['position']); target[2] += TRIAL_LIFT_M
        _, q = _poses(robot, [target], quaternion)
        report['facts'].append(_fact(assess, 'engaged_supported_block', before, before, {'target': target}))
        closed = _jaw(ctx, before, GRASP_GAP_M, report)
        lifted = _move(ctx, 'piper_move_pose', {'position': target, 'quaternion': q}, report)
        report['facts'].append(_fact(assess, 'retained_after_trial', closed, lifted,
                                    {'lift_m': TRIAL_LIFT_M, 'command_gap_m': GRASP_GAP_M}))
        return lifted
    return _run(ctx, 'grasp_and_trial_lift', operation)


def transport(ctx, *, waypoints, quaternion, assess):
    """Only with a retained block; path review includes payload and neighboring blocks."""
    def operation(report):
        robot, before = _start(ctx)
        points, q = _poses(robot, waypoints, quaternion)
        inputs = {'waypoints': points, 'quaternion': q}
        report['facts'].append(_fact(assess, 'retained_and_loaded_path', before, before, inputs))
        after = before
        for p in points:
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion': q}, report)
        report['facts'].append(_fact(assess, 'retained_at_destination', before, after, inputs))
        return after
    return _run(ctx, 'transport', operation)


def place_release_retreat(ctx, *, descent, retreat, quaternion, assess):
    """Fresh target after magnetic movement; support check before release.

    Review/preflight retreat as part of the supplied route; R3's z=.330 withdrawal
    failed IK. No generic 'up is reachable' assumption and no automatic recovery.
    """
    def operation(report):
        robot, before = _start(ctx)
        down, q = _poses(robot, descent, quaternion)
        up, _ = _poses(robot, retreat, quaternion)
        inputs = {'descent': down, 'retreat': up, 'quaternion': q}
        report['facts'].append(_fact(assess, 'placement_and_retreat_path', before, before, inputs))
        after = before
        for p in down:
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion': q}, report)
        report['facts'].append(_fact(assess, 'supported_before_release', before, after, inputs))
        after = _jaw(ctx, after, OPEN_GAP_M, report)
        for p in up:
            after = _move(ctx, 'piper_move_pose', {'position': p, 'quaternion': q}, report)
        report['facts'].append(_fact(assess, 'released_stable_and_clear', before, after, inputs))
        return after
    return _run(ctx, 'place_release_retreat', operation)
