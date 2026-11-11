"""Inspected one-step maneuvers distilled from r12. No task FSM or replay.

All targets refer to left_link6 flange. Scene checks are injected explicitly;
these primitives alone are NOT an unattended cup controller. A check receives a
saved observation and must return {'ok': bool, 'frame_id': ..., 'reason': ...}.
No check may replace hidden support with jaw arrival or an old scene coordinate.
Callbacks may use deterministic perception where sufficient; this module itself
does not call an auxiliary model or require one before/after every motion.
"""
import json
import math
from pathlib import Path

FINISH_RESERVE_S = 60
MAX_STEP_M = {'lift': .04, 'transfer': .055, 'lower': .02, 'retreat': .07}
MAX_JAW_STEP_M = .010


def decode(reply):
    return json.loads(next(b['text'] for b in reply['content'] if b['type'] == 'text'))


def inspect(ctx):
    """Read-only, all three views and current measured state."""
    state = ctx.value('status')
    reply = ctx.call('piper_observe', cameras=['head', 'left', 'right'], require_depth=True)
    return {'state': state, 'observation': decode(reply), 'images': ctx.images(reply)}


def checked_view(check, observation):
    if not callable(check):
        return {'ok': False, 'reason': 'A current scene pre/postcondition checker is required'}
    verdict = check(observation)
    if not isinstance(verdict, dict) or verdict.get('frame_id') != observation.get('frame_id'):
        return {'ok': False, 'reason': 'Checker did not identify the supplied current frame'}
    return verdict


def fn(ctx, *, maneuver, precheck, postcheck, delta=None, width_m=None):
    """One already-understood action, with explicit scene checks and receipt checks.

    lift: retained wall, overhead clear; postcheck cup follows without slipping.
    transfer: retained cup and base clearance, numerical path check plus visible
      obstacles; RGB does not certify the entire swept volume. Postcheck retained.
    lower: cup base footprint inside coaster and remaining gap known; postcheck
      alignment and contact/settling. Maximum20mm, use3mm near support.
    release: COMPLETE base support and stable upright cup required; postcheck
      cup stationary and jaw separation. Opening alone never proves support.
    grasp: empty jaw straddles supported cup wall below rim, handle/table clear;
      postcheck wall remains between pads. A separate lift proves retention.
    retreat: released, supported cup and clear path; postcheck separation/stability.

    Returns after one action so contact boundaries must be inspected. No retry,
    automatic recovery, home, shutdown release, or right-arm command.
    Numeric clearance/arrival belongs to geometry/feedback. Ask image callbacks
    only for relevant visible facts, never hidden undersides or no transient contact
    from endpoint frames. Scoped onsite evidence is distinct from image evidence;
    pre-motion confirmation never establishes a post-motion fact.
    """
    report = {'ok': False, 'maneuver': maneuver, 'executed': False}
    try:
        current = inspect(ctx)
        state, obs = current['state'], current['observation']
        report['before_frame'] = obs['frame_id']
        if state.get('control_uncertain'):
            return {**report, 'reason': 'Unknown control: stop, never replay'}
        if obs['wall_budget']['remaining_seconds'] <= FINISH_RESERVE_S:
            return {**report, 'reason': 'Finish reserve: no motion'}
        robot = state['robot_status']
        if not robot.get('motion_enabled') or not robot.get('arming', {}).get('armed'):
            return {**report, 'reason': 'Motion is not enabled and armed'}
        verdict = checked_view(precheck, obs)
        if verdict.get('ok') is not True:
            return {**report, 'reason': 'Precondition failed', 'check': verdict}
        q = obs['measured14']
        right = q[7:]
        if maneuver in ('grasp', 'release'):
            if width_m is None or not math.isfinite(width_m) or not 0 <= width_m <= .075:
                return {**report, 'reason': 'Invalid jaw target'}
            if abs(width_m - q[6]) > MAX_JAW_STEP_M + 1e-9:
                return {**report, 'reason': 'Jaw increment exceeds10mm'}
            if maneuver == 'release' and width_m <= q[6]:
                return {**report, 'reason': 'Release must open'}
            if maneuver == 'grasp' and width_m >= q[6]:
                return {**report, 'reason': 'Grasp must close'}
            tool, args = 'piper_gripper', {'width_m': width_m}
        elif maneuver in MAX_STEP_M:
            if delta is None or len(delta) != 3 or not all(math.isfinite(x) for x in delta):
                return {**report, 'reason': 'Supply measured relative displacement'}
            if math.dist(delta, [0, 0, 0]) > MAX_STEP_M[maneuver] + 1e-9:
                return {**report, 'reason': 'Step exceeds distilled maneuver range'}
            if maneuver in ('lift', 'lower'):
                if abs(delta[0]) + abs(delta[1]) > 1e-9:
                    return {**report, 'reason': 'Contact-stage lift/lower must be vertical'}
                if (maneuver == 'lift' and delta[2] <= 0) or (maneuver == 'lower' and delta[2] >= 0):
                    return {**report, 'reason': 'Wrong vertical direction'}
            if maneuver == 'transfer' and abs(delta[2]) > 1e-9:
                return {**report, 'reason': 'Transfer at verified clearance height'}
            fk = ctx.value('piper_fk', joints=q[:6], arm='left')
            target = [a+b for a,b in zip(fk['position'], delta)]
            tool, args = 'piper_move_pose', {'position': target}
        else:
            return {**report, 'reason': 'Unknown maneuver'}
        # Omit duration: sole owner chooses checked safe timing. Preserve orientation.
        report.update(executed=None, dispatch_attempted=True)
        reply = ctx.call(tool, observe_after=True, **args)
        receipt = decode(reply)
        report.update(executed=receipt.get('executed', False), receipt=receipt)
        if receipt.get('executed') is not True or receipt.get('measured_arrival') is not True:
            return {**report, 'reason': 'Unconfirmed motion: STOP; never replay'}
        if any(abs(a-b) > .01 for a,b in zip(right[:6], receipt['final14'][7:13])) or abs(right[6]-receipt['final14'][13])>.003:
            return {**report, 'reason': 'Unexpected right-arm state: STOP'}
        after = receipt.get('observation')
        if not after:
            # Successful move with missing images: observation only, no replay.
            reply = ctx.call('piper_observe', cameras=['head','left','right'], require_depth=True)
            after = decode(reply)
        report['after_frame'] = after['frame_id']
        if set(after.get('cameras', {})) != {'head','left','right'}:
            return {**report, 'reason': 'Missing camera: observe only'}
        verdict = checked_view(postcheck, after)
        return {**report, 'ok': verdict.get('ok') is True, 'check': verdict,
                'reason': verdict.get('reason', ''), 'image_paths': {
                    k:v['inspection_image'] for k,v in after['cameras'].items()}}
    except Exception as exc:
        # No implicit jaw release on exception or Context shutdown.
        return {**report, 'reason': str(exc), 'next': 'Stop and inspect status/outcome; never replay'}


def localize(ctx, frame_id, pixels, radius=3):
    """Supported saved head depth only; caller rejects background and silhouette samples."""
    return ctx.value('piper_pixel_to_position', camera='head', frame_id=frame_id,
                     pixels=pixels, radius=radius)


def main(ctx):
    """Read-only entrypoint. Full cup-task composition awaits a fresh reset/test."""
    current = inspect(ctx)
    return {'ok': True, 'mode': 'read_only', 'frame_id': current['observation']['frame_id'],
            'wall_budget': current['observation']['wall_budget'],
            'note': 'No motion performed. fn requires current scene-specific pre/postcheck functions.'}


if __name__ == '__main__':
    from api.runtime import Context
    with Context() as ctx:
        print(json.dumps(main(ctx), indent=2))
