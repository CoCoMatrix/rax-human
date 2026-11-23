"""Read-only R8 between-function calculations for controller.main.

No Context, sockets, archived input loading, motion dispatch or automatic recovery.
Constants are R8 posture/relative choices; see README for evidence and limits.
Every executed boundary still passes the original function's full checks.
"""
import math
import numpy as np
import controller as c
from scene_checks import (read_report, completed_action_state, measure_scene,
                          scene_control, resolve_empty_scene, PerceptionUnknown)

# R8 initial-plan / transition-01..03: tilted top+235mm, then down top+215mm.
# Only orientations and dimensionless interpolation are reused, never joints/XYZ.
ORANGE_DOWN = [math.cos(math.radians(1.5)), -math.sin(math.radians(1.5)), 0., 0.]
GREEN_DOWN = [math.sqrt(.5), -math.sqrt(.5), 0., 0.]
INSPECTION_TURN = [math.cos(math.pi/8), -math.sin(math.pi/8), 0., 0.]
ENTRY_ORIENTATION_FRACTIONS = (.65, .45, .30)  # all read-only IK-tested in R8
ENTRY_JOINT_FRACTION = .65  # R8's two distinct tilted transition boundaries
ORANGE_TILTED_HEIGHT = .235
ORANGE_HOVER_HEIGHT = .215
GREEN_HOVER_HEIGHT = .195  # preferred R8 height, not a mandatory target
from green_checks import PREGRASP_HEIGHT_RANGE as GREEN_HOVER_RANGE
GREEN_HOVER_RESOLUTION = .0005  # read-only search resolution, not an arrival tolerance
INSERTION_HEIGHT = .128   # both R8 final insertions relative to observed block top
APPROACH_STEP = .020      # R8 20mm segments and final 7mm remainder
INSPECTION_OFFSET = .100  # R8 high inspection: green Y+100mm


def _current(ctx, report, previous_report, *, depth=False):
    robot, obs = c._start(ctx, report=report, require_depth=depth, motion=False)
    prev = completed_action_state(read_report(previous_report), obs, robot)
    if prev.get('ok') is not True:
        raise PerceptionUnknown('Input calculation requires a successful predecessor')
    fk = ctx.value('piper_fk', joints=obs['measured14'][:6])
    return robot, obs, prev, fk


def _slerp(a, b, fraction):
    a, b = np.array(a, float), np.array(b, float)
    dot = float(a @ b)
    if dot < 0: b, dot = -b, -dot
    angle = math.acos(min(1., dot))
    value = ((1-fraction)*a + fraction*b if angle < 1e-8 else
             (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b)/math.sin(angle))
    return (value/np.linalg.norm(value)).tolist()


def _ik_path(ctx, start, position, quaternion, robot):
    c._poses(robot, [position], quaternion)
    sol = ctx.value('piper_solve_ik', position=position, quaternion_xyzw=quaternion, seed=start[:6])
    if sol.get('ok') is not True:
        raise PerceptionUnknown('No full-pose IK for scene-derived candidate')
    end = c._vector(sol['q'], 6)
    check = ctx.value('piper_check_path', start14=start, target14=end+start[6:])
    if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:
        raise PerceptionUnknown('Candidate native CAD/workspace check refused')
    return end, check


def _planning_refusal(exc):
    """Only explicit native read-only refusals permit trying another candidate."""
    import json
    try:
        text = str(exc)
        error = json.loads(text[text.index('{'):])['error']
        return (error.get('recovery_action') == 'replan_read_only'
                and error.get('control_uncertain') is False
                and error.get('no_action_executed') is True
                and error.get('dispatch_started') is False)
    except (ValueError, KeyError, TypeError):
        return False


def initial_state(ctx):
    """Guard native neutral/Prepare before any opening; fresh four-block scene."""
    from scene_checks import empty_opening
    def operation(report):
        robot, obs = c._start(ctx, report=report, require_depth=True, motion=False)
        q = obs['measured14']
        # Same startup bounds as the rehearsed empty-opening checker. A 50mm
        # held jaw, open prior-task pose, or unknown feedback is NOT a fresh run.
        if max(abs(x) for x in q[:6]) > .01 or not 0 <= q[6] <= .005:
            raise PerceptionUnknown('Fresh main requires native neutral/Prepare and closed empty jaw; inspect held state outside main')
        fact = c._fact(lambda **kw: empty_opening(ctx, kw['after'], c.OPEN_GAP_M),
                       'fresh_empty_start', obs, obs, {})
        scene_control(fact['scene'], robot, establish=True)
        report['facts'].append(fact)
        return obs
    return c._run(ctx, 'initial_state', operation, 'open_empty_gripper')


def orange_entry(ctx, *, previous_report):
    """Choose three fresh joint waypoints using bounded R8 posture candidates."""
    def operation(report):
        robot, obs, prev, fk = _current(ctx, report, previous_report, depth=True)
        if prev['stage'] != 'open_empty_gripper':
            raise PerceptionUnknown('Orange entry must follow initial empty opening')
        scene = measure_scene(ctx, obs)
        scene_control(scene, robot, establish=True)
        top = scene['blocks']['orange']['top_center']['surface_position_m']
        high = [*top[:2], top[2]+ORANGE_TILTED_HEIGHT]
        down = [*top[:2], top[2]+ORANGE_HOVER_HEIGHT]
        start = obs['measured14']; rows = []; report['candidates'] = rows
        # Prefer the rehearsed height; bounded lower alternatives keep the
        # exact scene XY and downward grasp orientation. Never relax guards.
        ceiling=min(ORANGE_HOVER_HEIGHT,robot['workspace_max'][2]-top[2]-.020)
        floor=max(INSERTION_HEIGHT+.020,obs.get('finger_geometry',{}).get('tip_axial_m',.170)+.020)
        if ceiling<floor:raise PerceptionUnknown('No hover height interval inside workspace and fingertip envelope')
        heights=list(np.arange(ceiling,floor,-.010))+[floor]
        for height in dict.fromkeys(float(h) for h in heights):
            down=[*top[:2],top[2]+height]
            high=[*top[:2],top[2]+height+.020]
            for fraction in ENTRY_ORIENTATION_FRACTIONS:
                row = {'hover_height_m':height,'phase':'tilted_ik','orientation_fraction': fraction, 'position': high, 'down_position': down}
                rows.append(row)
                try:
                    q = _slerp(fk['quaternion_xyzw'], ORANGE_DOWN, fraction)
                    tilted, check = _ik_path(ctx, start, high, q, robot)
                    delta=np.array(tilted)-start[:6]
                    split=min(ENTRY_JOINT_FRACTION,.95*(math.pi/3)/max(float(np.max(abs(delta))),1e-9))
                    middle = (np.array(start[:6]) + split*delta).tolist()
                    row['joint_split_fraction']=split
                    row['phase']='downward_hover_ik'
                    end, final_check = _ik_path(ctx, tilted+start[6:], down, ORANGE_DOWN, robot)
                    row['phase']='segmented_path'
                    waypoints = [middle, tilted, end]; anchor = start; checks = []
                    for joints in waypoints:
                        if max(abs(a-b) for a,b in zip(joints,anchor[:6])) > math.pi/3:
                            raise PerceptionUnknown('Candidate needs a >60deg empty segment')
                        check = ctx.value('piper_check_path', start14=anchor, target14=joints+start[6:])
                        if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:
                            raise PerceptionUnknown('Candidate transition path refused')
                        checks.append(check); anchor = joints+start[6:]
                    if height != ORANGE_HOVER_HEIGHT:
                        # A changed hover must retain the full empty route and the
                        # following insertion, not merely make endpoint IK succeed.
                        from scene_checks import installed_finger_corners
                        from empty_transition import geometry as empty_geometry
                        from continuous_approach import geometry as insertion_geometry
                        T=np.asarray(fk['T_base_flange'])
                        local=(installed_finger_corners(ctx,obs,scene,fk)-T[:3,3])@T[:3,:3]
                        anchor=start
                        row['phase']='adaptive_empty_envelope'
                        for joints in waypoints:
                            empty_geometry(ctx,anchor,joints,scene,local,robot)
                            anchor=joints+start[6:]
                        row['phase']='adaptive_insertion'
                        target=[*top[:2],top[2]+INSERTION_HEIGHT]
                        solution=ctx.value('piper_solve_ik',position=target,quaternion_xyzw=ORANGE_DOWN,seed=end)
                        if solution.get('ok') is not True:raise PerceptionUnknown('No insertion IK from adaptive hover')
                        row['insertion_check']=insertion_geometry(ctx,anchor,solution['q'],scene,'orange',local,robot=robot)
                    row['phase']='selected'
                    row.update(ok=True, joint_waypoints=waypoints, checks=checks,
                               down_preflight=final_check)
                    report['joint_waypoints'] = waypoints
                    report['scene'] = scene
                    return obs
                except PerceptionUnknown as exc:
                    row.update(ok=False, why=str(exc))
                except RuntimeError as exc:
                    row.update(ok=False, why=str(exc))
                    if not _planning_refusal(exc): raise
        raise PerceptionUnknown('No bounded orange entry candidate; '+ '; '.join(dict.fromkeys(r['phase']+': '+r.get('why','') for r in rows))[:1400]+'; no motion dispatched by planner')
    return c._run(ctx, 'orange_entry_inputs', operation, 'empty_joint_waypoint')


def approach_inputs(ctx, *, previous_report, color):
    def operation(report):
        robot, obs, prev, fk = _current(ctx, report, previous_report)
        scene = resolve_empty_scene(previous_report, obs, robot=robot, color=color,
            **({'initial_scene_report':previous_report, 'empty_history':[previous_report]} if color == 'green' else {}))
        p = fk['position']; goal = scene['blocks'][color]['top_center']['surface_position_m'][2]+INSERTION_HEIGHT
        n = math.ceil(round((p[2]-goal)/APPROACH_STEP, 10))
        if not 1 <= n <= math.ceil(.400/APPROACH_STEP) or p[2] <= goal:
            raise PerceptionUnknown('Insertion exceeds native400mm translation bound or does not descend')
        points = [[p[0],p[1],max(goal,p[2]-(i+1)*APPROACH_STEP)] for i in range(n)]
        report['inputs'] = {'previous_report':previous_report, 'color':color, 'engage':True,
                            'waypoints':points, 'quaternion':fk['quaternion_xyzw']}
        if color == 'green':
            report['inputs'].update(initial_scene_report=previous_report, empty_history=[previous_report])
        return obs
    return c._run(ctx, color+'_approach_inputs', operation, 'approach')


def pose_inputs(ctx, *, previous_report, mode, color='orange', support_report=None,
                placement_plan_report=None):
    """Small measured-FK calculations; modes correspond to existing R8 boundaries."""
    def operation(report):
        robot, obs, prev, fk = _current(ctx, report, previous_report)
        p = list(fk['position']); q = fk['quaternion_xyzw']
        inputs = {'previous_report':previous_report, 'quaternion':q}
        if mode == 'trial' or mode == 'base_plan':
            if mode == 'trial': inputs['color'] = color
        elif mode in ('orange_raise','green_raise_1','green_raise_2'):
            from lift_plan import choose_lift
            try:lift=choose_lift(ctx,obs,robot,prev,previous_report,mode,color,support_report)
            except PerceptionUnknown as exc:
                if hasattr(exc,'lift_diagnostics'):report['lift_plan']=exc.lift_diagnostics
                raise
            p=lift['selected']['position'];report['lift_plan']=lift
            inputs.update(waypoints=[p], color=color)
            if support_report: inputs['support_report'] = support_report
        elif mode == 'orange_transfer':
            plan = read_report(placement_plan_report)['result']['placement_plan']['selected']
            p[:2] = plan['transfer'][:2]
            inputs.update(waypoints=[p], color=color)
        elif mode in ('orange_lower_1','orange_lower_2'):
            # Nominal table height is evaluated at the actual arrived XY.
            fact = prev['facts'][0]; plane = fact['scene']['table']['z_ax_by_c']
            bottom_offset = fact['payload_origin']['flange_to_bottom_m']
            final_z = p[0]*plane[0]+p[1]*plane[1]+plane[2]+bottom_offset
            target = [p[0],p[1],(p[2]+final_z)/2 if mode.endswith('_1') else final_z]
            # Rebind to actual arrived XY/height. The second descent retains
            # the first descent's checked higher clearance, not the old plan.
            clearance_z=p[2]
            if mode=='orange_lower_2':
                prior=fact.get('retreat_position')
                if prior is None:
                    prior=read_report(previous_report)['inputs']['retreat'][0]
                clearance_z=max(clearance_z,prior[2])
            inputs.update(descent=[target], retreat=[[p[0],p[1],clearance_z]],
                          support_required=mode.endswith('_2'))
        elif mode in ('orange_retreat','green_retreat'):
            release = read_report(previous_report)
            lower = read_report(release['inputs']['previous_report'])
            # Return to this run's checked clearance, not a fixed additional rise.
            checked=lower['result']['facts'][0].get('retreat_position',lower['inputs']['retreat'][0])
            p[2] = checked[2]
            inputs['waypoints'] = [p]
        else:
            raise ValueError('Unknown R8 pose calculation: '+mode)
        report['inputs'] = inputs
        return obs
    return c._run(ctx, mode+'_inputs', operation)


def inspection_inputs(ctx, *, previous_report, turn):
    from history_scene import released_scene
    def operation(report):
        robot, obs, prev, fk = _current(ctx, report, previous_report)
        scene = released_scene(ctx, obs, robot, previous_report)
        g = scene['blocks']['green']['top_center']['surface_position_m']
        p = [g[0],g[1]+INSPECTION_OFFSET,fk['position'][2]] if turn == 1 else fk['position']
        q = INSPECTION_TURN if turn == 1 else GREEN_DOWN
        _ik_path(ctx, obs['measured14'], p, q, robot)
        report['inputs'] = {'position':p,'quaternion':q,'previous_report':previous_report}
        return obs
    return c._run(ctx, 'inspection_inputs', operation, 'empty_inspection')


def green_hover_inputs(ctx, *, previous_report, stationary_scene_report):
    from green_checks import GreenPregrasp,planned_green_path
    from green_route import choose,insertion_preview
    from scene_checks import same_state
    def operation(report):
        robot,obs,prev,fk=_current(ctx,report,previous_report)
        checker=GreenPregrasp(ctx,previous_report,stationary_scene_report)
        evidence=checker.prepare(obs)  # real evidence/history validated ONCE
        args,fact=choose(ctx,evidence,robot,report)  # numerical-only search
        # Search time is not image age permission. Obtain actual current evidence
        # once, bind to the same completed action/owner, then recheck chosen path.
        original_identity=(robot['bridge']['instance'],obs['state_binding']['owner_generation'])
        robot2,fresh=c._start(ctx,motion=False,require_depth=False,force_observe=True)
        if (robot2['bridge']['instance'],fresh['state_binding']['owner_generation'])!=original_identity:
            raise PerceptionUnknown('Owner changed during read-only green route search')
        if not same_state(obs['measured14'],fresh['measured14']):
            raise PerceptionUnknown('Robot moved during green route search; replan from actual state')
        rebound=checker.prepare(fresh)
        if (evidence['scene']['blocks']!=rebound['scene']['blocks'] or
            evidence['tcp_offset_flange']!=rebound['tcp_offset_flange']):
            raise PerceptionUnknown('Scene targets changed during green route search; replan')
        fact=planned_green_path(ctx,rebound['start'],rebound['scene'],rebound['local'],args)
        onward=insertion_preview(ctx,rebound,args['waypoints'][0],args['quaternion'],fact['end_joints'])
        report['facts'].append(dict(fact,frame_id=fresh['frame_id'],scene=rebound['scene']))
        report['route_plan']['rebind']={'source_frame':obs['frame_id'],'current_frame':fresh['frame_id'],
                                      'same_completed_action':True,'current_path_rechecked':True}
        report['route_plan']['rebound_insertion_preview']=onward
        report['inputs']={'position':args['waypoints'][0],'quaternion':args['quaternion'],
            'previous_report':previous_report,'stationary_scene_report':stationary_scene_report}
        report['selected_height_m']=report['route_plan']['selected']['height_above_top_m']
        report['observation_source']='fresh_after_numerical_search'
        return fresh
    return c._run(ctx,'green_hover_inputs',operation,'green_pregrasp')


def green_transfer_inputs(ctx, *, previous_report, support_report, remaining_segments):
    from stack_checks import loaded_state, stack_target
    def operation(report):
        robot, obs, prev, fk = _current(ctx, report, previous_report)
        _, scene, origin = loaded_state(ctx, previous_report, support_report, obs)
        destination = stack_target(scene, origin) # actual pickup relation, refreshed red
        if remaining_segments not in (1,2,3): raise ValueError('Three rehearsed lateral boundaries')
        p = np.array(fk['position'])
        if remaining_segments == 3:
            # Preserve R8's first lateral leg, before the second lift: current
            # X/Z and measured placed-orange Y minus 105 mm. Do not replace
            # the rehearsed obstacle-relative detour with a diagonal shortcut.
            p[1] = scene['blocks']['orange']['top_center']['surface_position_m'][1] - .105
        else:
            p[:2] += (np.array(destination[:2])-p[:2])/remaining_segments
        report['inputs'] = {'previous_report':previous_report,'support_report':support_report,
            'color':'green','waypoints':[p.tolist()],'quaternion':fk['quaternion_xyzw']}
        report['stack_target'] = destination
        return obs
    return c._run(ctx, 'green_transfer_inputs', operation, 'transport')


def green_lower_inputs(ctx, *, previous_report, support_report):
    from stack_checks import loaded_state, stack_target
    def operation(report):
        robot, obs, prev, fk = _current(ctx, report, previous_report)
        _, scene, origin = loaded_state(ctx, previous_report, support_report, obs)
        target = stack_target(scene, origin)
        p = fk['position']
        # XY comes from the actual arrived pose. Existing stack geometry checks
        # the measured block/flange offset and red-centered support corridor.
        report['inputs'] = {'previous_report':previous_report,'support_report':support_report,
            'descent':[[p[0],p[1],target[2]]], 'retreat':[p],
            'quaternion':fk['quaternion_xyzw'],'support_required':True}
        return obs
    return c._run(ctx, 'green_lower_inputs', operation, 'green_stack_lower')
