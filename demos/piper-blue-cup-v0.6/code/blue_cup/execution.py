"""State-driven task motions on the existing MCP tools; no camera/agent loop.

The task owns when to observe. Initial geometry is a static-scene assumption,
not a fresh image or proof of gripping/support. Meaningful object outcomes are
checked separately. A changed owner, intervening action or failure ends the run.
"""
import json,math,time
from pathlib import Path
import numpy as np
from perception import OUT,observation_pose_matches
from checks import modeled_obstacles


def state(ctx, reference):
    status=ctx.value('status');robot=status['robot_status']
    if (status.get('control_uncertain') or not robot.get('motion_ready')
            or not robot.get('motion_enabled') or not robot.get('arming',{}).get('armed')
            or robot['bridge']['phase'] not in ('IDLE','DONE')
            or robot['bridge']['instance']!=reference['bridge_instance']
            or robot['owner_status']['generation']!=reference['owner_generation']
            or robot.get('last_motion')!=reference.get('last_motion')
            or not observation_pose_matches(reference['expected14'],robot['measured_state']['values'])):
        raise ValueError('Scheduled execution state/owner/action changed; stop and inspect')
    if status['wall_budget']['remaining_seconds']<=90:raise ValueError('Finish reserve')
    return robot


def check_workspace_goal(position,robot):
    """Reject an impossible subgoal before any preparation/gripper motion."""
    low=np.asarray(robot.get('workspace_min'),float);high=np.asarray(robot.get('workspace_max'),float)
    p=np.asarray(position,float)
    if low.shape!= (3,) or high.shape!=(3,) or p.shape!=(3,) or not np.isfinite([low,high,p]).all() or not np.all(low<high):
        raise ValueError('Current robot flange workspace is unavailable')
    for axis in range(3):
        if p[axis]<low[axis] or p[axis]>high[axis]:
            raise ValueError(f'Preparation goal outside flange workspace: {"XYZ"[axis]}={p[axis]*1000:.3f} mm, '
                             f'allowed [{low[axis]*1000:.1f}, {high[axis]*1000:.1f}] mm; no preparation motion dispatched')


def move(ctx, inputs, reference):
    """One bounded numeric maneuver; callers explicitly request images if needed.

    Keep existing IK, whole-path collision checks, arrival and settling tolerances.
    No implicit image acquisition or model call; no automatic action replay.
    """
    start=time.monotonic();r=dict(function=inputs['name'],ok=False,executed=False,
        motion_arrived=False,visual_calls=0,wrist_depth_calls=0,perception_s=0,
        planning_s=0,motion_s=0,geometry_s=0,ik_calls=0,after_frame=None,
        observation_policy='task_scheduled',scene_source_frame=reference['source_frame'],
        scene_assumption='Known objects remain stationary until intentionally grasped; object outcomes checked at task boundaries')
    try:
        robot=state(ctx,reference);q=list(robot['measured_state']['values'])
        fk=ctx.value('piper_fk',joints=q[:6],arm='left');r['before_fk']=fk
        args=dict(inputs['args']);tool=inputs['tool']
        if tool=='piper_move_pose' and 'delta' in args:
            args['position']=(np.asarray(fk['position'])+args.pop('delta')).tolist()
        candidates=inputs.get('pose_candidates',[args]);selected=None;workspace_refusals=[]
        if not 1<=len(candidates)<=3:raise ValueError('Bounded candidate count required')
        obstacles=modeled_obstacles(reference['source_observation'])+inputs.get('obstacles',[])
        if not obstacles:raise ValueError('Scheduled execution requires the measured table model')
        for candidate in candidates:
            candidate=dict(candidate);target=list(q);t=time.monotonic()
            if tool=='piper_move_pose':
                if math.dist(candidate['position'],fk['position'])>.400000001:raise ValueError('Pose goal exceeds400mm')
                r['ik_calls']+=1
                try:
                    solved=ctx.value('piper_solve_ik',position=candidate['position'],
                        quaternion_xyzw=candidate.get('quaternion_xyzw',fk['quaternion_xyzw']),seed=q[:6])
                except RuntimeError as exc:
                    try:error=json.loads(str(exc).split('MCP tool failed: ',1)[1])['error']
                    except (ValueError,KeyError,IndexError,TypeError):raise exc
                    if not (error.get('no_action_executed') is True and error.get('stage')=='validation'
                            and 'MOVEIT_NO_IK_SOLUTION' in error.get('reason','')):raise
                    continue
                finally:r['planning_s']+=time.monotonic()-t
                target[:6]=solved['q'];dispatch='piper_move_joints';command={'joints':solved['q']}
            elif tool in ('piper_gripper','piper_close_gripper'):
                if not math.isfinite(candidate['width_m']) or (tool=='piper_gripper' and abs(candidate['width_m']-q[6])>.010001):
                    raise ValueError('Jaw step exceeds10mm')
                target[6]=candidate['width_m'];dispatch=tool;command={'width_m':candidate['width_m']}
            else:raise ValueError('Unsupported scheduled motion')
            if max(abs(a-b) for a,b in zip(target[:6],q[:6]))>math.radians(60)+1e-9:
                continue  # Pure planning rejection, not an executed-action retry.
            t=time.monotonic();geometry=ctx.value('piper_check_path',start14=q,target14=target,obstacles=obstacles)
            r['geometry_s']+=time.monotonic()-t
            if geometry.get('geometry_identity')!=reference['geometry_identity']:
                raise ValueError('Collision/initial geometry identity changed')
            if geometry.get('workspace_clear') is not True:
                workspace_refusals.append(geometry.get('workspace_check',{}).get('reason','workspace preflight unavailable'))
                continue
            if geometry.get('cad_clear') and geometry.get('whole_path_checked'):
                selected=(candidate,dispatch,command,geometry);break
        if selected is None:raise ValueError('; '.join(workspace_refusals) if workspace_refusals else 'No reachable clear modeled path')
        candidate,dispatch,command,geometry=selected
        current=state(ctx,reference)
        if not observation_pose_matches(q,current['measured_state']['values']):
            raise ValueError('Start state changed during planning')
        r.update(args=candidate,geometry=geometry,environment_clearance_certified=False)
        t=time.monotonic();r.update(executed=None,dispatch_monotonic=t)
        try:receipt=ctx.value(dispatch,observe_after=bool(inputs.get('observe_after')),**command)
        finally:r['motion_s']=time.monotonic()-t
        r['receipt']=receipt;r['executed']=receipt.get('executed')
        if receipt.get('executed') is not True or receipt.get('measured_arrival') is not True or receipt.get('measured_settled') is not True:
            raise ValueError('Unconfirmed measured arrival/settling; no replay')
        final=receipt['final14'];robot_after=ctx.value('piper_status')
        if (receipt['owner_generation']!=reference['owner_generation']
                or robot_after['bridge']['instance']!=reference['bridge_instance']
                or not robot_after.get('motion_ready')
                or robot_after['owner_status']['generation']!=reference['owner_generation']
                or (receipt.get('owner_request_id') is not None and
                    robot_after.get('last_motion',{}).get('owner_request_id')!=receipt['owner_request_id'])
                or not observation_pose_matches(final,robot_after['measured_state']['values'])):
            raise ValueError('Completion owner/state changed')
        if max(abs(a-b) for a,b in zip(q[7:13],final[7:13]))>.01 or abs(q[13]-final[13])>.001:
            raise ValueError('Right state changed')
        r.update(ok=True,motion_arrived=True,expected14=final,last_motion=robot_after.get('last_motion'),
            reason='Measured maneuver complete; no new object/scene fact asserted')
        obs=receipt.get('observation')
        if obs:r['after_frame']=obs['frame_id']
    except Exception as exc:
        r.update(reason=str(exc),next_action='inspect_actual_state_no_automatic_replay')
    finally:
        r['total_s']=time.monotonic()-start;p=OUT/(str(time.time_ns())+'-'+inputs['name']+'.json')
        r['report_path']=str(p);p.write_text(json.dumps(r,indent=2))
    return r
