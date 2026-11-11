"""Isolated v0.6 continuity experiment with two arm chunks. No saved-coordinate replay."""
import json,math,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scheduled import (_report,_add,subgoal_candidates,reference,MAX_POSE_GOAL_M)
from rehearsal import (preparation_goal,preparation_candidates,approach_goal,
    PREPARATION_MAX_STEPS,PREPARATION_ROTATION_ERROR_DEG,GOAL_POSITION_ERROR_M,compact)
from perception import localize_supported,observation_recent,visual,observation_pose_matches
from approach_target import create,localization,advance,save
from execution import state,move,check_workspace_goal
from checks import assess,context_for,modeled_obstacles


def solve_leg(ctx,q,candidates,ref):
    """Planning-only virtual FK. Never label predicted waypoints as measured."""
    fk=ctx.value('piper_fk',joints=q[:6],arm='left')
    obstacles=modeled_obstacles(ref['source_observation'])
    if not obstacles:raise ValueError('Measured table model required')
    errors=[]
    for pose in candidates:
        if math.dist(pose['position'],fk['position'])>.400000001:raise ValueError('Pose leg exceeds400mm')
        try:solved=ctx.value('piper_solve_ik',position=pose['position'],quaternion_xyzw=pose.get('quaternion_xyzw',fk['quaternion_xyzw']),seed=q[:6])
        except RuntimeError as exc:
            if 'MOVEIT_NO_IK_SOLUTION' not in str(exc):raise
            errors.append(str(exc));continue
        goal=list(q);goal[:6]=solved['q']
        if max(abs(a-b) for a,b in zip(goal[:6],q[:6]))>math.radians(60):continue
        check=ctx.value('piper_check_path',start14=q,target14=goal,obstacles=obstacles)
        if check.get('geometry_identity')!=ref['geometry_identity']:raise ValueError('Geometry changed during trajectory planning')
        if check.get('cad_clear') is True and check.get('whole_path_checked') is True and check.get('workspace_clear') is True:
            return goal
        errors.append(check.get('workspace_check',{}).get('reason','Path collision'))
    raise ValueError('No checked IK waypoint: '+'; '.join(errors)[-1500:])


def execute_chunk(ctx,ref,anchor,points,name,blend_indices=()):
    start=time.monotonic();r=dict(function=name,ok=False,executed=False,visual_calls=0,
        motion_s=0,planning_s=0,after_frame=None,planned_waypoints=[q[:6] for q in points],blend_indices=list(blend_indices))
    try:
        current=state(ctx,ref)
        if not observation_pose_matches(anchor,current['measured_state']['values']):raise ValueError('Trajectory planning anchor changed')
        r['executed']=None;t=time.monotonic()
        receipt=ctx.value('piper_move_waypoints',start14=anchor,waypoints=[q[:6] for q in points],obstacles=modeled_obstacles(ref['source_observation']),blend_indices=list(blend_indices))
        r.update(motion_s=time.monotonic()-t,receipt=receipt,executed=receipt.get('executed'))
        if not all(receipt.get(k) is True for k in ('executed','measured_arrival','measured_settled')):raise ValueError('Trajectory completion unconfirmed; no replay')
        after=ctx.value('piper_status')
        if (not after.get('motion_ready') or after['bridge']['instance']!=ref['bridge_instance']
                or after['owner_status']['generation']!=ref['owner_generation']
                or receipt['owner_generation']!=ref['owner_generation']
                or after.get('last_motion',{}).get('owner_request_id')!=receipt.get('owner_request_id')
                or not observation_pose_matches(receipt['final14'],after['measured_state']['values'])):
            raise ValueError('Trajectory completion state/owner mismatch')
        r.update(ok=True,expected14=receipt['final14'],last_motion=after['last_motion'],motion_arrived=True,
            reason='All samples acknowledged and measured terminal settling confirmed')
    except Exception as exc:r.update(reason=str(exc),recovery='Stop and inspect; explicit Prepare and new localization; no automatic replay')
    return _report(r,start)


def prepare_and_enter(ctx,inputs=None):
    start=time.monotonic();r=dict(function='prepare_and_enter',ok=False,executed=False,visual_calls=0,steps=[])
    try:
        loc=localize_supported(ctx,{})
        for k in ('visual_calls','perception_s'):r[k]=loc.get(k,0)
        if not loc.get('ok'):raise ValueError(loc.get('reason','Initial localization failed'))
        if not observation_recent(loc['observation'],require_depth=True):
            r['initial_boundary_refresh']=True
            loc=localize_supported(ctx,{'previous':loc})
            for k in ('visual_calls','perception_s'):r[k]+=loc.get(k,0)
            if not loc.get('ok'):raise ValueError(loc.get('reason','Initial refresh failed'))
        if not observation_recent(loc['observation'],require_depth=True):raise ValueError('Initial metric localization expired')
        target=create(loc,ctx.value('piper_status'),phase='preparation')
        r.update(target_id=target['id'],localization={k:v for k,v in localization(target,loc['observation']).items() if k!='observation'})
        initial=assess(ctx,loc['observation'],'Check the initially empty jaw and stable cup/table scene.',
            required_facts={'jaw_empty':'No held object between pads','cup_upright':'Cup upright on table'},
            context=context_for(ctx,loc['observation'],robot=ctx.value('piper_status'),action='prepare'),fallback=visual)
        for k in ('visual_calls','perception_s','wrist_depth_calls'):r[k]=r.get(k,0)+initial.get(k,0)
        if not initial.get('ok'):raise ValueError('Initial empty scene unresolved')
        robot=state(ctx,target);q=list(robot['measured_state']['values'])
        first_fk=ctx.value('piper_fk',joints=q[:6],arm='left');goal=preparation_goal(loc,first_fk)
        check_workspace_goal(goal['position'],robot)
        # Jaw boundaries are real actions; arm planning begins after their receipts.
        for _ in range(2):
            if .012<=q[6]<=.020:break
            if q[6]>.020:raise ValueError('Unexpected initial jaw opening')
            step=move(ctx,dict(name='preparation_open',tool='piper_gripper',args={'width_m':min(.016,q[6]+.0095)}),target)
            _add(r,step);advance(ctx,target,step);q=list(state(ctx,target)['measured_state']['values'])
        if not .012<=q[6]<=.020:raise ValueError('Open jaw not confirmed')
        anchor=q.copy();points=[];labels=[];planning=time.monotonic()
        for _ in range(PREPARATION_MAX_STEPS):
            fk=ctx.value('piper_fk',joints=q[:6],arm='left')
            distance=math.dist(goal['position'],fk['position'])
            angle=math.degrees((Rotation.from_quat(goal['quaternion_xyzw'])*Rotation.from_quat(fk['quaternion_xyzw']).inv()).magnitude())
            if distance<=GOAL_POSITION_ERROR_M and angle<=PREPARATION_ROTATION_ERROR_DEG:break
            candidates=preparation_candidates(fk,goal)
            if fk['position'][2]<goal['position'][2]-GOAL_POSITION_ERROR_M:candidates=subgoal_candidates(fk,[*fk['position'][:2],goal['position'][2]])
            elif angle<=PREPARATION_ROTATION_ERROR_DEG:candidates=subgoal_candidates(fk,goal['position'],goal['quaternion_xyzw'])
            q=solve_leg(ctx,q,candidates,target);points.append(q);labels.append('preparation')
        else:raise ValueError('Preparation planning bound exhausted; no arm motion sent')
        for phase in ('hover','entry'):
            for _ in range(8):
                fk=ctx.value('piper_fk',joints=q[:6],arm='left');g=approach_goal(loc,fk,phase,measured14=q)
                if g['goal_distance_m']<=GOAL_POSITION_ERROR_M:break
                q=solve_leg(ctx,q,subgoal_candidates(fk,g['goal_flange']),target);points.append(q);labels.append(phase)
            else:raise ValueError('Approach planning bound exhausted; no arm motion sent')
        # Plan the held transfer before approaching, using this run's geometry.
        # It is only a candidate; live pickup feedback must bind it before use.
        target['transfer_preplan']=plan_transfer_from_entry(ctx,target,loc,q)
        r['transfer_preplanning_s']=target['transfer_preplan']['planning_s']
        r['transfer_preplan_ok']=target['transfer_preplan']['ok']
        save(target)
        r['planning_s']=time.monotonic()-planning;r['waypoint_labels']=labels
        blend_indices=[i for i in range(len(labels)-1) if labels[i]==labels[i+1]=='preparation']
        r['blend_indices']=blend_indices
        if not points:raise ValueError('No approach trajectory; refuse replay')
        step=execute_chunk(ctx,target,anchor,points,'empty_approach_trajectory',blend_indices=blend_indices);_add(r,step);advance(ctx,target,step)
        actual=ctx.value('piper_fk',joints=step['expected14'][:6],arm='left')
        final_goal=approach_goal(loc,actual,'entry',measured14=step['expected14'])
        if final_goal['goal_distance_m']>GOAL_POSITION_ERROR_M:raise ValueError('Actual entry pose not within existing subgoal tolerance')
        target['phase']='entry_verified';save(target)
        r.update(ok=True,task_completed=True,goal_error_m=final_goal['goal_distance_m'],scene_verified=False)
    except Exception as exc:r['reason']=str(exc)
    return _report(r,start)


def transfer_and_lower(ctx,inputs):
    start=time.monotonic();r=dict(function='lower_to_height',ok=False,executed=False,steps=[],visual_calls=0)
    try:
        ref=reference(inputs);robot=state(ctx,ref);anchor=list(robot['measured_state']['values']);q=anchor.copy()
        fk=ctx.value('piper_fk',joints=q[:6],arm='left');support=ref['predicted_support_flange_z']
        if not 0<fk['position'][2]-support<.065:raise ValueError('Invalid same-run lift/support height')
        planning=time.monotonic();points=[]
        from approach_target import PATH
        approach=json.loads(PATH.read_text());candidate=approach.get('transfer_preplan',{})
        use=False
        if (candidate.get('target_id')==ref.get('target_id')
                and approach.get('bridge_instance')==ref['bridge_instance']
                and approach.get('geometry_identity')==ref['geometry_identity']
                and candidate.get('ok') is True
                and max(abs(a-b) for a,b in zip(anchor[:6],candidate['lift_joints']))<=.01):
            expected=[[*ref['transfer_goal'][:2],fk['position'][2]],[*ref['transfer_goal'][:2],support]]
            checks=[ctx.value('piper_fk',joints=q,arm='left') for q in candidate['waypoints']]
            use=all(math.dist(row['position'],pos)<=.001 for row,pos in zip(checks,expected))
            if use:
                points=[[*left,*anchor[6:]] for left in candidate['waypoints']]
                r['transfer_plan_reused']=True
        if not use:
            r['transfer_plan_reused']=False
            r['replan_reason']='Predicted plan unavailable or actual pickup/placement anchor differs'
            # Full legs only: partial transfer fallback must never proceed to descent.
            transfer=[*ref['transfer_goal'][:2],fk['position'][2]]
            if math.dist(transfer,fk['position'])>=.002:
                q=solve_leg(ctx,q,[dict(position=transfer,quaternion_xyzw=fk['quaternion_xyzw'])],ref);points.append(q)
            fk=ctx.value('piper_fk',joints=q[:6],arm='left')
            if math.dist(fk['position'][:2],ref['transfer_goal'][:2])>.002:raise ValueError('Planned transfer not aligned')
            q=solve_leg(ctx,q,[dict(position=[*ref['transfer_goal'][:2],support],quaternion_xyzw=fk['quaternion_xyzw'])],ref);points.append(q)
        r['planning_s']=time.monotonic()-planning
        # The horizontal endpoint is above support, not a contact endpoint.
        # Keep final placement stopped; the changed full curve is rechecked by the bridge.
        blends=[0] if len(points)==2 else []
        step=execute_chunk(ctx,ref,anchor,points,'held_transfer_lower_trajectory',blend_indices=blends);_add(r,step)
        actual=ctx.value('piper_fk',joints=step['expected14'][:6],arm='left')
        if abs(actual['position'][2]-support)>.002 or math.dist(actual['position'][:2],ref['transfer_goal'][:2])>.002:
            raise ValueError('Measured placement pose outside existing tolerance; keep jaw closed')
        r.update(ok=True,expected14=step['expected14'],last_motion=step['last_motion'],
            support_state='predicted_height_reached',support_visually_verified=False,predicted_support_flange_z=support)
    except Exception as exc:r['reason']=str(exc)
    _report(r,start);r['previous_report']=r['report_path'];Path(r['report_path']).write_text(json.dumps(r,indent=2));return r


def plan_transfer_from_entry(ctx,ref,loc,entry):
    """Pure preplanning; no motion and no claim of observed grasp/support."""
    start=time.monotonic();r=dict(ok=False,target_id=ref['id'],measured=False)
    try:
        q=list(entry);q[6]=.003
        efk=ctx.value('piper_fk',joints=q[:6],arm='left')
        lift=solve_leg(ctx,q,[dict(position=[*efk['position'][:2],efk['position'][2]+.040],quaternion_xyzw=efk['quaternion_xyzw'])],ref)
        fk=ctx.value('piper_fk',joints=lift[:6],arm='left')
        cup=np.asarray(loc['cup_center_estimate'])+np.asarray(fk['position'])-efk['position']
        xy=np.asarray(fk['position'])[:2]+np.asarray(loc['coaster_center'])[:2]-cup[:2]
        z=efk['position'][2]+loc['coaster_center'][2]-loc['table'][2]
        if np.linalg.norm(xy-np.asarray(fk['position'])[:2])>.22 or not 0<fk['position'][2]-z<.065:raise ValueError('Transfer outside characterized task bounds')
        a=solve_leg(ctx,lift,[dict(position=[*xy.tolist(),float(fk['position'][2])],quaternion_xyzw=fk['quaternion_xyzw'])],ref)
        b=solve_leg(ctx,a,[dict(position=[*xy.tolist(),float(z)],quaternion_xyzw=fk['quaternion_xyzw'])],ref)
        r.update(ok=True,lift_joints=lift[:6],waypoints=[a[:6],b[:6]],planned_support_z=z)
    except Exception as exc:r['reason']=str(exc)
    r['planning_s']=time.monotonic()-start;return r
