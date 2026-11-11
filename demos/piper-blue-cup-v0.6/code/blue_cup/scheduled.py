"""Franka-style ordinary task functions: measure once, observe at task boundaries.

The measured cup/coaster stay in base coordinates through this one run. No saved
pixel is reinterpreted as a new measurement. Normal numeric segments do not take
images. Unexpected state stops; object outcomes still require task observations.
The original rehearsal functions remain available in rehearsal/placement/transport.
"""
import json,math,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from perception import OUT,localize_supported,observation_recent,observation_pose_matches,visual
from approach_target import PATH,create,localization,save,advance
from execution import state,move,check_workspace_goal
from checks import assess,context_for,placement_execution_evidence
from rehearsal import (preparation_goal,preparation_candidates,approach_goal,compact,
    PREPARATION_MAX_STEPS,PREPARATION_ROTATION_ERROR_DEG,GOAL_POSITION_ERROR_M,
    APPROACH_MAX_STEPS,APPROACH_MIN_PROGRESS_M)


def _report(r,start):
    r['total_s']=time.monotonic()-start;p=OUT/(str(time.time_ns())+'-'+r['function']+'.json')
    r['report_path']=str(p);p.write_text(json.dumps(r,indent=2));return r


def _add(r,step):
    r.setdefault('steps',[]).append(compact(step))
    for k in ('visual_calls','wrist_depth_calls','perception_s','planning_s','motion_s','geometry_s','ik_calls'):
        r[k]=r.get(k,0)+step.get(k,0)
    if step.get('executed') is None:r['executed']=None
    elif step.get('executed'):r['executed']=True
    if not step.get('ok'):raise ValueError(step.get('reason','Maneuver failed'))


# Full subgoal first. Smaller alternatives are planning-only fallbacks; there is
# one selected motion per call, never a blended/multi-waypoint trajectory.
MAX_POSE_GOAL_M=.400

def subgoal_candidates(fk,position,quaternion=None):
    start=np.asarray(fk['position'],float);delta=np.asarray(position,float)-start
    if np.linalg.norm(delta)>MAX_POSE_GOAL_M+1e-9:raise ValueError('Subgoal exceeds400mm')
    return [dict(position=(start+fraction*delta).tolist(),
                 quaternion_xyzw=quaternion or fk['quaternion_xyzw'])
            for fraction in (1.,.5,.25)]


def _load(path):return json.loads(Path(path).read_text())


def prepare_for_approach(ctx,inputs=None):
    start=time.monotonic();r=dict(function='prepare_for_approach',ok=False,executed=False,visual_calls=0,steps=[])
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
        goal=None
        for _ in range(PREPARATION_MAX_STEPS+1):
            robot=state(ctx,target);q=robot['measured_state']['values'];fk=ctx.value('piper_fk',joints=q[:6],arm='left')
            if goal is None:
                goal=preparation_goal(loc,fk);r['goal']=goal
                check_workspace_goal(goal['position'],robot)
            distance=math.dist(goal['position'],fk['position'])
            angle=math.degrees((Rotation.from_quat(goal['quaternion_xyzw'])*Rotation.from_quat(fk['quaternion_xyzw']).inv()).magnitude())
            if .012<=q[6]<=.020 and distance<=GOAL_POSITION_ERROR_M and angle<=PREPARATION_ROTATION_ERROR_DEG:
                target['phase']='preparation_verified';save(target)
                r.update(ok=True,task_completed=True,goal_error_m=distance,orientation_error_deg=angle);break
            if len(r['steps'])>=PREPARATION_MAX_STEPS:raise ValueError('Preparation segment budget exhausted')
            if q[6]<.012:e=dict(name='preparation_open',tool='piper_gripper',args={'width_m':min(.016,q[6]+.0095)})
            elif q[6]>.020:raise ValueError('Unexpected initial jaw opening')
            else:
                candidates=preparation_candidates(fk,goal)
                # Keep the proven orientation staging; remove only artificial
                # translation slices when no rotation is being changed.
                if fk['position'][2]<goal['position'][2]-GOAL_POSITION_ERROR_M:
                    candidates=subgoal_candidates(fk,[*fk['position'][:2],goal['position'][2]])
                elif angle<=PREPARATION_ROTATION_ERROR_DEG:
                    candidates=subgoal_candidates(fk,goal['position'],goal['quaternion_xyzw'])
                if not candidates:raise ValueError('No bounded preparation candidate')
                e=dict(name='preparation_segment',tool='piper_move_pose',args=candidates[0],pose_candidates=candidates)
            step=move(ctx,e,target);_add(r,step);advance(ctx,target,step)
    except Exception as exc:r['reason']=str(exc)
    return _report(r,start)


def approach(ctx,inputs):
    start=time.monotonic();r=dict(function='approach',ok=False,executed=False,steps=[],visual_calls=0)
    try:
        target=_load(PATH);phase=inputs['phase']
        if phase not in ('hover','entry') or target['phase']!=('preparation_verified' if phase=='hover' else 'hover_verified'):
            raise ValueError('Approach phase cannot be replayed')
        target['phase']=phase;save(target);loc=localization(target,target['source_observation'])
        r.update(target_id=target['id'],localization={k:v for k,v in loc.items() if k!='observation'})
        previous_distance=None
        for _ in range(APPROACH_MAX_STEPS+1):
            robot=state(ctx,target);q=robot['measured_state']['values'];fk=ctx.value('piper_fk',joints=q[:6],arm='left')
            goal=approach_goal(loc,fk,phase,measured14=q);distance=goal['goal_distance_m'];r['goal']=goal
            if distance<=GOAL_POSITION_ERROR_M:
                target['phase']=phase+'_verified';save(target)
                r.update(ok=True,task_completed=True,goal_error_m=distance,scene_verified=False);break
            if previous_distance is not None and previous_distance-distance<APPROACH_MIN_PROGRESS_M:raise ValueError('No measured approach progress')
            if len(r['steps'])>=APPROACH_MAX_STEPS:raise ValueError('Approach segment budget exhausted')
            candidates=subgoal_candidates(fk,goal['goal_flange'])
            step=move(ctx,dict(name='approach_segment',tool='piper_move_pose',
                args=candidates[0],pose_candidates=candidates),target)
            _add(r,step);advance(ctx,target,step);previous_distance=distance
    except Exception as exc:r['reason']=str(exc)
    return _report(r,start)


def grasp(ctx,inputs=None):
    from placement import GRASP_WIDTH_M
    start=time.monotonic();r=dict(function='grasp',ok=False,executed=False,steps=[],visual_calls=0)
    try:
        target=_load(PATH)
        if target['phase']!='entry_verified':raise ValueError('Verified entry required')
        q=state(ctx,target)['measured_state']['values'];width=q[6]
        if not GRASP_WIDTH_M<width<=.020:raise ValueError('Unexpected pre-closure jaw state; no replay')
        step=move(ctx,dict(name='grasp_full_closure',tool='piper_close_gripper',args={'width_m':GRASP_WIDTH_M}),target)
        _add(r,step);advance(ctx,target,step)
        target['phase']='grasp_closed';target['contact_started']=True;save(target)
        r.update(ok=True,closure_target_m=GRASP_WIDTH_M,retention_verified=False)
    except Exception as exc:r['reason']=str(exc)
    return _report(r,start)


def localize_coaster(ctx,inputs):
    """Build transfer goal from INITIAL metric coaster, without another image/query."""
    start=time.monotonic();r=dict(function='localize_coaster',ok=False,executed=False,visual_calls=0,metric_localization_count=0)
    try:
        prep,entry,lift=[_load(inputs[k]) for k in ('preparation_report','entry_report','lift_report')]
        original=_load(PATH);robot=ctx.value('piper_status');last=lift['receipt']
        if (not all(x.get('ok') for x in (prep,entry,lift)) or lift['function']!='trial_lift'
                or prep['target_id']!=entry['target_id'] or prep['target_id']!=original['id']
                or robot['bridge']['instance']!=original['bridge_instance']
                or robot['owner_status']['generation']!=original['owner_generation']
                or not robot.get('motion_ready') or last['owner_generation']!=original['owner_generation']
                or (last.get('owner_request_id') is not None and
                    robot.get('last_motion',{}).get('owner_request_id')!=last['owner_request_id'])
                or not observation_pose_matches(last['final14'],robot['measured_state']['values'])):
            raise ValueError('Confirmed same-run pickup/owner required')
        source=prep['localization'];coaster=np.asarray(source['coaster_center'])
        er=_load(entry['steps'][-1]['report_path'])['receipt'] if entry['steps'] else {'final14':original['expected14']}
        efk=ctx.value('piper_fk',joints=er['final14'][:6],arm='left')
        fk=ctx.value('piper_fk',joints=last['final14'][:6],arm='left')
        cup=np.asarray(source['cup_center_estimate'])+np.asarray(fk['position'])-efk['position']
        goal=np.asarray(fk['position']);goal[:2]+=coaster[:2]-cup[:2]
        support_z=efk['position'][2]+coaster[2]-source['table'][2]
        if np.linalg.norm(goal[:2]-np.asarray(fk['position'])[:2])>.22 or not 0<fk['position'][2]-support_z<.065:
            raise ValueError('Transfer/support target outside measured task bounds')
        r.update(ok=True,localization_source='initial_same_run_metric_coaster',
            coaster_center=coaster.tolist(),coaster_pixel=source['contours']['coaster'][:2],
            cup_center_estimate=cup.tolist(),transfer_goal=goal.tolist(),predicted_support_flange_z=support_z,
            observation=original['source_observation'],source_observation=original['source_observation'],
            source_frame=original['source_frame'],measurement_frame=original['source_frame'],
            owner_generation=original['owner_generation'],bridge_instance=original['bridge_instance'],
            geometry_identity=original['geometry_identity'],expected14=last['final14'],
            last_motion=robot.get('last_motion'),target_id=original['id'],source_reports=inputs)
    except Exception as exc:r['reason']=str(exc)
    return _report(r,start)


def reference(inputs):
    target=_load(inputs['target_report']);previous=_load(inputs.get('previous_report',inputs['target_report']))
    if not target.get('ok') or not previous.get('ok'):raise ValueError('Successful task/previous result required')
    ref=dict(target);ref['expected14']=previous.get('expected14') or previous['receipt']['final14']
    if 'last_motion' not in previous:raise ValueError('Previous result lacks action binding')
    ref['last_motion']=previous['last_motion']
    return ref


def transfer_step(ctx,inputs):
    start=time.monotonic();r=dict(function='transfer_step',ok=False,executed=False,visual_calls=0)
    try:
        ref=reference(inputs);robot=state(ctx,ref);fk=ctx.value('piper_fk',joints=robot['measured_state']['values'][:6],arm='left')
        delta=np.asarray(ref['transfer_goal'])-fk['position'];delta[2]=0;distance=np.linalg.norm(delta)
        if distance<.002:raise ValueError('Already at transfer goal; no replay')
        candidates=subgoal_candidates(fk,np.asarray(fk['position'])+delta)
        r=move(ctx,dict(name='transfer_step',tool='piper_move_pose',args=candidates[0],pose_candidates=candidates),ref)
        if r['ok']:
            arrived=ctx.value('piper_fk',joints=r['expected14'][:6],arm='left')
            r['remaining_xy_m']=math.dist(arrived['position'][:2],ref['transfer_goal'][:2])
            r['transport_complete']=r['remaining_xy_m']<.002
    except Exception as exc:r['reason']=str(exc)
    return _report(r,start)


def lower_to_height(ctx,inputs):
    """Return the actual lift, accounting for INITIAL coaster/table height delta.

    No observation or support claim at this boundary; do not descend below the
    calculated height to force an uncertain visual result. Final image judges outcome.
    """
    start=time.monotonic();r=dict(function='lower_to_height',ok=False,executed=False,steps=[],visual_calls=0)
    try:
        ref=reference(inputs)
        for _ in range(9):
            robot=state(ctx,ref);q=robot['measured_state']['values'];fk=ctx.value('piper_fk',joints=q[:6],arm='left')
            remaining=fk['position'][2]-ref['predicted_support_flange_z']
            if math.dist(fk['position'][:2],ref['transfer_goal'][:2])>.002:raise ValueError('Cup not at transfer XY goal')
            if not math.isfinite(remaining) or remaining<-.002:raise ValueError('Below planned support height; stop')
            if remaining<=.002:
                r.update(ok=True,expected14=q,last_motion=robot.get('last_motion'),
                    support_state='predicted_height_reached',support_visually_verified=False,
                    predicted_support_flange_z=ref['predicted_support_flange_z']);break
            if len(r['steps'])>=8:raise ValueError('Descent segment budget exhausted')
            candidates=subgoal_candidates(fk,[*fk['position'][:2],ref['predicted_support_flange_z']])
            step=move(ctx,dict(name='lower_height_segment',tool='piper_move_pose',args=candidates[0],pose_candidates=candidates),ref)
            _add(r,step);ref.update(expected14=step['expected14'],last_motion=step['last_motion'])
    except Exception as exc:r['reason']=str(exc)
    _report(r,start);r['previous_report']=r['report_path']
    Path(r['report_path']).write_text(json.dumps(r,indent=2));return r


def release(ctx,inputs):
    start=time.monotonic();r=dict(function='release',ok=False,executed=False,visual_calls=0)
    try:
        prior=_load(inputs['previous_report']);ref=reference(inputs)
        if prior.get('function')!='lower_to_height' or prior.get('support_state')!='predicted_height_reached':
            raise ValueError('Completed planned descent required')
        robot=state(ctx,ref);fk=ctx.value('piper_fk',joints=robot['measured_state']['values'][:6],arm='left')
        if abs(fk['position'][2]-ref['predicted_support_flange_z'])>.002 or math.dist(fk['position'][:2],ref['transfer_goal'][:2])>.002:
            raise ValueError('Release pose changed from planned placement')
        r=move(ctx,dict(name='release',tool='piper_gripper',args={'width_m':inputs['width_m']}),ref)
        r.update(support_visually_verified=False,placement_verified=False)
    except Exception as exc:r.update(ok=False,reason=str(exc))
    return _report(r,start)


def retreat(ctx,inputs):
    start=time.monotonic();r=dict(function='retreat',ok=False,executed=False,visual_calls=0)
    try:
        ref=reference(inputs);prior=_load(inputs['previous_report'])
        if prior['function'] not in ('release','retreat'):raise ValueError('Completed jaw release required before retreat')
        released=prior if prior['function']=='release' else _load(prior['release_report'])
        d=inputs['delta']
        if len(d)!=3 or d[:2]!=[0,0] or not 0<d[2]<=MAX_POSE_GOAL_M:raise ValueError('Expected bounded upward retreat')
        r=move(ctx,dict(name='retreat',tool='piper_move_pose',args={'delta':d},observe_after=inputs.get('observe_result',False)),ref)
        r['release_report']=released['report_path']
        if r['ok'] and inputs.get('observe_result'):
            obs=r['receipt'].get('observation')
            if obs is None:obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=True)
            condition=context_for(ctx,obs,robot=ctx.value('piper_status'),action='retreat',inputs=inputs)
            condition.update(measured_arrival=r['receipt'].get('measured_arrival') is True,
                placement_executed=placement_execution_evidence(released,r,ref))
            result=assess(ctx,obs,'Check overall final cup placement: upright on intended coaster and gripper withdrawn. No exact centering requirement.',
                required_facts={'placement_stable':'Cup remains upright on coaster after retreat'},
                context=condition,fallback=visual)
            r.update(ok=result.get('ok') is True,post=result,observation=obs,visual_calls=result.get('visual_calls',0),
                perception_s=result.get('perception_s',0),wrist_depth_calls=result.get('wrist_depth_calls',0))
            r['reason']=('Final qualitative placement confirmed' if r['ok'] else
                         result.get('reason','Final placement not confirmed'))
    except Exception as exc:r.update(ok=False,reason=str(exc))
    return _report(r,start)
