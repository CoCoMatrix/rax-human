"""Small single-action functions. No controller, reset, or historical pose replay."""
import json,math,time
from pathlib import Path
from scipy.spatial.transform import Rotation,Slerp
import numpy as np
from perception import visual,OUT,localize_supported,observation_recent,BOUNDARY_MAX_AGE_S,observation_pose_matches
from checks import assess, context_for, current_depth, modeled_obstacles

def _help_message(action, facts, frame_id):
    return ('Without moving the robot or objects, confirm ONLY the following unresolved facts for this one action. '
            'If there is contact, an obstruction, or you cannot tell, say so explicitly. This is not approval for other actions. '
            +json.dumps({'action':action,'facts':facts,'before_frame':frame_id},sort_keys=True))


def _same_pose(a,b,*,joint_tolerance=.01):
    return observation_pose_matches(a,b,joint_tolerance=joint_tolerance)


def _onsite(ctx,path,action,required,obs,robot):
    """Read the original host answer, not an agent-written approval boolean."""
    path=Path(path)
    if path.with_suffix('.used.json').exists():raise ValueError('Onsite fact already consumed by a dispatch; observe only, never replay')
    record=json.loads(path.read_text());facts=record['facts'];before=record['before']
    if (record['action']!=action or not isinstance(facts,dict) or not facts
            or any(required.get(k)!=v for k,v in facts.items())
            or not _same_pose(before.get('measured14'),obs['measured14'])):
        raise ValueError('Onsite fact scope/action/pose changed; no motion')
    answer=ctx.value('request_operator',request_id=record['reply']['request_id'],timeout_s=1)
    binding=answer.get('context') or {}
    if (answer.get('status')!='answered' or answer.get('done') is not True
            or answer.get('question')!=_help_message(action,facts,before['frame_id'])
            or binding.get('bridge_instance')!=robot['bridge']['instance']
            or binding.get('owner_generation')!=robot['owner_status']['generation']
            or not _same_pose(binding.get('measured14'),obs['measured14'])
            or not isinstance(answer.get('reply'),str) or not answer['reply'].strip()):
        raise ValueError('Original operator answer is pending/negative or no longer bound to this action, owner and pose')
    return record,answer,{'facts':facts,'reply':answer['reply'],'request_id':answer['request_id']}


def move_once(ctx, explicit_inputs):
    """One checked action; ordinary conditions first, auxiliary vision on insufficiency."""
    e=explicit_inputs; start=time.monotonic(); r={'ok':False,'function':e['name'],'executed':False,'motion_arrived':False,'task_completed':False,'operator_assistance':None,'perception_s':0,'visual_calls':0,'planning_s':0,'motion_s':0}
    try:
        # Explicit empty dict means no scene fact is claimed at this boundary.
        # Missing/None must not accidentally disable a task's checks.
        for key in ('pre_facts','post_facts'):
            if not isinstance(e.get(key),dict):raise ValueError(key+' must be an explicit dict; {} means robot-only checks')
            if any(not isinstance(x,str) or not x.strip() for pair in e[key].items() for x in pair):
                raise ValueError(key+' requires nonempty fact names and descriptions')
            question=e.get(key.removesuffix('_facts'))
            if e[key] and (not isinstance(question,str) or not question.strip()):
                raise ValueError(key+' requires an explicit perception question before dispatch')
        if e.get('operator_file') and not e['pre_facts']:
            raise ValueError('An operator reply requires explicit precondition facts')
        status=ctx.value('status'); robot=status['robot_status']
        need_depth=bool(e['pre_facts'] and not e.get('operator_file'))
        obs=e.get('observation')
        if obs is not None:
            # Only a still-bound short boundary can refresh RGB. Long pauses,
            # invalid clocks, changed pose/owner require full re-localization.
            if (not observation_recent(obs,BOUNDARY_MAX_AGE_S) or
                    not _same_pose(obs.get('measured14'),robot['measured_state']['values']) or
                    obs.get('state_binding',{}).get('owner_generation')!=robot['owner_status']['generation']):
                r['reason']='Shared observation expired or state changed; re-observe/re-localize';return r
            r['observation_reused']=True
            if not observation_recent(obs,1.0):
                fresh=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=need_depth)
                if (not observation_recent(fresh,1.0) or
                        not _same_pose(obs.get('measured14'),fresh.get('measured14')) or
                        fresh.get('state_binding',{}).get('owner_generation')!=robot['owner_status']['generation']):
                    r['reason']='Boundary refresh is stale or state/owner changed; re-localize';return r
                obs=fresh;r['observation_reused']=False;r['boundary_reacquired']=True
        else:
            obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=need_depth)
            r['observation_reused']=False
        r.update(before_frame=obs['frame_id'],wall_budget=obs['wall_budget'])
        if status.get('control_uncertain') or not robot.get('motion_ready',False):
            r['reason']='Unknown or non-idle control; status/observe only';return r
        if not obs.get('state_binding',{}).get('rest_consistent'):
            r['reason']='Camera/state bracket is not at consistent rest';return r
        if not robot.get('motion_enabled') or not robot.get('arming',{}).get('armed') or obs['wall_budget']['remaining_seconds']<=90:
            r['reason']='Motion unavailable or finish reserve';return r
        if e.get('scene_binding'):
            binding=e['scene_binding']
            if time.monotonic()-binding['started_monotonic']>20 or not _same_pose(binding['measured14'],obs['measured14']):
                r['reason']='Localization state changed or expired; re-localize without moving';return r
        if e['pre_facts'] and not e.get('operator_file'):
            t=time.monotonic();obs=current_depth(ctx,obs);r['perception_s']+=time.monotonic()-t
        r['before_frame']=obs['frame_id'];r['wall_budget']=obs['wall_budget']
        q=obs['measured14'];fk=ctx.value('piper_fk',joints=q[:6],arm='left');r['before_fk']=fk
        obstacles=e.get('obstacles',[])+modeled_obstacles(obs)
        args=dict(e['args']);tool=e['tool']
        if tool=='piper_move_pose':
            if 'delta' in args: args['position']=(np.array(fk['position'])+np.array(args.pop('delta'))).tolist()
            if math.dist(args['position'],fk['position'])>.071:raise ValueError('Local step exceeds71mm')
        elif tool=='piper_gripper':
            if abs(args['width_m']-q[6])>.010001:raise ValueError('Jaw step exceeds10mm')
        else: raise ValueError('Unsupported action')
        r['args']=args
        dispatch_tool=tool;r['ik_calls']=0;r['candidate_checks']=[];r['geometry_s']=0
        candidates=e.get('pose_candidates',[args]) if tool=='piper_move_pose' else [args]
        if not isinstance(candidates,list) or not 1<=len(candidates)<=3:raise ValueError('Expected1..3 bounded pose candidates')
        selected=False
        for candidate in candidates:
            args=dict(candidate);target=list(q);dispatch_args=dict(args)
            if tool=='piper_move_pose':
                if math.dist(args['position'],fk['position'])>.071:raise ValueError('Candidate exceeds71mm')
                t=time.monotonic();r['ik_calls']+=1
                try:
                    solved=ctx.value('piper_solve_ik',position=args['position'],
                        quaternion_xyzw=args.get('quaternion_xyzw',fk['quaternion_xyzw']),seed=q[:6])
                except Exception as exc:
                    # Only a read-only, explicit unreachable candidate allows the
                    # next DIFFERENT target. Never retry an uncertain/dispatched action.
                    try:error=json.loads(str(exc).split('MCP tool failed: ',1)[1])['error']
                    except (ValueError,KeyError,IndexError,TypeError):raise exc
                    if not (error.get('no_action_executed') is True and error.get('stage')=='validation'
                            and 'MOVEIT_NO_IK_SOLUTION' in error.get('reason','')):raise
                    r['candidate_checks'].append({'args':args,'reachable':False,'error':error})
                    if len(candidates)==1:raise
                    continue
                finally:r['planning_s']+=time.monotonic()-t
                target[:6]=solved['q'];dispatch_tool='piper_move_joints'
                dispatch_args={'joints':solved['q'],**({'duration_s':args['duration_s']} if 'duration_s' in args else {})}
            else:target[6]=args['width_m']
            t=time.monotonic()
            geometry=ctx.value('piper_check_path',start14=q,target14=target,obstacles=obstacles)
            r['geometry_s']+=time.monotonic()-t
            if geometry.get('geometry_identity')!=obs.get('finger_geometry',{}).get('geometry_identity'):
                r['reason']='Observation and collision geometry identities differ';return r
            clear=geometry.get('cad_clear') and geometry.get('whole_path_checked')
            r['candidate_checks'].append({'args':args,'reachable':True,'modeled_path_clear':bool(clear)})
            if not clear:continue
            r['geometry']=geometry;r['args']=args;selected=True;break
        if not selected:r['reason']='No reachable candidate with a clear installed-geometry path';return r
        # Empty environment is explicitly NOT a table/cup/line clearance proof.
        r['environment_clearance_certified']=False
        action={'name':e['name'],'tool':tool,'args':args,'geometry_identity':geometry['geometry_identity'],
                'obstacles':obstacles}
        onsite=None;assistance=None
        if e.get('operator_file'):
            assistance,answer,onsite=_onsite(ctx,e['operator_file'],action,e.get('pre_facts') or {},obs,robot)
            r['operator_assistance']={k:answer[k] for k in ('request_id','reply','waited_s','answered_at')}
            r['operator_assistance']['facts']=onsite['facts']
        if e['pre_facts']:
            question=e['pre']+' Proposed command '+json.dumps(args)+'. Current flange '+json.dumps(fk['position'])+'. Assess the listed visible facts; report any visible obstruction. Do not certify numerical clearance or the entire swept volume.'
            t=time.monotonic()
            try:
                rules=context_for(ctx,obs,robot=robot,action=e['name'],fk=fk,target14=target,inputs=e,geometry=geometry)
                pre=assess(ctx,obs,question,prior=assistance['before'] if assistance else e.get('scene_reference'),cameras=tuple(e.get('cameras',['head','left'])),required_facts=e['pre_facts'],onsite=onsite,context=rules,fallback=visual)
            finally:r['perception_s']+=time.monotonic()-t
            r['visual_calls']+=pre.get('visual_calls',0)
            r['wrist_depth_calls']=r.get('wrist_depth_calls',0)+pre.get('wrist_depth_calls',0)
            rebound=pre.pop('_checked_observation',None)
            if rebound is not None:
                obs=rebound;q=obs['measured14'];r['before_frame']=obs['frame_id']
                fk=ctx.value('piper_fk',joints=q[:6],arm='left');r['before_fk']=fk
                checked=ctx.value('piper_check_path',start14=q,target14=target,obstacles=obstacles)
                if not checked.get('cad_clear') or not checked.get('whole_path_checked') or checked.get('geometry_identity')!=geometry['geometry_identity']:
                    r['reason']='Refreshed wrist-depth start path is not clear';return r
                r['wrist_depth_boundary_rechecked']=True
            r.setdefault('condition_checks',[]).append({'boundary':'pre','model_calls':pre.get('visual_calls',0),'fact_sources':pre.get('fact_sources',{}),'fallback_reason':pre.get('fallback_reason')})
            r['pre']=pre
            if pre.get('ok') is not True:
                r['reason']='Precondition failed'
                # A concrete visible hazard/contradiction cannot be overridden by a yes/no request.
                if (not onsite and pre.get('unresolved_facts') and not pre.get('contradicted_facts')
                        and not pre.get('invalid_evidence') and not pre.get('visible_hazards')):
                    facts={k:e['pre_facts'][k] for k in pre['unresolved_facts']}
                    path=OUT/(str(time.time_ns())+'-onsite.json')
                    path.write_text(json.dumps({'before':obs,'action':action,'facts':facts},indent=2))
                    r['operator_file']=str(path)
                    r['operator_request']={'kind':'help','message':_help_message(action,facts,obs['frame_id']),'timeout_s':300}
                    r['next_action']='Request these facts once with the native request_operator tool; save its reply in operator_file and rejoin that request_id on timeout.'
                return r
        else:
            r['pre']={'ok':True,'visual_used':False,'basis':'geometry_and_control_state',
                'fact_sources':{},'reason':'Checked robot state and modeled path only; no scene/contact fact asserted'}
        # A slow successful semantic query may outlive its source frame. Acquire
        # a real new frame; keep semantic provenance separate and compare every
        # supplied current view. A changed scene or any intervening action stops.
        if e.get('observation') is not None and not observation_recent(obs,BOUNDARY_MAX_AGE_S):
            if not e['pre_facts'] or onsite:
                r['reason']='Action boundary expired during checks; re-observe/re-localize';return r
            from perception import refresh_visual_boundary
            obs,rebound=refresh_visual_boundary(ctx,obs,robot,tuple(e.get('cameras',['head','left'])))
            q=obs['measured14'];r['visual_boundary_refresh']=rebound
            refreshed=ctx.value('piper_check_path',start14=q,target14=target,obstacles=obstacles)
            if (refreshed.get('geometry_identity')!=geometry['geometry_identity']
                    or not refreshed.get('cad_clear') or not refreshed.get('whole_path_checked')):
                r['reason']='Refreshed current-state path is not clear';return r
            fk=ctx.value('piper_fk',joints=q[:6],arm='left');r['before_fk']=fk
        # Refresh before dispatch; reject measured-state/owner drift, even without vision.
        current=ctx.value('piper_status')
        if (not current.get('motion_ready') or current['bridge']['instance']!=robot['bridge']['instance']
                or current['owner_status']['generation']!=robot['owner_status']['generation']
                or not _same_pose(q,current['measured_state']['values'])):
            r['reason']='State/owner changed during perception';return r
        if e.get('observation') is not None and not observation_recent(obs,BOUNDARY_MAX_AGE_S):
            r['reason']='Action boundary expired during checks; re-observe/re-localize';return r
        if current['wall_budget']['remaining_seconds']<=60:r['reason']='Finish reserve';return r
        if onsite:
            # Consume BEFORE dispatch: even an uncertain RPC outcome must not reuse this fact.
            with Path(e['operator_file']).with_suffix('.used.json').open('x') as used:
                json.dump({'request_id':onsite['request_id'],'action':action,'before_frame':obs['frame_id']},used)
        if e['name'] not in ('approach_segment','preparation_segment','preparation_open'):
            from approach_target import invalidate
            invalidate('Non-approach action: '+e['name'])
        t=time.monotonic();r.update(dispatch_monotonic=t,executed=None)
        receipt=ctx.value(dispatch_tool,observe_after=True,**dispatch_args);r['motion_s']=time.monotonic()-t;r['receipt']=receipt;r['executed']=receipt.get('executed')
        r['arrival_monotonic']=time.monotonic()
        # A numerical failure cannot be repaired by a visual approval.
        r['next_action']='observe_only'
        if receipt.get('executed') is not True or receipt.get('measured_arrival') is not True:
            r['reason']='Unconfirmed motion; STOP';return r
        final=np.asarray(receipt.get('final14'),dtype=float)
        if final.shape!=(14,) or not np.isfinite(final).all():
            r['reason']='Invalid final measured state; STOP';return r
        if max(abs(a-b) for a,b in zip(q[7:13],final[7:13]))>.01 or abs(q[13]-final[13])>.003:r['reason']='Right state changed; STOP';return r
        r['motion_arrived']=True
        completed_robot=ctx.value('piper_status')
        if (not completed_robot.get('motion_ready') or
                completed_robot['bridge']['instance']!=robot['bridge']['instance'] or
                completed_robot['owner_status']['generation']!=robot['owner_status']['generation'] or
                (receipt.get('owner_request_id') is not None and
                 completed_robot.get('last_motion',{}).get('owner_request_id')!=receipt['owner_request_id']) or
                not _same_pose(final.tolist(),completed_robot['measured_state']['values'])):
            r['reason']='Completion state/owner changed';return r
        r['expected14']=final.tolist();r['last_motion']=completed_robot.get('last_motion')
        r['feedback_checks']={'ok':True,'basis':'control_feedback',
            'verified':['measured_arrival','right_state_preserved'],
            'reason':'Backend measured arrival and unchanged right state; no retention/support/scene assertion'}
        after=receipt.get('observation') or ctx.value('piper_observe',cameras=['head','left','right'],require_depth=False)
        r['after_frame']=after['frame_id'];r['image_paths']={k:v['inspection_image'] for k,v in after['cameras'].items()}
        # Only requested scene outcomes need auxiliary perception. Ordinary empty
        # preparation completes on feedback; still return images for inspection.
        if e['post_facts']:
            # Trial pickup first uses fresh RGB temporal evidence; assess requests
            # local depth only if ordinary rules cannot resolve the required fact.
            if e['name']!='trial_lift':
                t=time.monotonic();after=current_depth(ctx,after);r['perception_s']+=time.monotonic()-t
            receipt['observation']=after;r['after_frame']=after['frame_id']
            r['image_paths']={k:v['inspection_image'] for k,v in after['cameras'].items()}
            # BEFORE onsite evidence never certifies the AFTER pose.
            control_context={'measured_arrival':True,'before_flange_position':fk['position'],
                'final_measured_joints':final[:6].tolist(),'final_gripper_width_m':float(final[6])}
            if tool=='piper_move_pose':
                control_context['commanded_flange_delta_m']=(np.asarray(args['position'])-np.asarray(fk['position'])).tolist()
            r['post_control_context']=control_context
            question=(e['post']+' Separate control evidence: '+json.dumps(control_context)+
                '. Arrival is reported by the backend, not inferred from image pixels; the commanded displacement is not an independent measurement of cup motion. '
                'Use this to interpret the moving-camera views. Control arrival alone cannot prove pickup, retention or support.')
            t=time.monotonic()
            try:
                final_fk=ctx.value('piper_fk',joints=final[:6].tolist(),arm='left')
                post_robot=ctx.value('piper_status')
                if (not post_robot.get('motion_ready') or not _same_pose(final,post_robot['measured_state']['values']) or
                        post_robot['owner_status']['generation']!=robot['owner_status']['generation'] or
                        post_robot['bridge']['instance']!=robot['bridge']['instance']):
                    raise ValueError('Control/owner changed before result check')
                rules=context_for(ctx,after,robot=post_robot,action=e['name'],fk=fk,final_fk=final_fk,inputs=e)
                # Ordinary temporal rules always use before/current. The model
                # receives current-only when requested, avoiding history confusion.
                rules_prior=obs
                def fallback(context,observation,prompt,**kw):
                    if e.get('current_only_post'):kw['prior']=None
                    return visual(context,observation,prompt,**kw)
                post=assess(ctx,after,question,prior=rules_prior,cameras=tuple(e.get('cameras',['head','left'])),required_facts=e['post_facts'],context=rules,fallback=fallback)
            finally:r['perception_s']+=time.monotonic()-t
            r['visual_calls']+=post.get('visual_calls',0)
            r['wrist_depth_calls']=r.get('wrist_depth_calls',0)+post.get('wrist_depth_calls',0)
            rebound=post.pop('_checked_observation',None)
            if rebound is not None:
                after=rebound;receipt['observation']=after;r['after_frame']=after['frame_id']
                r['image_paths']={k:v['inspection_image'] for k,v in after['cameras'].items()}
            r.setdefault('condition_checks',[]).append({'boundary':'post','model_calls':post.get('visual_calls',0),'fact_sources':post.get('fact_sources',{}),'fallback_reason':post.get('fallback_reason')})
        else:
            post={**r['feedback_checks'],'visual_used':False,'fact_sources':{}}
        r['post']=post;r['ok']=post.get('ok') is True;r['reason']=post.get('reason')
        r['wall_budget']=after.get('wall_budget',receipt.get('wall_budget',r['wall_budget']))
        if r['ok']:r.pop('next_action',None)
        return r
    except Exception as exc:
        r['visual_calls']+=getattr(exc,'rax_visual_calls',0)
        r['wrist_depth_calls']=r.get('wrist_depth_calls',0)+getattr(exc,'rax_wrist_depth_calls',0)
        if hasattr(exc,'rax_condition_report'):r['condition_report']=exc.rax_condition_report
        if hasattr(exc,'rax_rule_metrics'):r['failed_rule_metrics']=exc.rax_rule_metrics
        r['reason']=str(exc)
        if r.get('dispatch_monotonic') and not r.get('arrival_monotonic'):
            r['motion_s']=time.monotonic()-r['dispatch_monotonic']
        try:
            error=json.loads(str(exc).split('MCP tool failed: ',1)[1])['error'];r['tool_error']=error
            if error.get('no_action_executed') is True and not r.get('receipt') and 'dispatch_monotonic' not in r:r['executed']=False
        except (ValueError,KeyError,IndexError,TypeError):pass
        r['failure_fix']='Stop; inspect actual outcome. Never replay to repair a failed postcheck.';return r
    finally:
        r['total_s']=time.monotonic()-start
        path=OUT/(str(time.time_ns())+'-'+e['name']+'.json');r['report_path']=str(path);path.write_text(json.dumps(r,indent=2))
        (OUT/'latest.json').write_text(json.dumps(r,indent=2))
        with (OUT/'timings.jsonl').open('a') as f:f.write(json.dumps({k:v for k,v in r.items() if k not in ('receipt','before_fk')})+'\n')

def empty_open(ctx,explicit_inputs):
    return move_once(ctx,dict(**_options(explicit_inputs),name='empty_open',tool='piper_gripper',args={'width_m':explicit_inputs['width_m']},
        pre='Inspect the jaw contents and visible space on the two opening sides.',
        pre_facts={'jaw_empty':'No held object between the pads','opening_sides_visible_and_unobstructed':'Outward opening space appears broadly free of objects/lines; no need to see every hidden underside'},
        post_facts={}))  # Empty opening completion is measured jaw arrival.

def stage(ctx,explicit_inputs):
    return move_once(ctx,dict(**_options(explicit_inputs),name='stage',tool='piper_move_pose',args=_motion_args(explicit_inputs),
        pre='Inspect empty-flight staging including finger rotation about the flange, not only translation. Metric robot geometry/path is checked separately. If a specific visible obstacle conflicts with the route, report it; do not require sight of every underside.',
        pre_facts={'jaw_empty':'No held object','route_visible_and_unobstructed':'Visible free space in the intended short empty-flight direction, away from cup/coaster/lines'},
        post_facts={}))  # Empty staging completion is measured arm arrival.

def approach_step(ctx,explicit_inputs):
    return move_once(ctx,dict(**_options(explicit_inputs),name='approach_step',tool='piper_move_pose',args=_motion_args(explicit_inputs),
        pre='Judge the overall short approach scene; do not require exact per-finger gap or pixel alignment.',
        pre_facts={'approach_scene':'Empty left jaw near the intended upright cup, no obvious contact, handle obstruction or route conflict'},
        post='Judge the overall resulting approach only, not an unplanned next descent.',
        post_facts={'approach_scene':'Cup remains upright and left jaw is in the expected region without obvious collision or entanglement'}))

# Task geometry values; these are target semantics, not relaxed arrival tolerances.
HOVER_TCP_ABOVE_RIM_M=.050   # 30mm TCP-to-tip plus20mm nominal tip height
ENTRY_TCP_BELOW_RIM_M=.040  # User-selected candidate depth; relative to CURRENT rim, live validation pending
MAX_APPROACH_SEGMENT_M=.055
GOAL_POSITION_ERROR_M=.004

def approach_goal(localization,fk,phase,measured14=None):
    """Compute current-layout flange target once from calibrated TCP, not old poses."""
    if phase not in ('hover','entry'):raise ValueError('phase must be hover or entry')
    if not localization.get('ok'):raise ValueError('Current localization failed')
    wall=np.asarray(localization['near_wall'],float)
    tcp=fk.get('effective_contact_tcp',{})
    T=np.asarray(tcp.get('T_base_tcp'),float);flange=np.asarray(fk['position'],float)
    if (wall.shape!=(3,) or T.shape!=(4,4) or not np.isfinite(wall).all()
            or not np.isfinite(T).all() or not np.isfinite(flange).all()
            or tcp.get('calibration_id')!='left-tcp140-tip170-20261104'):
        raise ValueError('Missing current calibrated TCP or finite localization')
    if T[2,2]>-0.98:raise ValueError('Downward staging orientation required')
    # URDF: gripper-base Rz90 and finger Rx90 map the opening axis to link6 X.
    normal=wall[:2]-np.asarray(localization['cup_center_estimate'],float)[:2]
    opening=T[:2,0]
    if (np.linalg.norm(normal)<.01 or np.linalg.norm(opening)<.9
            or abs(float(np.dot(normal,opening)))/(np.linalg.norm(normal)*np.linalg.norm(opening))<math.cos(math.radians(15))):
        raise ValueError('Need opening axis aligned with local cup-wall normal before approach')
    q=localization['observation']['measured14'] if measured14 is None else measured14
    if not .012<=q[6]<=.020:raise ValueError('Approach needs verified empty open jaw12..20mm')
    current=T[:3,3];xy_error=float(np.linalg.norm(current[:2]-wall[:2]))
    if phase=='hover' and xy_error>GOAL_POSITION_ERROR_M and current[2]<wall[2]+HOVER_TCP_ABOVE_RIM_M-.002:
        raise ValueError('Need verified clear upward staging before horizontal approach below rim')
    if phase=='entry' and xy_error>GOAL_POSITION_ERROR_M:
        raise ValueError('Need hover alignment before wall entry')
    goal_tcp=wall+np.array([0,0,HOVER_TCP_ABOVE_RIM_M if phase=='hover' else -ENTRY_TCP_BELOW_RIM_M])
    if phase=='entry':
        table_z=float(localization['table'][2])
        if not math.isfinite(table_z) or goal_tcp[2]-.030 <= table_z+.005:
            raise ValueError('Characterized wall depth leaves insufficient nominal tip/table space; do not deepen')
    goal_flange=goal_tcp-(current-flange) # TCP offset transformed exactly once
    delta=goal_flange-flange;distance=float(np.linalg.norm(delta))
    target=flange+delta*min(1.,MAX_APPROACH_SEGMENT_M/max(distance,1e-12))
    return {'phase':phase,'goal_tcp':goal_tcp.tolist(),'goal_flange':goal_flange.tolist(),
            'segment_flange':target.tolist(),'goal_distance_m':distance,
            'localization_frame':localization['before_frame'],
            'calibration_id':tcp['calibration_id'],'metric_clearance_certified':False}


APPROACH_MAX_STEPS=6
APPROACH_MAX_SECONDS=180
APPROACH_MIN_PROGRESS_M=.001
SCENE_CHANGE_M=.020


def _accumulate(parent,child):
    for key in ('visual_calls','wrist_depth_calls','perception_s','planning_s','motion_s','ik_calls'):
        parent[key]=parent.get(key,0)+child.get(key,0)


TARGET_SCENE_FACT='Compared with the saved target measurement, visible cup/coaster portions and surroundings show no evidence of displacement, touch or changed obstacle. Do not claim hidden surfaces are observed. Expected arm occlusion alone does not invalidate the measured target; do not demand the hidden rim to be remeasured.'


def approach(ctx,explicit_inputs):
    """Complete ONE bounded hover/entry subgoal; never close, lift or auto-retry.

    The measured base-frame target survives this approach's hover/entry calls.
    New RGB and feedback check each action boundary; depth is measured once before
    occlusion, not reacquired at every segment. Contact/other motion invalidates it.
    """
    start=time.monotonic();r={'function':'approach','ok':False,'executed':False,
        'motion_arrived':False,'task_completed':False,'visual_calls':0,'perception_s':0,
        'motion_s':0,'planning_s':0,'steps':[]}
    from approach_target import PATH,create,observe,localization,advance,save,invalidate,retain_no_dispatch
    target=None;resume_phase=None;pending=None
    try:
        if set(explicit_inputs)-{'phase','obstacles'}:
            raise ValueError('approach takes phase/obstacles only; targets come from fresh localization')
        phase=explicit_inputs.get('phase','hover')
        if phase not in ('hover','entry'):raise ValueError('Unknown approach phase')
        if phase=='hover':
            saved=json.loads(PATH.read_text()) if PATH.exists() else None
            if saved and saved.get('valid') and saved.get('phase')=='preparation_verified':
                target=saved;obs=observe(ctx,target);resume_phase='preparation_verified'
                target['phase']='hover';save(target)
                loc=localization(target,obs)
            else:
                loc=localize_supported(ctx,{})
                _accumulate(r,loc)
                if not loc.get('ok'):r['reason']=loc.get('reason','Localization failed');return r
                target=create(loc,ctx.value('piper_status'))
        else:
            target=json.loads(PATH.read_text())
            if target.get('phase')!='hover_verified':raise ValueError('Entry requires this approach verified hover target')
            obs=observe(ctx,target);resume_phase='hover_verified'
            target['phase']='entry';save(target)
            loc=localization(target,obs)
        r['target_file']=str(PATH);r['target_id']=target['id']
        for iteration in range(APPROACH_MAX_STEPS+1):
            if time.monotonic()-start>APPROACH_MAX_SECONDS:
                r['reason']='Approach time budget exhausted';return r
            if iteration:
                obs=observe(ctx,target,previous=pending)
                if obs is pending:r['receipt_observations_reused']=r.get('receipt_observations_reused',0)+1
                loc=localization(target,obs)
            else:obs=loc['observation']
            r['localization']={k:v for k,v in loc.items() if k not in ('observation','pre')}
            fk=ctx.value('piper_fk',joints=obs['measured14'][:6],arm='left')
            goal=approach_goal(loc,fk,phase);r['goal']=goal;r['after_frame']=obs['frame_id']
            r['goal_error_m']=goal['goal_distance_m']
            if goal['goal_distance_m']<=GOAL_POSITION_ERROR_M:
                return _finish_retained_approach(ctx,r,obs,phase,target)
            if iteration==APPROACH_MAX_STEPS:
                r['reason']='Approach segment budget exhausted';return r
            # Keep route checking even outside contact: the environment is not fully modeled.
            # Near rim/entry also checks the overall wall approach for obvious obstruction.
            near=phase=='entry' or goal['goal_distance_m']<=MAX_APPROACH_SEGMENT_M
            facts={'jaw_empty':'No object held','cup_upright':'Cup upright on table',
                'route_visible_and_unobstructed':'Relevant visible space for THIS proposed short approach is free of cup/handle/table/lines'}
            facts['target_scene_consistent']=TARGET_SCENE_FACT
            if near:facts['contact_scene']='Left jaw is generally aligned with the intended wall approach, without obvious collision or handle/table trapping; exact millimetre alignment is not an image requirement'
            step=move_once(ctx,{'name':'approach_segment','tool':'piper_move_pose',
                'args':{'position':goal['segment_flange'],'quaternion_xyzw':fk['quaternion_xyzw']},
                'observation':obs,'scene_reference':target['source_observation'],
                'obstacles':explicit_inputs.get('obstacles',[]),
                'pre':'Inspect this proposed approach using head and wrist. Resolve 2D overlap across views; do not certify a hidden surface or an entire swept volume.',
                'pre_facts':facts,'post_facts':{}})
            r['steps'].append(compact(step));_accumulate(r,step)
            if step.get('executed') is True:r['executed']=True
            elif step.get('executed') is None:r['executed']=None
            r['motion_arrived']=step.get('motion_arrived',False)
            if not step.get('ok'):
                r['reason']=step.get('reason');r['next_action']=step.get('next_action','observe_only')
                for key in ('operator_file','operator_request'):
                    if key in step:r[key]=step[key]
                return r
            advance(ctx,target,step)
            receipt=step['receipt'];pending=receipt.get('observation')
            if pending is None:
                pending=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=False)
            final_fk=ctx.value('piper_fk',joints=receipt['final14'][:6],arm='left')
            error=math.dist(final_fk['effective_contact_tcp']['position'],goal['goal_tcp'])
            r.update(goal_error_m=error,after_frame=pending['frame_id'])
            if error<=GOAL_POSITION_ERROR_M:
                return _finish_retained_approach(ctx,r,pending,phase,target)
            if goal['goal_distance_m']-error<APPROACH_MIN_PROGRESS_M:
                r['reason']='No measured approach progress; inspect, never replay';return r
            # Fresh RGB checks scene consistency; the measured base-frame goal persists.
        r['reason']='Approach bounds exhausted';return r
    except Exception as exc:r['reason']=str(exc);return r
    finally:
        if not r['ok']:
            r.setdefault('next_action','observe_only')
            if retain_no_dispatch(target,r,resume_phase):
                r.update(target_retained=True,recovery_remaining=1,
                    next_action='retry_same_function_with_fresh_checks')
            else:invalidate(r.get('reason','Approach interrupted'))
        r['total_s']=time.monotonic()-start
        path=OUT/(str(time.time_ns())+'-approach.json');r['report_path']=str(path)
        path.write_text(json.dumps(r,indent=2));(OUT/'latest.json').write_text(json.dumps(r,indent=2))
        with (OUT/'timings.jsonl').open('a') as f:f.write(json.dumps(r)+'\n')


def _finish_retained_approach(ctx,r,obs,phase,target):
    from approach_target import save
    result=_verify_approach_goal(ctx,r,obs,phase,prior=target['source_observation'])
    if result['ok']:
        target['phase']=phase+'_verified';save(target)
    return result


def _verify_approach_goal(ctx,r,obs,phase,prior=None):
    """Finish a measured positioning subgoal, without claiming grasp readiness.

    Entry's route check and grasp's current qualitative check cover the next
    action boundary. Do not add an independent model gate between them merely
    to re-prove hidden inner/outer pad topology after the rim leaves view.
    """
    if not observation_recent(obs):
        obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=False)
        fk=ctx.value('piper_fk',joints=obs['measured14'][:6],arm='left')
        r['goal_error_m']=math.dist(fk['effective_contact_tcp']['position'],r['goal']['goal_tcp'])
    r['after_frame']=obs['frame_id']
    error=r.get('goal_error_m',float('inf'))
    ok=phase in ('hover','entry') and math.isfinite(error) and 0<=error<=GOAL_POSITION_ERROR_M
    r['post']={'ok':ok,'visual_used':False,'basis':'measured_target_arrival',
               'reason':'Positioning only; next action checks current scene, trial lift checks actual pickup'}
    r['scene_verified']=False;r['task_completed']=ok;r['ok']=ok
    r['reason']='Measured approach target reached; scene/grasp verification deferred to next action' if ok else 'Measured approach target not confirmed'
    r['next_action']='next_rehearsal_function' if r['ok'] else 'observe_only'
    return r


PREPARATION_STANDOFF_M=.100
PREPARATION_MAX_STEPS=10
PREPARATION_MAX_ROTATION_DEG=35
PREPARATION_ROTATION_ERROR_DEG=3

def preparation_goal(localization,fk):
    """Empty staging target outside the newly observed cup, in left_base_link."""
    if not localization.get('ok'):raise ValueError('Need supported current localization')
    wall=np.asarray(localization['near_wall'],float);center=np.asarray(localization['cup_center_estimate'],float)
    if wall.shape!=(3,) or center.shape!=(3,) or not np.isfinite([wall,center]).all():raise ValueError('Invalid scene points')
    normal=wall[:2]-center[:2];length=np.linalg.norm(normal)
    if length<.01:raise ValueError('Unresolved cup-wall normal')
    normal=normal/length;rot=Rotation.from_quat(fk['quaternion_xyzw']);orientations=[]
    for sign in (1,-1):
        x=np.array([sign*normal[0],sign*normal[1],0.]);z=np.array([0.,0.,-1.]);y=np.cross(z,x)
        target=Rotation.from_matrix(np.column_stack([x,y,z]));orientations.append(target)
    target=min(orientations,key=lambda x:(x*rot.inv()).magnitude())
    tcp=fk.get('effective_contact_tcp',{})
    if tcp.get('calibration_id')!='left-tcp140-tip170-20261104':raise ValueError('Missing installed TCP identity')
    position=[*(wall[:2]+normal*PREPARATION_STANDOFF_M),wall[2]+HOVER_TCP_ABOVE_RIM_M+.140]
    return {'position':position,'quaternion_xyzw':target.as_quat().tolist(),
            'near_wall':wall.tolist(),'localization_frame':localization['before_frame'],
            'metric_clearance_certified':False}


def preparation_candidates(fk,goal):
    """Small deterministic alternatives for SDK IK/path checks; no motion or solver."""
    p=np.asarray(fk['position'],float);g=np.asarray(goal['position'],float)
    current=Rotation.from_quat(fk['quaternion_xyzw']);target=Rotation.from_quat(goal['quaternion_xyzw'])
    angle=(target*current.inv()).magnitude();cq=current.as_quat().tolist()
    def bounded(delta):
        d=np.asarray(delta,float);return (p+d*min(1.,MAX_APPROACH_SEGMENT_M/max(np.linalg.norm(d),1e-12))).tolist()
    # Raise the flange before rotating the long fingers or translating at low height.
    if p[2]<g[2]-GOAL_POSITION_ERROR_M:
        return [{'position':bounded([0,0,min(.045,g[2]-p[2])]),'quaternion_xyzw':cq}]
    if angle>math.radians(PREPARATION_ROTATION_ERROR_DEG):
        ratio=min(1.,math.radians(PREPARATION_MAX_ROTATION_DEG)/angle)
        q=Slerp([0,1],Rotation.concatenate([current,target]))(ratio).as_quat().tolist()
        # Advance along X while remaining on the current lateral side of the cup.
        # This encodes the observed reachability issue without reusing world targets.
        advance=bounded([g[0]-p[0],0,0])
        values=[{'position':advance,'quaternion_xyzw':q},
                {'position':advance,'quaternion_xyzw':cq},
                {'position':p.tolist(),'quaternion_xyzw':q}]
    else:
        values=[{'position':bounded(g-p),'quaternion_xyzw':target.as_quat().tolist()}]
    unique=[]
    for value in values:
        if value not in unique and (math.dist(value['position'],p)>1e-5 or
                (Rotation.from_quat(value['quaternion_xyzw'])*current.inv()).magnitude()>1e-4):unique.append(value)
    return unique


def prepare_for_approach(ctx,explicit_inputs=None):
    """Prepare for cup approach from measured state, never home/zero or agent deltas.

    Finite normal Python function: current observations -> target/IK/path -> action.
    No main-agent decision between its bounded segments, no action retry on failure.
    """
    start=time.monotonic();r={'function':'prepare_for_approach','ok':False,'executed':False,'task_completed':False,
        'motion_arrived':False,'visual_calls':0,'perception_s':0,'planning_s':0,'motion_s':0,'ik_calls':0,'steps':[]}
    from approach_target import create,observe,localization,advance,save,invalidate
    target=None;pending=None
    try:
        e=explicit_inputs or {}
        if set(e)-{'obstacles'}:raise ValueError('prepare accepts obstacles only; targets derive from current state')
        goal=None
        for iteration in range(PREPARATION_MAX_STEPS):
            if target is None:
                loc=localize_supported(ctx,{})
                r['metric_localization_count']=r.get('metric_localization_count',0)+1
                if loc.get('ok') and not observation_recent(loc['observation'],require_depth=True):
                    # One read-only remeasurement at the function boundary. Preserve
                    # the first child's costs even if the fresh measurement fails.
                    r['visual_calls']+=loc.get('visual_calls',0);r['perception_s']+=loc.get('perception_s',0)
                    r['initial_localization_frame']=loc.get('before_frame')
                    loc=localize_supported(ctx,{'previous':loc})
                    r['metric_localization_count']+=1
                    r['localization_boundary_refresh']=True
            else:
                obs=observe(ctx,target,previous=pending);loc=localization(target,obs)
                if obs is pending:r['receipt_observations_reused']=r.get('receipt_observations_reused',0)+1
            r['visual_calls']+=loc.get('visual_calls',0);r['perception_s']+=loc.get('perception_s',0)
            if loc.get('observation_rebinding'):r.setdefault('observation_rebindings',[]).append(loc['observation_rebinding'])
            r['localization']= {k:v for k,v in loc.items() if k not in ('observation','pre')}
            if not loc.get('ok'):r['reason']=loc.get('reason','Localization failed');return r
            if target is None:target=create(loc,ctx.value('piper_status'),phase='preparation')
            r['localization']['measurement_frame']=target['source_frame']
            obs=loc['observation'];q=obs['measured14']
            if obs.get('wall_budget',{}).get('remaining_seconds',0)<=90:r['reason']='Finish reserve';return r
            fk=ctx.value('piper_fk',joints=q[:6],arm='left')
            if goal is None:goal=preparation_goal(loc,fk);r['goal']=goal
            r['target_id']=target['id']
            angle=math.degrees((Rotation.from_quat(goal['quaternion_xyzw'])*Rotation.from_quat(fk['quaternion_xyzw']).inv()).magnitude())
            distance=math.dist(goal['position'],fk['position']);r.update(goal_error_m=distance,orientation_error_deg=angle,after_frame=obs['frame_id'])
            if .012<=q[6]<=.020 and distance<=GOAL_POSITION_ERROR_M and angle<=PREPARATION_ROTATION_ERROR_DEG:
                # Empty-jaw facts were established by the preceding checked action;
                # when already prepared, verify them once instead of manufacturing a move.
                if not r['steps']:
                    obs=current_depth(ctx,obs)
                    fact=assess(ctx,obs,'Verify current empty prepared jaw only; do not claim grasp or hidden clearance.',required_facts={'jaw_empty':'No held object between pads','target_scene_consistent':TARGET_SCENE_FACT},prior=target['source_observation'],context=context_for(ctx,obs,robot=ctx.value('piper_status'),action='prepare'),fallback=visual)
                    r['visual_calls']+=fact.get('visual_calls',0)
                    r['perception_s']+=fact['perception_s'];r['post']=fact
                    if not fact.get('ok'):r['reason']='Prepared pose but jaw state unconfirmed';return r
                target['phase']='preparation_verified';save(target)
                r.update(ok=True,task_completed=True,reason='Empty preparation pose/opening verified; approach and grasp remain untested')
                return r
            if q[6]<.012:
                step=move_once(ctx,{'name':'preparation_open','tool':'piper_gripper',
                    'args':{'width_m':min(.016,q[6]+.0095)},'observation':obs,'scene_reference':target['source_observation'],
                    'pre':'Inspect empty jaw opening sides and visible scene consistency.',
                    'pre_facts':{'jaw_empty':'No held object between pads',
                        'opening_sides_visible_and_unobstructed':'Outward opening space appears broadly free of objects/lines; hidden underside proof not required',
                        'target_scene_consistent':TARGET_SCENE_FACT},'post_facts':{}})
            elif q[6]>.020:
                r['reason']='Unexpected opening outside characterized preparation range; no automatic closing';return r
            else:
                candidates=preparation_candidates(fk,goal)
                if not candidates:r['reason']='No useful bounded staging candidate';return r
                step=move_once(ctx,{'name':'preparation_segment','tool':'piper_move_pose','args':candidates[0],
                    'pose_candidates':candidates,'obstacles':e.get('obstacles',[]),
                    'observation':obs,'scene_reference':target['source_observation'],
                    'pre':'Inspect current empty-flight route including the proposed finger ROTATION and translation. The target is an outside-cup staging pose. Report concrete objects/lines conflicting with this short route; do not require hidden underside proof.',
                    'pre_facts':{'target_scene_consistent':TARGET_SCENE_FACT,'jaw_empty':'No held object','route_visible_and_unobstructed':'Relevant visible space for this short translation AND rotation is free of objects and lines'},'post_facts':{}})
            r['steps'].append(compact(step))
            for key in ('visual_calls','wrist_depth_calls','perception_s','planning_s','motion_s','ik_calls'):r[key]=r.get(key,0)+step.get(key,0)
            if step.get('executed') is True:r['executed']=True
            elif step.get('executed') is None:r['executed']=None
            r['motion_arrived']=step.get('motion_arrived',False)
            if not step.get('ok'):
                r['reason']=step.get('reason');r['next_action']=step.get('next_action','inspect_failed_step');return r
            advance(ctx,target,step)
            pending=step['receipt'].get('observation')
        r['reason']='Bounded preparation step budget exhausted; no autonomous retry'
        return r
    except Exception as exc:r['reason']=str(exc);return r
    finally:
        if not r['ok']:invalidate(r.get('reason','Preparation interrupted'))
        r['total_s']=time.monotonic()-start
        path=OUT/(str(time.time_ns())+'-prepare.json');r['report_path']=str(path)
        path.write_text(json.dumps(r,indent=2));(OUT/'latest.json').write_text(json.dumps(r,indent=2))
        with (OUT/'timings.jsonl').open('a') as f:f.write(json.dumps(r)+'\n')


def _options(inputs):
    return {k:inputs[k] for k in ('operator_file','cameras','obstacles','scene_binding','observation','target_report') if k in inputs}

def _motion_args(inputs):
    return {k:v for k,v in inputs.items() if k not in ('operator_file','cameras','obstacles','scene_binding','observation')}

def compact(r):
    # Full evidence is saved in report_path; normal main-agent replies stay small.
    excluded={'receipt','before_fk','pre','post','localization','observation','geometry','candidate_checks'}
    result={k:v for k,v in r.items() if k not in excluded}
    if 'steps' in result:
        keys={'function','ok','executed','motion_arrived','task_completed','reason',
              'total_s','visual_calls','wrist_depth_calls','perception_s','planning_s','motion_s',
              'report_path','operator_file','operator_request','next_action'}
        result['steps']=[{k:v for k,v in compact(step).items() if k in keys} for step in result['steps']]
    return result


def snapshot(ctx,explicit_inputs=None):
    """One read-only native Context entry for startup/resume, no decision or move."""
    import hashlib
    if explicit_inputs:raise ValueError('snapshot takes no inputs')
    start=time.monotonic();status=ctx.value('status')
    obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=False)
    r=status['robot_status'];path=OUT/(str(time.time_ns())+'-snapshot.json')
    path.write_text(json.dumps({'status':status,'observation':obs},indent=2))
    reports=sorted([*OUT.glob('*-prepare.json'),*OUT.glob('*-approach.json')])
    last=None
    if reports:
        saved=json.loads(reports[-1].read_text())
        last={k:saved.get(k) for k in ('function','ok','task_completed','reason','report_path','after_frame')}
        last['phase']=saved.get('goal',{}).get('phase')
    return {'function':'snapshot','read_only':True,'report_path':str(path),
        'control_uncertain':status.get('control_uncertain'),
        'motion_ready':r.get('motion_ready'),'owner_generation':r['owner_status']['generation'],
        'bridge_phase':r['bridge']['phase'],'bridge_instance':r['bridge']['instance'],
        'measured14':obs['measured14'],'state_binding':obs['state_binding'],
        'remaining_seconds':obs['wall_budget']['remaining_seconds'],
        'operator_requests':status.get('operator_requests',[]),
        'frame_id':obs['frame_id'],'image_paths':{k:v['inspection_image'] for k,v in obs['cameras'].items()},
        'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},
        'last_subgoal_report':last,'historical_report_is_current_scene_evidence':False,
        'total_s':time.monotonic()-start}

if __name__=='__main__':
    import sys
    from api.runtime import Context
    name=sys.argv[1];inputs=json.loads(sys.argv[2]);functions={'snapshot':snapshot,'prepare_for_approach':prepare_for_approach,'empty_open':empty_open,'stage':stage,'approach':approach,'approach_step':approach_step,'move_once':move_once}
    with Context() as ctx: print(json.dumps(compact(functions[name](ctx,inputs)),indent=2))
