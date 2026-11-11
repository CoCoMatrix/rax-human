"""Fresh coaster localization and bounded placement steps for v0.3 rehearsal."""
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from perception import OUT, visual, _contours, normalized_pixel, observation_pose_matches, observation_recent
from rehearsal import move_once, compact


def _save(r, start):
    r['total_s'] = time.monotonic() - start
    path = OUT / (str(time.time_ns()) + '-' + r['function'] + '.json')
    r['report_path'] = str(path)
    path.write_text(json.dumps(r, indent=2))
    with (OUT / 'timings.jsonl').open('a') as f:
        f.write(json.dumps(compact(r)) + '\n')


def localize_coaster(ctx, inputs):
    """No motion. Bind current coaster depth to this session's verified held cup."""
    start = time.monotonic()
    r = dict(function='localize_coaster', ok=False, executed=False, motion_s=0,
             planning_s=0, perception_s=0, visual_calls=0, metric_localization_count=0)
    try:
        prep, entry, lift = [json.loads(Path(inputs[k]).read_text()) for k in
                             ('preparation_report', 'entry_report', 'lift_report')]
        if not all(x.get('ok') for x in (prep, entry, lift)) or lift['function'] != 'trial_lift':
            raise ValueError('Successful preparation, entry and single trial lift required')
        obs = ctx.value('piper_observe', cameras=['head','left','right'], require_depth=True)
        status = ctx.value('status'); robot = status['robot_status']
        if status.get('control_uncertain') or not robot.get('motion_ready'):
            raise ValueError('Unknown control; no placement target')
        last = lift['receipt']; prior = last['observation']
        if (last['owner_generation'] != robot['owner_status']['generation'] or
                not observation_pose_matches(last['final14'], obs['measured14']) or
                obs['finger_geometry']['geometry_identity'] != prior['finger_geometry']['geometry_identity']):
            raise ValueError('Lift pose, owner or geometry changed')
        source=prep['localization']
        original=json.loads((OUT/'approach-target.json').read_text())
        if (original['bridge_instance']!=robot['bridge']['instance'] or
                original['owner_generation']!=robot['owner_status']['generation'] or
                prep['target_id']!=original['id'] or entry.get('target_id')!=original['id']):
            raise ValueError('Pickup provenance belongs to another approach/session')
        cam=obs['cameras']['head']
        if (obs.get('calibration_sha256')!=original.get('source_calibration') or
                cam['K']!=original['head_K'] or cam['T_base_camera']!=original['head_T_base_camera']):
            raise ValueError('Head calibration changed')
        if (source.get('measurement_frame')!=original['source_frame'] or
                source.get('contours')!=original.get('source_contours') or
                not source.get('contours',{}).get('coaster')):
            raise ValueError('Missing/mismatched source coaster identity; prepare report contract incomplete')
        im = cv2.imread(obs['cameras']['head']['inspection_image'])
        circles = _contours(im)
        old_coaster=np.asarray(prep['localization']['contours']['coaster'],float)
        matches=circles[(np.linalg.norm(circles[:,:2]-old_coaster[:2],axis=1)<5)&(abs(circles[:,2]-old_coaster[2])<4)]
        if len(matches)==1:
            guess=matches[0,:2];r['localization_source']='current_contour_association'
            # This only identifies the target. The next transfer's current-view
            # precondition still checks held-object state and payload route.
        else:
            r['visual_calls'] += 1;t=time.monotonic()
            try:
                v=visual(ctx,obs,'Identify only the intended circular coaster in CURRENT HEAD. '
                    'Return coaster_center_norm [x,y] normalized0..1000. No support or clearance judgment.',cameras=('head',))
            finally:r['perception_s']+=time.monotonic()-t
            r['pre']=v
            if not v.get('ok'):raise ValueError('Coaster identity ambiguous')
            guess=normalized_pixel(v['coaster_center_norm'],im.shape[1],im.shape[0])
            r['localization_source']='semantic_fallback'
        eligible = circles[np.linalg.norm(circles[:,:2]-guess,axis=1)<20]
        if len(eligible) != 1: raise ValueError('Coaster circle association ambiguous')
        old = eligible[0]
        # Fresh depth after semantic latency; verify the actual coaster contour again.
        fresh = obs if observation_recent(obs,require_depth=True) else ctx.value('piper_observe', cameras=['head','left','right'], require_depth=True)
        if not observation_pose_matches(obs['measured14'], fresh['measured14']):
            raise ValueError('State changed during localization')
        im2 = cv2.imread(fresh['cameras']['head']['inspection_image'])
        circles2 = _contours(im2)
        candidates = circles2[(np.linalg.norm(circles2[:,:2]-old[:2],axis=1)<5)&(abs(circles2[:,2]-old[2])<4)]
        if len(candidates) != 1: raise ValueError('Coaster moved or became ambiguous')
        cam = fresh['cameras']['head']
        if (fresh.get('calibration_sha256')!=original.get('source_calibration') or
                cam['K']!=original['head_K'] or cam['T_base_camera']!=original['head_T_base_camera']):
            raise ValueError('Head calibration changed')
        pixel = candidates[0,:2].astype(float).tolist()
        depth = ctx.value('piper_pixel_to_position', camera='head', frame_id=fresh['frame_id'], pixels=[pixel], radius=3)
        coaster = depth['points'][0]['position']; r['metric_localization_count']=1
        # Entry is the last open-jaw cup pose; pickup verified the object followed it.
        er = json.loads(Path(entry['steps'][-1]['report_path']).read_text())['receipt']
        efk = ctx.value('piper_fk', joints=er['final14'][:6], arm='left')
        fk = ctx.value('piper_fk', joints=fresh['measured14'][:6], arm='left')
        cup = np.array(source['cup_center_estimate']) + np.array(fk['position']) - np.array(efk['position'])
        goal = np.array(fk['position']); goal[:2] += np.array(coaster[:2])-cup[:2]
        support_z = efk['position'][2] + coaster[2] - source['table'][2]
        if np.linalg.norm(goal[:2]-np.array(fk['position'][:2])) > .22:
            raise ValueError('Coaster outside bounded220mm transfer scope')
        if not 0 < fk['position'][2]-support_z < .065:
            raise ValueError('Inconsistent predicted support height')
        r.update(ok=True, reason='Fresh coaster depth; held-center estimate from verified pickup, not contact proof',
                 observation=fresh, after_frame=fresh['frame_id'], source_observation=obs,
                 coaster_center=coaster, coaster_pixel=pixel, cup_center_estimate=cup.tolist(),
                 transfer_goal=goal.tolist(), predicted_support_flange_z=support_z,
                 owner_generation=robot['owner_status']['generation'], bridge_instance=robot['bridge']['instance'],
                 expected14=fresh['measured14'], geometry_identity=fresh['finger_geometry']['geometry_identity'],
                 target_id=prep['target_id'], source_reports=inputs, wall_budget=depth['wall_budget'])
    except Exception as exc: r['reason']=str(exc)
    finally: _save(r,start)
    return r


def transfer_step(ctx, inputs):
    """One <=55mm segment; caller inspects each result before continuing."""
    start=time.monotonic()
    r=dict(function='transfer_step',ok=False,executed=False,visual_calls=0,
           perception_s=0,planning_s=0,motion_s=0)
    try:
        target=json.loads(Path(inputs['target_report']).read_text())
        previous=json.loads(Path(inputs.get('previous_report',inputs['target_report'])).read_text())
        if not target.get('ok') or not previous.get('ok'):
            raise ValueError('Valid target and successful previous step required')
        obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=True)
        status=ctx.value('status'); robot=status['robot_status']
        if (status.get('control_uncertain') or not robot.get('motion_ready') or
                robot['bridge']['instance']!=target['bridge_instance'] or
                robot['owner_status']['generation']!=target['owner_generation'] or
                obs['finger_geometry']['geometry_identity']!=target['geometry_identity'] or
                not observation_pose_matches(previous['expected14'],obs['measured14'])):
            raise ValueError('Changed owner/state/geometry; stop transport')
        fk=ctx.value('piper_fk',joints=obs['measured14'][:6],arm='left')
        delta=np.array(target['transfer_goal'])-np.array(fk['position']); delta[2]=0
        distance=float(np.linalg.norm(delta))
        if distance<.002: raise ValueError('Transport already at goal; do not replay')
        delta*=min(1.,.055/distance)
        step=move_once(ctx,dict(name='transfer',tool='piper_move_pose',args={'delta':delta.tolist()},
            observation=obs,scene_reference=target['observation'],target_report=inputs['target_report'],
            pre='Compare with the coaster localization frame. The cup intentionally moves, the coaster should stay fixed. Check the held cup and this horizontal route.',
            pre_facts={'transfer_ready':'Cup appears held clear of support, with no obvious payload collision or entanglement along this short route',
                       'coaster_unchanged':'Visible coaster and surrounding table show no evidence of displacement; expected partial occlusion alone is acceptable'},
            post='Check overall cup retention and route outcome; minor tilt is not by itself failure.',
            post_facts={'cup_retained':'Cup followed the gripper and remains broadly stable at the pads, with no obvious slip/drop/collision'}))
        for k in ('ok','executed','visual_calls','wrist_depth_calls','perception_s','planning_s','motion_s','ik_calls','reason','after_frame','wall_budget','operator_request','operator_file','next_action'):
            if k in step:r[k]=step[k]
        r['step_report']=step['report_path'];r['target_report']=inputs['target_report']
        if step.get('ok'):
            r['expected14']=step['receipt']['final14']
            arrived=ctx.value('piper_fk',joints=r['expected14'][:6],arm='left')
            r['remaining_xy_m']=math.dist(arrived['position'][:2],target['transfer_goal'][:2])
            r['transport_complete']=r['remaining_xy_m']<.002
    except Exception as exc:r['reason']=str(exc)
    finally:_save(r,start)
    return r


def placement_context(ctx, inputs):
    """Bind a placement action to the measured coaster and last successful pose.

    The old frame supplies identity only; current images must establish alignment
    and support. No historical visual approval is carried into a dispatch.
    """
    target=json.loads(Path(inputs['target_report']).read_text())
    previous=json.loads(Path(inputs['previous_report']).read_text())
    if not target.get('ok') or not previous.get('ok'):
        raise ValueError('Placement requires successful target and previous action')
    expected=previous.get('expected14') or previous['receipt']['final14']
    obs=inputs.get('observation')
    if not observation_recent(obs,require_depth=True):
        obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=True)
    status=ctx.value('status'); robot=status['robot_status']
    if (status.get('control_uncertain') or not robot.get('motion_ready') or
            robot['bridge']['instance']!=target['bridge_instance'] or
            robot['owner_status']['generation']!=target['owner_generation'] or
            obs['finger_geometry']['geometry_identity']!=target['geometry_identity'] or
            not observation_pose_matches(expected,obs['measured14'])):
        raise ValueError('Changed owner/state/geometry; stop placement')
    return {'observation':obs}, placement_identity(target)


def placement_identity(target):
    x,y=target['coaster_pixel']
    h,w=target['observation']['cameras']['head']['shape_hw']
    return (f'Target coaster was localized in the fixed HEAD view at {100*x/w:.1f}% width, '
            f'{100*y/h:.1f}% height. These are historical identity coordinates, NOT proof of current support. '
            'ONLY CURRENT head/left images are supplied. Cup and arm have intentionally moved; '
            'do not identify another circle elsewhere as the target. Use qualitative overall alignment, '
            'upright posture, visible coaster edge and absence of gross displacement or tilt. '
            'Do not require an exposed full cup base, exact concentricity or a millimetre gap. ')
