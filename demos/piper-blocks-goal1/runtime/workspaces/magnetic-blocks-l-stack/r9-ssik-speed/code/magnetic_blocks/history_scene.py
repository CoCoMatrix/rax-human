"""Task-local reuse of an observed scene after a verified base-pair placement.

No hardware, motion, cache service or new task state machine. Read immutable
function reports; preserve source frames, update only the block actually moved.
"""
import copy
import hashlib
from pathlib import Path
import numpy as np
from scene_checks import (PerceptionUnknown, read_report, completed_action_state,
                          scene_control, scene_geometry, current_image)


def released_scene(ctx, obs, robot, previous_report, *, stationary_report=None, invalidated=False):
    if invalidated:
        raise PerceptionUnknown('Scene explicitly invalidated by reset/displacement/contact')
    current_image(obs, 'head')
    endpoint=obs; path=str(previous_report); history=[]; seen=set(); release=None
    # Only a completed open withdrawal and subsequent empty inspection moves
    # may bridge release to the next block. No unrecorded task actions accepted.
    for index in range(64):
        resolved=str(Path(path).resolve())
        if resolved in seen:raise PerceptionUnknown('Cyclic report history')
        seen.add(resolved)
        record=read_report(path)
        r=completed_action_state(record,endpoint,robot if index==0 else None,
                                 require_current=index==0)
        if not r.get('ok') or r['stage'] not in ('release','retreat','empty_inspection'):
            raise PerceptionUnknown('Scene reuse requires verified release/empty action history')
        start_robot=r.get('start_status',{}).get('robot_status',{})
        if start_robot.get('bridge',{}).get('instance') != robot.get('bridge',{}).get('instance'):
            raise PerceptionUnknown('Scene history belongs to another bridge instance')
        history.append({'report':path,'sha256':hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                        'stage':r['stage'],'request_id':r['receipts'][-1]['result']['owner_request_id']})
        if r['stage']=='release':release=r;break
        endpoint=r['initial_observation']
        path=record['inputs']['previous_report']
    if release is None or not any(h['stage']=='retreat' for h in history):
        raise PerceptionUnknown('No verified released-and-withdrawn base placement in history')
    evidence=next((f for f in release['facts'] if 'scene' in f and 'payload_origin' in f),None)
    if evidence is None:raise PerceptionUnknown('Release lacks the measured scene and payload origin')
    scene=copy.deepcopy(evidence['scene']);scene_control(scene,robot);scene_geometry(scene,obs)
    origin=evidence['payload_origin']
    if origin.get('color')!='orange':raise PerceptionUnknown('This scene update covers the orange base block only')
    if stationary_report:
        source=read_report(stationary_report)
        original=source['result']['facts'][0]['scene']
        scene_control(original,robot);scene_geometry(original,obs)
        if not source['result'].get('ok'):raise PerceptionUnknown('Unverified stationary scene source')
        for color in ('red','green'):
            if original['blocks'][color]['top_center']['surface_position_m'] != scene['blocks'][color]['top_center']['surface_position_m']:
                raise PerceptionUnknown('Stationary source disagrees with placement scene')
    # Same downward orientation during the validated transfer. Preserve the
    # observed block-to-flange relation, using actual release feedback, not the
    # requested target. This is an estimate, explicitly NOT new depth.
    release_q=release['receipts'][-1]['result']['final14'][:6]
    pose=np.asarray(ctx.value('piper_fk',joints=release_q)['position'])
    block=scene['blocks']['orange'];point=block['top_center']
    if point.get('measurement_kind')=='action_derived':
        raise PerceptionUnknown('Refuse to apply the same placement offset twice')
    point['source_surface_position_m']=copy.deepcopy(point['surface_position_m'])
    point['surface_position_m']=(np.asarray(point['surface_position_m'])+pose-np.asarray(origin['grasp_flange'])).tolist()
    point.update(measurement_kind='action_derived',action_frame=release['measured']['frame_id'],
                 meaning='Original measured block/flange relation translated by actual verified placement; not new depth')
    block['pixels_not_current']=True
    # This starts a NEW empty approach after a loaded placement. The previous
    # empty history ended before orange grasp and cannot be concatenated with
    # the upcoming green pregrasp. Keep it as provenance, not active history.
    if 'retention' in scene:
        scene['prior_empty_retention']=scene.pop('retention')
    scene['reuse']={'history':history,'current_observation_frame':obs['frame_id'],
        'placed_flange_m':pose.tolist(),
        'stationary_colors':['red','green'],'moved_color':'orange',
        'invalidates_on':['owner/calibration change','reset','observed displacement','unrecorded contact/action'],
        'weak_magnetism':'No automatic invalidation from mere proximity; visible displacement still invalidates',
        'limits':'Conditional same-run tabletop scene; wrist optical depth cannot supply base XYZ'}
    reject_visible_displacement(scene,obs)
    return scene


def validate_released_history(scene,obs,anchor):
    """Check the prior placement provenance when a new empty approach is rooted."""
    history=scene.get('reuse',{}).get('history',[])
    if not history or history[-1]['stage']!='release':raise PerceptionUnknown('Missing released-scene provenance')
    for h in history:
        raw=Path(h['report']).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=h['sha256']:raise PerceptionUnknown('Released-scene history changed')
        r=read_report(h['report'])['result']
        if not r.get('ok') or r['stage']!=h['stage'] or not r.get('action_completed'):
            raise PerceptionUnknown('Unverified released-scene history')
        bridge=r['start_status']['robot_status']['bridge']['instance']
        if bridge!=scene['control_identity']['bridge_instance']:raise PerceptionUnknown('Released scene owner changed')
    completed_action_state(read_report(history[0]['report']),anchor,require_current=False)
    release=read_report(history[-1]['report'])['result']
    fact=next(f for f in release['facts'] if 'scene' in f and 'payload_origin' in f)
    original=fact['scene'];scene_geometry(original,obs)
    for color in ('red','green'):
        if scene['blocks'][color]!=original['blocks'][color]:raise PerceptionUnknown('Stationary block data changed')
    expected=np.array(original['blocks']['orange']['top_center']['surface_position_m'])+np.array(scene['reuse']['placed_flange_m'])-np.array(fact['payload_origin']['grasp_flange'])
    if not np.allclose(expected,scene['blocks']['orange']['top_center']['surface_position_m'],atol=1e-10):
        raise PerceptionUnknown('Moved-block placement estimate changed')


def reject_visible_displacement(scene,obs,*,associated_colors=()):
    """Detect an obvious competing placement; partial/absent silhouette is not movement."""
    from perception import component
    from scene_checks import ViewUnavailable
    try:image=current_image(obs,'head')
    except ViewUnavailable as exc:return {'available':False,'why':str(exc),'unassociated_candidates':[]}
    head=obs['cameras']['head'];unassociated=[]
    camera_base=np.linalg.inv(np.asarray(head['T_base_camera']));K=np.asarray(head['K'])
    from ordinary_checks import visible_near_point
    for color,block in scene['blocks'].items():
        # Associate the expected object before considering a whole-image color
        # candidate. A partial target and a full background patch can coexist.
        if color in associated_colors:continue
        if visible_near_point(obs,color,block['top_center']['surface_position_m'])['visible']:continue
        try:visible,_=component(image,color)
        except PerceptionUnknown:continue  # incomplete silhouette is expected under occlusion
        x,y,w,h=visible['bbox_xywh']
        point=camera_base@np.r_[block['top_center']['surface_position_m'],1.]
        if point[2]<=0:raise PerceptionUnknown('Retained block is behind the calibrated head camera')
        uv=K@point[:3];expected=uv[:2]/uv[2]
        # This only catches conspicuous movement, not millimetre alignment.
        # Do not reuse historical pixels to obtain current depth.
        if np.linalg.norm(expected-[x+w/2,y+h/2])>max(w,h):
            # Color alone cannot establish identity after occlusion. The active
            # pick/place still needs its own positive bound head/wrist relation.
            unassociated.append({'color':color,'bbox_xywh':[x,y,w,h],
                'why':'Competing color region has no target association; not evidence of displacement'})
    return {'available':True,'unassociated_candidates':unassociated,
            'scope':'No relocalization or invented coordinates from unrelated background colors'}


def scene_height_bounds(scene):
    plane=np.asarray(scene['table']['z_ax_by_c']); blocks={}
    for color,b in scene['blocks'].items():
        p=np.asarray(b['top_center']['surface_position_m'])
        height=float(p[2]-(p[:2]@plane[:2]+plane[2]))
        if not np.isfinite(height) or not .035<=height<=.095:
            raise PerceptionUnknown('Retained scene is not the known tabletop base-pair configuration')
        blocks[color]={'height_p95_m':height,'source_frame':b['top_center']['frame_id'],
                       'measurement_kind':b['top_center'].get('measurement_kind','retained_measured')}
    return blocks
