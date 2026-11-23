from task_spec import COLORS
"""Synchronous RGB/depth checks for the current three-block tabletop scene.

Conditional geometry, not general obstacle recognition. Unknown facts fail closed.
No Context, device, fixture, model or background worker is constructed here.
"""
from pathlib import Path
import time
import cv2
import numpy as np
from perception import PerceptionUnknown, select_block, measure_block, HSV_RANGES, measure_center_surface, CenterDepthInconsistent, supported_depth_pixels

CALIBRATION_MARGIN_M = 0.020  # provisional head cross-validation max ~18 mm
# CALIBRATE: native 1280x720 views, installed pads, orange ~60mm cube only.
OBS_MAX_AGE_S = 10
WRIST_BODY_ROI = (420, 230, 950, 520)
WRIST_PAD_ROIS = ((140, 500, 410, 720), (950, 500, 1180, 720))
TRIAL_CORRELATION_MIN = 0.75  # diagnostic confidence only, never a motion gate
TRIAL_SHIFT_MAX_PX = 20
TRIAL_TEXTURE_STD_MIN = 8
ONSITE_FACT_MAX_AGE_S = 120  # fact lifetime; never a task/closeout budget
TOP_PATCH_SPREAD_MAX_M = .004  # offline candidate: horizontal cube top assumption
TOP_PATCH_SPAN_MIN_PX = 10    # native head pixels; not independent metric accuracy

class ViewUnavailable(PerceptionUnknown):
    """A bound view cannot be decoded; other views may still provide evidence."""


def same_state(a, b):
    """Strict finite 14-value comparison; truncated/NaN state never matches."""
    for v in (a, b):
        if (not isinstance(v, list) or len(v) != 14
            or any(type(x) not in (int, float) or not np.isfinite(x) for x in v)):
            return False
    return all(abs(x-y) <= (.0005 if i in (6, 13) else .01)
               for i, (x, y) in enumerate(zip(a, b)))


def current_image(obs, camera, *, require_current=True):
    """Native path/state binding shared by qualitative checks; never a fixture."""
    try:
        stamp = obs['captured_at_ns']
        if type(stamp) is not int or (require_current and
                not 0 <= (time.time_ns()-stamp)/1e9 <= OBS_MAX_AGE_S):
            raise PerceptionUnknown('Stale or malformed observation timestamp')
        b = obs['state_binding']
        if (b.get('owner_unchanged') is not True or b.get('rest_consistent') is not True
            or b.get('owner_generation') is None or not same_state(obs['measured14'], obs['measured14'])):
            raise PerceptionUnknown('Missing stable observation owner/state binding')
        frame = obs['frame_id']
        if not isinstance(frame, str) or len(frame) != 32 or any(c not in '0123456789abcdef' for c in frame):
            raise PerceptionUnknown('Invalid native frame id')
        if camera not in obs['cameras']:
            raise ViewUnavailable('Missing '+camera+' image')
        meta = obs['cameras'][camera]
        path = Path(meta['inspection_image'])
        if path.resolve() != (Path('runtime/frames')/frame/(camera+'.jpg')).resolve():
            raise PerceptionUnknown('Image path/frame mismatch')
        im = cv2.imread(str(path))
        if im is None:raise ViewUnavailable('Cannot decode '+camera+' image')
        if im.shape != (720, 1280, 3) or meta.get('shape_hw') != [720, 1280]:
            raise PerceptionUnknown('Unexpected native image shape')
        return im
    except (KeyError, TypeError, ValueError, OSError) as exc:
        raise PerceptionUnknown(f'Missing image/state evidence: {exc}') from exc


def depth_arrays(obs):
    """Read exactly associated saved native depth; never fill holes."""
    im = current_image(obs, 'head')
    head = obs['cameras']['head']
    if head.get('binding') != 'same immutable stereo/RGB packet' or head.get('metric_depth') is not True:
        raise PerceptionUnknown('Missing exact head RGB/depth association')
    with np.load(Path('runtime/frames')/obs['frame_id']/'head-depth.npz', allow_pickle=False) as d:
        xyz = d['xyz_rgb_m'].copy(); valid = d['valid'].copy()
    T = np.asarray(head['T_base_camera'], float)
    if xyz.shape != (720, 1280, 3) or valid.shape != (720, 1280) or T.shape != (4, 4) or not np.isfinite(T).all():
        raise PerceptionUnknown('Malformed native depth/transform')
    xyz = xyz@T[:3, :3].T + T[:3, 3]
    valid = valid.astype(bool) & np.isfinite(xyz).all(axis=2)
    return im, xyz, valid


def scene_geometry(scene, obs):
    """Retained metric data has its own source; current RGB need not have depth."""
    import json
    source=json.loads((Path('runtime/frames')/scene['frame_id']/'meta.json').read_text())
    if (source.get('calibration_sha256') != obs.get('calibration_sha256')
        or not source.get('finger_geometry',{}).get('geometry_identity')
        or source['finger_geometry']['geometry_identity'] != obs.get('finger_geometry',{}).get('geometry_identity')
        or any(source['cameras']['head'].get(k) != obs['cameras']['head'].get(k)
               for k in ('K','T_base_camera'))):
        raise PerceptionUnknown('Retained target calibration, finger geometry or head transform changed')
    return source


def scene_control(scene, robot, *, establish=False):
    """Bind to the unique bridge instance, not just a generation which restarts at 1."""
    bridge=robot.get('bridge',{})
    identity={'bridge_instance':bridge.get('instance'),
              'owner_generation':robot.get('owner_status',{}).get('generation')}
    if (not identity['bridge_instance'] or identity['owner_generation'] is None
        or robot.get('control_uncertain') or bridge.get('active_request')
        or bridge.get('phase') not in ('IDLE','DONE')):
        raise PerceptionUnknown('Missing or uncertain current owner identity')
    if establish:
        scene['control_identity']=identity
    elif scene.get('control_identity') != identity:
        raise PerceptionUnknown('Scene belongs to another owner/session; re-localize')


def installed_finger_corners(ctx, obs, scene, current_fk=None):
    """Transform the pinned rigid finger model when RGB omits projections.

    This is FK geometry, never replacement depth. Reuse is restricted to the
    same geometry identity and measured jaw opening as the source observation.
    """
    source = scene_geometry(scene, obs)
    if obs.get('finger_geometry', {}).get('fingers'):
        return np.concatenate([np.asarray(f['cad_corners_base_m'], float)
                               for f in obs['finger_geometry']['fingers']])
    if abs(obs['measured14'][6]-source['measured14'][6]) > .0005:
        raise PerceptionUnknown('Cannot reuse rigid finger envelope after jaw change')
    fingers = source.get('finger_geometry', {}).get('fingers')
    if not fingers:
        raise PerceptionUnknown('No pinned source finger geometry')
    corners = np.concatenate([np.asarray(f['cad_corners_base_m'], float) for f in fingers])
    old = np.asarray(ctx.value('piper_fk', joints=source['measured14'][:6])['T_base_flange'])
    now = np.asarray((current_fk or ctx.value('piper_fk', joints=obs['measured14'][:6]))['T_base_flange'])
    local = (corners-old[:3, 3]) @ old[:3, :3]
    return local @ now[:3, :3].T + now[:3, 3]


def read_report(path):
    """Resolve a read-only recovery to its immutable original action report."""
    import hashlib, json
    record = json.loads(Path(path).read_text())
    result = record['result']
    if result.get('stage') in ('verify_completed_action', 'verify_approach_boundary'):
        recovery = result.get('recovery', {})
        raw = Path(recovery['report']).read_bytes()
        if (result.get('ok') is not True or result.get('executed') is not False
                or hashlib.sha256(raw).hexdigest() != recovery.get('sha256')):
            raise PerceptionUnknown('Invalid or changed recovery source')
        original_record = json.loads(raw)
        original = original_record['result']
        if original.get('stage', '').startswith('verify_'):
            raise PerceptionUnknown('Recovery chains are not action history')
        # Preserve the original receipts/facts and action identity, augment only
        # the independently recovered endpoint. Never rewrite the source file.
        result = dict(original, measured=result['measured'], ok=True,
                      needs_review=False, recovered_endpoint=result['facts'])
        record = dict(original_record, result=result)
    return record


def completed_action_state(record, obs, robot=None, *, require_current=True):
    """Validate native action continuity even when its endpoint image is missing."""
    r = record['result']
    current_image(obs, 'head', require_current=require_current)
    if (not r.get('executed') or not r.get('action_completed') or not r.get('receipts')
            or r.get('hold') or r.get('hold_error') or r.get('control_uncertain')):
        raise PerceptionUnknown('No known completed action; inspect native status/receipts')
    source = r.get('initial_observation')
    if source is None:
        facts=r.get('facts') or []
        if not facts or not facts[0].get('frame_id'):
            raise PerceptionUnknown('Completed action lacks initial observation provenance')
        frame = facts[0]['frame_id']
        source = __import__('json').loads((Path('runtime/frames')/frame/'meta.json').read_text())
    owner = source['state_binding']['owner_generation']
    if (owner is None or source.get('calibration_sha256') is None
            or obs['state_binding']['owner_generation'] != owner
            or obs.get('calibration_sha256') != source['calibration_sha256']
            or obs['captured_at_ns'] < source['captured_at_ns']):
        raise PerceptionUnknown('Action owner/calibration/time changed')
    previous = source['measured14']
    for receipt in r['receipts']:
        native = receipt['result']
        if (receipt['tool'] not in ('piper_move_pose', 'piper_move_joints', 'piper_gripper', 'piper_gripper_stroke','piper_move_waypoints')
                or not all(native.get(k) is True for k in ('executed', 'measured_arrival', 'measured_settled'))
                or native.get('owner_generation') != owner
                or not same_state(native['anchor_validation']['measured14'], previous)
                or not same_state(native['final14'], native['final14'])):
            raise PerceptionUnknown('Incomplete or discontinuous native action receipts')
        previous = native['final14']
    if not same_state(previous, obs['measured14']):
        raise PerceptionUnknown('Current state differs from completed action receipt')
    if robot is not None:
        expected=r['receipts'][-1]['result'].get('owner_request_id')
        if not expected or (robot.get('last_motion') or {}).get('owner_request_id') != expected:
            raise PerceptionUnknown('An intervening action or restart invalidated this endpoint')
    last = r.get('measured') or {}
    if last.get('frame_id') and (last.get('calibration_sha256') != source['calibration_sha256']
            or last['state_binding']['owner_generation'] != owner
            or last['captured_at_ns'] > obs['captured_at_ns']
            or not same_state(last['measured14'], previous)):
        raise PerceptionUnknown('Recorded endpoint binding differs from receipt/current state')
    # A settled segment is not necessarily a completed function. Old reports
    # carry the requested waypoints in the runner inputs.
    inputs = record.get('inputs', {})
    planned = r.get('planned_motion_count')
    if planned is None and r['stage'] == 'approach':
        planned = len(inputs.get('waypoints', [])) or None
    if planned is not None and len(r['receipts']) != planned:
        raise PerceptionUnknown('Only part of the requested motion sequence completed')
    if r['stage'] in ('close_grasp', 'open_empty_gripper', 'release', 'green_stack_release'):
        target = .050 if r['stage'] == 'close_grasp' else inputs.get('gap_m', .069)
        if abs(r['receipts'][-1]['arguments']['width_m'] - target) > 1e-8:
            raise PerceptionUnknown('Only part of the requested jaw sequence completed')
    return r


def verified_previous_scene(path, obs, color='orange'):
    """Retain geometry only from a completed empty flight at this measured state."""
    import json
    record=read_report(path)
    previous=completed_action_state(record, obs)
    verified_result=previous.get('ok') is True
    if (previous.get('stage')=='approach' and previous.get('needs_review') is True
        and previous.get('review',{}).get('phase') in ('hover_alignment','grasp_alignment','observe_completed_action')):
        # Resolve ONLY missing endpoint vision from the current image. Never
        # replay the completed motion or erase its original needs_review report.
        wrist_between_pads(obs,color);verified_result=True
    if (previous.get('stage') not in ('empty_joint_waypoint','approach') or not verified_result
        or previous.get('executed') is not True or previous.get('action_completed') is not True):
        raise PerceptionUnknown('Reference is not a completed verified empty flight')
    return previous['facts'][0]['scene']


def resolve_empty_scene(previous_report, obs, *, initial_scene_report=None, empty_history=None,
                        invalidated=False, color='orange', robot=None):
    """Propagate original full XYZ through explicit or already-logged empty history."""
    scene = verified_previous_scene(previous_report, obs, color)
    if robot is not None:
        scene_control(scene,robot)
        completed_action_state(read_report(previous_report),obs,robot)
    if initial_scene_report is None and empty_history is None:
        prior = scene.get('retention', {}).get('history', [])
        history = [item['report'] for item in prior]
        if not history or history[-1] != str(previous_report):
            history.append(str(previous_report))
        initial_scene_report, empty_history = history[0], history
    if not empty_history or str(empty_history[-1]) != str(previous_report):
        raise PerceptionUnknown('Empty history must end at the supplied previous report')
    return retain_full_scene(initial_scene_report, empty_history, obs, invalidated=invalidated)


def validate_insertion_fact(fact, request_id, obs, robot, history_reports):
    """One scoped native fact at this pose, with no polling or approval files.

    Caller must supply complete same-approach history. This cannot detect omitted
    out-and-back motion, so report-boundary inspection remains mandatory.
    """
    import json
    context=fact.get('context',{})
    age=obs['captured_at_ns']/1e9-fact.get('answered_at',float('inf'))
    if (fact.get('request_id')!=request_id or fact.get('status')!='answered'
        or fact.get('done') is not True or fact.get('late') is not False
        or not str(fact.get('reply','')).startswith('用户原话：可以进入。')
        or 'remaining straight vertical insertion' not in fact.get('question','')
        or not 0<=age<=ONSITE_FACT_MAX_AGE_S
        or context.get('owner_generation')!=obs['state_binding']['owner_generation']
        or context.get('bridge_instance')!=robot['bridge']['instance']
        or not same_state(context.get('measured14'),obs['measured14'])
        or not history_reports):
        raise PerceptionUnknown('Native insertion reply/scope/time/owner/pose/history mismatch')
    for path in history_reports:
        previous=json.loads(Path(path).read_text())['result']
        if previous.get('assistance',{}).get('request_id')==request_id and previous.get('dispatch_attempted'):
            raise PerceptionUnknown('Insertion fact already consumed by an attempted dispatch')
    return {'request_id':request_id,'native_reply':fact,'age_s':age,
            'scope':'One remaining straight vertical insertion only; not grasp, lift or post-motion support',
            'consume_for_dispatch':1}


def validate_stationary_sources(scene, obs):
    """A retention marker cannot promote old XYZ to a new measurement."""
    import copy, hashlib
    for color, source in scene.get('retained_stationary', {}).items():
        raw = Path(source['source_report']).read_bytes()
        if hashlib.sha256(raw).hexdigest() != source.get('source_sha256'):
            raise PerceptionUnknown('Stationary source report changed')
        record = read_report(source['source_report'])
        old = record['result']['facts'][0]['scene']
        scene_geometry(old, obs)
        if (record['result'].get('ok') is not True or old['frame_id'] != source.get('source_frame')
                or old.get('control_identity') != scene.get('control_identity')):
            raise PerceptionUnknown('Stationary source frame/owner mismatch')
        original = copy.deepcopy(old['blocks'][color])
        if original['top_center'].get('retained_from_frame') or original['selection'].get('partial_visible'):
            raise PerceptionUnknown('Stationary source must be original full measurement')
        original['top_center']['retained_from_frame'] = original['top_center']['frame_id']
        if (scene['blocks'][color] != original
                or source.get('measurement') != original['top_center']):
            raise PerceptionUnknown('Stationary measurement changed or relabeled')


def validate_empty_motion_receipt(receipt):
    """Accept native empty moves, including a fully acknowledged waypoint batch.

    Contact/stage, owner, jaw and state-continuity checks remain with the caller.
    A batch endpoint alone is insufficient: all samples and waypoints must finish.
    """
    a=receipt['result'];tool=receipt['tool']
    if tool not in ('piper_move_joints','piper_move_pose','piper_move_waypoints') or not all(
            a.get(k) is True for k in ('executed','measured_arrival','measured_settled')):
        raise PerceptionUnknown('History includes closure/contact action or incomplete native receipt')
    if tool == 'piper_move_waypoints':
        progress=a.get('trajectory_progress') or {}
        waypoints=receipt.get('arguments',{}).get('waypoints')
        total=progress.get('total_samples')
        if (not isinstance(waypoints,list) or not waypoints
                or type(total) is not int or total <= 0
                or progress.get('phase') != 'DONE' or progress.get('fraction') != 1.0
                or progress.get('dispatched_samples') != total
                or a.get('sdk_completed_steps') != total
                or progress.get('waypoint_index') != len(waypoints)):
            raise PerceptionUnknown('Incomplete waypoint progress in retained-scene history')


def retain_full_scene(initial_report, history_reports, obs, *, invalidated=False):
    """Same-approach full XYZ retention, with explicit provenance/action history.

    No reset, closure, loaded transport, placement, magnetic displacement or
    contact may occur in this history. Current RGB resolves expected occlusion;
    no historical pixel is queried against a new depth frame.
    """
    import copy,json,hashlib
    if invalidated:raise PerceptionUnknown('Scene explicitly invalidated by reset/contact/displacement')
    if not history_reports or history_reports[0]!=initial_report:
        raise PerceptionUnknown('History must begin at the supplied measured scene report')
    first=read_report(initial_report)
    scene=copy.deepcopy(first['result']['facts'][0]['scene'])
    validate_stationary_sources(scene, obs)
    if set(scene['blocks']) != set(COLORS) or any(
            b['selection'].get('partial_visible') or (b['top_center'].get('retained_from_frame') and c not in scene.get('retained_stationary',{}))
            for c,b in scene['blocks'].items()):
        raise PerceptionUnknown('Initial full scene must precede occlusion; supply the original report/history')
    source=scene_geometry(scene,obs)
    owner=source['state_binding']['owner_generation'];cal=source['calibration_sha256']
    if owner is None or not isinstance(cal,str) or len(cal)!=64:
        raise PerceptionUnknown('Missing retained owner/calibration identity')
    if obs['state_binding'].get('owner_generation')!=owner or obs.get('calibration_sha256')!=cal:
        raise PerceptionUnknown('Retained scene owner/calibration changed')
    if obs['state_binding'].get('rest_consistent') is not True or obs['state_binding'].get('owner_unchanged') is not True:
        raise PerceptionUnknown('Current retention observation is not stable')
    im=current_image(obs,'head')
    anchor=source
    if scene.get('reuse'):
        from history_scene import validate_released_history
        anchor=first['result'].get('initial_observation')
        if anchor is None:raise PerceptionUnknown('New approach needs its actual starting observation')
        validate_released_history(scene,obs,anchor)
    previous=anchor['measured14'];history=[];last_stamp=anchor['captured_at_ns'];seen=set()
    for path in history_reports:
        if str(Path(path).resolve()) in seen:raise PerceptionUnknown('Duplicate retained-history report')
        seen.add(str(Path(path).resolve()))
        raw=Path(path).read_bytes();record=read_report(path);r=record['result']
        if r.get('stage') not in ('empty_joint_waypoint','approach') or not r.get('action_completed') or not r.get('executed') or not r.get('receipts'):
            raise PerceptionUnknown('History contains a nonempty, incomplete or unexecuted operation')
        if not r.get('ok') and not (r.get('needs_review') and r.get('review',{}).get('phase') in ('hover_alignment','grasp_alignment','observe_completed_action')):
            raise PerceptionUnknown('Unresolved history failure')
        # An intended engagement label is not physical contact. A recorded
        # high approach (>190mm flange above block top, 170mm fingers) is empty.
        heights=[f['flange_above_block_top_m'] for f in r.get('facts',[]) if 'flange_above_block_top_m' in f]
        engaged_at_contact=(r.get('engage') is True and (not heights or min(heights)<.190))
        if (r.get('hold') or r.get('control_uncertain') or engaged_at_contact
            or r.get('assistance') or any(f.get('flange_above_block_top_m',1)<.14 for f in r.get('facts',[]))):
            raise PerceptionUnknown('Contact/engagement or unresolved control invalidates empty retention')
        for receipt in r['receipts']:
            a=receipt['result']
            validate_empty_motion_receipt(receipt)
            anchor=a['anchor_validation']['measured14']
            if a.get('owner_generation') != owner or not same_state(anchor,previous):
                raise PerceptionUnknown('Gap or intervening motion in retained-scene history')
            final=a['final14']
            if (not same_state(final,final) or abs(final[6]-source['measured14'][6])>.0005 or final[6]<.065
                or not same_state([0.0]*7+final[7:],[0.0]*7+source['measured14'][7:])):
                raise PerceptionUnknown('Jaw closure invalidates empty-approach retention')
            previous=final
        last=r['measured']
        if not last.get('frame_id'):
            if str(path) != str(history_reports[-1]):
                raise PerceptionUnknown('Recover missing intermediate endpoint before extending history')
            completed_action_state(record, obs)
            last=obs
        if (last['state_binding']['owner_generation']!=owner or last.get('calibration_sha256')!=cal
            or last['captured_at_ns']<last_stamp or not same_state(last['measured14'],previous)):
            raise PerceptionUnknown('History owner/calibration/state/timestamp changed')
        last_stamp=last['captured_at_ns']
        history.append({'report':path,'sha256':hashlib.sha256(raw).hexdigest(),'stage':r['stage'],'executed_receipts':len(r['receipts'])})
    if obs['captured_at_ns']<last_stamp or not same_state(obs['measured14'],previous):
        raise PerceptionUnknown('Current state differs from last completed history receipt')
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);visible={}
    for color,block in scene['blocks'].items():
        x,y,w,h=block['selection']['bbox_xywh']
        if block['top_center'].get('measurement_kind')=='action_derived':
            head=obs['cameras']['head'];cam=np.linalg.inv(np.asarray(head['T_base_camera']))@np.r_[block['top_center']['surface_position_m'],1.]
            uv=np.asarray(head['K'])@cam[:3];u,v=uv[:2]/uv[2]
            x,y=int(round(u-w/2)),int(round(v-h/2))
        roi=np.zeros((720,1280),np.uint8);roi[max(0,y-8):min(720,y+h+8),max(0,x-8):min(1280,x+w+8)]=1
        mask=np.zeros_like(roi)
        for lo,hi in HSV_RANGES[color]:mask |= cv2.inRange(hsv,lo,hi)
        count=int(((mask>0)&(roi>0)).sum());dark=float((hsv[y:y+h,x:x+w,2]<100).mean())
        wrist_pixels=None
        if count<200 and dark<.45:
            # With no hand-eye, wrist color elsewhere cannot establish that a
            # stationary neighbor is still at its base location. Only the active
            # target can use its recognizable object/pad relation here.
            active_color=record.get('inputs',{}).get('color','orange')
            if color!=active_color:
                raise PerceptionUnknown(f'{color} disappeared from its clear head region; inspect scene change')
            try:wrist_fact=wrist_between_pads(obs,color)
            except PerceptionUnknown as exc:
                raise PerceptionUnknown(f'{color} missing from clear head region and wrist grasp relation') from exc
            wrist_pixels=wrist_fact['body_pixels']
        # Current visible compact blocks must overlap the original footprint.
        # RGB only: do not require new partial depth under expected arm occlusion.
        try:
            from perception import component
            candidate,_=component(im,color)
            xx,yy,ww,hh=candidate['bbox_xywh']
            # Partial visible silhouettes change their centroid under occlusion.
            # Reject a plainly separate object, not a few pixels of centroid drift.
            overlap=max(0,min(x+w,xx+ww)-max(x,xx))*max(0,min(y+h,yy+hh)-max(y,yy))
            if overlap/max(1,min(w*h,ww*hh))<.25:
                raise PerceptionUnknown(f'Visible {color} displacement invalidates retained XYZ')
        except PerceptionUnknown as e:
            if str(e).startswith('Visible '):raise
            if not str(e).startswith(('Expected one ','Insufficient interior')):raise
        visible[color]={'current_visible_color_pixels':count,'dark_occlusion_fraction':dark,'wrist_color_pixels':wrist_pixels,'xyz_retained_unchanged':True}
    scene['retention']={'source_frame_id':source['frame_id'],'source_captured_at_ns':source['captured_at_ns'],
       'calibration_sha256':cal,'owner_generation':owner,'current_frame_id':obs['frame_id'],
       'history':history,'visible_checks':visible,'invalidate_on':['reset','unrecorded contact','observed displacement','visible contradiction','owner/calibration change'],
       'meaning':'Original full XYZ, not remeasured or relabeled as current; fresh wrist/geometry gates still required',
       'limits':'History supplied by caller; cannot prove absence of hidden transient contact, omitted return-to-same-pose actions, or motion under complete occlusion'}
    return scene


def wrist_between_pads(obs, color, *, require_current=True):
    """Coarse visible alignment, allowing clipped/merged same-color silhouettes.

    R4 native wrist setup only. Regions locate body and pad portions, not exact
    edges or hidden metric gaps. Other geometry checks remain independent.
    """
    if color not in COLORS:raise PerceptionUnknown('Wrist block check supports configured task colors')
    im=current_image(obs,'left',require_current=require_current)
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    pad_mask=cv2.inRange(hsv,(8,100,70),(30,255,255))>0
    mask=pad_mask if color=='orange' else cv2.inRange(hsv,HSV_RANGES[color][0][0],HSV_RANGES[color][0][1])>0
    x0,y0,x1,y1=WRIST_BODY_ROI
    body=mask[y0:y1,x0:x1];ys,xs=np.nonzero(body)
    pads=[int(pad_mask[y:y2,x:x2].sum()) for x,y,x2,y2 in WRIST_PAD_ROIS]
    if len(xs)<.025*body.size or max(pads)<.005*(270*220):
        raise PerceptionUnknown('Current wrist lacks a recognizable body and pad reference')
    center=float(np.median(xs)+x0)
    if not x0+.05*(x1-x0)<=center<=x1-.05*(x1-x0) or np.ptp(xs)<.10*(x1-x0):
        raise PerceptionUnknown('Obvious wrist block displacement or insufficient visible body')
    columns=np.flatnonzero(body.sum(axis=0)>0)
    top=np.array([np.flatnonzero(body[:,x])[0] for x in columns])
    slope=float(np.polyfit(columns,top,1)[0]) if len(columns)>1 else None
    # Occlusion changes the visible boundary slope; it is not an uprightness test.
    return {'frame_id':obs['frame_id'],'body_pixels':int(len(xs)),
            'pad_region_pixels':pads,'body_center_x_px':center,'body_center_y_px':float(np.median(ys)+y0),'visible_top_slope':slope,
            'scope':'Coarse visible alignment; one pad may be occluded, clipped/merged silhouettes allowed; no minimum gap, wrist-to-base, support or retention claim'}


def approach_scene(ctx, obs, scene, color, target, *, engage=False, max_down_m=.021):
    try:
        scene_geometry(scene,obs)
        from qualitative import alignment_evidence
        wrist=alignment_evidence(ctx,obs,color)
        fk=ctx.value('piper_fk',joints=obs['measured14'][:6]);T=np.asarray(fk['T_base_flange'])
        p=np.asarray(fk['position']);target=np.asarray(target);block=np.asarray(scene['blocks'][color]['top_center']['surface_position_m'])
        if T[2,2]>-.995 or max(abs(target[:2]-p[:2]))>.001 or not -max_down_m<=target[2]-p[2]<=.001:
            raise PerceptionUnknown(f'Approach must be near-down, vertical and <= {max_down_m*1000:.0f}mm per segment')
        if np.linalg.norm(p[:2]-block[:2])>.012:
            raise PerceptionUnknown('Flange no longer aligned to measured block center')
        for c,b in scene['blocks'].items():
            if c!=color and np.linalg.norm(np.asarray(b['top_center']['surface_position_m'])[:2]-target[:2])<.10:
                raise PerceptionUnknown('Neighbor too close to this vertical approach corridor')
        plane=np.asarray(scene['table']['z_ax_by_c']);table_z=float(target[:2]@plane[:2]+plane[2])
        corners=installed_finger_corners(ctx,obs,scene,fk)
        tip_min=float(corners[:,2].min()+target[2]-p[2])
        if tip_min-table_z<.012:
            raise PerceptionUnknown('Nominal installed finger/table clearance below12mm; no descent')
        if target[2]<block[2]+.125:
            raise PerceptionUnknown('Below conditional block-relative contact depth')
        return {'ok':True,'frame_id':obs['frame_id'], 'scene':scene,'wrist':wrist,
                'why':f'Visible {color} inside the open pads; vertical bounded descent at retained measured center; neighbor separation and nominal table clearance checked',
                'nominal_tip_table_clearance_m':tip_min-table_z,
                'flange_above_block_top_m':float(target[2]-block[2]),
                'calibration_provisional':True,'scope':'Conditional supervised block approach; nominal table gap is not independent metrology'}
    except (PerceptionUnknown,KeyError,ValueError,OSError) as e:
        return {'ok':None,'frame_id':obs.get('frame_id'),'why':str(e)}


def supported_entry(ctx, obs, scene, color='orange'):
    """Coarse entry at retained XYZ; fresh head depth is optional corroboration."""
    try:
        if color not in COLORS:raise PerceptionUnknown('Entry supports configured task colors')
        scene_geometry(scene,obs)
        current_image(obs,'head')
        if not .065<=obs['measured14'][6]<=.074:raise PerceptionUnknown('Expected open jaw before first closure')
        ref=scene['blocks'][color];top=ref['top_center']['surface_position_m']
        from qualitative import alignment_evidence
        wrist=alignment_evidence(ctx,obs,color)
        fk=ctx.value('piper_fk',joints=obs['measured14'][:6]);p=np.asarray(fk['position']);R=np.asarray(fk['T_base_flange'])[:3,:3]
        if R[2,2]>-.995 or not .120<=p[2]-top[2]<=.135 or np.linalg.norm(p[:2]-np.asarray(top[:2]))>.012:
            raise PerceptionUnknown('Not at conditional top-relative insertion depth/alignment')
        return {'ok':True,'frame_id':obs['frame_id'],'scene':scene,'wrist':wrist,
                'measurement_frame':scene['frame_id'],'retained_top_m':list(top),
                'why':'Completed insertion at retained XYZ, coarse visible body/pad relation and measured flange geometry agree',
                'scope':'Conditional preclosure alignment; support inferred from same-approach history, not newly measured support or force'}
    except (PerceptionUnknown,KeyError,ValueError,OSError) as e:
        return {'ok':None,'frame_id':obs.get('frame_id'),'why':str(e)}


def trial_surface(ctx, obs, scene, baseline_z, color='orange'):
    """Measure observed top, without selecting for the commanded lift height.

    Baseline defines a broad +/-25mm association band; it is identical before
    and after. Stationary objects remain measurable and must fail the rise check.
    """
    im,xyz,valid=depth_arrays(obs)
    ref=scene['blocks'][color];x,y,w,h=ref['selection']['bbox_xywh']
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,HSV_RANGES[color][0][0],HSV_RANGES[color][0][1])>0;roi=np.zeros((720,1280),bool);roi[y-20:y+h+20,x-20:x+w+20]=True
    distance=cv2.distanceTransform((mask&roi).astype(np.uint8),cv2.DIST_L2,5)
    candidates=valid & (distance>=3) & (np.abs(xyz[:,:,2]-baseline_z)<.025)
    if candidates.sum()<50:raise PerceptionUnknown('Insufficient independent current top depth')
    observed_top=float(np.percentile(xyz[:,:,2][candidates],80))
    support=cv2.boxFilter((valid&(np.abs(xyz[:,:,2]-observed_top)<.004)).astype(np.float32),-1,(5,5),normalize=False)
    total=cv2.boxFilter(valid.astype(np.float32),-1,(5,5),normalize=False)
    work=np.where((support>=9)&(support>=.9*total),distance,0).astype(np.float32);pixels=[]
    for _ in range(5):
        _,v,_,p=cv2.minMaxLoc(work)
        if v<3:break
        pixels.append(list(p));cv2.circle(work,p,4,0,-1)
    if len(pixels)<3:raise PerceptionUnknown('Trial top depth unavailable; inspect without repeating lift')
    d=ctx.value('piper_pixel_to_position',camera='head',frame_id=obs['frame_id'],pixels=pixels,radius=2)
    return measure_block(obs,ref['selection']|{'frame_id':obs['frame_id'],'pixels':pixels},d)


def wrist_trial_match(before, after):
    if (before['frame_id']==after['frame_id'] or before['captured_at_ns']>=after['captured_at_ns']
        or before.get('calibration_sha256')!=after.get('calibration_sha256')
        or before['state_binding']['owner_generation']!=after['state_binding']['owner_generation']):
        raise PerceptionUnknown('Trial comparison requires distinct ordered frames with unchanged owner/calibration')
    a=cv2.cvtColor(current_image(before,'left',require_current=False),cv2.COLOR_BGR2GRAY)
    b=cv2.cvtColor(current_image(after,'left'),cv2.COLOR_BGR2GRAY)
    # Current top decoration/colored top, clear of the moving background/pads.
    template=a[340:490,470:800];search=b[315:515,445:825]
    texture=float(template.std())
    scores=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED);_,score,_,loc=cv2.minMaxLoc(scores)
    shift=[loc[0]-25,loc[1]-25]
    reliable=bool(texture>=TRIAL_TEXTURE_STD_MIN and np.isfinite(score) and score>=TRIAL_CORRELATION_MIN and max(map(abs,shift))<=TRIAL_SHIFT_MAX_PX)
    return {'correlation':float(score) if np.isfinite(score) else None,'shift_px':shift,
            'before_frame':before['frame_id'],'after_frame':after['frame_id'],
            'texture_std':texture,'reliable_match':reliable,'motion_authority':False,
            'scope':'Diagnostic patch similarity only; background/occlusion can change it. Use task evidence, not this score, to judge retention.'}


def current_arrays(obs, reference_scene=None, colors=None):
    rgb,xyz,valid=depth_arrays(obs)
    plane=np.asarray(tabletop(obs,xyz,valid,rgb)['z_ax_by_c'])
    def at_block_height(box):
        x,y,w,h=box;p=xyz[y:y+h,x:x+w][valid[y:y+h,x:x+w]]
        if len(p)<50:return False
        heights=p[:,2]-(p[:,:2]@plane[:2]+plane[2])
        return .035<=np.median(heights)<=.090
    # select_block validates exact RGB/depth association and native paths.
    selections={}
    for color in (colors or COLORS):
        try:selections[color]=select_block(obs,color,candidate_filter=at_block_height,valid_depth_mask=valid)
        except PerceptionUnknown as e:
            # Only the semantic shape failure can use a partial visible surface.
            # Timestamp, path, calibration and binding failures never fall back.
            if reference_scene is None or not str(e).startswith(('Expected one ','Insufficient interior')):raise
            ref=reference_scene['blocks'][color]['selection']
            im=cv2.imread(obs['cameras']['head']['inspection_image']);hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
            mask=np.zeros((720,1280),np.uint8)
            for lo,hi in HSV_RANGES[color]:mask |= cv2.inRange(hsv,lo,hi)
            x,y,w,h=ref['bbox_xywh'];roi=np.zeros_like(mask);roi[max(0,y-8):y+h+8,max(0,x-8):x+w+8]=1
            inside=(mask>0)&(roi>0)
            outside=((mask>0)&(roi==0)).astype(np.uint8)
            _,_,stats,_=cv2.connectedComponentsWithStats(outside)
            compact_elsewhere=[s.tolist() for s in stats[1:] if s[4]>=500 and min(s[2:4])>=25 and .55<=s[2]/s[3]<=1.8 and at_block_height(s[:4])]
            # Orange finger pads can contribute thin red HSV strips; these are
            # not compact red blocks. A competing compact region still refuses.
            if inside.sum()<500 or compact_elsewhere:
                raise PerceptionUnknown(f'Occluded {color} lacks unique visible color in its retained rectangle')
            distance=cv2.distanceTransform(inside.astype(np.uint8),cv2.DIST_L2,5)
            expected_z=reference_scene['blocks'][color]['top_center']['surface_position_m'][2]
            top=valid & (np.abs(xyz[:,:,2]-expected_z)<.006)
            support=cv2.boxFilter(top.astype(np.float32),-1,(7,7),normalize=False)
            total=cv2.boxFilter(valid.astype(np.float32),-1,(7,7),normalize=False)
            # Exclude the visible side face: retain only windows whose actual
            # direct samples agree with the previous top height. No hole fill.
            allowed=(support>=9)&(support>=.85*total)
            pixels=[];work=np.where(allowed,distance,0).astype(np.float32)
            for _ in range(5):
                _,val,_,point=cv2.minMaxLoc(work)
                if val<7:break
                pixels.append(list(point));cv2.circle(work,point,5,0,-1)
            if len(pixels)<3:raise PerceptionUnknown('Insufficient current partial surface interior')
            selections[color]=ref|{'frame_id':obs['frame_id'],'image':obs['cameras']['head']['inspection_image'],
                 'pixels':pixels,'partial_visible':True,'reference_frame_id':ref['frame_id'],
                 'visible_color_pixels':int(inside.sum())}
    return selections,xyz,valid,rgb


def tabletop(obs, xyz, valid, rgb):
    hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
    mask=valid & (hsv[:,:,1]<65) & (hsv[:,:,2]>65) & (hsv[:,:,2]<235)
    roi=np.zeros_like(mask);roi[140:580,180:1100]=True;mask &= roi
    points=xyz[mask]
    if len(points)<5000: raise PerceptionUnknown('Insufficient visible gray table depth')
    z0=np.percentile(points[:,2],35)
    points=points[np.abs(points[:,2]-z0)<.012]
    for _ in range(3):
        A=np.column_stack((points[:,:2],np.ones(len(points))))
        plane=np.linalg.lstsq(A,points[:,2],rcond=None)[0]
        errors=np.abs(A@plane-points[:,2]);points=points[errors<.006]
    if len(points)<5000 or np.ptp(points[:,0])<.15 or np.ptp(points[:,1])<.20 or np.linalg.norm(plane[:2])>.15:
        raise PerceptionUnknown('Table plane coverage/tilt outside candidate range')
    return {'z_ax_by_c':plane.tolist(),'points':len(points),
            'residual_p95_m':float(np.percentile(errors,95)),
            'calibration_margin_m':CALIBRATION_MARGIN_M}


def _recover_center_height(ctx, obs, sel, patch, depth, xyz, valid, rgb, reason):
    """Use a measured flat color patch; reselect in this same frame only if needed."""
    def flat(value, pixels):
        p = np.asarray(pixels)
        span = float(np.max(np.linalg.norm(p[:, None]-p[None, :], axis=2)))
        return value['surface_z_spread_m'] <= TOP_PATCH_SPREAD_MAX_M and span >= TOP_PATCH_SPAN_MIN_PX
    pixels = sel['pixels']; selected_patch = patch; used_depth = depth; reselected = False
    if not flat(patch, pixels):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV); mask = np.zeros(valid.shape, np.uint8)
        for lo, hi in HSV_RANGES[sel['color']]:
            mask |= cv2.inRange(hsv, lo, hi)
        x,y,w,h = sel['bbox_xywh']; roi = np.zeros_like(mask); roi[y:y+h,x:x+w] = mask[y:y+h,x:x+w]
        distance = cv2.distanceTransform((roi>0).astype(np.uint8), cv2.DIST_L2, 5)
        top_valid = valid & (np.abs(xyz[:,:,2]-patch['surface_position_m'][2]) <= .006)
        pixels, _ = supported_depth_pixels(distance, top_valid, [])
        used_depth = ctx.value('piper_pixel_to_position', camera='head', frame_id=obs['frame_id'], pixels=pixels, radius=3)
        selected_patch = measure_block(obs, sel | {'pixels':pixels}, used_depth)
        if not flat(selected_patch, pixels):
            raise PerceptionUnknown('Center depth inconsistent: insufficient flat, spatially spread real top-depth patch')
        reselected = True
    result = dict(selected_patch); result['surface_position_m'] = list(selected_patch['surface_position_m'])
    result['height_source'] = 'measured colored top patch; center depth unavailable or inconsistent'
    result['depth_consensus'] = {'method':'same-frame measured colored top', 'reason':reason,
        'reselected':reselected, 'pixels':pixels, 'synthesized_depth':False}
    return result, used_depth, pixels


def measure_scene(ctx, obs, reference_scene=None, colors=None):
    selections,xyz,valid,rgb=current_arrays(obs,reference_scene,colors)
    blocks={}
    for color,sel in selections.items():
        depth=ctx.value('piper_pixel_to_position',camera='head',frame_id=obs['frame_id'],pixels=sel['pixels'],radius=3)
        blocks[color]={'selection':sel,'depth':depth,'localization':measure_block(obs,sel,depth)}
        if sel.get('partial_visible'):
            old=reference_scene['blocks'][color]['top_center']
            center=old.copy();center['surface_position_m']=list(old['surface_position_m'])
            current_z=blocks[color]['localization']['surface_position_m'][2]
            if abs(current_z-center['surface_position_m'][2])>.008:
                raise PerceptionUnknown('Partially visible block top height changed')
            center['surface_position_m'][2]=current_z
            center['meaning']='Retained XY center; current partial surface z; no contact during preceding verified empty flight'
            center['retained_from_frame']=old['frame_id'];center['current_surface_frame']=obs['frame_id']
            blocks[color]['top_center']=center
            continue
        x,y,w,h=sel['bbox_xywh'];u,v=round(x+w/2),round(y+h/2)
        # White circular decoration is inside the colored rectangle, not a hole
        # in the object. Query its REAL direct depth; never fill metric depth.
        offsets=sorted([(dx,dy) for dx in (-12,-6,0,6,12) for dy in (-12,-6,0,6,12)],key=lambda p:p[0]*p[0]+p[1]*p[1])
        pixels=[[u+dx,v+dy] for dx,dy in offsets if valid[v+dy-3:v+dy+4,u+dx-3:u+dx+4].sum()>=9][:5]
        patch = blocks[color]['localization']
        if len(pixels)<3:
            center, center_depth, pixels = _recover_center_height(
                ctx, obs, sel, patch, depth, xyz, valid, rgb, 'Insufficient center depth windows')
        else:
            center_sel=sel|{'pixels':pixels}
            center_depth=ctx.value('piper_pixel_to_position',camera='head',frame_id=obs['frame_id'],pixels=center_sel['pixels'],radius=3)
            try:
                center=measure_center_surface(obs,center_sel,center_depth,patch)
                center['height_source']='measured windows near RGB rectangle center'
                if center.get('depth_consensus'):
                    center['height_source']='measured colored top plus consistent center majority'
            except CenterDepthInconsistent as exc:
                center, center_depth, pixels = _recover_center_height(
                    ctx, obs, sel, patch, depth, xyz, valid, rgb, str(exc))
        if abs(center['surface_position_m'][2]-blocks[color]['localization']['surface_position_m'][2])>.008:
            raise PerceptionUnknown('Rectangle center depth disagrees with colored top interior')
        # Intersection of the RGB rectangle-center ray with the measured top
        # height. This is a geometric estimate, not synthesized direct depth.
        head=obs['cameras']['head'];T=np.asarray(head['T_base_camera']);K=np.asarray(head['K'])
        ray=T[:3,:3]@np.linalg.solve(K,np.array([u,v,1.]))
        z=center['surface_position_m'][2]
        center['surface_position_m']=(T[:3,3]+ray*((z-T[2,3])/ray[2])).tolist()
        center['meaning']='RGB rectangle center ray intersected with native measured top height; provisional geometric estimate'
        center['center_depth_measured']=False
        center['assumption']='Upright cube with approximately horizontal top; RGB rectangle center approximates top center'
        center['center_pixel']=[u,v]
        center['max_depth_sample_offset_px']=max(float(np.linalg.norm(np.array(px)-[u,v])) for px in pixels)
        blocks[color]['top_center']=center
        blocks[color]['center_depth']=center_depth
    return {'frame_id':obs['frame_id'],'blocks':blocks,'table':tabletop(obs,xyz,valid,rgb)}


def empty_opening(ctx, obs, gap_m):
    """Restricted first opening at the empty, near-zero-joint startup pose."""
    try:
        q=np.asarray(obs['measured14'],float)
        if max(abs(q[:6]))>.01 or not 0<=q[6]<=.005:
            raise PerceptionUnknown('First-opening check requires near-zero joints and <=5mm empty jaw')
        if not q[6]<gap_m<=.073:
            raise PerceptionUnknown('Opening target outside conditional 0..73mm range')
        scene=measure_scene(ctx,obs)
        from task_geometry import opening_sweep
        sweep=opening_sweep(obs,scene,gap_m)
        # Pixel counts are diagnostics, not a second geometric collision gate.
        wrist=cv2.imread(obs['cameras']['left']['inspection_image'])
        if wrist is None or wrist.shape!=(720,1280,3):
            raise PerceptionUnknown('Missing native left wrist view')
        hsv=cv2.cvtColor(wrist,cv2.COLOR_BGR2HSV)
        pads=cv2.inRange(hsv,(8,120,80),(30,255,255))>0
        left=int(pads[490:720,400:640].sum());right=int(pads[490:720,640:880].sum())
        target=q.tolist();target[6]=gap_m
        check=ctx.value('piper_check_path',start14=q.tolist(),target14=target)
        if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:
            raise PerceptionUnknown('Opening CAD/workspace preflight failed')
        return {'ok':True,'frame_id':obs['frame_id'],
                'why':'Closed empty Prepare jaw; continuous installed-finger opening envelope clears modeled blocks/table; native CAD/workspace clear',
                'scene':scene,'opening_sweep':sweep,'wrist_pad_pixels':[left,right],
                'pad_pixels_diagnostic_only':True,
                'preflight':check,'scope':'Startup four-block tabletop only; not a general empty/obstacle detector'}
    except (PerceptionUnknown,KeyError,ValueError,OSError) as e:
        return {'ok':None,'frame_id':obs.get('frame_id'),'why':str(e)}


def empty_joint_path(ctx, obs, joints, retained_scene=None):
    """Current three-block/table geometry plus sampled installed finger envelope.

    Retained XYZ must come from resolve_empty_scene at this frame. Otherwise
    acquire a full scene. Fingers stay separated in 3D; no entry/contact.
    """
    try:
        if retained_scene is None:
            scene=measure_scene(ctx,obs)
        else:
            current_image(obs,'left')  # fresh complementary wrist, no depth re-localization
            if retained_scene.get('retention',{}).get('current_frame_id')!=obs['frame_id']:
                raise PerceptionUnknown('Retained scene not checked at this current observation')
            scene=retained_scene
        start=np.asarray(obs['measured14'][:6]);end=np.asarray(joints,float)
        if end.shape!=(6,) or not np.isfinite(end).all() or max(abs(end-start))>np.pi/3:
            raise PerceptionUnknown('Joint segment exceeds native60deg conditional range')
        if not .065<=obs['measured14'][6]<=.074:
            raise PerceptionUnknown('Empty transition requires the observed open block jaw')
        plane=np.asarray(scene['table']['z_ax_by_c']);top=[]
        for v in scene['blocks'].values():
            p=np.array(v['top_center']['surface_position_m']);height=p[2]-(p[:2]@plane[:2]+plane[2])
            if not .045<=height<=.085:
                raise PerceptionUnknown('Not all three blocks are visibly at table level')
            top.append(p[2])
        start_fk=ctx.value('piper_fk',joints=start.tolist())
        T0=np.asarray(start_fk['T_base_flange'])
        corners=installed_finger_corners(ctx,obs,scene,start_fk)
        local=(corners-T0[:3,3])@T0[:3,:3]
        from task_geometry import envelope_clearance
        rows=[];min_table=1.;min_top=1.
        for fraction in np.linspace(0,1,41):
            q=(start+(end-start)*fraction).tolist();fk=ctx.value('piper_fk',joints=q)
            T=np.asarray(fk['T_base_flange']);world=local@T[:3,:3].T+T[:3,3]
            clearance=envelope_clearance(list(world.reshape(-1,8,3)),scene)
            table_clear=clearance['min_table_clearance_lower_m'];block_clear=clearance['min_block_separation_lower_m']
            min_table=min(min_table,table_clear);min_top=min(min_top,block_clear)
            rows.append({'fraction':float(fraction),'joints':q,'flange':fk['position'],'finger_min_z_m':float(world[:,2].min())})
        return {'ok':True,'frame_id':obs['frame_id'],
                'why':'Measured or validated retained table-level geometry; sampled installed finger sweep stays separated from blocks/table in 3D; empty flight only',
                'scene':scene,'samples':rows,'min_table_clearance_lower_m':min_table,
                'min_block_clearance_lower_m':min_top,'scope':'Known three-block table; full CAD/workspace checked separately'}
    except (PerceptionUnknown,KeyError,ValueError,OSError) as e:
        return {'ok':None,'frame_id':obs.get('frame_id'),'why':str(e)}
