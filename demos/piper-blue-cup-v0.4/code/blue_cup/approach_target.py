"""A saved base-frame target for ONE empty approach, not a scene/motion permit."""
import copy,json,time,uuid
from pathlib import Path
import numpy as np
from perception import OUT,observation_recent,observation_pose_matches

PATH=OUT/'approach-target.json'
GEOMETRY_KEYS=('near_wall','cup_center_estimate','coaster_center','table')


def invalidate(reason):
    if PATH.exists():
        r=json.loads(PATH.read_text());r.update(valid=False,invalid_reason=reason);save(r)


def save(r):
    tmp=PATH.with_suffix('.partial');tmp.write_text(json.dumps(r,indent=2,allow_nan=False));tmp.replace(PATH)



def retain_no_dispatch(target, report, resume_phase):
    """Preserve only measurement provenance for one explicit, fresh-checked retry.

    No visual approval is retained. The next ordinary approach call re-observes,
    validates owner/pose and checks the actual proposed path/scene before motion.
    """
    if (not target or not target.get('valid') or target.get('contact_started')
            or resume_phase not in ('preparation_verified','hover_verified')
            or target.get('phase') != ('hover' if resume_phase=='preparation_verified' else 'entry')
            or target.get('pre_dispatch_recoveries',0)!=0
            or report.get('target_id')!=target.get('id')
            or report.get('executed') is not False or report.get('motion_arrived') is not False
            or len(report.get('steps',[]))!=1):return False
    step=report['steps'][0];error=step.get('tool_error',{})
    if (step.get('executed') is not False or step.get('motion_arrived') is not False
            or 'dispatch_monotonic' in step or 'receipt' in step
            or error.get('operation')!='gemini_generate' or error.get('stage')!='provider_request'
            or error.get('type') not in ('RemoteDisconnected','TimeoutError','ConnectionResetError')
            or error.get('no_action_executed') is not True
            or error.get('control_uncertain') is not False):return False
    try:
        if json.loads(PATH.read_text())!=target:return False
    except (OSError,ValueError):return False
    restored={**target,'phase':resume_phase,'pre_dispatch_recoveries':1}
    save(restored)
    return True


def create(localization,robot,phase='hover'):
    obs=localization['observation']
    if not localization.get('ok') or not observation_recent(obs,require_depth=True):
        raise ValueError('Target requires a fresh successful metric localization')
    cam=obs['cameras']['head']
    geometry={k:localization[k] for k in GEOMETRY_KEYS}
    if np.asarray(list(geometry.values())).shape!=(4,3) or not np.isfinite(list(geometry.values())).all():
        raise ValueError('Invalid target geometry')
    r={'id':uuid.uuid4().hex,'valid':True,'created_ns':time.time_ns(),'phase':phase,
       'frame':'left_base_link','geometry':geometry,'source_frame':obs['frame_id'],
       'source_observation':obs,'source_calibration':obs.get('calibration_sha256'),
       'head_K':cam['K'],'head_T_base_camera':cam['T_base_camera'],
       'owner_generation':robot['owner_status']['generation'],'bridge_instance':robot['bridge']['instance'],
       'geometry_identity':obs['finger_geometry']['geometry_identity'],
       'last_motion':robot.get('last_motion'),'expected14':obs['measured14'],'last_frame':obs['frame_id'],'contact_started':False}
    if 'contours' in localization:
        contours={k:localization['contours'][k] for k in ('cup','coaster')}
        values=np.asarray(list(contours.values()),float)
        if values.shape!=(2,3) or not np.isfinite(values).all() or np.any(values[:,2]<=0):
            raise ValueError('Invalid source identity contours')
        # Old pixels identify the object only. Transport must associate a CURRENT
        # contour and sample CURRENT depth, using this original measurement frame.
        r['source_contours']=copy.deepcopy(contours)
    if localization.get('contours',{}).get('cup'):
        T=np.asarray(cam['T_base_camera']);p=np.asarray(geometry['cup_center_estimate'])
        optical=(p-T[:3,3])@T[:3,:3]
        r['cup_radius_m']=float(localization['contours']['cup'][2]*optical[2]/min(cam['K'][0][0],cam['K'][1][1]))
    validate(r,obs,robot);save(r);return r


def validate(r,obs,robot):
    if (not r.get('valid') or r.get('contact_started') or r.get('frame')!='left_base_link'
            or r.get('phase') not in ('preparation','preparation_verified','hover','hover_verified','entry','entry_verified')):
        raise ValueError('Approach target invalid/consumed; acquire a new target')
    age=(time.time_ns()-r['created_ns'])/1e9
    if age<0:raise ValueError('Approach target measurement timestamp is in the future')
    # Target age is provenance, not an RGB/depth freshness timestamp. Current
    # scene checks plus owner/action scope govern reuse; native session budgets remain.
    if (not robot.get('motion_ready') or robot.get('control_uncertain')
            or robot['bridge']['phase'] not in ('IDLE','DONE')
            or robot['bridge']['instance']!=r['bridge_instance']
            or robot['owner_status']['generation']!=r['owner_generation']):
        raise ValueError('Approach target owner/control changed')
    if (not observation_recent(obs) or not obs['state_binding'].get('rest_consistent')
            or obs['state_binding'].get('owner_generation')!=r['owner_generation']
            or not observation_pose_matches(r['expected14'],obs['measured14'])
            or not observation_pose_matches(obs['measured14'],robot['measured_state']['values'])):
        raise ValueError('Unexpected motion or stale/unbound target observation')
    if (obs.get('calibration_sha256')!=r['source_calibration']
            or obs['finger_geometry']['geometry_identity']!=r['geometry_identity']):
        raise ValueError('Approach target calibration/geometry changed')
    if robot.get('last_motion')!=r.get('last_motion'):
        raise ValueError('An intervening action invalidated the approach target')
    cam=obs['cameras']['head']
    # RGB observations now carry the fixed-head calibration too. No depth required.
    if cam.get('K')!=r['head_K'] or cam.get('T_base_camera')!=r['head_T_base_camera']:
        raise ValueError('Head geometry changed since target measurement')


def observe(ctx,r,previous=None):
    """Use a fresh action receipt once; otherwise capture RGBD in one request."""
    status=ctx.value('status')
    if status.get('control_uncertain'):raise ValueError('Uncertain control invalidates retained target')
    if previous is not None and observation_recent(previous,require_depth=True):
        validate(r,previous,ctx.value('piper_status'))
        return previous
    obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=True)
    robot=ctx.value('piper_status');validate(r,obs,robot)
    return obs


def localization(r,obs):
    """Keep measurement provenance separate from the fresh execution observation."""
    result=dict(ok=True,**r['geometry'],observation=obs,before_frame=obs['frame_id'],
                after_frame=obs['frame_id'],target_id=r['id'],measurement_frame=r['source_frame'],
                localization_source='retained_base_frame_target',visual_calls=0,perception_s=0,
                target_age_s=(time.time_ns()-r['created_ns'])/1e9,
                clearance_certified=False)
    if 'source_contours' in r:result['contours']=copy.deepcopy(r['source_contours'])
    return result


def advance(ctx,r,step):
    receipt=step.get('receipt',{})
    if not step.get('ok') or not receipt.get('executed') or not receipt.get('measured_arrival'):
        invalidate('Approach motion not confirmed');raise ValueError('Cannot advance target after failed motion')
    robot=ctx.value('piper_status')
    if (not robot.get('motion_ready') or robot.get('control_uncertain')
            or robot['bridge']['phase'] not in ('IDLE','DONE') or robot['bridge']['instance']!=r['bridge_instance']
            or robot['owner_status']['generation']!=r['owner_generation']
            or not observation_pose_matches(receipt['final14'],robot['measured_state']['values'])):
        invalidate('Owner/state changed after approach');raise ValueError('Approach completion changed')
    r['last_motion']=robot.get('last_motion')
    r['expected14']=receipt['final14'];r['last_frame']=step['after_frame'];save(r)
