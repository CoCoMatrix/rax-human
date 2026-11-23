"""One bounded empty transition; original waypoints and native motion unchanged.

The first real observation supplies scene/rigid finger geometry. Between legs,
only current control feedback and numeric remaining-path checks are used. No
predicted state is represented as an image, depth measurement or scene refresh.
"""
import time
import numpy as np
import controller as c
from perception import PerceptionUnknown


def geometry(ctx, start, end, scene, local, robot):
    """Same 60-degree, table/block envelope and native CAD/workspace checks."""
    from task_geometry import envelope_clearance
    a=np.asarray(c._vector(start,14));b=np.asarray(c._vector(end,6))
    if max(abs(b-a[:6]))>np.pi/3:
        raise PerceptionUnknown('Joint segment exceeds native60deg conditional range')
    if not .065<=a[6]<=.074:
        raise PerceptionUnknown('Empty transition requires the observed open block jaw')
    table_min=block_min=float('inf')
    from fk_samples import fk_line
    for u,fk in fk_line(ctx,a,b,np.linspace(0.,1.,41)):
        T=np.asarray(fk['T_base_flange'],float)
        if T.shape!=(4,4) or not np.isfinite(T).all():
            raise PerceptionUnknown('Malformed empty transition FK')
        if np.any(T[:3,3]<robot['workspace_min']) or np.any(T[:3,3]>robot['workspace_max']):
            raise PerceptionUnknown('Empty transition leaves live flange workspace')
        world=local@T[:3,:3].T+T[:3,3]
        check=envelope_clearance(list(world.reshape(-1,8,3)),scene)
        table_min=min(table_min,check['min_table_clearance_lower_m'])
        block_min=min(block_min,check['min_block_separation_lower_m'])
    native=ctx.value('piper_check_path',start14=a.tolist(),target14=b.tolist()+a[6:].tolist())
    if native.get('cad_clear') is not True or native.get('workspace_clear') is not True:
        raise PerceptionUnknown('Native empty path preflight did not establish CAD/workspace clearance')
    return {'samples':41,'min_table_clearance_lower_m':table_min,
            'min_block_clearance_lower_m':block_min,'native_path':native,
            'start14':a.tolist(),'target_joints':b.tolist()}


def feedback(ctx, robot, expected, initial, previous_receipt=None):
    """Read a real idle state; retain original tolerances and owner identity."""
    from observations import identity
    from scene_checks import same_state
    state=ctx.value('status');c._budget(state,c.DISPATCH_ALLOWANCE_S)
    now=state['robot_status'];bridge=now.get('bridge',{})
    if (state.get('control_uncertain') is not False or now.get('motion_ready') is False
        or bridge.get('active_request') or bridge.get('phase') not in ('IDLE','DONE')):
        raise RuntimeError('Empty transition control is uncertain or not idle/ready')
    if identity(robot) is None or identity(now)!=identity(robot):
        raise RuntimeError('Empty transition owner/calibration/geometry identity changed')
    fb=bridge.get('feedback',{});stamp=fb.get('timestamp_ns');q=fb.get('q')
    if (type(stamp) is not int or not 0<=time.time_ns()-stamp<=300_000_000
        or fb.get('generation')!=now['owner_status']['generation']):
        raise RuntimeError('Empty transition lacks current owner feedback')
    if not same_state(q,expected):
        raise RuntimeError('Empty transition feedback differs from completed endpoint')
    fixed=list(q);fixed[:6]=initial[:6]
    if not same_state(fixed,initial):
        raise RuntimeError('Jaw/right arm changed during empty transition')
    if previous_receipt is not None:
        request=previous_receipt.get('owner_request_id')
        if not request or (now.get('last_motion') or {}).get('owner_request_id')!=request:
            raise RuntimeError('An intervening action invalidated the empty transition')
    return now,list(q)


def execute(ctx, joint_waypoints, report):
    from observations import identity
    from scene_checks import measure_scene,installed_finger_corners,scene_control,same_state
    points=[c._vector(q,6) for q in joint_waypoints]
    if not 1<=len(points)<=3:
        raise ValueError('Use the bounded one-to-three planned empty entry waypoints')
    robot,before=c._start(ctx,report=report,require_depth=True)
    if identity(robot) is None:raise RuntimeError('Missing empty transition control identity')
    scene=measure_scene(ctx,before);scene_control(scene,robot,establish=True)
    plane=np.asarray(scene['table']['z_ax_by_c'])
    for block in scene['blocks'].values():
        p=np.asarray(block['top_center']['surface_position_m'])
        if not .045<=p[2]-(p[:2]@plane[:2]+plane[2])<=.085:
            raise PerceptionUnknown('Not all three blocks are visibly at table level')
    initial=before['measured14'];fk=ctx.value('piper_fk',joints=initial[:6])
    T=np.asarray(fk['T_base_flange']);corners=installed_finger_corners(ctx,before,scene,fk)
    local=(corners-T[:3,3])@T[:3,:3]
    report['planned_motion_count']=len(points)
    report['empty_transition']={'waypoints_unchanged':True,'intermediate_images':False,
        'completed_segments':0,'segments':[],
        'scope':'Same bounded empty flight; retained scene is not a new observation'}
    report['facts'].append({'ok':True,'frame_id':before['frame_id'],'scene':scene,
        'why':'Initial current table/block scene retained during checked empty flight; each leg uses real feedback and numeric path validation'})
    after=before;previous=None
    for i,q in enumerate(points):
        started=time.monotonic()
        current,start=feedback(ctx,robot,after['measured14'],initial,previous)
        scene_control(scene,current)
        check=geometry(ctx,start,q,scene,local,current)
        row={'index':i+1,'path_check':check,'check_s':time.monotonic()-started,
             'observe_after':i==len(points)-1,'completed':False}
        report['empty_transition']['segments'].append(row)
        after=c._move(ctx,'piper_move_joints',{'joints':q},report,i==len(points)-1)
        previous=report['receipts'][-1]['result']
        # Receipt continuity is checked before another leg, even with no image.
        if (previous.get('owner_generation')!=robot['owner_status']['generation']
            or not same_state(previous.get('anchor_validation',{}).get('measured14'),start)
            or not same_state(previous.get('final14'),q+start[6:])):
            raise RuntimeError('Empty transition receipt does not match checked start/target')
        row.update(completed=True,elapsed_s=time.monotonic()-started,
                   request_id=previous.get('owner_request_id'))
        report['empty_transition']['completed_segments']=i+1
    return after
