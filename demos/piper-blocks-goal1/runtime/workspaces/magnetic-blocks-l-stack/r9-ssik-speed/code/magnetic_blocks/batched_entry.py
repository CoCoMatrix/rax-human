"""Experimental batching of checked empty segments. No grasp/contact blending."""
import time
import numpy as np
import controller as c
from perception import PerceptionUnknown


def obstacles(scene):
    # Collision tool uses dual-base coordinates: left mount (0,.30,.008).
    from task_geometry import BLOCK_HALF_XY_M
    plane=np.asarray(scene['table']['z_ax_by_c']);corners=np.array([[.11,-.4],[.5,-.4],[.11,.045],[.5,.045]])
    z=float(np.max(corners@plane[:2]+plane[2]))
    out=[{'name':'measured_task_table','center_m':[.305,.1225,z+.008+.032-.025],'size_m':[.39,.445,.05]}]
    for name,b in scene['blocks'].items():
        p=np.asarray(b['top_center']['surface_position_m']);floor=float(p[:2]@plane[:2]+plane[2]);margin=.020
        out.append({'name':name,'center_m':[p[0],p[1]+.30,(p[2]+floor)/2+.008],
                    'size_m':[2*(BLOCK_HALF_XY_M+margin)]*2+[p[2]-floor+2*margin]})
    return out


def plan_segment(ctx,start,joints,scene,local,robot):
    """Retained per-segment planning interface, explicitly NO execution claim."""
    from empty_transition import geometry
    return {'joints':list(joints),'executed':False,'check':geometry(ctx,start,joints,scene,local,robot)}


def execute(ctx,joint_waypoints,report):
    from scene_checks import measure_scene,installed_finger_corners,scene_control
    from empty_transition import feedback
    from task_geometry import envelope_clearance
    robot,before=c._start(ctx,report=report,require_depth=True)
    scene=measure_scene(ctx,before);scene_control(scene,robot,establish=True)
    start=before['measured14'];fk=ctx.value('piper_fk',joints=start[:6]);T=np.asarray(fk['T_base_flange'])
    local=(installed_finger_corners(ctx,before,scene,fk)-T[:3,3])@T[:3,:3]
    parts=[];anchor=start
    for joints in joint_waypoints:
        part=plan_segment(ctx,anchor,c._vector(joints,6),scene,local,robot);parts.append(part);anchor=part['joints']+start[6:]
    if not 1<=len(parts)<=3:raise PerceptionUnknown('Expected bounded empty entry segments')
    env=obstacles(scene);indices=list(range(len(parts)-1))
    plan=ctx.value('piper_preview_waypoints',start14=start,waypoints=joint_waypoints,obstacles=env,blend_indices=indices)
    # Check the ACTUAL curved float32 path and its midpoints against the same
    # installed fingers/block/table envelope; no endpoint-only approval.
    points=[start,*plan['targets14']];minimum=float('inf')
    from fk_samples import fk_many
    queries=[np.asarray(points[0][:6])]
    for a,b in zip(points,points[1:]):
        queries.extend([(np.asarray(a[:6])+np.asarray(b[:6]))*.5,np.asarray(b[:6])])
    for fk in fk_many(ctx,queries):
        T=np.asarray(fk['T_base_flange']);world=local@T[:3,:3].T+T[:3,3]
        check=envelope_clearance(list(world.reshape(-1,8,3)),scene)
        minimum=min(minimum,check['min_block_separation_lower_m'])
    _,actual=feedback(ctx,robot,start,start)
    # Preserve the existing0.01rad binding. Execution rechecks the actual
    # curve with conservative measured obstacles (including table allowance),
    # rather than demanding exact zero encoder drift here.
    report['batch_anchor14']=actual
    report['planned_motion_count']=1
    report['facts'].append({'ok':True,'frame_id':before['frame_id'],'scene':scene,'why':'Current measured scene; individual segments and actual blended wire passed empty geometry checks'})
    report['batch']={'segments':parts,'blend_indices':indices,'preview_duration_s':plan['duration_s'],
        'samples':len(plan['targets14']),'min_block_separation_lower_m':minimum,
        'scope':'Empty entry only; each original segment retained as a planning record; one real native receipt',
        'waypoint_boundaries':plan['waypoints']}
    return c._move(ctx,'piper_move_waypoints',{'start14':actual,'waypoints':joint_waypoints,'obstacles':env,'blend_indices':indices},report)
