"""Task-local read-only lift selection, using the SAME execution path checks.

No hardware dispatch, imagery synthesis, model calls, weakened margins or fixed
85/55/25mm target. Candidate resolution is not a new control tolerance.
"""
import math
import numpy as np
from scene_checks import PerceptionUnknown,scene_control,scene_geometry
from manipulation_checks import payload_origin,finger_locals,loaded_segment

RESOLUTION_M=.0025
PLANNING_RESERVE_M=.005  # above existing20mm clearance; never subtracts from checks


def choose_lift(ctx,obs,robot,prev,previous_report,mode,color,support_report=None):
    if (mode,color) not in (('orange_raise','orange'),('green_raise_1','green'),('green_raise_2','green')):
        raise ValueError('Lift mode/payload mismatch')
    if prev.get('ok') is not True or prev['stage'] not in ('trial_lift','transport'):
        raise PerceptionUnknown('Lift needs successful trial/transport endpoint')
    scene=prev['facts'][0]['scene'];scene_control(scene,robot);scene_geometry(scene,obs)
    if color=='green':
        if not support_report:raise PerceptionUnknown('Green lift requires verified support history')
        from stack_checks import support_scene
        scene=support_scene(ctx,previous_report,obs,robot,support_report,scene)
    origin=payload_origin(ctx,previous_report,prev,scene,color)
    local=finger_locals(ctx,obs);start=list(obs['measured14'])
    fk=ctx.value('piper_fk',joints=start[:6]);p=np.asarray(fk['position']);q=fk['quaternion_xyzw']
    return select_height(ctx,start,p,q,scene,origin,local,robot,mode)


def select_height(ctx,start,p,q,scene,origin,local,robot,mode):
    """Prospective numeric plan. Always rechecked from measured state at dispatch."""
    from composition import _planning_refusal
    from placement_plan import plan_side_by_side
    from stack_checks import stack_target,planned_stack_geometry
    p=np.asarray(p,float);color=origin['color'];plane=np.asarray(scene['table']['z_ax_by_c'])
    if mode=='orange_raise':
        red=np.asarray(scene['blocks']['red']['top_center']['surface_position_m'])
        destination=np.array([red[0],red[1]-.065,p[2]])  # actual spacing searched below
    elif mode=='green_raise_1':
        destination=p.copy();destination[1]=scene['blocks']['orange']['top_center']['surface_position_m'][1]-.105
    else:destination=np.asarray(stack_target(scene,origin))
    # Conservative straight-corridor floor is only a search starting point.
    # Joint-interpolated actual paths and all installed corners are then tested.
    low=p[2]+.005+1e-8;high=min(p[2]+.100-1e-8,robot['workspace_max'][2])
    neighbor_floor=[]
    lo=np.minimum(p[:2],destination[:2]);hi=np.maximum(p[:2],destination[:2])
    for name,block in scene['blocks'].items():
        if name==color:continue
        bp=np.asarray(block['top_center']['surface_position_m'])
        if np.all(bp[:2]>=lo-.090) and np.all(bp[:2]<=hi+.090):
            floor=bp[2]+origin['flange_to_bottom_m']+.020+PLANNING_RESERVE_M
            low=max(low,float(floor));neighbor_floor.append({'color':name,'minimum_flange_z_m':float(floor)})
    if mode=='orange_raise':
        # Two separate table descents, each retaining the original5..70mm range.
        support_z=float(destination[:2]@plane[:2]+plane[2]+origin['flange_to_bottom_m'])
        low=max(low,support_z+.025+PLANNING_RESERVE_M);high=min(high,support_z+.140)
    elif mode=='green_raise_2':
        low=max(low,destination[2]+.025+PLANNING_RESERVE_M);high=min(high,destination[2]+.050)
    heights=[]
    if low<=high:
        heights=[float(low+i*RESOLUTION_M) for i in range(int((high-low)/RESOLUTION_M)+1)]
        if high-heights[-1]>.0001:heights.append(float(high))
    rows=[]
    for z in heights:
        target=[*p[:2],z];row={'position':target,'rise_m':float(z-p[2])};rows.append(row)
        if not all(a<=v<=b for v,a,b in zip(target,robot['workspace_min'],robot['workspace_max'])):
            row.update(ok=False,why='Outside current workspace');continue
        try:
            lift=loaded_segment(ctx,start,target,q,scene,origin,local)
            lifted=lift['end_joints']+list(start[6:])
            if mode=='orange_raise':
                follow=plan_side_by_side(ctx,lifted,scene,origin,q,local)
                if follow.get('ok') is not True:
                    row.update(ok=False,why=follow.get('why'),placement_candidates=follow.get('candidates'));continue
            else:
                destination[2]=z
                transfer=loaded_segment(ctx,lifted,destination,q,scene,origin,local)
                follow={'transfer':transfer}
                if mode=='green_raise_1':
                    end=transfer['end_joints']+list(start[6:])
                    endpoint=ctx.value('piper_fk',joints=end[:6])
                    follow['next_lift']=select_height(ctx,end,endpoint['position'],q,scene,origin,local,robot,'green_raise_2')
                if mode=='green_raise_2':
                    end=transfer['end_joints']+list(start[6:]);support=stack_target(scene,origin)
                    descent=planned_stack_geometry(ctx,end,scene,origin,support,q,local,final=True)
                    reverse=ctx.value('piper_check_path',start14=descent['end_joints']+list(start[6:]),target14=end)
                    if reverse.get('cad_clear') is not True or reverse.get('workspace_clear') is not True:
                        raise PerceptionUnknown('Native stack retreat path refused')
                    follow.update(descent=descent,retreat=reverse)
            row.update(ok=True)
            return {'ok':True,'executed':False,'selected':row,'candidates':rows,
                    'neighbor_floor':neighbor_floor,'height_interval_m':[float(low),float(high)],
                    'lift':lift,'following_path':follow,
                    'scope':'Prospective native IK/CAD/workspace + installed finger/payload checks; actual execution revalidates measured state. No retention/support claim.'}
        except PerceptionUnknown as exc:
            row.update(ok=False,why=str(exc))
        except RuntimeError as exc:
            if not _planning_refusal(exc):raise
            row.update(ok=False,why='Native read-only IK refusal; no action dispatched')
    error=PerceptionUnknown('No feasible lift and following path within original motion/workspace limits')
    error.lift_diagnostics={'ok':False,'executed':False,'candidates':rows,'neighbor_floor':neighbor_floor,
                            'height_interval_m':[float(low),float(high)]}
    raise error
