"""Bounded task-local numerical pregrasp search; no observations or dispatch.

Snapshot provenance is validated by GreenPregrasp.prepare before search. Neither
historical frames nor timestamps are fabricated. Rebind to a real observation
before returning inputs and recheck again in the execution function.
"""
import math,time
from collections import Counter
import numpy as np
from scene_checks import PerceptionUnknown
from green_checks import planned_green_path,PREGRASP_HEIGHT_RANGE

PREFERRED_ALLOWANCE_M=.003  # search for slack beyond existing20mm, not a smaller safety margin
HEIGHT_NEIGHBORHOOD_M=.001
MAX_SEARCH_S=25.


def rotated(q,axis,degrees):
    v=np.zeros(3);v[axis]=math.sin(math.radians(degrees)/2);w=math.cos(math.radians(degrees)/2)
    b=np.asarray(q[:3]);d=q[3]
    return np.r_[w*b+d*v+np.cross(v,b),w*d-v@b].tolist()


def rotation_matrix(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def insertion_preview(ctx,evidence,hover,quaternion,end_joints):
    """Prospective native paths to insertion; actual alignment/entry still checked.

    Contact-area visual facts are NOT inferred here. Open jaw and same near-down
    posture remain unchanged; final grasp uses the original measured block XY.
    """
    from composition import INSERTION_HEIGHT,APPROACH_STEP
    scene=evidence['scene'];green=np.asarray(scene['blocks']['green']['top_center']['surface_position_m'])
    start=list(end_joints)+list(evidence['start'][6:]);p=np.asarray(hover);goal=green[2]+INSERTION_HEIGHT
    contact_offset=rotation_matrix(quaternion)@np.asarray(evidence['tcp_offset_flange'])
    if np.linalg.norm(p[:2]+contact_offset[:2]-green[:2])>1e-8:raise PerceptionUnknown('Preview TCP must retain measured green grasp center')
    for name,b in scene['blocks'].items():
        if name!='green' and np.linalg.norm(np.asarray(b['top_center']['surface_position_m'])[:2]-p[:2])<.10:
            raise PerceptionUnknown('Neighbor too close to this vertical approach corridor')
    rows=[];plane=np.asarray(scene['table']['z_ax_by_c']);local=evidence['local']
    while p[2]>goal+1e-8:
        p=p.copy();p[2]=max(goal,p[2]-APPROACH_STEP)
        sol=ctx.value('piper_solve_ik',position=p.tolist(),quaternion_xyzw=quaternion,seed=start[:6]);end=sol['q']
        check=ctx.value('piper_check_path',start14=start,target14=end+start[6:])
        if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:raise PerceptionUnknown('Native insertion preview path refused')
        minimum=1.
        from fk_samples import fk_line
        for t,fk in fk_line(ctx,start,end,np.linspace(0,1,41)):
            F=np.asarray(fk['T_base_flange'])
            corners=local@F[:3,:3].T+F[:3,3];gap=float(np.min(corners[:,2]-(corners[:,:2]@plane[:2]+plane[2])))
            minimum=min(minimum,gap)
            if F[2,2]>-.995 or gap<.012:raise PerceptionUnknown('Insertion preview violates near-down/table constraints')
        rows.append({'position':p.tolist(),'native_path':check,'min_finger_table_m':minimum});start=end+start[6:]
    return {'segments':rows,'scope':'Numerical lookahead only; no contact/retention claim and no motion issued'}


def choose(ctx,evidence,robot,report):
    from composition import _planning_refusal
    top=np.asarray(evidence['scene']['blocks']['green']['top_center']['surface_position_m'])
    base_q=evidence['fk']['quaternion_xyzw'];start=evidence['start'];scene=evidence['scene'];local=evidence['local']
    # Keep measured XY. Small rotations at an empty, clear approach may avoid
    # the old fixed-orientation IK boundary without shifting the grip off-center.
    rotations=[(None,0)]+[(axis,deg) for deg in (-2,2,-4,4) for axis in (1,0)]
    heights=sorted([round(.180+i*.005,6) for i in range(13)],key=lambda h:(round(abs(h-.195),6),-h))
    rows=[];report['candidates']=rows;started=time.monotonic()
    for height in heights:
        for axis,deg in rotations:
            if time.monotonic()-started>MAX_SEARCH_S:raise ValueError('Green route search budget exhausted; no movement; see candidate refusal summary')
            q=base_q if axis is None else rotated(base_q,axis,deg)
            contact_offset=rotation_matrix(q)@np.asarray(evidence['tcp_offset_flange'])
            target=[*(top[:2]-contact_offset[:2]),float(top[2]+height)];args={'waypoints':[target],'quaternion':q}
            row={'height_above_top_m':height,'rotation_axis_base':axis,'rotation_deg':deg,'position':target,'tcp_xy':(np.asarray(target[:2])+contact_offset[:2]).tolist()};rows.append(row)
            if not all(a<=v<=b for v,a,b in zip(target,robot['workspace_min'],robot['workspace_max'])):
                row.update(ok=False,why='Outside current workspace');continue
            try:
                fact=planned_green_path(ctx,start,scene,local,args)
                if fact['min_block_allowance_m']<PREFERRED_ALLOWANCE_M:
                    raise PerceptionUnknown('Candidate lacks preferred3mm planning slack beyond original20mm allowance')
                # Avoid accepting a single needle-thin numerical solution.
                band=[]
                for dz in (-HEIGHT_NEIGHBORHOOD_M,HEIGHT_NEIGHBORHOOD_M):
                    nearby=[*target[:2],target[2]+dz]
                    if not PREGRASP_HEIGHT_RANGE[0]<=height+dz<=PREGRASP_HEIGHT_RANGE[1]:raise PerceptionUnknown('Candidate too close to hover corridor boundary')
                    check=planned_green_path(ctx,start,scene,local,{'waypoints':[nearby],'quaternion':q})
                    band.append({'delta_z_m':dz,'min_allowance_m':check['min_block_allowance_m']})
                onward=insertion_preview(ctx,evidence,target,q,fact['end_joints'])
            except PerceptionUnknown as exc:
                row.update(ok=False,why=str(exc));continue
            except RuntimeError as exc:
                if not _planning_refusal(exc):raise
                row.update(ok=False,why='Native read-only IK refusal');continue
            row.update(ok=True,min_allowance_m=fact['min_block_allowance_m'])
            report['route_plan']={'selected':row,'height_neighborhood':band,'insertion_preview':onward,
                'executed':False,'source_frame_id':evidence['frame_id'],'candidate_count':len(rows)}
            return args,fact
    report['candidate_summary']=dict(Counter(row.get('why','unknown') for row in rows))
    raise ValueError('No green approach after '+str(len(rows))+' numerical candidates; no movement; '+str(report['candidate_summary']))
