"""Same-run empty insertion: plan the fewest checked joint-line descents.

Only ordinary FK/IK/path calls here. No predicted observation is represented as
measured. Full-path geometry is checked before dispatch; real RGB evidence is
checked at the two action boundaries. Closure/trial remain separate functions.
"""
import math
import time
import numpy as np
from perception import PerceptionUnknown
import controller as c


def geometry(ctx, start, end, scene, color, local, *, robot):
    """Check actual joint-interpolated path, not a Cartesian endpoint shortcut."""
    a=np.asarray(start,float);b=np.asarray(end,float)
    if a.shape!=(14,) or b.shape!=(6,) or not np.isfinite([*a,*b]).all():
        raise PerceptionUnknown('Invalid insertion joint path')
    delta=b-a[:6]
    if max(abs(delta))>math.pi/3:raise PerceptionUnknown('Insertion exceeds native60deg')
    block=np.asarray(scene['blocks'][color]['top_center']['surface_position_m'])
    plane=np.asarray(scene['table']['z_ax_by_c'])
    # At least41 samples, and at most0.002rad between samples. Quintic timing
    # stays on this same line; no unreviewed smoothing or endpoint-only claim.
    count=max(41,math.ceil(float(max(abs(delta)))/.002)+1)
    minimum=float('inf');max_xy=0.;last_z=None;first=None;last=None
    from fk_samples import fk_line
    for u,fk in fk_line(ctx,a,b,np.linspace(0.,1.,count)):
        T=np.asarray(fk['T_base_flange']);p=T[:3,3]
        if T.shape!=(4,4) or not np.isfinite(T).all():raise PerceptionUnknown('Invalid FK')
        if np.any(p<robot['workspace_min']) or np.any(p>robot['workspace_max']):
            raise PerceptionUnknown('Insertion path leaves current workspace')
        if T[2,2]>-.995:raise PerceptionUnknown('Insertion path loses near-down orientation')
        xy=float(np.linalg.norm(p[:2]-block[:2]));max_xy=max(max_xy,xy)
        if xy>.012:raise PerceptionUnknown('Insertion path leaves existing12mm block-center corridor')
        if p[2]<block[2]+.125:raise PerceptionUnknown('Insertion path exceeds existing contact depth')
        if last_z is not None and p[2]>last_z+.0002:
            raise PerceptionUnknown('Insertion path is not monotonically descending')
        for name,other in scene['blocks'].items():
            if name!=color and np.linalg.norm(np.asarray(other['top_center']['surface_position_m'])[:2]-p[:2])<.10:
                raise PerceptionUnknown('Insertion path violates existing neighbor separation')
        world=np.asarray(local)@T[:3,:3].T+p
        gap=float(np.min(world[:,2]-world[:,:2]@plane[:2]-plane[2]));minimum=min(minimum,gap)
        if gap<.012:raise PerceptionUnknown('Insertion finger/table clearance below existing12mm')
        if first is None:first=p.copy()
        last=p.copy();last_z=p[2]
    if not 0<first[2]-last[2]<=.4:raise PerceptionUnknown('Insertion must descend within400mm')
    native=ctx.value('piper_check_path',start14=a.tolist(),target14=b.tolist()+a[6:].tolist())
    if native.get('cad_clear') is not True or native.get('workspace_clear') is not True:
        raise PerceptionUnknown('Native insertion path refused')
    return {'samples':count,'min_tip_table_m':minimum,'max_center_offset_m':max_xy,
            'joint_sample_step_rad':float(max(abs(delta)))/(count-1),
            'native_cad_clear':True,'native_workspace_clear':True,
            'scope':'Same known scene and unchanged open jaw; no dynamic obstacle claim'}


def plan(ctx, robot, before, scene, color, points, quaternion):
    from scene_checks import installed_finger_corners
    from composition import _planning_refusal
    start=before['measured14'];fk=ctx.value('piper_fk',joints=start[:6]);T=np.asarray(fk['T_base_flange'])
    corners=installed_finger_corners(ctx,before,scene,fk);local=(corners-T[:3,3])@T[:3,:3]
    candidates=[]
    # Try one endpoint, then progressively finer subsets of the original
    # scene-derived points. No visual reacquisition inside numerical search.
    for n in range(1,len(points)+1):
        indices=sorted(set(math.ceil((i+1)*len(points)/n)-1 for i in range(n)))
        current=list(start);parts=[]
        try:
            for idx in indices:
                sol=ctx.value('piper_solve_ik',position=points[idx],quaternion_xyzw=quaternion,seed=current[:6])
                if sol.get('ok') is not True:raise PerceptionUnknown('Insertion IK unavailable')
                q=c._vector(sol['q'],6);check=geometry(ctx,current,q,scene,color,local,robot=robot)
                parts.append({'joints':q,'target':points[idx],'check':check});current=q+current[6:]
            candidates.append({'segments':n,'ok':True})
            return {'parts':parts,'candidates':candidates,'original_segments':len(points)},local
        except PerceptionUnknown as exc:candidates.append({'segments':n,'ok':False,'why':str(exc)})
        except RuntimeError as exc:
            if not _planning_refusal(exc):raise
            candidates.append({'segments':n,'ok':False,'why':str(exc)[:500]})
    exc=PerceptionUnknown('No insertion path passes full-path checks');exc.candidates=candidates;raise exc


def execute(ctx,robot,before,scene,color,points,quaternion,engage,report):
    from scene_checks import approach_scene,same_state,scene_control
    total=before['measured14'];fk=ctx.value('piper_fk',joints=total[:6])
    drop=fk['position'][2]-points[-1][2]
    inputs={'waypoints':points,'quaternion':quaternion}
    # Existing qualitative alignment/table/depth limits, with full numeric
    # path validation replacing the old per-command21mm distance gate.
    report['facts'].append(c._fact(lambda **kw:approach_scene(ctx,kw['after'],scene,color,
        points[-1],engage=engage,max_down_m=drop+.000001),'insertion_boundary',before,before,inputs))
    started=time.monotonic()
    chosen,local=plan(ctx,robot,before,scene,color,points,quaternion)
    report['continuous_approach']={**chosen,'planning_s':time.monotonic()-started}
    report['planned_motion_count']=len(chosen['parts'])
    rebound=False
    if time.monotonic()-started>5:
        refreshed_robot,fresh=c._start(ctx,motion=False,force_observe=True)
        if refreshed_robot['bridge']['instance']!=robot['bridge']['instance'] or not same_state(fresh['measured14'],before['measured14']):
            raise PerceptionUnknown('Owner/state changed during insertion planning')
        scene_control(scene,refreshed_robot)
        report['facts'].append(c._fact(lambda **kw:approach_scene(ctx,kw['after'],scene,color,
            points[-1],engage=engage,max_down_m=drop+.001),'rebound_insertion_boundary',before,fresh,inputs))
        before=fresh;robot=refreshed_robot;rebound=True
    after=before
    for i,part in enumerate(chosen['parts']):
        # Actual feedback after a split is mandatory; no synthetic frame or
        # assumed endpoint is allowed to stand in for it.
        check=(geometry(ctx,after['measured14'],part['joints'],scene,color,local,robot=robot)
               if i or rebound else part['check'])
        part['dispatch_check']=check
        after=c._move(ctx,'piper_move_joints',{'joints':part['joints']},report,i==len(chosen['parts'])-1)
    report['facts'].append(c._fact(lambda **kw:approach_scene(ctx,kw['after'],scene,color,
        points[-1],engage=engage),'grasp_alignment' if engage else 'hover_alignment',before,after,inputs))
    return after
