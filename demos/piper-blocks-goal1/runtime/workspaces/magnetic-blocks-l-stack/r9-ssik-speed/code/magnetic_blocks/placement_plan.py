"""Bounded, read-only placement candidates; never dispatch or relax collision.

60 mm is the block size, not a promise that fingers clear the neighbor there.
Select a nearby side-by-side position with nominal finger margin before moving.
"""
import numpy as np
from scene_checks import PerceptionUnknown,scene_control,scene_geometry


def side_by_side(ctx,obs,scene,origin,quaternion,*,spacing_min_m=.060,
                 spacing_max_m=.075,preferred_spacing_m=.065,finger_margin_m=.003):
    scene_control(scene,ctx.value('piper_status'));scene_geometry(scene,obs)
    from manipulation_checks import finger_locals
    result=plan_side_by_side(ctx,obs['measured14'],scene,origin,quaternion,finger_locals(ctx,obs),
        spacing_min_m=spacing_min_m,spacing_max_m=spacing_max_m,
        preferred_spacing_m=preferred_spacing_m,finger_margin_m=finger_margin_m)
    return dict(result,frame_id=obs['frame_id'])


def plan_side_by_side(ctx,start,scene,origin,quaternion,local,*,spacing_min_m=.060,
                      spacing_max_m=.075,preferred_spacing_m=.065,finger_margin_m=.003):
    """Prospective joints only; no synthetic observation or completed-action claim."""
    values=np.array([spacing_min_m,spacing_max_m,preferred_spacing_m,finger_margin_m],float)
    if not np.isfinite(values).all() or not .060<=spacing_min_m<=preferred_spacing_m<=spacing_max_m<=.075 or not .003<=finger_margin_m<=.015:
        raise ValueError('Side-by-side candidates: 60..75mm center spacing, at least3mm nominal finger margin')
    if origin.get('color')!='orange':raise PerceptionUnknown('Base-pair planner accepts orange only')
    f=ctx.value('piper_fk',joints=start[:6]);T=np.array(f['T_base_flange']);p=np.array(f['position'])
    if T[2,2]>-.995 or abs(T[0,0])<.98 or abs(np.dot(quaternion,f['quaternion_xyzw']))<.99999:
        raise PerceptionUnknown('Preserve the verified downward front/back grip for placement')
    red=np.array(scene['blocks']['red']['top_center']['surface_position_m']);plane=np.array(scene['table']['z_ax_by_c'])
    offset=origin['flange_to_bottom_m']
    if p[2]-offset-red[2]<.020:raise PerceptionUnknown('Plan transfer from a verified lifted pose above the neighboring block')
    values=sorted(set([spacing_min_m,preferred_spacing_m,spacing_max_m]+list(np.arange(spacing_min_m,spacing_max_m+.0001,.0025))),key=lambda x:abs(x-preferred_spacing_m))
    rows=[]
    for spacing in values:
        above=np.array([red[0],red[1]-spacing,p[2]])
        target=above.copy();target[2]=target[:2]@plane[:2]+plane[2]+offset
        row={'center_spacing_m':float(spacing),'transfer':above.tolist(),'descent':target.tolist(),'retreat':above.tolist()}
        try:
            from manipulation_checks import loaded_segment
            transfer_check=loaded_segment(ctx,start,above,quaternion,scene,origin,local)
            q0=list(start[:6]);minimum=1.;preflights=[]
            middle=target.copy();middle[2]=(above[2]+target[2])/2
            if not .005<=above[2]-middle[2]<=.070 or not .005<=middle[2]-target[2]<=.070:
                raise PerceptionUnknown('Placement needs two5..70mm checked descent boundaries')
            for leg,destination in [('transfer',above),('descent_1',middle),('descent_2',target),('retreat',above)]:
                sol=ctx.value('piper_solve_ik',position=destination.tolist(),quaternion_xyzw=quaternion,seed=q0)
                if not sol.get('ok'):raise PerceptionUnknown('No full-pose IK for '+leg)
                q1=sol['q'];check=ctx.value('piper_check_path',start14=q0+start[6:],target14=q1+start[6:])
                if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:
                    raise PerceptionUnknown('Native CAD/workspace refused '+leg)
                preflights.append({'leg':leg,'result':check})
                from fk_samples import fk_line
                for t,fk in fk_line(ctx,q0,q1,np.linspace(0,1,41)):
                    F=np.asarray(fk['T_base_flange'])
                    fp=F[:3,3];table=float(fp[:2]@plane[:2]+plane[2]);bottom=fp[2]-offset
                    if bottom<table-.002:raise PerceptionUnknown('Payload below modeled table')
                    for corners0 in local:
                        corners=corners0@F[:3,:3].T+fp;lo=corners.min(axis=0);hi=corners.max(axis=0)
                        if lo[2]-table<.012:raise PerceptionUnknown('Finger/table clearance below12mm')
                        sep=float(np.max(np.maximum(lo-(red+[.030,.030,0]),(red-[.030,.030,.060])-hi)))
                        minimum=min(minimum,sep)
                        if sep<finger_margin_m:raise PerceptionUnknown('Insufficient nominal finger/red margin')
                    for color,b in scene['blocks'].items():
                        if color in ('red','orange'):continue
                        bp=np.array(b['top_center']['surface_position_m'])
                        if max(np.max(np.abs(fp[:2]-bp[:2]))-.070,bottom-bp[2])<.020:
                            raise PerceptionUnknown('Payload too close to other block')
                q0=q1
            row.update(ok=True,min_nominal_finger_red_m=minimum,preflights=preflights)
            rows.append(row)
            return {'ok':True,'executed':False,'selected':row,'candidates':rows,
                    'quaternion':quaternion,'transfer_check':transfer_check,
                    'scope':'Planning only. Function dispatch rechecks actual state and path. Margin is nominal, not independent metrology.'}
        except (PerceptionUnknown,RuntimeError) as exc:
            from composition import _planning_refusal
            if isinstance(exc,RuntimeError) and not isinstance(exc,PerceptionUnknown) and not _planning_refusal(exc):raise
            # Only planning was attempted. Never catch/retry a hardware dispatch.
            row.update(ok=False,why=str(exc));rows.append(row)
    return {'ok':False,'executed':False,'candidates':rows,'why':'No candidate with nominal margin; adjust grasp orientation or obtain better geometry; do not shrink collision envelopes'}
