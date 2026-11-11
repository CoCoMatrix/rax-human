"""Task-local preconditions/results. Ordinary rules first; Gemini is a fallback.

These are qualitative blue-cup heuristics, not contact-force measurements or a
complete environment collision certificate. No motion, network or new Context.
Unknown stays unknown until the optional model resolves that particular fact.
"""
import json
import math
import time
from pathlib import Path
import cv2
import numpy as np
from perception import OUT, observation_recent, observation_pose_matches, unchanged_rgb
from scene_geometry import base_depth

RIM_SHIFT_PX=12
RIM_EDGE_FRACTION=.30
SUPPORT_BAND_M=.008
TRACK_MIN_POINTS=8
TRACK_MAX_SHIFT_PX=14


def current_depth(ctx, obs):
    """One fresh depth request only where ordinary geometry rules can use it."""
    head=obs.get('cameras',{}).get('head',{})
    if not head.get('K') or not head.get('T_base_camera') or head.get('metric_depth'):return obs
    fresh=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=True)
    if (not observation_recent(fresh) or not observation_pose_matches(obs['measured14'],fresh['measured14']) or
            fresh['state_binding']['owner_generation']!=obs['state_binding']['owner_generation'] or
            fresh['finger_geometry']['geometry_identity']!=obs['finger_geometry']['geometry_identity'] or
            fresh.get('calibration_sha256')!=obs.get('calibration_sha256')):
        raise ValueError('State/owner/calibration changed during current-depth measurement')
    return fresh


def context_for(ctx, obs, *, robot, action, fk=None, target14=None, inputs=None,
                before=None, final_fk=None, geometry=None):
    """Small explicit evidence bundle for the existing function, using read-only FK."""
    inputs=inputs or {};out={'robot':robot,'action':action,'require_support_state':inputs.get('assess_support',False) or action=='assess_support'}
    out['rebuild_inputs']={'action':action,'inputs':inputs,'target14':target14,'fk':fk,'final_fk':final_fk,'geometry':geometry}
    out['modeled_path_clear']=bool(geometry and geometry.get('cad_clear') and geometry.get('whole_path_checked') and geometry.get('geometry_identity')==obs.get('finger_geometry',{}).get('geometry_identity'))
    try:
        target=_target(obs,{})
        out['approach_target']=target
        if inputs.get('target_report'):
            out['placement_target']=json.loads(Path(inputs['target_report']).read_text())
        if action in ('trial_lift','transfer','lower','release','retreat','assess_support'):
            entry=ctx.value('piper_fk',joints=target['expected14'][:6],arm='left')
            current=final_fk or fk or ctx.value('piper_fk',joints=obs['measured14'][:6],arm='left')
            center=np.asarray(target['geometry']['cup_center_estimate'])+np.asarray(current['position'])-entry['position']
            if action=='retreat' and out.get('placement_target'):
                p=out['placement_target'];height=target['geometry']['cup_center_estimate'][2]-target['geometry']['table'][2]
                center=np.asarray(p['coaster_center'])+[0,0,height]
            out['cup_center_estimate']=center.tolist()
        if final_fk and fk:
            delta=np.asarray(final_fk['position'])-fk['position']
            out.update(measured_lift_m=float(delta[2]),measured_translation_m=float(np.linalg.norm(delta)),measured_arrival=True)
        if target14 is not None and fk and obs.get('finger_geometry',{}).get('fingers'):
            q=np.asarray(obs['measured14']);goal=np.asarray(target14);T=np.asarray(fk['T_base_flange'])
            local=[]
            for f in obs['finger_geometry']['fingers']:
                local.append((np.asarray(f['cad_corners_base_m'])-T[:3,3])@T[:3,:3])
            # This is a qualitative obstacle heuristic; the existing backend
            # still samples/checks the entire robot joint path at its own resolution.
            count=max(2,min(24,int(np.max(abs(q[:6]-goal[:6]))/.04)+2));samples=[];payload=[]
            for alpha in np.linspace(0,1,count):
                pose=fk if alpha==0 else ctx.value('piper_fk',joints=(q[:6]+alpha*(goal[:6]-q[:6])).tolist(),arm='left')
                transform=np.asarray(pose['T_base_flange']);parts=[]
                for i,points in enumerate(local):
                    # Installed parallel jaws open along flange X. Derive sign
                    # from the actual finger center; do not assume finger numbering.
                    points=points.copy();sign=np.sign(points[:,0].mean())
                    points[:,0]+=sign*alpha*(goal[6]-q[6])/2
                    parts.append(points@transform[:3,:3].T+transform[:3,3])
                samples.append(np.concatenate(parts).tolist())
                if action in ('trial_lift','transfer','lower') and 'cup_center_estimate' in out:
                    payload.append((np.asarray(out['cup_center_estimate'])+np.asarray(pose['position'])-fk['position']).tolist())
            out['finger_path_m']=samples
            out['payload_centers_m']=payload
            out['contact_entry']=action in ('grasp_close_segment','trial_lift','lower','release') or (action=='approach_segment' and target.get('phase')=='entry')
    except (ValueError,KeyError,OSError,TypeError) as exc:
        out['context_unavailable']=str(exc)
    return out


def modeled_obstacles(obs):
    """Add the measured work-table patch in the collision tool's dual-base frame.

    Left mount is URDF xyz=(0,.30,.008). Only the observed task-table extent is
    modeled; this does not claim a complete environment model.
    """
    try:
        target=_target(obs,{})
        z=float(target['geometry']['table'][2])
        if not -.01<z<.05:return []
        # The fixed 0.20m-wide mounting plate is an intended table fixture.
        # Model the task patch beyond its x=.10m edge; do not add a box through
        # the plate and then suppress the resulting legitimate FCL contact.
        return [{'name':'observed_task_table','center_m':[.255,-.1775+.30,z+.008-.025],
                 'size_m':[.29,.445,.05]}]
    except (ValueError,KeyError,OSError,TypeError):return []


def _image(obs, camera):
    image=cv2.imread(obs['cameras'][camera]['inspection_image'])
    if image is None:raise ValueError('Missing current '+camera+' image')
    return image


def _project(obs, points):
    c=obs['cameras']['head'];T=np.asarray(c['T_base_camera'],float)
    points=np.asarray(points,float).reshape(-1,3)
    camera=(points-T[:3,3])@T[:3,:3]
    if not np.isfinite(camera).all() or np.any(camera[:,2]<=0):raise ValueError('Invalid head projection')
    uv=camera@np.asarray(c['K'],float).T
    return uv[:,:2]/uv[:,2:3],camera[:,2]


def _target(obs, context):
    target=context.get('approach_target')
    if target is None:target=json.loads((OUT/'approach-target.json').read_text())
    # A consumed approach still supplies dimensions, never a motion permission.
    if (target['owner_generation']!=obs['state_binding']['owner_generation'] or
            target['geometry_identity']!=obs['finger_geometry']['geometry_identity'] or
            target['source_calibration']!=obs.get('calibration_sha256') or
            target['head_K']!=obs['cameras']['head']['K'] or
            target['head_T_base_camera']!=obs['cameras']['head']['T_base_camera']):
        raise ValueError('Task measurement belongs to a different owner/calibration')
    return target


def cup_radius(target):
    """Outer contour radius, not center-to-one-near-wall-sample distance."""
    if 'cup_radius_m' in target:return float(target['cup_radius_m'])
    from perception import _contours
    source=target['source_observation'];uv,z=_project(source,[target['geometry']['cup_center_estimate']])
    circles=_contours(_image(source,'head'));matches=circles[np.linalg.norm(circles[:,:2]-uv[0],axis=1)<12]
    if len(matches)!=1:raise ValueError('Source cup size missing/ambiguous')
    return float(matches[0,2]*z[0]/min(source['cameras']['head']['K'][0][0],source['cameras']['head']['K'][1][1]))


def rim_measurement(obs, target, center=None):
    """Search NEW pixels near a projected 3-D identity, allowing partial occlusion.

    Historical pixels are never queried for current depth. The circle is only a
    search prior; new edge support and new valid depth select the measurement.
    """
    g=target['geometry'];cup=np.asarray(g['cup_center_estimate'],float)
    center=np.asarray(center if center is not None else cup,float)
    radius=cup_radius(target)
    if not .020<radius<.065:raise ValueError('Cup radius outside characterized range')
    uv,_=_project(obs,[center,center+[radius,0,0]])
    cx,cy=uv[0];r=float(np.linalg.norm(uv[1]-uv[0]))
    im=_image(obs,'head');gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    edges=cv2.Canny(cv2.GaussianBlur(gray,(5,5),1),40,100)
    # A supported partial arc is enough; no requirement to see the hidden half.
    near=cv2.dilate(edges,np.ones((5,5),np.uint8))>0
    angles=np.linspace(0,2*np.pi,180,endpoint=False)
    candidates=[]
    for dx in range(-RIM_SHIFT_PX,RIM_SHIFT_PX+1,3):
        for dy in range(-RIM_SHIFT_PX,RIM_SHIFT_PX+1,3):
            x=np.rint(cx+dx+r*np.cos(angles)).astype(int)
            y=np.rint(cy+dy+r*np.sin(angles)).astype(int)
            inside=(x>=0)&(x<im.shape[1])&(y>=0)&(y<im.shape[0])
            if inside.mean()<.95:continue
            score=float(near[y[inside],x[inside]].mean())
            candidates.append((score,-math.hypot(dx,dy),cx+dx,cy+dy))
    if not candidates:raise ValueError('Cup outside current head image')
    score,_,cx,cy=max(candidates)
    if score<RIM_EDGE_FRACTION:raise ValueError('Current rim edge support insufficient/occluded')
    result={'center_px':[cx,cy],'radius_px':r,'edge_fraction':score,'radius_m':radius,
            'frame_id':obs['frame_id']}
    if obs['cameras']['head'].get('metric_depth'):
        xyz,valid=base_depth(obs);yy,xx=np.indices(valid.shape)
        ring=(abs(np.hypot(xx-cx,yy-cy)-r)<4)&near&valid
        ring &= abs(xyz[:,:,2]-center[2])<.025
        z=xyz[:,:,2][ring]
        spread=float(np.quantile(z,.9)-np.quantile(z,.1)) if len(z) else None
        result['depth_quality']={'samples':len(z),'p90_p10_m':spread,'minimum_samples':25,
            'maximum_spread_m':.014,'usable':len(z)>=25 and spread<.014}
        if result['depth_quality']['usable']:
            rim_z=float(np.median(z));cam=obs['cameras']['head'];T=np.asarray(cam['T_base_camera'])
            ray=T[:3,:3]@np.linalg.solve(np.asarray(cam['K']),[cx,cy,1])
            if abs(ray[2])>1e-6:
                result.update(center_base_m=(T[:3,3]+ray*((rim_z-T[2,3])/ray[2])).tolist(),
                              rim_z_m=rim_z,depth_samples=len(z))
    return result


def wrist_retention(before, after):
    """Forward/backward optical flow on blue cup pixels, excluding orange pads.

    The moving wrist needs no hand-eye for *relative* appearance. Textureless,
    lost, ambiguous or substantially slipping tracks return unknown, never held.
    """
    a=_image(before,'left');b=_image(after,'left')
    if a.shape!=b.shape:raise ValueError('Wrist image shape changed')
    hsv=cv2.cvtColor(a,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([80,18,35]),np.array([115,210,245]))
    h,w=mask.shape;mask[:int(h*.18)]=0;mask[:, :int(w*.22)]=0;mask[:,int(w*.92):]=0
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    ids=[i for i in range(1,n) if stats[i,cv2.CC_STAT_AREA]>800]
    if len(ids)!=1:raise ValueError('Wrist cup colour region missing/ambiguous')
    mask=(labels==ids[0]).astype('uint8')*255
    aa=cv2.cvtColor(a,cv2.COLOR_BGR2GRAY);bb=cv2.cvtColor(b,cv2.COLOR_BGR2GRAY)
    points=cv2.goodFeaturesToTrack(aa,100,.01,7,mask=mask)
    if points is None or len(points)<TRACK_MIN_POINTS:raise ValueError('Cup texture insufficient for wrist tracking')
    new,ok,_=cv2.calcOpticalFlowPyrLK(aa,bb,points,None)
    if new is None:raise ValueError('Cup track lost')
    back,ok2,_=cv2.calcOpticalFlowPyrLK(bb,aa,new,None)
    if back is None:raise ValueError('Cup reverse track lost')
    good=(ok[:,0]>0)&(ok2[:,0]>0)&(np.linalg.norm(back[:,0]-points[:,0],axis=1)<1.5)
    p=points[:,0][good];q=new[:,0][good]
    if len(p)<TRACK_MIN_POINTS or len(p)<.6*len(points) or np.min(np.ptp(p,axis=0))<20:
        raise ValueError('Cup track support insufficient')
    shift=np.linalg.norm(q-p,axis=1)
    # Re-check the current colour membership; a track onto the finger/background
    # is not positive cup evidence even if optical flow reports a match.
    current=cv2.cvtColor(b,cv2.COLOR_BGR2HSV);x,y=np.rint(q).astype(int).T
    if np.any(x<0)|np.any(y<0)|np.any(x>=w)|np.any(y>=h):raise ValueError('Cup track left view')
    blue=(current[y,x,0]>=80)&(current[y,x,0]<=115)&(current[y,x,1]>=18)
    if blue.mean()<.8 or float(np.quantile(shift,.8))>TRACK_MAX_SHIFT_PX:
        raise ValueError('Cup moved relative to pads or appearance changed')
    return {'tracked_points':len(p),'relative_shift_p80_px':float(np.quantile(shift,.8)),
            'before_frame':before['frame_id'],'current_frame':after['frame_id'],
            'basis':'current_wrist_cup_tracks_relative_to_camera'}



def _visible_pads(im):
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);h,w=im.shape[:2]
    orange=cv2.inRange(hsv,np.array([5,90,40]),np.array([30,255,255]))
    n,labels,stats,centers=cv2.connectedComponentsWithStats(orange,8)
    pads=[i for i in range(1,n) if stats[i,4]>=300 and centers[i,1]>.40*h]
    if len(pads)!=2:raise ValueError('Two visible pad regions unresolved')
    pads.sort(key=lambda i:centers[i,0])
    return centers[pads],[(labels==i).astype('uint8') for i in pads],stats[pads]


def _pale_cup_region(im):
    # This task's pale blue cup is weakly saturated; the old HSV saturation gate
    # confused it with the table. Use broad blue/green excess over red instead.
    b,g,r=[im[:,:,i].astype('int16') for i in range(3)]
    mask=((b-r>12)&(g-r>5)).astype('uint8')
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
    n,labels,stats,centers=cv2.connectedComponentsWithStats(mask,8)
    if n<2:raise ValueError('Pale cup region absent')
    i=1+int(np.argmax(stats[1:,4]));area=int(stats[i,4])
    if area<.01*im.shape[0]*im.shape[1]:raise ValueError('Pale cup region too small')
    return dict(area=area,center=centers[i].tolist(),box=stats[i,:4].tolist())


def wrist_shape_retention(before, after):
    """Coarse cup presence/position comparison for a measured trial lift.

    Partial pad occlusion and small cup seating shifts are normal. No exact
    contours, texture correspondence, or identical visible pad centroids required.
    """
    if (before['frame_id']==after['frame_id'] or
            after['cameras']['left']['source_timestamp_ns']<=before['cameras']['left']['source_timestamp_ns']):
        raise ValueError('Distinct chronological wrist observations required')
    images=[_image(o,'left') for o in (before,after)]
    if images[0].shape!=images[1].shape:raise ValueError('Wrist image shape changed')
    pads=[_visible_pads(im) for im in images]
    overlaps=[]
    for a,b in zip(pads[0][1],pads[1][1]):
        overlaps.append(max(float(cv2.dilate(b,np.ones((7,7),np.uint8))[a>0].mean()),
                            float(cv2.dilate(a,np.ones((7,7),np.uint8))[b>0].mean())))
    if min(overlaps)<.6:raise ValueError('Pad reference changed substantially')
    centers,_,stats=pads[0];h,w=images[0].shape[:2]
    x0=max(0,int(centers[0,0]-.12*w));x1=min(w,int(centers[1,0]+.25*w))
    y0=max(0,int(min(stats[:,1])-.45*h));y1=min(h,int(max(stats[:,1])+.20*h))
    regions=[_pale_cup_region(im[y0:y1,x0:x1]) for im in images]
    ratio=regions[1]['area']/regions[0]['area']
    shift=math.dist(regions[0]['center'],regions[1]['center'])
    span=float(centers[1,0]-centers[0,0])
    if not .35<=ratio<=2.8 or shift>.4*span:raise ValueError('Cup region lost or moved substantially relative to pads')
    return dict(basis='coarse pale cup presence near same pads; combine with measured lift and any available head evidence',
        visible_area_ratio=ratio,cup_region_shift_px=shift,pad_overlap=overlaps,
        roi_xyxy=[x0,y0,x1,y1],before_frame=before['frame_id'],current_frame=after['frame_id'])


def placement_execution_evidence(release, retreat, target):
    """Existing measured receipts and reached placement pose, not a support sensor."""
    a=release.get('receipt',{});b=retreat.get('receipt',{})
    position=release.get('before_fk',{}).get('position')
    valid=bool(release.get('ok') and release.get('function')=='release' and retreat.get('ok')
        and position and all(r.get('executed') is True and r.get('measured_arrival') is True
            and r.get('measured_settled') is True and r.get('owner_generation')==target.get('owner_generation')
            for r in (a,b))
        and abs(position[2]-target['predicted_support_flange_z'])<=.002
        and math.dist(position[:2],target['transfer_goal'][:2])<=.002
        and a.get('final14',[0]*14)[6]>=.010 and b.get('final14',[0]*14)[6]>=.010)
    return valid


def wrist_placement(obs):
    """Qualitative visible cup after release/withdrawal; no hidden support proof."""
    im=_image(obs,'left');h,w=im.shape[:2];centers,_,_= _visible_pads(im)
    # Exclude orange pads and favour the broad pale cup region above them.
    region=_pale_cup_region(im[:int(.9*h),int(.25*w):])
    center=np.asarray(region['center'])+[.25*w,0]
    grey=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    circles=cv2.HoughCircles(cv2.GaussianBlur(grey,(9,9),2),cv2.HOUGH_GRADIENT,1.5,.1*w,
        param1=40,param2=40,minRadius=int(.10*w),maxRadius=int(.24*w))
    candidates=[] if circles is None else [c for c in circles[0]
        if math.dist(c[:2],center)<c[2] and .2*w<c[0]<.95*w and c[1]<.85*h]
    if not candidates:raise ValueError('No broad visible cup rim after retreat')
    circle=min(candidates,key=lambda c:math.dist(c[:2],center))
    pad_center=centers.mean(axis=0);span=centers[1,0]-centers[0,0]
    if np.linalg.norm(center-pad_center)<.5*span:raise ValueError('Cup still at pad contact region')
    return dict(basis='visible pale cup and broad rim remain away from opened pads after measured placement/retreat',
        cup_region=region,cup_center_px=center.tolist(),rim_circle_px=circle.tolist(),
        scope='qualitative likely placement; no precise centering, force or hidden support-height claim')


def coaster_measurement(obs, placement):
    from perception import _contours
    circles=_contours(_image(obs,'head'));point=np.asarray(placement['coaster_pixel'])
    candidates=circles[np.linalg.norm(circles[:,:2]-point,axis=1)<8]
    if len(candidates)!=1:raise ValueError('Current coaster association missing/ambiguous')
    return {'current_circle':candidates[0].tolist(),'frame_id':obs['frame_id']}


def view_timing(obs):
    """Qualitative views need one unchanged rest boundary, not hardware sync."""
    stamps=[obs['cameras'][k]['source_timestamp_ns'] for k in ('head','left')]
    skew=abs(stamps[0]-stamps[1])/1e9
    if skew<=.25:return {'skew_s':skew,'basis':'closely_timed_views'}
    b=obs.get('state_binding',{});after=obs.get('after_ns');end=obs.get('captured_at_ns')
    if (type(after) is not int or type(end) is not int or
            not b.get('rest_consistent') or not b.get('owner_unchanged') or
            not observation_pose_matches(b.get('before14'),b.get('after14')) or
            not observation_pose_matches(b.get('after14'),obs.get('measured14')) or
            not all(after<=s<=end and end-s<1_000_000_000 for s in stamps)):
        raise ValueError('Head/wrist views do not share a fresh unchanged rest boundary')
    return {'skew_s':skew,'basis':'fresh_views_inside_same_verified_rest',
            'after_ns':after,'end_ns':end,'pixel_fusion':False}


def scene_change(obs,target,context=None):
    """Gross changes in this run's fixed-head view, allowing the known moving arm.

    This corroborates a checked bounded empty path. It is not an occupancy map
    or permission to ignore a depth obstacle. The saved image is a comparison
    reference only; its pixels are never queried as a new measurement.
    """
    source=target['source_observation'];before=_image(source,'head');now=_image(obs,'head')
    if before.shape!=now.shape:raise ValueError('Head comparison shape changed')
    oldgray=cv2.cvtColor(before,cv2.COLOR_BGR2GRAY);gray=cv2.cvtColor(now,cv2.COLOR_BGR2GRAY)
    shift=float(np.median(gray.astype(float)-oldgray))
    change=np.max(abs(now.astype(float)-before.astype(float)-shift),axis=2)>30
    changed=cv2.morphologyEx(change.astype('uint8'),cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    changed=cv2.dilate(changed,np.ones((7,7),np.uint8))
    seeds=np.zeros(gray.shape,np.uint8)
    for view in (source,obs):
        for finger in view.get('finger_geometry',{}).get('fingers',[]):
            uv,_=_project(obs,finger['cad_corners_base_m'])
            hull=np.rint(cv2.convexHull(uv.astype('float32'))).astype('int32')
            cv2.fillConvexPoly(seeds,hull,1)
    seeds=cv2.dilate(seeds,np.ones((17,17),np.uint8))>0
    relevant=np.ones(gray.shape,bool)
    path=(context or {}).get('finger_path_m')
    if path:
        relevant=np.zeros(gray.shape,np.uint8)
        for sample in path:
            uv,_=_project(obs,sample)
            cv2.fillConvexPoly(relevant,np.rint(cv2.convexHull(uv.astype('float32'))).astype('int32'),1)
        for key in ('cup_center_estimate','coaster_center'):
            if key in target['geometry']:
                uv,z=_project(obs,[target['geometry'][key]])
                radius=round(.065*obs['cameras']['head']['K'][0][0]/z[0])
                cv2.circle(relevant,tuple(np.rint(uv[0]).astype(int)),radius,1,-1)
        relevant=cv2.dilate(relevant,np.ones((31,31),np.uint8))>0
    n,labels,stats,_=cv2.connectedComponentsWithStats(changed,8)
    unexpected=[];robot_pixels=0
    for i in range(1,n):
        area=int(stats[i,4]);region=labels==i
        evidence_area=int((region&change&relevant).sum())
        # Dilation only joins related pixels; it must not magnify a few glare
        # pixels into an apparent new object.
        if evidence_area<200:continue
        dark=float(((gray<85)|(oldgray<85))[region].mean())
        expected=bool(np.any(region&seeds) and dark>.20 and area<.35*gray.size)
        if expected:robot_pixels+=area
        else:unexpected.append({'area_px':evidence_area,'bbox_xywh':stats[i,:4].tolist()})
    if unexpected:raise ValueError('New visible scene regions outside expected arm: '+json.dumps(unexpected[:4]))
    return {'basis':'current_head_compared_with_same_run_scene','expected_arm_change_px':robot_pixels,
            'unexpected_regions':0,'reference_frame':source['frame_id'],'current_frame':obs['frame_id'],
            'hidden_clearance_proven':False}


def wrist_cup_relation(obs, *, require_between=True):
    """Current LEFT RGB complements head occlusion at the characterized entry.

    Find both orange pads and inspect the phase-appropriate visible cup region.
    This is cup-region/foreign-object evidence, not inside/outside pad topology,
    a metric clearance, support, or a whole-arm swept-volume proof.
    """
    timing=view_timing(obs);skew=timing['skew_s']
    image=_image(obs,'left');h,w=image.shape[:2];hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    orange=cv2.inRange(hsv,np.array([5,90,40]),np.array([30,255,255]))
    n,labels,stats,centers=cv2.connectedComponentsWithStats(orange,8)
    pads=[i for i in range(1,n) if stats[i,4]>=300 and centers[i,1]>.40*h]
    if len(pads)!=2:raise ValueError('Both left orange pads not uniquely visible')
    pads.sort(key=lambda i:centers[i,0]);a,b=(stats[i] for i in pads)
    if min(a[4],b[4])/max(a[4],b[4])<.1:raise ValueError('Pad segmentation too asymmetric')
    x0=int(a[0]+a[2]+8);x1=int(b[0]-8)
    y0=int(max(a[1],b[1])+10);y1=int(min(a[1]+a[3],b[1]+b[3])-8)
    if not require_between:
        # At hover the cup is AHEAD of the fingertips, not yet between them.
        # This checks target visibility, never claims readiness to close.
        x0=max(0,int(a[0]-.10*w));x1=min(w,int(b[0]+b[2]+.10*w))
        y1=min(h,int(max(a[1],b[1])+.08*h));y0=max(0,y1-int(.60*h))
    if x1-x0<20 or y1-y0<20:raise ValueError('Visible between-pad band too small')
    band=hsv[y0:y1,x0:x1]
    blue=(band[:,:,0]>=80)&(band[:,:,0]<=115)&(band[:,:,1]>=18)&(band[:,:,2]>=35)
    foreign=((band[:,:,1]>65)&~blue)|(band[:,:,2]<35)
    _,_,components,_=cv2.connectedComponentsWithStats(foreign.astype('uint8'),8)
    largest=int(components[1:,cv2.CC_STAT_AREA].max(initial=0))
    if float(blue.mean())<(.20 if require_between else .08) or largest>max(128,int(.05*blue.size)):
        raise ValueError(f'Wrist target band unresolved/foreign region: blue={blue.mean():.3f}, foreign_component={largest}')
    return {'frame_id':obs['frame_id'],'camera':'left','pad_boxes':[a[:4].tolist(),b[:4].tolist()],
            'between_pad_roi_xyxy':[x0,y0,x1,y1],'blue_fraction':float(blue.mean()),
            'largest_foreign_component_px':largest,'head_wrist_skew_s':skew,
            'view_timing':timing,
            'relation_kind':'between_pads' if require_between else 'target_ahead_of_pads',
            'basis':('blue cup region visible between two orange pads' if require_between else
                     'blue cup region visible ahead of the pads')+'; no obvious foreign region',
            'metric_clearance_proven':False,'handeye_used':False}


def wrist_depth_relation(obs, relation):
    """Valid FFS samples in optical coordinates only; never a base-frame target."""
    cam=obs['cameras']['left']
    if (cam.get('metric_depth') is not True or cam.get('base_transform_available') is not False or
            cam.get('camera_frame')!='left_d435_color_optical_frame'):
        raise ValueError('Wrist depth must use its optical-only contract')
    with np.load(Path(cam['inspection_image']).with_name('left-depth.npz'),allow_pickle=False) as data:
        xyz=data['xyz_rgb_m'];valid=data['valid'].astype(bool)
    image=_image(obs,'left')
    if xyz.shape!=image.shape or valid.shape!=image.shape[:2]:raise ValueError('Wrist depth/RGB shape mismatch')
    valid&=np.isfinite(xyz).all(axis=2)&(xyz[:,:,2]>.08)&(xyz[:,:,2]<1.)
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    orange=cv2.inRange(hsv,np.array([5,90,40]),np.array([30,255,255]))>0
    blue=(hsv[:,:,0]>=80)&(hsv[:,:,0]<=115)&(hsv[:,:,1]>=18)&(hsv[:,:,2]>=35)
    masks=[]
    for x,y,w,h in relation['pad_boxes']:
        mask=np.zeros(valid.shape,bool);mask[y:y+h,x:x+w]=True;masks.append(mask&orange)
    x0,y0,x1,y1=relation['between_pad_roi_xyxy'];mask=np.zeros(valid.shape,bool)
    mask[y0:y1,x0:x1]=True;masks.append(mask&blue)
    rows=[]
    for label,mask in zip(('pad_1','pad_2','cup_band'),masks):
        points=xyz[valid&mask]
        if len(points)<25:
            if label=='cup_band':raise ValueError('Insufficient valid wrist depth on '+label)
            rows.append({'region':label,'valid_samples':len(points),'available':False})
            continue
        spread=float(np.quantile(points[:,2],.9)-np.quantile(points[:,2],.1))
        # A cup is not a plane: rim, wall and bottom legitimately differ in Z.
        # Keep the measured spread, never turn mixed surfaces into a metric gap.
        rows.append({'region':label,'valid_samples':len(points),'available':True,
                     'optical_median_xyz_m':np.median(points,axis=0).tolist(),
                     'optical_z_spread_m':spread,'compact_surface':spread<=.04})
    cup_z=rows[2]['optical_median_xyz_m'][2]
    delta=[cup_z-row['optical_median_xyz_m'][2] if row['available'] else None for row in rows[:2]]
    comparable=(relation.get('relation_kind')=='between_pads' and
                all(row.get('compact_surface') for row in rows))
    return {'frame_id':obs['frame_id'],'coordinate_frame':'left_d435_color_optical_frame',
            'regions':rows,'cup_minus_pad_optical_z_m':delta,
            'local_depth_consistent':max(abs(d) for d in delta)<=.08 if comparable else None,
            'comparison_available':comparable,
            'scope':'local surface proximity diagnostic; no base-frame transform/clearance/contact proof',
            'base_transform_applied':False,'motion_target_created':False}


def wrist_depth_needed(obs, required, context, metrics):
    if not required or not metrics.get('wrist_relation'):return False
    if obs['cameras'].get('left',{}).get('metric_depth'):return False
    if not context.get('robot',{}).get('capabilities',{}).get('wrist_metric_depth'):return False
    relevant={'contact_scene','grasp_setup','cup_lifted_and_held','cup_retained','placement_scene','cup_supported','release_ready','lowering_result','retreat_ready','route_visible_and_unobstructed'}
    close=context.get('contact_entry') or context.get('action') in ('trial_lift','transfer','lower','release','retreat','assess_support')
    head_insufficient=('rim' in metrics.get('unavailable',{}) or 'rim_z_m' not in metrics.get('rim',{}) or
                       (metrics.get('route') is not None and not metrics['route']['visibility_sufficient']))
    return bool(close and relevant&required.keys() and head_insufficient)


def observe_wrist_depth(ctx, obs, context):
    """One on-demand FFS capture, all bindings checked; no motion or retry loop."""
    fresh=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=True,depth_cameras=['head','left'])
    robot=ctx.value('piper_status');old=context['robot']
    if (not observation_recent(fresh) or not fresh['state_binding'].get('rest_consistent') or
            not robot.get('motion_ready') or robot.get('control_uncertain') or
            not observation_pose_matches(obs['measured14'],fresh['measured14']) or
            not observation_pose_matches(fresh['measured14'],robot['measured_state']['values']) or
            fresh['state_binding']['owner_generation']!=old['owner_status']['generation'] or
            robot['owner_status']['generation']!=old['owner_status']['generation'] or
            robot['bridge']['instance']!=old['bridge']['instance'] or robot.get('last_motion')!=old.get('last_motion') or
            fresh.get('calibration_sha256')!=obs.get('calibration_sha256') or
            fresh['finger_geometry']['geometry_identity']!=obs['finger_geometry']['geometry_identity']):
        raise ValueError('State/owner/scene binding changed during wrist depth capture')
    if not fresh['cameras']['left'].get('metric_depth'):raise ValueError('Requested wrist depth was not returned')
    return fresh,robot


def route_measurement(obs, context, target):
    """Coarse current-depth task-region obstacle heuristic, alongside CAD path.

    Samples contain installed finger corners along the actual planned joint path.
    Occluded/invalid depth is not free space. This checks the visible hand route;
    the backend remains responsible for full-robot CAD path checks.
    """
    samples=np.asarray(context['finger_path_m'],float)
    if samples.ndim!=3 or samples.shape[1:]!=(16,3) or len(samples)<2 or not np.isfinite(samples).all():
        raise ValueError('Missing sampled installed-finger path')
    table=float(target['geometry']['table'][2]);minimum=float(samples[:,:,2].min())
    if minimum<table+.008:raise ValueError('Finger/table nominal margin insufficient')
    xyz,valid=base_depth(obs);cam=obs['cameras']['head'];depth=cam.get('metric_depth')
    if not depth:raise ValueError('Current route depth unavailable')
    # Only mask the installed current fingers, not arbitrary dark pixels/robot
    # silhouettes. Other foreground objects and missing rays remain unresolved.
    own=np.zeros(valid.shape,bool)
    for finger in obs['finger_geometry']['fingers']:
        p=np.asarray(finger['cad_corners_base_m']);own|=np.all((xyz>=p.min(0)-.008)&(xyz<=p.max(0)+.008),axis=2)
    original=np.asarray(target['geometry']['cup_center_estimate']);radius=cup_radius(target)
    center=np.asarray(context.get('cup_center_estimate',original))
    intended=context.get('contact_entry',False)
    cup=(np.linalg.norm(xyz[:,:,:2]-center[:2],axis=2)<radius+.008)&(xyz[:,:,2]<center[2]+.012)&(xyz[:,:,2]>table)
    payload=context.get('payload_centers_m',[])
    volumes=[[f for f in np.split(sample,2)] for sample in samples]
    if payload:
        from itertools import product
        height=float(original[2]-table)
        for row,position in zip(volumes,payload):
            p=np.asarray(position);bottom=p[2]-height
            if bottom<table-.005:raise ValueError('Predicted payload passes below table')
            row.append(np.array(list(product([p[0]-radius,p[0]+radius],[p[1]-radius,p[1]+radius],[bottom,p[2]]))))
        own|=cup  # the currently tracked payload is not a new obstacle to itself
    hits=0;ray_checks=0;seen=0;foreground=np.zeros(valid.shape,bool)
    camera_T=np.asarray(cam['T_base_camera']);camera_xyz=(xyz-camera_T[:3,3])@camera_T[:3,:3]
    for row in volumes:
        for finger in row:
            lo=finger.min(0)-.004;hi=finger.max(0)+.004
            occupied=valid&~own&np.all((xyz>=lo)&(xyz<=hi),axis=2)
            if intended:occupied&=~cup
            hits+=int(occupied.sum())
            foreground|=occupied
            # Visible returns behind the sampled volume support empty rays.
            uv,z=_project(obs,finger);poly=np.rint(cv2.convexHull(uv.astype('float32'))).astype('int32')
            region=np.zeros(valid.shape,np.uint8);cv2.fillConvexPoly(region,poly,1)
            region=region.astype(bool)&~own
            if intended:region&=~cup
            ray_checks+=int(region.sum());seen+=int((region&valid&(camera_xyz[:,:,2]>z.max()+.005)).sum())
    coverage=seen/max(1,ray_checks)
    _,_,components,_=cv2.connectedComponentsWithStats(foreground.astype('uint8'),8)
    largest=int(components[1:,cv2.CC_STAT_AREA].max(initial=0))
    if largest>=40:raise ValueError(f'Route foreground cluster unresolved: largest={largest}, unique={int(foreground.sum())}, repeated={hits}')
    return {'nominal_table_margin_m':minimum-table,'foreground_samples':hits,
            'unique_foreground_px':int(foreground.sum()),'largest_foreground_cluster_px':largest,
            'visible_free_fraction':coverage,'path_samples':len(samples),
            'visibility_sufficient':coverage>=.30,
            'payload_included':bool(payload),
            'scope':'qualitative visible hand/payload route; not full environment certification'}


def ordinary_rules(obs, prior, required, context):
    """Return only facts actually supported by code; all others need fallback."""
    facts={};metrics={};errors={};support=None
    def measure(name,fn):
        if name not in metrics and name not in errors:
            try:metrics[name]=fn()
            except (ValueError,KeyError,OSError,TypeError,IndexError,cv2.error,np.linalg.LinAlgError) as exc:errors[name]=str(exc)
        return metrics.get(name)
    target=measure('target',lambda:_target(obs,context))
    if target is None:return facts,{'unavailable':errors},support
    if (set(required) and set(required)<={'placement_stable','released_on_coaster'}
            and context.get('placement_executed') and float(obs['measured14'][6])>=.010):
        final=measure('wrist_placement',lambda:wrist_placement(obs))
        if final:
            for name in required:
                facts[name]={'state':'supported','source':'ordinary_code','cameras':['left'],
                    'evidence':'Measured placement/release/retreat completed; current wrist cup appearance is qualitatively consistent with placement. No exact support-height claim.'}
            metrics.pop('target',None);metrics['unavailable']=errors
            return facts,metrics,None
    center=context.get('cup_center_estimate')
    rim=measure('rim',lambda:rim_measurement(obs,target,center))
    table=target['geometry']['table'][2]
    original_height=target['geometry']['cup_center_estimate'][2]-table
    upright=bool(rim and 'rim_z_m' in rim and abs(rim['rim_z_m']-(center or target['geometry']['cup_center_estimate'])[2])<.015)
    on_table=bool(upright and abs(rim['rim_z_m']-target['geometry']['cup_center_estimate'][2])<.010)
    # Before contact the retained 3-D measurement is checked against the current
    # visible contour. Millimetre-quality rim depth is unnecessary for opening an
    # empty jaw or changing its orientation. It remains necessary for support.
    empty_phase=(target.get('valid') and not target.get('contact_started') and
                 target.get('phase') in ('preparation','preparation_verified','hover','hover_verified','entry','entry_verified'))
    current_identity=bool(rim and empty_phase)
    if rim and 'rim_z_m' in rim and abs(rim['rim_z_m']-target['geometry']['cup_center_estimate'][2])>.025:
        current_identity=False
    scene=measure('scene_change',lambda:scene_change(obs,target,context)) if current_identity else None
    qualitative_scene=bool(current_identity and scene)
    width=float(obs['measured14'][6]);tcp=np.asarray(obs['finger_geometry'].get('effective_contact_tcp',{}).get('position',[math.nan]*3))
    source=np.asarray(target['geometry']['cup_center_estimate'])
    empty=bool((on_table or qualitative_scene) and (np.linalg.norm(tcp-source)>.08 or
               (empty_phase and context.get('action') in ('preparation_open','preparation_segment','approach_segment'))))
    route_names={'opening_sides_visible_and_unobstructed','route_visible_and_unobstructed','lift_route','transfer_ready','lowering_ready','retreat_ready','contact_scene','approach_scene','grasp_setup','release_ready'}
    route=measure('route',lambda:route_measurement(obs,context,target)) if route_names&required.keys() else None
    partial_route=route
    if route and not route['visibility_sufficient']:
        errors['route_visibility']='Head depth alone does not cover the visible route sufficiently';route=None
    if (not route and partial_route and qualitative_scene and context.get('modeled_path_clear') and
            context.get('action') in ('preparation_open','preparation_segment','approach_segment') and
            not context.get('contact_entry')):
        route={**partial_route,'basis':'checked_empty_path_and_current_qualitative_scene',
               'scope':'bounded empty approach before contact; not a complete environment certificate'}
        metrics['combined_empty_rule']=route
    local_names={'contact_scene','approach_scene','grasp_setup','route_visible_and_unobstructed'}
    wrist_names=local_names|{'cup_retained','placement_scene','cup_supported','release_ready','lowering_result'}
    needs_between=context.get('action')!='approach_segment'
    near=context.get('contact_entry') or context.get('action') in ('trial_lift','transfer','lower','release','retreat','assess_support','grasp_close_segment')
    wrist=measure('wrist_relation',lambda:wrist_cup_relation(obs,require_between=needs_between)) if near and wrist_names&required.keys() else None
    wrist_depth=measure('wrist_depth',lambda:wrist_depth_relation(obs,wrist)) if wrist and obs['cameras']['left'].get('metric_depth') else None
    if wrist_depth and wrist_depth['local_depth_consistent'] is False:
        errors['wrist_depth']='Optical cup/pad depths differ substantially; qualitative fallback required'
        wrist=None
    # A close, characterized entry has a different condition from empty flight:
    # expected cup occlusion is allowed. Combine modeled path/table clearance,
    # head target identity and the current wrist target band. Never use this to
    # excuse observed foreground, arbitrary transit, or unbound old targets.
    entry_bound=(context.get('action') in ('approach_segment','grasp_close_segment') and
                 target.get('valid') and target.get('phase') in ('entry','entry_verified') and
                 not target.get('contact_started') and context.get('contact_entry') and
                 observation_pose_matches(target.get('expected14'),obs.get('measured14')) and
                 target.get('last_motion')==context.get('robot',{}).get('last_motion'))
    if not route and partial_route and wrist and (on_table or qualitative_scene) and entry_bound and context.get('modeled_path_clear'):
        route={**partial_route,'basis':'modeled_path_and_current_head_wrist_entry_relation',
               'scope':'characterized local cup entry only; no hidden-clearance claim'}
        metrics['combined_entry_rule']=route
    held_names={'cup_lifted_and_held','cup_retained','transfer_ready','lowering_result','lowering_ready'}
    retained=None
    if prior and held_names&required.keys() and 'cup_lifted_and_held' not in required:
        retained=measure('retention',lambda:wrist_retention(prior,obs))
    # Qualitative outcome: evaluate wrist evidence independently. Missing precise
    # head depth is not a failed grasp. A reliable contradictory head rise still
    # prevents ordinary acceptance; measured motion alone is never pickup proof.
    combined_lift=False;head_contradiction=False
    if prior and 'cup_lifted_and_held' in required and context.get('measured_arrival'):
        lift=context.get('measured_lift_m',0)
        head_rise=(rim['rim_z_m']-target['geometry']['cup_center_estimate'][2]) if rim and 'rim_z_m' in rim else None
        head_contradiction=bool(head_rise is not None and not .25*lift<=head_rise<=2*lift)
        metrics['pickup_evidence_gate']={'measured_lift_m':lift,'observed_head_rise_m':head_rise,
            'head_evidence':'contradictory' if head_contradiction else 'unavailable' if head_rise is None else 'consistent',
            'wrist_evaluation':'not_run_motion_or_width_unconfirmed'}
        if .030<=lift<=.065 and .002<=width<=.006:
            shape=measure('wrist_shape_retention',lambda:wrist_shape_retention(prior,obs))
            metrics['pickup_evidence_gate']['wrist_evaluation']='supported' if shape else 'unresolved'
            if shape and not head_contradiction:
                combined_lift=True
                metrics['combined_lift']={'observed_head_rim_rise_m':head_rise,'measured_lift_m':lift,
                    'scope':'qualitative likely pickup from measured lift and retained wrist cup appearance; head depth optional, contradictions not ignored'}
    if prior and 'cup_lifted_and_held' in required and not combined_lift and not head_contradiction:
        retained=measure('retention',lambda:wrist_retention(prior,obs))
    placement_names={'placement_scene','cup_supported','release_ready','released_on_coaster','placement_stable','retreat_ready','lowering_result'}
    placement=context.get('placement_target')
    coaster=measure('coaster',lambda:coaster_measurement(obs,placement)) if placement and 'coaster_unchanged' in required else None
    aligned=False
    if placement and rim and 'center_base_m' in rim:
        coaster=np.asarray(placement['coaster_center']);cup=np.asarray(rim['center_base_m'])
        aligned=bool(np.linalg.norm(cup[:2]-coaster[:2])<=.018)
        gap=float(rim['rim_z_m']-original_height-coaster[2]);metrics['estimated_base_gap_m']=gap
        if aligned and abs(gap)<=SUPPORT_BAND_M:support='resting'
        elif aligned and gap>SUPPORT_BAND_M:support='suspended'
    final_placement=None
    if ('placement_stable' in required or 'released_on_coaster' in required) and context.get('placement_executed') and width>=.010:
        final_placement=measure('wrist_placement',lambda:wrist_placement(obs))
    for name in required:
        reason=None
        if name in ('target_scene_consistent','cup_upright') and (on_table or qualitative_scene):reason='Current cup contour and scene agree with this run\'s retained geometry; no precision rim-depth requirement before contact'
        elif name=='coaster_unchanged' and coaster:reason='Current contour agrees with this run\'s coaster identity'
        elif name=='jaw_empty' and empty:reason='Current cup remains at table height and open/away jaw has not picked it up'
        elif name in ('opening_sides_visible_and_unobstructed','route_visible_and_unobstructed','lift_route') and route:reason=route.get('basis','Sampled installed-finger route plus current visible depth')
        elif (name in ('contact_scene','approach_scene') and qualitative_scene and route and
              context.get('action')=='approach_segment' and not context.get('contact_entry')):
            reason='Empty hover remains above the measured rim on the checked path; no demand that cup is already between pads'
        elif name in ('contact_scene','approach_scene','grasp_setup') and (on_table or qualitative_scene) and route and wrist:reason='Current head cup geometry and wrist target relation agree with this approach phase; hidden pad topology not required'
        elif name=='cup_lifted_and_held' and combined_lift:reason='Measured trial lift completed and cup remained broadly near the same wrist pads; available head evidence has no clear contradiction'
        elif name=='cup_lifted_and_held' and retained and context.get('measured_lift_m',0)>=.030:reason='Cup tracks remain with moving wrist after measured upward motion'
        elif name=='cup_retained' and retained and context.get('measured_translation_m',0)>=.005:reason='Cup tracks remain with moving wrist after measured transport'
        elif name=='transfer_ready' and route and context.get('held_at_boundary'):reason='Freshly rebound held result and current route rule'
        elif name=='lowering_ready' and aligned and route and context.get('held_at_boundary'):reason='Current cup/coaster alignment, held boundary and short route'
        elif name=='lowering_result' and aligned and (retained or support=='resting'):reason='Current rim/coaster depth and relative cup tracking'
        elif name=='placement_scene' and aligned and support:reason='Current rim depth and coarse cup/coaster alignment'
        elif name=='cup_supported' and support=='resting':reason='Current depth suggests cup base at coaster height; qualitative support estimate'
        elif name=='release_ready' and support=='resting' and route:reason='Qualitative current support estimate and checked opening route'
        elif name in ('released_on_coaster','placement_stable') and final_placement:reason='Measured placement, release and retreat completed; current wrist shows the cup remaining away from open pads, qualitatively consistent with placement'
        elif name in ('released_on_coaster','placement_stable') and support=='resting' and context.get('measured_arrival') and width>=.010:reason='Current cup still at coaster height after measured opening/retreat'
        elif name=='retreat_ready' and support=='resting' and width>=.010 and route:reason='Current cup/coaster relationship and released-jaw retreat path'
        if reason:facts[name]={'state':'supported','evidence':reason,'cameras':['head','left'],'source':'ordinary_code'}
    # Raw targets/images are already saved separately; keep the decision report small.
    metrics.pop('target',None);metrics['unavailable']=errors
    return facts,metrics,support


def assess(ctx, obs, question, *, required_facts, prior=None, context=None,
           fallback, cameras=('head','left'), onsite=None):
    """At most one Gemini request for unresolved facts, none on normal rule success."""
    start=time.monotonic();context=context or {}
    def diagnostic(stage,metrics,unresolved=None):
        OUT.mkdir(parents=True,exist_ok=True)
        path=OUT/('condition-'+str(time.time_ns())+'.json')
        path.write_text(json.dumps({'stage':stage,'frame_id':obs['frame_id'],
            'action':context.get('action'),'rule_metrics':metrics,
            'required_facts':required_facts,'unresolved_facts':unresolved},indent=2))
        return str(path)
    if not observation_recent(obs,20):raise ValueError('Condition observation expired')
    cached=getattr(ctx,'_rax_condition_boundary',None);reused={};reused_state=None
    if cached and not onsite:
        try:
            old=cached['observation'];a=cached['robot'];b=context['robot']
            if (not observation_recent(old,20) or not observation_pose_matches(old['measured14'],obs['measured14']) or
                    old.get('calibration_sha256')!=obs.get('calibration_sha256') or
                    old['finger_geometry']['geometry_identity']!=obs['finger_geometry']['geometry_identity'] or
                    a['bridge']['instance']!=b['bridge']['instance'] or a.get('last_motion')!=b.get('last_motion') or
                    a['owner_status']['generation']!=b['owner_status']['generation']):raise ValueError('Different boundary')
            unchanged_rgb(old,obs)
            # Never reuse a route/opening approval for a different proposed action.
            reusable={'cup_lifted_and_held','cup_retained','cup_supported','placement_scene','released_on_coaster','placement_stable','jaw_empty','cup_upright','target_scene_consistent'}
            reused={k:v for k,v in cached['result'].get('facts',{}).items() if k in reusable and v.get('state')=='supported'}
            reused_state=cached['result'].get('support_state')
            context={**context,'held_at_boundary':bool({'cup_lifted_and_held','cup_retained'}&reused.keys())}
        except (ValueError,KeyError,OSError,TypeError):pass
    if onsite:
        supported={};metrics={'unavailable':{'onsite':'Original human reply needs native scoped validation'}};state=None
    else:
        supported,metrics,state=ordinary_rules(obs,prior,required_facts,context)
        wrist_requests=0;refreshed=None
        if wrist_depth_needed(obs,{k:v for k,v in required_facts.items() if k not in supported},context,metrics):
            before_depth_report=diagnostic('before_wrist_depth',metrics)
            wrist_requests=1;original=obs
            try:obs,new_robot=observe_wrist_depth(ctx,obs,context)
            except Exception as exc:
                exc.rax_wrist_depth_calls=1
                exc.rax_condition_report=before_depth_report
                exc.rax_rule_metrics=metrics
                raise
            rebuild=context.get('rebuild_inputs')
            if rebuild:
                rebuild=dict(rebuild)
                if rebuild.get('target14') is not None:rebuild['fk']=ctx.value('piper_fk',joints=obs['measured14'][:6],arm='left')
                context=context_for(ctx,obs,robot=new_robot,**rebuild)
            else:context={**context,'robot':new_robot}
            try:unchanged_rgb(original,obs)
            except (ValueError,KeyError,OSError):reused={};reused_state=None;context.pop('held_at_boundary',None)
            supported,metrics,state=ordinary_rules(obs,prior,required_facts,context)
            metrics['wrist_depth_request']={'previous_frame':original['frame_id'],'current_frame':obs['frame_id'],'requests':1}
            refreshed=obs
        for k in required_facts:
            if k in reused and k not in supported:supported[k]={**reused[k],'source':'same_boundary_current_image_comparison'}
        if state is None:state=reused_state
        metrics['reused_same_boundary_facts']=list(reused.keys()&required_facts.keys())
    unresolved={k:v for k,v in required_facts.items() if k not in supported}
    if context.get('require_support_state') and state is None:
        # A valid cup-retention rule is not a support decision. Ask once at this
        # boundary if current depth cannot distinguish suspension from support.
        unresolved=dict(required_facts);supported={}
    result={'ok':True,'facts':supported,'fact_sources':{k:'ordinary_code' for k in supported},
            'visual_used':False,'visual_calls':0,'rule_metrics':metrics,'frame_id':obs['frame_id'],
            'reason':'Function conditions supported by ordinary code/current observations',
            'unresolved_facts':[],'contradicted_facts':[],'invalid_evidence':[],'visible_hazards':[]}
    if unresolved:
        condition_report=diagnostic('before_optional_gemini',metrics,list(unresolved))
        if metrics.get('wrist_depth'):
            question+=' Current wrist FFS optical-only measurements: '+json.dumps(metrics['wrist_depth'])+'. Optical z is camera-relative, NOT table height, base-frame clearance, or proof of support/contact. Use as supplementary local surface evidence with the current images.'
        try:result=fallback(ctx,obs,question,prior=prior,cameras=cameras,required_facts=unresolved,onsite=onsite)
        except Exception as exc:
            exc.rax_visual_calls=1
            exc.rax_wrist_depth_calls=0 if onsite else wrist_requests
            exc.rax_condition_report=condition_report
            exc.rax_rule_metrics=metrics
            raise
        result.update(visual_used=True,visual_calls=1,rule_metrics=metrics,
                      condition_report=condition_report,
                      fallback_reason={k:metrics.get('unavailable',{}) for k in unresolved})
        supplied=result.get('facts',{})
        contradictions=[k for k,v in supplied.items() if k in required_facts and isinstance(v,dict) and v.get('state')=='contradicted']
        result['facts']={**supported,**{k:v for k,v in supplied.items() if k in unresolved or k in contradictions}}
        result['fact_sources']={**{k:'ordinary_code' for k in supported},**{k:v for k,v in result.get('fact_sources',{}).items() if k in unresolved}}
        if contradictions:result.update(ok=False,contradicted_facts=contradictions)
    if state is not None and 'support_state' not in result:result['support_state']=state
    result['required_facts']=required_facts;result['perception_s']=time.monotonic()-start
    result['wrist_depth_calls']=0 if onsite else wrist_requests
    if not onsite and refreshed is not None:result['_checked_observation']=refreshed
    if result.get('ok') and context.get('robot'):
        # Preserve only revalidated object facts across checks at this same rest.
        cached_result={**result,'facts':{**reused,**result.get('facts',{})}}
        ctx._rax_condition_boundary={'observation':obs,'robot':context['robot'],'result':cached_result}
    return result
