from task_spec import SUPPORTS
import task_spec as TS
from numeric_policy import OPEN_JAW_ERROR_M, LOCALIZATION_ERROR_M, DOWN_COS
"""Task-local qualitative RGB + measured-placement evidence. No model calls."""
import copy
import cv2
import numpy as np
from perception import HSV_RANGES
from scene_checks import current_image, PerceptionUnknown, ViewUnavailable


def view_evidence(function,*args):
    """Only unavailable images are optional; identity/state errors propagate."""
    try:return function(*args)
    except ViewUnavailable as exc:return {'visible':False,'ok':None,'available':False,'why':str(exc)}


def visible_near_point(obs, color, point):
    """Partial colored surface near its calibrated projection, not pixel metrology."""
    image=current_image(obs,'head');meta=obs['cameras']['head']
    T=np.linalg.inv(np.asarray(meta['T_base_camera'],float));K=np.asarray(meta['K'],float)
    p=T@np.r_[point,1.]
    if not np.isfinite(p).all() or p[2]<=0:raise PerceptionUnknown('Invalid projected object target')
    uv=K@p[:3];uv=uv[:2]/uv[2]
    # Include a partial face plus provisional calibration/perspective variation.
    radius=int(np.clip(abs(K[0,0])*.055/p[2],45,150))
    x,y=np.rint(uv).astype(int);h,w=image.shape[:2]
    x0,x1=max(0,x-radius),min(w,x+radius);y0,y1=max(0,y-radius),min(h,y+radius)
    if x0>=x1 or y0>=y1:return {'visible':False,'why':'Target outside head image'}
    hsv=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2HSV);mask=np.zeros(hsv.shape[:2],np.uint8)
    for lo,hi in HSV_RANGES[color]:mask|=cv2.inRange(hsv,lo,hi)
    count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
    area=int(max(stats[1:,cv2.CC_STAT_AREA],default=0))
    return {'visible':area>=100,'projected_px':uv.tolist(),'roi':[int(v) for v in (x0,y0,x1,y1)],
            'visible_component_pixels':area,'scope':'Recognizable partial color surface near expected region only'}


def wrist_body(obs,color):
    """Recognizable block body in the central region, excluding side-pad ROIs."""
    image=current_image(obs,'left');hsv=cv2.cvtColor(image[230:660,430:910],cv2.COLOR_BGR2HSV)
    mask=np.zeros(hsv.shape[:2],np.uint8)
    for lo,hi in HSV_RANGES[color]:mask|=cv2.inRange(hsv,lo,hi)
    _,_,stats,_=cv2.connectedComponentsWithStats(mask)
    area=int(max(stats[1:,cv2.CC_STAT_AREA],default=0))
    return {'visible':area>=300,'scope':'Central wrist block-color region, not support-force measurement'}


def wrist_base_pair(obs):
    """Broad neighboring red/orange surfaces, with side pads excluded.

    Image adjacency is auxiliary evidence, not a base-frame depth measurement or
    proof of support. The caller still requires the checked release/placed pose.
    """
    return wrist_pair(obs,('red','orange'))


def wrist_pair(obs,colors):
    """Partial neighboring surfaces, without exact alignment or pixel ratios."""
    im=current_image(obs,'left');h,w=im.shape[:2]
    roi=im[:int(.78*h),int(.28*w):int(.72*w)]
    hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV);boxes={}
    for color in colors:
        mask=np.zeros(hsv.shape[:2],np.uint8)
        for lo,hi in HSV_RANGES[color]:mask|=cv2.inRange(hsv,lo,hi)
        _,_,stats,_=cv2.connectedComponentsWithStats(mask)
        candidates=[row for row in stats[1:] if row[4]>=100 and min(row[2:4])>=12]
        if not candidates:return {'ok':None,'why':'Wrist base pair not recognizable'}
        x,y,bw,bh,area=max(candidates,key=lambda row:row[4]);boxes[color]=[int(x),int(y),int(bw),int(bh)]
    a,b=[np.asarray(boxes[c],float) for c in colors]
    gap=np.maximum(0,np.maximum(a[:2],b[:2])-np.minimum(a[:2]+a[2:],b[:2]+b[2:]))
    # Allow perspective, partial crop and a generous block-sized gap; reject
    # clearly unrelated color regions, without exact alignment/area ratios.
    adjacent=float(np.linalg.norm(gap))<=max(*a[2:],*b[2:])
    return {'ok':True if adjacent else None,'boxes':boxes,
            'why':'Recognizable adjacent '+ '/'.join(colors)+' surfaces in wrist view' if adjacent else 'Wrist color surfaces are clearly separated',
            'scope':'Qualitative neighboring surfaces only; no hand-eye or metric wrist target'}


def wrist_alignment(obs,color):
    body=wrist_body(obs,color)
    if not body['visible']:return body
    image=current_image(obs,'left');hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    pads=cv2.inRange(hsv,(8,100,70),(30,255,255))>0
    from scene_checks import WRIST_PAD_ROIS
    counts=[int(pads[y:y2,x:x2].sum()) for x,y,x2,y2 in WRIST_PAD_ROIS]
    return dict(body,visible=max(counts)>=100,pad_region_pixels=counts,
                scope='Recognizable central block and pad reference; geometry supplies alignment, not a pixel-area ratio')


def placed_region_evidence(obs,scene,color,point):
    """Visible placed body; adjacency is optional and refers to the CURRENT scene.

    Callers must validate actual support/release/withdrawal separately. A wrist
    patch is qualitative corroboration, never independent proof of release.
    """
    head=view_evidence(visible_near_point,obs,color,point)
    wrist={'visible':False,'not_needed':True};pair={'ok':None,'not_needed':True}
    if not head.get('visible'):
        wrist=view_evidence(wrist_body,obs,color)
    if not head.get('visible') and not wrist.get('visible'):
        # Only supplement with a neighbor already near this placed endpoint;
        # never demand the final assembly while its other block has not moved.
        # Its support, or for a table block a goal neighbour on the table (the other base block before).
        beside=[c for c in TS.goal_neighbors(color) if TS.LEVEL[c]==TS.LEVEL[color]]
        for neighbor in ([SUPPORTS[color]] if color in SUPPORTS else beside):
          block=scene.get('blocks',{}).get(neighbor)
          if block is not None and pair.get('ok') is not True:
            delta=np.asarray(block['top_center']['surface_position_m'])-np.asarray(point)
            if np.linalg.norm(delta[:2])<=.09 and abs(delta[2])<=.08:
                colors=(neighbor,color)
                pair=view_evidence(wrist_pair,obs,colors)
    return {'visible':bool(head.get('visible') or wrist.get('visible') or pair.get('ok') is True),
            'head':head,'wrist':wrist,'wrist_pair':pair,
            'scope':'Current placed body plus separately verified physical receipts; adjacency optional'}


def released_endpoint(ctx,after,scene,color,release,retreat):
    """Completed release/withdrawal plus a head location OR wrist structure."""
    if not release or release['support'].get('ok') is not True:
        raise PerceptionUnknown('Missing completed support/release evidence')
    native=release['receipt']
    for receipt in (native,retreat):
        if not all(receipt.get(k) is True for k in ('executed','measured_arrival','measured_settled')):
            raise PerceptionUnknown('Release/retreat receipt incomplete')
        if receipt.get('owner_generation')!=after['state_binding']['owner_generation']:
            raise PerceptionUnknown('Release/retreat owner changed')
    from scene_checks import same_state
    if not same_state(after.get('measured14'),retreat.get('final14')):
        raise PerceptionUnknown('Retreat observation does not match completed receipt')
    if after['measured14'][6]<.065-OPEN_JAW_ERROR_M or native['final14'][6]<.065-OPEN_JAW_ERROR_M:
        raise PerceptionUnknown('Release/retreat jaw is not open')
    released=np.asarray(ctx.value('piper_fk',joints=native['final14'][:6])['position'])
    current=np.asarray(ctx.value('piper_fk',joints=after['measured14'][:6])['position'])
    if np.linalg.norm(current-released)<.005-LOCALIZATION_ERROR_M:raise PerceptionUnknown('No distinct completed withdrawal endpoint')
    point=scene['blocks'][color]['top_center']['surface_position_m']
    evidence=placed_region_evidence(after,scene,color,point)
    if not evidence['visible']:
        raise PerceptionUnknown('Placed body not recognizable in head or wrist after completed open release; inspect endpoint, never replay retreat')
    from history_scene import reject_visible_displacement
    diagnostics=reject_visible_displacement(scene,after,associated_colors=(color,))
    return {'ok':True,'frame_id':after['frame_id'],**evidence,'scene_diagnostics':diagnostics,
            'why':'Verified support, open release and distinct withdrawal agree with visible placed body',
            'scope':'Qualitative placement result; wrist appearance alone cannot prove release'}


def placement(ctx,phase,after,scene,origin,*,placed_pose=None,placed_transform=None,withdrawn=False):
    """Use the checked path, actual FK and broad visible location at the endpoint.

    Caller supplies provenance-validated scene/origin and confirmed receipts.
    It is a qualitative placement estimate, never an independent support sensor.
    """
    fact={'ok':None,'frame_id':after['frame_id'],'phase':phase,
        'method':'ordinary_rgb_measured_placement','supported':False,
        'scope':'Qualitative agreement with the modeled support; no hidden contact/force claim'}
    try:
        color=origin['color']
        fk=ctx.value('piper_fk',joints=after['measured14'][:6]) if placed_pose is None else None
        pose=np.asarray(fk['position'] if fk is not None else placed_pose)
        original=np.asarray(scene['blocks'][color]['top_center']['surface_position_m'])
        if 'object_top_flange_m' in origin:
            transform=placed_transform if placed_transform is not None else (fk or {}).get('T_base_flange')
            if transform is None:raise PerceptionUnknown('Rigid placement estimate requires the measured release transform')
            T=np.asarray(transform,float)
            if T.shape!=(4,4) or not np.isfinite(T).all():raise PerceptionUnknown('Invalid placement transform')
            point=T[:3,:3]@np.asarray(origin['object_top_flange_m'])+T[:3,3]
            corners=np.asarray(origin['corners_flange_m'])@T[:3,:3].T+T[:3,3]
            bottom=float(corners[:,2].min())
            fact['placement_relation']='measured_rigid_grasp_transform'
        else:
            point=original+pose-np.asarray(origin['grasp_flange'])
            bottom=float(pose[2]-origin['flange_to_bottom_m'])
            fact['placement_relation']='legacy_fixed_orientation_translation'
        plane=np.asarray(scene['table']['z_ax_by_c'])
        support=float(scene['blocks'][SUPPORTS[color]]['top_center']['surface_position_m'][2] if color in SUPPORTS
                      else point[:2]@plane[:2]+plane[2])
        error=bottom-support
        fact['estimated_support_height_error_m']=error
        fact['support_height_limit_m']=.010+LOCALIZATION_ERROR_M
        if not np.isfinite(error) or abs(error)>.010+LOCALIZATION_ERROR_M:
            return fact|{'ok':False,'why':'Measured placement is visibly inconsistent with modeled support height'}
        expected=copy.deepcopy(scene);expected['blocks'][color]['top_center']['surface_position_m']=point.tolist()
        from history_scene import reject_visible_displacement
        evidence=placed_region_evidence(after,expected,color,point)
        associated=(color,) if evidence['visible'] else ()
        diagnostics=reject_visible_displacement(expected,after,associated_colors=associated)
        fact.update(head=evidence['head'],wrist=evidence['wrist'],wrist_pair=evidence['wrist_pair'],
                    scene_diagnostics=diagnostics,estimated_object_top_m=point.tolist())
        if not evidence['visible']:
            return fact|{'why':'Object region not recognizable in available views; inspect endpoint'}
        return fact|{'ok':True,'supported':True,
            'why':'Checked measured support endpoint and recognizable object at the expected broad location; no obvious displacement'}
    except (PerceptionUnknown,KeyError,TypeError,ValueError,OSError) as exc:
        return fact|{'why':str(exc)}


def after_release(ctx,phase,after,previous):
    """Derive placed position from actual release, never from the raised wrist."""
    from scene_checks import read_report
    record=read_report(previous);r=record['result']
    # Retreat endpoint may be rechecked read-only after a failed image check.
    if r['stage'] in ('retreat','green_stack_retreat'):
        record=read_report(record['inputs']['previous_report']);r=record['result']
    if r['stage'] not in ('release','green_stack_release') or not r.get('ok'):
        raise PerceptionUnknown('Need verified release predecessor')
    receipts=r.get('receipts',[])
    if not receipts or not all(all(x['result'].get(k) is True for k in
            ('executed','measured_arrival','measured_settled')) for x in receipts):
        raise PerceptionUnknown('Release needs complete native receipts')
    last=receipts[-1]['result']
    if (last.get('owner_generation')!=after['state_binding'].get('owner_generation')
            or after['measured14'][6]<.065-OPEN_JAW_ERROR_M):
        raise PerceptionUnknown('Release owner changed or jaw no longer open')
    f=r['facts'][0]
    release_fk=ctx.value('piper_fk',joints=r['receipts'][-1]['result']['final14'][:6]);pose=release_fk['position']
    return placement(ctx,phase,after,f['scene'],f['payload_origin'],placed_pose=pose,placed_transform=release_fk.get('T_base_flange'),withdrawn=True)
