"""FFS geometric quality and conservative RGB rasterization. No model change.

Original nearest-point observations remain available. Additional surface support
is attributed to its source pixel, not called a new measurement or confidence.
The footprint idea follows librealsense/src/proc/align.cpp; unlike its unrestricted
rectangle transfer, we exclude holes, depth discontinuities and large footprints.
"""
import numpy as np
from services.piper.sensor.stereo_geometry import align_left_xyz_to_rgb, validate_se3


def left_right_consistency(left, right, threshold_px=1.0):
    """right is positive disparity indexed in the ORIGINAL right image."""
    left, right = np.asarray(left), np.asarray(right)
    if left.ndim != 2 or right.shape != left.shape or not 0 < threshold_px <= 2:
        raise ValueError('LEFT_RIGHT_CONTRACT')
    yy, xx = np.indices(left.shape)
    xr = xx-left
    valid = np.isfinite(left) & (left > 0) & (xr >= 0) & (xr <= left.shape[1]-1)
    safe = np.where(valid, xr, 0)
    lo, hi = np.floor(safe).astype(int), np.ceil(safe).astype(int)
    a, b = right[yy,lo], right[yy,hi]
    valid &= np.isfinite(a) & np.isfinite(b) & (a > 0) & (b > 0)
    error = np.abs(left - (a+(safe-lo)*(b-a)))
    return valid & (error <= threshold_px), np.where(valid, error, np.nan)


def project(points, intr):
    p = points.reshape(-1,3)
    if not len(p): return np.empty((0,2),float)
    if intr.distortion_model == 'brown_conrady':
        import cv2
        return cv2.projectPoints(p,np.zeros(3),np.zeros(3),intr.matrix,np.asarray(intr.coeffs))[0].reshape(-1,2)
    x, y = p[:,0]/p[:,2], p[:,1]/p[:,2]
    if intr.distortion_model in ('modified_brown_conrady','inverse_brown_conrady'):
        k1,k2,p1,p2,k3 = intr.coeffs
        r2=x*x+y*y; f=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        x,y=x*f,y*f
        x,y=x+2*p1*x*y+p2*(r2+2*x*x), y+2*p2*x*y+p1*(r2+2*y*y)
    return np.column_stack((x*intr.fx+intr.cx,y*intr.fy+intr.cy))


def align_surface_footprints(xyz, valid, transform, source_intrinsics, rgb_intrinsics,
                             max_local_span_m=.005, max_footprint_px=4):
    if not 0 < max_local_span_m <= .01 or not 1 <= max_footprint_px <= 4:
        raise ValueError('SURFACE_SUPPORT_LIMIT')
    if source_intrinsics.distortion_model != 'none':
        raise ValueError('SOURCE_MUST_BE_RECTIFIED')
    transform=validate_se3(transform)
    xyz=np.asarray(xyz); valid=np.asarray(valid)
    if xyz.shape != (source_intrinsics.height,source_intrinsics.width,3):
        raise ValueError('SOURCE_INTRINSICS_SHAPE')
    direct=align_left_xyz_to_rgb(xyz,valid,transform,rgb_intrinsics)
    h,w=valid.shape; z=xyz[...,2]
    supported=valid & np.isfinite(xyz).all(2) & (z>0)
    local_min=np.full((h,w),np.inf);local_max=np.full((h,w),-np.inf)
    # Require all 3x3 source neighbors: holes and silhouettes never spread.
    zp=np.pad(z,1,constant_values=np.nan);vp=np.pad(supported,1,constant_values=False)
    for dy in range(3):
        for dx in range(3):
            patch=zp[dy:dy+h,dx:dx+w]
            supported &= vp[dy:dy+h,dx:dx+w]
            local_min=np.minimum(local_min,np.where(np.isfinite(patch),patch,np.inf))
            local_max=np.maximum(local_max,np.where(np.isfinite(patch),patch,-np.inf))
    supported &= (local_max-local_min <= max_local_span_m)
    ids=np.flatnonzero(supported);points=xyz.reshape(-1,3)[ids]
    target=points@transform[:3,:3].T+transform[:3,3]
    corners=[]
    # Map all four pixel corners. This uses the source sample's local surface,
    # not neighboring target holes and not arbitrary RGB interpolation.
    for dx,dy in [(-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5)]:
        q=points.copy();q[:,0]+=dx*q[:,2]/source_intrinsics.fx;q[:,1]+=dy*q[:,2]/source_intrinsics.fy
        q=q@transform[:3,:3].T+transform[:3,3]
        corners.append(project(q,rgb_intrinsics))
    uv_min=corners[0].copy();uv_max=corners[0].copy()
    finite=(target[:,2]>0)
    for uv in corners:
        finite &= np.isfinite(uv[:,0]) & np.isfinite(uv[:,1])
        np.minimum(uv_min,uv,out=uv_min);np.maximum(uv_max,uv,out=uv_max)
    low=np.floor(np.where(finite[:,None],uv_min,0)+.5).astype(int)
    high=np.floor(np.where(finite[:,None],uv_max,0)+.5).astype(int)
    size=high-low+1
    keep=finite & (low[:,0]>=0) & (low[:,1]>=0) & (high[:,0]<rgb_intrinsics.width) & (high[:,1]<rgb_intrinsics.height)
    keep &= (size[:,0]>0) & (size[:,1]>0) & (size[:,0]<=max_footprint_px) & (size[:,1]<=max_footprint_px)
    ids,target,low,high=ids[keep],target[keep],low[keep],high[keep]
    out=direct['xyz_rgb_m'].copy();source=direct['source_flat_index'].copy()
    flat=out.reshape(-1,3)
    zbuffer=np.where(np.isfinite(flat[:,2]),flat[:,2],np.inf)
    batches=[]
    for dy in range(max_footprint_px):
        for dx in range(max_footprint_px):
            pix=low+[dx,dy];eligible=(pix[:,0]<=high[:,0]) & (pix[:,1]<=high[:,1])
            indexes=pix[eligible,1]*rgb_intrinsics.width+pix[eligible,0]
            zs=target[eligible,2];origin=ids[eligible]
            batches.append((indexes,zs,origin))
            np.minimum.at(zbuffer,indexes,zs)
    missing=np.iinfo(np.int64).max
    winners=np.full(zbuffer.shape,missing,np.int64)
    direct_ids=direct['source_flat_index'].ravel()
    keep_direct=(direct_ids>=0) & np.isclose(flat[:,2],zbuffer,rtol=0,atol=1e-12)
    winners[keep_direct]=direct_ids[keep_direct]
    for indexes,zs,origin in batches:
        keep=np.isclose(zs,zbuffer[indexes],rtol=0,atol=1e-12)
        np.minimum.at(winners,indexes[keep],origin[keep])
    filled=winners!=missing
    all_points=xyz.reshape(-1,3)
    flat[filled]=all_points[winners[filled]]@transform[:3,:3].T+transform[:3,3]
    source=winners.reshape(source.shape);source[~filled.reshape(source.shape)]=-1
    mask=source>=0
    return {'xyz_rgb_m':out,'valid_mask':mask,'source_flat_index':source,
            'direct_valid_mask':direct['valid_mask'],'direct_xyz_rgb_m':direct['xyz_rgb_m'],
            'surface_resampled_mask':mask & (~direct['valid_mask'] | (source!=direct['source_flat_index'])),
            'source_surface_support':supported,'confidence_available':False,
            'alignment':'conservative_source_pixel_footprint_zbuffer',
            'max_local_span_m':max_local_span_m,'max_footprint_px':max_footprint_px,
            'motion_permit':False}
