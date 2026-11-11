"""Task-specific circle/height association. No model, SDK or motion imports.

Qualified only offline for one circular cup and one circular coaster on the known
fixed-camera table. Ambiguity returns to semantic perception, never invents points.
The returned coordinates are measured separately by the existing depth tools.
"""
from pathlib import Path
import numpy as np

TABLE_Z_RANGE=(-.01,.05)
CUP_HEIGHT_RANGE=(.045,.120)
COASTER_HEIGHT_RANGE=(-.005,.020)
MIN_DEPTH_SAMPLES=25


def base_depth(obs):
    cam=obs['cameras']['head']
    if not cam.get('metric_depth'):raise ValueError('Metric head depth required')
    with np.load(Path(cam['inspection_image']).with_name('head-depth.npz')) as data:
        xyz=data['xyz_rgb_m'];valid=data['valid'].astype(bool)
    T=np.asarray(cam['T_base_camera'],float)
    if T.shape!=(4,4) or xyz.shape[:2]!=valid.shape or xyz.shape[2]!=3:
        raise ValueError('Depth geometry shape mismatch')
    base=xyz@T[:3,:3].T+T[:3,3]
    return base,valid & np.isfinite(base).all(axis=2)


def table_sample(obs,circles):
    base,valid=base_depth(obs);yy,xx=np.indices(valid.shape)
    mask=valid & (base[:,:,0]>.055)&(base[:,:,0]<.40)&(base[:,:,1]>-.40)&(base[:,:,1]<.045)
    mask &= (base[:,:,2]>TABLE_Z_RANGE[0])&(base[:,:,2]<TABLE_Z_RANGE[1])
    values=circles.values() if isinstance(circles,dict) else circles
    for x,y,r in values:mask &= ((xx-x)**2+(yy-y)**2>(r+12)**2)
    z=base[:,:,2][mask]
    if z.size<100:raise ValueError('Insufficient visible tabletop depth')
    bins=np.arange(TABLE_Z_RANGE[0],TABLE_Z_RANGE[1]+.002,.002)
    hist,_=np.histogram(z,bins);i=int(hist.argmax());plane=(bins[i]+bins[i+1])/2
    mask &= abs(base[:,:,2]-plane)<.003
    candidates=np.column_stack([xx[mask],yy[mask]])
    if len(candidates)<100:raise ValueError('No dominant table region')
    # Select interior of the supported region rather than one predetermined pixel.
    import cv2
    counts=cv2.boxFilter(mask.astype('float32'),-1,(7,7),normalize=False,borderType=cv2.BORDER_CONSTANT)
    counts[~mask]=0
    y,x=np.unravel_index(counts.argmax(),counts.shape)
    if counts[y,x]<25:raise ValueError('Table support patch too small')
    return [int(x),int(y)],float(np.median(base[:,:,2][mask]))


def classify_scene(obs,circles):
    circles=np.asarray(circles,float).reshape(-1,3)
    pixel,table_z=table_sample(obs,circles);base,valid=base_depth(obs)
    yy,xx=np.indices(valid.shape);cups=[];coasters=[]
    for circle in circles:
        x,y,r=circle
        if min(x-r,y-r)<8 or x+r>=valid.shape[1]-8 or y+r>=valid.shape[0]-8:continue
        rr=np.hypot(xx-x,yy-y);rim=valid&(rr>r-7)&(rr<r+1)
        rz=base[:,:,2][rim]-table_z
        high=rz[(rz>CUP_HEIGHT_RANGE[0])&(rz<CUP_HEIGHT_RANGE[1])]
        if len(high)>=MIN_DEPTH_SAMPLES and np.quantile(high,.9)-np.quantile(high,.1)<.025:
            cups.append(circle.tolist());continue
        inside=valid&(rr<r*.55);z=base[:,:,2][inside]-table_z
        if len(z)>=MIN_DEPTH_SAMPLES and COASTER_HEIGHT_RANGE[0]<np.median(z)<COASTER_HEIGHT_RANGE[1] and np.quantile(z,.9)-np.quantile(z,.1)<.012:
            coasters.append(circle.tolist())
    if len(cups)!=1 or len(coasters)!=1:raise ValueError('Circle/height identity not unique')
    if np.linalg.norm(np.array(cups[0][:2])-coasters[0][:2])<=cups[0][2]+coasters[0][2]+10:
        raise ValueError('Cup/coaster overlap; fresh separated layout required')
    return {'cup':cups[0],'coaster':coasters[0]},pixel


def refresh_table_pixel(old_obs,new_obs,pixel,old_image,new_image):
    """Historical semantic pixel is only a search seed; measure NEW valid points."""
    x,y=map(int,pixel);radius=12
    if old_image.shape!=new_image.shape or min(x,y)<radius or x+radius>=new_image.shape[1] or y+radius>=new_image.shape[0]:
        raise ValueError('Table region changed/cropped')
    a=old_image[y-radius:y+radius+1,x-radius:x+radius+1]
    b=new_image[y-radius:y+radius+1,x-radius:x+radius+1]
    if np.mean(abs(a.astype(float)-b))>18:raise ValueError('Table appearance changed')
    old,ov=base_depth(old_obs);base,valid=base_depth(new_obs)
    old_z=old[y-radius:y+radius+1,x-radius:x+radius+1,2][ov[y-radius:y+radius+1,x-radius:x+radius+1]]
    if len(old_z)<25:raise ValueError('Historical table patch missing')
    yy,xx=np.indices(valid.shape)
    mask=valid&(abs(xx-x)<=radius)&(abs(yy-y)<=radius)&(abs(base[:,:,2]-np.median(old_z))<.008)
    pixels=np.column_stack([xx[mask],yy[mask]])
    if len(pixels)<25:raise ValueError('Current table patch missing')
    return pixels[np.argmin(np.linalg.norm(pixels-[x,y],axis=1))].tolist()
