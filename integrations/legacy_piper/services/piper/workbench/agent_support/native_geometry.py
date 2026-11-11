"""Read original stereo-grid measurements and same-frame RGB associations.
No filled RGB depth is used as a measurement. No hardware access.
"""
import json
from pathlib import Path
import numpy as np

def read_native(snapshot):
 obs=snapshot['observation'];folder=Path(obs['path']);head=json.loads((folder/'head.json').read_text())
 if head!=obs['head'] or folder.name!=obs['frame_id']:raise ValueError('NATIVE_FRAME_IDENTITY')
 meta=head.get('native_geometry',{})
 if meta.get('schema')!='rax.native_geometry.v1' or meta.get('coordinate_frame')!='left_rectified_optical':raise ValueError('NATIVE_GEOMETRY_REQUIRED')
 with np.load(folder/'depth.npz',allow_pickle=False) as z:
  depth,uv,visible=z['native_depth_m'],z['native_uv_rgb'],z['native_visible'];rgb_shape=z['valid'].shape
 h,w=meta['shape_hw']
 if depth.shape!=(h,w) or uv.shape!=(h,w,2) or visible.shape!=(h,w) or visible.dtype!=bool:raise ValueError('NATIVE_ARRAY_SHAPE')
 valid=np.isfinite(depth)&(depth>0)&np.isfinite(uv).all(2)&visible
 # Bound before rounding/casting; out-of-view points remain stored but unassociated.
 valid&=(uv[...,0]>=-.5)&(uv[...,0]<rgb_shape[1]-.5)&(uv[...,1]>=-.5)&(uv[...,1]<rgb_shape[0]-.5)
 ids=np.flatnonzero(valid);pixels=np.floor(uv.reshape(-1,2)[ids]+.5).astype(np.int32)
 return dict(depth=depth,ids=ids,pixels=pixels,meta=meta,rgb_shape=rgb_shape)

def select_native(data,*,roi=None,rgb_mask=None):
 ids,pixels=data['ids'],data['pixels'];keep=np.ones(len(ids),bool)
 if roi is not None:
  x0,y0,x1,y1=roi;keep&=(pixels[:,0]>=x0)&(pixels[:,0]<x1)&(pixels[:,1]>=y0)&(pixels[:,1]<y1)
 if rgb_mask is not None:
  if rgb_mask.shape!=data['rgb_shape']:raise ValueError('NATIVE_RGB_MASK_SHAPE')
  keep&=rgb_mask[pixels[:,1],pixels[:,0]]>0
 ids,pixels=ids[keep],pixels[keep];m=data['meta'];h,w=data['depth'].shape
 fx,fy,cx,cy=m['intrinsics_fx_fy_cx_cy'];z=data['depth'].ravel()[ids].astype(float)
 xyz=np.column_stack([((ids%w)-cx)*z/fx,((ids//w)-cy)*z/fy,z]);T=np.asarray(m['T_rgb_from_leftrect'],float)
 if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('NATIVE_TRANSFORM')
 points=xyz@T[:3,:3].T+T[:3,3]
 return dict(camera_points=points,source_flat_index=ids.astype(np.int32),rgb_pixels=pixels,
             geometry_source='original_stereo_grid',surface_samples_used=False)

def associate_segmentation(snapshot,result):
 import cv2
 from scene_contract import bind_result,digest
 bind_result(snapshot,result);data=read_native(snapshot);folder=Path(result['directory']).resolve();rows=[]
 for obj in [*result['views']['head']['objects'],*result['views']['head'].get('candidates',[])]:
  path=(folder/obj['mask']).resolve()
  if path.parent!=folder or digest(path)!=obj['mask_sha256']:raise ValueError('NATIVE_MASK_IDENTITY')
  mask=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
  if mask is None:raise ValueError('NATIVE_MASK_MISSING')
  selected=select_native(data,rgb_mask=mask);destination=folder/(path.stem+'-native.npz')
  np.savez(destination,xyz_rgb_m=selected['camera_points'].astype(np.float32),source_flat_index=selected['source_flat_index'],rgb_pixels=selected['rgb_pixels'])
  rows.append(dict(concept=obj['concept'],confidence_status=obj.get('confidence_status'),mask=obj['mask'],path=str(destination),
                   sha256=digest(destination),points=len(selected['source_flat_index']),frame_id=snapshot['observation']['frame_id'],motion_permit=False))
 return dict(schema='rax.semantic_native_association.v1',objects=rows,occlusion_test='current_frame_footprint_zbuffer_1mm',
             geometry_source='original_stereo_grid',physical_accuracy_verified=False,motion_permit=False)
