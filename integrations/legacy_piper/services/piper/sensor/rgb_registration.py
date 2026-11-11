"""Native-grid geometry + conservative RGB footprint projection, without motion.
Build explicitly with scripts/build_rgb_registration.py; never compile at startup.
"""
import ctypes
import hashlib
import json
from pathlib import Path
import numpy as np
from services.piper.sensor.stereo_geometry import validate_se3

SOURCE=Path(__file__).with_suffix('.cpp')
BUILD=SOURCE.parents[3]/'commissioning/build/rgb-registration'
_LIB=None

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def library():
 global _LIB
 if _LIB is None:
  key=digest(SOURCE);path=BUILD/(key+'.so');receipt=json.loads(path.with_suffix('.json').read_text())
  if receipt['source_sha256']!=key or receipt['binary_sha256']!=digest(path):raise ValueError('RGB_REGISTRATION_BUILD_MISMATCH')
  lib=ctypes.CDLL(str(path));fn=lib.register_v1
  fn.argtypes=[ctypes.c_int]*4+[ctypes.c_void_p]*5+[ctypes.c_int,ctypes.c_double,ctypes.c_int]+[ctypes.c_void_p]*6
  fn.restype=None;_LIB=lib
 return _LIB

def register(xyz,valid,transform,source_intrinsics,rgb_intrinsics,max_local_span_m=.005,max_footprint_px=4):
 src,rgb=source_intrinsics,rgb_intrinsics
 xyz=np.ascontiguousarray(xyz,dtype=np.float64);valid=np.ascontiguousarray(valid)
 if (xyz.shape!=(src.height,src.width,3) or valid.shape!=xyz.shape[:2] or valid.dtype!=bool
     or src.distortion_model!='none' or not 0<max_local_span_m<=.01 or type(max_footprint_px)!=int or not 1<=max_footprint_px<=4):
  raise ValueError('RGB_REGISTRATION_INPUT')
 if rgb.distortion_model not in ('none','brown_conrady','modified_brown_conrady','inverse_brown_conrady'):
  raise ValueError('RGB_DISTORTION_MODEL')
 t=np.ascontiguousarray(validate_se3(transform));ks=np.array([src.fx,src.fy],dtype=np.float64)
 kr=np.array([rgb.fx,rgb.fy,rgb.cx,rgb.cy,*rgb.coeffs],dtype=np.float64)
 if len(kr)!=9:raise ValueError('RGB_INTRINSICS')
 model=0 if rgb.distortion_model=='none' else 1 if rgb.distortion_model=='brown_conrady' else 2
 q=np.empty_like(xyz);uv=np.empty((*valid.shape,2),dtype=np.float64)
 direct=np.empty((rgb.height,rgb.width),np.int32);surface=np.empty_like(direct)
 support=np.empty(valid.shape,bool);visible=np.empty_like(support)
 ptr=lambda a:a.ctypes.data
 library().register_v1(src.height,src.width,rgb.height,rgb.width,*map(ptr,[xyz,valid,t,ks,kr]),model,max_local_span_m,max_footprint_px,*map(ptr,[q,uv,direct,surface,support,visible]))
 out=np.full((rgb.height,rgb.width,3),np.nan,np.float32);mask=direct>=0
 out[mask]=q.reshape(-1,3)[direct[mask]]
 depth=np.full(mask.shape,np.nan,np.float32);sm=surface>=0;depth[sm]=q.reshape(-1,3)[surface[sm],2]
 # Persisted UVs are float32; a double just inside the image may round outside.
 # Restrict association after serialization. Never clamp/relabel a source point.
 uv32=uv.astype(np.float32)
 visible &= (np.isfinite(uv32).all(axis=-1) & (uv32[...,0]>=-.5)
             & (uv32[...,0]<rgb.width-.5) & (uv32[...,1]>=-.5)
             & (uv32[...,1]<rgb.height-.5))
 return dict(xyz_rgb_m=out,valid=mask,direct_source_index=direct,surface_source_index=surface,
             surface_depth_rgb_m=depth,surface_resampled=sm&((direct<0)|(surface!=direct)),
             native_xyz_rgb_m=q,native_uv_rgb=uv32,native_visible=visible,
             source_surface_support=support,surface_used_for_motion=False,
             implementation_sha256=digest(SOURCE),alignment='conservative_source_pixel_footprint_zbuffer')
