"""Known tabletop scene checks with installed envelopes, never a motion permit.

Opening is a continuous linear-prismatic sweep. Arm moves still use the existing
FK samples plus native CAD/workspace path checking; these are not full environment
or continuous arm collision certification. All coordinates are base metres.
"""
import numpy as np
from perception import PerceptionUnknown
CALIBRATION_MARGIN_M=.020
MIN_TABLE_GAP_M=.012
# 60mm nominal cube: circumscribe arbitrary yaw; no unmeasured axis alignment.
BLOCK_HALF_XY_M=.030*np.sqrt(2.)

def array(v,shape,label):
 a=np.asarray(v,float)
 if a.shape!=shape or not np.isfinite(a).all():raise PerceptionUnknown('Invalid '+label)
 return a

def envelope_clearance(envelopes,scene):
 plane=array(scene['table']['z_ax_by_c'],(3,),'table plane')
 if not scene.get('blocks') or not envelopes:raise PerceptionUnknown('Missing known environment/finger envelopes')
 blocks={}
 for name,b in scene['blocks'].items():
  p=array(b['top_center']['surface_position_m'],(3,),name+' center')
  floor=float(p[:2]@plane[:2]+plane[2])
  if not .035<=p[2]-floor<=.095:raise PerceptionUnknown('Expected upright tabletop block: '+name)
  # Account for table tilt across the yaw-independent block footprint.
  low=floor-BLOCK_HALF_XY_M*float(np.abs(plane[:2]).sum())
  blocks[name]=(p-np.array([BLOCK_HALF_XY_M,BLOCK_HALF_XY_M,0]),p+np.array([BLOCK_HALF_XY_M,BLOCK_HALF_XY_M,0]))
  blocks[name][0][2]=low
 minimum_table=float('inf');minimum_block=float('inf');details=[]
 for i,points in enumerate(envelopes):
  pts=np.asarray(points,float)
  if pts.ndim!=2 or pts.shape[1]!=3 or len(pts)<8 or not np.isfinite(pts).all():raise PerceptionUnknown('Invalid finger envelope')
  table=float(np.min(pts[:,2]-pts[:,:2]@plane[:2]-plane[2]))-CALIBRATION_MARGIN_M
  minimum_table=min(minimum_table,table)
  if table<MIN_TABLE_GAP_M:
   raise PerceptionUnknown(f'Finger {i+1}/table: lower clearance {table*1000:.1f}mm < {MIN_TABLE_GAP_M*1000:.0f}mm after20mm allowance')
  lo,hi=pts.min(axis=0),pts.max(axis=0)
  for name,(blo,bhi) in blocks.items():
   # Positive separation on ANY axis proves boxes are disjoint. AABBs are
   # conservative and include the whole finger motion, not just its TCP.
   gap=float(np.max(np.maximum(lo-bhi,blo-hi)))-CALIBRATION_MARGIN_M
   minimum_block=min(minimum_block,gap)
   details.append({'finger':i+1,'block':name,'separation_lower_m':gap})
   if gap<=0:raise PerceptionUnknown(f'Finger {i+1}/{name}: swept envelope overlap or insufficient allowance ({gap*1000:.1f}mm after20mm allowance)')
 return {'min_table_clearance_lower_m':minimum_table,'min_block_separation_lower_m':minimum_block,
         'separations':details,'calibration_allowance_m':CALIBRATION_MARGIN_M,
         'block_half_xy_m':BLOCK_HALF_XY_M,'scope':'Known upright ~60mm blocks and modeled table only; provisional geometry'}

def opening_sweep(obs,scene,target_gap):
 g=obs['finger_geometry'];q=array(obs['measured14'],(14,),'measured state')
 if not (g.get('geometry_identity') and g.get('jaw_motion_model')=='linear_prismatic_full_gap_v1'):
  raise PerceptionUnknown('Missing installed jaw motion model; reconnect updated observation backend')
 if not np.isfinite(target_gap) or not 0<=q[6]<target_gap<=.073 or not np.isfinite(g.get('measured_full_gap_m',float('nan'))) or abs(g.get('measured_full_gap_m',float('nan'))-q[6])>.0005:
  raise PerceptionUnknown('Opening geometry/gap differs from measured jaw')
 rows=g.get('fingers',[])
 if len(rows)!=2 or {f.get('finger') for f in rows}!={1,2}:raise PerceptionUnknown('Need both installed finger envelopes')
 envelopes=[]
 for f in rows:
  p=array(f['cad_corners_base_m'],(8,3),'finger corners')
  axis=array(f['translation_per_full_gap_base'],(3,),'jaw axis')
  if not .49<=np.linalg.norm(axis)<=.51:raise PerceptionUnknown('Unexpected Piper full-gap mimic scale')
  end=p+axis*(target_gap-q[6])
  envelopes.append(np.concatenate([p,end]))
 result=envelope_clearance(envelopes,scene)
 return result|{'sweep':'continuous linear prismatic envelope','start_gap_m':float(q[6]),'target_gap_m':target_gap,
                'swept_bounds_base_m':[{'min':p.min(axis=0).tolist(),'max':p.max(axis=0).tolist()} for p in envelopes]}


RETREAT_ANCHOR_ALLOWANCE_M = .005  # plan-to-measured binding, NOT collision clearance

def retreat_anchor(requested, current, *, reverse_to_start=False):
 """Rebind a nearby recovery anchor before checking its actual new path.

 Small arrival differences may not turn an upward escape into a stale lower
 endpoint. Preserve a deliberately higher orange clearance; green reversals
 return to the measured start. Never alter an obstacle or contact tolerance.
 """
 planned=array(requested,(3,),'requested retreat');actual=array(current,(3,),'measured retreat start')
 delta=planned-actual;allowance=RETREAT_ANCHOR_ALLOWANCE_M
 if reverse_to_start:
  if np.linalg.norm(delta)>allowance+1e-12:
   raise PerceptionUnknown('Stack retreat anchor differs from measured start by more than5mm')
  effective=actual.copy()
 else:
  if np.linalg.norm(delta[:2])>allowance+1e-12 or delta[2]<-allowance-1e-12:
   raise PerceptionUnknown('Retreat anchor differs from measured start by more than5mm')
  effective=actual.copy();effective[2]=max(planned[2],actual[2])
 return {'position':effective.tolist(),'requested':planned.tolist(),'measured_start':actual.tolist(),
         'reanchored':bool(np.max(np.abs(effective-planned))>1e-12),
         'adjustment_m':(effective-planned).tolist(),'binding_allowance_m':allowance,
         'scope':'Target rebinding only; native and scene path validation still required'}
