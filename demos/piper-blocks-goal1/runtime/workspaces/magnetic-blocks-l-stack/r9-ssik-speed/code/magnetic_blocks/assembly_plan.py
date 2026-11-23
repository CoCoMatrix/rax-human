"""A+ finite whole-task search. Only pure FK/IK/path tools; no dispatch.

Goal axis is fixed: red/orange may swap alongY; green always over red. A goal
translation is selected before any grasp. Predicted scenes never become images.
"""
import copy,hashlib,json,math,time
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from perception import PerceptionUnknown
from fk_samples import fk_many
from numeric_policy import EXTRA_GEOMETRY_ERROR_M, GEOMETRY_ERROR_M, CORRIDOR_ERROR_M, LOCALIZATION_ERROR_M, LAYOUT_ERROR_M, DOWN_COS, OPEN_JAW_ERROR_M, clearance_minimum, policy_record
from numeric_policy import task_orientation_limit_deg,TASK_ORIENTATION_ERROR_DEG

from task_spec import COLORS, SUPPORTS, BASE_COLORS, VERSION, PREFLIGHT_BUDGET_S, FAST_PASS_BUDGET_S, LAYOUT_SLICE_FAST_S, LAYOUT_SLICE_FULL_S
import task_spec as TS
OPEN=.073
CLOSED=.050
SPACING=.068  # 68 mm base center spacing gives nominal 8 mm between 60 mm blocks.
YAW_DEGREES=(180.,0.,-90.,90.,177.,-3.,-93.,87.,-177.,3.,-87.,93.)
# Task object outer-edge layout bounds, NOT flange workspace or joint limits.
LAYOUT_MIN=np.array([.090,-.420])
LAYOUT_MAX=np.array([.520,.065])
MIN_TABLE=.012
MARGIN=.003
# Operator-authorized tolerance for the nominal 3mm finger/object AABB rule.
# The original tolerance keeps 0.5mm; the separate1mm comparison allowance
# is capped at zero clearance. Native FCL still rejects touching/overlap.
FINGER_GAP_TOLERANCE_M=.0025  # +0.5mm operator-approved model/rebinding allowance; positive0.5mm minimum
FINGER_GAP_COMPARISON_ERROR_M=.001  # Additional operator-authorized task AABB error; no overlap exemption.
RELEASED_RETREAT_AABB_ERROR_M=.001  # Only open, unloaded withdrawal from its released target; native collision unchanged.
RELEASE_JAW_FEEDBACK_ERROR_M=.001  # Full jaw opening, not per-finger displacement.


def orientation_evidence(T,color,reference_T=None):
 # Keep the optional legacy argument for callers, but never grant a jaw-only
 # exception that the immediately following loaded path would reject.
 T=np.asarray(T,float);base,limit=task_orientation_limit_deg(color)
 if T.shape!=(4,4) or not np.isfinite(T).all():raise NoPlan('Task orientation requires finite4x4 FK transform')
 tilt=math.degrees(math.acos(float(np.clip(-T[2,2],-1,1))))
 fact={'measured_tilt_deg':tilt,'planning_limit_deg':base,'accepted_limit_deg':limit,
       'comparison_error_deg':TASK_ORIENTATION_ERROR_DEG,'signed_bounds_deg':[-limit,limit],
       'reference_used':False,'policy':'same task tilt bound for contact, jaw and loaded path samples'}
 if tilt<=limit+1e-9:return fact
 raise NoPlan(f'Task orientation outside shared bound: measured={tilt:.6f}deg nominal_limit={base:.6f}deg comparison_error={TASK_ORIENTATION_ERROR_DEG:.3f}deg accepted_limit={limit:.6f}deg')

def retreat_progress_evidence(samples):
 """AABB projection may wobble; tool-center distance must steadily increase.
 Native oriented collision checking independently protects every sampled link.
 """
 values=np.asarray(samples,float)
 if values.ndim!=2 or values.shape[1]!=2 or len(values)<2 or not np.isfinite(values).all():
  raise NoPlan('Released retreat lacks a finite sampled departure')
 distances=values[:,0];backtrack=float(np.max(np.maximum.accumulate(distances)-distances))
 if backtrack>1e-6 or distances[-1]<=distances[0]+1e-6 or values[-1,1]<=0:
  raise NoPlan('Released retreat comparison allowance requires progressive departure and a clear final envelope')
 return {'tool_center_distance_start_m':float(distances[0]),'tool_center_distance_end_m':float(distances[-1]),
         'maximum_backtrack_m':backtrack,'final_envelope_gap_m':float(values[-1,1]),'progressive_departure':True}

def finger_gap_minimum(nominal):
 return max(0.,nominal-FINGER_GAP_TOLERANCE_M-FINGER_GAP_COMPARISON_ERROR_M) if abs(nominal-.003)<1e-12 else clearance_minimum(nominal)

def release_feedback_gaps():
 return (max(0.,OPEN-RELEASE_JAW_FEEDBACK_ERROR_M),min(.076,OPEN+RELEASE_JAW_FEEDBACK_ERROR_M))

PAYLOAD_MIN_CLEARANCE_M=.020
PAYLOAD_PREFERRED_CLEARANCE_M=.023

# Additional comparison allowance only for the inflated carried-block/neighbor
# envelopes in free flight. Native collision and deliberate contact keep their
# own checks. Plan heights at the former minimum before spending this allowance.
PAYLOAD_PLAN_FLOOR_M=clearance_minimum(PAYLOAD_MIN_CLEARANCE_M)
PAYLOAD_EXECUTION_MIN_M=.015
PAYLOAD_COMPARISON_M=PAYLOAD_PLAN_FLOOR_M-PAYLOAD_EXECUTION_MIN_M

def payload_clearance_fact(gap):
 return {"minimum_m":PAYLOAD_EXECUTION_MIN_M,"nominal_minimum_m":PAYLOAD_MIN_CLEARANCE_M,"tolerance_m":GEOMETRY_ERROR_M+PAYLOAD_COMPARISON_M,"extra_payload_comparison_m":PAYLOAD_COMPARISON_M,"preferred_m":PAYLOAD_PREFERRED_CLEARANCE_M,
         "observed_model_gap_m":gap,"preferred_met":gap is None or gap>=PAYLOAD_PREFERRED_CLEARANCE_M,
         "scope":"Modeled envelope clearance; preference is not an execution refusal"}


class NoPlan(PerceptionUnknown):
 reason_category='planning_or_geometry'
class IKUnavailable(NoPlan):
 reason_category='ik_no_solution'
class NativePathRefusal(NoPlan):
 reason_category='native_path_collision'
class PlanningTimeout(RuntimeError):
 reason_category='planning_budget'
class FastPassTimeout(PlanningTimeout):pass
class CandidateBudgetExceeded(Exception):pass
class GraspBudgetExceeded(Exception):pass

# Same 0.01 rad as the admitted endpoint feedback error. Leave that room
# inside the URDF on commands moving toward a stop; keep the actual hard limits.
JOINT_ENDPOINT_RESERVE_RAD=.01

@lru_cache(maxsize=2)
def arm_bounds(side):
 from native_fk.geometry import Kinematics
 return np.asarray([(joint[2],joint[3]) for joint in Kinematics().chains[side]])

def measured_limits(q):
 for side,offset in (('left',0),('right',7)):
  bounds=arm_bounds(side)
  for j,(angle,(lo,hi)) in enumerate(zip(q[offset:offset+6],bounds),1):
   if not lo<=angle<=hi:
    raise RuntimeError(f'{side} J{j} measured {angle:.9f} rad outside URDF [{lo:.9f}, {hi:.9f}]; no new motion')

def check_command_reserve(start,points):
 bounds=arm_bounds('left');anchor=np.asarray(start[:6],float)
 # Permit holding or moving inwards from an in-limit Prepare pose near a stop.
 lo=np.minimum(anchor,bounds[:,0]+JOINT_ENDPOINT_RESERVE_RAD)
 hi=np.maximum(anchor,bounds[:,1]-JOINT_ENDPOINT_RESERVE_RAD)
 vals=np.atleast_2d(np.asarray(points,float))[:,:6]
 # Wire samples are float32. This epsilon applies only to the reserve.
 bad=np.argwhere((vals<lo-1e-7)|(vals>hi+1e-7))
 if len(bad):
  idx,j=bad[0]
  raise NoPlan(f'Left J{j+1} requested {vals[idx,j]:.7f} rad leaves insufficient endpoint reserve ({JOINT_ENDPOINT_RESERVE_RAD:.3f} rad inside URDF)')

def xyz(scene,color):return np.asarray(scene['blocks'][color]['top_center']['surface_position_m'],float)
def table(scene,p):return float(np.r_[p[:2],1.]@np.asarray(scene['table']['z_ax_by_c']))
def height(scene,color):
 p=xyz(scene,color);return float(scene['blocks'][color].get('height_m',p[2]-table(scene,p)))
def quat(yaw):return (Rotation.from_euler('z',yaw,degrees=True)*Rotation.from_euler('x',180,degrees=True)).as_quat().tolist()
def source_inward(scene,color):
 # At extended reach try bringing the flange towards the base without moving
 # the observed object or changing the accepted contact/tilt envelope.
 src=xyz(scene,color);rad=np.linalg.norm(src[:2])
 if rad<.50 or color in TS.HIGH_COLORS:return
 axis=np.array([-src[1]/rad,src[0]/rad,0.])
 for tilt,dz,off in ((-6.7,.004,.010),(-6.55,.005,.008),(-6.5,.004,.009),(-6.7,.003,.011),(-6.5,.006,.009),(-6.5,.004,.003),(-5.,.006,.006),(-6.5,.006,0.)):
  for yaw in (-90.,180.,0.,90.):
   rot=Rotation.from_rotvec(axis*math.radians(tilt))*Rotation.from_quat(quat(yaw))
   long=rot.apply([0,1.,0])[:2];long=long/np.linalg.norm(long);long*=(-1 if long@src[:2]>0 else 1)
   p=src+np.r_[off*long,-.012+dz]-rot.apply([0,0,.140])
   yield dict(yaw=yaw,quaternion=rot.as_quat().tolist(),position=p.tolist(),height_delta_m=dz,tilt_degrees=tilt,tcp_m=.140,inward_m=off,source_family='outer-inward')
def grasp_variants(scene,color):
 src=xyz(scene,color)
 if color in TS.HIGH_COLORS:
  # Tilt about the jaw-opening axis: keep the opening direction and the
  # upright cube footprint; move the flange, not the measured object target.
  for dz in (.006,0.):
   for tilt in (15.,25.,-15.,-25.):
    for yaw in (-90.,180.,0.,90.):
     R=Rotation.from_quat(quat(yaw))*Rotation.from_euler('x',tilt,degrees=True)
     grip=src+[0,0,-.012+dz]-R.apply([0,0,.140])
     yield {'yaw':yaw,'quaternion':R.as_quat().tolist(),'position':grip.tolist(),
            'height_delta_m':dz,'tilt_degrees':tilt,'tcp_m':.140,'tilt_axis':'jaw_opening'}
 # Rank tilt reducing radial flange reach ahead of the opposite directions.
 axis=np.array([-src[1],src[0],0.]);axis/=max(np.linalg.norm(axis),1e-9)
 for tilt in (0.,-3.,-5.,3.,5.):
  for dz in ((.006,-.006) if tilt==0 else (0.,.006,-.006)):
   for yaw in (180.,0.,-90.,90.):
    R=Rotation.from_rotvec(axis*math.radians(tilt))*Rotation.from_quat(quat(yaw))
    grip=src+[0,0,-.012+dz]-R.apply([0,0,.140])
    yield {'yaw':yaw,'quaternion':R.as_quat().tolist(),'position':grip.tolist(),
           'height_delta_m':dz,'tilt_degrees':tilt,'tcp_m':.140}


def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
def box_corners(lo,hi):return np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
def block_bounds(scene,color):
 p=xyz(scene,color);return p-[.030,.030,height(scene,color)],p+[.030,.030,0.]
def boxes(scene,exclude=(),table_allowance=0.):
 corners=np.array([[x,y] for x in (LAYOUT_MIN[0],LAYOUT_MAX[0]) for y in (LAYOUT_MIN[1],LAYOUT_MAX[1])])
 surface=float(np.max(corners@np.asarray(scene['table']['z_ax_by_c'])[:2]+scene['table']['z_ax_by_c'][2]))
 # Preserve the existing robot mounting cutout at x<110mm. The explicit
 # finger/payload table-plane checks cover the whole expanded task region.
 native_min=np.maximum(LAYOUT_MIN,[.110,-1.]);center=(native_min+LAYOUT_MAX)/2;size=LAYOUT_MAX-native_min
 out=[{'name':'measured_task_table','center_m':[float(center[0]),float(center[1]+.30),surface+.008+table_allowance-.025],'size_m':[float(size[0]),float(size[1]),.05]}]
 # Native FCL dual-base coordinate offset is fixed by the same installed URDF.
 for color in scene['blocks']:
  if color in exclude:continue
  lo,hi=block_bounds(scene,color)
  out.append({'name':color,'center_m':((lo+hi)/2+[0,.30,.008]).tolist(),'size_m':(hi-lo).tolist(),'finger_clearance_tolerance_m':FINGER_GAP_TOLERANCE_M,'finger_clearance_comparison_error_m':FINGER_GAP_COMPARISON_ERROR_M})
 return out

def initial_scene(scene):
 s=copy.deepcopy(scene)
 if set(s['blocks'])!=set(COLORS):raise NoPlan('c-Four requires exactly red, orange, green and yellow')
 for color in COLORS:
  h=height(s,color)
  if not .035<=h<=.095:raise NoPlan('Expected four upright tabletop blocks')
  s['blocks'][color]['height_m']=h
 return s

def goal_scene(scene,anchor,side=-1):
 """Top-centre positions of the goal description with its anchor block at `anchor`.
 Table blocks take the measured table height at their own position; a stacked block
 sits on its support by its own measured height."""
 s=copy.deepcopy(scene);tops={}
 for color in TS.PLACE_ORDER:
  dx,dy=TS.offset_xy(color,side);p=np.asarray([*anchor,0.],float);p[0]+=dx;p[1]+=dy
  if color in SUPPORTS:p[2]=tops[SUPPORTS[color]][2]+height(scene,color)
  else:p[2]=table(scene,p)+height(scene,color)
  tops[color]=p
 for color in COLORS:
  s['blocks'][color]['top_center']['surface_position_m']=tops[color].tolist()
 return s

def assert_goal(scene):
 anchor=xyz(scene,TS.ANCHOR)
 def matches(side):
  return all(np.allclose(xyz(scene,c)[:2]-anchor[:2],TS.offset_xy(c,side),atol=1e-8) for c in COLORS)
 if not any(matches(side) for side in TS.SIDES):raise NoPlan('Goal structure/direction changed')
 for color,support in SUPPORTS.items():
  if abs(xyz(scene,color)[2]-xyz(scene,support)[2]-height(scene,color))>1e-8:raise NoPlan(color+' support relation changed')

def placed_neighbors(scene,color,target):
 """Goal neighbours that already stand one tight pitch from this block's target.
 Returns None, one colour, or a list of colours (the former single-neighbour values are unchanged)."""
 found=[];limit=.003+CORRIDOR_ERROR_M
 for other in TS.goal_neighbors(color):
  d=np.asarray(target[:2],float)-xyz(scene,other)[:2]
  for side in TS.SIDES:
   want=np.asarray(TS.offset_xy(color,side))-TS.offset_xy(other,side)
   if np.linalg.norm(want)<=SPACING+1e-9 and abs(d[0]-want[0])<=limit and abs(d[1]-want[1])<=limit:found.append(other);break
 return None if not found else found[0] if len(found)==1 else found

def is_neighbor(color,assembly_neighbor):
 return color==assembly_neighbor or isinstance(assembly_neighbor,(list,tuple)) and color in assembly_neighbor

def pickup_source_conflicts(scene,color):
 """Necessary free-pickup clearance at its first sample; independent of yaw/height.
 Same nominal payload corners and 5mm XY expansions as _check. Minimum
 free-path allowance is20mm; an extra3mm is a planning preference. Rejecting this state cannot discard a path
 accepted by the same loaded free-flight check.
 """
 lo,hi=block_bounds(scene,color);lo[:2]-=.005;hi[:2]+=.005;issues=[]
 for other in scene['blocks']:
  if other==color:continue
  blo,bhi=block_bounds(scene,other);blo[:2]-=.005;bhi[:2]+=.005
  separation=float(np.max(np.maximum(lo-bhi,blo-hi)))
  if separation<PAYLOAD_EXECUTION_MIN_M:issues.append({'neighbor':other,'separation_m':separation,'required_m':PAYLOAD_EXECUTION_MIN_M})
 return issues

def ranked_orders(scene,target,start_xy,orders=None):
 """Rank the valid layer-by-layer orders by blocking, work and empty travel.
 A heuristic only: every candidate still requires full geometry/IK validation.
 """
 rows=[]
 for order in (TS.task_orders(TS.SIDES[0]) if orders is None else orders):
  pending=copy.deepcopy(scene);pos=np.asarray(start_xy,float);blocked=0;distance=0.;moves=0
  for color in order:
   source=xyz(pending,color);goal=xyz(target,color)
   if np.linalg.norm(source-goal)<.003:continue
   blocked+=len(pickup_source_conflicts(pending,color));moves+=1
   distance+=float(np.linalg.norm(source[:2]-pos)+np.linalg.norm(goal[:2]-source[:2]))
   pos=goal[:2];pending['blocks'][color]=copy.deepcopy(target['blocks'][color])
  rows.append({'order':order,'blocked_pickups':blocked,'moves':moves,'travel_xy_m':distance})
 return sorted(rows,key=lambda row:(row['blocked_pickups'],row['moves'],row['travel_xy_m']))

def placement_yaws(grasp_yaw):
 # Same orientation first; exact quarter turns preserve a cube's footprint.
 return [float((grasp_yaw+delta+180.)%360.-180.) for delta in (0.,90.,-90.,180.)]

def destination_heights(scene,color,target):
 """Necessary free-flight endpoint heights, before incoming route search.
 Derived from the same inflated payload boxes and clearance as _check. This
 only screens the endpoint; every complete route still passes original checks.
 """
 target=np.asarray(target,float);place_z=float(target[2]+.128)
 minimum=place_z+.020;preferred=place_z+.030
 for other in scene['blocks']:
  if other==color:continue
  p=xyz(scene,other)
  xy_gap=float(np.max(np.abs(target[:2]-p[:2])-.070))
  if xy_gap<PAYLOAD_PLAN_FLOOR_M:
   minimum=max(minimum,float(p[2]+height(scene,color)+.128+PAYLOAD_PLAN_FLOOR_M))
   preferred=max(preferred,float(p[2]+height(scene,color)+.128+PAYLOAD_PREFERRED_CLEARANCE_M))
 # Small increments are alternatives, not a compulsory global cruise height.
 fallback_floor=max(place_z+.020,minimum-PAYLOAD_COMPARISON_M)
 buffered=[minimum-drop for drop in (.002,.004) if minimum-drop>=fallback_floor]
 return list(dict.fromkeys([z for z in (.30,.29,.305) if minimum<=z<=preferred+.04]+[preferred,minimum+.002,minimum+.001,minimum+.0005,minimum,preferred+.005]+buffered+[fallback_floor]))

def destination_hovers(scene,color,target,place):
 heights=destination_heights(scene,color,target)
 xy=np.asarray(place[:2],float)
 offset=place[2]-(target[2]+.128)
 norm=np.linalg.norm(xy)
 for h in heights:
  z=max(h+offset,place[2]+.020)
  yield [float(xy[0]),float(xy[1]),z]
  if norm and h<=min(heights)+PAYLOAD_COMPARISON_M+.003:
   # Only the hover moves. The lower path still has to satisfy the original
   # vertical assembly corridor, placement endpoint and native checks.
   inward=xy-.003*xy/norm
   yield [float(inward[0]),float(inward[1]),z]

def layout_candidates(scene,robot):
 """No row/order assumption. Sources, inward shifts, scene centroid and a
 bounded workspace grid; target structure and measured table remain fixed.
 """
 red=xyz(scene,TS.ANCHOR);points=[xyz(scene,c)[:2] for c in COLORS]
 center=np.mean(points,axis=0);anchors=list(points)+[center]
 for p in [red[:2],center]:
  for dx in (-.04,-.08,-.12):
   for dy in (0.,-.04,.04,-.08,.08):anchors.append(p+[dx,dy])
 lo=np.maximum(LAYOUT_MIN+.030,np.asarray(robot['workspace_min'])[:2]);hi=np.minimum(LAYOUT_MAX-.030,np.asarray(robot['workspace_max'])[:2])
 for x in np.arange(lo[0],hi[0]+1e-8,.04):
  for y in np.arange(lo[1],hi[1]+1e-8,.04):anchors.append(np.array([x,y]))
 # Keep established useful seeds as well as the larger workspace-derived grid.
 anchors += [np.array([x,y]) for x in (.18,.22,.26,.28,.30,.32,.34) for y in (-.18,-.12,-.08,-.05,0.)]
 unique={tuple(map(float,a)):a for a in anchors};rows=[]
 for a in unique.values():
  for side in TS.SIDES:
   target=goal_scene(scene,a,side)
   if any(np.any(xyz(target,c)[:2]-[.030,.030]<LAYOUT_MIN-LAYOUT_ERROR_M) or np.any(xyz(target,c)[:2]+[.030,.030]>LAYOUT_MAX+LAYOUT_ERROR_M) for c in target['blocks']):continue
   travel=sum(np.linalg.norm(xyz(scene,c)[:2]-xyz(target,c)[:2]) for c in scene['blocks'])
   # A preference, never a reachability boundary. Full pose/path checks decide.
   reach_penalty=max(0.,float(np.linalg.norm(a))-.34)*2.
   stationary=np.linalg.norm(a-red[:2])<.003
   # Reserve useful room around the still-unmoved green block. This is a
   # ranking preference, not a distance gate; tight layouts remain candidates.
   access_penalty=0.
   for upper in TS.SECOND_LAYER:
    green=xyz(scene,upper)[:2]
    separation=min(float(np.max(np.abs(xyz(target,c)[:2]-green))) for c in BASE_COLORS)
    access_penalty=max(access_penalty,max(0.,.13-separation)*3.)
   # Preserve a low inward corridor for distant pickups before choosing where
   # to assemble the base. Ranking only: no candidate is declared unreachable.
   corridor_penalty=0.
   for source in points:
    radius=float(np.linalg.norm(source))
    if radius<=.43:continue
    corridor=np.linspace(source,source*(.30/radius),9)
    for base in BASE_COLORS:
     separation=float(np.min(np.max(abs(corridor-xyz(target,base)[:2]),axis=1)))
     corridor_penalty=max(corridor_penalty,max(0.,.105-separation))
   score=float(travel+reach_penalty+access_penalty+6*corridor_penalty-(.06 if stationary else 0.))+TS.ROTATION_PENALTY_M[side]
   rows.append(((corridor_penalty>0,not stationary,score),a.tolist(),side))
 ranked=[(a,side) for _,a,side in sorted(rows,key=lambda row:row[0])]
 # Preserve all established layout owners; add the two exact stationary-orange goals at the tail.
 for side in TS.SIDES:
  for fixed in BASE_COLORS[1:]:
   a=xyz(scene,fixed)[:2]-TS.offset_xy(fixed,side); pair=(a.tolist(),side)
   if pair not in ranked and all(np.all(p-.030>=LAYOUT_MIN-LAYOUT_ERROR_M) and np.all(p+.030<=LAYOUT_MAX+LAYOUT_ERROR_M) for p in [a+TS.offset_xy(c,side) for c in BASE_COLORS]):ranked.append(pair)
 return ranked

class Planner:
 def __init__(self,ctx,scene,robot,local_open,local_closed):
  self.ctx=ctx;self.scene=initial_scene(scene);self.robot=robot
  self.local={OPEN:np.asarray(local_open),CLOSED:np.asarray(local_closed)}
  self.diagnostics=[];self.ik_cache={};self.fk_cache={};self.route_cache={};self.route_failures={};self.edge_routes={};self.edge_route_failures={};self.reuse_counts={'edge_success':0,'edge_failure':0,'duplicate_transfer':0};self.path_cache={};self.failed_checks={};self.native_cache={};self.started=time.monotonic();self.progress_at=0.;self.searching={};self.search_active=False;self.ik_mode='full';self.ik_query_counts={'fast':0,'full':0};self.sample_fk_cache={};self.ik_reused_successes=0;self.candidate_deadline=None;self.candidate_yielded=False
  self.defer_native=False;self.lazy_stats={'geometric_candidates':0,'native_candidates':0,'native_rejections':0,'native_certified_actions':0}
 def progress(self,**details):
  if not self.search_active:return
  self.searching.update(details)
  now=time.monotonic()
  if now-self.started>PREFLIGHT_BUDGET_S:
   raise PlanningTimeout(f'Whole-task preflight exceeded{PREFLIGHT_BUDGET_S:g}s before motion; last candidate='+str(self.searching))
  if not getattr(self,'certifying_complete_plan',False) and self.ik_mode=='fast' and now-self.started>FAST_PASS_BUDGET_S and (getattr(self,'candidate_yielded',False) or any(k[0]=='fast' and v is None for k,v in self.ik_cache.items())):
   raise FastPassTimeout('Fast whole-task portfolio budget reached; preserve progress for unresolved KDL')
  if getattr(self,'candidate_deadline',None) is not None and now>self.candidate_deadline:
   raise CandidateBudgetExceeded('Candidate budget exhausted; trying another layout/order')
  if getattr(self,'grasp_deadline',None) is not None and now>self.grasp_deadline:
   raise GraspBudgetExceeded('Grasp family slice exhausted; try another grasp')
  callback=getattr(self.ctx,'_blocks_preflight_event',None)
  if callback and now-self.progress_at>=1:
   callback(dict(self.searching,ik_pass=self.ik_mode,ik_budget_ms=5 if self.ik_mode=='fast' else 100,elapsed_s=now-self.started,ik_queries=dict(self.ik_query_counts),ik_successes_reused=self.ik_reused_successes,unique_fk=len(self.fk_cache),unique_ik=len(self.ik_cache),checked_paths=len(self.path_cache),rejected_paths=len(self.failed_checks)))
   self.progress_at=now
 def fk(self,q):
  from pure_fk import local_kinematics,row
  key=tuple(float(x) for x in q[:6])
  if key not in self.fk_cache:
   if key in getattr(self,'sample_fk_cache',{}):
    value=self.sample_fk_cache[key]
    self.fk_cache[key]=value if 'quaternion_xyzw' in value else row(value['T_base_flange'])
   else:
    kin=local_kinematics(self.ctx,key)
    self.fk_cache[key]=row(kin.fk(list(key))) if kin is not None else self.ctx.value('piper_fk',joints=list(key))
  return self.fk_cache[key]
 def sample_fks(self,points):
  from pure_fk import local_kinematics,batch_rows
  keys=[tuple(float(x) for x in q[:6]) for q in points]
  missing=list(dict.fromkeys(k for k in keys if k not in self.sample_fk_cache and k not in self.fk_cache))
  if missing:
   kin=local_kinematics(self.ctx,missing[0])
   values=batch_rows(kin,missing) if kin is not None else list(fk_many(self.ctx,missing))
   if len(values)!=len(missing):raise ValueError('Incomplete sampled FK batch')
   self.sample_fk_cache.update(zip(missing,values))
  rows=[self.fk_cache[k] if k in self.fk_cache else self.sample_fk_cache[k] for k in keys]
  while len(self.sample_fk_cache)>8192:self.sample_fk_cache.pop(next(iter(self.sample_fk_cache)))
  return rows
 def prepare_full_recheck(self):
  # Promote only validated successes for exactly the same pose+seed. A short
  # refusal remains unresolved and receives a full query when encountered.
  positives=[(k,v) for k,v in self.ik_cache.items() if k[0]=='fast' and v is not None]
  pending=sum(k[0]=='fast' and v is None for k,v in self.ik_cache.items())
  for key,value in positives:self.ik_cache[('full',key[1])]=value
  # Route/prefix failures may depend on unresolved IK: retry these traversals.
  # Pure geometry failures and checked paths do not depend on IK budget.
  for name in ('route_failures','prefix_failures'):
   if hasattr(self,name):getattr(self,name).clear()
  # Resume partially checked routes and live generators. Only exhausted fast
  # traversals need reopening; geometric refusals stay cached.
  for key,state in list(getattr(self,'route_resume',{}).items()):
   if state.get('exhausted'):del self.route_resume[key]
  for job in getattr(self,'transfer_jobs',{}).values():
   for task in job.pop('failed_fast',[]):
    task['generator']=self._family_steps(*job['args'],task['candidate']);task['query_mode']='full'
    job['queue'].append(task)
  self.ik_reused_successes=len(positives)
  return {'validated_ik_reused':len(positives),'unresolved_ik':pending,
          'retained_fk':len(self.fk_cache),'retained_checked_paths':len(self.path_cache),
          'retained_native_paths':len(self.native_cache),'retained_geometry_refusals':len(self.failed_checks)}
 def _solve_candidates(self,p,q,seeds,tail,query_mode=None):
  self.progress()
  if np.any(np.asarray(p)<self.robot['workspace_min']) or np.any(np.asarray(p)>self.robot['workspace_max']):raise NoPlan('Pose outside workspace')
  mode=query_mode or (getattr(self,'query_mode',None) or self.ik_mode if self.search_active else 'full');pending=[];seen=set()
  if mode=='full' and self.search_active:
   fresh=[head for head in seeds if ('fast',tuple(float(v) for v in np.r_[p,q,head[:6]])) not in self.ik_cache]
   if fresh:
    try:self._solve_candidates(p,q,fresh,tail,query_mode='fast')
    except IKUnavailable:pass
   for head in seeds:
    pose=tuple(float(v) for v in np.r_[p,q,head[:6]])
    if self.ik_cache.get(('fast',pose)) is not None:self.ik_cache[('full',pose)]=self.ik_cache[('fast',pose)]
  cached_solution=None
  for seed in seeds:
   head=tuple(float(v) for v in seed[:6])
   if head in seen:continue
   seen.add(head);key=(mode,tuple(float(v) for v in np.r_[p,q,head]))
   if key in self.ik_cache:
    if self.ik_cache[key] is not None:
     if not pending:return list(self.ik_cache[key])+list(tail)
     cached_solution=list(self.ik_cache[key]);break  # test earlier unresolved seeds first
   else:pending.append((key,head))
  if pending:
   r=self.ctx.value('piper_solve_ik_candidates',position=list(map(float,p)),quaternion_xyzw=list(map(float,q)),seeds=[list(head) for _,head in pending],mode=mode)
   from native_fk.geometry import URDF_SHA
   if r.get('executed') is not False or r.get('hardware_writes')!=0 or r.get('frame')!='left_base_link' or r.get('controlled_link')!='left_link6' or r.get('urdf_sha256')!=URDF_SHA:raise ValueError('Batch IK model/frame mismatch')
   rows=r.get('attempts',[])
   if not 1<=len(rows)<=len(pending):raise ValueError('Incomplete batch IK reply')
   for i,row in enumerate(rows):
    key,head=pending[i]
    if row.get('seed')!=list(head):raise ValueError('Batch IK seed identity mismatch')
    self.ik_query_counts[mode]+=1
    if row.get('ok') is True:
     if i!=len(rows)-1 or r.get('ok') is not True:raise ValueError('Batch IK stop contract mismatch')
     solution=np.asarray(row['q'],float)
     if solution.shape!=(6,) or not np.isfinite(solution).all():raise ValueError('Invalid batch IK solution')
     self.ik_cache[key]=solution.tolist();self.remember_solution(p,solution,q);return solution.tolist()+list(tail)
    if row.get('ok') is not False or row.get('reason')!='MOVEIT_NO_IK_SOLUTION':raise ValueError('Unexpected batch IK refusal')
    self.ik_cache[key]=None
   if len(rows)!=len(pending) or r.get('ok') is not False:raise ValueError('Truncated batch IK refusal')
  if cached_solution is not None:return cached_solution+list(tail)
  raise IKUnavailable('No full-pose IK after bounded diverse seeds')
 def remember_solution(self,p,solution,q=None):
  # Index successful seeds once, rather than rescanning thousands of failed
  # pose/seed entries on every fallback. Exact identities; no rounded cache.
  if not hasattr(self,'seed_bank'):
   self.seed_bank=[];self.seed_bank_keys=set();self.seed_bank_arrays=None
  if q is not None:
   if not hasattr(self,'pose_solutions'):self.pose_solutions={}
   pose=tuple(float(v) for v in np.r_[p,q]);values=self.pose_solutions.setdefault(pose,[])
   if list(solution) not in values:values.append(list(solution))
  key=tuple(float(v) for v in np.r_[p,solution])
  if key not in self.seed_bank_keys:
   self.seed_bank_keys.add(key);self.seed_bank.append((list(p),list(solution)));self.seed_bank_arrays=None
 def nearest_seed(self,p):
  if not getattr(self,'seed_bank',None):return None
  if self.seed_bank_arrays is None:
   self.seed_bank_arrays=np.asarray([r[0] for r in self.seed_bank],float)
  delta=self.seed_bank_arrays-np.asarray(p)
  i=int(np.argmin(np.einsum('ij,ij->i',delta,delta)))
  return list(self.seed_bank[i][1])
 def solve(self,p,q,seed):
  self.last_pose_reuse=False
  if self.search_active and not getattr(self,'pose_cache_disabled',False):
   values=getattr(self,'pose_solutions',{}).get(tuple(float(v) for v in np.r_[p,q]),[])
   if values:
    near=min(values,key=lambda v:max(abs(np.asarray(v)-seed[:6])))
    # Preference for an already certified exact pose on a nearby branch, NOT
    # a motion tolerance. Every connecting path still passes all checks.
    if max(abs(np.asarray(near)-seed[:6]))<.2:
     self.progress();self.last_pose_reuse=True
     self.pose_solution_reuses=getattr(self,'pose_solution_reuses',0)+1
     return list(near)+list(seed[6:])
  base=float(math.atan2(p[1],p[0]));pool=[seed[:6]]
  nearest=self.nearest_seed(p)
  if nearest is not None:pool.append(nearest)
  pool += [[base,1.4,-1.,1.2,0.,0.],[base,1.9,-1.8,1.5,0.,0.]]
  if getattr(getattr(self,'ctx',None),'_blocks_batch_ik',False):
   # Original seed and bounded alternatives in one RPC. Server stops on the
   # first success; same query budgets and native solution verification.
   return self._solve_candidates(p,q,pool,seed[6:])
  seen=set()
  for head in pool:
   if tuple(head) in seen:continue
   seen.add(tuple(head))
   try:return self._solve_single(p,q,list(head)+list(seed[6:]))
   except IKUnavailable:pass
  raise IKUnavailable('No full-pose IK after bounded diverse seeds')
 def _solve_single(self,p,q,seed,query_mode=None):

  self.progress()
  if np.any(np.asarray(p)<self.robot['workspace_min']) or np.any(np.asarray(p)>self.robot['workspace_max']):raise NoPlan('Pose outside workspace')
  mode=query_mode or (getattr(self,'query_mode',None) or self.ik_mode if self.search_active else 'full')
  key=(mode,tuple(float(v) for v in np.r_[p,q,seed[:6]]))
  if mode=='full' and self.search_active and getattr(getattr(self,'ctx',None),'_blocks_batch_ik',False):
   fast_key=('fast',key[1])
   if fast_key not in self.ik_cache:
    try:self._solve_single(p,q,seed,query_mode='fast')
    except IKUnavailable:pass
   if self.ik_cache.get(fast_key) is not None:self.ik_cache[key]=self.ik_cache[fast_key]
  if key not in self.ik_cache:
   try:
    self.ik_query_counts[mode]+=1
    r=self.ctx.value('piper_solve_ik_fast' if mode=='fast' else 'piper_solve_ik',position=list(map(float,p)),quaternion_xyzw=list(map(float,q)),seed=list(map(float,seed[:6])))
    if not r.get('ok'):raise NoPlan('No full-pose IK')
    self.ik_cache[key]=r['q'];self.remember_solution(p,r['q'],q)
   except Exception as exc:
    from composition import _planning_refusal
    if isinstance(exc,NoPlan) or _planning_refusal(exc) or type(exc).__name__=='IKNoSolution':self.ik_cache[key]=None
    else:raise
  if self.ik_cache[key] is None:raise IKUnavailable('No full-pose IK')
  return list(self.ik_cache[key])+list(seed[6:])
 def check(self,a,b,scene,**opts):
  self.progress()
  key=digest([a,b,scene['table'],{k:(xyz(scene,k).tolist(),height(scene,k)) for k in scene['blocks']},opts,self.search_active,self.defer_native])
  if key in self.failed_checks:raise NoPlan(self.failed_checks[key])
  try:return self._check(a,b,scene,**opts)
  except NoPlan as exc:self.failed_checks[key]=str(exc);raise
 def _check(self,a,b,scene,*,active=None,payload=None,contact=False,support=None,withdraw=False,assembly_neighbor=None,reserve_m=None,curve14=None,native_result=None,geometry_only=False,pickup=False,orientation_reference14=None,fixed_jaw_feedback_m=None,single_leg_wire=False,released_empty_curve=False):
  """Same samples for planning/execution; native full-arm plus explicit fingers/payload."""
  key=digest([a,b,scene['table'],{k:(xyz(scene,k).tolist(),height(scene,k)) for k in scene['blocks']},active,payload,contact,support,withdraw,assembly_neighbor,reserve_m,curve14,native_result,geometry_only,pickup,orientation_reference14,fixed_jaw_feedback_m,single_leg_wire,released_empty_curve,self.search_active,self.defer_native])
  if key in self.path_cache:return self.path_cache[key]
  check_command_reserve(a,curve14 if curve14 is not None else b)
  if geometry_only and (curve14 is not None or not np.array_equal(a,b)):raise NoPlan('Deferred native check only for a stationary endpoint')
  if curve14 is None and max(abs(np.asarray(b[:6])-a[:6]))>2*math.pi/3+1e-6:raise NoPlan('Segment above120deg')
  if not np.allclose(a[7:],b[7:],atol=1e-12):raise NoPlan('Right side must remain fixed')
  jaw_motion=abs(a[6]-b[6])>1e-9
  if jaw_motion and max(abs(np.asarray(b[:6])-a[:6]))>1e-9:raise NoPlan('Arm/jaw simultaneous path unsupported')
  if released_empty_curve and not (curve14 is not None and active=='green' and payload is None and not jaw_motion and not pickup and (contact or withdraw) and min(a[6],b[6])>=.065-OPEN_JAW_ERROR_M):raise NoPlan('Released empty curve requires open, unloaded green withdrawal')
  if fixed_jaw_feedback_m is not None:
   if not np.isfinite(fixed_jaw_feedback_m) or jaw_motion or not 0<=fixed_jaw_feedback_m<=.076 or abs(fixed_jaw_feedback_m-a[6])>.0005+1e-12:raise NoPlan('Invalid fixed-command jaw feedback envelope')
  def finger_local(gap):return self.local[CLOSED]+(self.local[OPEN]-self.local[CLOSED])*((gap-CLOSED)/(OPEN-CLOSED))
  jaw_envelope=(max(0.,min(a[6],b[6])-.003),min(.076,max(a[6],b[6])+.003)) if jaw_motion and b[6]>a[6] else None
  if payload and np.linalg.norm(np.asarray(self.fk(a)['position'])-self.fk(b)['position'])>.600:raise NoPlan('Loaded segment exceeds600mm')
  count=1 if not jaw_motion and np.array_equal(a[:6],b[:6]) else max(41,math.ceil(float(max(abs(np.asarray(b[:6])-a[:6])))/.002)+1)
  if curve14 is None:qs=[(np.asarray(a[:6])+(np.asarray(b[:6])-a[:6])*u).tolist() for u in np.linspace(0,1,count)]
  if curve14 is not None:
   curve=np.asarray(curve14,float)
   if jaw_motion or ((contact or withdraw) and not (single_leg_wire or released_empty_curve)):raise NoPlan('Contact/withdraw curve requires a certified straight single leg')
   if curve.ndim!=2 or curve.shape[1]!=14 or len(curve)<2 or not np.isfinite(curve).all():raise NoPlan('Invalid sampled curve')
   if not np.allclose(curve[0],a,atol=1e-9) or not np.allclose(curve[-1],b,atol=1e-9):raise NoPlan('Curve endpoints differ')
   if not np.allclose(curve[:,6:],np.asarray(a)[6:],atol=1e-9):raise NoPlan('Curve changes gripper/right side')
   if single_leg_wire:
    delta=np.asarray(b[:6])-a[:6];length2=float(delta@delta)
    alpha=(curve[:,:6]-a[:6])@delta/max(length2,1e-20)
    residual=curve[:,:6]-(np.asarray(a[:6])+alpha[:,None]*delta)
    if np.max(abs(residual))>1e-6 or np.any(alpha < -1e-5) or np.any(alpha>1+1e-5) or np.any(np.diff(alpha)<-1e-5):raise NoPlan('Single-leg wire departs from monotone reviewed segment')
   qs=curve[:,:6].tolist();count=len(qs)
   if not native_result or native_result.get('cad_clear') is not True or native_result.get('workspace_clear') is not True:raise NoPlan('Curve lacks native collision/workspace preview')
  orientation_fact=None
  if orientation_reference14 is not None:
   ref=np.asarray(orientation_reference14,float)
   if ref.shape!=(14,) or not np.isfinite(ref).all() or curve14 is not None or payload or not contact or not active or not np.array_equal(np.asarray(a[:6]),np.asarray(b[:6])) or max(abs(ref[:6]-np.asarray(a[:6])))>.01+1e-12:
    raise NoPlan('Orientation reference requires a bound stationary contact jaw stroke within0.01rad')
  retreat_comparison=bool(withdraw and not contact and not payload and active in scene['blocks']
      and not jaw_motion and min(a[6],b[6])>=.065-OPEN_JAW_ERROR_M)
  retreat_samples=[];retreat_used_m=0.;retreat_allowance_used=False
  min_table=math.inf;min_neighbor=math.inf;min_payload=math.inf
  origin=np.asarray(self.fk(a)['position']);dest=np.asarray(self.fk(b)['position'])
  source_T=np.asarray(self.fk(a)['T_base_flange']);source_tcp=source_T[:3,3]+source_T[:3,:3]@np.array([0.,0.,.140])
  if pickup and (not payload or not active or np.linalg.norm(source_tcp[:2]-xyz(scene,active)[:2])>.012+LOCALIZATION_ERROR_M or contact or withdraw or np.linalg.norm(origin[:2]-dest[:2])>.002+CORRIDOR_ERROR_M or dest[2]<origin[2]-.001):raise NoPlan('Pickup departure must rise from its source column')
  # Block boxes and their two inflation policies are invariant over this one checked path.
  finger_boxes={};loaded_boxes={}
  for c in scene['blocks']:
   lo,hi=block_bounds(scene,c);blo,bhi=lo.copy(),hi.copy()
   if not contact and not withdraw and payload is None and c!=active:lo[:2]-=.01243;hi[:2]+=.01243
   if not contact:blo[:2]-=.005;bhi[:2]+=.005
   finger_boxes[c]=(lo,hi);loaded_boxes[c]=(blo,bhi)
  # Geometry of every sample at once. Batched matmul, min/max and elementwise operations are
  # bit-identical to the former per-sample calls, so each comparison below sees exactly the
  # same numbers, in the same order, with the same first refusal and the same message.
  fks=[self.fk(a)] if count==1 else self.sample_fks(qs)
  us=np.linspace(0,1,count);n=len(us)
  Ts=np.asarray([np.asarray(f['T_base_flange']) for f in fks],float)
  Rt=np.swapaxes(Ts[:,:3,:3],1,2);ps=Ts[:,:3,3]
  jaws=jaw_envelope[0]+us*(jaw_envelope[1]-jaw_envelope[0]) if jaw_envelope else a[6]+us*(b[6]-a[6])
  locals_=self.local[CLOSED][None]+(self.local[OPEN]-self.local[CLOSED])[None]*((jaws-CLOSED)/(OPEN-CLOSED))[:,None,None]
  finger_vertices=8
  if fixed_jaw_feedback_m is not None:
   # Bound both pads over the entire linear-prismatic feedback interval.
   fixed=finger_local(fixed_jaw_feedback_m).reshape(2,8,3)
   locals_=np.concatenate((locals_.reshape(n,2,8,3),np.broadcast_to(fixed,(n,2,8,3))),axis=2).reshape(n,-1,3)
   finger_vertices=16
  fingers_all=np.matmul(locals_,Rt)+ps[:,None,:]
  table_gaps=(fingers_all[...,2]-fingers_all[...,:2]@np.asarray(scene['table']['z_ax_by_c'])[:2]-scene['table']['z_ax_by_c'][2]).min(axis=1)
  split_all=fingers_all.reshape(n,-1,finger_vertices,3);fmins=split_all.min(axis=2);fmaxs=split_all.max(axis=2)
  finger_blocks=[c for c in scene['blocks'] if not (c==active and (contact or payload))]
  finger_seps={c:np.maximum(fmins-finger_boxes[c][1],finger_boxes[c][0]-fmaxs).max(axis=2) for c in finger_blocks}
  if payload:
   pcs=np.matmul(np.asarray(payload['corners_flange'])[None],Rt)+ps[:,None,:];plos=pcs.min(axis=1);phis=pcs.max(axis=1)
   grow=np.array([.005,.005,0.]);plos_grown=plos-grow;phis_grown=phis+grow
   payload_seps={};support_xy={}
   for c in scene['blocks']:
    if c==active:continue
    blo,bhi=loaded_boxes[c]
    payload_seps[c]=(np.maximum(plos_grown-bhi,blo-phis_grown).max(axis=1),np.maximum(plos-bhi,blo-phis).max(axis=1))
    support_xy[c]=(np.maximum((plos_grown[:,:2]-bhi[:2]).max(axis=1),(blo[:2]-phis_grown[:,:2]).max(axis=1)),
                   np.maximum((plos[:,:2]-bhi[:2]).max(axis=1),(blo[:2]-phis[:,:2]).max(axis=1)))
  for i,u in enumerate(us):
   T=Ts[i];p=ps[i]
   if np.any(p<self.robot['workspace_min']) or np.any(p>self.robot['workspace_max']):raise NoPlan('Sample outside workspace')
   gap=float(table_gaps[i]);min_table=min(min_table,gap)
   if gap<MIN_TABLE+(0 if contact or payload or withdraw else .020)-EXTRA_GEOMETRY_ERROR_M:raise NoPlan('Finger/table path clearance')
   if contact or payload:
    orientation_fact=orientation_evidence(T,active)
   if contact and not jaw_motion:
    if np.linalg.norm(origin[:2]-dest[:2])<.002+CORRIDOR_ERROR_M and np.linalg.norm(p[:2]-origin[:2])>.003+CORRIDOR_ERROR_M:raise NoPlan('Vertical contact path bows outside nominal3mm +2mm corridor')
   if contact and active and not payload:
    tcp=p+T[:3,:3]@np.array([0.,0.,.140])
    if np.linalg.norm(tcp[:2]-xyz(scene,active)[:2])>.012+LOCALIZATION_ERROR_M:raise NoPlan('Contact path outside nominal12mm +3mm center corridor')
   retreat_gap=math.inf
   for c in finger_blocks:
    allowance=.003 if contact or payload or withdraw or c==active else .020
    # Planning and execution use the same minimum. Extra source clearance is
    # a route-ranking preference below, never another acceptance threshold.
    for sep in finger_seps[c][i].tolist():
     min_neighbor=min(min_neighbor,sep)
     scoped=retreat_comparison and c==active
     if scoped:retreat_gap=min(retreat_gap,sep)
     if sep<=0. or sep<finger_gap_minimum(allowance):
      if scoped and sep>=-RELEASED_RETREAT_AABB_ERROR_M:
       retreat_allowance_used=True
       retreat_used_m=max(retreat_used_m,max(0.,-sep))
      else:raise NoPlan(f'Finger/{c} path conflict gap={sep*1000:.6f}mm minimum={finger_gap_minimum(allowance)*1000:.6f}mm nominal={allowance*1000:.3f}mm')
   if retreat_comparison:
    center=xyz(scene,active)-np.array([0.,0.,height(scene,active)/2])
    tcp=p+T[:3,:3]@np.array([0.,0.,.140])
    retreat_samples.append((float(np.linalg.norm(tcp-center)),retreat_gap))
   if payload:
    lo=plos[i]
    bottom=lo[2]-table(scene,lo)
    if bottom<(-.002 if contact or pickup else .004-EXTRA_GEOMETRY_ERROR_M):raise NoPlan('Payload/table clearance')
    if pickup and bottom<.010 and np.linalg.norm(p[:2]-origin[:2])>.003+CORRIDOR_ERROR_M:raise NoPlan('Pickup must clear table before lateral transfer')
    for c in scene['blocks']:
     if c==active:continue
     blo,bhi=loaded_boxes[c]
     # Existing70mm loaded envelope/20mm free-flight allowance;
     # deliberate assembly contact uses the rehearsed nominal support boxes.
     # Deliberate final assembly uses nominal block faces, like A's placement
     # planner. Inflated70mm transport envelopes would forbid the close base pair.
     # Apply only to the declared neighbor in a vertical placement corridor;
     # free transport and unrelated objects retain their full envelope/margin.
     adjacent_assembly=bool(contact and is_neighbor(c,assembly_neighbor) and c in TS.goal_neighbors(active))
     if adjacent_assembly:
      if np.linalg.norm(origin[:2]-dest[:2])>.002+CORRIDOR_ERROR_M or np.linalg.norm(p[:2]-origin[:2])>.003+CORRIDOR_ERROR_M:
       raise NoPlan('Assembly proximity is only valid in a vertical placement corridor')
     variant=1 if adjacent_assembly else 0
     sep=float(payload_seps[c][variant][i])
     if c==support and contact and float(support_xy[c][variant][i])<0:
      if lo[2]<bhi[2]-.002:raise NoPlan('Payload penetrates support')
     else:
      if not contact:min_payload=min(min_payload,sep)
      # One execution minimum in planning, batch and single-leg checks.
      # reserve_m remains a compatibility hint, never a second hard limit.
      required=MARGIN if adjacent_assembly else .001 if contact else PAYLOAD_MIN_CLEARANCE_M
      minimum=clearance_minimum(required) if contact else PAYLOAD_EXECUTION_MIN_M
      if sep<minimum:raise NoPlan(f'Payload/{c} path conflict: separation={sep*1000:.3f}mm minimum={minimum*1000:.3f}mm nominal={required*1000:.3f}mm, contact={contact}, at_sample={u:.3f}')
  retreat_progress=retreat_progress_evidence(retreat_samples) if retreat_allowance_used else None
  if geometry_only:result={'deferred_to_full_path':True}
  elif curve14 is not None:result=native_result
  else:
   obstacles=boxes(scene,(active,) if active and (contact or payload) else (),table_allowance=.032 if not (contact or payload or withdraw) else 0.)
   # Exact same full14 start/goal and obstacle/geometry identity only. Reserve
   # changes still run the payload/finger checks above; never round anchors.
   native_a=list(a);native_b=list(b)
   if jaw_envelope:native_a[6],native_b[6]=jaw_envelope
   if fixed_jaw_feedback_m is not None:native_a[6]=fixed_jaw_feedback_m
   native_key=digest([native_a,native_b,obstacles,self.robot.get('geometry_identity'),self.robot.get('owner_status',{}).get('generation')])
   pending=None
   if native_key not in self.native_cache and self.defer_native:
    result={'deferred':True,'collision_checked':False,'hardware_execution_approved':False}
    pending={'key':native_key,'arguments':{'start14':native_a,'target14':native_b,'obstacles':obstacles}}
   elif native_key not in self.native_cache:
    self.native_cache[native_key]=self.ctx.value('piper_check_path',start14=native_a,target14=native_b,obstacles=obstacles)
    result=self.native_cache[native_key]
   else:result=self.native_cache[native_key]
  if not geometry_only and not result.get('deferred') and (result.get('cad_clear') is not True or result.get('workspace_clear') is not True):raise NativePathRefusal('Native CAD/workspace refusal: '+str(result.get('contacts',[]))[:250])
  row={'fixed_command_jaw_feedback_m':fixed_jaw_feedback_m,'jaw_feedback_envelope_m':jaw_envelope,'numeric_policy':policy_record(),'finger_gap_policy':{'nominal_m':MARGIN,'tolerance_m':FINGER_GAP_TOLERANCE_M,'comparison_error_m':FINGER_GAP_COMPARISON_ERROR_M,'minimum_m':finger_gap_minimum(MARGIN),'overlap_allowed':False,'native_comparison_error_m':FINGER_GAP_COMPARISON_ERROR_M},'min_payload_neighbor_m':None if min_payload==math.inf else min_payload,'samples':count,'native':result,'min_finger_table_m':min_table,'min_neighbor_m':None if min_neighbor==math.inf else min_neighbor}
  if retreat_comparison:row['released_retreat_comparison']={'limit_m':RELEASED_RETREAT_AABB_ERROR_M,
      'used_m':retreat_used_m,'target':active,'progress':retreat_progress,
      'scope':'Only simplified finger AABB vs released target; native obstacle and collision margins unchanged',
      'native_result_field':'native','native_certification_required':True}
  if orientation_fact is not None:row['orientation']=orientation_fact
  if payload and not contact:row['payload_clearance']=payload_clearance_fact(row['min_payload_neighbor_m'])
  if result.get('deferred'):row['_pending_native']=pending
  self.path_cache[key]=row;return row
 def certify_moves(self,moves):
  """LazyPRM-style validation of a COMPLETE geometric task candidate.

  Exact FK, fingers, payloads, table and workspace were checked on every leg.
  No candidate may leave this method as executable with a deferred native edge.
  Native failures remain cached; rebuilding then cannot pick that same edge.
  """
  self.lazy_stats['native_candidates']+=1
  checks=[a['check'] for m in moves for a in m['actions']]
  # Feedback-envelope paths are part of the complete candidate, including
  # their deferred native checks. Never certify just the nominal 73mm route.
  for move in moves:
   for action in move['actions']:
    checks.extend(action.get('release_feedback_preflight',{}).get('checks',[]))
    if 'next_pickup_check' in action:checks.append(action['next_pickup_check'])
  for check in checks:
   request=check.get('_pending_native')
   if request:
    self.progress(phase='native_candidate_validation')
    key=request['key']
    if key not in self.native_cache:
     self.native_cache[key]=self.ctx.value('piper_check_path',**request['arguments'])
    result=self.native_cache[key]
    if result.get('cad_clear') is not True or result.get('workspace_clear') is not True:
     self.lazy_stats['native_rejections']+=1
     # Derived routes may embed this rejected edge. Keep exact IK/FK and native
     # evidence; discard only candidate caches which can contain stale routes.
     for name in ('route_cache','route_failures','edge_routes','edge_route_failures','prefix_cache','prefix_failures','path_cache','transfer_jobs','finish_cache','route_resume'):
      if hasattr(self,name):getattr(self,name).clear()
     raise NativePathRefusal('Native CAD/workspace refusal after complete geometry: '+str(result.get('contacts',[]))[:250])
    check['native']=result;check.pop('_pending_native')
   result=check.get('native',{})
   if result.get('cad_clear') is not True or result.get('workspace_clear') is not True or result.get('deferred'):
    raise RuntimeError('Complete candidate lacks native certification; no execution')
  action_count=sum(len(m['actions']) for m in moves)
  self.lazy_stats['native_certified_actions']+=action_count
  self.lazy_stats['native_certified_feedback_paths']=self.lazy_stats.get('native_certified_feedback_paths',0)+len(checks)-action_count
 def certify_complete_plan(self,moves):
  # Search timeslices end when a complete geometric plan exists. Mandatory
  # certification still obeys the unchanged90s global deadline at every edge.
  saved_deadline=self.candidate_deadline
  self.candidate_deadline=None;self.certifying_complete_plan=True
  try:self.certify_moves(moves)
  finally:self.candidate_deadline=saved_deadline;self.certifying_complete_plan=False
 def joint_route(self,start,end,scene,**opts):
  n=max(1,math.ceil(float(max(abs(np.asarray(end[:6])-start[:6])))/(2*math.pi/3*.95)));points=[];a=list(start)
  for u in np.linspace(0,1,n+1)[1:]:
   b=(np.asarray(start[:6])+(np.asarray(end[:6])-start[:6])*u).tolist()+list(start[6:])
   check=self.check(a,b,scene,**opts);points.append({'start14':a,'target14':b,'check':check});a=b
  return points
 def connect_edge(self,a,dest,q,scene,**opts):
  if not hasattr(self,'edge_routes'):self.edge_routes={};self.edge_route_failures={};self.reuse_counts={'edge_success':0,'edge_failure':0,'duplicate_transfer':0}
  key=digest([a,np.asarray(dest).tolist(),np.asarray(q).tolist(),scene.get('table'),
              {c:(xyz(scene,c).tolist(),height(scene,c)) for c in scene.get('blocks',{})},opts,self.search_active,getattr(self,'defer_native',False)])
  failure_key=(getattr(self,'query_mode',None) or getattr(self,'ik_mode','full'),key)
  if key in self.edge_routes:
   self.reuse_counts['edge_success']+=1;return copy.deepcopy(self.edge_routes[key])
  if failure_key in self.edge_route_failures:
   self.reuse_counts['edge_failure']+=1;raise NoPlan(self.edge_route_failures[failure_key])
  reused=False
  try:
   b=self.solve(dest,q,a);reused=getattr(self,'last_pose_reuse',False)
   check_command_reserve(a,b)
   # An occupied endpoint cannot be repaired by a different incoming curve.
   # Reject it before sampling/subdividing every alternative path to it.
   self.check(b,b,scene,geometry_only=True,**opts)
   try:parts=self.joint_route(a,b,scene,**opts)
   except NoPlan as original_error:
    # A joint-line can bow through an obstacle even when its Cartesian
    # corridor is clear. Subdivide that corridor, retaining every native and
    # payload check; never smooth an unchecked curve around the refusal.
    pose=self.fk(a);p0=np.asarray(pose['position']);q0=pose['quaternion_xyzw']
    rotation=Slerp([0,1],Rotation.from_quat([q0,q]))
    for n in (2,4,8):
     sub=list(a);trial=[]
     try:
      for u in np.linspace(0,1,n+1)[1:]:
       middle=self.solve((p0+(np.asarray(dest)-p0)*u).tolist(),rotation([u])[0].as_quat().tolist(),sub)
       trial+=self.joint_route(sub,middle,scene,**opts);sub=middle
      parts=trial;b=sub;break
     except NoPlan:pass
    else:raise original_error
   self.edge_routes[key]=(parts,b);return copy.deepcopy((parts,b))
  except NoPlan as exc:
   if reused and not getattr(self,'pose_cache_disabled',False):
    self.pose_cache_disabled=True
    self.pose_solution_retries=getattr(self,'pose_solution_retries',0)+1
    try:return self.connect_edge(a,dest,q,scene,**opts)
    finally:self.pose_cache_disabled=False
   self.edge_route_failures[failure_key]=str(exc);raise

 def route(self,start,position,quaternion,scene,**opts):
  self.progress()
  key=digest([start,np.asarray(position).tolist(),np.asarray(quaternion).tolist(),scene['table'],{c:(xyz(scene,c).tolist(),height(scene,c)) for c in scene['blocks']},opts,self.search_active])
  failure_key=(getattr(self,'query_mode',None) or getattr(self,'ik_mode','fast'),key)
  if failure_key in self.route_failures:raise NoPlan(self.route_failures[failure_key])
  if key not in self.route_cache:
   try:self.route_cache[key]=self._route(start,position,quaternion,scene,**opts)
   except NoPlan as exc:self.route_failures[failure_key]=str(exc);raise
  return copy.deepcopy(self.route_cache[key])
 def _route(self,start,position,quaternion,scene,*,active=None,payload=None,contact=False,support=None,withdraw=False,assembly_neighbor=None,pickup=False):
  # Every route includes its start. A detour cannot repair an already invalid
  # loaded start; validate it once at the least permitted free-path margin.
  if payload and not contact:
   self.check(start,start,scene,active=active,payload=payload,pickup=pickup,support=support,withdraw=withdraw,reserve_m=.003,geometry_only=True)
  # Prefer direct; then bounded inward corridors, without changing goal direction.
  p=np.asarray(self.fk(start)['position']);dst=np.asarray(position);options=[[]];strategies={id(options[0]):'direct_checked'}
  if payload is None and not (contact or withdraw or pickup):
   # A free endpoint above the previous stack does not imply that turning
   # immediately there is free. Translate with its current orientation first,
   # then turn toward the next source; every edge keeps the same checks.
   current=self.fk(start)['quaternion_xyzw']
   if np.asarray(self.fk(start)['T_base_flange'])[2,2]<-DOWN_COS and abs(float(np.dot(current,quaternion)))<.99999:
    # After stacking, a free endpoint may be beside (not above) the stack.
    # Keep its orientation while clearing the stack before turning toward
    # the next pickup. The original XY portals alone can bow into that stack.
    rotations=[Rotation.from_quat(current).as_matrix(),Rotation.from_quat(quaternion).as_matrix()]
    finger_bottom=min(float((self.local[OPEN]@r.T)[:,2].min()) for r in rotations)
    nearby=[block_bounds(scene,c)[1][2] for c in scene.get('blocks',{}) if np.linalg.norm(xyz(scene,c)[:2]-p[:2])<.18]
    clear_z=max(nearby,default=table(scene,p))-finger_bottom+finger_gap_minimum(.020)+.005
    heights=list(dict.fromkeys([min(max(float(p[2])+.010,clear_z),self.robot['workspace_max'][2]-.002),float(p[2])+.020,float(p[2])+.040]))
    for z in heights:
     if not p[2]<z<=self.robot['workspace_max'][2]:continue
     lift=[float(p[0]),float(p[1]),float(z)]
     for via in ([(lift,current)],[(lift,current),(lift,quaternion)]):
      options.append(via);strategies[id(via)]='clear_stack_up_before_turn'
    directions=[dst[:2]-p[:2],np.array([1.,0.]),np.array([-1.,0.]),np.array([0.,1.]),np.array([0.,-1.])]
    directions += [p[:2]-xyz(scene,c)[:2] for c in scene.get('blocks',{})]
    portals=[];seen_portals=set()
    for direction in directions:
     norm=float(np.linalg.norm(direction))
     if norm<1e-6:continue
     direction=direction/norm
     for distance in (.04,.08,.12,.16,.20):
      point=p.copy();point[:2]+=direction*distance
      if np.any(point<self.robot['workspace_min']) or np.any(point>self.robot['workspace_max']):continue
      key=tuple(np.round(point,6))
      if key in seen_portals:continue
      seen_portals.add(key)
      portals.append((float(np.linalg.norm(point-p)+np.linalg.norm(dst-point)),point.tolist()))
    for _,point in sorted(portals,key=lambda row:row[0]):
     for via in ([(point,current)],[(point,current),(point,quaternion)]):
      options.append(via);strategies[id(via)]='clear_stack_before_turn'
  if payload and not contact and np.linalg.norm(dst[:2]-p[:2])>.002:
   high=max(float(p[2]),float(dst[2]))
   def prefer(label,via):
    points=[];previous=p
    for v in via:
     v=list(map(float,v))
     if np.linalg.norm(np.asarray(v)-previous)>1e-8:points.append(v);previous=np.asarray(v)
    if points:options.append(points);strategies[id(points)]=label
   # Keep initial straight candidate first. Then lift-and-cross / oblique climb
   # within the source-destination corridor, before any fixed inward detour.
   # Derive low-to-high staging points outside each real obstacle envelope.
   # These are candidates only; complete payload/finger/native checks still run.
   if scene.get('blocks') and payload.get('corners_flange'):
    rotated=np.asarray(payload['corners_flange'])@Rotation.from_quat(quaternion).as_matrix().T
    for other in scene['blocks']:
     if other==active:continue
     lo,hi=block_bounds(scene,other)
     for axis in (0,1):
      for side in (-1,1):
       xy=dst[:2].copy()
       xy[axis]=(lo[axis]-rotated[:,axis].max()-.010-PAYLOAD_PREFERRED_CLEARANCE_M if side<0 else
                 hi[axis]-rotated[:,axis].min()+.010+PAYLOAD_PREFERRED_CLEARANCE_M)
       if np.any(xy<np.asarray(self.robot['workspace_min'])[:2]) or np.any(xy>np.asarray(self.robot['workspace_max'])[:2]):continue
       prefer('obstacle_side_then_climb',[[*xy,p[2]],[*xy,high]])
       turn=p[:2].copy();turn[axis]=xy[axis]
       prefer('obstacle_dogleg_then_climb',[[*turn,p[2]],[*xy,p[2]],[*xy,high]])
   if np.linalg.norm(p[:2])>.35:
    directions=[dst[:2]-p[:2],-p[:2]]
    for direction in directions:
     length=float(np.linalg.norm(direction))
     if length<1e-6:continue
     direction=direction/length
     for distance in (.025,.05,.08):
      xy=p[:2]+direction*min(distance,length)
      for rise in (0.,.01):
       prefer('inward_before_climb',[[*xy,min(high,p[2]+rise)]])
   prefer('raise_then_translate',[[p[0],p[1],high]])
   prefer('translate_then_raise',[[dst[0],dst[1],p[2]]])
   for u in (.25,.5,.75):
    xy=p[:2]+(dst[:2]-p[:2])*u
    for z in (high,p[2]+(high-p[2])*u):
     prefer('progressive_climb',[[*xy,z]])
   for extra in (.005,.010):
    prefer('raise_then_translate',[[p[0],p[1],high+extra],[dst[0],dst[1],high+extra]])
  target_tcp=dst+Rotation.from_quat(quaternion).apply([0.,0.,.140])
  if payload is None and not (contact or withdraw) and active and target_tcp[2]-xyz(scene,active)[2]<.039 and np.linalg.norm(target_tcp[:2]-xyz(scene,active)[:2])<=.012+LOCALIZATION_ERROR_M:
   # Enter an open jaw along its length, not laterally through a finger. This
   # is especially useful when far-side IK requires a low source hover.
   halves=self.local[OPEN].reshape(2,8,3).mean(axis=1)
   opening=Rotation.from_quat(quaternion).apply(halves[1]-halves[0])
   direction=np.array([-opening[1],opening[0],0.]);direction/=np.linalg.norm(direction)
   corridors=[(dst+direction*d).tolist() for d in (.05,-.05,.08,-.08)]
   corridors.sort(key=lambda v:np.linalg.norm(np.asarray(v)-p))
   # Stage above neighbours on the inward side, then approach along finger
   # length. The far-side hover may be reachable only at low Z. These are
   # ordering hints; actual IK, every sampled leg and native checks still apply.
   rotated=self.local[OPEN]@Rotation.from_quat(quaternion).as_matrix().T
   top=max((block_bounds(scene,c)[1][2] for c in scene['blocks'] if c!=active),default=table(scene,dst))
   # A portal only needs its own obstacle clearance. Carrying the preceding
   # retreat height into it can make every far-side portal fail IK again.
   safe_z=max(float(dst[2]),float(top-rotated[:,2].min()+clearance_minimum(.020)+.005))
   portals=[]
   for d in (.05,-.05,.055,-.055,.060,-.060,.070,-.070,.08,-.08,.11,-.11):
    low=dst+direction*d;high=low.copy();high[2]=safe_z
    if np.any(high<self.robot['workspace_min']) or np.any(high>self.robot['workspace_max']):continue
    # Rank using the same full finger envelope at both portals. Not a gate:
    # approximate Cartesian clearance never replaces sampled FK/native checks.
    deficit=0.
    for point in (high,low):
     fingers=rotated+point
     for c in scene['blocks']:
      lo,hi=block_bounds(scene,c);margin=clearance_minimum(.003 if c==active else .020)
      for finger in fingers.reshape(2,8,3):
       sep=float(np.max(np.maximum(finger.min(0)-hi,lo-finger.max(0))))
       deficit=max(deficit,margin-sep)
    length=float(np.linalg.norm(high-p)+np.linalg.norm(high-low)+np.linalg.norm(low-dst))
    portals.append(((deficit>1e-8,max(0.,deficit),length),high.tolist(),low.tolist()))
   # Preserve the rehearsed rotation-from-Prepare entry. This alternative
   # handles a down-facing hand transitioning between objects only.
   already_down=np.asarray(self.fk(start)['T_base_flange'])[2,2] < -DOWN_COS
   for _,high,low in (sorted(portals,key=lambda item:item[0]) if already_down else []):
    for via in ([high],[high,low]):
     options.append(via);strategies[id(via)]='low_entry_high_portal'
    # Leave the previous placed object vertically before lowering toward a
    # distant low portal. Direct joint interpolation can dip through it even
    # when both endpoint poses are clear. These are fully checked alternatives.
    current_q=self.fk(start)['quaternion_xyzw']
    for rise in (.005,.010,.020):
     lift=[float(p[0]),float(p[1]),float(p[2])+rise]
     via=[(lift,current_q),high,low];options.append(via);strategies[id(via)]='clear_previous_then_low_portal'
   options += [[[.18,p[1],p[2]],[.18,v[1],v[2]],v] for v in corridors]
   options += [[v] for v in corridors]
  if contact or withdraw:
   for n in (2,4,8):options.append([(p+(dst-p)*u).tolist() for u in np.linspace(0,1,n+1)[1:-1]])
  if not contact and not pickup:
   xs=[float(p[0]-d) for d in (.01,.02,.04,.06)]+[.18,.14,.22] if payload else [.18,.14,.22]
   for x in dict.fromkeys(xs):
    for z in sorted(set([float(p[2]),float(dst[2]),.27,.29])):
     options.append([[x,p[1],p[2]],[x,dst[1],z]])
  if payload is None and not contact:
   current=self.fk(start)['quaternion_xyzw']
   rots=Rotation.from_quat([current,quaternion]);mid=Slerp([0,1],rots)([.5])[0].as_quat().tolist()
   options.extend([ [([.22,-.06,.30],mid)] ])
  if payload and not (contact or pickup):
   current_q=self.fk(start)['quaternion_xyzw']
   if abs(float(np.dot(current_q,quaternion)))<.99999:
    # Keep the grasp orientation while leaving a low/far source, then turn
    # near the destination. Turning during the first low leg can sweep the
    # payload into neighbours even when its center corridor is clear.
    held=[]
    for via in options[1:]:
     if any(isinstance(v,tuple) for v in via):continue
     preserved=[(v,current_q) for v in via]
     held.append(preserved);strategies[id(preserved)]=strategies.get(id(via),'free_routing')+'_turn_late'
    options=[options[0]]+held+options[1:]
  if payload and not (contact or pickup) and len(options)>2 and any(c!=active for c in scene.get('blocks',{})):
   # Rank alternatives using cheap Cartesian payload envelopes. This is only
   # an ordering hint: joint/FCL sampling below remains mandatory for every leg.
   # The direct candidate is always attempted first.
   rotated=np.asarray(payload['corners_flange'])@Rotation.from_quat(quaternion).as_matrix().T
   rlo,rhi=rotated.min(0),rotated.max(0)
   others=[block_bounds(scene,c) for c in scene['blocks'] if c!=active]
   def route_hint(via):
    nodes=[p]+[np.asarray(v[0] if isinstance(v,tuple) else v,float) for v in via]+[dst]
    length=sum(float(np.linalg.norm(b-a)) for a,b in zip(nodes,nodes[1:]))
    pts=np.concatenate([np.linspace(a,b,9) for a,b in zip(nodes,nodes[1:])])
    lo=pts+rlo;hi=pts+rhi;lo[:,:2]-=.005;hi[:,:2]+=.005
    deficit=0.
    for blo,bhi in others:
     blo=blo.copy();bhi=bhi.copy();blo[:2]-=.005;bhi[:2]+=.005
     gaps=np.max(np.maximum(lo-bhi,blo-hi),axis=1)
     deficit=max(deficit,float(max(0.,PAYLOAD_EXECUTION_MIN_M-min(gaps))))
    return (deficit>1e-8,deficit,length)
   options=[options[0]]+sorted(options[1:],key=route_hint)
  unique=[];seen=set()
  for via in options:
   key=digest([(np.asarray(v[0]).tolist(),np.asarray(v[1]).tolist()) if isinstance(v,tuple) else (np.asarray(v).tolist(),np.asarray(quaternion).tolist()) for v in via])
   if key not in seen:unique.append(via);seen.add(key)
  options=unique
  if not hasattr(self,'route_resume'):self.route_resume={}
  resume_key=digest([self.search_active,start,position,quaternion,scene,active,payload,contact,support,withdraw,assembly_neighbor,pickup,getattr(self,'defer_native',False)])
  state=self.route_resume.setdefault(resume_key,{'index':0,'leg':0,'rows':[],'a':list(start),'errors':[],'best':None,'score':None,'preferred_start':None,'extra':0})
  if state['index'] or state['leg']:
   if hasattr(self,'search_resume_counts'):self.search_resume_counts['route']+=1
  errors=state['errors'];best=state['best'];best_score=state['score'];preferred_search_start=state['preferred_start'];extra_options=state['extra']
  for index in range(state['index'],len(options)):
   via=options[index]
   if best is not None:
    # A valid route exists: try at most three nearby alternatives or1s of
    # additional search between candidates, then keep the checked route.
    if extra_options>=3 or time.monotonic()-preferred_search_start>=1.:return best
    extra_options+=1
   a=list(state['a']);rows=list(state['rows'])
   try:
    destinations=[]
    for v in via:
     if isinstance(v,tuple):destinations.append(v)
     else:destinations.append((v,quaternion))
    destinations.append((position,quaternion))
    opts=dict(active=active,payload=payload,contact=contact,support=support,withdraw=withdraw,assembly_neighbor=assembly_neighbor,pickup=pickup)
    if payload and not contact and self.search_active:opts['reserve_m']=.003 if id(via) in strategies else .005
    for leg in range(state['leg'],len(destinations)):
     dest,q=destinations[leg]
     parts,b=self.connect_edge(a,dest,q,scene,**opts)
     rows+=parts;a=b
     state.update(leg=leg+1,rows=list(rows),a=list(a))
    for row in rows:
     row['route_strategy']=strategies.get(id(via),'free_routing')
     row['planning_reserve_m']=opts.get('reserve_m',0.)
    # Same-stage free points have no observation/contact dependencies. Try a
    # longer checked leg first, retaining original points when it fails.
    if not (contact or withdraw or pickup) and len(rows)>1:
     short=[];i=0
     while i<len(rows):
      chosen=rows[i];j=i
      for k in range(len(rows)-1,i,-1):
       try:
        check=self.check(rows[i]['start14'],rows[k]['target14'],scene,**opts)
        if payload:
         span=[r['check'].get('min_payload_neighbor_m') for r in rows[i:k+1] if r['check'].get('min_payload_neighbor_m') is not None]
         shortened=check.get('min_payload_neighbor_m')
         if span and shortened is not None and shortened<min(min(span),PAYLOAD_PREFERRED_CLEARANCE_M):continue
        chosen=dict(rows[i],target14=rows[k]['target14'],check=check,shortcut_removed=k-i);j=k;break
       except NoPlan:pass
      short.append(chosen);i=j+1
     rows=short
    prefer_payload=bool(payload and not contact and self.search_active)
    prefer_source=bool(payload is None and active and not (contact or withdraw) and self.search_active)
    if not (prefer_payload or prefer_source):return rows
    metric='min_payload_neighbor_m' if prefer_payload else 'min_neighbor_m'
    preferred=PAYLOAD_PREFERRED_CLEARANCE_M if prefer_payload else finger_gap_minimum(MARGIN)+.001
    gaps=[r['check'].get(metric) for r in rows if r['check'].get(metric) is not None]
    gap=min(gaps) if gaps else float('inf')
    for row in rows:
     row['clearance_policy']=payload_clearance_fact(None if not gaps else gap) if prefer_payload else {'minimum_m':finger_gap_minimum(MARGIN),'preferred_m':preferred,'observed_model_gap_m':gap,'preference_only':True}
    if gap>=preferred:return rows
    distance=sum(float(np.linalg.norm(np.asarray(r['target14'][:6])-r['start14'][:6])) for r in rows)
    score=(-gap,distance)
    if best_score is None or score<best_score:best=rows;best_score=score
    if preferred_search_start is None:preferred_search_start=time.monotonic()
    state.update(best=best,score=best_score,preferred_start=preferred_search_start,extra=extra_options)
   except GraspBudgetExceeded:
    if best is not None:return best
    raise
   except NoPlan as exc:errors.append(str(exc))
   state.update(index=index+1,leg=0,rows=[],a=list(start))
  if best is not None:return best
  state['exhausted']=True
  raise NoPlan('No bounded route: '+','.join(list(dict.fromkeys(errors))[:4]))
 def source_approaches(self,scene,color,yaw,offsets,grasp=None):
  """Candidate geometry only. Every incoming/entry leg still needs full checks.

  Low portals follow the finger length, keeping the final contact segment
  inside its existing center corridor. Heights come from the actual fingers
  and table plane, not an assumption that an overhead pose must be reachable.
  """
  src=xyz(scene,color)
  shift=np.zeros(3) if grasp is None else np.asarray(grasp['position'])-(src+[0,0,.128])
  if not isinstance(offsets,str):
   return [{'position_m':(src+[0,0,d]+shift).tolist(),'offset_m':d+shift[2],'strategy':'overhead' if grasp is None else 'pose_family_overhead'} for d in offsets]
  if offsets not in ('low_overhead','adaptive'):raise ValueError('Unknown source entry family')
  rotated=self.local[OPEN]@Rotation.from_quat(quat(yaw) if grasp is None else grasp['quaternion']).as_matrix().T
  halves=rotated.reshape(2,8,3).mean(axis=1);opening=halves[1]-halves[0]
  along=np.array([-opening[1],opening[0],0.]);norm=np.linalg.norm(along)
  if norm<1e-9:raise NoPlan('Invalid open-finger geometry for low entry')
  along/=norm
  # Keep 2mm inside the original nominal12mm contact center corridor.
  shifts=[np.zeros(3)] if offsets=='low_overhead' else [along*.010,-along*.010]
  shifts.sort(key=lambda d:np.linalg.norm((src+d)[:2]))
  native_table=boxes(scene,(),table_allowance=.032)[0]
  # Match the execution table AABB as well as the local sloped plane. Native
  # world Z is base Z +8mm, and its existing distance margin is3mm.
  native_floor=native_table['center_m'][2]+native_table['size_m'][2]/2-.008-float(rotated[:,2].min())+.003+.00025
  out=[]
  if grasp and grasp.get('source_family')=='outer-inward' and offsets=='low_overhead':
   inward=along.copy();inward*=(-1 if inward[:2]@src[:2]>0 else 1)
   R=Rotation.from_quat(grasp['quaternion'])
   for h in (.030,.029,.0295,.028,.027,.026,.025,.022,.034):
    # Near full reach, 1 mm of portal height trades wrist headroom for table
    # clearance. Also stay inside the existing 15 mm contact-center corridor.
    for lateral in ((.0145,.0149) if h in (.029,.0295) else (.014,.012,.010,.008)):
     point=src+np.r_[lateral*inward[:2],h-.012]-R.apply([0,0,.140])
     if point[2]<native_floor:continue
     out.append(dict(position_m=point.tolist(),offset_m=float(point[2]-src[2]),strategy='inner_pad_entry'))
  for shift in shifts:
   point=src+shift
   if grasp is not None:point[:2]+=np.asarray(grasp['position'])[:2]-src[:2]
   floor=max(table(scene,point+v)-v[2] for v in rotated)
   z=max(float(src[2]+.140),float(floor+MIN_TABLE+.020+.002),native_floor)
   for extra in (0.,.005):
    position=[float(point[0]),float(point[1]),z+extra]
    out.append({'position_m':position,'offset_m':z+extra-float(src[2]),'strategy':'low_inward_entry' if np.linalg.norm(shift)>0 else 'low_overhead_entry'})
  return out
 def pickup_prefix(self,start,scene,color,yaw,hover_offset,support,hover_position=None,grasp=None):
  """Reuse only exact scene/start/yaw/hover checks; no movement or old frames."""
  if not hasattr(self,'prefix_cache'):self.prefix_cache={};self.prefix_failures={}
  key=digest([start,scene,color,yaw,hover_offset,support,hover_position,grasp])
  failure_key=(getattr(self,'query_mode',None) or getattr(self,'ik_mode','fast'),key)
  if failure_key in self.prefix_failures:raise NoPlan(self.prefix_failures[failure_key])
  if key in self.prefix_cache:return copy.deepcopy(self.prefix_cache[key])
  src=xyz(scene,color);q=quat(yaw) if grasp is None else grasp['quaternion'];grip=[src[0],src[1],src[2]+.128] if grasp is None else grasp['position'];a=list(start);a[6]=OPEN;actions=[]
  try:
   hover=[src[0],src[1],src[2]+hover_offset] if hover_position is None else list(hover_position)
   if grasp and grasp.get('source_family')=='outer-inward':
    # Eliminate a low portal before constructing a full tower around it.
    # Only this stationary endpoint is eager; every incoming edge is still certified.
    joint=self.endpoint(hover,q,start);joint[6]=OPEN
    env=boxes(scene,(),table_allowance=.032)
    nk=digest([joint,joint,env,self.robot.get('geometry_identity'),self.robot.get('owner_status',{}).get('generation')])
    if nk not in self.native_cache:self.native_cache[nk]=self.ctx.value('piper_check_path',start14=joint,target14=joint,obstacles=env)
    n=self.native_cache[nk]
    if not n.get('cad_clear') or not n.get('workspace_clear'):raise NoPlan('Outer entry native endpoint refused: '+str(n.get('contacts'))[:160])
   for label,dest,contact in [('hover',hover,False),('entry',grip,True)]:
    self.prefix_phase=label;self.progress(color=color,yaw=yaw,hover_offset_m=hover_offset,phase=label)
    route=self.route(a,dest,q,scene,active=color,contact=contact,support=support)
    for part in route:actions.append(dict(kind='arm',stage=color+'-'+label,color=color,scene=copy.deepcopy(scene),payload=None,contact=contact,support=support,withdraw=False,assembly_neighbor=None,pickup=False,**part))
    a=route[-1]['target14']
   T=np.asarray(self.fk(a)['T_base_flange']);lo,hi=block_bounds(scene,color)
   payload={'corners_flange':((box_corners(lo,hi)-T[:3,3])@T[:3,:3]).tolist(),'source_top':src.tolist(),'grasp_T':T.tolist(),'color':color}
   b=a.copy();b[6]=CLOSED;checked=b.copy();checked[6]=.048
   self.prefix_phase='close';check=self.check(a,checked,scene,active=color,contact=True)
   actions.append(dict(kind='jaw',stage=color+'-close',color=color,width_m=CLOSED,start14=a.copy(),target14=b.copy(),scene=copy.deepcopy(scene),source_top=src.tolist(),check=check));a=b
   self.check(a,a,scene,active=color,payload=payload,contact=True,support=support)
   self.check(a,a,scene,active=color,payload=payload,pickup=True,support=support,reserve_m=.003,geometry_only=True)
   result={'actions':actions,'start14':a,'payload':payload};self.prefix_cache[key]=copy.deepcopy(result);return result
  except NoPlan as exc:self.prefix_failures[failure_key]=str(exc);raise
 def endpoint(self,p,q,seed):
  # Endpoint feasibility is independent of the incoming path seed. Actual
  # connection still solves/checks from the preceding measured/planned joint.
  cache=getattr(self,'endpoint_cache',None)
  if cache is None:self.endpoint_cache={};cache=self.endpoint_cache
  mode=(getattr(self,'query_mode',None) or self.ik_mode) if self.search_active else 'full'
  pose=tuple(float(v) for v in np.r_[p,q]);key=(mode,pose)
  if key not in cache and mode=='full' and cache.get(('fast',pose)) is not None:
   cache[key]=cache[('fast',pose)]
  if key not in cache:
   base=float(math.atan2(p[1],p[0]));cache_value=None
   heads=([0.]*6,[base,1.4,-1.,1.2,0.,0.],[base,1.9,-1.8,1.5,0.,0.])
   if getattr(getattr(self,'ctx',None),'_blocks_batch_ik',False):
    try:cache_value=self._solve_candidates(p,q,heads,seed[6:])[:6]
    except IKUnavailable:pass
    cache[key]=cache_value
    if cache_value is None:raise IKUnavailable('Endpoint unavailable after bounded canonical seeds')
    return list(cache_value)+list(seed[6:])
   for head in heads:
    try:cache_value=self._solve_single(p,q,list(head)+list(seed[6:]))[:6];break
    except IKUnavailable:pass
   cache[key]=cache_value
  if cache[key] is None:raise IKUnavailable('Endpoint unavailable after bounded canonical seeds')
  return list(cache[key])+list(seed[6:])
 def search_step(self,label,fn):
  # Yield keeps this generator's locals and the enclosing candidate cursor.
  # Only scheduler exhaustion is resumable; geometry/service errors propagate.
  while True:
   try:
    self.progress(phase=label)
    value=fn()
   except FastPassTimeout:
    yield {'phase':label,'resume':True,'upgrade':True}
   except GraspBudgetExceeded:
    yield {'phase':label,'resume':True}
   except CandidateBudgetExceeded:
    yield {'phase':label,'resume':True,'layout_yield':True}
   else:
    return value
 def _family_steps(self,start,scene,color,target,candidate):
  src=xyz(scene,color);yaws=YAW_DEGREES if candidate is None else (candidate['yaw'],)
  feasible=[]
  for yaw in yaws:
   q=quat(yaw) if candidate is None else candidate['quaternion']
   grip=(src+[0,0,.128]).tolist() if candidate is None else candidate['position']
   try:yield from self.search_step('source_endpoint',lambda:self.endpoint(grip,q,start))
   except IKUnavailable:continue
   for pyaw in placement_yaws(yaw):
    pq=quat(pyaw) if candidate is None else (Rotation.from_euler('z',pyaw-yaw,degrees=True)*Rotation.from_quat(q)).as_quat().tolist()
    place=np.asarray(target)+[0,0,.128] if candidate is None else np.asarray(target)-Rotation.from_quat(pq).apply(Rotation.from_quat(q).inv().apply(src-np.asarray(grip)))
    try:yield from self.search_step('placement_endpoint',lambda:self.endpoint(place,pq,start));feasible.append(yaw);break
    except IKUnavailable:pass
  if not feasible:raise NoPlan('No source/placement endpoint pair')
  if candidate is not None:
   j=yield from self.search_step('source_endpoint',lambda:self.endpoint(candidate['position'],candidate['quaternion'],start));j[6]=OPEN
   yield from self.search_step('source_geometry',lambda:self.check(j,j,scene,active=color,contact=True))
  return (yield from self._transfer_steps(start,scene,color,target,grasp=candidate,feasible_yaws=feasible))
 def transfer(self,start,scene,color,target):
  conflicts=pickup_source_conflicts(scene,color)
  if conflicts:raise NoPlan('Pickup source blocked before path search: '+str(conflicts))
  if not hasattr(self,'transfer_jobs'):self.transfer_jobs={};self.search_resume_counts={'family':0,'route':0,'finish_cache':0}
  key=digest([start,scene,color,target])
  if key not in self.transfer_jobs:
   variants=list(grasp_variants(scene,color));preferred=[]
   for tilt,dz in ((0.,-.006),(-3.,0.),(-5.,0.),(3.,0.),(5.,0.)):
    preferred.append(next(v for v in variants if v['tilt_degrees']==tilt and v['height_delta_m']==dz and v['yaw']==180.))
   if color in TS.HIGH_COLORS:
    # The fourth pickup starts after a stack retreat. A fixed -90deg priority
    # forces a large wrist turn near the stack even when a continuation of
    # the current heading works. Rank (do not discard) all original poses by
    # orientation continuity, retaining the proven high-placement tilt first.
    current=Rotation.from_quat(self.fk(start)['quaternion_xyzw'])
    def turn_cost(v):return float((current.inv()*Rotation.from_quat(v['quaternion'])).magnitude())
    preferred=sorted([v for v in variants if v.get('tilt_axis')=='jaw_opening'], key=lambda v:(v['height_delta_m']!=.006,v['tilt_degrees']!=25.,turn_cost(v),float(np.linalg.norm(v['position'][:2]))))+preferred
   ordered=[*preferred,None,*[v for v in variants if v not in preferred]] if color in TS.HIGH_COLORS else [None,*preferred,*[v for v in variants if v not in preferred]]
   ordered=[*source_inward(scene,color),*ordered]
   ordered=list({digest(v):v for v in ordered}.values())  # identical families share one cursor; nearby poses remain alternatives
   def task(v):return {'candidate':v,'generator':self._family_steps(start,scene,color,target,v),'resume_count':0,'finish_extensions':0,'query_mode':'fast'}
   self.transfer_jobs[key]={'queue':[task(v) for v in ordered[:3]],'pending':ordered[3:],'turns':0,'errors':[],'failed_fast':[],'args':(start,scene,color,target)}
  job=self.transfer_jobs[key]
  if 'result' in job:return copy.deepcopy(job['result'])
  while job['queue'] or job['pending']:
   if not job['queue'] or (job['turns']>=3 and job['pending']):
    v=job['pending'].pop(0);job['queue'].append({'candidate':v,'generator':self._family_steps(start,scene,color,target,v),'resume_count':0,'finish_extensions':0,'query_mode':'fast'});job['turns']=0
   task=job['queue'].pop(0);previous=getattr(self,'grasp_deadline',None)
   previous_query=getattr(self,'query_mode',None);self.query_mode=task.get('query_mode','fast')
   self.grasp_deadline=time.monotonic()+(1.2 if self.query_mode=='fast' else 2.5)
   try:
    self.progress(color=color,grasp_family=task['candidate'] or 'default',resumed=task['resume_count'])
    event=next(task['generator'])
   except StopIteration as done:
    job['result']=done.value;return copy.deepcopy(done.value)
   except NoPlan as exc:
    job['errors'].append(str(exc))
    if self.query_mode=='fast':
     if self.ik_mode=='fast':job['failed_fast'].append(task)
     else:
      task['query_mode']='full';task['generator']=self._family_steps(*job['args'],task['candidate']);job['queue'].append(task)
      # Only this exhausted route traversal is retried with longer IK. Live
      # fast generators keep their cursor and inexpensive query policy.
      for k,state in list(getattr(self,'route_resume',{}).items()):
       if state.get('exhausted'):del self.route_resume[k]
    continue
   except (CandidateBudgetExceeded,FastPassTimeout):
    # These global/layout boundaries currently unwind generators. Retain
    # completed transfers and exact route stages; recreate only this family.
    if task['generator'].gi_frame is None:task['generator']=self._family_steps(start,scene,color,target,task['candidate'])
    job['queue'].insert(0,task);raise
   finally:self.grasp_deadline=previous;self.query_mode=previous_query
   if event.get('upgrade'):
    job['queue'].insert(0,task)
    raise FastPassTimeout('Fast pass yielded with active candidate preserved')
   if event.get('layout_yield'):
    job['queue'].insert(0,task)
    raise CandidateBudgetExceeded('Layout slice yielded with candidate progress retained')
   task['resume_count']+=1;self.search_resume_counts['family']+=1;job['turns']+=1
   # A finite finish preference; never extend the layout/global deadline.
   near_finish=event['phase'] in ('finish','release','retreat','lower','transfer')
   if near_finish and task['finish_extensions']<2:
    task['finish_extensions']+=1;job['queue'].insert(0,task)
   else:job['queue'].append(task)
   if hasattr(self.ctx,'plan_event'):self.ctx.plan_event(dict(event,color=color,candidate=task['candidate'],resumed=task['resume_count']))
  raise NoPlan(color+' grasp families exhausted: '+str(job['errors'][-4:]))
 def _transfer_once(self,*args,**kw):
  # Compatibility entry for direct offline callers; production uses scheduler.
  gen=self._transfer_steps(*args,**kw)
  while True:
   try:next(gen)
   except StopIteration as done:return done.value
 def finish_steps(self,place_joint,scene,color,target,payload,place_q,support,neighbor):
  if not hasattr(self,'finish_cache'):self.finish_cache={}
  key=digest([place_joint,scene,color,target,payload,place_q,support,neighbor,getattr(self,'defer_native',False)])
  if key in self.finish_cache:
   if hasattr(self,'search_resume_counts'):self.search_resume_counts['finish_cache']+=1
   return copy.deepcopy(self.finish_cache[key])
  yield from self.search_step('finish',lambda:self.check(place_joint,place_joint,scene,active=color,payload=payload,contact=True,support=support,assembly_neighbor=neighbor))
  placed=copy.deepcopy(scene);placed['blocks'][color]['top_center']['surface_position_m']=list(target)
  opened=list(place_joint);opened[6]=OPEN
  jaw=yield from self.search_step('release',lambda:self.check(place_joint,opened,placed,active=color,contact=True))
  release=dict(kind='jaw',stage=color+'-release',color=color,width_m=OPEN,start14=list(place_joint),target14=opened.copy(),scene=copy.deepcopy(placed),target_top=list(target),check=jaw)
  place=self.fk(place_joint)['position'];errors=[]
  z_room=max(0.,float(getattr(self,'robot',{}).get('workspace_max',[0,0,1.])[2])-place[2]-.002)
  rises=list(dict.fromkeys(min(r,z_room) for r in (.075,.065,.055)))
  candidates=[([place[0],place[1],place[2]+rise],False) for rise in rises if rise>=.005]
  lateral_rises=list(dict.fromkeys(min(r,z_room) for r in (.025,.010,0.)))
  candidates += [([place[0]+dx,place[1]+dy,place[2]+rise],True) for distance in (.10,.14,.18) for rise in lateral_rises for dx,dy in ((0,distance),(0,-distance),(-distance,0),(distance,0))]
  # Try direct withdrawals across the endpoint portfolio BEFORE exploring
  # many detours to the first endpoint. A different endpoint is usually a
  # cheaper remedy for limited overhead reach; retain routed alternatives.
  for routed,dest,withdraw in [(routed,dest,withdraw) for routed in (False,True) for dest,withdraw in candidates]:
   try:
    opts=dict(active=color,contact=not withdraw,support=support,withdraw=withdraw)
    if routed:
     route=yield from self.search_step('retreat',lambda:self.route(opened,dest,place_q,placed,**opts))
    else:
     route,_=yield from self.search_step('retreat_direct',lambda:self.connect_edge(opened,dest,place_q,placed,**opts))
    actions=[dict(kind='arm',stage=color+'-retreat',color=color,scene=copy.deepcopy(placed),payload=None,contact=not withdraw,support=support,withdraw=withdraw,assembly_neighbor=None,pickup=False,**part) for part in route]
    for action in actions:
     feedback_checks=[]
     for gap in release_feedback_gaps():
      a=list(action['start14']);b=list(action['target14']);a[6]=gap;b[6]=gap
      feedback_checks.append((yield from self.search_step('retreat',lambda:self.check(a,b,placed,active=color,contact=not withdraw,support=support,withdraw=withdraw))))
     action['release_feedback_preflight']={'gap_range_m':list(release_feedback_gaps()),'checks':feedback_checks,'scope':'Full retreat paths at both feedback bounds; execution rechecks measured opening and placed scene.'}
    if color!=TS.FINAL_COLOR:
     # A nonfinal retreat must leave a valid FREE-flight starting state for
     # the next pickup, not merely clear its own near-contact margin.
     end=actions[-1]['target14']
     actions[-1]['next_pickup_check']=yield from self.search_step('retreat_next_pickup',lambda:self.check(end,end,placed))
    result={'actions':[release]+actions,'end14':actions[-1]['target14'],'scene_after':placed}
    self.finish_cache[key]=copy.deepcopy(result);return result
   except NoPlan as exc:errors.append(str(exc))
  # Only the final yellow release may end at the checked open pose. Exhaust
  # complete direct/routed withdrawals with full IK first; never convert a
  # timeout, transport fault, invalid placement or failed release into success.
  mode=getattr(self,'query_mode',None) or self.ik_mode
  if color==TS.FINAL_COLOR and mode=='full':
   feedback_checks=[]
   for gap in release_feedback_gaps():
    endpoint=list(opened);endpoint[6]=gap
    feedback_checks.append((yield from self.search_step('terminal_release_hold',lambda:self.check(place_joint,endpoint,placed,active=color,contact=True))))
   terminal={'color':color,'mode':'released_in_place','retreat_performed':False,
             'reason':'Finite direct/routed withdrawal candidates exhausted under full IK; final placement and opening remain checked',
             'withdrawal_refusals':len(errors),'last_withdrawal_refusals':errors[-3:],
             'automatic_prepare':False,'automatic_next_task':False}
   release['terminal_release_hold']=terminal
   release['release_feedback_preflight']={'gap_range_m':list(release_feedback_gaps()),'checks':feedback_checks,'scope':'Final release sweep at both feedback bounds; no retreat or next pickup follows.'}
   result={'actions':[release],'end14':opened,'scene_after':placed,'terminal_release_hold':terminal}
   self.finish_cache[key]=copy.deepcopy(result);return result
  raise NoPlan('No checked empty withdrawal after release: '+str(errors))
 def _transfer_steps(self,start,scene,color,target,grasp=None,feasible_yaws=None):

  src=xyz(scene,color);h=height(scene,color);support=SUPPORTS.get(color) if target[2]>table(scene,target)+h+.02 else None
  conflicts=pickup_source_conflicts(scene,color)
  if conflicts:raise NoPlan('Pickup source blocked before path search: '+str(conflicts))
  yield from self.search_step('source_endpoints',lambda:None)
  candidates=[];attempted_transfers=set();yaws=YAW_DEGREES if grasp is None else (grasp['yaw'],)
  if feasible_yaws is not None:yaws=tuple(y for y in yaws if y in feasible_yaws)
  def orientation(yaw):return quat(yaw) if grasp is None else grasp['quaternion']
  families=((.195,.180,.215),(.170,.160,.150),'low_overhead','adaptive') if grasp is None else ('low_overhead','adaptive',(.150,.160,.170),(.180,.195,.215))
  if np.linalg.norm(src[:2])>.42 and grasp is None:families=('low_overhead','adaptive',(.170,.160,.150),(.195,.180,.215))
  for offsets in families:
   hover_options={yaw:[] for yaw in yaws}
   for yaw in yaws:
    for approach in self.source_approaches(scene,color,yaw,offsets,**({} if grasp is None else {"grasp":grasp})):
     hover_options[yaw].append(approach)
   if not any(hover_options.values()):continue
   yaw_order=list(yaws)
   if isinstance(offsets,str) or offsets[0]<.180:
    inward=np.array([.18-src[0],0.]);inward/=max(np.linalg.norm(inward),1e-9)
    halves=self.local[OPEN].reshape(2,8,3).mean(axis=1)
    def alignment(yaw):
     opening=Rotation.from_quat(orientation(yaw)).apply(halves[1]-halves[0])[:2]
     opening/=max(np.linalg.norm(opening),1e-9)
     return abs(float(opening@inward))
    yaw_order.sort(key=alignment)
   for yaw in yaw_order:
    q=orientation(yaw)
    for approach in hover_options[yaw]:
     hover_offset=approach['offset_m'];hover=approach['position_m']
     try:
      hover_joint=yield from self.search_step('hover_endpoint',lambda:self.endpoint(hover,q,start))
      check_command_reserve(start,hover_joint)
      grip=[src[0],src[1],src[2]+.128] if grasp is None else grasp['position']
      gj=yield from self.search_step('source_endpoint',lambda:self.endpoint(grip,q,start));gj[6]=CLOSED
      gT=np.asarray(self.fk(gj)['T_base_flange']);lo,hi=block_bounds(scene,color)
      provisional={'corners_flange':((box_corners(lo,hi)-gT[:3,3])@gT[:3,:3]).tolist(),'source_top':src.tolist(),'grasp_T':gT.tolist(),'color':color}
      finish_yaws=[]
      for pyaw in placement_yaws(yaw):
       pq=quat(pyaw) if grasp is None else (Rotation.from_euler('z',pyaw-yaw,degrees=True)*Rotation.from_quat(q)).as_quat().tolist()
       pp=np.asarray(target)+[0,0,.128] if grasp is None else np.asarray(target)-Rotation.from_quat(pq).apply(gT[:3,:3].T@(src-gT[:3,3]))
       neighbor=placed_neighbors(scene,color,target)
       try:
        pj=yield from self.search_step('placement_endpoint',lambda:self.endpoint(pp,pq,gj))
        yield from self.finish_steps(pj,scene,color,target,provisional,pq,support,neighbor)
        finish_yaws.append(pyaw)
       except NoPlan:pass
      if not finish_yaws:raise NoPlan('Placement/opening/withdrawal screen failed before pickup path')
      if approach['strategy']=='overhead':prefix=yield from self.search_step('pickup',lambda:self.pickup_prefix(start,scene,color,yaw,hover_offset,support))
      else:prefix=yield from self.search_step('pickup',lambda:self.pickup_prefix(start,scene,color,yaw,hover_offset,support,hover_position=hover,**({} if grasp is None else {"grasp":grasp})))
     except NoPlan as exc:
      event={'color':color,'yaw':yaw,'hover':hover_offset,'phase':getattr(self,'prefix_phase','finish_screen'),'why':str(exc),'prefix_rejected_once':True}
      candidates.append(event)
      if hasattr(self.ctx,'plan_event'):self.ctx.plan_event(event)
      continue
     # A square upright block can keep its target footprint after a quarter-turn.
     # The measured rigid payload relation and every loaded path remain checked.
     for placement_yaw in finish_yaws:
      place_q=quat(placement_yaw) if grasp is None else (Rotation.from_euler('z',placement_yaw-yaw,degrees=True)*Rotation.from_quat(q)).as_quat().tolist()
      place=[target[0],target[1],target[2]+.128]
      if grasp is not None:
       gT=np.asarray(prefix['payload']['grasp_T'])
       offset=Rotation.from_quat(place_q).apply(gT[:3,:3].T@(src-gT[:3,3]))
       place=(np.asarray(target)-offset).tolist()
      try:
       # A final placement or jaw-opening conflict cannot be cured by changing
       # cruise height or detours. Check it once before constructing those routes.
       place_joint=yield from self.search_step('placement_endpoint',lambda:self.solve(place,place_q,prefix['start14']))
       neighbor=placed_neighbors(scene,color,target)
       finish=yield from self.finish_steps(place_joint,scene,color,target,prefix['payload'],place_q,support,neighbor)
      except NoPlan as exc:
       event={'color':color,'yaw':yaw,'placement_yaw':placement_yaw,'hover':hover_offset,'phase':'placement_endpoint','why':str(exc)}
       candidates.append(event)
       if hasattr(self.ctx,'plan_event'):self.ctx.plan_event(event)
       continue
      for actual_above in destination_hovers(scene,color,target,place):
       travel_z=actual_above[2]
       actions=[];a=list(start);a[6]=OPEN;payload=None;scene_before=scene;phase='endpoint_ik'
       try:
        grip=[src[0],src[1],src[2]+.128] if grasp is None else grasp['position']
        # Reject impossible destination poses before expensive incoming-path work.
        yield from self.search_step('endpoint_ik',lambda:self.endpoint(place,place_q,start))
        yield from self.search_step('endpoint_ik',lambda:self.endpoint(actual_above,place_q,start))
        yield from self.search_step('endpoint_ik',lambda:self.endpoint(grip,q,start));yield from self.search_step('endpoint_ik',lambda:self.endpoint(hover,q,start))
        # Filter reachable local lift endpoints BEFORE constructing incoming and
        # placement paths. The far-side source can require a lower empty hover
        # and a lower local lift before an inward, rising transport corridor.
        lift_z=None
        # Keep the normal lift preferences. When the source column is short,
        # a checked smaller departure can clear the table before inward travel.
        # The existing pickup rule still forbids lateral motion below10mm.
        T=np.asarray(self.fk(prefix['start14'])['T_base_flange'])
        corners=np.asarray(prefix['payload']['corners_flange'])@T[:3,:3].T+T[:3,3]
        bottom=min(float(v[2]-table(scene,v)) for v in corners)
        departure_rise=max(.010,.010-bottom+.00001)
        for offset in (.180,.170,.160,.150,*[grip[2]-src[2]+departure_rise+extra for extra in (.002,.001,0.)]):
         z=min(src[2]+offset,travel_z)
         if z<grip[2]+departure_rise-1e-8:continue
         seed=prefix['start14'] if offset<.150 else [0.]*6+start[6:]
         try:
          lift_joint=yield from self.search_step('lift_endpoint',lambda:self.solve([grip[0],grip[1],z],q,seed))
          check_command_reserve(prefix['start14'],lift_joint)
          lift_z=z;break
         except NoPlan:pass
        if lift_z is None:raise NoPlan('Source lift endpoint IK unavailable')
        transfer_key=digest([prefix['start14'],prefix['payload'],lift_z,actual_above,place_q])
        if transfer_key in attempted_transfers:
         self.reuse_counts['duplicate_transfer']+=1;continue
        attempted_transfers.add(transfer_key)
        def arm(label,dest,*,contact=False,loaded=False,withdraw=False,assembly_neighbor=None,pickup=False,orientation=None):
         nonlocal a,phase
         phase=label
         route=yield from self.search_step(label,lambda:self.route(a,dest,q if orientation is None else orientation,scene,active=color,payload=payload if loaded else None,contact=contact,support=support,withdraw=withdraw,assembly_neighbor=assembly_neighbor,pickup=pickup))
         for part in route:
          actions.append(dict(kind='arm',stage=color+'-'+label,color=color,scene=copy.deepcopy(scene),payload=payload if loaded else None,contact=contact,support=support,withdraw=withdraw,assembly_neighbor=assembly_neighbor,pickup=pickup,**part))
         a=route[-1]['target14']
        actions=copy.deepcopy(prefix['actions']);a=list(prefix['start14']);payload=copy.deepcopy(prefix['payload'])
        # Choose a reachable local lift before a routed transit. Far positions
        # need an inward corridor before reaching a higher transport altitude.
        yield from arm('raise',[grip[0],grip[1],lift_z],loaded=True,pickup=True)
        yield from arm('transfer',actual_above,loaded=True,orientation=place_q)
        # Close placement is intentional only next to goal neighbours already in place.
        neighbor=placed_neighbors(scene,color,target)
        yield from arm('lower',place,contact=True,loaded=True,assembly_neighbor=neighbor,orientation=place_q)
        # The executed lower route may pick another IK branch. Bind finish to
        # its exact joint endpoint; only identical finish contracts reuse checks.
        finish=yield from self.finish_steps(a,scene,color,target,payload,place_q,support,neighbor)
        actions.extend(finish['actions']);a=finish['end14'];placed=finish['scene_after']
        return {'actions':actions,'end14':a,'scene_after':placed,'yaw_degrees':yaw,'placement_yaw_degrees':placement_yaw,'color':color,'source_top':src.tolist(),'target_top':list(target),'payload':payload,'source_approach':approach,'departure_rise_m':lift_z-grip[2],'grasp_candidate':grasp,'placement_flange_m':place}
       except NoPlan as exc:
        self.last_candidate={'color':color,'yaw':yaw,'hover':hover_offset,'phase':phase,'why':str(exc)}
        if hasattr(self.ctx,'plan_event'):self.ctx.plan_event(self.last_candidate)
        if 'scene_before' in locals():scene=scene_before
        candidates.append({'yaw':yaw,'hover':hover_offset,'transport_z':travel_z,'why':str(exc)})
  raise NoPlan(color+' pick/place unreachable: '+str(candidates[-4:]))
 def plan(self,start):
  self.search_active=True;self.started=time.monotonic();attempts=[];self.ik_query_counts={'fast':0,'full':0}
  try:
   for mode in ('fast','full'):
    self.ik_mode=mode;begin=time.monotonic()
    reused={}
    if mode=='full':
     reused=self.prepare_full_recheck()
     self.searching={};self.progress_at=0.
     self.progress(phase='long_budget_recheck')
    try:
     result=self._plan(start)
     attempts.append({'pass':mode,'outcome':'complete','elapsed_s':time.monotonic()-begin,'ik_queries':self.ik_query_counts[mode],**reused})
     result['ik_search']={'attempts':attempts,'fast_budget_ms':5,'full_budget_ms':100,'total_budget_s':PREFLIGHT_BUDGET_S,'fallback_used':mode=='full','policy':'reuse exact validated results; full KDL only for unresolved/new queries'}
     result['reuse_counts']=dict(getattr(self,'reuse_counts',{}))
     result['elapsed_s']=time.monotonic()-self.started
     return result
    except (NoPlan,FastPassTimeout) as exc:
     attempts.append({'pass':mode,'outcome':'no_complete_plan','elapsed_s':time.monotonic()-begin,'ik_queries':self.ik_query_counts[mode],'why':str(exc),**reused})
     if mode=='full' or not (getattr(self,'candidate_yielded',False) or any(k[0]=='fast' and v is None for k,v in self.ik_cache.items())):raise
    except Exception as exc:
     attempts.append({'pass':mode,'outcome':'aborted','elapsed_s':time.monotonic()-begin,'ik_queries':self.ik_query_counts[mode],'why':str(exc),**reused})
     raise
   raise AssertionError('Unreachable search state')
  finally:
   self.search_active=False;self.defer_native=False;self.ik_mode='full';self.ik_attempts=attempts;self.candidate_deadline=None
 def note_screen(self,color,stage,exc):
  msg=str(exc)
  label=('指尖离桌不足' if 'Finger/table' in msg else '带载路径距邻块不足：'+msg.split(' path conflict')[0] if msg.startswith('Payload/') else '姿态无IK解' if isinstance(exc,IKUnavailable) else '位姿超工作区' if 'outside workspace' in msg else '退开候选失败' if 'withdrawal' in msg else msg.split(':')[0][:75])
  d=self.__dict__.setdefault('screen_counts',{});k=color+' '+stage+' '+label;d[k]=d.get(k,0)+1
 def _layout_steps(self,start,anchor,side):
  target=goal_scene(self.scene,anchor,side);assert_goal(target)
  if any(np.any(xyz(target,c)[:2]-[.030,.030]<LAYOUT_MIN-LAYOUT_ERROR_M) or np.any(xyz(target,c)[:2]+[.030,.030]>LAYOUT_MAX+LAYOUT_ERROR_M) for c in target['blocks']):raise NoPlan('Layout outside task bounds')
  # Screen each stacked placement AND its next-pickup clearance before
  # spending budget on earlier transfers. Unmoved objects keep source poses.
  for stacked in SUPPORTS:
   top_scene=copy.deepcopy(self.scene)
   for predecessor in TS.PLACE_ORDER:
    if predecessor==stacked:break
    top_scene['blocks'][predecessor]=copy.deepcopy(target['blocks'][predecessor])
   top_target=xyz(target,stacked).tolist()
   top_candidates=[{'position':(xyz(target,stacked)+[0,0,.128]).tolist(),'quaternion':quat(yaw)} for yaw in (180.,0.,-90.,90.)]
   top_candidates.extend(grasp_variants(target,stacked))
   while True:
    top_ok=False
    for candidate in top_candidates:
     try:
      pj=yield from self.search_step(stacked+'_finish_screen',lambda:self.solve(candidate['position'],candidate['quaternion'],start));pj[6]=CLOSED
      T=np.asarray(self.fk(pj)['T_base_flange']);lo,hi=block_bounds(target,stacked)
      load={'corners_flange':((box_corners(lo,hi)-T[:3,3])@T[:3,:3]).tolist(),'color':stacked}
      above=list(candidate['position']);above[2]+=.020
      aj=yield from self.search_step(stacked+'_approach_screen',lambda:self.solve(above,candidate['quaternion'],pj));aj[6]=CLOSED
      beside=placed_neighbors(top_scene,stacked,top_target)
      yield from self.search_step(stacked+'_approach_screen',lambda:self.check(aj,pj,top_scene,active=stacked,payload=load,contact=True,support=SUPPORTS[stacked],assembly_neighbor=beside))
      yield from self.finish_steps(pj,top_scene,stacked,top_target,load,candidate['quaternion'],SUPPORTS[stacked],beside)
      top_ok=True;break
     except NoPlan as why:
      self.note_screen(stacked,'放置/退开',why)
    if top_ok:break
    if self.ik_mode!='fast':raise NoPlan(stacked+' placement/release/retreat unavailable for layout')
    yield {'needs_full':True,'phase':stacked+'_finish_screen'}
  pending=copy.deepcopy(self.scene)
  for base in BASE_COLORS:pending['blocks'][base]=copy.deepcopy(target['blocks'][base])
  for color in SUPPORTS:
   lo,hi=block_bounds(pending,color)
   if pickup_source_conflicts(pending,color):raise NoPlan('Earlier layout blocks '+color+' departure')
   candidates=[{'position':(xyz(pending,color)+[0,0,.128]).tolist(),'quaternion':quat(yaw)} for yaw in YAW_DEGREES]
   candidates=[*source_inward(pending,color),*candidates,*grasp_variants(pending,color)]
   while True:
    can_enter=False
    for candidate in candidates:
     try:
      joint=yield from self.search_step(color+'_entry_screen',lambda:self.solve(candidate['position'],candidate['quaternion'],start));joint[6]=OPEN
      yield from self.search_step(color+'_entry_screen',lambda:self.check(joint,joint,pending,active=color,contact=True));can_enter=True;break
     except NoPlan as why:
      self.note_screen(color,'抓取入口',why)
    if can_enter:break
    if self.ik_mode!='fast':raise NoPlan('Earlier layout blocks all '+color+' grasp candidates')
    yield {'needs_full':True,'phase':color+'_entry_screen'}
   pending['blocks'][color]=copy.deepcopy(target['blocks'][color])
  seen_orders=set()
  for ranking in ranked_orders(self.scene,target,self.fk(start)['position'][:2],TS.task_orders(side)):
   order=ranking['order']
   if ranking['blocked_pickups']:
    self.diagnostics.append({'anchor':anchor,'orange_side':side,'order':order,'why':'Earlier placements obstruct a later pickup'});continue
   effective=tuple(c for c in order if np.linalg.norm(xyz(self.scene,c)-xyz(target,c))>=.003)
   if effective in seen_orders:continue
   seen_orders.add(effective)
   while True:
    scene=copy.deepcopy(self.scene);a=list(start);a[6]=OPEN;moves=[]
    try:
     self.lazy_stats['geometric_candidates']+=1
     for color in order:
      if np.linalg.norm(xyz(scene,color)-xyz(target,color))<.003:continue
      while True:
       try:
        self.progress(order=list(order),order_score=ranking,completed_colors=[m['color'] for m in moves])
        move=self.transfer(a,scene,color,xyz(target,color).tolist());break
       except FastPassTimeout:yield {'upgrade':True,'phase':self.searching.get('phase'),'completed':len(moves)}
       except CandidateBudgetExceeded:yield {'resume':True,'phase':self.searching.get('phase'),'completed':len(moves)}
       except NoPlan:
        if self.ik_mode!='fast':raise
        yield {'needs_full':True,'phase':self.searching.get('phase'),'completed':len(moves)}
      moves.append(move);a=move['end14'];scene=move['scene_after']
     assert_goal(scene)
     try:self.certify_complete_plan(moves)
     except NativePathRefusal as exc:
      self.diagnostics.append({'anchor':anchor,'orange_side':side,'order':order,'why':str(exc),'lazy_edge_rejected':True})
      continue
     return {'ok':True,'version':VERSION,'source_scene':self.scene,'goal_scene':target,'moves':moves,'goal_anchor':anchor,'orange_side':side,'order':[m['color'] for m in moves],'requested_order':list(order),'order_score':ranking,'layout_choice':{'rank':self.layout_candidates.index((anchor,side)),'policy':'ranked resumable layout/order queue; first fully certified plan','orange_transfer_xy_m':float(np.linalg.norm(xyz(self.scene,'orange')[:2]-xyz(target,'orange')[:2]))},'local_fk':getattr(self.ctx,'_blocks_fk_identity',{'mode':'service'}),'layout_outer_bounds_m':{'min':LAYOUT_MIN.tolist(),'max':LAYOUT_MAX.tolist()},'goal':{'digest':TS.GOAL_DIGEST,'source':TS.GOAL_SOURCE,'spec':TS.GOAL,'side':side,'final_color':TS.FINAL_COLOR},'diagnostics':self.diagnostics,'elapsed_s':time.monotonic()-self.started,'fully_preflighted':True,'executed':False,'source_digest':digest(self.scene),'start14':list(start),'opening_check':self.opening_check,'lazy_validation':dict(self.lazy_stats),'reuse_counts':dict(getattr(self,'reuse_counts',{})),'search_resume_counts':dict(getattr(self,'search_resume_counts',{})),'layout_resume_count':self.layout_resume_count,'pose_solution_reuses':getattr(self,'pose_solution_reuses',0),'pose_solution_retries':getattr(self,'pose_solution_retries',0)}
    except NoPlan as exc:
     self.diagnostics.append({'anchor':anchor,'orange_side':side,'order':order,'why':str(exc)})
     break
  raise NoPlan('All task orders exhausted for layout')
 def _plan(self,start):
  if not hasattr(self,'layout_queue'):
   opened=list(start);opened[6]=OPEN;self.defer_native=False
   self.opening_check=self.check(start,opened,self.scene)
   self.layout_candidates=layout_candidates(self.scene,self.robot)
   partition=getattr(self.ctx,'_blocks_layout_partition',None)
   if partition is not None:
    from assembly_parallel import partition_layouts
    self.layout_candidates,self.assigned_layout_ids=partition_layouts(self.layout_candidates,*partition)
    # Try the no-orange-transfer option early only in its owning lane. Never drop its alternatives.
    pairs=list(zip(self.layout_candidates,self.assigned_layout_ids)); orange=xyz(self.scene,BASE_COLORS[1])[:2]
    pairs.sort(key=lambda p:np.linalg.norm(np.asarray(p[0][0])+TS.offset_xy(BASE_COLORS[1],p[0][1])-orange)>=.003)
    self.layout_candidates=[p[0] for p in pairs];self.assigned_layout_ids=[p[1] for p in pairs]
    callback=getattr(self.ctx,'_blocks_partition_event',None)
    if callback:callback(self.assigned_layout_ids)
   self.layout_queue=[];self.layout_pending=[];self.layout_cursor=0;self.layout_resume_count=0
  self.defer_native=True
  if self.ik_mode=='full' and self.layout_pending:
   self.layout_queue+=self.layout_pending;self.layout_pending=[]
  if self.ik_mode=='full':self.layout_queue.sort(key=lambda task:-task.get('completed',0))
  # Two active layouts ensure that a yielded promising plan gets another turn
  # before hundreds of new layouts consume the budget. Admit a new alternative
  # after two turns, while retaining all older generators in this same queue.
  turns=0;finish_turns=0
  while self.layout_queue or self.layout_cursor<len(self.layout_candidates):
   self.candidate_deadline=None;self.progress()
   # Up to three extra slices for a near-complete layout, then a normal FIFO
   # portfolio turn. Preserve every alternative and its generator cursor.
   if self.layout_queue and finish_turns<3:
    best=max(range(len(self.layout_queue)),key=lambda i:self.layout_queue[i].get('completed',0))
    if self.layout_queue[best].get('completed',0)>=len(COLORS)-1:
     self.layout_queue.insert(0,self.layout_queue.pop(best));finish_turns+=1
   else:finish_turns=0
   if len(self.layout_queue)<2 or (turns>=2 and self.layout_cursor<len(self.layout_candidates)):
    if self.layout_cursor<len(self.layout_candidates):
     anchor,side=self.layout_candidates[self.layout_cursor];self.layout_cursor+=1
     self.layout_queue.append({'anchor':anchor,'side':side,'generator':self._layout_steps(start,anchor,side),'resumes':0,'completed':0})
    turns=0
   if not self.layout_queue:continue
   task=self.layout_queue.pop(0)
   self.candidate_deadline=time.monotonic()+(LAYOUT_SLICE_FAST_S if self.ik_mode=='fast' else LAYOUT_SLICE_FULL_S)
   try:
    self.progress(anchor=task['anchor'],orange_side=task['side'],layout_resume=task['resumes'])
    event=next(task['generator'])
   except StopIteration as done:return done.value
   except NoPlan as exc:
    self.diagnostics.append({'anchor':task['anchor'],'orange_side':task['side'],'why':str(exc)});continue
   except FastPassTimeout:
    self.layout_queue.insert(0,task);raise
   finally:self.candidate_deadline=None
   task['resumes']+=1;self.layout_resume_count+=1;advanced=event.get('completed',task['completed'])>task['completed'];task['completed']=event.get('completed',task['completed'])
   self.diagnostics.append({'anchor':task['anchor'],'orange_side':task['side'],'resume':task['resumes'],**event})
   if event.get('upgrade'):
    self.candidate_yielded=True;self.layout_queue.insert(0,task);raise FastPassTimeout('Upgrade unresolved IK without restarting layouts/orders')
   if event.get('needs_full') and self.ik_mode=='fast':self.layout_pending.append(task)
   elif advanced:self.layout_queue.insert(0,task)
   else:self.layout_queue.append(task)
   self.candidate_yielded=True;turns+=1
  if self.layout_pending and self.ik_mode=='fast':raise FastPassTimeout('Only unresolved fast candidates remain')
  raise NoPlan('预检候选拒绝记录 '+str(getattr(self,'screen_counts',{}))+'; 最后布局 '+str(self.diagnostics[-3:]))
