"""R6 bounded loaded-motion checks. No hidden motion or approval state.

Conditional three-block scene, same downward orientation, retained wrist pose.
The head transform remains provisional. Payload is a conservative 70mm cube
around the same-run grasp; supplied original scene is provenance, not new depth.
"""
import numpy as np
from qualitative import retention_evidence, retention_at_endpoint
from scene_checks import (PerceptionUnknown,current_image,read_report,completed_action_state,
 scene_control,scene_geometry,wrist_between_pads)

MAX_LOADED_LATERAL_M = .400  # native single-motion limit; geometry still checks the entire new path
MAX_PATH_SAMPLE_SPACING_M = .003  # preserve 120mm/40 sampling density on longer paths

class LoadedChecks:
 def __init__(self,ctx,previous_report,color='orange',support_report=None):
  self.ctx=ctx;self.previous_report=previous_report;self.color=color;self.support_report=support_report
 def __call__(self,*,phase,before,after,inputs):
  try:
   if phase=='retained_and_loaded_path':return self.path(before,inputs)
   if phase=='retained_at_destination':
    match=retention_evidence(self.ctx,before,after,self.color);wrist=match
    if abs(after['measured14'][6]-before['measured14'][6])>.003:raise PerceptionUnknown('Jaw changed during loaded transport')
    return {'ok':True,'frame_id':after['frame_id'],'why':'Block appearance retained relative to wrist after checked loaded segment','wrist':wrist,'wrist_match':match}
   raise PerceptionUnknown('Unsupported loaded-check phase')
  except (PerceptionUnknown,KeyError,ValueError,OSError) as e:
   return {'ok':None,'frame_id':after.get('frame_id'),'why':str(e)}
 def path(self,obs,inputs):
  ctx=self.ctx;robot=ctx.value('piper_status')
  record=read_report(self.previous_report);prev=completed_action_state(record,obs,robot)
  recovering=prev['stage']=='lower_to_support' and prev.get('support_required') is False
  if not prev.get('ok') or (prev['stage'] not in ('trial_lift','transport') and not recovering):
   raise PerceptionUnknown('Requires successful trial or transport endpoint')
  scene=prev['facts'][0]['scene'];scene_control(scene,robot);scene_geometry(scene,obs)
  if self.support_report is not None:
   if self.color!='green':raise PerceptionUnknown('Fresh stack support is for green transport only')
   from stack_checks import support_scene
   scene=support_scene(ctx,self.previous_report,obs,robot,self.support_report,scene)
  match=retention_at_endpoint(ctx,prev,obs,self.color);wrist=match
  points=inputs['waypoints'];q=inputs['quaternion']
  if len(points)!=1:raise PerceptionUnknown('First loaded rehearsals require one checked segment')
  start=obs['measured14'];target=np.asarray(points[0]);f=ctx.value('piper_fk',joints=start[:6]);p=np.asarray(f['position'])
  if recovering:
   effective=prev['facts'][0].get('retreat_position')
   planned=[effective] if effective is not None else record['inputs'].get('retreat',[])
   if np.linalg.norm(target[:2]-p[:2])>=.002 or len(planned)!=1 or np.linalg.norm(target-np.array(planned[0]))>.002:
    raise PerceptionUnknown('Placement recovery only rises to its prechecked retreat target')
  origin=payload_origin(ctx,self.previous_report,prev,scene,self.color)
  numeric=loaded_segment(ctx,start,target,q,scene,origin,finger_locals(ctx,obs,f),recovering=recovering,start_fk=f)
  return dict(numeric,frame_id=obs['frame_id'],why='Successful retained predecessor, fresh wrist agreement, nativeCAD/workspace and sampled payload/finger clearance passed',
   scene=scene,payload_origin=origin,wrist=wrist,wrist_match=match,prechecked_placement_retreat=recovering,
   retained_neighbor_scope='Other blocks were remote from grasp; any new contact/displacement invalidates positions')


def finger_locals(ctx,obs,fk=None):
 """Installed corners relative to actual flange; usable for hypothetical FK only."""
 T=np.asarray((fk or ctx.value('piper_fk',joints=obs['measured14'][:6]))['T_base_flange'])
 fingers=obs.get('finger_geometry',{}).get('fingers')
 if not fingers:raise PerceptionUnknown('Current installed finger geometry missing')
 return [(np.asarray(f['cad_corners_base_m'])-T[:3,3])@T[:3,:3] for f in fingers]

def payload_origin(ctx,previous_report,prev,scene,color):
 origin=prev['facts'][0].get('payload_origin')
 if origin is None:
  if prev['stage']!='trial_lift':raise PerceptionUnknown('Missing verified payload origin')
  close=read_report(read_report(previous_report)['inputs']['previous_report'])['result']
  if close.get('ok') is not True or close.get('stage')!='close_grasp':raise PerceptionUnknown('Missing successful grasp predecessor')
  cp=np.asarray(ctx.value('piper_fk',joints=close['measured']['measured14'][:6])['position'])
  plane=np.asarray(scene['table']['z_ax_by_c']);ztable=float(cp[:2]@plane[:2]+plane[2])
  origin={'grasp_flange':cp.tolist(),'flange_to_bottom_m':float(cp[2]-ztable),'color':color,'half_xy_m':.035,
          'scope':'Approximate70mm envelope from same-scene table and grip pose; no independent payload metrology'}
 if origin.get('color')!=color:raise PerceptionUnknown('Payload color differs from verified grasp origin')
 return origin

def loaded_segment(ctx,start,target,q,scene,origin,local_fingers,*,recovering=False,start_fk=None):
 """Read-only numerical path check from explicit planned joints, NOT an observation.

 No receipt/retention/owner claim is created. Actual execution validates those
 separately and reruns this identical native/path/envelope check.
 """
 payload_color=origin['color'];start=list(start);target=np.asarray(target,float)
 fk=start_fk or ctx.value('piper_fk',joints=start[:6]);T=np.asarray(fk['T_base_flange']);p=np.asarray(fk['position'])
 local=np.concatenate(local_fingers)
 if T[2,2]>-.995 or abs(np.dot(q,fk['quaternion_xyzw']))<.99999:
  raise PerceptionUnknown('Loaded segment must preserve current near-down orientation')
 if not .048<=start[6]<=.054:raise PerceptionUnknown('Outside conditional loaded jaw feedback range')
 vertical=np.linalg.norm(target[:2]-p[:2])<.002
 if vertical:
  if not .005<=target[2]-p[2]<=.100:raise PerceptionUnknown('Loaded vertical rehearsal must rise5..100mm')
 elif abs(target[2]-p[2])>.002 or np.linalg.norm(target[:2]-p[:2])>MAX_LOADED_LATERAL_M:
  raise PerceptionUnknown('Loaded lateral motion requires level path <=400mm')
 solution=ctx.value('piper_solve_ik',position=target.tolist(),quaternion_xyzw=q,seed=start[:6])
 end=solution['q'];check=ctx.value('piper_check_path',start14=start,target14=end+start[6:])
 if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:raise PerceptionUnknown('Native loaded path not clear')
 plane=np.array(scene['table']['z_ax_by_c']);bottom_offset=origin['flange_to_bottom_m'];half=origin['half_xy_m']
 rows=[];min_table=1.;min_neighbor=1.;last_recovery_bottom=None;initial_recovery_xy_gap=None
 sample_count=max(41,int(np.ceil(np.linalg.norm(target-p)/MAX_PATH_SAMPLE_SPACING_M))+1)
 from fk_samples import fk_line
 for t,f in fk_line(ctx,start,end,np.linspace(0,1,sample_count)):
  F=np.array(f['T_base_flange']);fp=np.array(f['position']);corn=local@F[:3,:3].T+F[:3,3]
  bottom=fp[2]-bottom_offset;table=float(fp[:2]@plane[:2]+plane[2]);min_table=min(min_table,bottom-table)
  if bottom-table<.004:raise PerceptionUnknown('Loaded payload may reach table')
  if corn[:,2].min()-max(table,0)<.012:raise PerceptionUnknown('Installed fingers too low for loaded path')
  for color,b in scene['blocks'].items():
   if color==payload_color:continue
   bp=np.array(b['top_center']['surface_position_m'])
   xy_gap=float(np.max(np.abs(fp[:2]-bp[:2]))-2*half)
   zgap=float(bottom-bp[2]);min_neighbor=min(min_neighbor,max(xy_gap,zgap))
   if xy_gap<.020 and zgap<.020:
    if not (recovering and color=='red'):
     raise PerceptionUnknown('Payload envelope lacks20mm separation from neighbor')
    # Reverse the already checked partial descent: preserve XY, rise only,
    # never add overlap to the intended placement-side proximity.
    if initial_recovery_xy_gap is None:initial_recovery_xy_gap=xy_gap
    # Joint interpolation of the verified descent bowed1.12mm inXY.
    # Bound that bow at2mm and separately guard the neighbor-facing axis.
    if np.linalg.norm(fp[:2]-p[:2])>.002 or xy_gap<initial_recovery_xy_gap-.0005 or (last_recovery_bottom is not None and bottom<last_recovery_bottom-.0005):
     raise PerceptionUnknown('Placement retreat adds lateral/downward movement')
    rlo=bp-np.array([.030,.030,.060]);rhi=bp+np.array([.030,.030,0.])
    for flocal in local_fingers:
     fc=flocal@F[:3,:3].T+F[:3,3]
     if np.max(np.maximum(fc.min(axis=0)-rhi,rlo-fc.max(axis=0)))<0:
      raise PerceptionUnknown('Placement retreat finger envelope intersects red')
    last_recovery_bottom=bottom
  rows.append({'fraction':float(t),'flange':fp.tolist(),'payload_bottom_m':float(bottom)})
 return {'ok':True,'end_joints':end,'preflight':check,'samples':rows,
         'min_payload_table_m':min_table,'min_neighbor_separation_m':min_neighbor}


class LowerChecks:
 """One vertical placement segment with a preplanned upward retreat."""
 def __init__(self,ctx,previous_report,support_required=True):
  self.ctx=ctx;self.previous_report=previous_report;self.support_required=support_required
 def __call__(self,*,phase,before,after,inputs):
  try:
   if phase=='placement_and_retreat_path':
    self.placement_context=self.path(before,inputs);return self.placement_context
   if phase=='supported_before_release':
    if self.support_required:
     # At support, the stationary neighbor enters the old wrist template.
     # Judge placement/support here, not retention of that fixed image patch.
     from qualitative import assess_images
     from ordinary_checks import placement
     context=getattr(self,'placement_context',None)
     if context:
      ordinary=placement(self.ctx,phase,after,context['scene'],context['payload_origin'])
      if ordinary.get('ok') is not None:return ordinary
     fact=assess_images(self.ctx,phase,before,after,
       'Orange has just been lowered to the table-height target checked by geometry. Judge the broad visible outcome: orange upright beside red, its lower part at table level, no obvious hovering, tipping, trapped finger or displaced neighbor. Does the visible scene agree with a supported side-by-side placement suitable for opening in place? Judge ordinary visible support; do not require a visible hidden underside or proof of zero force.')
     return fact|{'supported':fact.get('ok') is True}
    match=retention_evidence(self.ctx,before,after,'orange');wrist=match
    return {'ok':True,'frame_id':after['frame_id'],'why':'Intermediate lowering checkpoint; block retained, support/release NOT established','wrist':wrist,'wrist_match':match,'supported':False}
   raise PerceptionUnknown('Unsupported placement phase')
  except (PerceptionUnknown,KeyError,ValueError,OSError) as e:return {'ok':None,'frame_id':after.get('frame_id'),'why':str(e)}
 def path(self,obs,inputs):
  ctx=self.ctx;robot=ctx.value('piper_status');prev=completed_action_state(read_report(self.previous_report),obs,robot)
  if not prev.get('ok') or prev['stage'] not in ('transport','lower_to_support'):raise PerceptionUnknown('Requires verified loaded predecessor')
  scene=prev['facts'][0]['scene'];origin=prev['facts'][0]['payload_origin'];scene_control(scene,robot);scene_geometry(scene,obs)
  if origin.get('color')!='orange':raise PerceptionUnknown('Table placement supports orange only; green stack is unimplemented')
  match=retention_at_endpoint(ctx,prev,obs,'orange');wrist=match
  if len(inputs['descent'])!=1 or len(inputs['retreat'])!=1:raise PerceptionUnknown('One placement segment and one retreat target required')
  start=obs['measured14'];f=ctx.value('piper_fk',joints=start[:6]);T=np.array(f['T_base_flange']);p=np.array(f['position']);target=np.array(inputs['descent'][0]);q=inputs['quaternion']
  if T[2,2]>-.995 or abs(T[0,0])<.98 or abs(np.dot(q,f['quaternion_xyzw']))<.99999:raise PerceptionUnknown('Expected preserved downward front/back grip')
  if np.linalg.norm(target[:2]-p[:2])>.002 or not .005<=p[2]-target[2]<=.070:raise PerceptionUnknown('Placement must be vertical descent5..70mm')
  if not .048<=start[6]<=.054:raise PerceptionUnknown('Unexpected loaded jaw opening')
  red=np.array(scene['blocks']['red']['top_center']['surface_position_m']);plane=np.array(scene['table']['z_ax_by_c'])
  if abs(target[0]-red[0])>.015 or not .055<=red[1]-target[1]<=.075:raise PerceptionUnknown('Outside conditional red/orange side-by-side corridor')
  bottom_target=target[2]-origin['flange_to_bottom_m'];table_target=float(target[:2]@plane[:2]+plane[2])
  if bottom_target-table_target<-.002:raise PerceptionUnknown('Target would push payload below nominal table')
  if self.support_required and abs(bottom_target-table_target)>.004:raise PerceptionUnknown('Support target not at same-scene table')
  sol=ctx.value('piper_solve_ik',position=target.tolist(),quaternion_xyzw=q,seed=start[:6]);end=sol['q']
  check=ctx.value('piper_check_path',start14=start,target14=end+start[6:])
  if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:raise PerceptionUnknown('Descent CAD/workspace failed')
  from task_geometry import retreat_anchor
  anchor=retreat_anchor(inputs['retreat'][0],p.tolist());up=np.array(anchor['position'])
  if np.linalg.norm(up[:2]-target[:2])>.002 or up[2]<p[2]:raise PerceptionUnknown('Retreat must rise to at least current height')
  ret=ctx.value('piper_solve_ik',position=up.tolist(),quaternion_xyzw=q,seed=end)
  rc=ctx.value('piper_check_path',start14=end+start[6:],target14=ret['q']+start[6:])
  if rc.get('cad_clear') is not True or rc.get('workspace_clear') is not True:raise PerceptionUnknown('No checked loaded upward retreat')
  fingers=obs.get('finger_geometry',{}).get('fingers')
  if not fingers:raise PerceptionUnknown('Current closed finger geometry missing')
  locals=[(np.array(a['cad_corners_base_m'])-T[:3,3])@T[:3,:3] for a in fingers]
  rows=[];min_table=1.;min_red=1.
  from fk_samples import fk_line
  for t,ff in fk_line(ctx,start,end,np.linspace(0,1,41)):
   F=np.array(ff['T_base_flange']);fp=np.array(ff['position'])
   bottom=fp[2]-origin['flange_to_bottom_m'];table=float(fp[:2]@plane[:2]+plane[2])
   if bottom-table<-.002:raise PerceptionUnknown('Sampled payload path goes below nominal table')
   for local in locals:
    corners=local@F[:3,:3].T+F[:3,3];lo=corners.min(axis=0);hi=corners.max(axis=0);min_table=min(min_table,lo[2]-table)
    if lo[2]-table<.012:raise PerceptionUnknown('Sampled finger/table clearance below12mm')
    redlo=red-np.array([.030,.030,.060]);redhi=red+np.array([.030,.030,0.])
    separation=float(np.max(np.maximum(lo-redhi,redlo-hi)));min_red=min(min_red,separation)
    if separation<0:raise PerceptionUnknown('Nominal finger envelope intersects neighboring red cube')
   rows.append({'fraction':float(t),'flange':fp.tolist(),'payload_bottom_m':float(bottom)})
  return {'ok':True,'frame_id':obs['frame_id'],'why':'Retained block, front/back grip, vertical side-by-side corridor, sampled fingers and nominal support height checked; intended block-side proximity only',
   'scene':scene,'payload_origin':origin,'wrist':wrist,'wrist_match':match,'preflight':check,'retreat_preflight':rc,'retreat_position':up.tolist(),'retreat_anchor':anchor,'samples':rows,
   'min_finger_table_m':min_table,'min_nominal_finger_red_separation_m':min_red,'support_required':self.support_required,
   'limits':'60mm red envelope and provisional geometry; images must judge visible relation and support before release'}

class VisualReleaseChecks:
 """Fresh broad support check at the completed final lowering endpoint."""
 def __init__(self,ctx,previous_report):self.ctx=ctx;self.previous_report=previous_report
 def __call__(self,*,phase,before,after,inputs):
  try:
   if phase!='supported_before_release':raise PerceptionUnknown('Unsupported release phase')
   robot=self.ctx.value('piper_status');prev=completed_action_state(read_report(self.previous_report),after,robot)
   if not prev.get('ok') or prev['stage']!='lower_to_support' or not prev.get('support_required'):
    raise PerceptionUnknown('Requires verified final support descent')
   scene=prev['facts'][0]['scene'];scene_control(scene,robot);scene_geometry(scene,after)
   from qualitative import assess_images,reuse_support_endpoint
   fact=reuse_support_endpoint(prev,after,phase)
   if fact is None:
    from ordinary_checks import placement
    fact=placement(self.ctx,phase,after,scene,prev['facts'][0]['payload_origin'])
   if fact.get('ok') is None:fact=assess_images(self.ctx,phase,prev['measured'],after,
    'At the completed table-height placement pose, is orange upright beside red at table level, with no visible change, tipping, hovering or trapped finger that would prevent opening the gripper in place? Judge broad visible support and alignment, not a hidden exact contact surface or force.')
   return fact|{'scene':scene,'payload_origin':prev['facts'][0]['payload_origin']}
  except (PerceptionUnknown,KeyError,ValueError,OSError) as e:return {'ok':None,'frame_id':after.get('frame_id'),'why':str(e)}

class SupportedReleaseChecks:
 """Consume a native operator support fact at its unchanged action endpoint.

 Late answers are usable only after fresh images, unchanged owner and pose,
 and matching last-motion identity. This is supervised assistance, not vision.
 """
 def __init__(self,ctx,previous_report,request_id):self.ctx=ctx;self.previous_report=previous_report;self.request_id=request_id
 def __call__(self,*,phase,before,after,inputs):
  from scene_checks import same_state
  try:
   if phase!='supported_before_release':raise PerceptionUnknown('Support fact only covers opening at this pose')
   robot=self.ctx.value('piper_status');prev=completed_action_state(read_report(self.previous_report),after,robot)
   if prev['stage']!='lower_to_support' or not prev.get('support_required'):raise PerceptionUnknown('Not a completed final support descent')
   f=self.ctx.value('request_operator',request_id=self.request_id,timeout_s=0);context=f.get('context',{})
   if f.get('status')!='answered' or f.get('done') is not True:raise PerceptionUnknown('No native positive support answer')
   if 'support at the current pose before opening' not in f.get('question',''):raise PerceptionUnknown('Operator fact has another scope')
   reply=f.get('reply','')
   if not all(t in reply for t in ('已由桌面支撑','两块直立','夹指未卡住','可以原位张爪')):raise PerceptionUnknown('Support answer requires Agent interpretation; no assumed positive reply')
   if context.get('owner_generation')!=after['state_binding']['owner_generation'] or context.get('bridge_instance')!=robot['bridge']['instance'] or not same_state(context.get('measured14'),after['measured14']):raise PerceptionUnknown('Support fact owner or pose changed')
   if not 0<f.get('answered_at',0)*1e9<=after['captured_at_ns']:raise PerceptionUnknown('Capture fresh evidence after native answer')
   current_image(after,'head');wrist=wrist_between_pads(after,'orange')
   return {'ok':True,'frame_id':after['frame_id'],'why':'Native operator confirmed table support, upright blocks and untrapped fingers; current receipt/owner/pose and coarse wrist remain consistent',
    'assistance':f,'wrist':wrist,'scene':prev['facts'][0]['scene'],'payload_origin':prev['facts'][0]['payload_origin'],
    'scope':'Only opening at current supported pose; not post-release stability or final task completion'}
  except (PerceptionUnknown,KeyError,ValueError,OSError) as e:return {'ok':None,'frame_id':after.get('frame_id'),'why':str(e)}

class RetreatChecks:
 """Upward withdrawal from an inspected release; does not certify full stack."""
 def __init__(self,ctx,previous_report,release_stage='release'):
  if release_stage not in ('release','green_stack_release'):raise ValueError('Unsupported release stage')
  self.ctx=ctx;self.previous_report=previous_report;self.release_stage=release_stage
 def __call__(self,*,phase,before,after,inputs):
  try:
   if phase=='released_and_retreat_path':return self.path(before,inputs)
   if phase=='released_stable_and_clear':
    from qualitative import assess_images
    from ordinary_checks import after_release
    ordinary=after_release(self.ctx,phase,after,self.previous_report)
    if ordinary.get('ok') is not None:return ordinary
    assessment=assess_images(self.ctx,phase,before,after,
      'The open gripper has risen away after a supported orange release. Are red and orange still upright side by side on the table, and is the gripper visibly withdrawn without carrying or continuing to support orange? Judge broad visible stability and separation across views; no pixel-area, width-ratio or hidden exact-gap requirement.')
    assessment['ordinary_evidence']=ordinary
    if assessment.get('ok') is None:assessment['why']=ordinary.get('why','Unresolved endpoint')+'; '+assessment['why']
    return assessment
   raise PerceptionUnknown('Unsupported retreat check')
  except (PerceptionUnknown,KeyError,ValueError,OSError) as e:return {'ok':None,'frame_id':after.get('frame_id'),'why':str(e)}
 def path(self,obs,inputs):
  ctx=self.ctx;robot=ctx.value('piper_status');prev=completed_action_state(read_report(self.previous_report),obs,robot)
  if not prev.get('ok') or prev['stage']!=self.release_stage:raise PerceptionUnknown('Requires completed supported release')
  if not .065<=obs['measured14'][6]<=.074:raise PerceptionUnknown('Retreat requires open jaw')
  current_image(obs,'head');current_image(obs,'left')
  start=obs['measured14'];f=ctx.value('piper_fk',joints=start[:6]);T=np.array(f['T_base_flange']);p=np.array(f['position']);q=inputs['quaternion'];points=inputs['waypoints']
  if len(points)!=1:raise PerceptionUnknown('One retreat segment required')
  target=np.array(points[0])
  if np.linalg.norm(target[:2]-p[:2])>.002 or not .025<=target[2]-p[2]<=.11 or abs(np.dot(q,f['quaternion_xyzw']))<.99999:raise PerceptionUnknown('Retreat must preserve orientation and rise25..110mm vertically')
  sol=ctx.value('piper_solve_ik',position=target.tolist(),quaternion_xyzw=q,seed=start[:6]);end=sol['q'];check=ctx.value('piper_check_path',start14=start,target14=end+start[6:])
  if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:raise PerceptionUnknown('Retreat CAD/workspace failed')
  fingers=obs.get('finger_geometry',{}).get('fingers')
  if not fingers:raise PerceptionUnknown('Current open finger model missing')
  corners=np.concatenate([np.array(a['cad_corners_base_m']) for a in fingers]);local=(corners-T[:3,3])@T[:3,:3];z0=corners[:,2].min();rows=[]
  from fk_samples import fk_line
  for t,ff in fk_line(ctx,start,end,np.linspace(0,1,41)):
   F=np.array(ff['T_base_flange']);fp=np.array(ff['position']);c=local@F[:3,:3].T+F[:3,3]
   if c[:,2].min()<z0-.002 or np.linalg.norm(fp[:2]-p[:2])>.005:raise PerceptionUnknown('Retreat path drifts sideways or below current fingers')
   rows.append({'fraction':float(t),'flange':fp.tolist(),'finger_min_z_m':float(c[:,2].min())})
  return {'ok':True,'frame_id':obs['frame_id'],'why':'Completed supported release, open pads and stable visible block; modeled near-vertical rising path avoids descending below current fingers',
   'preflight':check,'samples':rows,'scope':'Current release pose only; update moved orange; retain stationary neighbors unless displacement is observed'}


def inspect_retreat_endpoint(*,phase,before,after,inputs,ctx=None):
 """The same qualitative result check for normal completion and recovery."""
 try:
  if phase!='final_structure':raise PerceptionUnknown('Unexpected recovery phase')
  from qualitative import assess_images
  return assess_images(ctx,phase,before,after,
   'After opening and withdrawing, are the base blocks broadly upright and stable, and the open fingers separated from them? Use both views; do not require pixel-area ratios, hidden underside visibility or exact alignment.')
 except (PerceptionUnknown,KeyError,ValueError,OSError) as e:
  return {'ok':None,'frame_id':after.get('frame_id'),'why':str(e)}
