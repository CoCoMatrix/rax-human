from numeric_policy import OPEN_JAW_ERROR_M, LOCALIZATION_ERROR_M, DOWN_COS
"""On-demand qualitative image checks, preserving numeric control contracts."""
import base64,json
from pathlib import Path
from scene_checks import current_image,PerceptionUnknown,same_state

class EvidenceBindingError(PerceptionUnknown):
 """The endpoint changed; optional perception cannot hide this discrepancy."""

# This is a brief same-endpoint reuse window, never a permission to skip a
# support judgment. Longer pauses or visible change use a new qualitative call.
SUPPORT_REUSE_MAX_AGE_S = 30

def reuse_support_endpoint(previous, after, phase):
 """Reuse an already positive support relation, not infer support from stillness.

 Caller must first validate the completed action, last native request, owner,
 scene and release preconditions. Both fresh views are checked here for change.
 Image thresholds are coarse change detectors; they do not measure alignment,
 support force, underside clearance, or certify an invisible contact.
 """
 import cv2,numpy as np
 from scene_checks import same_state
 try:
  old=previous['measured'];fact=(previous.get('recovered_endpoint') or previous['facts'])[-1]
  age=(after['captured_at_ns']-old['captured_at_ns'])/1e9
  if (previous.get('ok') is not True or
      previous.get('stage') not in ('lower_to_support','green_stack_lower') or
      fact.get('ok') is not True or fact.get('supported') is not True or
      fact.get('frame_id')!=old['frame_id'] or not 0<=age<=SUPPORT_REUSE_MAX_AGE_S or
      not same_state(old['measured14'],after['measured14']) or
      old['state_binding']['owner_generation']!=after['state_binding']['owner_generation'] or
      not old.get('calibration_sha256') or old['calibration_sha256']!=after.get('calibration_sha256') or
      not old.get('finger_geometry',{}).get('geometry_identity') or
      old['finger_geometry']['geometry_identity']!=after.get('finger_geometry',{}).get('geometry_identity')):
   return None
  diagnostics={}
  for camera in ('head','left'):
   a=current_image(old,camera,require_current=False)
   b=current_image(after,camera)
   # Local tiles catch a moved block even when it occupies little of the image.
   a=cv2.resize(cv2.GaussianBlur(a,(7,7),0),(320,180)).astype(float)
   b=cv2.resize(cv2.GaussianBlur(b,(7,7),0),(320,180)).astype(float)
   diff=np.max(np.abs(a-b),axis=2)
   tiles=diff.reshape(18,10,32,10).mean(axis=(1,3))
   changed=int(np.count_nonzero(tiles>18))
   diagnostics[camera]={'changed_tiles':changed,'largest_tile_difference':float(tiles.max())}
   if changed>=2:return None
  return {'ok':True,'supported':True,'frame_id':after['frame_id'],
          'source_frame_id':old['frame_id'],'reused_support':True,'age_s':age,
          'phase':phase,'change_diagnostics':diagnostics,
          'why':'Recent positive qualitative support; same checked endpoint and no substantial change in fresh head/wrist views',
          'scope':'Reuse of this final lowering support judgment only; not proof of hidden contact forces'}
 except (PerceptionUnknown,KeyError,TypeError,ValueError,OSError):
  return None

def wrist_depth_evidence(ctx,obs,color,*,request_if_missing=False):
 """Optional camera-optical evidence, always selected in its own RGB frame."""
 import cv2,numpy as np
 from scene_checks import same_state,scene_geometry,WRIST_BODY_ROI
 from perception import HSV_RANGES,supported_depth_pixels
 current_image(obs,'left')
 sample=obs
 if not obs['cameras']['left'].get('metric_depth'):
  if not request_if_missing:return {'available':False,'why':'No bound wrist depth in this observation'}
  sample=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=False,depth_cameras=['left'])
  if (not same_state(obs['measured14'],sample['measured14']) or
      obs['state_binding']['owner_generation']!=sample['state_binding']['owner_generation'] or
      sample['captured_at_ns']<obs['captured_at_ns'] or sample['frame_id']==obs['frame_id']):
   raise EvidenceBindingError('State changed during optional wrist observation')
  try:scene_geometry({'frame_id':obs['frame_id']},sample)
  except PerceptionUnknown as exc:raise EvidenceBindingError(str(exc)) from exc
 rgb=current_image(sample,'left');meta=sample['cameras']['left']
 if meta.get('binding')!='same immutable stereo/RGB packet' or not meta.get('metric_depth') or meta.get('base_transform_available'):
  raise PerceptionUnknown('Expected bound wrist optical depth without a base transform')
 with __import__('numpy').load(Path('runtime/frames')/sample['frame_id']/'left-depth.npz',allow_pickle=False) as d:
  xyz=d['xyz_rgb_m'];valid=d['valid'].astype(bool)&np.isfinite(xyz).all(axis=2)&(xyz[:,:,2]>0)
 if xyz.shape!=(720,1280,3) or valid.shape!=(720,1280):raise PerceptionUnknown('Malformed wrist depth')
 hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV);mask=np.zeros((720,1280),np.uint8)
 for lo,hi in HSV_RANGES[color]:mask|=cv2.inRange(hsv,lo,hi)
 x0,y0,x1,y1=WRIST_BODY_ROI;roi=np.zeros_like(mask);roi[y0:y1,x0:x1]=1
 distance=cv2.distanceTransform(((mask>0)&(roi>0)).astype(np.uint8),cv2.DIST_L2,5)
 pixels,selection=supported_depth_pixels(distance,valid,[])
 result=ctx.value('piper_pixel_to_position',camera='left',frame_id=sample['frame_id'],pixels=pixels,radius=3,output_frame='camera_optical')
 if (result.get('robot_target_usable') is not False or result.get('base_transform_applied') is not False
     or result.get('frame_id')!=sample['frame_id'] or result.get('camera')!='left'
     or result.get('frame')!='left_d435_color_optical_frame'
     or len(result.get('points',[]))!=len(pixels)):
  raise PerceptionUnknown('Wrist result violates optical-only contract')
 return {'available':True,'frame_id':sample['frame_id'],'observation':sample,'points':result['points'],
         'selection':selection,'robot_target_usable':False,'base_transform_applied':False,
         'scope':'Visible colored object optical depth; cannot establish hidden support or base-frame clearance'}


def alignment_evidence(ctx,obs,color,*,scene=None,reference14=None):
 if scene is not None:
  import numpy as np
  from scene_checks import scene_geometry
  from ordinary_checks import view_evidence,visible_near_point,wrist_alignment
  scene_geometry(scene,obs)
  top=np.asarray(scene['blocks'][color]['top_center']['surface_position_m'])
  T=np.asarray(ctx.value('piper_fk',joints=obs['measured14'][:6])['T_base_flange'])
  delta=T[:3,3]-top
  tcp=T[:3,3]+T[:3,:3]@np.array([0.,0.,.140])
  center_error=tcp-(top+[0,0,-.012])
  from assembly_plan import orientation_evidence
  if reference14 is not None:
   if not same_state(reference14,reference14):raise PerceptionUnknown('Malformed grasp orientation reference')
   bound_reference=list(reference14)
   # Only the arm orientation is referenced. Actual jaw opening was already
   # checked against the previous receipt by Execution.guard; a nominal open
   # width must not reintroduce a second0.5mm opening-completion threshold.
   bound_reference[6]=obs['measured14'][6]
   if not same_state(obs['measured14'],bound_reference):raise PerceptionUnknown('Grasp orientation reference no longer bound to measured state')
  orientation=orientation_evidence(T,color)
  if np.linalg.norm(center_error[:2])>.012+LOCALIZATION_ERROR_M or abs(center_error[2])>.012+LOCALIZATION_ERROR_M:
   raise PerceptionUnknown('Measured grasp geometry contradicts retained block target')
  head=view_evidence(visible_near_point,obs,color,top)
  wrist=view_evidence(wrist_alignment,obs,color) if not head.get('visible') else {'visible':False,'not_needed':True}
  if head.get('visible') or wrist.get('visible'):
   return {'ok':True,'frame_id':obs['frame_id'],'head':head,'wrist':wrist,'flange_relative_to_top_m':delta.tolist(),'orientation':orientation,
           'why':'Measured grasp geometry and a recognizable head/wrist target agree qualitatively'}
  fact=assess_images(ctx,'qualitative_alignment',None,obs,
       f'Is the {color} block broadly aligned with the open pads? Measured geometry is consistent; allow partial occlusion.',color=color)
  if fact.get('ok') is not True:raise PerceptionUnknown('Alignment unresolved in available views: '+fact['why'])
  return fact
 from scene_checks import wrist_between_pads
 try:return wrist_between_pads(obs,color)
 except PerceptionUnknown as e:
  # A partial silhouette is an invitation to inspect other evidence, not a
  # reason to require the same pixel mask again before reaching the view pose.
  fact=assess_images(ctx,'qualitative_alignment',None,obs,
    f'Is the {color} block broadly aligned between the two open pads for the already geometry-checked approach? Partial occlusion is allowed. Look for obvious displacement or tilt; do not require complete silhouettes or hidden millimetre gaps.',color=color)
  if fact.get('ok') is not True:raise PerceptionUnknown('Alignment needs Agent review: '+fact['why']) from e
  return fact


def retention_at_endpoint(ctx,previous,after,color):
 """Consume a verified endpoint; do not compare an immutable frame to itself.

 Caller first validates completed_action_state with current native status and
 scene identity. Only the brief receipt-cache interval can reuse the SAME frame.
 A new frame follows the existing visual comparison; actual motion endpoints
 must always use retention_evidence, which still requires distinct frames.
 """
 import time
 from observations import MAX_REUSE_S
 old=previous['measured']
 if old.get('frame_id')!=after.get('frame_id'):
  return retention_evidence(ctx,old,after,color)
 facts=previous.get('facts') or []
 fact=facts[-1] if facts else {}
 origin=facts[0].get('payload_origin',{}) if facts else {}
 held=previous.get('held_block',origin.get('color'))
 positive=(fact.get('ok') is True and fact.get('frame_id')==old.get('frame_id') and
           (fact.get('wrist_match',{}).get('ok') is True or
            ('match' in fact and previous.get('stage')=='green_stack_lower')))
 if (previous.get('ok') is not True or
     previous.get('stage') not in ('trial_lift','transport','lower_to_support','green_stack_lower') or
     held!=color or not positive or old!=after or
     not 0<=(time.time_ns()-after['captured_at_ns'])/1e9<=MAX_REUSE_S):
  raise PerceptionUnknown('Same-frame retention requires a recent successful bound loaded endpoint')
 for camera in ('head','left'):current_image(after,camera)
 return {'ok':True,'frame_id':after['frame_id'],'reused_retention':True,
         'source_stage':previous['stage'],'source_frame_id':old['frame_id'],
         'why':'Reuse verified retention at the unchanged completed action endpoint',
         'scope':'Prior positive retention only; new path geometry remains independently checked'}


def retention_evidence(ctx,before,after,color):
 import numpy as np
 from scene_checks import wrist_trial_match,wrist_between_pads,current_image
 match=wrist_trial_match(before,after)
 # Coarse object-to-pad relation is independent of a fixed background patch.
 # Numerical segmentation is only a recognizable-object test, not metrology.
 try:
  a=wrist_between_pads(before,color,require_current=False);b=wrist_between_pads(after,color)
  delta=np.array([b['body_center_x_px']-a['body_center_x_px'],b['body_center_y_px']-a['body_center_y_px']])
  if np.linalg.norm(delta)>80:raise PerceptionUnknown('Visible block moved substantially relative to pads')
  if match['reliable_match'] or np.linalg.norm(delta)<35:
   return {'ok':True,'match':match,'body_shift_px':delta.tolist(),
           'why':'Recognizable body remains broadly in the same pad-relative region; patch score is diagnostic only'}
 except PerceptionUnknown:
  pass
 fact=assess_images(ctx,'qualitative_retention',before,after,
   f'Does the {color} block remain broadly at the same grip position relative to the pads, without obvious slip, drop or tilt? Compare object and pads, allowing background and perspective changes; no exact patch-match requirement.',color=color)
 if fact.get('ok') is not True:raise PerceptionUnknown('Retention needs Agent review: '+fact['why'])
 return fact|{'match':match}

def assess_images(ctx,phase,before,after,question,*,color='orange'):
 if getattr(ctx,'allow_auxiliary_vision',False) is not True:
  return {'ok':None,'frame_id':after.get('frame_id'),'phase':phase,
          'why':'Ordinary evidence unresolved; optional auxiliary vision is disabled',
          'optional_auxiliary_available':True,'model_called':False}
 # Qualitative support/alignment does not itself require metric depth.
 # Use already-bound wrist depth if available; do not start FFS merely because
 # the head image is occluded. Metric localization has its separate entry.
 depth=None
 if ctx is not None:
  from perception import component
  try:component(current_image(after,'head'),color)
  except PerceptionUnknown:
   try:depth=wrist_depth_evidence(ctx,after,color,request_if_missing=False)
   except EvidenceBindingError:raise
   except (PerceptionUnknown,ValueError,OSError,RuntimeError) as exc:
    depth={'available':False,'why':str(exc)}
 content=[{'type':'text','text':'Assess broad task relations using the provided measured-motion context and the head/wrist views together. Small misalignment and partial occlusion are acceptable. Do not require perfect overlap, visible hidden undersides, exact contact or force proof. Use no only for an obvious task-relevant contradiction, not perspective foreshortening alone; use unknown only when the broad relation itself remains unresolved. Do not invent hidden facts. Orange fork-shaped objects are gripper pads, not blocks. The head camera looks down toward the table; the wrist camera moves with the tool and its image axes are not world height. Judge 3D layout from surfaces and perspective, not image up/down alone. Earlier images precede current images. '+question+' Return JSON {"verdict":"yes"|"no"|"unknown","reason":string}. Use yes when the requested broad visible relation is supported, no for visible contradiction, unknown when the broad task relation cannot be judged from the available views and context. Do not use pixel-area or numerical alignment thresholds.'}]
 visual_after=depth.get('observation',after) if depth else after
 if depth:content.append({'type':'text','text':'Optional wrist optical evidence (NOT base targets): '+json.dumps({k:v for k,v in depth.items() if k!='observation'})})
 for label,obs in [('before',before),('current',visual_after)]:
  if obs is None:continue
  for camera in ['head','left']:
   current_image(obs,camera,require_current=label=='current')
   content += [{'type':'text','text':label+' '+camera+' frame '+obs['frame_id']},
    {'type':'image','mimeType':'image/jpeg','data':base64.b64encode(Path(obs['cameras'][camera]['inspection_image']).read_bytes()).decode()}]
 r=ctx.value('gemini_generate',messages=[{'role':'user','content':content}],reasoning={'effort':'low'},max_tokens=1600,response_format={'type':'json_object'})
 if r.get('finish_reason')!='stop' or r.get('message',{}).get('refusal'):raise PerceptionUnknown('Incomplete qualitative visual response')
 d=json.loads(r['message']['content'])
 if d.get('verdict') not in ('yes','no','unknown') or not isinstance(d.get('reason'),str):raise PerceptionUnknown('Malformed visual verdict')
 return {'ok':{'yes':True,'no':False,'unknown':None}[d['verdict']],'frame_id':after['frame_id'],
         'visual_frame_id':visual_after['frame_id'],'wrist_depth':depth,'why':d['reason'],'visual_verdict':d,'phase':phase,'usage':r.get('usage'),'budget':r.get('budget'),'scope':'Qualitative image evidence only; numerical/path checks independent'}
