"""One green-relative empty approach with retained same-run scene provenance."""
import numpy as np
from task_geometry import envelope_clearance
from scene_checks import PerceptionUnknown,read_report,completed_action_state,installed_finger_corners

PREGRASP_HEIGHT_RANGE = (.180, .240)  # shared planner/execution corridor

def refreshed_base_scene(ctx,obs,robot,stationary_report,*,previous_report=None,invalidated=False):
 """Preserve stationary anchors and update orange from the completed placement.

 Fresh head localization is an explicit recovery, never a mandatory side effect
 of moving toward green. Wrist evidence is complementary, not base metrology.
 """
 if previous_report is None:
  raise PerceptionUnknown('Green entry needs completed empty/release history')
 from history_scene import released_scene
 return released_scene(ctx,obs,robot,previous_report,stationary_report=stationary_report,invalidated=invalidated)

class GreenPregrasp:
 def __init__(self,ctx,previous_report,stationary_report,invalidated=False):self.ctx=ctx;self.previous=previous_report;self.stationary=stationary_report;self.invalidated=invalidated
 def __call__(self,*,phase,before,after,inputs):
  try:
   if phase=='empty_approach_path':return self.path(before,inputs)
   if phase=='grasp_alignment':
    from qualitative import alignment_evidence
    w=alignment_evidence(self.ctx,after,'green')
    return {'ok':True,'frame_id':after['frame_id'],'why':'Green broadly centered between open pads at measured pregrasp endpoint; no closure or retention claim','wrist':w}
   raise PerceptionUnknown('Unexpected green pregrasp phase')
  except (PerceptionUnknown,KeyError,ValueError,OSError) as e:return {'ok':None,'frame_id':after.get('frame_id'),'why':str(e)}
 def prepare(self,obs):
  ctx=self.ctx;robot=ctx.value('piper_status');prev=completed_action_state(read_report(self.previous),obs,robot)
  if not prev.get('ok') or prev['stage']!='empty_inspection':raise PerceptionUnknown('Green pregrasp requires verified empty inspection')
  if not .065<=obs['measured14'][6]<=.074:raise PerceptionUnknown('Expected open empty jaw')
  scene=refreshed_base_scene(ctx,obs,robot,self.stationary,previous_report=self.previous,invalidated=self.invalidated)
  start=obs['measured14'];fk=ctx.value('piper_fk',joints=start[:6]);T=np.asarray(fk['T_base_flange'])
  corn=installed_finger_corners(ctx,obs,scene,fk)
  tool=robot.get('left_tool_calibration',{});tcp=np.asarray(tool.get('T_flange_tcp'),float)
  if tcp.shape!=(4,4) or not np.isfinite(tcp).all() or tool.get('sha256',tool.get('id')) is None:
   raise PerceptionUnknown('Missing installed TCP for orientation planning')
  return {'tcp_offset_flange':tcp[:3,3].tolist(),'start':start,'scene':scene,'local':(corn-T[:3,3])@T[:3,:3],'fk':fk,'frame_id':obs['frame_id']}
 def path(self,obs,inputs):
  evidence=self.prepare(obs)
  result=planned_green_path(self.ctx,evidence['start'],evidence['scene'],evidence['local'],inputs)
  return dict(result,frame_id=obs['frame_id'],scene=evidence['scene'],why='Current completed empty predecessor and scene validated; numerical pregrasp path checked')


def planned_green_path(ctx,start,scene,local,inputs):
 """Pure numeric preview from explicit joints/scene, never an observed state.

 Used identically by candidate search and actual pre-dispatch preflight. Allows
 a bounded near-down empty orientation adjustment; preserves near-down fingers and grasp center.
 """
 p=np.array(inputs['waypoints'][0]);q=inputs['quaternion'];g=np.array(scene['blocks']['green']['top_center']['surface_position_m'])
 if len(inputs['waypoints'])!=1 or np.linalg.norm(p[:2]-g[:2])>.012 or not PREGRASP_HEIGHT_RANGE[0]<=p[2]-g[2]<=PREGRASP_HEIGHT_RANGE[1]:raise PerceptionUnknown('Outside green-relative pregrasp corridor')
 fk=ctx.value('piper_fk',joints=start[:6]);T=np.array(fk['T_base_flange']);p0=np.array(fk['position'])
 if T[2,2]>-.995:raise PerceptionUnknown('Pregrasp starts outside near-down orientation')
 if abs(np.dot(q,fk['quaternion_xyzw']))<np.cos(np.deg2rad(10)/2)-1e-9:
  raise PerceptionUnknown('Empty pregrasp orientation adjustment exceeds10deg')
 q=np.asarray(q,float)
 if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-5:raise PerceptionUnknown('Invalid pregrasp quaternion')
 x,y,z,w=q;target_axis=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
 if target_axis[2]>-.995:raise PerceptionUnknown('Pregrasp target must remain near-down')
 q=q.tolist()
 # Match native piper_move_pose; remove only the obsolete120mm task cap.
 if np.linalg.norm(p-p0)>.4+1e-9:raise PerceptionUnknown('Pregrasp translation exceeds native400mm limit')
 sol=ctx.value('piper_solve_ik',position=p.tolist(),quaternion_xyzw=q,seed=start[:6]);end=sol['q'];check=ctx.value('piper_check_path',start14=start,target14=end+start[6:])
 if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:raise PerceptionUnknown('Native pregrasp path failed')
 top=max(b['top_center']['surface_position_m'][2] for b in scene['blocks'].values());samples=[];minimum=1.;min_allowance=1.
 for t in np.linspace(0,1,41):
  f=ctx.value('piper_fk',joints=(np.array(start[:6])+(np.array(end)-start[:6])*t).tolist());F=np.array(f['T_base_flange']);c=local@F[:3,:3].T+F[:3,3];gap=float(c[:,2].min()-top);minimum=min(minimum,gap)
  if F[2,2]>-.995:raise PerceptionUnknown('Native pregrasp interpolation leaves near-down orientation')
  clearance=envelope_clearance(list(c.reshape(-1,8,3)),scene)
  min_allowance=min(min_allowance,clearance['min_block_separation_lower_m'])
  samples.append({'fraction':float(t),'flange':f['position'],'finger_min_z':float(c[:,2].min())})
 return {'ok':True,'end_joints':end,'preflight':check,'samples':samples,'min_nominal_finger_top_m':minimum,'min_block_allowance_m':min_allowance,
  'why':'Prospective numerical full-finger path separated from modeled blocks/table; actual state and provenance are validated by caller',
  'limits':'3D envelope check with20mm allowance is task geometry, not independently calibrated clearance; native guards unchanged; inspect current wrist before descent'}
