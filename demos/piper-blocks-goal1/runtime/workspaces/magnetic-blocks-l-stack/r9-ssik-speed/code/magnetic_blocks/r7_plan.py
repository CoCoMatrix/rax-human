"""Read-only current-scene candidate comparison, no motion."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from api.runtime import Context
from scene_checks import measure_scene,scene_control

def main(ctx):
 s=ctx.value('status');o=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=True)
 scene=measure_scene(ctx,o);scene_control(scene,s['robot_status'],establish=True)
 f=ctx.value('piper_fk',joints=o['measured14'][:6]);p=scene['blocks']['orange']['top_center']['surface_position_m'][:];p[2]+=.235
 R0=Rotation.from_matrix(np.array(f['T_base_flange'])[:3,:3]);R1=Rotation.from_euler('z',-3,degrees=True)*Rotation.from_euler('x',180,degrees=True)
 interp=Slerp([0,1],Rotation.concatenate([R0,R1]));rows=[]
 for frac in [1,.65,.45,.3]:
  q=interp(frac).as_quat().tolist();row={'fraction':frac,'position':p,'quaternion':q}
  try:
   r=ctx.value('piper_solve_ik',position=p,quaternion_xyzw=q,seed=o['measured14'][:6]);row['ik']=r
   row['max_joint_delta']=float(max(abs(np.array(r['q'])-o['measured14'][:6])))
   row['path']=ctx.value('piper_check_path',start14=o['measured14'],target14=r['q']+o['measured14'][6:])
  except RuntimeError as e:
   row['error']=str(e)
   if 'replan_read_only' not in str(e):raise
  rows.append(row)
 return {'scene':scene,'observation':o,'status':s,'fk':f,'candidates':rows}
if __name__=='__main__':
 with Context() as ctx:r=main(ctx)
 path=Path(sys.argv[1]);path.write_text(json.dumps(r,indent=2))
 for c in r['candidates']:
  print(json.dumps({k:v for k,v in c.items() if k not in ['ik','path']}|{'q':c.get('ik',{}).get('q'),'path':{k:c.get('path',{}).get(k) for k in ['cad_clear','workspace_clear']}}))
 print('budget',r['status']['wall_budget'])
