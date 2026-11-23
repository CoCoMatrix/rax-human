"""Read-only current orange-relative candidate comparison, no robot motion."""
import json,sys,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from api.runtime import Context
from scene_checks import measure_scene,scene_control

def main(ctx):
 s=ctx.value('status');o=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=True)
 scene=measure_scene(ctx,o);scene_control(scene,s['robot_status'],establish=True)
 p=scene['blocks']['orange']['top_center']['surface_position_m'][:];p[2]+=.225
 rows=[]
 for yaw in [0,-3,-6,-10]:
  q=(Rotation.from_euler('z',yaw,degrees=True)*Rotation.from_euler('x',180,degrees=True)).as_quat().tolist()
  row={'yaw_deg':yaw,'position':p,'quaternion_xyzw':q}
  try:
   r=ctx.value('piper_solve_ik',position=p,quaternion_xyzw=q,seed=o['measured14'][:6]);row['ik']=r
  except RuntimeError as e:
   row['error']=str(e)
   if 'replan_read_only' not in str(e): raise
  rows.append(row)
 return {'scene':scene,'observation':o,'status':s,'candidates':rows}
if __name__=='__main__':
 with Context() as ctx:r=main(ctx)
 Path('output/rehearsal/r6-plan-01.json').write_text(json.dumps(r,indent=2))
 for c in r['candidates']:print(json.dumps(c))
 print('budget',r['status']['wall_budget'])
