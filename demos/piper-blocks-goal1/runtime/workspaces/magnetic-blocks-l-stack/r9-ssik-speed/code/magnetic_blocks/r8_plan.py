"""Bounded current-state native IK/path comparison; never motion."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from api.runtime import Context

def main(ctx,candidates):
 s=ctx.value('status');o=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=False)
 f=ctx.value('piper_fk',joints=o['measured14'][:6]);rows=[]
 if len(candidates)>4:raise ValueError('At most four explicit candidates')
 for candidate in candidates:
  row=dict(candidate)
  try:
   ik=ctx.value('piper_solve_ik',position=candidate['position'],quaternion_xyzw=candidate['quaternion'],seed=o['measured14'][:6]);row['ik']=ik
   row['max_joint_delta']=max(abs(a-b) for a,b in zip(ik['q'],o['measured14'][:6]))
   row['path']=ctx.value('piper_check_path',start14=o['measured14'],target14=ik['q']+o['measured14'][6:])
  except RuntimeError as e:
   row['error']=str(e)
   if 'replan_read_only' not in str(e):raise
  rows.append(row)
 return dict(status=s,observation=o,fk=f,candidates=rows)
if __name__=='__main__':
 with Context() as ctx:r=main(ctx,json.loads(Path(sys.argv[1]).read_text()))
 Path(sys.argv[2]).write_text(json.dumps(r,indent=2))
 print(json.dumps({'frame':r['observation']['frame_id'],'fk':r['fk']['position'],'candidates':[{k:v for k,v in c.items() if k not in ('ik','path')}|{'q':c.get('ik',{}).get('q'),'path':{k:c.get('path',{}).get(k) for k in ['cad_clear','workspace_clear']}} for c in r['candidates']],'budget':r['observation']['wall_budget']},indent=2))
