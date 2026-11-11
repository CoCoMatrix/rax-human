"""Read-only reachability probes; no motion."""
import json
from api.runtime import Context
with Context() as ctx:
    state=ctx.value('piper_status')
    q=state['measured_state']['values'][:6]
    fk=ctx.value('piper_fk',joints=q,arm='left')
    for p in [[.24006,-.08356,.30057],[.24006,-.10356,.30057],[.24006,-.08356,.29057]]:
        try:
            r=ctx.value('piper_solve_ik',position=p,quaternion_xyzw=fk['quaternion_xyzw'],seed=q)
            print(json.dumps({'position':p,'result':r}))
        except Exception as e: print(json.dumps({'position':p,'error':str(e)}))
