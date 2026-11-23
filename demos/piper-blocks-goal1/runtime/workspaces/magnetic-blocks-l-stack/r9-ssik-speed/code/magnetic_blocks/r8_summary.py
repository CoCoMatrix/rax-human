"""Read-only concise R8 invocation ledger; preserves original evidence."""
import json
from pathlib import Path

def main():
 rows=[]
 for p in sorted(Path('output/rehearsal').glob('r8-*.json')):
  r=json.loads(p.read_text())
  if 'result' not in r:continue
  z=r['result']; o=z.get('measured') or {}
  rows.append({'report':str(p),'function':r.get('function'),'started_unix':r.get('started_unix'),
   'ok':z.get('ok'),'executed':z.get('executed'),'receipts':len(z.get('receipts',[])),
   'wall_s':r.get('wall_s'),'auxiliary_visual_calls':r.get('auxiliary_visual_calls'),
   'why':z.get('why'),'frame_id':o.get('frame_id'),'assistance':z.get('assistance')})
 rows.sort(key=lambda x:x['started_unix'])
 Path('output/rehearsal/r8-run-summary.json').write_text(json.dumps(rows,indent=2))
 old=Path('memory/RUNS.md').read_text().split('\n# R8 live rehearsal')[0]
 text='\n# R8 live rehearsal\n\nSupervised single-function invocations; not unattended runs. Primary Agent decisions between invocations; primary-model call count is not instrumented by the runner. Auxiliary calls and duration are measured. Report code_hashes identify each exact implementation.\n\n| Report | Function | OK | Motions | Seconds | Auxiliary calls |\n|---|---|---|---|---|---|\n'
 for r in rows:text+=f"| {Path(r['report']).name} | {r['function']} | {r['ok']} | {r['receipts']} | {r['wall_s']:.3f} | {r['auxiliary_visual_calls']} |\n"
 Path('memory/RUNS.md').write_text(old+text)
 print(json.dumps(rows[-3:],indent=2))
if __name__=='__main__':main()
