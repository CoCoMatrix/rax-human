"""Invoke exactly one named candidate function, recording actual elapsed time."""
import argparse,importlib,json,os,sys,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--session',type=Path,required=True);p.add_argument('--name',required=True);p.add_argument('--function',required=True);p.add_argument('--arguments',default='{}');a=p.parse_args()
ready=json.load(open(a.session/'REHEARSAL-READY.json'));w=Path(ready['workspace']);os.chdir(w);sys.path.insert(0,str(w));sys.path.insert(0,str(w/'code/blue_cup'));os.environ['RAX_MCP_SOCKET']=ready['socket']
import controller as c
import maneuvers as m
from api.runtime import Context
functions={name:getattr(m,name) for name in ('pose_step','jaw_check','close_wall','retention_lift','transfer_step','lower_step','release_step','retreat_step')};functions.update(locate=c.locate,inspect_only=c.inspect_only)
out=w/'output/rehearsal'/f'{a.name}.json';out.parent.mkdir(parents=True,exist_ok=True)
if out.exists():raise ValueError('Step name already recorded; choose a new attempt name')
started=time.time();t=time.monotonic();r={'name':a.name,'function':a.function,'arguments':json.loads(a.arguments),'started_at':started,'ok':False}
try:
 with Context() as ctx:
  if a.function=='observe':
   s=ctx.value('status');o=m.observe(ctx);v={'ok':True,'status':s,'observation':o['value']}
  else:v=functions[a.function](ctx,**r['arguments'])
  r.update(ok=v.get('ok') is True,result=v)
except Exception as e:r['error']=repr(e)
finally:
 r['elapsed_s']=time.monotonic()-t;out.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
raise SystemExit(0 if r['ok'] else 1)
