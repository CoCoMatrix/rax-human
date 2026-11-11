#!/usr/bin/env python3
"""Run the native XPolicy owner/UI for Prepare; no agent, policy or motion bridge.
Prepare is the owner's ordinary initial-pose command (zero joints + closed jaws).
"""
import argparse,copy,json,os,secrets,signal,subprocess,sys,time
from pathlib import Path
import yaml
from piper_observe_only import ROOT,OWNER,GRAPH,OWNER_PYTHON,DORA,make_graph,no_existing_devices
from piper_minimal_live import Operator

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true');p.add_argument('--seconds',type=int,default=1800);a=p.parse_args()
 out=a.output.resolve()
 if not out.is_relative_to(ROOT/'commissioning') or out.exists():p.error('Use new commissioning output')
 if not 30<=a.seconds<=3600:p.error('seconds30..3600')
 source=yaml.safe_load(GRAPH.read_text());profile=yaml.safe_load((OWNER/'real_world/robots/piper_x_dual/profile.yml').read_text())
 # Restore ordinary Prepare's declared jaw targets; no RAX home/preserve-jaw override.
 profile['live_takeover']['gripper_write_enabled']=True
 plan={'native_prepare':profile['prepare'],'model_started':False,'policy_started':False,'return_to_zero_by':'XPolicy native Prepare','profile_change':{'live_takeover.gripper_write_enabled':True}}
 if not a.run:print(json.dumps(plan,indent=2));return
 no_existing_devices();out.mkdir(parents=True,mode=0o700)
 for n in ('private','tmp','cache'):(out/n).mkdir(mode=0o700)
 token=secrets.token_urlsafe(32);(out/'private/owner-token').write_text(token);(out/'private/owner-token').chmod(0o600)
 (out/'profile.yml').write_text(yaml.safe_dump(profile,sort_keys=False));(out/'tasks.json').write_text(json.dumps({'canonical_prompts':['XPolicy Prepare: zero joints and closed jaws']}))
 graph=make_graph(source,out,token);orig={n['id']:n for n in source['nodes']}
 mux=copy.deepcopy(orig['piper_control_mux']);mux['env']=graph['nodes'][0]['env'].copy();mux['path']='/usr/bin/env';mux['args']=f"{OWNER_PYTHON} -m "+mux['args'].split(' -m ',1)[1];graph['nodes'].append(mux)
 for n in graph['nodes']:
  e=n['env'];e.update(ENABLE_WRITE='1',PIPER_PROFILE=str(out/'profile.yml'),PIPER_PREPARE_HOLD_CURRENT='0',RAX_PREPARE_LEFT_HOME='0',RAX_LEFT_GRIPPER_DISPATCH='0',REALSENSE_STEREO_EXPORT_DIR='',TASK='XPolicy Prepare: zero joints and closed jaws')
  if n['id']=='piper_x_dual_owner':n['inputs']['button_command']=orig[n['id']]['inputs']['button_command']
 (out/'graph.yml').write_text(yaml.safe_dump(graph));(out/'graph.yml').chmod(0o600);(out/'PLAN.json').write_text(json.dumps(plan,indent=2))
 env={k:v for k,v in os.environ.items() if not k.startswith(('PIPER_','RAX_','REALSENSE_','ENABLE_WRITE','POLICY_','ACTION_','XPOLICY_'))};env.update(PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(OWNER))
 with (out/'graph.log').open('w') as log:child=subprocess.Popen([str(DORA),'run','--stop-after',str(a.seconds)+'s',str(out/'graph.yml')],cwd=out,env=env,stdout=log,stderr=log,start_new_session=True)
 def stop(*_):raise KeyboardInterrupt
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
 sys.path.insert(0,str(ROOT));from services.piper.owner import OwnerReader
 from services.piper.cameras import Cameras
 op=Operator(out);reader=OwnerReader();result={}
 try:
  deadline=time.monotonic()+45
  while True:
   if child.poll() is not None:raise RuntimeError('Owner graph exited')
   try:
    s=reader.state()
    if s['robot_status'].get('status')=='READY_LIVE_TAKEOVER':break
   except (OSError,ValueError,KeyError):pass
   if time.monotonic()>deadline:raise TimeoutError('Owner ready')
   time.sleep(.1)
  obs,_=Cameras(reader,out/'frames').capture(['head','left','right'],False)
  (out/'READY.json').write_text(json.dumps({'state':s,'observation':obs,'pid':os.getpid()},indent=2))
  print(json.dumps({'ready_for_native_prepare':True,'output':str(out),'frame_id':obs['frame_id']}),flush=True)
  while child.poll() is None:time.sleep(.5)
 except BaseException as exc:
  result['exit_reason']=type(exc).__name__+': '+str(exc);print(result['exit_reason'],file=sys.stderr,flush=True)
 finally:
  try:
   result['last_state']=reader.state();result['stop']=op.command('stop')
  except Exception as exc:result['stop_error']=str(exc)
  if child.poll() is None:
   os.killpg(child.pid,signal.SIGINT)
   try:child.wait(timeout=25)
   except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
  result['graph_exit_code']=child.returncode;(out/'RESULT.json').write_text(json.dumps(result,indent=2))
 if result.get('exit_reason') and not result['exit_reason'].startswith('KeyboardInterrupt'):raise SystemExit(1)
if __name__=='__main__':main()
