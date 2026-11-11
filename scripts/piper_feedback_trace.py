#!/usr/bin/env python3
"""Bounded read-only owner + passive SocketCAN/SDK/HTTP/clock trace."""
import argparse,copy,json,os,secrets,signal,socket,struct,subprocess,threading,time,urllib.request
from pathlib import Path
import yaml
from piper_observe_only import ROOT,OWNER,GRAPH,OWNER_PYTHON,DORA,make_graph,no_existing_devices

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=int,default=45);p.add_argument('--hold-current',action='store_true');a=p.parse_args()
 out=a.output.resolve()
 if not out.is_relative_to(ROOT/'commissioning') or out.exists() or not 15<=a.seconds<=180:p.error('new commissioning output, 15..180s')
 no_existing_devices();out.mkdir(parents=True,mode=0o700)
 for name in ['tmp','cache']:(out/name).mkdir()
 (out/'tasks.json').write_text(json.dumps({'canonical_prompts':['Read-only feedback diagnosis']}))
 source=yaml.safe_load(GRAPH.read_text());token=secrets.token_urlsafe(32)
 (out/'private').mkdir(mode=0o700);(out/'private/owner-token').write_text(token);(out/'private/owner-token').chmod(0o600)
 graph=make_graph(source,out,token)
 if a.hold_current:
  original={n['id']:n for n in source['nodes']}
  mux=copy.deepcopy(original['piper_control_mux']);mux['env']=graph['nodes'][0]['env'].copy();mux['path']='/usr/bin/env';mux['args']=str(OWNER_PYTHON)+' -m '+mux['args'].split(' -m ',1)[1];graph['nodes'].append(mux)
  for n in graph['nodes']:
   n['env'].update(ENABLE_WRITE='1',PIPER_PREPARE_HOLD_CURRENT='1',PIPER_GPT_TRAJECTORY_RELEASE='1',RAX_PREPARE_LEFT_HOME='0',RAX_LEFT_GRIPPER_DISPATCH='0',PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M='')
   if n['id']=='piper_x_dual_owner':n['inputs']['button_command']=original[n['id']]['inputs']['button_command']
 for n in graph['nodes']:
  n['env'].update(PYTHONPATH=str(ROOT)+':'+str(OWNER),TASK='Read-only feedback diagnosis',REALSENSE_STEREO_EXPORT_DIR='',RAX_FEEDBACK_TRACE=str(out/'sdk-owner.jsonl'))
  if n['id']=='piper_x_dual_owner':n['args']=str(OWNER_PYTHON)+' -m scripts.piper_feedback_trace_owner'
 (out/'graph.yml').write_text(yaml.safe_dump(graph))
 done=threading.Event();errors=[]
 def raw():
  s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW);s.bind(('can_px_00440026',));s.settimeout(.1)
  with (out/'can.jsonl').open('w',buffering=1) as f:
   while not done.is_set():
    try:frame=s.recv(16)
    except socket.timeout:continue
    cid,n,data=struct.unpack('=IB3x8s',frame);cid &=0x1fffffff
    if cid in [0x2A5,0x2A6,0x2A7,0x251,0x252,0x253,0x254,0x255,0x256,0x150,0x151,0x155,0x156,0x157,0x2A1]:
     f.write(json.dumps({'mono_ns':time.monotonic_ns(),'wall_ns':time.time_ns(),'id':hex(cid),'data':data[:n].hex()})+'\n')
  s.close()
 def http():
  opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
  with (out/'http.jsonl').open('w',buffering=1) as f:
   while not done.is_set():
    try:
     before=time.monotonic_ns()
     with opener.open('http://127.0.0.1:8091/api/state',timeout=1) as response:d=json.load(response)
     f.write(json.dumps({'mono_before_ns':before,'mono_ns':time.monotonic_ns(),'wall_ns':time.time_ns(),'robot_state':d.get('robot_state'),'robot_status':d.get('robot_status')})+'\n')
    except Exception as e:errors.append(str(e)) if len(errors)<10 else None
    done.wait(.02)
 def clocks():
  with (out/'clocks.jsonl').open('w',buffering=1) as f:
   while not done.is_set():
    before=time.monotonic_ns();wall=time.time_ns();after=time.monotonic_ns()
    f.write(json.dumps({'mono_before_ns':before,'wall_ns':wall,'mono_ns':after})+'\n');done.wait(.01)
 threads=[threading.Thread(target=fn,daemon=True) for fn in [raw,http,clocks]]
 for t in threads:t.start()
 env=os.environ.copy();env.update(PYTHONPATH=str(ROOT)+':'+str(OWNER),PYTHONDONTWRITEBYTECODE='1')
 log=(out/'graph.log').open('w')
 child=subprocess.Popen([str(DORA),'run','--stop-after',str(a.seconds)+'s',str(out/'graph.yml')],cwd=out,env=env,stdout=log,stderr=log,start_new_session=True)
 op=None
 try:
  if a.hold_current:
   from piper_minimal_live import Operator
   op=Operator(out);deadline=time.monotonic()+40
   while True:
    try:
     state=op.state()
     if state['robot_status'].get('status')=='READY_LIVE_TAKEOVER':break
    except (OSError,KeyError,ValueError):pass
    if child.poll() is not None or time.monotonic()>deadline:raise RuntimeError('owner not ready')
    time.sleep(.1)
   events=[]
   def event(phase):
    events.append({'phase':phase,'mono_ns':time.monotonic_ns(),'state':op.state()})
    (out/'phases.json').write_text(json.dumps(events,indent=2))
   event('enabled_idle');time.sleep(10)
   op.command('prepare');event('prepare_current_requested');deadline=time.monotonic()+20
   while op.state()['robot_status'].get('lifecycle_state')!='prepared':
    if time.monotonic()>deadline:raise RuntimeError('prepare-current timeout')
    time.sleep(.1)
   event('prepared_current');time.sleep(15)
   op.command('start');event('started_no_policy');time.sleep(15)
   op.command('stop');event('stopped');time.sleep(5)
  code=child.wait(timeout=a.seconds+30)
 finally:
  if op is not None and child.poll() is None:
   try:op.command('stop')
   except Exception as e:errors.append('stop: '+str(e))
  if child.poll() is None:
   os.killpg(child.pid,signal.SIGINT);child.wait(timeout=15)
  done.set()
  for t in threads:t.join(timeout=3)
  log.close()
 result={'graph_exit_code':child.returncode,'hardware_write_enabled':a.hold_current,'task_motion_source_present':False,'capture_http_startup_errors':errors}
 (out/'RESULT.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
