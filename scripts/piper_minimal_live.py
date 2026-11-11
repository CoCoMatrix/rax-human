#!/usr/bin/env python3
"""One supervised Piper interface test; default is a plan, never hardware.

--run starts the existing write owner: dual-arm Enable, Prepare at measured
current joints, Start, LEFT J1 +0.5 degrees, verified arrival, return, Stop.
Both jaw writers are disabled. This is not a cup task or an agent loop.
--stop OUTPUT is an independent, session-bound Stop/disarm path.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import importlib
import json
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
import urllib.request
from services.piper.installed_settling import state_matches

from piper_observe_only import ROOT, OLD, OWNER, GRAPH, OWNER_PYTHON, DORA, HOST_PYTHON, POLICY_ADAPTER, make_graph, no_existing_devices, sensor_transport

TASK = 'Piper interface check: left joint 1 positive half degree and return; jaws unchanged.'
CANDIDATE = ROOT / 'verification/piper-minimal-live-candidate.json'


def live_graph(source, out, control_root, sensor_root, token='PRIVATE_TOKEN_REQUIRED_AT_RUN'):
    """Original owner/policy wire only; no old launcher, deployment, or task FSM."""
    graph = make_graph(source, out, token)
    original = {n['id']: n for n in source['nodes']}
    common = graph['nodes'][0]['env'].copy()
    common.update(ENABLE_WRITE='1', TASK=TASK, PIPER_DEPLOYMENT='',
        PIPER_TASK_CATALOG_PATH=str(out/'tasks.json'), PIPER_POLICY_LABEL='RAX bounded interface check',
        PIPER_BOUNDED_REVIEW_RELEASE='1', PIPER_CONTINUOUS_REVIEW='1', PIPER_GPT_TRAJECTORY_RELEASE='1',
        PIPER_PREPARE_HOLD_CURRENT='1', RAX_PREPARE_LEFT_HOME='0', RAX_LEFT_GRIPPER_DISPATCH='0',
        PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M='', PIPER_RELEASE_SOFTWARE_STOP_ARM='', PIPER_CALIBRATE_CLOSED_GRIPPER='',
        PIPER_TRANSITION_POSE_LIBRARY_PATH='', PIPER_TRANSITION_POSE_LIBRARY_SHA256='',
        PIPER_REVIEW_RECEIPT_JOURNAL=str(control_root/'receipts.jsonl'), RAX_VOLATILE_CONTROL_ROOT=str(control_root),
        RAX_MOTION_LEASE=str(control_root/'bridge/lease.json'), RAX_ARMING_FILE=str(control_root/'operator/armed.json'),
        REALSENSE_STEREO_EXPORT_DIR='', REALSENSE_STEREO_CALIBRATION='', REALSENSE_STEREO_ADAPTER_ROOT='',
        PIPER_MODEL_FILTER_ENABLED='0',
        ACTION_RTC_MODE='disabled', ACTION_TRAJECTORY_TIME_SCALE='1', POLICY_PREFETCH_ENABLED='0',
        XPOLICY_ALLOW_NO_ACTION='0', POLICY_URL='ws://127.0.0.1:19027',
        PIPER_POLICY_ACTION_HORIZON='600', POLICY_EXECUTE_CHUNK_SIZE='600', POLICY_EXECUTE_CHUNK_MODE='auto',
        POLICY_ACTION_FREQUENCY_HZ='30', MODEL_RETRY_SECONDS='0.5', ROBOT_ID='piper_x_dual_01')
    for ident in ('piper_control_mux', 'piper_x_dual_policy'):
        graph['nodes'].append(copy.deepcopy(original[ident]))
    for node in graph['nodes']:
        module=original[node['id']]['args'].split(' -m ',1)[1]
        node['path']='/usr/bin/env';node['args']=f'{OWNER_PYTHON} -m {module}';node['env']=common.copy()
        if node['id']=='piper_x_dual_owner':
            allowed={'tick','button_command','camera_status','model_action','model_status'}
            node['inputs']={k:v for k,v in original[node['id']]['inputs'].items() if k in allowed}
        elif node['id']=='piper_operator_ui':
            node['inputs']=copy.deepcopy(original[node['id']]['inputs'])
    return graph


class Operator:
    """Independent session-bound HTTP; never owns CAN, SDK, or motion targets."""
    def __init__(self,out):
        self.token=(out/'private/owner-token').read_text().strip()
        self.identity=hashlib.sha256(self.token.encode()).hexdigest()
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self,*args,**kwargs):raise ValueError('Owner redirect rejected')
        self.http=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect())

    def request(self,path,payload=None):
        request=urllib.request.Request('http://127.0.0.1:8091'+path,
            data=None if payload is None else json.dumps(payload).encode(),
            headers={'Content-Type':'application/json','X-XPolicy-Operator-Token':self.token})
        with self.http.open(request,timeout=.5) as response:
            if response.headers.get('X-XPolicy-Operator-Session')!=self.identity:raise ValueError('Different owner session; no takeover')
            raw=response.read(1_000_001)
        if len(raw)>1_000_000:raise ValueError('Oversized owner response')
        return json.loads(raw)

    def state(self):return self.request('/api/state')

    def command(self,command):
        if command not in ('prepare','start','stop'):raise ValueError('Command not allowed')
        self.state()  # Verify this token/session before mutation.
        result=self.request('/api/command',{'command':command,'source':'keyboard_ui'})
        if result.get('ok') is not True or result.get('command')!=command:raise ValueError('Owner command not acknowledged')
        return result


def disarm(out):
    from services.franka import arming
    errors=[]
    for root in (out/'session',out/'rax-session'):
        try:arming.write(root,False,reason='minimal interface test stopped')
        except Exception as exc:errors.append(f'{root.name}: {type(exc).__name__}: {exc}')
    if errors:raise RuntimeError('; '.join(errors))


def measured_rest(reader,operator,timeout=4):
    import numpy as np
    deadline=time.monotonic()+timeout;anchor=None;first=None;generation=None;last=None;count=0
    while time.monotonic()<deadline:
        state=reader.state();q=np.asarray(state['robot_state']['values']);status=operator.state()['robot_status']
        stamp=state['robot_state']['feedback_metadata']['source_timestamp_ns']
        if status.get('lifecycle_state') not in ('idle','stopped'):
            time.sleep(.05);continue
        if anchor is None:anchor=q;first=stamp;generation=status['generation']
        if status['generation']!=generation or not state_matches(q,anchor,jaw_tolerance=.0003):raise ValueError('Owner changed or motion remains after Stop')
        if stamp!=last:count+=1;last=stamp
        if stamp-first>=350_000_000 and count>=4:
            return {'measured_rest':True,'samples':count,'measured14':q.tolist(),'source_span_s':(stamp-first)/1e9,'physical_emergency_stop_tested':False}
        time.sleep(.05)
    raise TimeoutError('Fresh measured rest after Stop not confirmed')


def stop_session(out):
    from services.piper.owner import OwnerReader
    result={'disarmed':False,'stop_acknowledged':None,'rest':None};errors=[]
    # Local lease withdrawal must work even if the UI is unavailable/replaced.
    try:disarm(out);result['disarmed']=True
    except Exception as exc:errors.append('disarm: '+str(exc))
    try:
        operator=Operator(out)
        result['stop_acknowledged']=operator.command('stop')  # Session check before HTTP mutation.
        result['rest']=measured_rest(OwnerReader(),operator)
    except Exception as exc:errors.append('owner Stop: '+str(exc))
    result['errors']=errors
    (out/'STOP-RESULT.json').write_text(json.dumps(result,indent=2))
    if errors:raise RuntimeError('; '.join(errors))
    return result


def main():
    import yaml
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--stop',type=Path)
    args=parser.parse_args();sys.path.insert(0,str(ROOT))
    if args.stop:
        if args.run or args.output:parser.error('--stop is an independent mode')
        out=args.stop.resolve()
        if not out.is_relative_to(ROOT/'commissioning'):parser.error('Expected this repository commissioning session')
        print(json.dumps(stop_session(out)));return
    if args.output is None:parser.error('--output is required')
    out=args.output.resolve()
    if not out.is_relative_to(ROOT/'commissioning') or out.exists():parser.error('Use a NEW commissioning directory')
    candidate=json.loads(CANDIDATE.read_text())
    source=yaml.safe_load(GRAPH.read_text());profile=yaml.safe_load((OWNER/'real_world/robots/piper_x_dual/profile.yml').read_text())
    if profile['live_takeover']['gripper_write_enabled'] is not False:raise ValueError('Owner profile must disable all normal jaw writes')
    if not candidate['collision_check']['passed']:raise ValueError('Offline candidate checks failed')
    future_control=Path('/run/user')/str(os.getuid())/'SESSION_PRIVATE_CONTROL'
    future_sensors=Path('/run/user')/str(os.getuid())/'SESSION_PRIVATE_SENSORS'
    dora=[str(DORA),'run','--stop-after','180s',str(out/'graph.yml')]
    bridge=[str(HOST_PYTHON),'-m','services.piper.motion_bridge','--runtime',str(OWNER),
        '--output',str(out/'bridge'),'--journal',str(future_control/'receipts.jsonl'),
        '--token',str(out/'private/owner-token'),'--session-root',str(out/'session'),'--task',TASK,
        '--motion-profile','commissioning','--max-joint-step','.02','--joint-tolerance-rad','.001']
    opts={'address':'127.0.0.1:8091','motion_enabled':True,
          'bridge_root':str(out/'bridge'),'owner_session_root':str(out/'session'),'agent_image_width':640,
          'workspace_min':candidate['candidate_workspace_min_m'],'workspace_max':candidate['candidate_workspace_max_m']}
    plan={'mode':'LIVE_ONLY_IF_RUN','starts_devices':args.run,'output':str(out),'graph':live_graph(source,out,future_control,future_sensors),
          'candidate':candidate,'backend_options':opts,'commands':{'bridge':bridge,'graph':dora,
          'independent_stop':[str(HOST_PYTHON),str(Path(__file__).resolve()),'--stop',str(out)]},
          'initialization_hardware_writes':['Enable both arms; SDK automatic mode switching disabled',
              'Prepare sends BOTH measured current joint targets and J/CAN mode at profile prepare speed 15%; no home target',
              'Start starts the existing local policy wire; idle output is continuous_review_wait',
              'Each trajectory target and Stop/final hold sends BOTH arm targets and J/CAN mode; right target preserved',
              'All jaw writers/initialization/calibration/maintenance/software-stop-release disabled'],
          'motion_scope':'one left J1 +0.5deg, measured arrival, return to q0; no task, no grasp, no model/FFS',
          'operator_clearance':'explicitly confirmed in the current parent conversation; this box is only this probe flange envelope'}
    if not args.run:print(json.dumps(plan,indent=2));return
    no_existing_devices()
    if shutil.disk_usage(ROOT).free<5*1024**3:raise RuntimeError('Insufficient free evidence disk space')
    out.mkdir(parents=True,mode=0o700)
    for name in ('private','tmp','cache'):(out/name).mkdir(mode=0o700)
    transport=sensor_transport();control=importlib.import_module('_rax_observe_existing_transport.volatile_control')
    control_root=control.create(out);sensor_root=transport.create(out)
    token=secrets.token_urlsafe(32);tokenfile=out/'private/owner-token';tokenfile.write_text(token);tokenfile.chmod(0o600)
    (out/'tasks.json').write_text(json.dumps({'canonical_prompts':[TASK]},indent=2))
    graph=live_graph(source,out,control_root,sensor_root,token)
    (out/'graph.yml').write_text(yaml.safe_dump(graph,sort_keys=False));(out/'graph.yml').chmod(0o600)
    bridge[bridge.index('--journal')+1]=str(control_root/'receipts.jsonl')
    plan['commands'].update(bridge=bridge);plan['runtime']={'control':str(control_root),'sensors':str(sensor_root)}
    plan['graph']=live_graph(source,out,control_root,sensor_root)
    (out/'PLAN.json').write_text(json.dumps(plan,indent=2));(out/'backend-options.json').write_text(json.dumps(opts,indent=2))
    disarm(out)
    from services.franka import arming
    from services.piper.backend import PiperBackend
    from services.piper.owner import OwnerReader
    from services.piper.cameras import Cameras
    import numpy as np
    env={k:v for k,v in os.environ.items() if not k.startswith(('PIPER_','REALSENSE_','ACTION_','POLICY_','ENABLE_WRITE','DAGGER_','MODEL_','RAX_','XPOLICY_','RECAP_'))}
    env.update(PYTHONDONTWRITEBYTECODE='1',TMPDIR=str(out/'tmp'),XDG_CACHE_HOME=str(out/'cache'))
    children=[];backend=None;operator=None;result={'passed':False,'jaw_dispatch_enabled':False,'events':[]};started=time.monotonic()
    def spawn(command,name,child_env):
        with (out/(name+'.log')).open('x') as log:c=subprocess.Popen(command,cwd=out,env=child_env,stdout=log,stderr=log,start_new_session=True)
        children.append((name,c));return c
    def wait(predicate,seconds,label):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if any(c.poll() is not None for _,c in children):raise RuntimeError('Child exited while waiting for '+label)
            try:
                value=predicate()
                if value:return value
            except (OSError,KeyError,ValueError):pass
            time.sleep(.05)
        raise TimeoutError(label)
    def event(name,**data):result['events'].append({'event':name,'at_ns':time.time_ns(),**data})
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    try:
        no_existing_devices()
        spawn(bridge,'bridge',{**env,'PYTHONPATH':str(OLD)})
        wait(lambda:(out/'bridge/state.json').is_file(),30,'bridge ready')
        spawn(dora,'graph',{**env,'PYTHONPATH':str(OWNER)})
        operator=Operator(out);reader=OwnerReader()
        initial=wait(lambda:(s if (s:=operator.state())['robot_status'].get('status')=='READY_LIVE_TAKEOVER' else None),45,'write owner ready')
        initial=reader.state();q0=np.asarray(initial['robot_state']['values']);status=initial['robot_status']
        if np.max(abs(q0-np.asarray(candidate['q0'])))>.0005:raise ValueError('Current joints differ from reviewed baseline; no Prepare or probe')
        if status['rax_motion'].get('enabled') is not True or status['rax_motion'].get('left_gripper_dispatch') is not False:raise ValueError('Owner lease/jaw configuration mismatch')
        event('dual_enable_completed',state=initial)
        operator.command('prepare')
        wait(lambda:operator.state()['robot_status'].get('lifecycle_state')=='prepared',20,'Prepare current pose complete')
        if np.max(abs(np.asarray(reader.state()['robot_state']['values'])-q0))>.0005:raise ValueError('Current-pose Prepare changed pose')
        event('prepare_current_pose_completed')
        for session in (out/'session',out/'rax-session'):arming.write(session,True,minutes=2,reason='operator authorized exact half-degree interface test')
        operator.command('start')
        wait(lambda:operator.state()['robot_status'].get('lifecycle_state')=='running',5,'Start')
        cameras=Cameras(reader,out/'session/stream/multiview')
        backend=PiperBackend(SimpleNamespace(root=out/'rax-session',backend_options=opts,observation_profile='rgb-only'),owner=reader,cameras=cameras)
        wait(lambda:((s:=backend.bridge.status()).get('latest_policy_observation') is not None and s['phase']=='IDLE'),5,'local policy observation')
        before,_=backend.call('piper_observe',{'require_depth':False});event('before_observation',frame_id=before['frame_id'],state=reader.state())
        fresh=reader.state();fresh_q=np.asarray(fresh['robot_state']['values'])
        if np.max(abs(fresh_q-q0))>.0003:raise ValueError('Pose drifted after Prepare/Start; no probe')
        q0=fresh_q.copy();event('fresh_probe_anchor',state=fresh)
        target=q0[:6].copy();target[0]+=np.deg2rad(.5)
        forward,_=backend.call('piper_move_joints',{'joints':target.tolist(),'duration_s':3.0});event('forward_arrival',result=forward)
        reached=np.asarray(reader.state()['robot_state']['values'])
        if np.max(abs(reached[:6]-target))>.001 or np.max(abs(reached[6:]-q0[6:]))>.001:raise ValueError('Measured forward endpoint/fixed axes mismatch; do not return automatically')
        after_forward,_=backend.call('piper_observe',{'require_depth':False});return_state=reader.state()
        event('forward_observation',frame_id=after_forward['frame_id'],state=return_state)
        return_anchor=np.asarray(return_state['robot_state']['values'])
        if np.max(abs(return_anchor[:6]-target))>.001 or np.max(abs(return_anchor[6:]-q0[6:]))>.001:
            raise ValueError('Fresh forward pose changed before return; Stop without automatic correction')
        returned,_=backend.call('piper_move_joints',{'joints':q0[:6].tolist(),'duration_s':3.0});event('return_arrival',result=returned)
        final=np.asarray(reader.state()['robot_state']['values'])
        if np.max(abs(final-q0))>.001:raise ValueError('Return endpoint mismatch')
        after,_=backend.call('piper_observe',{'require_depth':False});event('after_observation',frame_id=after['frame_id'],state=reader.state())
        # Exercise the same independent process entrypoint available to the operator.
        stopped=subprocess.run(plan['commands']['independent_stop'],cwd=out,env={**env,'PYTHONPATH':str(ROOT)},capture_output=True,text=True,timeout=8)
        if stopped.returncode:raise RuntimeError('Independent Stop failed: '+stopped.stderr[-1000:])
        result.update(passed=True,q0=q0.tolist(),q_forward=reached.tolist(),q_return=final.tolist(),stop=json.loads(stopped.stdout))
    except BaseException as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        try:disarm(out)
        except Exception as exc:result.update(passed=False,disarm_error=f'{type(exc).__name__}: {exc}')
        if operator is not None and not result.get('stop'):
            try:result['stop']=stop_session(out)
            except Exception as exc:result['stop_error']=f'{type(exc).__name__}: {exc}'
        if backend is not None:
            try:backend.close()
            except Exception as exc:result.update(passed=False,backend_close_error=f'{type(exc).__name__}: {exc}')
        def send_signal(child,sig):
            try:os.killpg(child.pid,sig)
            except ProcessLookupError:pass  # Normal exit between poll and signal.
        for name,child in sorted(children,key=lambda row:{'graph':0,'bridge':1}[row[0]]):
            try:
                if child.poll() is None:send_signal(child,signal.SIGINT if name=='graph' else signal.SIGTERM)
                try:child.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    result['passed']=False;result['shutdown_timeout']=name;send_signal(child,signal.SIGTERM)
                    try:child.wait(timeout=10)
                    except subprocess.TimeoutExpired:send_signal(child,signal.SIGKILL);child.wait(timeout=5)
            except Exception as exc:
                result['passed']=False;result.setdefault('shutdown_errors',{})[name]=f'{type(exc).__name__}: {exc}'
        result['process_exit_codes']={name:c.returncode for name,c in children};result['elapsed_s']=time.monotonic()-started
        for name,helper,root in (('control',control,control_root),('sensor',transport,sensor_root)):
            try:
                if any(c.poll() is None for _,c in children):raise RuntimeError('Producer still running; preserve tmpfs without archival')
                helper.archive(out,root);result[name+'_archived']=True
            except Exception as exc:result.update(passed=False,**{name+'_archive_error':f'{type(exc).__name__}: {exc}'})
        (out/'RESULT.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    if not result['passed']:raise SystemExit(2)


if __name__=='__main__':main()
