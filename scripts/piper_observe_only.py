#!/usr/bin/env python3
"""Plan/run a bounded observation-only graph with existing Piper device modules.

Default prints a reviewable plan. --run explicitly opens read-only robot feedback
and cameras. It never enables motors, sends Prepare/Start, starts an agent, or
constructs a motion bridge. New graph/logs/packets/depth stay in this repository.
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
import signal
import socket
import subprocess
import sys
import time
import types

ROOT = Path(__file__).resolve().parents[1]
from site_paths import path as site_path, value as site_value
OLD = site_path('legacy_root', ROOT/'integrations/legacy_piper')
OWNER = site_path('owner_root', ROOT/'integrations/xpolicy_owner')
HOST_PYTHON = site_path('host_python', sys.executable)
GRAPH = OWNER / 'runtime/robot_dagger/graphs/piper-x-dual-policy-30hz-keyboard.yml'
OWNER_PYTHON = site_path('owner_python', ROOT/'.venv-owner/bin/python')
DORA = site_path('dora', ROOT/'.venv-owner/bin/dora')
POLICY_ADAPTER = site_path('policy_adapter_root', ROOT/'integrations/policy_adapter')
FFS_PYTHON = site_path('ffs_python', ROOT/'.venv-ffs/bin/python')
CALIBRATION = site_path('head_calibration', ROOT/'services/piper/assets/head-stereo-calibration.json')
MANIFEST = site_path('ffs_manifest', ROOT/'runtime/models/fast-foundation-stereo/model-manifest.json')
WARMUP = site_path('ffs_warmup_packet', ROOT/'runtime/calibration/ffs-warmup')
OBSERVATION_TASK = 'Read current robot and three cameras; no hardware writes.'


def sensor_transport():
    # Load only the existing, tested tmpfs transport helpers, in an isolated
    # namespace so the new backend retains its own services.piper package.
    name = '_rax_observe_existing_transport'
    package = types.ModuleType(name)
    package.__path__ = [str(OLD / 'services/piper')]
    sys.modules[name] = package
    return importlib.import_module(name + '.volatile_sensors')


def record_write_status(result, status):
    count = status.get('hardware_write_count')
    if type(count) is int and count >= 0:
        result['hardware_writes'] = max(result['hardware_writes'] or 0, count)
    if status.get('hardware_write_enabled') is not False or count != 0 or type(count) is not int:
        raise PermissionError('Observed hardware write invariant failure')


def make_graph(source, out, token='GENERATED_AT_RUN_NOT_DISPLAYED'):
    # Reuse node implementations, not the old task/controller/workbench.
    ids = {'realsense_multi','piper_operator_ui','piper_x_dual_owner'}
    graph = {'nodes':[copy.deepcopy(n) for n in source['nodes'] if n['id'] in ids]}
    if {n['id'] for n in graph['nodes']} != ids: raise ValueError('Owner graph contract changed')
    common = {'PYTHONPATH':str(OWNER),'PYTHONDONTWRITEBYTECODE':'1','ENABLE_WRITE':'0',
              'PIPER_PROFILE':str(OWNER/'real_world/robots/piper_x_dual/profile.yml'),
              'PIPER_DEVICE_INSTANCE':str(OWNER/'runtime/robot_dagger/config/robot_instances/piper_x_dual_01.yml'),
              'REALSENSE_PROFILE':str(OWNER/'real_world/robots/piper_x_dual/profile.yml'),
              'REALSENSE_DEVICE_INSTANCE':str(OWNER/'runtime/robot_dagger/config/robot_instances/piper_x_dual_01.yml'),
              'REALSENSE_CAMERA_PROFILE':str(OWNER/'runtime/robot_dagger/config/camera_profiles/piper-x-dual-rgb-1280x720-30.yml'),
              'PIPER_CONTROL_SOURCE':'keyboard','PIPER_MODEL_FILTER_ENABLED':'0','ACTION_RTC_MODE':'disabled',
              'ENABLE_DAGGER_RECORDING':'0','RECAP_COLLECTION_MODE':'OFF',
              'TMPDIR':str(out/'tmp'),'XDG_CACHE_HOME':str(out/'cache'),
              'PIPER_OPERATOR_UI_HOST':'127.0.0.1','PIPER_OPERATOR_UI_PORT':'8091','PIPER_OPERATOR_UI_AUTO_OPEN':'0',
              'PIPER_OPERATOR_UI_CAMERA_MAX_WIDTH':'1280','PIPER_OPERATOR_UI_CAMERA_JPEG_QUALITY':'95',
              'PIPER_OPERATOR_UI_TOKEN':token,'PIPER_POLICY_LABEL':'RAX observation only — no action source',
              'TASK':OBSERVATION_TASK,'PIPER_TASK_CATALOG_PATH':str(out/'tasks.json'),
              'PIPER_OPERATOR_DEMO_ROOT':str(out/'ui-demo'),'PIPER_OPERATOR_VIDEO_ROOT':str(out/'ui-video'),
              'PIPER_TRAJECTORY_DIR':str(out/'trajectories'),
              'REALSENSE_STEREO_ADAPTER_ROOT':str(POLICY_ADAPTER),
              'REALSENSE_STEREO_CALIBRATION':str(CALIBRATION),'REALSENSE_STEREO_EXPORT_DIR':str(out/'packets'),
              'REALSENSE_STEREO_MAX_PACKETS':'600','REALSENSE_STEREO_INTERVAL_MS':'500',
              'REALSENSE_SDK_LOG_FILE':str(out/'realsense.log')}
    for node in graph['nodes']:
        module = node['args'].split(' -m ',1)[1]
        node['path']='/usr/bin/env';node['args']=f'{OWNER_PYTHON} -m {module}'
        node['env']=common.copy()
        if node['id']=='piper_x_dual_owner':
            # There is no Prepare/Start/transition/action route, even from the UI.
            node['inputs']={k:v for k,v in node['inputs'].items() if k in ('tick','camera_status')}
        elif node['id']=='piper_operator_ui':
            node['inputs']={k:v for k,v in node['inputs'].items() if k not in ('policy_status','model_info')}
    return graph


def commands(out,seconds):
    return ([str(DORA),'run','--stop-after',f'{seconds}s',str(out/'graph.yml')],
            [str(FFS_PYTHON),'-m','services.piper.ffs_worker','--calibration',str(CALIBRATION),
             '--manifest',str(MANIFEST),'--packet',str(WARMUP),'--packet-root',str(out/'packets'),
             '--output',str(out/'depth'),'--seconds',str(seconds+30),'--max-packets','600',
             '--retain-results','16','--retire-packets-after-s','20'])


def no_existing_devices():
    names=(b'.dora_owner_node',b'.realsense_dora_node',b'.operator_ui_node')
    for proc in Path('/proc').glob('[0-9]*/cmdline'):
        try:parts=proc.read_bytes().split(b'\0')
        except (OSError,PermissionError):continue
        if b'-m' in parts and any(p.endswith(names) for p in parts):
            raise RuntimeError('Existing device owner/camera/UI; not stopping or replacing it')
    for port in (8091,19027):
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1',port))==0:raise RuntimeError(f'Port {port} occupied; not replacing owner')


def main():
    import yaml
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=120)
    p.add_argument('--run',action='store_true')
    a=p.parse_args();out=a.output.resolve()
    if not out.is_relative_to(ROOT/'commissioning') or out.exists():p.error('Use a NEW directory under this repository commissioning/')
    if not 30<=a.seconds<=300:p.error('seconds must be30..300')
    source=yaml.safe_load(GRAPH.read_text());graph=make_graph(source,out);dora,worker=commands(out,a.seconds)
    inputs=[GRAPH,OWNER_PYTHON,DORA,CALIBRATION,MANIFEST,WARMUP/'packet.json',WARMUP/'images.npz',
            OLD/'services/piper/ffs_worker.py',OLD/'services/piper/volatile_sensors.py',OLD/'services/piper/volatile_control.py']
    for item in inputs:
        if not item.is_file():raise FileNotFoundError(item)
    plan={'mode':'OBSERVE_ONLY','new_output':str(out),'starts_devices':bool(a.run),'hardware_write_enabled':False,
          'prepare_start_routes':False,'model_agent_started':False,'motion_bridge_started':False,
          'sensor_transport':{'kind':'private_tmpfs','base':f'/run/user/{os.getuid()}',
                              'archive_after_all_children_stop':True,'original_timestamps_preserved':True},
          'dora_command':dora,'ffs_command':worker,'graph':graph,
          'sources':{str(item):hashlib.sha256(item.read_bytes()).hexdigest() for item in inputs},
          'backend_options':{'address':'127.0.0.1:8091','depth_root':str(out/'depth'),'motion_enabled':False,'agent_image_width':640}}
    if not a.run:print(json.dumps(plan,indent=2));return
    no_existing_devices()
    if __import__('shutil').disk_usage(ROOT).free<5*1024**3:raise RuntimeError('Insufficient free capture disk space')
    out.mkdir(parents=True,mode=0o700)
    for name in ('tmp','cache','private'):(out/name).mkdir(mode=0o700)
    (out/'tasks.json').write_text(json.dumps({'canonical_prompts':[OBSERVATION_TASK]},indent=2))
    token=secrets.token_urlsafe(32);tokenfile=out/'private/owner-token';tokenfile.write_text(token);tokenfile.chmod(0o600)
    transport=sensor_transport();sensor_root=transport.create(out)
    graph=make_graph(source,out,token)
    for node in graph['nodes']:transport.configure(sensor_root,worker,node['env'])
    (out/'graph.yml').write_text(yaml.safe_dump(graph,sort_keys=False));(out/'graph.yml').chmod(0o600)
    plan['sensor_transport']['root']=str(sensor_root);plan['ffs_command']=worker
    (out/'PLAN.json').write_text(json.dumps(plan,indent=2));(out/'backend-options.json').write_text(json.dumps(plan['backend_options'],indent=2))
    env={k:v for k,v in os.environ.items() if not k.startswith(('PIPER_','REALSENSE_','ACTION_','POLICY_','ENABLE_WRITE','DAGGER_','MODEL_','RAX_'))}
    env.update(PYTHONDONTWRITEBYTECODE='1',TMPDIR=str(out/'tmp'),XDG_CACHE_HOME=str(out/'cache'),ENABLE_WRITE='0')
    sys.path.insert(0,str(ROOT))
    from services.piper.owner import OwnerReader
    from services.piper.cameras import Cameras
    reader=OwnerReader();cameras=Cameras(reader,out/'session/stream/multiview',out/'depth')
    children=[];samples=[];snapshots=[];errors=[];result={'hardware_writes':None,'physical_motion_test':False}
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def spawn(command,name,child_env):
        with (out/(name+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=out,env=child_env,stdout=log,stderr=log,start_new_session=True)
        children.append((name,child));return child
    try:
        ffs=spawn(worker,'ffs',{**env,'PYTHONPATH':str(OLD)})
        deadline=time.monotonic()+120
        while not (out/'depth/live-worker-ready.json').is_file():
            if ffs.poll() is not None or time.monotonic()>deadline:raise RuntimeError('FFS startup failed; inspect ffs.log')
            time.sleep(.2)
        no_existing_devices()  # Recheck after model warmup, before opening cameras/CAN feedback.
        graph_process=spawn(dora,'graph',{**env,'PYTHONPATH':str(OWNER)})
        deadline=time.monotonic()+a.seconds+10
        while graph_process.poll() is None and time.monotonic()<deadline:
            try:
                state=reader.state();status=state['robot_status']
                record_write_status(result,status)
                samples.append({'at_ns':time.time_ns(),'state':state['robot_state'],'status':status})
                if len(samples)%10==0:transport.check_capacity(sensor_root)
                if len(snapshots)<3 and len(samples)>len(snapshots)*10:
                    meta,_=cameras.capture(['head','left','right'],True);snapshots.append(meta)
            except PermissionError:raise
            except (OSError,ValueError,KeyError) as exc:errors.append(str(exc))
            if ffs.poll() is not None:raise RuntimeError('FFS worker exited during observation')
            time.sleep(.5)
        result.update(passed=bool(samples) and len(snapshots)==3,owner_samples=len(samples),snapshot_ids=[s['frame_id'] for s in snapshots],errors=errors[-10:])
    except KeyboardInterrupt:result.update(passed=False,interrupted=True)
    except BaseException as exc:
        result.update(passed=False,error=f'{type(exc).__name__}: {exc}');raise
    finally:
        for name,child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT if name=='graph' else signal.SIGTERM)
            try:child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
        result['process_exit_codes']={name:child.returncode for name,child in children}
        try:
            transport.archive(out,sensor_root)
            result['sensor_archive_verified']=True
        except Exception as exc:
            result.update(passed=False,sensor_archive_verified=False,sensor_archive_error=f'{type(exc).__name__}: {exc}')
        (out/'state-samples.json').write_text(json.dumps(samples,indent=2));(out/'snapshots.json').write_text(json.dumps(snapshots,indent=2))
        (out/'RESULT.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    if not result.get('passed'):raise SystemExit(2)


if __name__=='__main__':main()
