#!/usr/bin/env python3
"""One bounded native RAX session on Piper with operator-supplied task files; default is a plan, never hardware.

--run starts the existing write owner at its current pose, native Codex, and closes the session with independent Stop.
Left gripper task dispatch is enabled; initialization preserves both jaws.
--stop OUTPUT is an independent, session-bound Stop/disarm path.
"""
from __future__ import annotations
import argparse
import copy
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

from piper_observe_only import ROOT, OLD, OWNER, GRAPH, OWNER_PYTHON, DORA, HOST_PYTHON, POLICY_ADAPTER, make_graph, no_existing_devices, sensor_transport

from services.piper.installed_settling import state_matches
from site_paths import path as site_path, value as site_value

DEFAULT_TASK_CONFIG = ROOT/'examples/tasks/blue-cup/session.json'


def packet_budget(seconds):
    """Match the installed bounded FFS worker before starting any devices."""
    count=max(7200,(seconds+120)*3)
    if count>10000:
        raise ValueError('Session budget exceeds installed FFS 10000-packet contract; use --seconds <=3213 (e.g.3000)')
    return count


def archive_left_packets(out, sensor_root):
    """Preserve the bounded wrist IR window before legacy head archive removes tmpfs.

    Called only after all producers exited. A failed copy leaves the source intact.
    """
    import hashlib
    source=Path(sensor_root)/'left-packets';out=Path(out)
    if not source.exists():return {'files':0,'source_absent':True}
    dest=out/'left-packets';partial=out/'left-packets.partial'
    if dest.exists() or partial.exists():raise ValueError('Wrist archive destination already exists')
    if any(p.is_symlink() for p in source.rglob('*')):raise ValueError('Unexpected wrist packet symlink')
    size=sum(p.stat().st_size for p in source.rglob('*') if p.is_file())
    if shutil.disk_usage(out).free<size+64*1024**2:raise RuntimeError('Wrist archive disk reserve; preserve original')
    def manifest(base):
        hashes={}
        for p in base.rglob('*'):
            if p.is_file():
                with p.open('rb') as stream:hashes[str(p.relative_to(base))]=hashlib.file_digest(stream,'sha256').hexdigest()
        return hashes
    original=manifest(source);shutil.copytree(source,partial)
    if manifest(partial)!=original:raise ValueError('Wrist archive hash mismatch; preserve original')
    partial.rename(dest)
    record={'verified':True,'source_root':str(source),'files':original,'bytes':size}
    (out/'LEFT-SENSOR-ARCHIVE.json').write_text(json.dumps(record,indent=2))
    return {'files':len(original),'bytes':size,'verified':True}


def live_graph(source, out, control_root, sensor_root, token='PRIVATE_TOKEN_REQUIRED_AT_RUN', task='RAX task'):
    """Original owner/policy wire only; no old launcher, deployment, or task FSM."""
    from services.piper.installed_settling import JOINT_ARRIVAL_TOLERANCE_RAD, state_matches
    graph = make_graph(source, out, token)
    original = {n['id']: n for n in source['nodes']}
    common = graph['nodes'][0]['env'].copy()
    common.update(ENABLE_WRITE='1', TASK=task, PIPER_DEPLOYMENT='',
        RAX_MAX_JOINT_ARRIVAL_TOLERANCE_RAD=str(JOINT_ARRIVAL_TOLERANCE_RAD),
        RAX_JOINT_STATE_TOLERANCE_RAD=str(JOINT_ARRIVAL_TOLERANCE_RAD),
        PIPER_TASK_CATALOG_PATH=str(out/'tasks.json'), PIPER_POLICY_LABEL='RAX exploration',
        PIPER_BOUNDED_REVIEW_RELEASE='1', PIPER_CONTINUOUS_REVIEW='1', PIPER_GPT_TRAJECTORY_RELEASE='1',
        PIPER_PREPARE_HOLD_CURRENT='1', RAX_PREPARE_LEFT_HOME='0', RAX_LEFT_GRIPPER_DISPATCH='1',
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
        if node['id']=='piper_x_dual_policy':
            node['args']=f'{OWNER_PYTHON} {ROOT}/scripts/piper_policy.py'
        if node['id']=='piper_x_dual_owner':
            allowed={'tick','button_command','camera_status','model_action','model_status'}
            node['inputs']={k:v for k,v in original[node['id']]['inputs'].items() if k in allowed}
        elif node['id']=='piper_operator_ui':
            node['inputs']=copy.deepcopy(original[node['id']]['inputs'])
    return graph


# Reuse the already validated independent Stop/current-pose lifecycle from the tiny probe.
from piper_minimal_live import Operator, disarm, measured_rest, stop_session
from piper_handoff import checkpoint, validate_source, import_continuation, save_handoff, import_memory, copy_memory_evidence


def checked_current_pose(state, options):
    """Validate this measured pose only; historical targets never become an anchor."""
    import numpy as np
    from services.piper.geometry import Kinematics
    q = np.asarray(state['robot_state']['values'], dtype=float)
    if q.shape != (14,) or not np.isfinite(q).all(): raise ValueError('Invalid current measured14')
    kinematics = Kinematics(); flange = kinematics.fk(q[:6].tolist())[:3,3]
    kinematics.check(q[7:13].tolist(), 'right')
    if np.any(flange < options['workspace_min']) or np.any(flange > options['workspace_max']):
        raise ValueError('Current measured flange is outside the reviewed task workspace; no Prepare')
    if not 0 <= q[6] <= .075 or not 0 <= q[13] <= .075: raise ValueError('Current jaw feedback outside installed range')
    return q


def complete_ready_state(reader):
    """The first READY announcement precedes the periodic lease status."""
    state = reader.state()
    status = state['robot_status']
    if (status.get('status') != 'READY_LIVE_TAKEOVER'
            or not isinstance(status.get('rax_motion'), dict)):
        return None
    return state


def load_task(path=None):
    """Operator task data; no executable plugin or controller is loaded here."""
    path = Path(path or DEFAULT_TASK_CONFIG).resolve()
    task = json.loads(path.read_text())
    for key in ('description', 'backend_options', 'seed_memory'):
        task[key] = (path.parent/task[key]).resolve()
    task['prompts'] = {key: (path.parent/value).read_text() for key, value in task['prompts'].items()}
    return task


def task_prompt(start_prompt, continuation=False, return_to_zero_first=False, task=None):
    if return_to_zero_first:
        raise ValueError('Return to zero belongs to XPolicy, never the exploration model')
    prompts = (task or load_task())['prompts']
    return start_prompt + '\n' + prompts['continuation' if continuation else 'fresh'] + prompts['common']


def rehearsal_prompt(start_prompt, task=None):
    return start_prompt + (task or load_task())['prompts']['rehearsal']


def working_rehearsal_prompt(start_prompt, task=None):
    return (task or load_task())['prompts']['working_rehearsal'].lstrip() + '\n\nBase session contract (same reads, not a second startup):\n' + start_prompt


def validate_rehearsal_source(candidate, root):
    """Accept a writable copy with a verified immutable origin, or a legacy candidate."""
    import hashlib
    candidate=candidate.resolve(); root=root.resolve()
    def verify(base, manifest):
        for name, digest in manifest.items():
            path=(base/name).resolve()
            if not path.is_relative_to(base) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                raise ValueError('Frozen/candidate manifest changed: '+name)
    if candidate.is_relative_to(root/'workspaces'):
        metadata=json.loads((candidate/'WORKSPACE.json').read_text())
        origin=(root/metadata['source_version']).resolve()
        if not origin.is_relative_to(root/'versions') or metadata.get('writable') is not True:
            raise ValueError('Expected writable workspace with frozen version origin')
        manifest=origin/'SHA256SUMS.json'
        if hashlib.sha256(manifest.read_bytes()).hexdigest()!=metadata['source_manifest_sha256']:
            raise ValueError('Frozen manifest identity changed')
        verify(origin,json.loads(manifest.read_text()))
        if not any((candidate/'code').rglob('*.py')):
            raise ValueError('Working copy lacks distilled maneuvers')
        for directory in ('code','memory','calibration'):
            for path in (candidate/directory).rglob('*'):
                if path.is_symlink():raise ValueError('Editable source must not contain symlinks')
        return {'kind':'working_copy','origin':str(origin),'metadata':metadata}
    if not candidate.is_relative_to(root/'candidates') or not any((candidate/'code').rglob('*.py')):
        raise ValueError('Expected rehearsal source under candidates/ or workspaces/')
    verify(candidate,json.loads((candidate/'CANDIDATE_SHA256SUMS.json').read_text()))
    return {'kind':'legacy_candidate'}


def configure_motion_speed(graph, profile, out, speed_profile, *, write=False):
    """Clone only this physical session's owner profile; never change native Prepare."""
    if speed_profile not in ('standard','faster'):raise ValueError('Unknown workspace motion_speed_profile')
    if speed_profile=='standard':return
    selected=copy.deepcopy(profile);selected['live_takeover']['speed_percent']=50
    path=out/'owner-profile.yml'
    if write:
        import yaml
        path.write_text(yaml.safe_dump(selected,sort_keys=False))
    for node in graph['nodes']:node['env']['PIPER_PROFILE']=str(path)


def main():
    import yaml
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-config', type=Path, default=DEFAULT_TASK_CONFIG, help='Task description, prompts, seed memory and bounds as JSON paths')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--stop',type=Path)
    parser.add_argument('--rehearsal-candidate',type=Path,help='Load candidate or writable workspace functions; model starts only with --rehearsal-agent')
    parser.add_argument('--rehearsal-agent',action='store_true',help='With --rehearsal-candidate, start a NEW native RAX conversation to rehearse its functions')
    parser.add_argument('--continue-from',type=Path,help='Finished physical run whose memory/evidence is imported, never replayed')
    parser.add_argument('--return-to-zero-first',action='store_true',help='Retired: return-to-zero is exclusively an XPolicy operation')
    parser.add_argument('--seconds',type=int,default=1800)
    parser.add_argument('--max-model-calls',type=int,default=120)
    parser.add_argument('--http-proxy',default=site_value('vision_proxy',''),help='XQNetwork proxy for auxiliary vision; main model bypasses it with --direct-model')
    parser.add_argument('--direct-model',action='store_true',help='Connect main model directly to relay; preserve auxiliary vision proxy')
    parser.add_argument('--gemini-key-file',type=Path,default=site_path('gemini_key_file','~/.config/rax-human/gemini-key.py'))
    args=parser.parse_args();sys.path.insert(0,str(ROOT))
    if args.return_to_zero_first:parser.error('Return to zero via XPolicy; never via exploration model')
    if args.stop:
        if args.run or args.output or args.continue_from or args.rehearsal_candidate or args.rehearsal_agent:parser.error('--stop is an independent mode')
        out=args.stop.resolve()
        if not out.is_relative_to(ROOT/'commissioning'):parser.error('Expected this repository commissioning session')
        print(json.dumps(stop_session(out)));return
    if args.output is None:parser.error('--output is required')
    out=args.output.resolve()
    if not out.is_relative_to(ROOT/'commissioning') or out.exists():parser.error('Use a NEW commissioning directory')
    if not 60 <= args.seconds <= 604800 or args.max_model_calls < 1:parser.error('Invalid positive session budget')
    try: packets=packet_budget(args.seconds)
    except ValueError as exc:parser.error(str(exc))
    candidate=args.rehearsal_candidate.resolve() if args.rehearsal_candidate else None
    if args.rehearsal_agent and not candidate:parser.error('--rehearsal-agent requires --rehearsal-candidate')
    starts_model=not bool(candidate) or args.rehearsal_agent
    rehearsal_source=None
    if candidate:
        if args.continue_from:parser.error('Use independent rehearsal source, not continuation')
        rehearsal_source=validate_rehearsal_source(candidate,ROOT)
    working_copy=bool(rehearsal_source and rehearsal_source['kind']=='working_copy')
    speed_profile=(rehearsal_source['metadata'].get('motion_speed_profile','standard') if working_copy else 'standard')
    if speed_profile not in ('standard','faster'):raise ValueError('Unknown workspace motion_speed_profile')
    task = load_task(args.task_config)
    from agent_owned_workflow import install as install_agent_workflow
    task_label = task['title'] + (' function rehearsal' if candidate else ' exploration')
    previous=validate_source(args.continue_from,ROOT/'commissioning') if args.continue_from else None
    prior=checkpoint(previous) if previous else None
    memory_source=(candidate/'memory' if candidate else previous/'rax-session/agent-workspace/memory' if previous else task['seed_memory'])
    copy_memory_evidence(memory_source.parent,memory_source.parent,memory_dir=memory_source)
    task_options=json.loads(task['backend_options'].read_text())
    source=yaml.safe_load(GRAPH.read_text());profile=yaml.safe_load((OWNER/'real_world/robots/piper_x_dual/profile.yml').read_text())
    if profile['live_takeover']['gripper_write_enabled'] is not False:raise ValueError('Owner profile must disable all normal jaw writes')
    future_control=Path('/run/user')/str(os.getuid())/'SESSION_PRIVATE_CONTROL'
    future_sensors=Path('/run/user')/str(os.getuid())/'SESSION_PRIVATE_SENSORS'
    dora=[str(DORA),'run','--stop-after',f'{args.seconds+120}s',str(out/'graph.yml')]
    bridge=[str(HOST_PYTHON),str(ROOT/'scripts/piper_motion_bridge.py'),'--runtime',str(OWNER),
        '--output',str(out/'bridge'),'--journal',str(future_control/'receipts.jsonl'),
        '--token',str(out/'private/owner-token'),'--session-root',str(out/'session'),'--task',task_label,
        '--motion-profile','task60','--left-gripper','--speed-profile',speed_profile]
    opts={'address':'127.0.0.1:8091','motion_enabled':True,'depth_root':str(out/'depth'),
          'collision_legacy_root':str(OLD),'stream_enabled':True,'left_depth_root':str(out/'depth-left'),
          'bridge_root':str(out/'bridge'),'owner_session_root':str(out/'session'),'agent_image_width':640,
          'workspace_min':task_options['workspace_min'],'workspace_max':task_options['workspace_max']}
    plan={'main_model_transport':'direct' if args.direct_model else 'configured_proxy','mode':'LIVE_ONLY_IF_RUN','starts_devices':args.run,'output':str(out),'graph':live_graph(source,out,future_control,future_sensors,task=task_label),
          'motion_speed_profile':speed_profile,'sdk_speed_percent':50 if speed_profile=='faster' else profile['live_takeover']['speed_percent'],
          'auxiliary_vision':{'provider':'google','model':'gemini-3.8-flash','tool':'gemini_generate','key_file':str(args.gemini_key_file)},
          'rehearsal_candidate':str(candidate) if candidate else None,'rehearsal_agent':args.rehearsal_agent,'main_planning_model_started':starts_model,'continue_from':str(previous) if previous else None,'historical_checkpoint':prior,
          'agent_owned_workflow':{'enabled':starts_model, 'stage':'function_rehearsal' if candidate else 'exploration', 'closeout':'separate_software_only', 'closeout_task_budget':None},
          'evidence_disk':{'startup_free_gib':3,'runtime_stop_free_gib':1},
          'depth_pipeline':{'head':'continuous FFS','left':'on-demand FFS; own IR pair/calibration; optical coordinates only','right':'RGB only','wrist_hand_eye':False},
          'return_to_zero_first':args.return_to_zero_first,'session_seconds':args.seconds,'max_model_calls':args.max_model_calls,'backend_options':opts,'commands':{'bridge':bridge,'graph':dora,
          'independent_stop':[str(HOST_PYTHON),str(Path(__file__).resolve()),'--stop',str(out)]},
          'initialization_hardware_writes':['Enable both arms; SDK automatic mode switching disabled',
              'Prepare sends BOTH measured current joint targets and J/CAN mode at profile prepare speed 15%; no home target',
              'Start starts the existing local policy wire; idle output is continuous_review_wait',
              'Each trajectory target and Stop/final hold sends BOTH arm targets and J/CAN mode; right target preserved',
              'Only LEFT task gripper dispatch enabled; Stop/finally may hold measured current width; no jaw initialization, calibration or software-stop release'],
          'motion_scope':('candidate single-step rehearsal; native RAX agent' if args.rehearsal_agent else 'candidate single-step rehearsal without a planning model' if candidate else 'continue remaining task from fresh measured state; no new grasp/home/replay' if previous else 'native Codex direct task exploration; left arm and left gripper only'),
          'operator_clearance':'A run requires current operator authorization; task flange bounds are not obstacle clearance proof'}
    configure_motion_speed(plan['graph'],profile,out,speed_profile)
    if not args.run:print(json.dumps(plan,indent=2));return
    no_existing_devices()
    # Bounded FFS retention: 3 GiB startup includes a 1 GiB shutdown/archive reserve.
    if shutil.disk_usage(ROOT).free<3*1024**3:raise RuntimeError('Need 3 GiB free evidence disk space at startup')
    out.mkdir(parents=True,mode=0o700)
    os.environ['RAX_ARTIFACT_ROOT']=str(out)
    for name in ('private','tmp','cache'):(out/name).mkdir(mode=0o700)
    transport=sensor_transport();control=importlib.import_module('_rax_observe_existing_transport.volatile_control')
    control_root=control.create(out);sensor_root=transport.create(out)
    token=secrets.token_urlsafe(32);tokenfile=out/'private/owner-token';tokenfile.write_text(token);tokenfile.chmod(0o600)
    (out/'tasks.json').write_text(json.dumps({'canonical_prompts':[task_label]},indent=2))
    graph=live_graph(source,out,control_root,sensor_root,token,task=task_label)
    configure_motion_speed(graph,profile,out,speed_profile,write=True)
    from piper_observe_only import commands, CALIBRATION, FFS_PYTHON, MANIFEST
    _,worker=commands(out,args.seconds+120)
    transport.configure(sensor_root,worker,{})
    packet_limit=str(packets)
    worker[worker.index('--max-packets')+1]=packet_limit
    for node in graph['nodes']:
        e=node['env'];e['REALSENSE_STEREO_MAX_PACKETS']=packet_limit;e['REALSENSE_STEREO_EXPORT_DIR']=str(sensor_root/'packets');e['REALSENSE_STEREO_CALIBRATION']=str(CALIBRATION);e['REALSENSE_STEREO_ADAPTER_ROOT']=str(POLICY_ADAPTER)

    left_worker=[str(FFS_PYTHON),str(ROOT/'scripts/left_ffs_worker.py'),
        '--legacy-root',str(OLD),'--calibration',str(ROOT/'services/piper/assets/left-stereo-calibration.json'),
        '--manifest',str(MANIFEST),'--packet-root',str(sensor_root/'left-packets'),
        '--output',str(out/'depth-left'),'--seconds',str(args.seconds+120)]
    for node in graph['nodes']:
        node['env'].update(REALSENSE_LEFT_STEREO_EXPORT_DIR=str(sensor_root/'left-packets'),
            REALSENSE_LEFT_STEREO_CALIBRATION=str(ROOT/'services/piper/assets/left-stereo-calibration.json'))
    (out/'graph.yml').write_text(yaml.safe_dump(graph,sort_keys=False));(out/'graph.yml').chmod(0o600)
    bridge[bridge.index('--journal')+1]=str(control_root/'receipts.jsonl')
    plan['commands'].update(bridge=bridge);plan['runtime']={'control':str(control_root),'sensors':str(sensor_root)}
    plan['graph']=copy.deepcopy(graph)
    for node in plan['graph']['nodes']:node['env']['PIPER_OPERATOR_UI_TOKEN']='PRIVATE_TOKEN_NOT_EXPORTED'
    plan['commands']['ffs']=worker
    plan['commands']['left_ffs']=left_worker
    (out/'PLAN.json').write_text(json.dumps(plan,indent=2));(out/'backend-options.json').write_text(json.dumps(opts,indent=2))
    # The lease resolves into tmpfs; connector workspace checks use its parent.
    # Point at the same canonical options instead of maintaining a second copy.
    (control_root/'backend-options.json').symlink_to(out/'backend-options.json')
    disarm(out)
    from services import arming
    from services.piper.owner import OwnerReader
    from services.piper.cameras import Cameras
    import numpy as np
    env={k:v for k,v in os.environ.items() if not k.startswith(('PIPER_','REALSENSE_','ACTION_','POLICY_','ENABLE_WRITE','DAGGER_','MODEL_','RAX_','XPOLICY_','RECAP_'))}
    env.update(RAX_SITE_CONFIG=os.environ.get('RAX_SITE_CONFIG',str(ROOT/'site.local.json')), RAX_POLICY_ADAPTER_ROOT=str(POLICY_ADAPTER), PYTHONDONTWRITEBYTECODE='1',TMPDIR=str(out/'tmp'),XDG_CACHE_HOME=str(out/'cache'))
    children=[];operator=None;result={'passed':False,'jaw_dispatch_enabled':True,'events':[], 'continued_from':str(previous) if previous else None};started=time.monotonic()
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
        spawn(worker,'ffs',{**env,'PYTHONPATH':str(OLD)})
        spawn(left_worker,'left-ffs',{**env,'PYTHONPATH':str(OLD)})
        wait(lambda:(out/'depth/live-worker-ready.json').is_file(),120,'FFS ready')
        wait(lambda:(out/'depth-left/ready.json').is_file(),120,'Left FFS loaded')
        no_existing_devices()
        spawn(bridge,'bridge',{**env,'PYTHONPATH':str(OLD)})
        wait(lambda:(out/'bridge/state.json').is_file(),30,'bridge ready')
        spawn(dora,'graph',{**env,'PYTHONPATH':str(OWNER)})
        operator=Operator(out);reader=OwnerReader()
        initial=wait(lambda:complete_ready_state(reader),45,'write owner ready with lease status')
        q0=checked_current_pose(initial,opts);status=initial['robot_status']
        if status['rax_motion'].get('enabled') is not True or status['rax_motion'].get('left_gripper_dispatch') is not True:raise ValueError('Owner lease/jaw configuration mismatch')
        event('dual_enable_completed',state=initial)
        operator.command('prepare')
        wait(lambda:operator.state()['robot_status'].get('lifecycle_state')=='prepared',20,'Prepare current pose complete')
        if not state_matches(reader.state()['robot_state']['values'],q0):raise ValueError('Current-pose Prepare changed pose')
        event('prepare_current_pose_completed',state=reader.state())
        from services.controller.config import Configuration
        from harness.codex_cli.launcher import prepare
        config=Configuration(root=out/'rax-session',task=task['id'],task_description=task['description'].read_text(),skill_library_root=str(ROOT/'runtime/skill-library'),robot='piper',backend_options=opts,observation_profile='rgbd',agent_provider='codex-relay',agent_model='gpt-6-astra',agent_effort='high',agent_auto_compact_token_limit=task.get('agent_auto_compact_token_limit'),session_seconds=args.seconds,max_model_calls=args.max_model_calls)
        workspace=prepare(config,site_path('key_file','~/.config/rax-human/main-key.py'),site_value('relay_base_url','https://YOUR-RELAY.example/v1'),site_path('agent_runtime',ROOT/'.venv-agent'),args.http_proxy,gemini_key_file=args.gemini_key_file,model_http_proxy='' if args.direct_model else None)
        if previous:
            audit=ROOT/'verification/exploration-review/SUMMARY.json'
            import_continuation(previous,workspace,audit if audit.is_file() and json.loads(audit.read_text()).get('source_run')==str(previous) else None)
        else:
            import_memory(task['seed_memory'],workspace)
        initial_code=[str(p.relative_to(workspace)) for p in (workspace/'code').rglob('*') if p.is_file()]
        if initial_code:raise ValueError('Task code must start empty before candidate import')
        if candidate:
            shutil.copytree(candidate/'code',workspace/'code',dirs_exist_ok=True)
            import_memory(candidate/'memory',workspace)
            initial_code=[str(p.relative_to(workspace)) for p in (workspace/'code').rglob('*') if p.is_file()]
            if working_copy:
                import hashlib
                (out/'REHEARSAL-SOURCE.json').write_text(json.dumps(rehearsal_source,indent=2))
                imported={str(p.relative_to(workspace)):hashlib.sha256(p.read_bytes()).hexdigest()
                          for d in ('code','memory') for p in (workspace/d).rglob('*') if p.is_file()}
                (out/'CANDIDATE-SOURCE-SHA256SUMS.json').write_text(json.dumps(imported,indent=2))
                shutil.copytree(candidate/'calibration',workspace/'calibration',dirs_exist_ok=True)
                # Successful decisive frames are reference evidence, never current inputs.
                copy_memory_evidence(candidate,workspace)
                (workspace/'runtime/frames').chmod(0o755)
            else:
                shutil.copy2(candidate/'VERSION.json',out/'CANDIDATE-VERSION.json')
                shutil.copy2(candidate/'CANDIDATE_SHA256SUMS.json',out/'CANDIDATE-SOURCE-SHA256SUMS.json')
        (out/'task-code-at-start.json').write_text(json.dumps({'files':initial_code,'continued_from':str(previous) if previous else None,'fixed_task_controller_loaded':bool(candidate),'mode':'single_step_rehearsal' if candidate else 'exploration','observed_at_ns':time.time_ns()},indent=2))
        (workspace/'references').mkdir(exist_ok=True)
        if candidate:
            shutil.copy2(candidate/'HANDOFF.md',workspace/'references/rehearsal-handoff.md')
        if working_copy and task.get('compact_rehearsal_brief') is True:
            from rehearsal_brief import install as install_rehearsal_brief
            install_rehearsal_brief(workspace, candidate)
        history_lessons=task.get('history_lessons',str(ROOT/'docs/HISTORY_LESSONS.md'))
        if history_lessons:
            shutil.copy2(Path(history_lessons),workspace/'references/history-lessons.md')
        if not previous and not candidate:
            (workspace/'memory/PROGRESS.md').write_text('New RAX exploration. No task motion yet in this session. Read current status/images and conditional experience. Current pose is not inherited from historical trials. After first task success stop new task motion and save a concise checkpoint; the RAX main Agent continues memory/code/offline checks/git in a separate software-only closeout; do not start physical practice/scale testing.\n')
        if starts_model:
            install_agent_workflow(workspace, stage='function_rehearsal' if candidate else 'exploration',
                                   source=candidate)
        # New source images and feedback are saved before arming; no historical pose is trusted.
        cameras=Cameras(reader,workspace/'runtime/frames',opts['depth_root'])
        initial_frame,_=wait(lambda:cameras.capture(['head','left','right'],True),15,'fresh three views and head depth')
        current=reader.state();current_q=checked_current_pose(current,opts)
        if not state_matches(current_q,q0):raise ValueError('Pose changed before agent start; no arming')
        current_record={'measured_at_ns':time.time_ns(),'state':current,'frame_id':initial_frame['frame_id'],
                        'source':'new physical session; independent of historical handoff'}
        (workspace/'references/current-session-start.json').write_text(json.dumps(current_record,indent=2))
        event('current_session_observed',**current_record)
        (out/'operator-input.json').write_text(json.dumps(config.to_dict(),indent=2))
        for session in (out/'session',out/'rax-session'):arming.write(session,True,minutes=(args.seconds+120)/60,reason='user requested single-step rehearsal; left arm/jaw only' if candidate else 'user requested task continuation/exploration; left arm/jaw only')
        operator.command('start')
        wait(lambda:operator.state()['robot_status'].get('lifecycle_state')=='running',5,'Start')
        if candidate and not args.rehearsal_agent:
            from harness.codex_cli.launcher import private_socket, wait_frontend
            sock=private_socket(config.root,'.robot')
            agent_name='frontend'
            agent=spawn([str(HOST_PYTHON),'-m','harness.codex_cli.frontend','--config',str(config.root/'configuration.json'),'--socket',str(sock),'--started-at',str(time.time()),'--budget-started-monotonic',str(time.monotonic())],'frontend',{**env,'PYTHONPATH':str(ROOT),'RAX_ARTIFACT_ROOT':str(out)})
            wait_frontend(agent,sock,config.root)
            (out/'REHEARSAL-READY.json').write_text(json.dumps({'workspace':str(workspace),'socket':str(sock),'supervisor_pid':os.getpid(),'main_planning_model_started':False},indent=2))
            print(json.dumps({'rehearsal_ready':str(out),'main_planning_model_started':False}),flush=True)
        else:
            agent_name='agent'
            prompt=((working_rehearsal_prompt if working_copy else rehearsal_prompt)((workspace/'START_PROMPT.md').read_text(),task=task) if candidate else task_prompt((workspace/'START_PROMPT.md').read_text(),continuation=previous is not None,return_to_zero_first=args.return_to_zero_first,task=task))
            if starts_model:
                prompt = ('CURRENT AUTHORING OWNER: RAX main Agent. Read AGENT_OWNERSHIP.md once for stage, '
                          'experience/code/validation/version deliverables and separate software closeout. '
                          'Mac supervises and archives; it does not finish your controller. '
                          'Older stop-after-success text ends task motion, not this software closeout.\n\n' + prompt)
            (out/('rehearsal-prompt.txt' if candidate else 'exploration-prompt.txt')).write_text(prompt)
            agent=spawn([str(HOST_PYTHON),'-m','harness.codex_cli.launcher','--config',str(out/'operator-input.json'),'--exec-prompt',prompt],'agent',{**env,'PYTHONPATH':str(ROOT),'RAX_ARTIFACT_ROOT':str(out)})
        deadline=time.monotonic()+args.seconds+30
        while agent.poll() is None:
            if candidate and not args.rehearsal_agent and (out/'REHEARSAL-DONE.json').is_file():break
            if any(c.poll() is not None for name,c in children if name!=agent_name):raise RuntimeError('Device service exited during exploration')
            if time.monotonic()>deadline:raise TimeoutError('Exploration window exhausted')
            transport.check_capacity(sensor_root)
            if shutil.disk_usage(ROOT).free<1024**3:raise RuntimeError('Evidence disk below 1 GiB shutdown reserve')
            time.sleep(.5)
        result.update(agent_exit_code=agent.returncode,agent_session=str(config.root),task_success='See agent memory and saved images; process exit alone is not task success')
        result['stop']=stop_session(out)
        report_path=workspace/'output/rehearsal/SUMMARY.json' if args.rehearsal_agent else out/'REHEARSAL-DONE.json'
        result['passed']=(report_path.is_file() and json.loads(report_path.read_text()).get('ok') is True) if candidate else agent.returncode==0
        if candidate:result.update(mode='single_step_rehearsal',main_planning_model_started=starts_model,unattended_test=False,task_success=f'See {report_path} and individual function reports; not an unattended test')
    except BaseException as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        try:disarm(out)
        except Exception as exc:result.update(passed=False,early_disarm_error=str(exc))
        if operator is not None and not result.get('stop'):
            try:result['stop']=stop_session(out)
            except Exception as exc:result['stop_error']=str(exc)
        for name,child in children:
            if name in ('agent','frontend') and child.poll() is None:
                try:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=20)
                except (ProcessLookupError,subprocess.TimeoutExpired):pass
        try:disarm(out)
        except Exception as exc:result.update(passed=False,disarm_error=f'{type(exc).__name__}: {exc}')
        if operator is not None and not result.get('stop'):
            try:result['stop']=stop_session(out)
            except Exception as exc:result['stop_error']=f'{type(exc).__name__}: {exc}'
        def send_signal(child,sig):
            try:os.killpg(child.pid,sig)
            except ProcessLookupError:pass  # Normal exit between poll and signal.
        for name,child in sorted(children,key=lambda row:{'agent':0,'frontend':0,'graph':1,'bridge':2,'ffs':3,'left-ffs':3}[row[0]]):
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
                if name=='sensor':result['left_sensor_archive']=archive_left_packets(out,root)
                helper.archive(out,root);result[name+'_archived']=True
            except Exception as exc:result.update(passed=False,**{name+'_archive_error':f'{type(exc).__name__}: {exc}'})
        (out/'RESULT.json').write_text(json.dumps(result,indent=2))
        try:
            result['memory_evidence']=copy_memory_evidence(out/'rax-session/agent-workspace',out/'rax-session/agent-workspace')
            result['memory_evidence_valid']=True
        except Exception as exc:
            result['memory_evidence_valid']=False;result['memory_evidence_error']=f'{type(exc).__name__}: {exc}'
        try:save_handoff(out,result);result['handoff_saved']=True
        except Exception as exc:result.update(passed=False,handoff_error=f'{type(exc).__name__}: {exc}')
        (out/'RESULT.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    # The task budget has ended and all owner/sensor processes have exited.
    # Never consume the task's remaining time/calls to perform software closeout.
    if starts_model and (out/'rax-session/configuration.json').is_file():
        from harness.codex_cli.closeout import eligible, run as run_closeout
        allowed, why = eligible(result)
        if allowed:
            try:
                result['software_closeout'] = run_closeout(out/'rax-session', result)
            except Exception as exc:
                result['software_closeout'] = {'status':'failed', 'error':f'{type(exc).__name__}: {exc}', 'robot_connected':False}
        else:
            result['software_closeout'] = {'status':'not_started', 'reason':why, 'robot_connected':False}
        (out/'RESULT.json').write_text(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(2)


if __name__=='__main__':main()
