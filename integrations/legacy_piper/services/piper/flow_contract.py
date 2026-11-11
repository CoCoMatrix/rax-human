"""Site preflight, not a new agent loop or physical acceptance authority."""
import json
import subprocess
import shutil
from pathlib import Path

PACKAGES=('numpy','scipy','cv2','PIL','matplotlib','pandas','sklearn','skimage','sympy','shapely','trimesh','open3d','pinocchio','yourdfpy','transforms3d')
REQUIRED=('status','request_operator','piper_status','piper_observe','piper_pixel_to_position','piper_preview_path','piper_preview_gripper','piper_hold')

def python_packages(executable):
    # No model, no imports of SDKs, no credential reads.
    code='import importlib.util,json;print(json.dumps({k:importlib.util.find_spec(k) is not None for k in '+repr(PACKAGES)+ '}))'
    p=subprocess.run([str(executable),'-c',code],text=True,capture_output=True,timeout=15,check=True)
    return json.loads(p.stdout)

def assess(phase,status,tools):
    if phase not in ('observe','probe','explore','task'):raise ValueError('phase must be observe, probe, explore or task')
    names={x['name'] for x in tools};missing=sorted(set(REQUIRED)-names);blockers=[]
    if missing:blockers.append('missing MCP tools: '+', '.join(missing))
    if status.get('closed') is True:blockers.append('backend is closed')
    if status.get('owner_error') or not status.get('measured_state'):blockers.append('fresh measured owner unavailable')
    if status.get('depth',{}).get('ready') is not True:blockers.append('fresh exact-bound FFS unavailable')
    if phase=='probe' and status.get('owner_hardware_write_enabled') is not False:
        blockers.append('planning stage requires no-write owner')
    if phase=='observe' and status.get('capabilities',{}).get('general_physical_dispatch') is True:
        blockers.append('observation commissioning requires a backend without physical dispatch')
    if phase=='probe':
        if not {'piper_request_joint_probe','piper_review_status'}<=names or status.get('operator_review_available') is not True:
            blockers.append('review-only probe bridge unavailable')
    if phase=='explore':
        candidate=status.get('camera_calibration_candidate',{})
        if candidate.get('loaded') is not True or candidate.get('provisional_use') is not True:
            blockers.append('pinned operator-selected calibration unavailable')
        if status.get('motion_qualification',{}).get('ready') is not True:
            blockers.append('evidence-bound supervised exploration qualification missing')
        if status.get('capabilities',{}).get('general_physical_dispatch') is not True:
            blockers.append('qualified bounded dispatch unavailable')
        if status.get('motion_bridge',{}).get('phase') not in ('IDLE','DONE'):
            blockers.append('motion bridge is not ready')
        if status.get('recording',{}).get('enabled') is not True:blockers.append('continuous session recording disabled')
        if 'skill_library_merge' not in names:blockers.append('official skill library merge unavailable')
    if phase=='task':
        for k in ('physical_calibration_validated','collision_acceptance','motion_transport_present',
                  'controller_characterization_accepted','physical_stop_accepted'):
            if status.get(k) is not True:blockers.append(k+' not accepted')
        capabilities=status.get('capabilities',{})
        if capabilities.get('general_physical_dispatch') is not True:
            blockers.append('general physical dispatch capability unavailable')
        if 'skill_library_merge' not in names:blockers.append('official skill library merge unavailable')
        if status.get('recording',{}).get('enabled') is not True:
            blockers.append('continuous session recording disabled')
        if status.get('mode')=='NO_SEND':blockers.append('physical task unavailable in NO_SEND build')
    return dict(phase=phase,ready=not blockers,blockers=blockers,missing_tools=missing,
                motion_permit=False,automatic_launch=False,
                scope='Readiness diagnosis only; never an arming or execution permit')

def preflight(config,ctx,phase):
    workspace=Path(config.root)/'agent-workspace'
    packages=python_packages(config.agent_python)
    result=assess(phase,ctx.value('piper_status'),ctx.request('tools/list',{})['tools'])
    result['packages']=packages
    result['free_bytes']=shutil.disk_usage(config.root).free
    result['minimum_free_bytes']=3*1024**3
    if result['free_bytes']<result['minimum_free_bytes']:result['blockers'].append('less than 3 GiB startup disk reserve')
    for module in ('numpy','scipy','cv2','PIL'):
        if not packages.get(module):result['blockers'].append('agent dependency missing: '+module)
    task=workspace/'TASK.md'
    description=config.task_description.strip()
    expected=f'# Task: {config.task}\n\n{description}'
    if not description or not task.is_file() or task.read_text().strip() not in (expected,description):result['blockers'].append('TASK.md does not match operator configuration')
    if not (workspace/'skill-library/INDEX.md').is_file() or not (workspace/'skill-library/.git').is_dir():result['blockers'].append('official skill-library checkout missing')
    stream=workspace/'runtime/stream'
    if not stream.is_symlink() or stream.resolve()!=(Path(config.root)/'stream').resolve():
        result['blockers'].append('official recording stream link missing or wrong session')
    if not (workspace/'memory').is_dir():result['blockers'].append('official memory directory missing')
    result['ready']=not result['blockers']
    return result
