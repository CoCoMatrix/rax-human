"""Evidence-bound readiness for supervised, bounded exploration (not task success)."""
import hashlib
import json
from pathlib import Path


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path, expected_sha):
    path=Path(path)
    if sha(path)!=expected_sha:raise ValueError('qualification SHA mismatch')
    row=json.loads(path.read_text())
    if row.get('schema') not in ('rax.piper.exploration_qualification.v1','rax.piper.exploration_qualification.v2') or row.get('scope')!='supervised_bounded_exploration':
        raise ValueError('qualification schema/scope mismatch')
    if row.get('accepted') is not True:raise ValueError('qualification not accepted')
    if not row.get('sources'):raise ValueError('qualified source identities missing')
    for filename,digest in row['sources'].items():
        if sha(filename)!=digest:raise ValueError('qualified source changed: '+filename)
    evidence={}
    for kind,item in row['evidence'].items():
        if sha(item['path'])!=item['sha256']:raise ValueError('acceptance evidence changed: '+kind)
        evidence[kind]=json.loads(Path(item['path']).read_text())
        if evidence[kind].get('passed') is not True:raise ValueError('acceptance did not pass: '+kind)
    joint=evidence['joints']['results']
    if len(joint)!=6 or any(r.get('measured_arrival') is not True or r.get('sdk_completed_steps')!=120
        or r.get('joint_tolerance_rad')!=.003 or r['joint_error_max_rad']>.003 for r in joint):
        raise ValueError('joint round-trip evidence incomplete')
    stop=evidence['joint_stop']['results']
    if not any(r.get('cancellation',{}).get('measured_displacement_rad',0)>=.0005 for r in stop):
        raise ValueError('joint stop did not occur during measured motion')
    if not any(r.get('measured_rest') is True and r.get('samples',0)>=10 and r.get('max_variation_rad_m',1)<=.0003 for r in stop):
        raise ValueError('joint stop rest evidence missing')
    tools=evidence['tools']['results']
    if [r['tool'] for r in tools]!=['piper_gripper','piper_gripper','piper_move_eef','piper_move_eef','left_jaw_disarm']:
        raise ValueError('public motion tool checks incomplete')
    if any(r['result'].get('measured_arrival') is not True for r in tools[:4]):
        raise ValueError('motion tool arrival missing')
    if tools[-1]['cancellation']['jaw_displacement_m']<.0005 or tools[-1]['rest_variation_rad_m']>.0003:
        raise ValueError('jaw stop/rest evidence invalid')
    if row['schema']=='rax.piper.exploration_qualification.v2':
        validate_task60(row,evidence)
    elif row['limits']!={'max_joint_displacement_rad':.02,'joint_tolerance_rad':.003,'max_left_jaw_step_m':.01}:
        raise ValueError('unqualified exploration limits')
    return row


def validate_task60(row,evidence):
    import math
    from .motion_profiles import TASK60
    if row['limits']!=TASK60:raise ValueError('unqualified task60 limits')
    for case in ('canary','characterize','stop'):
        e=evidence['task60_'+case]
        if e.get('schema')!='rax.piper.task60_acceptance.v1' or e.get('case')!=case or e.get('limits')!=TASK60:
            raise ValueError('task60 evidence identity mismatch')
    checks=evidence['task60_characterize']['results']
    names={r.get('label') for r in checks}
    required={f'left J{j} 10 deg' for j in range(1,7)}|{f'left J{j} return' for j in range(1,7)}|{
        'flange lift 100 mm','flange lower 99 mm','left J6 near-60 deg','left J6 near-60 return','return to measured baseline'}
    if not required<=names:raise ValueError('task60 characterization incomplete')
    for r in evidence['task60_canary']['results']+checks:
        result=r.get('result',{})
        if result.get('measured_arrival') is not True or result.get('joint_error_max_rad',1)>.003:
            raise ValueError('task60 measured arrival missing')
        if r.get('measured_peak_joint_speed_rad_s_100ms',999)>math.radians(11):raise ValueError('task60 measured speed exceeded acceptance tolerance')
    if max(r.get('measured_peak_joint_speed_rad_s_100ms',0) for r in checks)<math.radians(8):
        raise ValueError('task60 requested speed not exercised')
    if any(r['result'].get('measured_fk_position_error_m',1)>.003 for r in checks if r['tool']=='piper_move_eef'):
        raise ValueError('task60 flange arrival inaccurate')
    stop=evidence['task60_stop']['results'][-1]
    if stop.get('measured_rest') is not True or stop.get('rest_samples',0)<20 or stop.get('rest_variation_rad_m',1)>.0003:
        raise ValueError('task60 measured rest missing')
    if stop.get('cancellation',{}).get('measured_displacement_rad',0)<math.radians(5) or stop.get('measured_post_cancel_travel_rad',1)>math.radians(2):
        raise ValueError('task60 stop not verified during substantial motion')
