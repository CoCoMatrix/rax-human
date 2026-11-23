"""Switch the supervised 8097 instance between trial and baseline, initially idle.

No Start/Prepare requests are sent. Only the matching managed launcher is stopped.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
OUT=Path('/home/ubuntu/code_junqi/migration/native-smooth-trial-20261010')
DEPLOYMENT='junqi-bat23-mixed2500-native-rtc5-instance01'


def switch(mode):
    state=json.load(urllib.request.urlopen('http://127.0.0.1:8097/api/state',timeout=5))
    if state['policy_status']['status'] != 'paused' or state['robot_status']['lifecycle_state'] not in ('prepared','idle','stopped'):
        raise RuntimeError('Finish/stop the current trial before switching configuration')
    launchers=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:args=proc.joinpath('cmdline').read_bytes().decode().split('\0')
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if str(ROOT/'scripts/launch_robot.sh') in args and DEPLOYMENT in args and '8097' in args:
            launchers.append(int(proc.name))
    if len(launchers)!=1:raise RuntimeError(f'Expected exactly one matching launcher, found {launchers}')
    stamp=time.strftime('%Y%m%d-%H%M%S'); OUT.mkdir(exist_ok=True)
    (OUT/f'{stamp}-{mode}-before.json').write_text(json.dumps(state,indent=2)+'\n')
    pid=launchers[0];os.kill(pid,signal.SIGTERM)
    for _ in range(500):
        try:
            status=Path(f'/proc/{pid}/status').read_text()
            if '\nState:\tZ' in status:break
        except FileNotFoundError:break
        time.sleep(.1)
    else:raise RuntimeError('Old launcher did not exit; no replacement started')
    wrapper = {'trial': 'junqi_bat23_native_smooth_trial.sh',
               'baseline': 'junqi_bat23_native_smooth_baseline.sh',
               'velocity': 'junqi_bat23_velocity_seam_trial.sh',
               'one-euro': 'junqi_bat23_one_euro_trial.sh',
               'joint-clip': 'junqi_bat23_joint_clip_trial.sh'}[mode]
    args=['bash',str(ROOT/'scripts'/wrapper),'--model-server','managed','--model-port','19028',
          '--no-pigey','--control-source','keyboard','--operator-ui-port','8097',
          '--no-model-filter','--enable-write','--no-open-ui']
    log=OUT/f'{stamp}-{mode}-live.log'
    env=dict(os.environ)
    env.pop('PIPER_NATIVE_TRAJECTORY_PROFILE',None)
    env.pop('PIPER_NATIVE_VELOCITY_SEAM_PROFILE',None)
    env.pop('PIPER_NATIVE_ONE_EURO_PROFILE',None)
    env.pop('PIPER_NATIVE_JOINT_CLIP_PROFILE',None)
    with log.open('xb') as handle:
        child=subprocess.Popen(args,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,
                               stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
    record={'pid':child.pid,'mode':mode,'args':args,'log':str(log),'auto_start':False}
    (OUT/'active-launch.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['trial','baseline','velocity','one-euro','joint-clip'])
    switch(parser.parse_args().mode)
