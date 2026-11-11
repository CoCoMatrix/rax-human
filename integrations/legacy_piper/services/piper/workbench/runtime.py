"""Supervise one unchanged formal-owner launcher; connection does not move."""
import argparse
import signal
import subprocess
import sys
import time

from .common import ROOT, PYTHON, DATA, GUARD, read, write


def launcher_command(live, prepare_mode, runtime, deployment, task):
    if prepare_mode not in ('home', 'hold-current'):
        raise ValueError('Unknown preparation mode')
    cmd = [str(PYTHON), 'scripts/piper_motion.py', '--run', '--output', str(live),
           '--runtime', str(runtime), '--deployment', deployment, '--task', task,
           '--volatile-control', '--left-gripper', '--joint-tolerance-rad', '.003',
           '--motion-profile', 'task60']
    # The qualified launcher already supports current-pose preparation by default.
    # Never select the left-home/closed-jaw variant while resuming a held object.
    if prepare_mode == 'home': cmd.append('--left-home-prepare')
    return cmd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True)
    parser.add_argument('--prepare-mode', choices=('home','hold-current'), default='home')
    args = parser.parse_args()
    from pathlib import Path
    run = Path(args.run).resolve()
    if not run.is_relative_to(DATA / 'runs') or (run / 'live').exists():
        raise ValueError('必须使用工作台的新运行目录')
    sys.path.insert(0, str(ROOT / 'scripts'))
    from piper_ready import check_clock_guard, wait_ready
    from piper_prepare15_build import DEST, NAME, TASK
    from services.piper.calibration_web import Owner
    from services import arming
    from services.piper.qualification import load, sha
    load(ROOT / 'commissioning/task60-20261029/QUALIFICATION.json', sha(ROOT / 'commissioning/task60-20261029/QUALIFICATION.json'))
    child = owner = perception_child = None
    live = run / 'live'
    result = {'connected': False, 'prepare_sent': False, 'policy_started': False}
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        check_clock_guard(GUARD)
        cmd = launcher_command(live, args.prepare_mode, DEST, NAME, TASK)
        with (run / 'motion-launch.log').open('x') as log:
            child = subprocess.Popen(cmd, cwd=ROOT, stdout=log, stderr=log)
        initial = wait_ready(live, child)
        perception_workspace = live / 'session/agent-workspace'
        with (run / 'perception-worker.log').open('x') as log:
            perception_child = subprocess.Popen([
                '/home/ubuntu/code_junqi/perception/sam3/.venv/bin/python',
                str(Path(__file__).parent/'sam3_worker.py'), '--workspace', str(perception_workspace)],
                cwd=ROOT, stdout=log, stderr=log,
                env={**__import__('os').environ, 'PYTHONDONTWRITEBYTECODE':'1',
                     'XDG_CACHE_HOME':'/home/ubuntu/code_junqi/.cache',
                     'HF_HOME':'/home/ubuntu/code_junqi/.cache/huggingface',
                     'TORCH_HOME':'/home/ubuntu/code_junqi/.cache/torch'})
        perception_deadline = time.monotonic()+30
        while not (perception_workspace/'output/perception-queue/ready.json').is_file():
            if perception_child.poll() is not None or time.monotonic()>perception_deadline:
                raise RuntimeError('SAM3 broker startup failed; inspect perception-worker.log')
            check_clock_guard(GUARD)
            time.sleep(.1)
        owner = Owner((live / 'private/owner-token').read_text().strip())
        result['connected'] = True
        write(run / 'CONNECTED.json', {'at': time.time(), 'initial14': initial['robot_state']['values'],
                                      'no_prepare_or_start': True, 'prepare_mode': args.prepare_mode})
        while child.poll() is None:
            check_clock_guard(GUARD)
            time.sleep(1)
        raise RuntimeError('硬件进程退出；请查看 motion-launch.log')
    except KeyboardInterrupt:
        result['operator_stopped'] = True
    except BaseException as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        write(run / 'ERROR.json', result)
        raise
    finally:
        if perception_child and perception_child.poll() is None:
            perception_child.terminate()
            try: perception_child.wait(timeout=10)
            except subprocess.TimeoutExpired: perception_child.kill(); perception_child.wait(timeout=5)
        if (live / 'session').exists():
            arming.write(live / 'session', False, reason='workbench hardware supervisor stopped')
        if owner:
            try:
                owner.command('stop')
            except Exception as exc:
                result['stop_error'] = str(exc)
        if child and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=65)
            except subprocess.TimeoutExpired:
                result['cleanup_error'] = 'formal launcher exceeded stop deadline'
        result['stopped'] = (live / 'STOPPED.json').exists()
        write(run / 'RESULT.json', result)


if __name__ == '__main__':
    main()
