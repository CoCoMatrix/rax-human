"""Bounded no-write supervisor using only the existing XPolicy operator API."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import re
import signal
import time
import urllib.request

from .backend import OwnerObserver, write_json
from .contracts import require


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--owner-url', default='http://127.0.0.1:18091')
    p.add_argument('--task', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--steps', type=int, default=64)
    args = p.parse_args()
    require(16 <= args.steps <= 128 and args.steps % 16 == 0, '16..128 steps in complete prefixes')
    args.output.mkdir(parents=True, exist_ok=False)
    owner = OwnerObserver(args.owner_url, args.task)
    history, commands = [], []
    report = {'schema': 'junqi.gap.piper.owner-shadow.v1', 'status': 'running',
              'started_ns': time.time_ns(), 'requested_control_steps': args.steps}
    token = None
    armed = False
    last_observation = None
    last_advanced = time.monotonic()

    def fetch(path):
        with urllib.request.urlopen(owner.url + path, timeout=2) as response:
            return response.read()

    def read():
        nonlocal last_observation, last_advanced
        value = owner.snapshot()
        history.append({'read_ns': time.time_ns(), 'state': value})
        robot = value['robot_status']
        if robot.get('observations') != last_observation:
            last_observation = robot.get('observations')
            last_advanced = time.monotonic()
        require(time.monotonic() - last_advanced <= 2, 'robot observations stopped advancing')
        require(all(value.get('cameras', {}).get(role, {}).get('available')
                    for role in ('head', 'left', 'right')), 'missing camera')
        measured = value.get('robot_state', {})
        require(measured.get('available') and measured.get('schema_id') == 'xpolicy.piper_x_dual.state.v1',
                'missing PiPER state')
        state = measured.get('values', [])
        require(len(state) == 14 and all(math.isfinite(v) for v in state), 'invalid measured state')
        return value

    def post(path, payload):
        request = urllib.request.Request(owner.url + path, data=json.dumps(payload).encode(),
            method='POST', headers={'Content-Type': 'application/json', 'X-XPolicy-Operator-Token': token})
        with urllib.request.urlopen(request, timeout=2) as response:
            result = json.load(response)
        require(result.get('ok') is True, 'formal operator command rejected')
        commands.append({'path': path, 'payload': payload, 'time_ns': time.time_ns(), 'result': result})

    def command(name):
        post('/api/command', {'command': name, 'source': 'keyboard_ui'})

    def wait(predicate, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            state = read()
            if predicate(state):
                return state
            require(state.get('policy_status', {}).get('status') != 'error',
                    f"policy error: {state.get('policy_status')}")
            time.sleep(.08)
        raise TimeoutError('formal owner did not reach expected state')

    def interrupted(signum, frame):
        raise RuntimeError(f'shadow monitor interrupted: {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        deadline = time.monotonic() + 45
        while True:
            try:
                initial = read()
                break
            except (OSError, ValueError, RuntimeError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.25)
        require(initial['robot_status'].get('lifecycle_state') == 'idle', 'owner not idle')
        require(initial['robot_status'].get('selected_action_source') == 'HOLD', 'owner not holding')
        report['initial'] = initial
        match = re.search(r'const token = ("[^\"]+");', fetch('/').decode())
        require(match is not None, 'missing formal operator token')
        token = json.loads(match.group(1))
        for role in ('head', 'left', 'right'):
            (args.output / f'initial-camera-{role}.jpg').write_bytes(fetch(f'/api/camera/{role}.jpg'))
        armed = True
        command('prepare')
        wait(lambda s: s['robot_status'].get('lifecycle_state') == 'prepared'
             and not s['robot_status'].get('boundary_event'), 15)
        post('/api/rollout', {'max_control_steps': args.steps, 'source': 'keyboard_ui'})
        wait(lambda s: s['robot_status'].get('rollout', {}).get('pending_max_control_steps') == args.steps, 3)
        read()  # Recheck both no-write gates immediately before logical start.
        command('start')
        final = wait(lambda s: s['robot_status'].get('lifecycle_state') == 'prepared'
            and not s['robot_status'].get('boundary_event')
            and s['robot_status'].get('rollout', {}).get('last', {}).get('model_steps') == args.steps
            and s['robot_status'].get('rollout', {}).get('last', {}).get('termination_reason') == 'pigey_rollout_limit'
            and s.get('policy_status', {}).get('status') == 'paused', 40)
        require(final['robot_status'].get('model_candidates', 0)
                - initial['robot_status'].get('model_candidates', 0) == args.steps,
                'owner model candidate count differs from bounded cap')
        require(final['robot_status']['hardware_write_count'] == 0, 'unexpected hardware dispatch')
        report.update(final=final, actual_model_candidates=args.steps, hardware_write_count=0,
                      task_success_evaluated=False, status='passed')
        command('stop')
        report['after_stop'] = wait(lambda s: s['robot_status'].get('lifecycle_state') == 'idle', 4)
        armed = False
    except Exception as exc:
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}')
    finally:
        if armed and token is not None:
            try:
                command('stop')
            except Exception as exc:
                report['stop_error'] = str(exc)
        report.update(commands=commands, completed_ns=time.time_ns(), snapshots=len(history))
        write_json(args.output / 'operator-snapshots.json', history)
        write_json(args.output / 'report.json', report)
        print(json.dumps({k: v for k, v in report.items() if k not in ('initial','final','after_stop','commands')}))
    if report['status'] != 'passed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
