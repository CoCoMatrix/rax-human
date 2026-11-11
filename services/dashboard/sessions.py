"""Operator-side session and robot access for the dashboard (trusted, never agent-facing).

Everything here reads session artifacts directly or runs the same operator scripts a human
would: scripts/configure_session.py, scripts/start_agent.sh and services.arming.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time

from services import arming
from services.controller import operator
from services.storage_root import PROJECT, artifact_root

SESSION_ID = re.compile(r'[A-Za-z0-9_-]+')
FRAME_ID = re.compile(r'[A-Za-z0-9]+')
CAMERA = re.compile(r'[A-Za-z0-9_-]+')
ACTIVE = ('preparing', 'running', 'usage_wait')
WORKSPACE_NAME = 'agent-workspace'
HOME_NAME = 'claude-home'
TAIL_BYTES = 256 * 1024
STOP_GRACE_SECONDS = 45  # the launcher stops the agent, then the robot service (30 s grace)
HOLD = 'disarm written; backend stops on its next control tick; physical stop not confirmed'


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def tail_lines(path, limit, max_bytes=TAIL_BYTES):
    """Last `limit` complete lines of a text file, reading at most `max_bytes`."""
    try:
        with open(path, 'rb') as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            stream.seek(max(0, size - max_bytes))
            data = stream.read()
    except OSError:
        return []
    lines = data.decode('utf-8', errors='replace').splitlines()
    if size > max_bytes and lines:
        lines = lines[1:]  # the first line is probably cut
    return lines[-limit:]


def pid_alive(pid):
    if not isinstance(pid, int) or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:
        status = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
        return status != 'Z'
    except (OSError, IndexError):
        return True


def is_launcher(pid):
    try:
        command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    except OSError:
        return False
    return any(name in command for name in ('harness.claude_cli.launcher', 'harness.codex_cli.launcher', 'start_agent.sh', 'start_codex.sh'))


def truncate(value, limit):
    text = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False)
    return text if len(text) <= limit else text[:limit] + f'... [{len(text) - limit} more chars]'


class Sessions:
    def __init__(self, *, project=PROJECT, python=None, launcher=None, robot=None, telemetry=None):
        self.project = Path(project)
        self.python = python or sys.executable
        self.launcher = launcher
        self.robot_kind = robot or os.environ.get('RAX_ROBOT', 'mock')
        from services.backends import BACKENDS
        if self.robot_kind not in BACKENDS:
            raise ValueError('Unknown dashboard robot')
        self.telemetry = telemetry
        self.lock = threading.Lock()

    # ----- discovery -------------------------------------------------------------------
    @property
    def base(self):
        return artifact_root()/'sessions'

    def root(self, session_id):
        if not isinstance(session_id, str) or not SESSION_ID.fullmatch(session_id):
            raise ValueError('invalid session id')
        root = self.base/session_id
        if not (root/'operator-input.json').is_file():
            raise KeyError(f'unknown session {session_id}')
        return root

    def roots(self):
        if not self.base.is_dir():
            return []
        return sorted((p for p in self.base.iterdir() if (p/'operator-input.json').is_file()),
                      key=lambda p: p.name, reverse=True)

    def process(self, root):
        """The live launcher of a session: {'pid', 'pgid'|None, 'source'} or None."""
        launched = read_json(root/'dashboard-launch.json', {})
        if pid_alive(launched.get('pid')) and is_launcher(launched['pid']):
            return {'pid': launched['pid'], 'pgid': launched.get('pgid'), 'source': 'dashboard'}
        lifecycle = read_json(root/'agent-lifecycle.json', {})
        pid = lifecycle.get('pid')
        if lifecycle.get('status') in ACTIVE and pid_alive(pid) and is_launcher(pid):
            return {'pid': pid, 'pgid': None, 'source': 'terminal'}
        return None

    def running(self):
        return [root.name for root in self.roots() if self.process(root)]

    @staticmethod
    def usage(root):
        totals = {'calls': (read_json(root/'model-calls.json', {}) or {}).get('calls', 0),
                  'input_tokens': 0, 'output_tokens': 0, 'cache_read_input_tokens': 0,
                  'cache_creation_input_tokens': 0, 'cost_usd': 0.0, 'responses': 0}
        try:
            with open(root/'model-usage.jsonl') as stream:
                for line in stream:
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue
                    totals['responses'] += 1
                    for key in ('input_tokens', 'output_tokens', 'cache_read_input_tokens',
                                'cache_creation_input_tokens'):
                        totals[key] += int(row.get(key, 0) or 0)
                    totals['cost_usd'] += float(row.get('cost_usd', 0) or 0)
        except OSError:
            pass
        totals['cost_usd'] = round(totals['cost_usd'], 4)
        return totals

    def summary(self, root):
        config = read_json(root/'operator-input.json', {}) or {}
        lifecycle = read_json(root/'agent-lifecycle.json', {}) or {}
        process = self.process(root)
        status = lifecycle.get('status', 'configured')
        if status in ACTIVE and not process:
            status = f'{status} (launcher gone)'
        return {
            'id': root.name, 'task': config.get('task'), 'robot': config.get('robot'),
            'created': (root/'operator-input.json').stat().st_mtime,
            'status': status, 'returncode': lifecycle.get('returncode'),
            'started_at': lifecycle.get('started_at'), 'finished_at': lifecycle.get('finished_at'),
            'attempts': len(lifecycle.get('attempts', [])) + (1 if lifecycle else 0),
            'running': bool(process), 'process': process,
            'motion': arming.read(root),
            'requests': operator.pending(root),
            'model': config.get('agent_model') or 'provider default (Opus 5.5)',
            'provider': config.get('agent_provider'), 'effort': config.get('agent_effort'),
            'session_hours': round(config.get('session_seconds', 0) / 3600, 2),
            'observation_profile': config.get('observation_profile'),
            'usage': self.usage(root),
            'config_path': str(root/'operator-input.json'),
            'commands': self.commands(root),
        }

    def list(self):
        return [self.summary(root) for root in self.roots()]

    def launcher_command(self, root):
        config = read_json(root/'operator-input.json', {})
        script = 'start_codex.sh' if config.get('agent_provider') == 'codex-relay' else 'start_agent.sh'
        return self.launcher or ['bash', str(self.project/'scripts'/script)]

    def commands(self, root):
        config = root/'operator-input.json'
        script = self.launcher_command(root)[-1]
        here = f'cd {self.project} && '
        return {'interactive': f'{here}bash {script} --config {config}',
                'headless': f'{here}bash {script} --config {config} --non-interactive',
                'resume': f'{here}bash {script} --config {config} --resume',
                'arm': f'{self.python} {self.project/"scripts/arm_robot.py"} {root} arm'}

    # ----- configure / launch / stop ---------------------------------------------------
    def configure(self, form):
        task = str(form.get('task', '')).strip()
        if not SESSION_ID.fullmatch(task):
            raise ValueError('task id: letters, digits, _ and - only')
        command = [self.python, str(self.project/'scripts/configure_session.py'), '--task', task,
                   '--task-text', str(form.get('task_text', '')), '--robot', self.robot_kind,
                   '--agent-effort', str(form.get('effort') or 'max'),
                   '--agent-provider', str(form.get('provider') or 'claude-login'),
                   '--session-hours', str(float(form.get('session_hours') or 4)),
                   '--observation-profile', str(form.get('observation_profile') or 'rgbd'),
                   '--subagents', str(int(form.get('subagents') or 0))]
        if form.get('model'):
            command += ['--agent-model', str(form['model'])]
        result = subprocess.run(command, cwd=self.project, capture_output=True, text=True, timeout=60)
        if result.returncode != 0:
            raise ValueError((result.stderr or result.stdout).strip().splitlines()[-1] if
                             (result.stderr or result.stdout).strip() else 'configure_session.py failed')
        path = Path(result.stdout.strip().splitlines()[-1])
        root = path.parent
        return {'id': root.name, 'config_path': str(path), 'commands': self.commands(root)}

    def launch(self, session_id, *, resume=False):
        with self.lock:
            root = self.root(session_id)
            running = self.running()
            if running:
                raise RuntimeError(f'session {running[0]} is running; one robot, one session at a time')
            lifecycle = read_json(root/'agent-lifecycle.json')
            if lifecycle and not resume:
                raise RuntimeError('this session was already launched; use Resume')
            if resume and not lifecycle:
                raise RuntimeError('nothing to resume: this session was never launched')
            command = [*self.launcher_command(root), '--config', str(root/'operator-input.json'), '--non-interactive']
            if resume:
                command.append('--resume')
            with open(root/'launcher.log', 'ab') as log:
                log.write(f'\n=== dashboard {"resume" if resume else "launch"} {time.ctime()} ===\n'.encode())
                log.flush()
                process = subprocess.Popen(command, cwd=self.project, stdin=subprocess.DEVNULL, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
            record = {'pid': process.pid, 'pgid': process.pid, 'started_at': time.time(), 'resume': resume,
                      'command': command}
            (root/'dashboard-launch.json').write_text(json.dumps(record))
            threading.Thread(target=process.wait, daemon=True).start()  # reap it
            return record

    def stop(self, session_id, *, grace=STOP_GRACE_SECONDS):
        root = self.root(session_id)
        process = self.process(root)
        if not process:
            raise RuntimeError('the session is not running')

        def signal_it(sig):
            try:
                if process['pgid']:
                    os.killpg(process['pgid'], sig)
                else:
                    os.kill(process['pid'], sig)
            except ProcessLookupError:
                pass
        signal_it(signal.SIGTERM)  # the launcher stops the agent and robot service itself

        def escalate():
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                if not pid_alive(process['pid']):
                    return
                time.sleep(0.5)
            signal_it(signal.SIGKILL)
        threading.Thread(target=escalate, daemon=True).start()
        return {'signalled': process}

    # ----- arming ------------------------------------------------------------------------
    def arm(self, session_id, *, confirmed, minutes=None):
        if confirmed is not True:
            raise PermissionError('confirm that a person is at the emergency stop and the workspace is clear')
        root = self.root(session_id)
        if minutes is not None:
            minutes = float(minutes)
            if not 0 < minutes <= 24 * 60:
                raise ValueError('minutes must be in (0, 1440]')
        arming.write(root, True, minutes=minutes, by='dashboard')
        return arming.read(root)

    def reply(self, session_id, request_id, text, *, done=True):
        return operator.reply(self.root(session_id), request_id, text, done=done, by='dashboard')

    def disarm(self, session_id):
        root = self.root(session_id)
        arming.write(root, False, by='dashboard', reason='disarmed from the dashboard')
        return {'motion': arming.read(root), 'hold': HOLD}

    def disarm_all(self):
        disarmed = []
        for root in self.roots():
            if arming.arm_file(root).exists() or self.process(root):
                arming.write(root, False, by='dashboard', reason='DISARM ALL from the dashboard')
                disarmed.append(root.name)
        return {'disarmed': disarmed, 'hold': HOLD}

    # ----- robot ---------------------------------------------------------------------------
    def robot(self):
        try:
            if self.telemetry is None:
                from services.backends import make_telemetry
                self.telemetry = make_telemetry(self.robot_kind)
            if self.telemetry is None:
                return {'kind': self.robot_kind, 'ok': False, 'error': 'No read-only telemetry registered'}
            return self.telemetry.snapshot()
        except Exception as exc:
            return {'kind': self.robot_kind, 'ok': False, 'error': str(exc)[:300]}

    # ----- frames and activity ---------------------------------------------------------
    def frames_dir(self, root):
        return root/WORKSPACE_NAME/'runtime'/'frames'

    def latest_frame(self, session_id):
        root = self.root(session_id)
        directory = self.frames_dir(root)
        if not directory.is_dir():
            return None
        frames = sorted((p for p in directory.iterdir() if p.is_dir() and FRAME_ID.fullmatch(p.name)),
                        key=lambda p: p.name)
        for frame in reversed(frames):
            cameras = sorted(p.stem for p in frame.glob('*.jpg') if CAMERA.fullmatch(p.stem))
            if cameras:
                meta = read_json(frame/'meta.json', {}) or {}
                return {'frame_id': frame.name, 'cameras': cameras, 'time': meta.get('time', frame.stat().st_mtime),
                        'tcp_position': meta.get('tcp_position'), 'count': len(frames)}
        return None

    def frame_image(self, session_id, frame_id, camera):
        if not FRAME_ID.fullmatch(frame_id or '') or not CAMERA.fullmatch(camera or ''):
            raise ValueError('invalid frame or camera')
        path = self.frames_dir(self.root(session_id))/frame_id/f'{camera}.jpg'
        if not path.is_file():
            raise KeyError('no such image')
        return path.read_bytes()

    def transcript_file(self, root):
        projects = root/HOME_NAME/'projects'
        files = sorted(projects.glob('*/*.jsonl'), key=lambda p: p.stat().st_mtime) if projects.is_dir() else []
        return files[-1] if files else None

    @staticmethod
    def transcript_events(lines, limit=80):
        events = []
        for line in lines:
            try:
                entry = json.loads(line)
            except ValueError:
                continue
            kind, message = entry.get('type'), entry.get('message') or {}
            stamp = entry.get('timestamp')
            content = message.get('content')
            if isinstance(content, str):
                content = [{'type': 'text', 'text': content}]
            if kind not in ('assistant', 'user') or not isinstance(content, list):
                continue
            for block in content:
                if not isinstance(block, dict):
                    continue
                if block.get('type') == 'text' and block.get('text', '').strip():
                    events.append({'time': stamp, 'kind': 'assistant' if kind == 'assistant' else 'user',
                                   'text': truncate(block['text'], 4000)})
                elif block.get('type') == 'thinking' and block.get('thinking', '').strip():
                    events.append({'time': stamp, 'kind': 'thinking', 'text': truncate(block['thinking'], 1500)})
                elif block.get('type') == 'tool_use':
                    events.append({'time': stamp, 'kind': 'tool_call', 'name': block.get('name'),
                                   'text': truncate(block.get('input', {}), 1500)})
                elif block.get('type') == 'tool_result':
                    result = block.get('content')
                    if isinstance(result, list):
                        parts = []
                        for item in result:
                            if isinstance(item, dict) and item.get('type') == 'text':
                                parts.append(item.get('text', ''))
                            elif isinstance(item, dict) and item.get('type') == 'image':
                                parts.append('[image]')
                        result = '\n'.join(parts)
                    events.append({'time': stamp, 'kind': 'tool_result', 'error': bool(block.get('is_error')),
                                   'text': truncate(result if result is not None else '', 1200)})
        return events[-limit:]

    def activity(self, session_id, limit=80):
        root = self.root(session_id)
        transcript = self.transcript_file(root)
        mcp = []
        for line in tail_lines(root/'mcp.jsonl', 60):
            try:
                row = json.loads(line)
            except ValueError:
                continue
            mcp.append({'time': row.get('time') or row.get('ts'), 'event': row.get('event'), 'tool': row.get('tool'),
                        'text': truncate({k: v for k, v in row.items()
                                          if k not in ('time', 'ts', 'event', 'tool', 'call_id')}, 600)})
        return {
            'transcript_file': transcript.name if transcript else None,
            'transcript': self.transcript_events(tail_lines(transcript, 400), limit) if transcript else [],
            'mcp': mcp[-40:],
            'frontend_log': tail_lines(root/'frontend.log', 60),
            'launcher_log': tail_lines(root/'launcher.log', 60),
            'camera_log': tail_lines(root/'camera-server.log', 20),
        }
