"""Codex launcher preserving upstream RAX workspace, MCP owner and memory contracts.

Prepare does not open hardware. Run requires Linux bubblewrap and a physical backend
whose normal operator arming applies; this launcher never arms or resets hardware.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

from services.controller.config import Configuration
from services.json_io import write_private_json
from services.workspace import Setup, build, recipe_for
from services.workspace.setup import ROBOT_FACTS
from .credentials import load_credentials
from .relay import ResponsesRelay

PROJECT = Path(__file__).resolve().parents[2]


def private_socket(root, suffix):
    directory = Path('/tmp') / f'rax-codex-{os.getuid()}'
    directory.mkdir(mode=0o700, exist_ok=True)
    directory.chmod(0o700)
    return directory / (hashlib.sha256(str(root).encode()).hexdigest()[:16] + suffix)


def codex_config(config, *, offline=False):
    model = config.agent_model or 'gpt-6-astra'
    # Use the CLI's native compaction, not a relay rewrite or a second compactor.
    # The operator may compact earlier without falsifying model context capacity.
    compact = (f'model_auto_compact_token_limit = {config.agent_auto_compact_token_limit}\n'
               if config.agent_auto_compact_token_limit is not None else '')
    text = f'''model = {json.dumps(model)}
model_provider = "rax_relay"
model_reasoning_effort = {json.dumps(config.agent_effort)}
approval_policy = "never"
sandbox_mode = "danger-full-access"
web_search = "disabled"
{compact}

[model_providers.rax_relay]
name = "RAX host credential relay"
base_url = "http://127.0.0.1:8789/v1"
env_key = "RAX_RELAY_PLACEHOLDER"
wire_api = "responses"
request_max_retries = 0
stream_max_retries = 0

[mcp_servers.robot]
command = "/usr/bin/python3"
args = ["/opt/rax-mcp.py", "/run/robot.sock"]
startup_timeout_sec = 30
tool_timeout_sec = {config.session_seconds + 60}
'''
    return text.split('[mcp_servers.robot]', 1)[0] if offline else text


def prepare(config, key_file, base_url, agent_runtime=None, http_proxy=None, *, gemini_key_file=None, model_http_proxy=None):
    if config.agent_provider != 'codex-relay':
        raise ValueError('Codex launcher requires agent_provider=codex-relay')
    root = config.root
    workspace = root/'agent-workspace'
    if workspace.exists():
        raise FileExistsError('Existing workspace: use --resume, not a second prepare')
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    root.chmod(0o700)
    workspace.mkdir(mode=0o700)
    for name in ('code', 'memory', 'output', 'runtime/frames', 'tmp'):
        (workspace/name).mkdir(parents=True, exist_ok=True)
    (root/'stream').mkdir(mode=0o755, exist_ok=True)
    (workspace/'runtime/stream').symlink_to(root/'stream', target_is_directory=True)
    from services import skill_library
    library = skill_library.checkout(config, workspace)
    manifest = build(recipe_for(), Setup.from_config(config, gemini=bool(gemini_key_file),
        vision=not bool(gemini_key_file), gemini_official=bool(gemini_key_file)), workspace)
    manifest['skill_library'] = library
    (workspace/'TASK.md').write_text(f'# Task: {config.task}\n\n{config.task_description.strip()}\n')
    home = root/'codex-home'
    home.mkdir(mode=0o700)
    (home/'config.toml').write_text(codex_config(config))
    write_private_json(root/'configuration.json', config.to_dict())
    write_private_json(root/'workspace-manifest.json', manifest)
    write_private_json(root/'codex-relay.json', {
        'key_file': str(Path(key_file).resolve()), 'base_url': base_url,
        'agent_runtime': str(Path(agent_runtime).resolve()) if agent_runtime else None,
        'primary_model': config.agent_model or 'gpt-6-astra', 'primary_effort': config.agent_effort,
        'vision_provider': 'google' if gemini_key_file else 'codex',
        'gemini_key_file': str(Path(gemini_key_file).resolve()) if gemini_key_file else None,
        'vision_model': 'gemini-3.8-flash' if gemini_key_file else 'gpt-6-astra',
        'vision_effort': None if gemini_key_file else 'medium', 'http_proxy': http_proxy,
        'model_http_proxy':http_proxy if model_http_proxy is None else model_http_proxy})
    return workspace


def sandbox_command(config, robot_socket, model_socket, *, prompt, resume=False, agent_runtime=None,
                    offline=False, workspace=None, home=None):
    bwrap = shutil.which('bwrap')
    codex = shutil.which('codex')
    if not bwrap or not codex or sys.platform != 'linux':
        raise RuntimeError('Execution requires Linux, bubblewrap and Codex CLI; prepare-only works elsewhere')
    workspace = Path(workspace) if workspace is not None else config.root/'agent-workspace'
    home = Path(home) if home is not None else config.root/'codex-home'
    cmd = [bwrap, '--die-with-parent', '--new-session', '--unshare-all', '--clearenv']
    for name in ('/usr', '/bin', '/sbin', '/lib', '/lib64'):
        p = Path(name)
        if p.is_symlink():
            cmd += ['--symlink', os.readlink(p), name]
        elif p.exists():
            cmd += ['--ro-bind', name, name]
    cmd += ['--proc', '/proc', '--dev', '/dev', '--tmpfs', '/tmp', '--dir', '/run',
            '--dir', '/opt', '--dir', '/home/agent',
            '--bind', str(workspace), str(workspace),
            '--bind', str(home), '/home/agent/.codex',
            '--ro-bind', str(model_socket), '/run/model.sock',
            '--ro-bind', str(PROJECT/'harness/codex_cli/sandbox_entry.py'), '/opt/rax-entry.py']
    if not offline:
        cmd += ['--ro-bind', str(config.root/'stream'), str(config.root/'stream'),
                '--ro-bind', str(robot_socket), '/run/robot.sock',
                '--ro-bind', str(PROJECT/'harness/claude_cli/mcp_bridge.py'), '/opt/rax-mcp.py']
    from services.skill_library import library_path
    library = library_path(config)
    if not offline and (workspace/'skill-library/.git').is_dir():
        cmd += ['--ro-bind', str(library), str(library)]
    path = '/usr/local/bin:/usr/bin:/bin'
    if agent_runtime:
        runtime = Path(agent_runtime).resolve()
        if not (runtime/'bin/python').is_file() or not (runtime/'lib').is_dir():
            raise ValueError('agent runtime must contain its own bin/python and lib directory')
        cmd += ['--ro-bind', str(runtime), '/agent-runtime']
        path = '/agent-runtime/bin:' + path
    env = {'HOME': '/home/agent', 'CODEX_HOME': '/home/agent/.codex', 'PATH': path,
           'PYTHONPATH': str(workspace), 'PYTHONDONTWRITEBYTECODE': '1',
           'RAX_MCP_SOCKET': '/run/robot.sock', 'RAX_RELAY_PLACEHOLDER': 'host-relay',
           'LANG': 'C.UTF-8', 'TERM': 'xterm-256color'}
    if offline:
        env.pop('RAX_MCP_SOCKET', None)
    for key, value in env.items():
        cmd += ['--setenv', key, value]
    cmd += ['--chdir', str(workspace), '/usr/bin/python3', '/opt/rax-entry.py', codex,
            'exec', '--dangerously-bypass-approvals-and-sandbox', '--json', '--skip-git-repo-check']
    if resume:
        cmd += ['resume', '--last']
    cmd += [prompt]
    return cmd


def wait_frontend(process, socket_path, root, timeout=30):
    """Require this child process's ready record, not a stale socket from a prior run."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'Robot service exited during startup; inspect {root}/frontend.log')
        try:
            ready = json.loads((root/'frontend-ready.json').read_text())
            if ready.get('pid') == process.pid:
                with socket.socket(socket.AF_UNIX) as connection:
                    connection.settimeout(.2)
                    connection.connect(str(socket_path))
                return
        except (OSError, ValueError):
            pass
        time.sleep(.05)
    raise TimeoutError(f'Robot service startup timed out; inspect {root}/frontend.log')


def stop_process(process, *, grace=10, group=False):
    """Bounded process cleanup; process exit is never evidence of physical rest."""
    result = {'stop_requested': False, 'forced_kill': False, 'exit_code': None,
              'physical_stop_confirmed': False}
    if process is None:
        return result
    if process.poll() is None:
        result['stop_requested'] = True
        try:
            os.killpg(process.pid, signal.SIGTERM) if group else process.terminate()
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            result['forced_kill'] = True
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
    result['exit_code'] = process.returncode
    return result


def run(config, *, prompt=None, resume=False):
    root = config.root
    settings = json.loads((root/'codex-relay.json').read_text())
    key, base_url = load_credentials(settings['key_file'], settings['base_url'])
    robot_socket, model_socket = private_socket(root, '.robot'), private_socket(root, '.model')
    lifecycle_path = root/'codex-lifecycle.json'
    lifecycle = json.loads(lifecycle_path.read_text()) if lifecycle_path.exists() else {'elapsed_s': 0, 'launches': []}
    remaining = config.session_seconds - lifecycle['elapsed_s']
    if remaining <= 0:
        raise RuntimeError('No session wall-clock budget remains')
    prompt = prompt or (('Read memory/PROGRESS.md and memory/INDEX.md if present, call status and continue '
                         'from the live state without repeating completed work.') if resume else
                         (root/'agent-workspace/START_PROMPT.md').read_text())
    prompt += (f'\n\nMeasured session budget at launch: {remaining:.1f} seconds remain '
               f'(total {config.session_seconds}s, already used {lifecycle["elapsed_s"]:.1f}s). '
               'Every robot tool response contains wall_budget with actual remaining time. '
               'Never estimate elapsed time from the number of steps. In the final '
               f'{min(60, config.session_seconds/3):g}s finish reserve, stop starting motions, '
               'save a concise factual PROGRESS.md and INDEX.md, then return a final handoff. '
               'Do not release a held object merely because the budget is ending.')
    cmd = sandbox_command(config, robot_socket, model_socket, prompt=prompt, resume=resume,
                          agent_runtime=settings.get('agent_runtime'))
    started = time.time()
    started_monotonic = time.monotonic()
    write_private_json(root/'codex-budget.json', {
        'started_at': started, 'previous_elapsed_s': lifecycle['elapsed_s'],
        'session_seconds': config.session_seconds, 'remaining_at_launch_s': remaining,
        'deadline_at': started+remaining, 'finish_reserve_s': min(60, config.session_seconds/3),
        'resume': resume})
    write_private_json(root/'agent-lifecycle.json', {'status': 'preparing', 'pid': os.getpid(),
        'started_at': started, 'provider': config.agent_provider})
    frontend = relay = process = frontend_log = None
    received_signal = None
    code = None
    def interrupted(signum, _):
        nonlocal received_signal
        received_signal = signum
        raise SystemExit(128 + signum)
    old = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGTERM, signal.SIGINT)}
    try:
        for name in ('frontend-ready.json', 'frontend-exit.json'):
            (root/name).unlink(missing_ok=True)
        frontend_log = (root/'frontend.log').open('ab')
        frontend = subprocess.Popen([
            sys.executable, '-m', 'harness.codex_cli.frontend', '--config', str(root/'configuration.json'),
            '--socket', str(robot_socket), '--started-at', str(started-lifecycle['elapsed_s']),
            '--budget-started-monotonic', str(started_monotonic),
            '--budget-previous-elapsed', str(lifecycle['elapsed_s'])],
            cwd=PROJECT, env={**os.environ, 'PYTHONPATH': str(PROJECT)},
            stdin=subprocess.DEVNULL, stdout=frontend_log, stderr=frontend_log, start_new_session=True)
        wait_frontend(frontend, robot_socket, root, timeout=min(30, remaining))
        relay = ResponsesRelay(model_socket, key, base_url, settings['primary_model'], settings['primary_effort'],
                               root, config.max_model_calls, proxy_url=settings.get('model_http_proxy',settings.get('http_proxy')), native_camera_jpeg=ROBOT_FACTS[config.robot].get('native_camera_jpeg', False))
        relay.start()
        log = root/f'codex-{int(started)}.jsonl'
        print(json.dumps({'status': 'launching', 'transcript': str(log), 'remaining_seconds': remaining,
                          'frontend_pid': frontend.pid}), flush=True)
        with log.open('w') as output:
            process = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=output,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            write_private_json(root/'agent-lifecycle.json', {'status': 'running', 'pid': os.getpid(),
                'started_at': started, 'provider': config.agent_provider, 'transcript': log.name})
            deadline = started_monotonic + remaining
            while process.poll() is None:
                if frontend.poll() is not None:
                    raise RuntimeError('Robot service exited while the agent was running; stopping agent')
                if time.monotonic() >= deadline:
                    code = 124
                    break
                time.sleep(.1)
            if code is None:
                code = process.returncode
        return code
    finally:
        # Stop the agent first. Only the host child owns and closes its backend/solver.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        agent_stop = frontend_stop = None
        try:
            agent_stop = stop_process(process, grace=5, group=True)
        finally:
            try:
                frontend_stop = stop_process(frontend, grace=10)
            finally:
                try:
                    if relay:
                        relay.close()
                finally:
                    if frontend_log:
                        frontend_log.close()
                    shutdown = {}
                    try:
                        shutdown = json.loads((root/'frontend-exit.json').read_text())
                    except (OSError, ValueError):
                        pass
                    clean = (frontend_stop is not None and not frontend_stop['forced_kill']
                             and shutdown.get('shutdown_completed') is True)
                    elapsed = time.monotonic()-started_monotonic
                    write_private_json(root/'agent-lifecycle.json', {'status': 'completed' if code == 0 and clean else 'failed',
                        'pid': os.getpid(), 'started_at': started, 'finished_at': time.time(),
                        'returncode': code, 'provider': config.agent_provider, 'shutdown_completed': clean})
                    lifecycle['elapsed_s'] += elapsed
                    lifecycle['launches'].append({'started_at': started, 'elapsed_s': elapsed, 'resume': resume,
                        'exit_code': process.returncode if process else None, 'launcher_result': code,
                        'received_signal': received_signal, 'agent_stop': agent_stop, 'frontend_stop': frontend_stop,
                        'frontend_shutdown': shutdown, 'control_state': 'not_assessed' if clean else 'unknown',
                        'physical_stop_confirmed': False})
                    write_private_json(lifecycle_path, lifecycle)
                    for sig, handler in old.items():
                        signal.signal(sig, handler)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--key-file', type=Path, default=Path('~/.config/rax-human/main-key.py').expanduser())
    parser.add_argument('--base-url')
    parser.add_argument('--http-proxy', help='Host-only HTTP proxy URL for model requests')
    parser.add_argument('--gemini-key-file', type=Path, help='Host-only Google official Gemini key; enables native gemini_generate instead of Codex vision')
    parser.add_argument('--agent-runtime-dir', type=Path)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--exec-prompt')
    args = parser.parse_args()
    config = Configuration.load(args.config)
    config.root.mkdir(mode=0o700, parents=True, exist_ok=True)
    with (config.root/'launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not args.resume and not (config.root/'codex-relay.json').exists():
            prepare(config, args.key_file, args.base_url, args.agent_runtime_dir, args.http_proxy,
                    gemini_key_file=args.gemini_key_file)
        elif not args.resume and not args.prepare_only and (config.root/'codex-lifecycle.json').exists():
            raise SystemExit('Session already launched: use --resume to retain its conversation')
        if args.prepare_only:
            print(json.dumps({'status': 'prepared_no_hardware', 'root': str(config.root)}))
            return
        raise SystemExit(run(config, prompt=args.exec_prompt, resume=args.resume))


if __name__ == '__main__':
    main()
