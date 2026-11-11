#!/usr/bin/env python3
"""Open the installed console and connect read-only preview once, never move arms."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shlex
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

HERE = Path(__file__).resolve().parent


def load_config(path=None):
    chosen = Path(path) if path else HERE / 'console.local.json'
    if path is None and not chosen.exists():
        chosen = HERE / 'console.example.json'
    config = json.loads(chosen.read_text())
    for key in ('host', 'root', 'python'):
        if not isinstance(config.get(key), str) or not config[key] or '\n' in config[key]:
            raise ValueError('Invalid console setting: ' + key)
    if config['host'].startswith('-'):
        raise ValueError('SSH host cannot start with a dash')
    for key in ('remote_port', 'local_port'):
        if type(config.get(key)) is not int or not 1024 <= config[key] <= 65535:
            raise ValueError('Invalid port: ' + key)
    timeout = config.get('connect_timeout_s', 75)
    if not isinstance(timeout, (float, int)) or not 1 <= timeout <= 180:
        raise ValueError('Invalid connection timeout')
    return config


def bootstrap_script(config):
    # Sent through stdin, never interpolated into a shell or logged with credentials.
    return '''from pathlib import Path
import json,os,socket,subprocess,time
cfg=json.loads(%r)
root=Path(cfg['root']); runtime=root/'runtime/scale-tests'
required=['services/dashboard/piper_scale.py','services/dashboard/server.py',
          'scripts/piper_preview.py','scripts/piper_prepare.py','scripts/piper_explore.py']
missing=[name for name in required if not (root/name).is_file()]
if missing: raise RuntimeError('Missing installed RAX runtime files: '+', '.join(missing))
runtime.mkdir(parents=True,exist_ok=True)
def listening():
 with socket.socket() as sock:
  sock.settimeout(1); return sock.connect_ex(('127.0.0.1',cfg['remote_port']))==0
if not listening():
 with (runtime/'web-server.log').open('ab') as log:
  process=subprocess.Popen([cfg['python'],'-m','services.dashboard.piper_scale','--port',str(cfg['remote_port'])],
    cwd=root,env={**os.environ,'PYTHONPATH':str(root)},stdin=subprocess.DEVNULL,
    stdout=log,stderr=log,start_new_session=True)
 for _ in range(100):
  if listening() and (runtime/'access-token').is_file(): break
  if process.poll() is not None: raise RuntimeError('Web service exited; inspect runtime/scale-tests/web-server.log')
  time.sleep(.2)
 if not listening(): raise RuntimeError('Web service startup timeout')
print(json.dumps({'token':(runtime/'access-token').read_text().strip()}))
''' % json.dumps(config)


def bootstrap(config, local=False):
    command = [config['python'], '-'] if local else [
        'ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
        config['host'], shlex.quote(config['python']) + ' -']
    result = subprocess.run(command, input=bootstrap_script(config), text=True,
                            capture_output=True, timeout=40)
    if result.returncode:
        raise RuntimeError('Web service/SSH startup failed: ' + result.stderr[-1200:])
    return json.loads(result.stdout)['token']


def ensure_tunnel(config):
    # Match the existing operator launcher's socket on the current 5090 profile.
    cache = Path.home() / '.cache/rax-scale-console'
    cache.mkdir(parents=True, exist_ok=True)
    default = (config['host'], config['local_port'], config['remote_port']) == ('ubuntu@192.168.200.102', 8781, 8781)
    identity = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()[:12]
    sock = cache / ('ssh.sock' if default else identity + '.sock')
    check = subprocess.run(['ssh', '-S', str(sock), '-O', 'check', config['host']],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
    if check.returncode:
        forward = f"127.0.0.1:{config['local_port']}:127.0.0.1:{config['remote_port']}"
        result = subprocess.run(['ssh', '-M', '-S', str(sock), '-fN', '-o', 'BatchMode=yes',
            '-o', 'ExitOnForwardFailure=yes', '-o', 'ConnectTimeout=8',
            '-o', 'ServerAliveInterval=30', '-o', 'ServerAliveCountMax=3',
            '-L', forward, config['host']], capture_output=True, text=True, timeout=20)
        if result.returncode:
            raise RuntimeError('SSH tunnel failed; choose a free local_port: ' + result.stderr[-600:])


class Console:
    def __init__(self, base, token):
        self.base, self.token = base, token
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(self, path, body=None):
        request = urllib.request.Request(self.base + path,
            data=None if body is None else json.dumps(body).encode(),
            headers={'X-Operator-Token': self.token, 'Content-Type': 'application/json'})
        try:
            with self.opener.open(request, timeout=12) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            try:
                message = json.loads(exc.read()).get('error', '')
            except (ValueError, AttributeError):
                message = ''
            raise RuntimeError(f'Console HTTP {exc.code}: {message}') from None

    def status(self):
        state = self.request('/api/status')
        if not all(key in state for key in ('busy', 'connected', 'phase', 'mode', 'version')):
            raise RuntimeError('Port does not serve the expected RAX console')
        return state


def connect_readonly(console, timeout=75, clock=time.monotonic, sleep=time.sleep):
    state = console.status()
    # Reopening the launcher must preserve an existing Prepare or task session.
    if state['busy'] or state['connected']:
        return {'connection': 'reused', 'phase': state['phase']}
    console.request('/api/action/connect', {})  # exactly one POST; no automatic retry
    deadline = clock() + timeout
    while clock() < deadline:
        state = console.status()
        if state['connected'] and not state['busy']:
            cameras = state.get('telemetry', {}).get('cameras', {})
            if state['mode'] == 'preview':
                if not all(cameras.get(n, {}).get('available') for n in ('head', 'left', 'right')):
                    raise RuntimeError('Read-only preview is missing camera feedback')
                return {'connection': 'readonly_connected', 'phase': state['phase']}
            return {'connection': 'reused', 'phase': state['phase']}
        if not state['busy'] and state.get('error'):
            raise RuntimeError(state['error'])
        if not state['busy'] and not state['connected']:
            raise RuntimeError('Read-only connection ended: ' + str(state['phase']))
        sleep(.4)
    raise TimeoutError('Read-only connection timeout; inspect the open Web console, no retry sent')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='JSON connection settings; no credentials')
    parser.add_argument('--local', action='store_true', help='Run on the 5090 desktop without SSH')
    parser.add_argument('--open-only', action='store_true', help='Open page without connecting hardware')
    parser.add_argument('--no-browser', action='store_true', help='Validate connection without opening a window')
    args = parser.parse_args(argv)
    token = ''
    try:
        config = load_config(args.config)
        token = bootstrap(config, args.local)
        if not args.local:
            ensure_tunnel(config)
        port = config['remote_port'] if args.local else config['local_port']
        console = Console(f'http://127.0.0.1:{port}', token)
        console.status()  # authenticate and verify service before opening the browser
        if not args.no_browser:
            url = console.base + '/?' + urllib.parse.urlencode({'token': token})
            opener = 'open' if sys.platform == 'darwin' else 'xdg-open'
            opened = subprocess.run([opener, url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if opened.returncode:
                raise RuntimeError('Browser could not open; retry from a graphical desktop')
        result = {'connection': 'open_only'} if args.open_only else connect_readonly(console, config.get('connect_timeout_s', 75))
        print(json.dumps(result, ensure_ascii=False))
        print('控制台连接已验证；未调用 Prepare、Run 或夹爪动作。')
        return 0
    except Exception as exc:
        message = str(exc).replace(token, '[redacted]') if token else str(exc)
        print('无法完成连接：' + message, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
