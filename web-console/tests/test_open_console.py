"""Launcher boundaries without SSH, cameras, CAN or motor access."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import open_console as launch
import install_web


class Client:
    def __init__(self, states, error=None):
        self.states = iter(states)
        self.posts = []
        self.error = error

    def status(self):
        return next(self.states)

    def request(self, path, body):
        self.posts.append((path, body))
        if self.error:
            raise self.error
        return {'accepted': True}


def state(**values):
    return {'busy': False, 'connected': False, 'phase': '未连接', 'mode': 'disconnected', **values}


@pytest.mark.parametrize('active', [state(busy=True), state(connected=True, mode='prepare'),
                                    state(connected=True, mode='preview')])
def test_reopen_preserves_existing_session(active):
    client = Client([active])
    assert launch.connect_readonly(client)['connection'] == 'reused'
    assert client.posts == []


def test_connect_once_then_wait_for_all_cameras():
    ready = state(connected=True, mode='preview', phase='只读预览',
                  telemetry={'cameras': {n: {'available': True} for n in ('head', 'left', 'right')}})
    client = Client([state(), state(busy=True), ready])
    assert launch.connect_readonly(client, sleep=lambda _: None)['connection'] == 'readonly_connected'
    assert client.posts == [('/api/action/connect', {})]


def test_post_failure_is_not_retried():
    client = Client([state()], error=RuntimeError('owner is busy'))
    with pytest.raises(RuntimeError, match='owner is busy'):
        launch.connect_readonly(client)
    assert len(client.posts) == 1


def test_feedback_failure_never_starts_prepare_or_run():
    client = Client([state(), state(error='Camera unavailable')])
    with pytest.raises(RuntimeError, match='Camera unavailable'):
        launch.connect_readonly(client)
    assert client.posts == [('/api/action/connect', {})]


def test_timeout_does_not_resubmit():
    client = Client([state()])
    ticks = iter([0, 100])
    with pytest.raises(TimeoutError):
        launch.connect_readonly(client, timeout=1, clock=lambda: next(ticks))
    assert len(client.posts) == 1


def test_bootstrap_uses_python_stdin_and_quotes_interpreter(monkeypatch):
    config = launch.load_config()
    config['python'] = '/tmp/a b/python'
    config['root'] = "/tmp/example's root"
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, '{"token":"test-only"}', '')
    monkeypatch.setattr(subprocess, 'run', run)
    assert launch.bootstrap(config) == 'test-only'
    command, options = calls[0]
    assert command[-1] == "'/tmp/a b/python' -"
    compile(options['input'], '<bootstrap>', 'exec')
    assert config['root'] not in command[-1]


def test_invalid_settings(tmp_path):
    path = tmp_path / 'config.json'
    config = launch.load_config(); config['host'] = '-danger'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        launch.load_config(path)


def runtime(tmp_path):
    for name in install_web.REQUIRED:
        path = tmp_path / name; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('placeholder')
    (tmp_path / 'examples/blue-cup-exploration.json').write_text(json.dumps({
        'workspace_min': [.055, -.4, .17], 'workspace_max': [.5, .045, .36]}))
    return tmp_path


def test_install_backs_up_files_and_never_launches(tmp_path, monkeypatch):
    root = runtime(tmp_path)
    for name in install_web.FILES:
        p = root / name; p.parent.mkdir(parents=True, exist_ok=True); p.write_text('old')
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('No processes at install'))
    result = install_web.install(root)
    assert result['hardware_commands'] == 0
    for name in install_web.FILES:
        assert (Path(result['backup']) / name).read_text() == 'old'
        assert (root / name).read_bytes() == (ROOT / 'runtime-overlay' / name).read_bytes()


def test_install_refuses_active_server(tmp_path):
    import fcntl
    root = runtime(tmp_path); p = root / 'runtime/scale-tests/server.lock'
    p.parent.mkdir(parents=True)
    with p.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match='active'):
            install_web.install(root)
    assert not (root / 'backups').exists()


def test_check_has_no_writes_and_rejects_wrong_workspace(tmp_path):
    root = runtime(tmp_path)
    assert install_web.install(root, check=True)['check_only']
    assert not (root / 'runtime').exists()
    (root / 'examples/blue-cup-exploration.json').write_text('{}')
    with pytest.raises(ValueError, match='workspace'):
        install_web.install(root)
