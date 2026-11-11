"""scripts/configure_session.py: a Franka session carries the operator's camera configuration."""
import json
import os
from pathlib import Path
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[2]


def configure(tmp_path, *args, **environment):
    env = {k: v for k, v in os.environ.items() if k != 'RAX_BACKEND_OPTIONS'}
    env.update({'RAX_ARTIFACT_ROOT': str(tmp_path), **environment})
    return subprocess.run([sys.executable, str(PROJECT/'scripts/configure_session.py'), '--task', 'stack_cups',
                           '--task-text', 'Stack the cups.', *args], capture_output=True, text=True, env=env,
                          timeout=60)


def test_a_franka_session_without_cameras_is_refused(tmp_path):
    result = configure(tmp_path)
    assert result.returncode == 2
    assert 'a franka session needs its cameras' in result.stderr
    assert not (tmp_path/'sessions').exists()


def test_backend_options_default_to_the_environment_file(tmp_path):
    example = PROJECT/'examples/franka.json'
    result = configure(tmp_path, RAX_BACKEND_OPTIONS=str(example))
    assert result.returncode == 0, result.stderr
    saved = json.loads(Path(result.stdout.strip()).read_text())
    assert saved['robot'] == 'franka'
    assert saved['backend_options'] == json.loads(example.read_text())
    assert Path(saved['root']).is_relative_to(tmp_path/'sessions')


def test_the_mock_robot_needs_no_options(tmp_path):
    result = configure(tmp_path, '--robot', 'mock')
    assert result.returncode == 0, result.stderr
    assert json.loads(Path(result.stdout.strip()).read_text())['backend_options'] == {}
