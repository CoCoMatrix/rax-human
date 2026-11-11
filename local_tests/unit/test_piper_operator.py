"""Operator stop is independent of the MCP dispatch lock and never enables motion."""
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from services.franka import arming
from scripts import piper_operator


def configuration(tmp_path, *, bridge=True):
    root = tmp_path / 'new-session'
    owner = tmp_path / 'new-owner-session'
    options = ({'owner_session_root': str(owner), 'bridge_root': str(tmp_path / 'bridge')}
               if bridge else {})
    return SimpleNamespace(root=root, robot='piper', backend_options=options)


def seed_arming(config):
    arming.write(config.root, True)
    owner = config.backend_options.get('owner_session_root')
    if owner:
        arming.write(owner, True)


def test_disarms_both_before_reading_bridge_and_only_requests_cancel(tmp_path, monkeypatch):
    config = configuration(tmp_path)
    seed_arming(config)
    events = []
    class Bridge:
        def __init__(self, root, owner, session):
            self.owner, self.session = owner, session
        def status(self):
            assert arming.read(self.session)['armed'] is False
            assert arming.read(self.owner)['armed'] is False
            events.append('status')
            return {'phase': 'RUNNING', 'instance': 'live-instance'}
        def cancel(self):
            events.append('cancel')
            return {'stop_requested': True, 'physical_stop_confirmed': False}
        def move(self, *_):
            pytest.fail('Operator disarm must never dispatch movement')
    monkeypatch.setattr(piper_operator, 'make_bridge', Bridge)
    result = piper_operator.disarm(config)
    assert events == ['status', 'cancel']
    assert result['session_disarmed'] and result['owner_disarmed']
    assert result['cancel_requested'] and not result['physical_stop_confirmed']
    assert not result['errors']


def test_missing_bridge_still_persists_both_disarms(tmp_path):
    config = configuration(tmp_path)
    seed_arming(config)
    result = piper_operator.disarm(config)
    assert arming.read(config.root)['armed'] is False
    assert arming.read(config.backend_options['owner_session_root'])['armed'] is False
    assert result['session_disarmed'] and result['owner_disarmed']
    assert not result['cancel_requested'] and not result['physical_stop_confirmed']
    assert result['errors'][0]['stage'] == 'bridge_cancel'
    assert not (tmp_path / 'bridge/cancel.json').exists()


@pytest.mark.parametrize('bad', ['wrong_session', 'stale'])
def test_cancel_requires_current_matching_owner_identity(tmp_path, bad):
    config = configuration(tmp_path)
    seed_arming(config)
    bridge = tmp_path / 'bridge'
    bridge.mkdir()
    status = {'schema': 'rax.piper.motion_bridge.v1', 'instance': 'expected',
              'session_root': config.backend_options['owner_session_root'],
              'timestamp_ns': time.time_ns(), 'phase': 'RUNNING'}
    if bad == 'wrong_session':
        status['session_root'] = str(tmp_path / 'unrelated-owner')
    else:
        status['timestamp_ns'] -= 2_000_000_000
    (bridge / 'state.json').write_text(json.dumps(status))
    result = piper_operator.disarm(config)
    assert result['session_disarmed'] and result['owner_disarmed']
    assert not result['cancel_requested'] and result['errors']
    assert not (bridge / 'cancel.json').exists()


def test_real_file_protocol_writes_bound_cancel_without_motion(tmp_path):
    config = configuration(tmp_path)
    seed_arming(config)
    bridge = tmp_path / 'bridge'
    bridge.mkdir()
    (bridge / 'state.json').write_text(json.dumps({
        'schema': 'rax.piper.motion_bridge.v1', 'instance': 'expected',
        'session_root': config.backend_options['owner_session_root'],
        'timestamp_ns': time.time_ns(), 'phase': 'RUNNING'}))
    result = piper_operator.disarm(config)
    request = json.loads((bridge / 'cancel.json').read_text())
    assert request['instance'] == 'expected' and type(request['timestamp_ns']) is int
    assert result['cancel_requested'] and not result['physical_stop_confirmed']
    assert not (bridge / 'requests').exists()


def test_local_write_failure_does_not_prevent_owner_disarm_or_cancel(tmp_path, monkeypatch):
    config = configuration(tmp_path)
    seed_arming(config)
    write = arming.write
    def flaky(root, *args, **kwargs):
        if Path(root) == config.root:
            raise PermissionError('simulated unwritable new session')
        return write(root, *args, **kwargs)
    monkeypatch.setattr(arming, 'write', flaky)
    class Bridge:
        def __init__(self, *_): pass
        def status(self): return {'phase': 'RUNNING', 'instance': 'expected'}
        def cancel(self): return {'stop_requested': True}
    monkeypatch.setattr(piper_operator, 'make_bridge', Bridge)
    result = piper_operator.disarm(config)
    assert not result['session_disarmed'] and result['owner_disarmed']
    assert arming.read(config.backend_options['owner_session_root'])['armed'] is False
    assert result['cancel_requested'] and result['errors'][0]['stage'] == 'session_disarm'
    assert not result['physical_stop_confirmed']


def test_no_send_configuration_only_disarms_its_session(tmp_path):
    config = configuration(tmp_path, bridge=False)
    seed_arming(config)
    result = piper_operator.disarm(config)
    assert result['session_disarmed'] and result['owner_disarmed'] is None
    assert not result['cancel_requested'] and not result['physical_stop_confirmed']
    assert not result['errors']


def test_cli_has_no_arm_or_enable_action(tmp_path):
    for action in ('arm', 'enable', 'start', 'prepare'):
        with pytest.raises(SystemExit) as error:
            piper_operator.main(['--config', str(tmp_path / 'unused.json'), action])
        assert error.value.code == 2
    assert not list(tmp_path.rglob('armed.json'))


@pytest.mark.parametrize('invalid', ['', 'relative-owner'])
def test_invalid_owner_path_does_not_prevent_new_session_disarm(tmp_path, invalid, monkeypatch):
    config = configuration(tmp_path)
    arming.write(config.root, True)
    config.backend_options['owner_session_root'] = invalid
    monkeypatch.chdir(tmp_path)
    result = piper_operator.disarm(config)
    assert result['session_disarmed']
    assert not arming.read(config.root)['armed']
    assert result['errors'][0]['stage'] == 'configuration'
    assert not (tmp_path/'operator').exists()
    assert not (tmp_path/'relative-owner').exists()
