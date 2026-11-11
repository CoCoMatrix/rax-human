import json
import threading

import pytest

from services.piper.bridge import OwnerBridge, RejectedBeforeDispatch


def bridge_at(tmp_path, session='session'):
    root = tmp_path / 'bridge'
    (root / 'requests').mkdir(parents=True, exist_ok=True)
    (root / 'results').mkdir(exist_ok=True)
    return OwnerBridge(root, tmp_path / 'owner', tmp_path / session)


def move_in_thread(bridge):
    outcome = []

    def move():
        try:
            outcome.append(bridge.move([0.] * 14))
        except BaseException as exc:
            outcome.append(exc)

    thread = threading.Thread(target=move)
    thread.start()
    return thread, outcome


def test_close_during_preflight_never_publishes(tmp_path, monkeypatch):
    bridge = bridge_at(tmp_path)
    entered, proceed = threading.Event(), threading.Event()
    calls = 0

    def status():
        nonlocal calls
        calls += 1
        if calls == 1:
            entered.set()
            assert proceed.wait(5), 'test did not release preflight'
            return {'phase': 'IDLE', 'request_id': None}
        return {'phase': 'FAULT', 'request_id': None}

    monkeypatch.setattr(bridge, 'status', status)
    monkeypatch.setattr(bridge, 'armed', lambda: None)
    thread, outcome = move_in_thread(bridge)
    try:
        assert entered.wait(5), 'move did not enter preflight'
        bridge.close()
        assert bridge.closed and bridge.task_lock is None
    finally:
        proceed.set()
        thread.join(5)
    assert not thread.is_alive()
    assert not list((bridge.root / 'requests').glob('*.json'))
    assert len(outcome) == 1 and isinstance(outcome[0], RejectedBeforeDispatch)
    assert outcome[0].dispatch_started is False


def test_active_close_requests_cancel_without_releasing_claim_early(tmp_path, monkeypatch):
    bridge = bridge_at(tmp_path)
    other = bridge_at(tmp_path, 'other-session')
    entered, finish = threading.Event(), threading.Event()
    calls = 0

    def status():
        nonlocal calls
        calls += 1
        if calls == 1:
            return {'phase': 'IDLE', 'request_id': None}
        entered.set()
        assert finish.wait(5), 'test did not release active move'
        return {'phase': 'FAULT', 'request_id': None}

    monkeypatch.setattr(bridge, 'status', status)
    monkeypatch.setattr(bridge, 'armed', lambda: None)
    thread, outcome = move_in_thread(bridge)
    try:
        assert entered.wait(5), 'move did not publish'
        bridge.close()
        assert bridge.closed and bridge.active and thread.is_alive()
        assert (bridge.root / 'cancel.json').is_file()
        with pytest.raises(ValueError, match='Another RAX session'):
            other._claim()
    finally:
        finish.set()
        thread.join(5)
    assert not thread.is_alive()
    assert len(outcome) == 1 and isinstance(outcome[0], RuntimeError)
    assert not bridge.active and bridge.task_lock is None
    other._claim()
    other.close()


def test_close_cannot_split_request_publication_from_active_state(tmp_path, monkeypatch):
    bridge = bridge_at(tmp_path)
    writing, publish, closing = (threading.Event() for _ in range(3))
    close_done, polling, finish = (threading.Event() for _ in range(3))
    lifecycle = bridge._lifecycle

    class ObservedLock:
        def __enter__(self):
            if threading.current_thread().name == 'close-client':
                closing.set()
            return lifecycle.__enter__()

        def __exit__(self, *args):
            return lifecycle.__exit__(*args)

    bridge._lifecycle = ObservedLock()
    write = bridge._write

    def gated_write(path, value):
        if path.parent.name == 'requests':
            writing.set()
            assert publish.wait(5), 'test did not release publication'
        write(path, value)

    calls = 0

    def status():
        nonlocal calls
        calls += 1
        if calls == 1:
            return {'phase': 'IDLE', 'request_id': None}
        polling.set()
        assert finish.wait(5), 'test did not release active move'
        return {'phase': 'FAULT', 'request_id': None}

    def close():
        bridge.close()
        close_done.set()

    monkeypatch.setattr(bridge, '_write', gated_write)
    monkeypatch.setattr(bridge, 'status', status)
    monkeypatch.setattr(bridge, 'armed', lambda: None)
    move_thread, outcome = move_in_thread(bridge)
    close_thread = threading.Thread(target=close, name='close-client')
    try:
        assert writing.wait(5), 'move did not enter publication'
        close_thread.start()
        assert closing.wait(5), 'close did not reach lifecycle lock'
        assert not close_done.is_set() and not bridge.closed
        publish.set()
        assert polling.wait(5), 'move did not enter result wait'
        assert close_done.wait(5), 'close waited for motion completion'
        assert bridge.active and bridge.task_lock is not None
        assert (bridge.root / 'cancel.json').is_file()
    finally:
        publish.set()
        finish.set()
        move_thread.join(5)
        if close_thread.ident is not None:
            close_thread.join(5)
    assert not move_thread.is_alive() and not close_thread.is_alive()
    assert len(outcome) == 1 and isinstance(outcome[0], RuntimeError)
    assert bridge.closed and not bridge.active and bridge.task_lock is None


def test_closed_client_cannot_start_another_move(tmp_path, monkeypatch):
    bridge = bridge_at(tmp_path)
    bridge.close()
    monkeypatch.setattr(bridge, 'status', lambda: pytest.fail('closed move read owner'))
    with pytest.raises(RejectedBeforeDispatch, match='closed'):
        bridge.move([0.] * 14)
    assert not list((bridge.root / 'requests').glob('*.json'))


def test_operator_cancel_does_not_need_motion_claim(tmp_path):
    bridge = bridge_at(tmp_path)
    operator = bridge_at(tmp_path, 'operator')
    bridge._claim()
    operator.instance = 'current-owner-instance'
    try:
        result = operator.cancel()
        assert result['stop_requested'] is True
        assert result['physical_stop_confirmed'] is False
        assert json.loads((bridge.root / 'cancel.json').read_text())['instance'] == operator.instance
        assert operator.task_lock is None
    finally:
        bridge.close()
        operator.close()


@pytest.mark.parametrize('extra', [
    {'observation_recovery': {'samples': 2}},
    {'error': 'feedback unavailable'},
    {'stop': {'reason': 'operator Stop'}},
])
def test_idle_recovery_or_error_cannot_publish_motion(tmp_path, monkeypatch, extra):
    bridge = bridge_at(tmp_path)
    monkeypatch.setattr(bridge, 'status', lambda: {'phase': 'DONE', 'request_id': None, **extra})
    monkeypatch.setattr(bridge, 'armed', lambda: None)
    try:
        with pytest.raises(RejectedBeforeDispatch) as error:
            bridge.move([0.] * 14)
        assert error.value.dispatch_started is False
        assert not list((bridge.root / 'requests').glob('*.json'))
    finally:
        bridge.close()
