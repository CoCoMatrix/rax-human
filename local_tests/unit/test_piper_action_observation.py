"""Post-action images must never turn a completed motion into a retryable failure."""
import json
import time

import pytest

from conftest import configuration
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.franka import arming
from services.piper.backend import PiperBackend
from test_piper_adapter import Bridge, Owner


def rig(tmp_path):
    config = configuration(tmp_path, robot='piper', observation_profile='rgb-only',
                           backend_options={'motion_enabled': True,
                                            'workspace_min': [-1, -1, -1],
                                            'workspace_max': [1, 1, 1]})
    events = []
    bridge = Bridge()
    original_move = bridge.move
    bridge.durations = []
    owner=Owner()
    owner_state=owner.state
    measured=[None]
    def state():
        row=owner_state()
        if measured[0] is not None:row['robot_state']['values']=measured[0].copy()
        row['robot_state']['feedback_metadata']={'source_timestamp_ns':time.time_ns(),'maximum_age_ms':0}
        row['robot_status']['lifecycle_state']='running'
        return row
    owner.state=state

    def move(q, duration):
        events.append('motion')
        bridge.durations.append(duration)
        measured[0]=list(q)
        return {**original_move(q, duration),'measured_settled':True,'target14':list(q),
                'joint_tolerance_rad':.003,'owner_generation':1}

    bridge.move = move
    backend = PiperBackend(config, owner=owner, bridge=bridge)
    original_capture = backend.cameras.capture

    def capture(*args, **kwargs):
        events.append('observe')
        return original_capture(*args, **kwargs)

    backend.cameras.capture = capture
    arming.write(config.root, True, minutes=2)
    return Frontend(Session(config), backend), bridge, events


def value(reply):
    return json.loads(reply['content'][0]['text'])


def test_arrival_failure_preserves_measured_error_without_claiming_hold(tmp_path):
    frontend, bridge, _ = rig(tmp_path)
    def fail(q, duration):
        raise RuntimeError('motion arrival timeout')
    bridge.move = fail
    try:
        reply = frontend.call('piper_move_joints', {'joints': [0, 0, 0, .004965634, 0, 0]})
        error = value(reply)['error']
        assert error['control_uncertain'] is True
        measured = error['robot_state']
        assert measured['values'][3] == 0
        assert measured['motion_failure']['max_error_joint'] == 'left_j4'
        assert measured['motion_failure']['max_joint_error_rad'] == pytest.approx(.004965634)
        assert measured['motion_failure']['physical_stop_confirmed'] is False
    finally:
        frontend.close()


@pytest.mark.parametrize('name,args', [
    ('piper_move_joints', {'joints': [.01, 0, 0, 0, 0, 0]}),
    ('piper_move_pose', {'position': [.057, 0, .213]}),
    ('piper_gripper', {'width_m': .02}),
])
def test_one_action_returns_measured_receipt_and_saved_images(tmp_path, name, args):
    frontend, bridge, events = rig(tmp_path)
    frontend.backend.solver.solve = lambda *args: {'q': [.01, 0, 0, 0, 0, 0]}
    try:
        reply = frontend.call(name, {**args, 'observe_after': True})
        result = value(reply)
        assert not reply.get('isError'), result
        assert result['executed'] is True and result['measured_arrival'] is True
        assert events == ['motion', 'observe']
        assert bridge.durations == [None]  # Owner chooses timing; no invented default.
        assert len(bridge.moves) == 1
        assert len([b for b in reply['content'] if b['type'] == 'image']) == 3
        frame = frontend.s.config.root / 'agent-workspace/runtime/frames' / result['observation']['frame_id']
        assert (frame / 'meta.json').is_file()
        assert all((frame / (camera + '.png')).is_file() for camera in ('head', 'left', 'right'))
    finally:
        frontend.close()


def test_capture_failure_preserves_success_and_only_observation_is_retried(tmp_path):
    frontend, bridge, events = rig(tmp_path)
    original_capture = frontend.backend.cameras.capture

    def fail(*args, **kwargs):
        events.append('observe_failed')
        raise ValueError('Cannot read /private/camera/sample: stale frame')

    frontend.backend.cameras.capture = fail
    try:
        reply = frontend.call('piper_move_joints', {'joints': [.01, 0, 0, 0, 0, 0], 'observe_after': True})
        result = value(reply)
        assert not reply.get('isError'), result
        assert result['executed'] is True and result['measured_arrival'] is True
        assert result['observation'] is None
        assert 'stale frame' in result['observation_error']['reason']
        assert '/private/' not in result['observation_error']['reason']
        assert 'observe' in result['observation_error']['recovery']
        assert frontend.control_uncertain is False
        assert events == ['motion', 'observe_failed'] and len(bridge.moves) == 1
        frontend.backend.cameras.capture = original_capture
        assert not frontend.call('piper_observe', {}).get('isError')
        assert events == ['motion', 'observe_failed', 'observe'] and len(bridge.moves) == 1
    finally:
        frontend.close()


def test_post_action_boundary_is_taken_after_receipt_and_capture_timeout_is_not_motion_failure(tmp_path, monkeypatch):
    frontend, bridge, events = rig(tmp_path)
    clock = {'ns': 100}
    original_move = bridge.move
    started=time.monotonic_ns()
    monkeypatch.setattr('services.piper.backend.time.time_ns', lambda: clock['ns']+time.monotonic_ns()-started)

    def move(*args):
        result = original_move(*args)
        clock['ns'] = 200
        return result

    def capture(*args, **kwargs):
        events.append('observe')
        from services.piper.installed_settling import SETTLING_WINDOW_S
        assert kwargs['after_ns'] >= int(SETTLING_WINDOW_S*1e9)+200  # after local stability, not just receipt
        raise TimeoutError('No post-action frame within 2 seconds')

    bridge.move = move
    frontend.backend.cameras.capture = capture
    try:
        reply = frontend.call('piper_gripper', {'width_m': .02, 'observe_after': True})
        result = value(reply)
        assert not reply.get('isError')
        assert result['executed'] is True and result['measured_arrival'] is True
        assert result['observation_error']['type'] == 'TimeoutError'
        assert events == ['motion', 'observe'] and len(bridge.moves) == 1
    finally:
        frontend.close()


@pytest.mark.parametrize('kind', ['unknown_motion', 'invalid_receipt'])
def test_no_post_observation_without_confirmed_motion_completion(tmp_path, kind):
    frontend, bridge, events = rig(tmp_path)

    def incomplete(*args):
        events.append('motion')
        if kind == 'unknown_motion':
            raise TimeoutError('Unknown motion outcome')
        return {'executed': True, 'measured_arrival': False}

    bridge.move = incomplete
    try:
        reply = frontend.call('piper_move_joints', {'joints': [.01, 0, 0, 0, 0, 0], 'observe_after': True})
        assert reply['isError'] is True
        assert value(reply)['error']['control_uncertain'] is True
        assert events == ['motion']
    finally:
        frontend.close()


def test_default_and_explicit_false_preserve_action_only_contract(tmp_path):
    frontend, bridge, events = rig(tmp_path)
    try:
        for options in ({}, {'observe_after': False}):
            reply = frontend.call('piper_gripper', {'width_m': .02, **options})
            assert not reply.get('isError')
            assert 'observation' not in value(reply)
            assert len(reply['content']) == 1
        assert events == ['motion', 'motion'] and len(bridge.moves) == 2
    finally:
        frontend.close()


def test_invalid_observe_option_rejected_before_motion(tmp_path):
    frontend, bridge, events = rig(tmp_path)
    try:
        reply = frontend.call('piper_gripper', {'width_m': .02, 'observe_after': 'yes'})
        assert reply['isError'] is True and value(reply)['error']['no_action_executed'] is True
        assert not events and not bridge.moves
    finally:
        frontend.close()


@pytest.mark.parametrize('earlier_unknown', [False, True])
@pytest.mark.parametrize('failure', [ValueError('No full-pose IK'), TimeoutError('Solver response timeout')])
def test_pure_ik_failure_reports_no_dispatch_without_clearing_earlier_unknown(tmp_path, earlier_unknown, failure):
    frontend, bridge, events = rig(tmp_path)
    frontend.control_uncertain = earlier_unknown

    def fail(*args):
        raise failure

    frontend.backend.solver.solve = fail
    try:
        reply = frontend.call('piper_solve_ik', {'position': [.1, 0, .2],
                                               'quaternion_xyzw': [0, 0, 0, 1], 'seed': [0.] * 6})
        error = value(reply)['error']
        assert reply['isError'] is True
        assert error['stage'] == 'validation'
        assert error['executed'] is False and error['dispatch_started'] is False
        assert 'no motion requested or dispatched' in error['reason']
        assert error['control_uncertain'] is earlier_unknown
        assert error['no_action_executed'] is not earlier_unknown
        assert not events and not bridge.moves
    finally:
        frontend.close()
