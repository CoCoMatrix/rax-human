"""Only an authenticated solver no-solution may invite read-only replanning.

Exercise the real solver parser, backend wrapping and public MCP error together;
replace only the external solver pipe and hardware transport for offline tests.
"""
import json
from types import SimpleNamespace

import pytest

from test_piper_action_observation import rig, value


def solver_reply(monkeypatch, solver, *, error='MOVEIT_NO_IK_SOLUTION', bad_id=False):
    requests = []
    stdin = SimpleNamespace(write=lambda data: requests.append(json.loads(data)), flush=lambda: None)
    monkeypatch.setattr(solver, 'start', lambda: None)
    monkeypatch.setattr(solver, 'process', SimpleNamespace(stdin=stdin))
    monkeypatch.setattr(solver, 'close', lambda: None)
    monkeypatch.setattr(solver, '_read', lambda timeout: {
        'id': 'bad' if bad_id else requests[-1]['id'], 'ok': False,
        'error': error, 'hardware_writes': 0, 'error_code': -31,
        'solver': 'moveit2_kdl', 'elapsed_s': .1})
    return requests


@pytest.mark.parametrize('tool', ['piper_solve_ik', 'piper_move_pose'])
def test_no_solution_explicitly_allows_read_only_reselection_without_dispatch(tmp_path, monkeypatch, tool):
    f, bridge, _ = rig(tmp_path)
    requests = solver_reply(monkeypatch, f.backend.solver)
    pose = {'position': [.096638, 0, .246805], 'quaternion_xyzw': [0, 1, 0, 0]}
    try:
        error = value(f.call(tool, pose))['error']
        assert error.get('recovery_action') == 'replan_read_only'
        assert error['dispatch_started'] is False and error['no_action_executed']
        assert error['control_uncertain'] is False and not bridge.moves
        # No automatic retry or relaxation of the solver timeout.
        assert len(requests) == 1 and requests[0]['timeout_s'] == .1
    finally:
        f.close()


@pytest.mark.parametrize('tool', ['piper_solve_ik', 'piper_move_pose'])
def test_no_solution_never_clears_previous_unknown_motion(tmp_path, monkeypatch, tool):
    f, bridge, _ = rig(tmp_path)
    solver_reply(monkeypatch, f.backend.solver)
    f.control_uncertain = True
    try:
        error = value(f.call(tool, {'position': [.096638, 0, .246805],
                                   'quaternion_xyzw': [0, 1, 0, 0]}))['error']
        assert error['control_uncertain'] and not error['no_action_executed']
        assert error.get('recovery_action') != 'replan_read_only'
        assert not bridge.moves
    finally:
        f.close()


@pytest.mark.parametrize('error,bad_id', [('MOVEIT_NO_IK_SOLUTION', True),
    ('IK_RESIDUAL_OR_LIMIT', False), ('MODEL_FILE', False)])
def test_solver_protocol_and_verification_failures_are_not_planning_choices(tmp_path, monkeypatch, error, bad_id):
    f, bridge, _ = rig(tmp_path)
    solver_reply(monkeypatch, f.backend.solver, error=error, bad_id=bad_id)
    try:
        result = value(f.call('piper_solve_ik', {'position': [.1, 0, .25],
                                                'quaternion_xyzw': [0, 1, 0, 0]}))['error']
        assert result.get('recovery_action') != 'replan_read_only'
        assert not bridge.moves
    finally:
        f.close()


def test_owner_failure_with_same_text_is_not_misclassified(tmp_path, monkeypatch):
    f, bridge, _ = rig(tmp_path)
    def bad_feedback():
        raise ValueError('MOVEIT_NO_IK_SOLUTION')
    monkeypatch.setattr(f.backend.owner, 'state', bad_feedback)
    try:
        result = value(f.call('piper_move_pose', {'position': [.1, 0, .25]}))['error']
        assert result.get('recovery_action') != 'replan_read_only'
        assert not bridge.moves
    finally:
        f.close()


def test_actual_arrival_timeout_never_allows_candidate_reselection(tmp_path):
    f, bridge, _ = rig(tmp_path)
    def failed_motion(q, duration):
        raise RuntimeError('motion arrival timeout')
    bridge.move = failed_motion
    try:
        result = value(f.call('piper_move_joints', {'joints': [0, 0, 0, .005, 0, 0]}))['error']
        assert result['control_uncertain'] and not result['no_action_executed']
        assert result.get('recovery_action') != 'replan_read_only'
        assert result['robot_state']['motion_failure']['max_error_joint'] == 'left_j4'
    finally:
        f.close()
