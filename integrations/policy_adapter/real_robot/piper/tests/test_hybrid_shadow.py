"""One-shot orchestration contracts with fake owner/student/teacher; no network."""
import asyncio
from copy import deepcopy
import json
from types import SimpleNamespace
import time

import numpy as np
import pytest

from real_robot.piper import hybrid_shadow as h
from real_robot.piper.contracts import (CAMERA_ROLES, FIELDS, UNITS, STATE_SCHEMA_ID,
    PiperObservation, ValidatedCandidate, vector_schema)
from real_robot.piper.backend import array_sha256
from real_robot.piper.teacher_client import BOTTLE_TASK, STUDENT_TASK


def observation():
    return PiperObservation(np.zeros(14, np.float32),
        {role: np.zeros((32, 64, 3), np.uint8) for role in CAMERA_ROLES},
        STUDENT_TASK, time.time_ns(), 7, 'test_robot', {'robot_id': 'test_robot'}, 1., {})


class Owner:
    def __init__(self, fault=None): self.count, self.fault = 0, fault
    def snapshot(self):
        self.count += 1
        value = {'task': STUDENT_TASK, 'control_source': 'keyboard', 'write_enabled': False,
            'pigey': {'active': False}, 'robot_state': {'available': True, 'names': list(FIELDS),
                'schema_id': STATE_SCHEMA_ID, 'units': list(UNITS), 'values': [0.] * 14},
            'robot_status': {'hardware_write_enabled': False, 'hardware_write_count': 0,
                'status': 'READY_NO_WRITE', 'generation': 4, 'expert_candidates': 0,
                'observations': self.count}}
        if self.fault == 'writes': value['robot_status']['hardware_write_count'] = 1
        if self.count == 3:
            if self.fault == 'generation': value['robot_status']['generation'] = 5
            if self.fault == 'motion': value['robot_state']['values'][0] = .002
            if self.fault == 'counter': value['robot_status']['observations'] = 0
        return value


class Contract:
    def validate_candidate(self, actions, **kwargs):
        assert kwargs['schema'] == vector_schema(action=True)
        return ValidatedCandidate(actions.copy(), 30., 16, {'robot_id': 'test_robot'}, {})


class Backend:
    task = STUDENT_TASK
    identity = {'test': 'student'}
    description = None
    contract = Contract()
    def __init__(self):
        self.calls, self.resets = 0, 0
        self.student = self
        self.raw = np.full((32, 14), -.001, np.float32)
    def observe(self, obs):
        assert (time.time_ns() - obs.timestamp_ns) / 1e6 <= 300
        return obs
    async def reset(self, **kwargs): self.resets += 1
    async def propose(self, obs):
        self.calls += 1
        return {'actions': self.raw, 'latency_ms': 1., '_gap_binding': {'request_id': 'student'}}
    def diagnostics(self, obs, raw):
        return ({'raw_sha256': array_sha256(raw), 'owner_boundary_adjustments': [{'step': 0}],
                 'current_flange': {'coordinate_frame': 'dual_base_link'},
                 'student_flange_trajectory': [{'step': i} for i in range(32)]},
                np.zeros((32, 14), np.float64))
    def execute(self, *args, **kwargs): raise AssertionError('must never submit nonempty action')


class Teacher:
    def __init__(self, callback=None, fail=False): self.calls, self.callback, self.fail = [], callback, fail
    def review(self, **kwargs):
        self.calls.append(kwargs)
        if self.callback: self.callback()
        if self.fail: raise RuntimeError('offline fixture failure')
        return {'decision': {'binding': kwargs['binding'], 'mode': 'stop', 'reason': 'uncertain'},
                'api_decision': {}, 'report': {'returned_model': 'test-no-api'}}


class Reviewer:
    def __init__(self, reject=False): self.calls, self.reject = [], reject
    def review(self, obs, proposal, decision, **kwargs):
        self.calls.append((obs, proposal, decision, kwargs))
        assert kwargs['envelope'].now_ns >= obs.timestamp_ns
        assert kwargs['envelope'].max_observation_age_ms == 60000
        envelope = kwargs['candidate_envelope']
        assert envelope.max_joint_step_rad == .03
        assert envelope.max_joint_delta_from_measured_rad == .10
        assert envelope.max_gripper_delta_m == .02
        if self.reject: raise ValueError('candidate exceeds declared max_joint_step_rad')
        return SimpleNamespace(mode='stop', candidate=None, assessment_gate='none', metadata={})


def make(tmp_path, *, owner=None, teacher=None, reviewer=None):
    backend = Backend()
    model = h.HybridShadowModel(backend, owner or Owner(), reviewer or Reviewer(),
                                teacher or Teacher(), output=tmp_path)
    return model


def assert_empty(result):
    assert result['result_kind'] == 'no_action'
    assert result['actions'].shape == (0, 14) and result['actions'].dtype == np.float32
    assert result['action_schema'] == vector_schema(action=True)
    assert result['action_frequency_hz'] == 30
    assert result['policy_metadata']['transport_receipt']['dispatch_authorized'] is False


def test_one_shot_nonblocking_reset_never_rearms_and_raw_effective_binding(tmp_path):
    model = make(tmp_path)
    obs = observation()
    async def scenario():
        first = await model.infer(obs)
        assert_empty(first)
        assert not model.pending.done() and model.backend.calls == 0
        assert first['policy_metadata']['inference_executed'] is False
        for result in await asyncio.gather(*[model.infer(obs) for _ in range(4)]): assert_empty(result)
        await model.reset()
        await model.pending
        await model.reset()
        assert_empty(await model.infer(obs))
    asyncio.run(scenario())
    assert model.backend.calls == len(model.teacher.calls) == 1
    assert model.backend.resets == 0
    call = model.teacher.calls[0]
    assert call['context']['user_goal'] == BOTTLE_TASK
    assert call['context']['task'] == call['binding']['task'] == STUDENT_TASK
    assert call['context']['executed_steps'] == 0
    assert len(call['context']['student_actions']) == len(call['context']['student_flange_trajectory']) == 32
    norm = call['context']['normalization_receipt']
    assert norm['raw_actions_sha256'] != norm['effective_preview_sha256']
    assert norm['review_proposal_sha256'] == call['binding']['proposal_sha256']
    stored = np.load(tmp_path / 'actions.npz')
    np.testing.assert_array_equal(stored['raw_actions'], model.backend.raw)
    assert stored['owner_boundary_preview'].dtype == np.float64
    result = json.loads((tmp_path / 'teacher-result.json').read_text())
    assert result['review_validation']['status'] == 'accepted_offline_review'
    assert result['review_validation']['candidate_present'] is False
    assert result['observation_timestamp_ns'] == obs.timestamp_ns
    assert result['dispatch_authorized'] is False


@pytest.mark.parametrize('fault', ['generation', 'motion', 'counter'])
def test_changed_owner_after_api_is_hold_with_decision_preserved(tmp_path, fault):
    model = make(tmp_path, owner=Owner(fault))
    async def scenario():
        assert_empty(await model.infer(observation()))
        await model.pending
    asyncio.run(scenario())
    result = json.loads((tmp_path / 'teacher-result.json').read_text())
    assert result['decision']['mode'] == 'stop'
    assert result['review_validation']['status'] == 'HOLD'
    assert len(model.teacher.calls) == 1 and not model.reviewer.calls
    assert result['dispatch_authorized'] is False


def test_owner_writes_rejected_before_student_or_teacher(tmp_path):
    model = make(tmp_path, owner=Owner('writes'))
    async def scenario():
        assert_empty(await model.infer(observation()))
        await model.pending
        assert_empty(await model.infer(observation()))
    asyncio.run(scenario())
    result = json.loads((tmp_path / 'teacher-error.json').read_text())
    assert result['teacher_called'] is False
    assert len(model.teacher.calls) == model.backend.calls == 0


def test_api_failure_consumes_budget_and_returns_only_no_action(tmp_path):
    model = make(tmp_path, teacher=Teacher(fail=True))
    async def scenario():
        assert_empty(await model.infer(observation()))
        await model.pending
        await model.reset()
        assert_empty(await model.infer(observation()))
    asyncio.run(scenario())
    assert len(model.teacher.calls) == 1
    error = json.loads((tmp_path / 'teacher-error.json').read_text())
    assert error['phase'] == 'teacher_api' and error['dispatch_authorized'] is False


def test_old_review_timestamp_never_relabelled_execution_fresh(tmp_path, monkeypatch):
    clock = [1789690000000000000]
    monkeypatch.setattr(h.time, 'time_ns', lambda: clock[0])
    teacher = Teacher(callback=lambda: clock.__setitem__(0, clock[0] + 2_000_000_000))
    model = make(tmp_path, teacher=teacher)
    obs = observation()
    async def scenario():
        assert_empty(await model.infer(obs))
        await model.pending
    asyncio.run(scenario())
    result = json.loads((tmp_path / 'teacher-result.json').read_text())
    assert result['observation_timestamp_ns'] == obs.timestamp_ns
    assert result['observation_age_at_review_ms'] == 2000
    assert result['execution_observation_stale'] is True
    assert result['review_scope'] == 'historical stationary shadow review only'


def test_reviewer_envelope_failure_is_hold_not_student_fallback(tmp_path):
    model = make(tmp_path, reviewer=Reviewer(reject=True))
    async def scenario():
        assert_empty(await model.infer(observation()))
        await model.pending
    asyncio.run(scenario())
    result = json.loads((tmp_path / 'teacher-result.json').read_text())
    assert result['review_validation']['status'] == 'HOLD'
    assert 'max_joint_step' in result['review_validation']['error']
    assert not (tmp_path / 'reviewed_candidate_preview.npy').exists()


def test_reject_stale_input_reserves_budget_without_api(tmp_path):
    model = make(tmp_path)
    obs = observation()
    old = PiperObservation(obs.state14, obs.images, obs.instruction,
                           obs.timestamp_ns - 2_000_000_000, obs.sequence,
                           obs.robot_id, obs.identity, 2000., obs.metadata)
    async def scenario():
        assert_empty(await model.infer(old))
        assert_empty(await model.infer(obs))
    asyncio.run(scenario())
    assert model.pending is None and len(model.teacher.calls) == 0


def test_wrong_user_goal_fails_before_start(tmp_path):
    with pytest.raises(ValueError, match='pins separate'):
        h.HybridShadowModel(Backend(), Owner(), Reviewer(), Teacher(), output=tmp_path,
                            user_goal='Place on coaster instead')


def test_invalid_normalized_candidate_does_not_call_teacher_or_silently_clip(tmp_path):
    model = make(tmp_path)
    def invalid(*args, **kwargs): raise ValueError('float32 transport crosses command limits')
    model.backend.contract = SimpleNamespace(validate_candidate=invalid)
    async def scenario():
        assert_empty(await model.infer(observation()))
        await model.pending
    asyncio.run(scenario())
    error = json.loads((tmp_path / 'teacher-error.json').read_text())
    assert error['phase'] == 'proposal_validation' and error['teacher_called'] is False
    assert len(model.teacher.calls) == 0
    assert (tmp_path / 'actions.npz').is_file()


def test_multi_step_teacher_correction_rejected_before_reviewer(tmp_path):
    class MultiStepTeacher(Teacher):
        def review(self, **kwargs):
            result = super().review(**kwargs)
            result['decision'].update(mode='edit', steps=2)
            return result
    model = make(tmp_path, teacher=MultiStepTeacher())
    async def scenario():
        assert_empty(await model.infer(observation()))
        await model.pending
    asyncio.run(scenario())
    result = json.loads((tmp_path / 'teacher-result.json').read_text())
    assert result['review_validation']['status'] == 'HOLD'
    assert 'exactly one step' in result['review_validation']['error']
    assert not model.reviewer.calls


def test_waits_for_published_owner_counter_without_weakening_stationarity(tmp_path):
    class CachedOwner(Owner):
        def snapshot(self):
            result = super().snapshot()
            result['robot_status']['observations'] = max(1, self.count - 2)
            return result
    model = make(tmp_path, owner=CachedOwner())
    async def scenario():
        await model.infer(observation())
        await model.pending
    asyncio.run(scenario())
    assert len(model.teacher.calls) == 1
    result = json.loads((tmp_path / 'teacher-result.json').read_text())
    assert result['review_validation']['status'] == 'accepted_offline_review'


def test_counter_that_never_advances_times_out_without_teacher(tmp_path):
    class StuckOwner(Owner):
        def snapshot(self):
            result = super().snapshot()
            result['robot_status']['observations'] = 1
            return result
    model = make(tmp_path, owner=StuckOwner())
    async def scenario():
        before = model.owner.snapshot()
        with pytest.raises(ValueError, match='did not advance'):
            await model._advanced_snapshot('bounded-test', before, timeout_s=.03)
    asyncio.run(scenario())
    assert not model.teacher.calls
