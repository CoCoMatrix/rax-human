"""Actual Frame/profile/Mink with a mock student; no sockets, GPU or hardware."""
import asyncio
from copy import deepcopy
import json
import os
import time
from types import SimpleNamespace

import numpy as np
import pytest

from real_robot.piper.backend import OwnerObserver, PiperXPolicyBackend, array_sha256
from real_robot.piper.contracts import (CAMERA_ROLES, ContractError, PiperContract,
                                       runtime_module, vector_schema)
from real_robot.piper.serve import ShadowModel


TASK = "Place the cup."


def owner_state():
    return {"task": TASK, "control_source": "keyboard", "write_enabled": False,
            "pigey": {"active": False}, "robot_status": {
                "hardware_write_enabled": False, "hardware_write_count": 0,
                "status": "READY_NO_WRITE", "generation": 4, "expert_candidates": 0}}


def state14():
    return np.asarray([0.] * 6 + [0.0003] + [0.] * 6 + [0.0004], dtype=np.float32)


class Student:
    def __init__(self):
        self.actions = np.tile(state14(), (32, 1))
        self.payload = {"actions": self.actions, "action_schema": vector_schema(action=True),
                        "action_frequency_hz": 30, "latency_ms": 1.0}
        self.inputs = []
        self.reset_calls = []

    async def infer(self, observation):
        self.inputs.append(observation)
        return SimpleNamespace(payload=self.payload)

    async def reset(self, **kwargs):
        self.reset_calls.append(kwargs)


@pytest.fixture(scope="module")
def contract():
    root = os.environ.get("PIPER_RUNTIME_ROOT")
    if root is None:
        pytest.skip("requires authoritative runtime assets")
    pytest.importorskip("mink")
    return PiperContract.from_runtime(root)


@pytest.fixture
def backend(contract, tmp_path):
    return PiperXPolicyBackend(contract, Student(), tmp_path / "shadow", task=TASK,
                               identity={"checkpoint": "offline_test_fixture"})


def frame(contract, *, sequence=5):
    types = runtime_module(contract.repo_root, "real_world.contracts")
    schema = runtime_module(contract.repo_root, "real_world.robots.piper_x_dual.mapping").PIPER_X_DUAL_STATE_SCHEMA
    return types.ObservationFrame(
        state_schema=schema, state=state14(),
        images={role: np.full((720, 1280, 3), [12, 34, 56], dtype=np.uint8) for role in CAMERA_ROLES},
        instruction=TASK, timestamp_ns=time.time_ns(), sequence=sequence,
        robot_type="piper_x_dual", robot_id="piper_x_dual_01")


def proposed(backend):
    obs = backend.observe(frame(backend.contract))
    payload = asyncio.run(backend.propose(obs))
    return obs, payload


def test_actual_frame_mock_student_formal_payload_keeps_raw_and_effective_distinct(backend):
    backend.student.actions[:, 1] = -0.001  # Owner-defined small-joint snap.
    backend.student.actions[:, 6] = -0.002  # Owner-defined gripper clamp.
    expected = backend.student.actions.copy()
    obs, payload = proposed(backend)
    result = backend.execute(obs, payload, owner_before=owner_state(), owner_after=owner_state())
    np.testing.assert_array_equal(result["actions"], expected)
    np.testing.assert_array_equal(backend.student.inputs[0]["state"]["vector"], obs.state14)
    for role in CAMERA_ROLES:
        np.testing.assert_array_equal(backend.student.inputs[0]["vision"][role]["color"], obs.images[role])
    assert result["actions"].dtype == np.float32
    assert result["action_schema"] == vector_schema(action=True)
    assert result["action_frequency_hz"] == 30
    assert result["policy_metadata"]["hardware_execution_confirmed"] is False
    assert result["policy_metadata"]["gap_mode"] == "pi05_only_shadow"
    assert "_gap_binding" not in result
    saved = np.load(backend.output / "proposal_0001/actions.npz")
    np.testing.assert_array_equal(saved["raw_actions"], expected)
    effective = saved["owner_boundary_preview"]
    assert effective.dtype == np.float64
    assert np.all(effective[:, 1] == 0) and np.all(effective[:, 6] == 0)
    record = json.loads((backend.output / "proposal_0001/proposal.json").read_text())
    assert record["raw_command_valid"] is False
    assert record["effective_preview_exact_limits_valid"] is True
    assert record["raw_sha256"] == array_sha256(expected)
    assert record["owner_boundary_adjustments"]
    assert record["hardware_execution_confirmed"] is False
    assert record["normalization_is_preview_only"] is True


@pytest.mark.parametrize("fault,match", [
    ("ui_write", "UI write"), ("owner_write", "owner write"),
    ("write_count", "hardware write"), ("readiness", "no-write shadow"),
    ("generation_missing", "generation"), ("generation_bool", "generation"),
    ("expert", "expert"), ("pigey", "Pigey"), ("task", "task"),
])
def test_owner_observer_rejects_unsafe_shadow_state(fault, match):
    owner = owner_state()
    if fault == "ui_write": owner["write_enabled"] = True
    elif fault == "owner_write": owner["robot_status"]["hardware_write_enabled"] = True
    elif fault == "write_count": owner["robot_status"]["hardware_write_count"] = 1
    elif fault == "readiness": owner["robot_status"]["status"] = "READY_LIVE_TAKEOVER"
    elif fault == "generation_missing": owner["robot_status"].pop("generation")
    elif fault == "generation_bool": owner["robot_status"]["generation"] = True
    elif fault == "expert": owner["robot_status"]["expert_candidates"] = 1
    elif fault == "pigey": owner["pigey"]["active"] = True
    elif fault == "task": owner["task"] = "Wrong task"
    with pytest.raises(ContractError, match=match):
        OwnerObserver.validate(owner, TASK)


def test_execute_independently_checks_write_gate_and_generation(backend):
    obs, payload = proposed(backend)
    owner = owner_state()
    owner["write_enabled"] = True
    with pytest.raises(ContractError, match="UI write"):
        backend.execute(obs, payload, owner_before=owner_state(), owner_after=owner)
    owner = owner_state()
    owner["robot_status"]["generation"] += 1
    with pytest.raises(ContractError, match="generation"):
        backend.execute(obs, payload, owner_before=owner_state(), owner_after=owner)
    assert backend.request_count == 0


@pytest.mark.parametrize("fault,match", [("shape", "32,14"), ("dtype", "float32"),
                                        ("nan", "finite"), ("schema", "schema"), ("frequency", "frequency")])
def test_bad_student_output_never_becomes_submission(backend, fault, match):
    if fault == "shape": backend.student.payload["actions"] = np.zeros((32, 32), dtype=np.float32)
    elif fault == "dtype": backend.student.payload["actions"] = backend.student.actions.astype(np.float64)
    elif fault == "nan": backend.student.actions[0, 0] = np.nan
    elif fault == "schema": backend.student.payload["action_schema"]["units"][6] = "normalized"
    elif fault == "frequency": backend.student.payload["action_frequency_hz"] = 20
    with pytest.raises(ContractError, match=match):
        proposed(backend)
    assert backend.request_count == 0


def test_replay_and_modified_action_are_rejected(backend):
    obs, payload = proposed(backend)
    altered = dict(payload, actions=payload["actions"].copy())
    altered["actions"][0, 0] = 0.001
    with pytest.raises(ContractError, match="changed"):
        backend.execute(obs, altered, owner_before=owner_state(), owner_after=owner_state())
    backend.execute(obs, payload, owner_before=owner_state(), owner_after=owner_state())
    with pytest.raises(ContractError, match="replay|already submitted"):
        backend.execute(obs, payload, owner_before=owner_state(), owner_after=owner_state())
    assert backend.request_count == 1


@pytest.mark.parametrize("field", ["action_schema", "action_frequency_hz"])
def test_execute_rechecks_schema_and_cadence_binding(backend, field):
    obs, payload = proposed(backend)
    if field == "action_schema":
        payload[field] = deepcopy(payload[field])
        payload[field]["semantics"][0] = "delta_joint"
    else:
        payload[field] = 20
    with pytest.raises(ContractError, match="schema or frequency changed"):
        backend.execute(obs, payload, owner_before=owner_state(), owner_after=owner_state())
    assert backend.request_count == 0


def test_shadow_model_checks_owner_before_and_after_inference(backend):
    class Owner:
        def __init__(self): self.calls = 0
        def snapshot(self):
            self.calls += 1
            value = owner_state()
            if self.calls == 2:
                value["robot_status"]["hardware_write_count"] = 1
            OwnerObserver.validate(value, TASK)
            return value
    owner = Owner()
    model = ShadowModel(backend, owner)
    with pytest.raises(ContractError, match="hardware write"):
        asyncio.run(model.infer(frame(backend.contract)))
    assert owner.calls == 2 and backend.request_count == 0


def test_reset_is_student_cache_only(backend):
    model = ShadowModel(backend, owner=None)
    assert asyncio.run(model.reset()) == {"physical_reset": False}
    assert backend.student.reset_calls == [{"trial_id": "junqi-piper-shadow"}]
    assert model.get_capabilities()["hardware_write_allowed"] is False


def test_historical_replay_mode_cannot_infer_or_submit(backend):
    backend.mode = 'offline_replay'
    with pytest.raises(ContractError, match='offline replay cannot request'):
        asyncio.run(backend.propose(None))
    with pytest.raises(ContractError, match='offline replay cannot submit'):
        backend.execute(None, {}, owner_before={}, owner_after={})
    assert backend.request_count == 0
