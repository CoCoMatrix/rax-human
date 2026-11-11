from copy import deepcopy
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from real_robot.piper.contracts import (CAMERA_ROLES, CandidateEnvelope, ContractError,
                                       PiperContract, runtime_module, vector_schema)

NOW = 1_000_000_000


@pytest.fixture(scope="module")
def contract():
    root = os.environ.get("PIPER_RUNTIME_ROOT")
    if root is None:
        pytest.skip("PIPER_RUNTIME_ROOT is required for authoritative profile tests")
    return PiperContract.from_runtime(root)


def wire():
    return {"data_format_version": "xpolicylab.real_world.v1", "instruction": "Place the cup.",
            "state": {"vector": np.array([0.] * 6 + [0.0003] + [0.] * 6 + [0.0004], dtype=np.float32),
                      "schema": vector_schema()},
            "vision": {role: {"color": np.arange(36, dtype=np.uint8).reshape(3, 4, 3)}
                       for role in CAMERA_ROLES},
            "additional_info": {"timestamp_ns": NOW, "sequence": 4, "robot_type": "piper_x_dual",
                                "robot_id": "piper_x_dual_01", "metadata": {
                                    "inference_context": {"queued_actions": np.zeros((2, 14), dtype=np.float32),
                                                          "execution_step": 7}}}}


def test_pure_modules_do_not_import_kinematic_or_hardware_dependencies():
    code = ("import sys; import real_robot.piper.contracts; import real_robot.piper.kinematics; "
            "assert not any(n == 'mink' or n == 'mujoco' or n == 'torch' or "
            "'pyagxarm' in n.lower() or n.startswith('dora') for n in sys.modules)")
    subprocess.run([sys.executable, "-c", code], check=True)


def test_observation_preserves_values_rgb_units_and_inference_context(contract):
    source = wire()
    result = contract.validate_observation(source, max_age_ms=100, now_ns=NOW + 50_000_000)
    np.testing.assert_array_equal(result.state14, source["state"]["vector"])
    assert result.state14.dtype == np.float32 and not result.state14.flags.writeable
    assert result.age_ms == 50.0
    source["vision"]["camera_head"]["color"][:] = 0
    assert result.images["camera_head"][0, 0].tolist() == [0, 1, 2]
    assert not result.images["camera_head"].flags.writeable
    roundtrip = result.to_policy_observation()
    np.testing.assert_array_equal(roundtrip["additional_info"]["metadata"]["inference_context"]["queued_actions"],
                                  source["additional_info"]["metadata"]["inference_context"]["queued_actions"])
    assert roundtrip["state"]["schema"] == vector_schema()
    assert roundtrip["vision"]["camera_head"]["encoding"] == "rgb8"


def test_real_observation_frame_is_accepted(contract):
    frame_type = runtime_module(contract.repo_root, "real_world.contracts").ObservationFrame
    frame = frame_type.from_policy_observation(wire())
    result = contract.validate_observation(frame, max_age_ms=100, now_ns=NOW)
    assert result.sequence == 4 and result.robot_id == "piper_x_dual_01"


@pytest.mark.parametrize("fault,match", [
    ("arx", "identity"), ("wrong_robot_id", "identity"),
    ("wrong_units", "schema"), ("reordered", "schema"),
    ("relative", "schema"), ("flattened_state", "shape"),
    ("list_state", "float32"), ("float64_wire", "float32"),
    ("nan", "NaN"), ("missing_camera", "three"),
    ("extra_camera", "three"), ("bgr", "rgb8"),
    ("flattened_image", "HxWx3"), ("float_image", "HxWx3"),
    ("empty_image", "HxWx3"), ("stale", "stale"),
    ("future", "future"), ("blank_task", "nonempty"),
    ("fake_profile", "backend identity"),
])
def test_observation_rejects_mismatches(contract, fault, match):
    value = wire()
    if fault == "arx": value["additional_info"]["robot_type"] = "arx_x5"
    elif fault == "wrong_robot_id": value["additional_info"]["robot_id"] = "piper_x_dual_02"
    elif fault == "wrong_units": value["state"]["schema"]["units"][6] = "normalized"
    elif fault == "reordered": value["state"]["schema"]["fields"] = value["state"]["schema"]["fields"][::-1]
    elif fault == "relative": value["state"]["schema"]["semantics"][0] = "delta_joint"
    elif fault == "flattened_state": value["state"]["vector"] = value["state"]["vector"][None]
    elif fault == "list_state": value["state"]["vector"] = value["state"]["vector"].tolist()
    elif fault == "float64_wire": value["state"]["vector"] = value["state"]["vector"].astype(np.float64)
    elif fault == "nan": value["state"]["vector"][0] = np.nan
    elif fault == "missing_camera": value["vision"].pop("camera_left")
    elif fault == "extra_camera": value["vision"]["arx_front"] = value["vision"]["camera_head"]
    elif fault == "bgr": value["vision"]["camera_head"]["encoding"] = "bgr8"
    elif fault == "flattened_image": value["vision"]["camera_head"]["color"] = np.zeros(36, dtype=np.uint8)
    elif fault == "float_image": value["vision"]["camera_head"]["color"] = np.zeros((3, 4, 3), dtype=np.float32)
    elif fault == "empty_image": value["vision"]["camera_head"]["color"] = np.zeros((0, 4, 3), dtype=np.uint8)
    elif fault == "stale": value["additional_info"]["timestamp_ns"] = NOW - 101_000_000
    elif fault == "future": value["additional_info"]["timestamp_ns"] = NOW + 1
    elif fault == "blank_task": value["instruction"] = " "
    elif fault == "fake_profile": value["additional_info"]["metadata"]["piper_backend_identity"] = {}
    with pytest.raises(ContractError, match=match):
        contract.validate_observation(value, max_age_ms=100, now_ns=NOW)


def test_repeated_sequence_rejected(contract):
    with pytest.raises(ContractError, match="advance"):
        contract.validate_observation(wire(), max_age_ms=100, now_ns=NOW, previous_sequence=4)


def test_measurement_noise_is_preserved_but_not_a_command(contract):
    state = wire()["state"]["vector"]
    state[6] = -0.0001
    np.testing.assert_array_equal(contract.validate_measured_state(state), state)
    with pytest.raises(ContractError, match="command limits"):
        contract.validate_command_state(state)


def candidate(contract):
    state = wire()["state"]["vector"]
    values = np.tile(state, (32, 1))
    values[:, 0] = np.arange(32, dtype=np.float32) * 0.0001
    return state, values


def test_candidate_keeps_full_horizon_and_exact_values(contract):
    state, values = candidate(contract)
    checked = contract.validate_candidate(values, schema=vector_schema(action=True), frequency_hz=30,
                                          measured_state14=state, prefix_steps=16)
    output = checked.to_policy_result()
    np.testing.assert_array_equal(output["actions"], values)
    assert output["actions"].shape == (32, 14) and not output["actions"].flags.writeable
    assert output["action_schema"]["units"][6] == "m"
    assert checked.prefix_steps == 16


@pytest.mark.parametrize("fault,match", [
    ("32d_padding", "N,14"), ("flatten", "N,14"), ("empty", "horizon"),
    ("long", "horizon"), ("frequency", "30 Hz"), ("schema", "schema"),
    ("nan", "NaN"), ("gripper", "command limits"), ("joint", "command limits"),
    ("prefix", "prefix"), ("delta", "max_joint_delta"),
    ("velocity", "max_joint_velocity"), ("acceleration", "max_joint_acceleration"),
])
def test_candidate_fail_closed(contract, fault, match):
    state, values = candidate(contract)
    schema = vector_schema(action=True)
    hz, prefix, envelope = 30, 16, None
    if fault == "32d_padding": values = np.zeros((32, 32), dtype=np.float32)
    elif fault == "flatten": values = values.flatten()
    elif fault == "empty": values = values[:0]
    elif fault == "long": values = np.tile(state, (33, 1))
    elif fault == "frequency": hz = 20
    elif fault == "schema": schema["units"][6] = "fraction"
    elif fault == "nan": values[3, 0] = np.inf
    elif fault == "gripper": values[3, 6] = 0.11
    elif fault == "joint": values[3, 1] = -0.0001
    elif fault == "prefix": prefix = 17
    elif fault == "delta": envelope = CandidateEnvelope(max_joint_delta_from_measured_rad=0.001)
    elif fault == "velocity": envelope = CandidateEnvelope(max_joint_velocity_rad_s=0.0001)
    elif fault == "acceleration": envelope = CandidateEnvelope(max_joint_acceleration_rad_s2=0.0001)
    with pytest.raises(ContractError, match=match):
        contract.validate_candidate(values, schema=schema, frequency_hz=hz,
                                    measured_state14=state, prefix_steps=prefix, envelope=envelope)


def test_pinned_source_identity_rejects_drift(contract):
    with pytest.raises(ContractError, match="pinned profile"):
        PiperContract.from_runtime(contract.repo_root, expected_profile_sha256="0" * 64)


def test_declared_envelope_applies_first_step_from_measured_state(contract):
    state, values = candidate(contract)
    values[:, 0] = 0.01
    with pytest.raises(ContractError, match="max_joint_velocity"):
        contract.validate_candidate(values, schema=vector_schema(action=True), frequency_hz=30,
                                    measured_state14=state,
                                    envelope=CandidateEnvelope(max_joint_velocity_rad_s=0.15))


def test_float32_quantization_cannot_bypass_declared_envelope(contract):
    state, values = candidate(contract)
    values = values.astype(np.float64)
    value64 = 0.1
    value32 = float(np.float32(value64))
    assert value32 > value64
    values[:, 0] = value64
    limit = (value64 + value32) / 2
    with pytest.raises(ContractError, match="max_joint_delta"):
        contract.validate_candidate(values, schema=vector_schema(action=True), frequency_hz=30,
                                    measured_state14=state,
                                    envelope=CandidateEnvelope(max_joint_delta_from_measured_rad=limit))


def test_effective_owner_boundary_is_not_a_transport_candidate(contract):
    # The owner clamps raw float32 predictions after decoding, yielding exact
    # Python-float bounds. Recasting that preview would add a second rounding.
    measured = wire()["state"]["vector"]
    effective = measured.astype(np.float64)
    effective[6] = 0.1
    contract.validate_command_state(effective)
    assert float(np.float32(effective[6])) > 0.1
    with pytest.raises(ContractError, match="float32 transport"):
        contract.validate_candidate(np.tile(effective, (32, 1)), schema=vector_schema(action=True),
                                    frequency_hz=30, measured_state14=measured)


@pytest.mark.parametrize("value", [0, -1, np.nan, np.inf, True])
def test_envelope_invalid_configuration(value):
    with pytest.raises(ContractError):
        CandidateEnvelope(max_joint_step_rad=value)
