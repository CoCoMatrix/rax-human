import json
import math
import os

import numpy as np
import pytest

from real_robot.piper.contracts import ContractError, FIELDS, PiperContract
from real_robot.piper.description import MAST_JOINT, PiperDescription
from real_robot.piper.kinematics import PiperKinematics


@pytest.fixture(scope="module")
def description():
    runtime, asset = os.environ.get("PIPER_RUNTIME_ROOT"), os.environ.get("PIPER_DESCRIPTION_ASSET_ROOT")
    if not runtime or not asset:
        pytest.skip("set PIPER_RUNTIME_ROOT and PIPER_DESCRIPTION_ASSET_ROOT for actual asset tests")
    return PiperDescription(PiperContract.from_runtime(runtime), asset)


def zero_state():
    state = np.zeros(14)
    state[6], state[13] = 0.04, 0.08
    return state


def test_mapping_exact14_order_mimics_and_no_implicit_mast(description):
    result = description.joint_configuration(zero_state(), mast_joint_rad=None)
    assert list(result["actuated"]) == list(FIELDS)
    assert MAST_JOINT not in result["actuated"]
    assert result["mimic"] == {"left_gripper_joint1": 0.02, "left_gripper_joint2": -0.02,
                                "right_gripper_joint1": 0.04, "right_gripper_joint2": -0.04}


def test_fixed_head_default_equals_zero_and_rejects_old_extra_angle(description):
    default = description.frames(zero_state(), mast_joint_rad=None)
    explicit = description.frames(zero_state(), mast_joint_rad=0.0)
    assert default['rgb_optical_frames'] == explicit['rgb_optical_frames']
    assert default['mast']['joint_rad'] == 0.0
    assert not default['camera_extrinsics_calibrated']
    with pytest.raises(ContractError, match='fixed'):
        description.frames(zero_state(), mast_joint_rad=math.radians(25))


def test_fixed_75_degree_mast_nominal_rgb_pose(description):
    result = description.frames(zero_state(), mast_joint_rad=0.0)
    head = result["rgb_optical_frames"]["camera_head"]
    np.testing.assert_allclose(head["position_m"], [0.280118489624, 0.015, 0.723057523080], atol=2e-9)
    assert abs(result["mast"]["total_forward_tilt_rad"] - math.radians(75)) < 1e-11
    np.testing.assert_allclose(np.asarray(head["transform"])[:3,2], [0,0,-1], atol=1e-10)
    assert head["calibration_status"] == "nominal_urdf_not_calibrated"
    assert result["identity"]["verified_asset_file_count"] == 53


def test_mimic_finger_origin_separation_is_full_gripper_width(description):
    result = description.frames(zero_state(), mast_joint_rad=None)
    for side, width in (("left", 0.04), ("right", 0.08)):
        fingers = result["finger_link_origins"][side]
        separation = np.linalg.norm(np.asarray(fingers[0]["position_m"]) - fingers[1]["position_m"])
        assert abs(separation - width) < 1e-10
        assert all(item["is_tcp"] is False for item in fingers)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True, math.radians(36), math.radians(-51)])
def test_bad_mast_angle_rejected(description, value):
    with pytest.raises(ContractError, match="mast"):
        description.frames(zero_state(), mast_joint_rad=value)


def test_joint6_owner_only_range_is_preserved_for_measurement_and_rejected_for_candidate(description):
    state = zero_state()
    state[5] = 2.5
    result = description.frames(state, mast_joint_rad=None)
    assert result["input_state14"][5] == 2.5 and result["state_was_clipped"] is False
    assert result["description_limit_warnings"][0]["field"] == "left_joint6"
    with pytest.raises(ContractError, match="owner/v12 intersection"):
        description.validate_candidate_limits(state[None])


def test_description_candidate_passes_exact_intersection_and_rejects_bad_shapes(description):
    assert description.validate_candidate_limits(zero_state()[None])["valid"]
    for value in (zero_state(), np.zeros((2, 32)), np.full((1, 14), np.nan)):
        with pytest.raises(ContractError):
            description.validate_candidate_limits(value)
    invalid = zero_state()
    invalid[0] = 8
    with pytest.raises(ContractError, match="feedback tolerance"):
        description.frames(invalid, mast_joint_rad=None)


def test_flange_fk_matches_actual_owner_over_seeded_random_states(description):
    kin = PiperKinematics(description.contract)
    rng = np.random.default_rng(20260918)
    for _ in range(48):
        state = rng.uniform(description.contract.lower14, description.contract.upper14)
        result = description.frames(state, mast_joint_rad=0.0)
        owner = kin.forward(state)
        for side, expected in (("left", owner.left_transform), ("right", owner.right_transform)):
            np.testing.assert_allclose(result["flanges"][side]["transform"], expected, atol=2e-12, rtol=0)
