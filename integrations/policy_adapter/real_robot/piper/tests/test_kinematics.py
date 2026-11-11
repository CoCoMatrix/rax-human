import os
from types import SimpleNamespace

import numpy as np
import pytest

from real_robot.piper.contracts import ContractError, PiperContract
from real_robot.piper.kinematics import KinematicsError, PiperKinematics, transform_from_pose


@pytest.fixture(scope="module")
def kinematics():
    root = os.environ.get("PIPER_RUNTIME_ROOT")
    if root is None:
        pytest.skip("PIPER_RUNTIME_ROOT is required for actual Mink tests")
    pytest.importorskip("mink")
    return PiperKinematics(PiperContract.from_runtime(root))


def state():
    return np.array([0.] * 6 + [0.0003] + [0.] * 6 + [0.0004], dtype=np.float32)


def test_real_fk_home_mounts_and_wxyz_roundtrip(kinematics):
    measured = state()
    pose = kinematics.forward(measured)
    np.testing.assert_array_equal(pose.state14, measured)
    np.testing.assert_allclose(pose.positions_m[:, 0], [0.096638, 0.096638], atol=2e-6)
    np.testing.assert_allclose(pose.positions_m[:, 1], [0.3, -0.3], atol=1e-8)
    np.testing.assert_allclose(pose.positions_m[:, 2], [0.2248045904] * 2, atol=1e-9)
    for index, transform in enumerate((pose.left_transform, pose.right_transform)):
        reconstructed = transform_from_pose(pose.positions_m[index], pose.quaternions_wxyz[index])
        np.testing.assert_allclose(reconstructed, transform, atol=1e-9)
    assert pose.coordinate_frame == "dual_base_link" and not pose.collision_certified


def test_real_ik_small_dual_lift_preserves_measured_grippers(kinematics):
    measured = state()
    pose = kinematics.forward(measured)
    left, right = pose.left_transform.copy(), pose.right_transform.copy()
    left[2, 3] += 0.01
    right[2, 3] += 0.01
    result = kinematics.inverse(measured, left_target=left, right_target=right,
                                coordinate_frame="dual_base_link")
    np.testing.assert_array_equal(result.action14[[6, 13]], measured[[6, 13]])
    assert max(result.position_error_m) <= 0.002
    assert max(result.orientation_error_rad) <= 0.03
    assert result.max_joint_jump_rad <= kinematics.config.max_solution_jump_rad
    assert not result.collision_certified


@pytest.mark.parametrize("fault,match", [
    ("flat", "explicit"), ("reflection", "SO\\(3\\)"), ("nan", "finite"),
    ("bad_bottom", "homogeneous"), ("wrong_frame", "dual_base_link"),
    ("bad_seed", "clipping"), ("negative_gripper", "command limits"),
])
def test_ik_rejects_ambiguous_or_unsafe_inputs(kinematics, fault, match):
    measured = state()
    pose = kinematics.forward(measured)
    left, right = pose.left_transform.copy(), pose.right_transform.copy()
    frame = "dual_base_link"
    if fault == "flat": left = left.flatten()
    elif fault == "reflection": left[:3, 0] *= -1
    elif fault == "nan": left[0, 3] = np.nan
    elif fault == "bad_bottom": left[3, 0] = 0.01
    elif fault == "wrong_frame": frame = "left_base_link"
    elif fault == "bad_seed": measured[1] = -0.001
    elif fault == "negative_gripper": measured[6] = -0.001
    with pytest.raises(ContractError, match=match):
        kinematics.inverse(measured, left_target=left, right_target=right, coordinate_frame=frame)


def test_ik_nonconvergence_returns_no_fallback(kinematics, monkeypatch):
    measured = state()
    pose = kinematics.forward(measured)
    monkeypatch.setattr(kinematics._solver, "solve", lambda *args, **kwargs:
                        SimpleNamespace(valid=False, joint_positions=None, reason="ik_residual_exceeded",
                                        position_error_m=None, orientation_error_rad=None, iterations=0))
    with pytest.raises(KinematicsError, match="rejected target"):
        kinematics.inverse(measured, left_target=pose.left_transform, right_target=pose.right_transform,
                           coordinate_frame="dual_base_link")


def test_ik_branch_jump_rejected(kinematics, monkeypatch):
    measured = state()
    pose = kinematics.forward(measured)
    joints = np.zeros(12)
    joints[0] = kinematics.config.max_solution_jump_rad + 0.01
    monkeypatch.setattr(kinematics._solver, "solve", lambda *args, **kwargs:
                        SimpleNamespace(valid=True, joint_positions=tuple(joints)))
    with pytest.raises(KinematicsError, match="jump"):
        kinematics.inverse(measured, left_target=pose.left_transform, right_target=pose.right_transform,
                           coordinate_frame="dual_base_link")


def test_ik_independent_fk_rejects_false_solver_success(kinematics, monkeypatch):
    measured = state()
    pose = kinematics.forward(measured)
    joints = np.zeros(12)
    joints[0] = 0.2
    monkeypatch.setattr(kinematics._solver, "solve", lambda *args, **kwargs:
                        SimpleNamespace(valid=True, joint_positions=tuple(joints)))
    with pytest.raises(KinematicsError, match="FK residual"):
        kinematics.inverse(measured, left_target=pose.left_transform, right_target=pose.right_transform,
                           coordinate_frame="dual_base_link")


def test_tolerances_cannot_be_weakened(kinematics):
    with pytest.raises(ContractError, match="never relax"):
        PiperKinematics(kinematics.contract, position_tolerance_m=0.003)


def test_quaternion_normalization_is_not_guessed():
    with pytest.raises(ContractError, match="unit length"):
        transform_from_pose(np.zeros(3), np.array([2., 0, 0, 0]))
