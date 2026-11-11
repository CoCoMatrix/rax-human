"""Pure FK/IK through the selected XPolicy runtime's PiPER Mink implementation.

Targets are flange transforms in dual_base_link, never inferred tool-center
poses. Quaternions use explicit [w,x,y,z] order. No hardware device is opened.
The repository URDF is kinematics-only and cannot certify collision clearance.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import threading
from typing import Any

import numpy as np

from .contracts import (ContractError, GRIPPER_COLUMNS, JOINT_COLUMNS, JOINT_FIELDS, PiperContract,
                        _readonly, require, runtime_module)


class KinematicsError(ContractError):
    """No usable pose/action is returned when kinematic acceptance fails."""


def _transform(value: Any, name: str) -> np.ndarray:
    require(isinstance(value, np.ndarray) and value.shape == (4, 4),
            f"{name} must be an explicit ndarray[4,4]; flattened transforms are rejected")
    require(value.dtype.kind == "f" and bool(np.isfinite(value).all()), f"{name} must be finite floating point")
    require(bool(np.allclose(value[3], [0, 0, 0, 1], atol=1e-8, rtol=0)),
            f"{name} homogeneous row is invalid")
    rotation = value[:3, :3]
    require(bool(np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0))
            and abs(float(np.linalg.det(rotation)) - 1.0) <= 1e-6,
            f"{name} rotation must be proper orthonormal SO(3)")
    return _readonly(value.astype(np.float64))


def transform_from_pose(position_m: np.ndarray, quaternion_wxyz: np.ndarray) -> np.ndarray:
    """Explicit metre/wxyz pose conversion; no implicit quaternion-order or unit mapping."""
    require(isinstance(position_m, np.ndarray) and position_m.shape == (3,)
            and position_m.dtype.kind == "f" and bool(np.isfinite(position_m).all()),
            "position_m must be floating ndarray[3]")
    require(isinstance(quaternion_wxyz, np.ndarray) and quaternion_wxyz.shape == (4,)
            and quaternion_wxyz.dtype.kind == "f" and bool(np.isfinite(quaternion_wxyz).all()),
            "quaternion_wxyz must be floating ndarray[4]")
    require(abs(float(np.linalg.norm(quaternion_wxyz)) - 1) <= 1e-6,
            "quaternion_wxyz must already be unit length")
    from scipy.spatial.transform import Rotation

    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion_wxyz[[1, 2, 3, 0]]).as_matrix()
    result[:3, 3] = position_m
    return _readonly(result)


@dataclass(frozen=True, slots=True)
class DualFlangePose:
    left_transform: np.ndarray
    right_transform: np.ndarray
    positions_m: np.ndarray
    quaternions_wxyz: np.ndarray
    state14: np.ndarray
    coordinate_frame: str = "dual_base_link"
    endpoint_links: tuple[str, str] = ("left_link6", "right_link6")
    arm_order: tuple[str, str] = ("left_arm", "right_arm")
    collision_certified: bool = False


@dataclass(frozen=True, slots=True)
class IKCandidate:
    action14: np.ndarray
    flange_pose: DualFlangePose
    position_error_m: tuple[float, float]
    orientation_error_rad: tuple[float, float]
    max_joint_jump_rad: float
    iterations: int
    collision_certified: bool = False


class PiperKinematics:
    def __init__(self, contract: PiperContract, *, position_tolerance_m: float | None = None,
                 orientation_tolerance_rad: float | None = None):
        contract.verify_identity()
        self.contract = contract
        runtime = runtime_module(contract.repo_root, "real_world.robots.piper_x_dual.mink_retarget")
        config = runtime.PiperMinkConfig.from_profile(contract.profile)
        updates = {"measured_state_limit_tolerance_rad": 0.0}
        for name, value in (("position_tolerance_m", position_tolerance_m),
                            ("orientation_tolerance_rad", orientation_tolerance_rad)):
            if value is not None:
                require(np.isfinite(value) and 0 < value <= getattr(config, name),
                        f"{name} may tighten but never relax the profile tolerance")
                updates[name] = value
        self.config = replace(config, **updates)
        self._solver = runtime.DualPiperMinkIK(contract.urdf_path,
                                             contract.profile["geometry"]["end_effector_links"],
                                             self.config)
        self._lock = threading.RLock()

    def _seed(self, state14: Any) -> np.ndarray:
        state = self.contract.validate_measured_state(state14)
        joints = state[list(JOINT_COLUMNS)]
        require(bool(np.all(joints >= self._solver.lower) and np.all(joints <= self._solver.upper)),
                "FK/IK seed outside exact URDF limits; refusing hidden clipping of measurements")
        return state

    def _pose(self, state: np.ndarray) -> DualFlangePose:
        from scipy.spatial.transform import Rotation

        transforms = self._solver.forward(state[list(JOINT_COLUMNS)])
        left, right = (_transform(item, name) for item, name in
                       zip(transforms, ("left FK", "right FK"), strict=True))
        positions = np.stack([left[:3, 3], right[:3, 3]])
        xyzw = np.stack([Rotation.from_matrix(item[:3, :3]).as_quat() for item in (left, right)])
        wxyz = xyzw[:, [3, 0, 1, 2]]
        return DualFlangePose(left, right, _readonly(positions), _readonly(wxyz), _readonly(state))

    def forward(self, state14: np.ndarray) -> DualFlangePose:
        """FK preserves the exact input measurement and returns both flange frames."""
        self.contract.verify_identity()
        state = self._seed(state14)
        with self._lock:
            return self._pose(state)

    def inverse(self, seed14: np.ndarray, *, left_target: np.ndarray, right_target: np.ndarray,
                coordinate_frame: str) -> IKCandidate:
        """Solve one explicit dual-flange target, preserving measured gripper widths.

        Invalid inputs raise ContractError; solver/refusal failures raise
        KinematicsError. Neither branch returns a fallback movement.
        """
        self.contract.verify_identity()
        require(coordinate_frame == self.contract.identity["coordinate_frame"],
                "IK targets must explicitly use dual_base_link")
        seed = self._seed(seed14)
        self.contract.validate_command_state(seed)  # Kept grippers must themselves be commandable.
        left_target, right_target = _transform(left_target, "left_target"), _transform(right_target, "right_target")
        with self._lock:
            solve = self._solver.solve(seed[list(JOINT_COLUMNS)], left_target, right_target,
                                       require_convergence=True)
            # At exact zero limits the QP can return e.g. +2e-16 rad for an
            # idle joint whose upper bound is 0. Recover only converged,
            # finite machine-roundoff candidates, never measured feedback.
            # The returned bounded candidate still passes all independent
            # exact command, float32, jump and FK checks below.
            if not solve.valid and solve.reason == "" and solve.position_error_m is not None:
                raw = np.asarray(self._solver.configuration.q[self._solver.qpos_addresses], dtype=float)
                overshoot = max(float(np.max(self._solver.lower-raw)),
                                float(np.max(raw-self._solver.upper)))
                converged = (max(solve.position_error_m) <= self.config.position_tolerance_m
                             and max(solve.orientation_error_rad) <= self.config.orientation_tolerance_rad)
                if converged and np.isfinite(raw).all() and 0 < overshoot <= 1e-12:
                    bounded = np.clip(raw, self._solver.lower, self._solver.upper)
                    solve = replace(solve, valid=True, joint_positions=tuple(bounded),
                                    reason="solver_roundoff_at_exact_limit")
            if not solve.valid or solve.joint_positions is None:
                raw = np.asarray(self._solver.configuration.q[self._solver.qpos_addresses], dtype=float)
                finite = bool(np.isfinite(raw).all())
                near = [i for i in range(12) if finite and
                        min(raw[i]-self._solver.lower[i], self._solver.upper[i]-raw[i]) < .01]
                raise KinematicsError(f"PiPER IK rejected target: {solve.reason}", details={
                    "code": "ik_rejected", "solver_reason": solve.reason,
                    "position_error_m": list(solve.position_error_m) if solve.position_error_m is not None else None,
                    "orientation_error_rad": list(solve.orientation_error_rad) if solve.orientation_error_rad is not None else None,
                    "position_tolerance_m": self.config.position_tolerance_m,
                    "orientation_tolerance_rad": self.config.orientation_tolerance_rad,
                    "joints_near_limits": [{"field": JOINT_FIELDS[i], "value": float(raw[i]),
                        "lower": float(self._solver.lower[i]), "upper": float(self._solver.upper[i]),
                        "unit": "rad", "candidate_is_executable": False} for i in near],
                    "iterations": solve.iterations})
            joints = np.asarray(solve.joint_positions, dtype=np.float64)
            if joints.shape != (12,) or not np.isfinite(joints).all():
                raise KinematicsError("PiPER IK returned invalid joint vector")
            jump = float(np.max(np.abs(joints - seed[list(JOINT_COLUMNS)])))
            if jump > self.config.max_solution_jump_rad:
                raise KinematicsError("PiPER IK joint jump exceeds profile bound", details={
                    "code": "ik_joint_jump", "max_joint_jump_rad": jump,
                    "limit_rad": self.config.max_solution_jump_rad})
            action = seed.astype(np.float64)
            action[list(JOINT_COLUMNS)] = joints
            self.contract.validate_command_state(action)
            # Re-evaluate the actual returned joints rather than trusting residual
            # fields from the solver. Also check the exact float32 wire targets.
            wire_action = self.contract.encode_command_float32(action)
            # Carry representable joints into subsequent IK and interpolation.
            # Preserve measured grippers in IK output; wire widths are encoded
            # independently without changing the observation.
            action[list(JOINT_COLUMNS)] = wire_action[list(JOINT_COLUMNS)].astype(np.float64)
            self.contract.validate_command_state(action)
            jump = float(np.max(np.abs(action[list(JOINT_COLUMNS)] - seed[list(JOINT_COLUMNS)])))
            require(jump <= self.config.max_solution_jump_rad, "encoded IK joint jump exceeds profile bound")
            pose = self._pose(action)
            wire_pose = self._pose(wire_action)
            position_errors, orientation_errors = [], []
            for result_pose in (pose, wire_pose):
                current = [self._solver._errors(actual, target) for actual, target in
                           zip((result_pose.left_transform, result_pose.right_transform),
                               (left_target, right_target), strict=True)]
                if any(position > self.config.position_tolerance_m
                       or orientation > self.config.orientation_tolerance_rad
                       for position, orientation in current):
                    raise KinematicsError("PiPER IK FK residual exceeds profile convergence tolerance", details={
                        "code": "encoded_fk_residual", "position_error_m": [float(x[0]) for x in current],
                        "orientation_error_rad": [float(x[1]) for x in current],
                        "position_tolerance_m": self.config.position_tolerance_m,
                        "orientation_tolerance_rad": self.config.orientation_tolerance_rad})
                position_errors.append(tuple(float(item[0]) for item in current))
                orientation_errors.append(tuple(float(item[1]) for item in current))
            require(bool(np.array_equal(action[list(GRIPPER_COLUMNS)], seed[list(GRIPPER_COLUMNS)])),
                    "IK unexpectedly changed measured grippers")
            return IKCandidate(_readonly(action), pose,
                               tuple(max(a, b) for a, b in zip(*position_errors, strict=True)),
                               tuple(max(a, b) for a, b in zip(*orientation_errors, strict=True)),
                               jump, solve.iterations)
