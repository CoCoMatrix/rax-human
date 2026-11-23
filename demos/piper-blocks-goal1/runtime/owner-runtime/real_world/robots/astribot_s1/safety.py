from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ACTION_NAMES,
    ARM_JOINT_ACTION_NAMES,
    ARM_NAMES,
    AUXILIARY_NAMES,
    CHASSIS_VELOCITY_NAMES,
    GRIPPER_ACTION_NAMES,
    GRIPPER_NAMES,
    PART_DOFS,
    action_dict,
    action_vector,
    arm_joint_action_dict,
    auxiliary_action_dict,
    chassis_velocity_action_dict,
    gripper_action_dict,
    quaternion_to_rotation_matrix,
)

_LIMIT_EPSILON = 1e-6


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    status: str
    action: dict[str, float] | None
    reasons: tuple[str, ...]

    @property
    def accepted(self) -> bool:
        return self.action is not None


class AstribotS1CartesianSafetyGuard:
    """Fail-HOLD gate for the initial dual-arm Cartesian command profile."""

    def __init__(
        self,
        *,
        max_state_age_ms: float,
        max_command_age_ms: float,
        max_translation_step_m: float,
        max_rotation_step_rad: float,
        max_gripper_step_percent: float,
        workspace: Mapping[str, Mapping[str, Sequence[float]]],
        joint_state_limit_tolerance: Mapping[str, float] | None = None,
        gripper_limits: Sequence[float] = (0.0, 100.0),
        quaternion_norm_tolerance: float = 0.05,
    ) -> None:
        positive = (
            max_state_age_ms,
            max_command_age_ms,
            max_translation_step_m,
            max_rotation_step_rad,
            max_gripper_step_percent,
            quaternion_norm_tolerance,
        )
        if not all(np.isfinite(positive)) or any(value <= 0 for value in positive):
            raise ValueError("Astribot safety thresholds must be positive")
        if len(gripper_limits) != 2 or gripper_limits[0] > gripper_limits[1]:
            raise ValueError("invalid gripper limits")
        self.max_state_age_ns = int(max_state_age_ms * 1_000_000)
        self.max_command_age_ns = int(max_command_age_ms * 1_000_000)
        self.max_translation_step_m = float(max_translation_step_m)
        self.max_rotation_step_rad = float(max_rotation_step_rad)
        self.max_gripper_step_percent = float(max_gripper_step_percent)
        self.gripper_limits = tuple(map(float, gripper_limits))
        self.quaternion_norm_tolerance = float(quaternion_norm_tolerance)
        self.workspace: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.joint_state_limit_tolerance = {
            str(name): float(value)
            for name, value in (joint_state_limit_tolerance or {}).items()
        }
        if set(self.joint_state_limit_tolerance) - set(PART_DOFS):
            raise ValueError("joint-state tolerance contains an unknown part")
        if any(value < 0 for value in self.joint_state_limit_tolerance.values()):
            raise ValueError("joint-state tolerances must be non-negative")
        for side in ("left", "right"):
            bounds = workspace[side]
            lower = np.asarray(bounds["lower"], dtype=float)
            upper = np.asarray(bounds["upper"], dtype=float)
            if lower.shape != (3,) or upper.shape != (3,) or np.any(lower > upper):
                raise ValueError(f"invalid {side} workspace")
            self.workspace[side] = (lower, upper)

    @classmethod
    def from_profile(cls, path: str | Path) -> AstribotS1CartesianSafetyGuard:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        return cls(**profile["safety"])

    @staticmethod
    def _hold(*reasons: str) -> SafetyDecision:
        return SafetyDecision("HOLD", None, tuple(reasons))

    def evaluate(
        self,
        action: Mapping[str, Any],
        current_poses: Mapping[str, Sequence[float]],
        current_joints: Mapping[str, Sequence[float]],
        joint_limits: Mapping[str, tuple[Sequence[float], Sequence[float]]],
        *,
        enabled: bool,
        robot_alive: bool,
        have_control_rights: bool,
        generation: int,
        expected_generation: int,
        state_timestamp_ns: int,
        command_timestamp_ns: int,
        now_ns: int,
    ) -> SafetyDecision:
        if not enabled:
            return self._hold("control_disabled")
        if not robot_alive:
            return self._hold("robot_not_alive")
        if not have_control_rights:
            return self._hold("control_rights_missing")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        if state_timestamp_ns <= 0 or now_ns < state_timestamp_ns:
            return self._hold("invalid_state_timestamp")
        if now_ns - state_timestamp_ns > self.max_state_age_ns:
            return self._hold("stale_robot_state")
        if command_timestamp_ns <= 0 or now_ns < command_timestamp_ns:
            return self._hold("invalid_command_timestamp")
        if now_ns - command_timestamp_ns > self.max_command_age_ns:
            return self._hold("stale_command")
        if set(action) != set(ACTION_NAMES):
            return self._hold("invalid_action_schema")
        if set(current_poses) != set(ARM_NAMES):
            return self._hold("invalid_cartesian_state")
        if not self._joints_within_sdk_limits(current_joints, joint_limits):
            return self._hold("joint_state_outside_sdk_limits")
        try:
            target = action_dict(action_vector(action))
            current = {
                side: np.asarray(current_poses[arm], dtype=float)
                for side, arm in zip(("left", "right"), ARM_NAMES, strict=True)
            }
        except (KeyError, TypeError, ValueError):
            return self._hold("non_numeric_or_invalid_action")
        if any(
            pose.shape != (7,) or not np.isfinite(pose).all()
            for pose in current.values()
        ):
            return self._hold("non_finite_cartesian_state")

        safe = dict(target)
        reasons: list[str] = []
        for side in ("left", "right"):
            position = np.array([target[f"{side}.{axis}"] for axis in "xyz"])
            quaternion = np.array(
                [target[f"{side}.{name}"] for name in ("qx", "qy", "qz", "qw")]
            )
            norm = float(np.linalg.norm(quaternion))
            if norm < 1e-8 or abs(norm - 1.0) > self.quaternion_norm_tolerance:
                return self._hold("invalid_quaternion")
            quaternion /= norm
            lower, upper = self.workspace[side]
            clipped_position = np.clip(position, lower, upper)
            if not np.allclose(position, clipped_position):
                reasons.append("workspace_clipped")
            delta = clipped_position - current[side][:3]
            distance = float(np.linalg.norm(delta))
            if distance > self.max_translation_step_m:
                clipped_position = current[side][:3] + delta * (
                    self.max_translation_step_m / distance
                )
                reasons.append("translation_step_limited")

            current_rotation = quaternion_to_rotation_matrix(current[side][3:])
            target_rotation = quaternion_to_rotation_matrix(quaternion)
            relative = target_rotation @ current_rotation.T
            angle = float(
                np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
            )
            if angle > self.max_rotation_step_rad:
                # Normalized quaternion interpolation with sign correction.
                current_q = current[side][3:] / np.linalg.norm(current[side][3:])
                if float(np.dot(current_q, quaternion)) < 0:
                    quaternion = -quaternion
                fraction = self.max_rotation_step_rad / angle
                quaternion = (1.0 - fraction) * current_q + fraction * quaternion
                quaternion /= np.linalg.norm(quaternion)
                reasons.append("rotation_step_limited")

            for axis, value in zip("xyz", clipped_position, strict=True):
                safe[f"{side}.{axis}"] = float(value)
            for name, value in zip(("qx", "qy", "qz", "qw"), quaternion, strict=True):
                safe[f"{side}.{name}"] = float(value)
            gripper = target[f"{side}.gripper"]
            clipped_gripper = float(np.clip(gripper, *self.gripper_limits))
            if clipped_gripper != gripper:
                reasons.append("gripper_clipped")
            gripper_part = (
                "astribot_gripper_left" if side == "left" else "astribot_gripper_right"
            )
            current_gripper = float(current_joints[gripper_part][0])
            delta_gripper = clipped_gripper - current_gripper
            if abs(delta_gripper) > self.max_gripper_step_percent:
                clipped_gripper = current_gripper + np.clip(
                    delta_gripper,
                    -self.max_gripper_step_percent,
                    self.max_gripper_step_percent,
                )
                reasons.append("gripper_step_limited")
            safe[f"{side}.gripper"] = clipped_gripper

        unique_reasons = tuple(dict.fromkeys(reasons))
        return SafetyDecision(
            "ADJUSTED" if unique_reasons else "PASS",
            {name: safe[name] for name in ACTION_NAMES},
            unique_reasons,
        )

    def _joints_within_sdk_limits(
        self,
        current: Mapping[str, Sequence[float]],
        limits: Mapping[str, tuple[Sequence[float], Sequence[float]]],
    ) -> bool:
        if set(current) != set(PART_DOFS) or set(limits) != set(PART_DOFS):
            return False
        for part, dof in PART_DOFS.items():
            values = np.asarray(current[part], dtype=float)
            lower = np.asarray(limits[part][0], dtype=float)
            upper = np.asarray(limits[part][1], dtype=float)
            tolerance = self.joint_state_limit_tolerance.get(part, 0.0)
            if (
                values.shape != (dof,)
                or lower.shape != (dof,)
                or upper.shape != (dof,)
                or not np.isfinite(values).all()
                or np.any(values < lower - tolerance)
                or np.any(values > upper + tolerance)
            ):
                return False
        return True


class AstribotS1JointSafetyGuard:
    """All-or-nothing freshness and hard-limit gate for both S1 arms."""

    def __init__(
        self,
        *,
        max_state_age_ms: float,
        max_command_age_ms: float,
        joint_state_limit_tolerance: Mapping[str, float] | None = None,
    ) -> None:
        values = (max_state_age_ms, max_command_age_ms)
        if not all(np.isfinite(values)) or any(value <= 0 for value in values):
            raise ValueError("Astribot joint safety thresholds must be positive")
        self.max_state_age_ns = int(max_state_age_ms * 1_000_000)
        self.max_command_age_ns = int(max_command_age_ms * 1_000_000)
        self.joint_state_limit_tolerance = {
            str(name): float(value)
            for name, value in (joint_state_limit_tolerance or {}).items()
        }

    @classmethod
    def from_profile(cls, path: str | Path) -> AstribotS1JointSafetyGuard:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        return cls(
            max_state_age_ms=profile["safety"]["max_state_age_ms"],
            max_command_age_ms=profile["safety"]["max_command_age_ms"],
            joint_state_limit_tolerance=profile["safety"].get(
                "joint_state_limit_tolerance"
            ),
        )

    @staticmethod
    def _hold(*reasons: str) -> SafetyDecision:
        return SafetyDecision("HOLD", None, tuple(reasons))

    def evaluate(
        self,
        action: Mapping[str, Any],
        current_joints: Mapping[str, Sequence[float]],
        joint_limits: Mapping[str, tuple[Sequence[float], Sequence[float]]],
        *,
        enabled: bool,
        robot_alive: bool,
        have_control_rights: bool,
        generation: int,
        expected_generation: int,
        state_timestamp_ns: int,
        command_timestamp_ns: int,
        now_ns: int,
    ) -> SafetyDecision:
        if not enabled:
            return self._hold("control_disabled")
        if not robot_alive:
            return self._hold("robot_not_alive")
        if not have_control_rights:
            return self._hold("control_rights_missing")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        if state_timestamp_ns <= 0 or now_ns < state_timestamp_ns:
            return self._hold("invalid_state_timestamp")
        if now_ns - state_timestamp_ns > self.max_state_age_ns:
            return self._hold("stale_robot_state")
        if command_timestamp_ns <= 0 or now_ns < command_timestamp_ns:
            return self._hold("invalid_command_timestamp")
        if now_ns - command_timestamp_ns > self.max_command_age_ns:
            return self._hold("stale_command")
        if set(action) != set(ARM_JOINT_ACTION_NAMES):
            return self._hold("invalid_action_schema")
        if set(current_joints) != set(PART_DOFS) or set(joint_limits) != set(PART_DOFS):
            return self._hold("invalid_joint_state")
        for part, dof in PART_DOFS.items():
            current = np.asarray(current_joints[part], dtype=float)
            lower = np.asarray(joint_limits[part][0], dtype=float)
            upper = np.asarray(joint_limits[part][1], dtype=float)
            tolerance = self.joint_state_limit_tolerance.get(part, 0.0)
            if (
                current.shape != (dof,)
                or lower.shape != (dof,)
                or upper.shape != (dof,)
                or not np.isfinite(current).all()
                or np.any(current < lower - tolerance)
                or np.any(current > upper + tolerance)
            ):
                return self._hold("joint_state_outside_sdk_limits")
        try:
            target = arm_joint_action_dict(action)
        except (TypeError, ValueError):
            return self._hold("non_numeric_or_invalid_action")

        for arm in ARM_NAMES:
            lower = np.asarray(joint_limits[arm][0], dtype=float)
            upper = np.asarray(joint_limits[arm][1], dtype=float)
            for index in range(PART_DOFS[arm]):
                name = f"{arm}.joint_{index}"
                desired = target[name]
                if (
                    desired < lower[index] - _LIMIT_EPSILON
                    or desired > upper[index] + _LIMIT_EPSILON
                ):
                    return self._hold("joint_target_outside_sdk_limits")
        return SafetyDecision("PASS", target, ())


class AstribotS1AuxiliarySafetyGuard:
    """All-or-nothing freshness and hard-limit gate for torso and head."""

    def __init__(
        self,
        *,
        max_state_age_ms: float,
        max_command_age_ms: float,
    ) -> None:
        if max_state_age_ms <= 0 or max_command_age_ms <= 0:
            raise ValueError("Astribot auxiliary freshness thresholds must be positive")
        self.max_state_age_ns = int(max_state_age_ms * 1_000_000)
        self.max_command_age_ns = int(max_command_age_ms * 1_000_000)

    @classmethod
    def from_profile(cls, path: str | Path) -> AstribotS1AuxiliarySafetyGuard:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        return cls(
            max_state_age_ms=profile["safety"]["max_state_age_ms"],
            max_command_age_ms=profile["safety"]["max_command_age_ms"],
        )

    @staticmethod
    def _hold(*reasons: str) -> SafetyDecision:
        return SafetyDecision("HOLD", None, tuple(reasons))

    def evaluate(
        self,
        action: Mapping[str, Any],
        current_joints: Mapping[str, Sequence[float]],
        joint_limits: Mapping[str, tuple[Sequence[float], Sequence[float]]],
        *,
        enabled: bool,
        robot_alive: bool,
        have_control_rights: bool,
        generation: int,
        expected_generation: int,
        state_timestamp_ns: int,
        command_timestamp_ns: int,
        now_ns: int,
    ) -> SafetyDecision:
        common = _common_joint_gate(
            enabled=enabled,
            robot_alive=robot_alive,
            have_control_rights=have_control_rights,
            generation=generation,
            expected_generation=expected_generation,
            state_timestamp_ns=state_timestamp_ns,
            command_timestamp_ns=command_timestamp_ns,
            now_ns=now_ns,
            max_state_age_ns=self.max_state_age_ns,
            max_command_age_ns=self.max_command_age_ns,
        )
        if common is not None:
            return self._hold(common)
        if set(current_joints) != set(PART_DOFS) or set(joint_limits) != set(PART_DOFS):
            return self._hold("invalid_joint_state")
        try:
            target = auxiliary_action_dict(action)
        except (TypeError, ValueError):
            return self._hold("invalid_action_schema")
        for part in AUXILIARY_NAMES:
            current = np.asarray(current_joints[part], dtype=float)
            lower = np.asarray(joint_limits[part][0], dtype=float)
            upper = np.asarray(joint_limits[part][1], dtype=float)
            if (
                current.shape != (PART_DOFS[part],)
                or lower.shape != current.shape
                or upper.shape != current.shape
                or not current.size
                or not np.isfinite(current).all()
                or np.any(current < lower - _LIMIT_EPSILON)
                or np.any(current > upper + _LIMIT_EPSILON)
            ):
                return self._hold("joint_state_outside_sdk_limits")
            for index in range(PART_DOFS[part]):
                name = f"{part}.joint_{index}"
                desired = target[name]
                if (
                    desired < lower[index] - _LIMIT_EPSILON
                    or desired > upper[index] + _LIMIT_EPSILON
                ):
                    return self._hold("joint_target_outside_sdk_limits")
        return SafetyDecision("PASS", target, ())


class AstribotS1ChassisVelocitySafetyGuard:
    """Freshness and SDK-limit gate for local-frame vx, vy and yaw rate."""

    def __init__(
        self,
        *,
        max_state_age_ms: float,
        max_command_age_ms: float,
        max_velocity: Sequence[float],
    ) -> None:
        values = np.asarray(max_velocity, dtype=float)
        if (
            max_state_age_ms <= 0
            or max_command_age_ms <= 0
            or values.shape != (3,)
            or not np.isfinite(values).all()
            or np.any(values <= 0)
        ):
            raise ValueError("invalid Astribot chassis safety thresholds")
        self.max_state_age_ns = int(max_state_age_ms * 1_000_000)
        self.max_command_age_ns = int(max_command_age_ms * 1_000_000)
        self.max_velocity = values

    @classmethod
    def from_profile(cls, path: str | Path) -> AstribotS1ChassisVelocitySafetyGuard:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        return cls(
            max_state_age_ms=profile["safety"]["max_state_age_ms"],
            max_command_age_ms=profile["safety"]["max_command_age_ms"],
            max_velocity=profile["full_body"]["chassis_max_velocity"],
        )

    @staticmethod
    def _hold(*reasons: str) -> SafetyDecision:
        return SafetyDecision("HOLD", None, tuple(reasons))

    def evaluate(
        self,
        action: Mapping[str, Any],
        sdk_velocity_limits: Sequence[float],
        *,
        enabled: bool,
        robot_alive: bool,
        have_control_rights: bool,
        generation: int,
        expected_generation: int,
        state_timestamp_ns: int,
        command_timestamp_ns: int,
        now_ns: int,
    ) -> SafetyDecision:
        common = _common_joint_gate(
            enabled=enabled,
            robot_alive=robot_alive,
            have_control_rights=have_control_rights,
            generation=generation,
            expected_generation=expected_generation,
            state_timestamp_ns=state_timestamp_ns,
            command_timestamp_ns=command_timestamp_ns,
            now_ns=now_ns,
            max_state_age_ns=self.max_state_age_ns,
            max_command_age_ns=self.max_command_age_ns,
        )
        if common is not None:
            return self._hold(common)
        try:
            target = chassis_velocity_action_dict(action)
        except (TypeError, ValueError):
            return self._hold("invalid_action_schema")
        sdk = np.asarray(sdk_velocity_limits, dtype=float)
        if sdk.shape != (3,) or np.any(sdk <= 0):
            return self._hold("invalid_sdk_velocity_limits")
        requested = np.asarray([target[name] for name in CHASSIS_VELOCITY_NAMES])
        limits = np.minimum(self.max_velocity, sdk)
        if np.any(requested < -limits - _LIMIT_EPSILON) or np.any(
            requested > limits + _LIMIT_EPSILON
        ):
            return self._hold("chassis_velocity_outside_sdk_limits")
        return SafetyDecision("PASS", target, ())


def _common_joint_gate(
    *,
    enabled: bool,
    robot_alive: bool,
    have_control_rights: bool,
    generation: int,
    expected_generation: int,
    state_timestamp_ns: int,
    command_timestamp_ns: int,
    now_ns: int,
    max_state_age_ns: int,
    max_command_age_ns: int,
) -> str | None:
    if not enabled:
        return "control_disabled"
    if not robot_alive:
        return "robot_not_alive"
    if not have_control_rights:
        return "control_rights_missing"
    if int(generation) != int(expected_generation):
        return "generation_mismatch"
    if state_timestamp_ns <= 0 or now_ns < state_timestamp_ns:
        return "invalid_state_timestamp"
    if now_ns - state_timestamp_ns > max_state_age_ns:
        return "stale_robot_state"
    if command_timestamp_ns <= 0 or now_ns < command_timestamp_ns:
        return "invalid_command_timestamp"
    if now_ns - command_timestamp_ns > max_command_age_ns:
        return "stale_command"
    return None


class AstribotS1GripperSafetyGuard:
    """All-or-nothing freshness and hard-limit gate for both S1 grippers."""

    def __init__(
        self,
        *,
        max_state_age_ms: float,
        max_command_age_ms: float,
        state_limit_tolerance_percent: float = 1.0,
    ) -> None:
        values = (
            max_state_age_ms,
            max_command_age_ms,
            state_limit_tolerance_percent,
        )
        if not all(np.isfinite(values)) or any(value <= 0 for value in values):
            raise ValueError("Astribot gripper safety thresholds must be positive")
        self.max_state_age_ns = int(max_state_age_ms * 1_000_000)
        self.max_command_age_ns = int(max_command_age_ms * 1_000_000)
        self.state_limit_tolerance_percent = float(state_limit_tolerance_percent)

    @classmethod
    def from_profile(cls, path: str | Path) -> AstribotS1GripperSafetyGuard:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        return cls(
            max_state_age_ms=profile["safety"]["max_state_age_ms"],
            max_command_age_ms=profile["safety"]["max_command_age_ms"],
            state_limit_tolerance_percent=profile["gripper_teleop"].get(
                "state_limit_tolerance_percent", 1.0
            ),
        )

    @staticmethod
    def _hold(*reasons: str) -> SafetyDecision:
        return SafetyDecision("HOLD", None, tuple(reasons))

    def evaluate(
        self,
        action: Mapping[str, Any],
        current_joints: Mapping[str, Sequence[float]],
        joint_limits: Mapping[str, tuple[Sequence[float], Sequence[float]]],
        *,
        enabled: bool,
        robot_alive: bool,
        have_control_rights: bool,
        generation: int,
        expected_generation: int,
        state_timestamp_ns: int,
        command_timestamp_ns: int,
        now_ns: int,
    ) -> SafetyDecision:
        if not enabled:
            return self._hold("control_disabled")
        if not robot_alive:
            return self._hold("robot_not_alive")
        if not have_control_rights:
            return self._hold("control_rights_missing")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        if state_timestamp_ns <= 0 or now_ns < state_timestamp_ns:
            return self._hold("invalid_state_timestamp")
        if now_ns - state_timestamp_ns > self.max_state_age_ns:
            return self._hold("stale_robot_state")
        if command_timestamp_ns <= 0 or now_ns < command_timestamp_ns:
            return self._hold("invalid_command_timestamp")
        if now_ns - command_timestamp_ns > self.max_command_age_ns:
            return self._hold("stale_command")
        try:
            target = gripper_action_dict(action)
        except (TypeError, ValueError):
            return self._hold("non_numeric_or_invalid_action")

        for part, name in zip(GRIPPER_NAMES, GRIPPER_ACTION_NAMES, strict=True):
            if part not in current_joints or part not in joint_limits:
                return self._hold("invalid_gripper_state")
            current_values = np.asarray(current_joints[part], dtype=float)
            lower_values = np.asarray(joint_limits[part][0], dtype=float)
            upper_values = np.asarray(joint_limits[part][1], dtype=float)
            if (
                current_values.shape != (1,)
                or lower_values.shape != (1,)
                or upper_values.shape != (1,)
                or not np.isfinite(current_values).all()
            ):
                return self._hold("invalid_gripper_state")
            lower, upper = float(lower_values[0]), float(upper_values[0])
            current = float(current_values[0])
            tolerance = self.state_limit_tolerance_percent
            if current < lower - tolerance or current > upper + tolerance:
                return self._hold("gripper_state_outside_sdk_limits")
            desired = target[name]
            if (
                desired < lower - _LIMIT_EPSILON
                or desired > upper + _LIMIT_EPSILON
            ):
                return self._hold("gripper_target_outside_sdk_limits")
        return SafetyDecision("PASS", target, ())
