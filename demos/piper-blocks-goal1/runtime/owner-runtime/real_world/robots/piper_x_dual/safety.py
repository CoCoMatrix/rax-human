from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from .cartesian_retarget import matrix_pose6, pose6_matrix
from .mapping import ACTION_NAMES, GRIPPER_NAMES, JOINT_NAMES, action_dict, action_vector


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    status: str
    action: dict[str, float] | None
    reasons: tuple[str, ...]

    @property
    def accepted(self) -> bool:
        return self.action is not None


@dataclass(frozen=True, slots=True)
class BoundaryAdjustment:
    name: str
    original: float
    adjusted: float
    overshoot: float

    def as_dict(self) -> dict[str, float | str]:
        return {
            "name": self.name,
            "original": self.original,
            "adjusted": self.adjusted,
            "overshoot": self.overshoot,
        }


def normalize_model_action_boundaries(
    action: Mapping[str, Any],
    *,
    joint_limits_rad: Mapping[str, tuple[float, float]],
    gripper_limits_m: tuple[float, float],
    joint_tolerance_rad: float,
) -> tuple[dict[str, float], tuple[BoundaryAdjustment, ...]]:
    """Normalize learned-model targets against the physical command ranges.

    Gripper targets are absolute widths, so every finite model prediction is
    saturated to the physical gripper range. Joint targets are only snapped
    when their overshoot is within the configured tolerance; larger joint
    violations remain unchanged for the final all-or-nothing safety gate.
    Non-finite values are never normalized and are rejected by that gate.
    """

    if not 0.0 < float(joint_tolerance_rad) <= 0.03:
        raise ValueError("model joint boundary tolerance must be in (0, 0.03] rad")
    normalized = action_dict(action)
    adjustments: list[BoundaryAdjustment] = []
    limits = {
        **{name: tuple(map(float, joint_limits_rad[name])) for name in JOINT_NAMES},
        **{name: tuple(map(float, gripper_limits_m)) for name in GRIPPER_NAMES},
    }
    for name in ACTION_NAMES:
        value = normalized[name]
        lower, upper = limits[name]
        adjusted: float | None = None
        if name in GRIPPER_NAMES:
            if np.isfinite(value):
                adjusted = min(max(value, lower), upper)
                if adjusted == value:
                    adjusted = None
        else:
            tolerance = float(joint_tolerance_rad)
            if lower - tolerance <= value < lower:
                adjusted = lower
            elif upper < value <= upper + tolerance:
                adjusted = upper
        if adjusted is None:
            continue
        adjustments.append(
            BoundaryAdjustment(
                name=name,
                original=value,
                adjusted=adjusted,
                overshoot=abs(value - adjusted),
            )
        )
        normalized[name] = adjusted
    return normalized, tuple(adjustments)


class DualPiperJointSafetyGuard:
    """All-or-nothing final gate for a future dual-arm command writer."""

    def __init__(
        self,
        *,
        max_state_age_ms: float,
        max_command_age_ms: float,
        measured_state_limit_tolerance_rad: float,
        joint_limits_rad: Mapping[str, tuple[float, float]],
        gripper_limits_m: tuple[float, float] = (0.0, 0.1),
        measured_gripper_limit_tolerance_m: float = 0.0,
    ) -> None:
        positive = (
            max_state_age_ms,
            max_command_age_ms,
            measured_state_limit_tolerance_rad,
        )
        if not np.isfinite(positive).all() or any(value <= 0.0 for value in positive):
            raise ValueError("Piper safety thresholds must be positive")
        if set(joint_limits_rad) != set(JOINT_NAMES):
            raise ValueError("Piper safety limits must cover exactly twelve joints")
        limits = {
            name: (float(bounds[0]), float(bounds[1]))
            for name, bounds in joint_limits_rad.items()
        }
        if any(lower >= upper for lower, upper in limits.values()):
            raise ValueError("invalid Piper joint limit ordering")
        if len(gripper_limits_m) != 2 or gripper_limits_m[0] >= gripper_limits_m[1]:
            raise ValueError("invalid Piper gripper limits")
        if (
            not np.isfinite(measured_gripper_limit_tolerance_m)
            or not 0.0 <= measured_gripper_limit_tolerance_m <= 0.01
        ):
            raise ValueError(
                "measured gripper limit tolerance must be in [0.0, 0.01] m"
            )
        self.max_state_age_ns = int(max_state_age_ms * 1_000_000)
        self.max_command_age_ns = int(max_command_age_ms * 1_000_000)
        self.measured_state_limit_tolerance_rad = float(
            measured_state_limit_tolerance_rad
        )
        self.joint_limits_rad = limits
        self.gripper_limits_m = tuple(map(float, gripper_limits_m))
        self.measured_gripper_limit_tolerance_m = float(
            measured_gripper_limit_tolerance_m
        )

    @staticmethod
    def _hold(*reasons: str) -> SafetyDecision:
        return SafetyDecision("HOLD", None, tuple(reasons))

    def evaluate(
        self,
        action: Mapping[str, Any],
        robot_state: Mapping[str, Any],
        *,
        enabled: bool,
        write_authorized: bool,
        both_arms_ready: bool,
        generation: int,
        expected_generation: int,
        state_timestamp_ns: int,
        command_timestamp_ns: int,
        now_ns: int,
    ) -> SafetyDecision:
        if not enabled:
            return self._hold("control_disabled")
        if not write_authorized:
            return self._hold("write_not_authorized")
        if not both_arms_ready:
            return self._hold("dual_arm_not_ready")
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
        if set(action) != set(ACTION_NAMES) or set(robot_state) != set(ACTION_NAMES):
            return self._hold("invalid_14d_schema")
        try:
            target = action_vector(action)
            current = action_vector(robot_state)
        except (TypeError, ValueError):
            return self._hold("non_numeric_value")
        if not np.isfinite(target).all():
            return self._hold("non_finite_action")
        if not np.isfinite(current).all():
            return self._hold("non_finite_state")

        current_by_name = action_dict(current)
        for name in JOINT_NAMES:
            lower, upper = self.joint_limits_rad[name]
            value = current_by_name[name]
            tolerance = self.measured_state_limit_tolerance_rad
            if not lower - tolerance <= value <= upper + tolerance:
                return self._hold("state_outside_joint_limits")
        for name in GRIPPER_NAMES:
            tolerance = self.measured_gripper_limit_tolerance_m
            if not (
                self.gripper_limits_m[0] - tolerance
                <= current_by_name[name]
                <= self.gripper_limits_m[1] + tolerance
            ):
                return self._hold("state_outside_gripper_limits")

        safe = action_dict(target)
        for name in JOINT_NAMES:
            lower, upper = self.joint_limits_rad[name]
            if not lower <= safe[name] <= upper:
                return self._hold("target_outside_joint_limits")
        for name in GRIPPER_NAMES:
            if not self.gripper_limits_m[0] <= safe[name] <= self.gripper_limits_m[1]:
                return self._hold("target_outside_gripper_limits")

        return SafetyDecision(
            "PASS",
            {name: safe[name] for name in ACTION_NAMES},
            (),
        )


@dataclass(frozen=True, slots=True)
class CartesianSafetyDecision:
    status: str
    left_pose: tuple[float, ...] | None
    right_pose: tuple[float, ...] | None
    reasons: tuple[str, ...]

    @property
    def accepted(self) -> bool:
        return self.left_pose is not None and self.right_pose is not None


class DualPiperCartesianSafetyGuard:
    """Final Cartesian gate before Piper V189 firmware performs move_p IK."""

    def __init__(
        self,
        *,
        max_state_age_ms: float,
        max_command_age_ms: float,
        workspace_xyz_m: Mapping[str, tuple[float, float]],
    ) -> None:
        values = (max_state_age_ms, max_command_age_ms)
        if not np.isfinite(values).all() or any(value <= 0.0 for value in values):
            raise ValueError("Piper Cartesian safety thresholds must be positive")
        if tuple(workspace_xyz_m) != ("x", "y", "z"):
            raise ValueError("Piper Cartesian workspace must define x, y, z")
        self.workspace = {
            axis: tuple(map(float, workspace_xyz_m[axis]))
            for axis in ("x", "y", "z")
        }
        if any(lower >= upper for lower, upper in self.workspace.values()):
            raise ValueError("invalid Piper Cartesian workspace ordering")
        self.max_state_age_ns = int(max_state_age_ms * 1_000_000)
        self.max_command_age_ns = int(max_command_age_ms * 1_000_000)

    @staticmethod
    def _hold(*reasons: str) -> CartesianSafetyDecision:
        return CartesianSafetyDecision("HOLD", None, None, tuple(reasons))

    def evaluate(
        self,
        *,
        left_target: Any,
        right_target: Any,
        left_current: Any,
        right_current: Any,
        enabled: bool,
        write_authorized: bool,
        both_arms_ready: bool,
        generation: int,
        expected_generation: int,
        state_timestamp_ns: int,
        command_timestamp_ns: int,
        now_ns: int,
    ) -> CartesianSafetyDecision:
        if not enabled:
            return self._hold("control_disabled")
        if not write_authorized:
            return self._hold("write_not_authorized")
        if not both_arms_ready:
            return self._hold("dual_arm_not_ready")
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
            targets = (
                pose6_matrix(left_target, "left_target"),
                pose6_matrix(right_target, "right_target"),
            )
            pose6_matrix(left_current, "left_current")
            pose6_matrix(right_current, "right_current")
        except (TypeError, ValueError):
            return self._hold("invalid_cartesian_pose")
        for target in targets:
            for axis_index, axis in enumerate(("x", "y", "z")):
                lower, upper = self.workspace[axis]
                if not lower <= target[axis_index, 3] <= upper:
                    return self._hold("target_outside_workspace")
        # Validation is fail-closed and never rewrites a valid command. Mixing
        # measured feedback into the target here would create a second,
        # feedback-driven incremental loop.
        exact = [matrix_pose6(target, "validated_target") for target in targets]
        return CartesianSafetyDecision(
            "PASS",
            exact[0],
            exact[1],
            (),
        )
