from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ARM_NAMES,
    PART_DOFS,
    arm_joint_action_dict,
    auxiliary_action_dict,
    gripper_action_dict,
    matrix_to_pose,
    pose_to_matrix,
    rotation_matrix_to_quaternion,
)


@dataclass(frozen=True, slots=True)
class TeleopTarget:
    action: dict[str, float]
    calibrated: bool


class QnexoCartesianRetargeter:
    """Incremental QNexo end-effector retargeting for Astribot control.

    The first enabled sample initializes the previous QNexo endpoint. Each
    later sample integrates the frame-to-frame QNexo SE(3) delta onto the
    previous command target. The SDK still receives an absolute Cartesian pose
    because that is its command contract. Using a command accumulator avoids a
    feedback loop through a lagging measured robot pose.
    """

    def __init__(
        self,
        *,
        axis_map: Any = np.eye(3),
        translation_scale: float = 1.0,
        hand_open_value: float = 0.0,
        hand_closed_value: float = -2.0,
    ) -> None:
        axes = np.asarray(axis_map, dtype=float)
        if axes.shape != (3, 3) or not np.isfinite(axes).all():
            raise ValueError("axis_map must be a finite 3x3 matrix")
        if not np.allclose(axes.T @ axes, np.eye(3), atol=1e-5):
            raise ValueError("axis_map must be orthonormal")
        if not np.isclose(abs(np.linalg.det(axes)), 1.0, atol=1e-5):
            raise ValueError("axis_map must be an orthogonal basis mapping")
        if not np.isfinite(translation_scale) or translation_scale <= 0:
            raise ValueError("translation_scale must be positive")
        if hand_open_value == hand_closed_value:
            raise ValueError("hand open and closed values must differ")
        self.axis_map = axes
        self.translation_scale = float(translation_scale)
        self.hand_open_value = float(hand_open_value)
        self.hand_closed_value = float(hand_closed_value)
        self._exo_previous: dict[str, np.ndarray] | None = None
        self._command_pose: dict[str, np.ndarray] | None = None

    @property
    def calibrated(self) -> bool:
        return self._exo_previous is not None

    def release(self) -> None:
        self._exo_previous = None
        self._command_pose = None

    def update(
        self,
        payload: Mapping[str, Any],
        current_robot_poses: Mapping[str, Any],
    ) -> TeleopTarget:
        exo = {
            "left": self._payload_matrix(payload, "left_pose"),
            "right": self._payload_matrix(payload, "right_pose"),
        }
        robot = {
            "left": pose_to_matrix(current_robot_poses[ARM_NAMES[0]]),
            "right": pose_to_matrix(current_robot_poses[ARM_NAMES[1]]),
        }
        just_calibrated = False
        if self._exo_previous is None:
            self._exo_previous = {side: value.copy() for side, value in exo.items()}
            self._command_pose = {
                side: value.copy() for side, value in robot.items()
            }
            just_calibrated = True

        assert self._command_pose is not None
        action: dict[str, float] = {}
        for side in ("left", "right"):
            exo_previous = self._exo_previous[side]
            command_previous = self._command_pose[side]
            translation_delta = self.axis_map @ (
                exo[side][:3, 3] - exo_previous[:3, 3]
            )
            target_position = (
                command_previous[:3, 3]
                + self.translation_scale * translation_delta
            )
            exo_rotation_delta = exo[side][:3, :3] @ exo_previous[:3, :3].T
            robot_rotation_delta = (
                self.axis_map @ exo_rotation_delta @ self.axis_map.T
            )
            target_rotation = robot_rotation_delta @ command_previous[:3, :3]
            quaternion = rotation_matrix_to_quaternion(target_rotation)
            target_pose = np.eye(4)
            target_pose[:3, :3] = target_rotation
            target_pose[:3, 3] = target_position
            self._command_pose[side] = target_pose
            for name, value in zip(
                ("x", "y", "z", "qx", "qy", "qz", "qw"),
                np.concatenate((target_position, quaternion)),
                strict=True,
            ):
                action[f"{side}.{name}"] = float(value)
            action[f"{side}.gripper"] = self._gripper_percent(
                payload.get(f"{side}_hand")
            )
        self._exo_previous = {side: value.copy() for side, value in exo.items()}
        return TeleopTarget(action, just_calibrated)

    @staticmethod
    def _payload_matrix(payload: Mapping[str, Any], key: str) -> np.ndarray:
        matrix = np.asarray(payload[key], dtype=float)
        if matrix.size != 16:
            raise ValueError(f"QNexo {key} must contain 16 values")
        matrix = matrix.reshape(4, 4)
        # Validate homogeneous and rotation properties through the shared codec.
        matrix_to_pose(matrix)
        return matrix

    def _gripper_percent(self, hand: Any) -> float:
        values = np.asarray(hand, dtype=float)
        if values.ndim != 1 or values.size == 0 or not np.isfinite(values).all():
            raise ValueError("QNexo hand payload must contain finite joint values")
        open_fraction = (
            float(values[0]) - self.hand_closed_value
        ) / (self.hand_open_value - self.hand_closed_value)
        return float(np.clip(100.0 * (1.0 - open_fraction), 0.0, 100.0))


class QnexoJointDeltaRetargeter:
    """GR3-style reference-aligned QNexo to Astribot arm joint mapping."""

    def __init__(self, *, joint_scales: Mapping[str, Any]) -> None:
        self.joint_scales = {}
        for side in ("left", "right"):
            values = np.asarray(joint_scales[side], dtype=float)
            if values.shape != (7,) or not np.isfinite(values).all():
                raise ValueError(f"{side} joint scales must contain 7 finite values")
            self.joint_scales[side] = values
        self._exo_reference: dict[str, np.ndarray] | None = None
        self._robot_reference: dict[str, np.ndarray] | None = None

    def release(self) -> None:
        self._exo_reference = None
        self._robot_reference = None

    @staticmethod
    def _source_joints(payload: Mapping[str, Any], side: str) -> np.ndarray:
        values = np.asarray(payload[f"{side}_arm_joints"], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError(f"QNexo {side} arm must contain 7 finite joints")
        return values

    def update(
        self,
        payload: Mapping[str, Any],
        current_robot_joints: Mapping[str, Any],
    ) -> TeleopTarget:
        exo = {
            side: self._source_joints(payload, side)
            for side in ("left", "right")
        }
        robot = {
            side: np.asarray(current_robot_joints[arm], dtype=float)
            for side, arm in zip(("left", "right"), ARM_NAMES, strict=True)
        }
        if any(values.shape != (7,) or not np.isfinite(values).all() for values in robot.values()):
            raise ValueError("Astribot arm state must contain 7 finite joints per side")
        just_calibrated = False
        if self._exo_reference is None:
            self._exo_reference = {side: values.copy() for side, values in exo.items()}
            self._robot_reference = {side: values.copy() for side, values in robot.items()}
            just_calibrated = True

        assert self._robot_reference is not None
        action: dict[str, float] = {}
        for side, arm in zip(("left", "right"), ARM_NAMES, strict=True):
            # Encoder angles are periodic. Use the shortest signed delta from
            # the takeover reference before applying the GR3 sign mapping.
            raw_delta = exo[side] - self._exo_reference[side]
            delta = np.arctan2(np.sin(raw_delta), np.cos(raw_delta))
            target = self._robot_reference[side] + self.joint_scales[side] * delta
            for index in range(PART_DOFS[arm]):
                action[f"{arm}.joint_{index}"] = float(target[index])
        return TeleopTarget(arm_joint_action_dict(action), just_calibrated)


class QnexoGripperRetargeter:
    """Map the independent QNexo triggers to S1 gripper closure percent."""

    @staticmethod
    def update(payload: Mapping[str, Any]) -> dict[str, float]:
        triggers = np.asarray(
            [payload["left_trigger"], payload["right_trigger"]], dtype=float
        )
        if triggers.shape != (2,) or not np.isfinite(triggers).all():
            raise ValueError("QNexo triggers must contain two finite values")
        if np.any(triggers < 0.0) or np.any(triggers > 1.0):
            raise ValueError("QNexo triggers must be in [0, 1]")
        # QNexo: 1=open, 0=closed. Astribot: 0=open, 100=closed.
        return gripper_action_dict(100.0 * (1.0 - triggers))


class QnexoDirectTorsoRetargeter:
    """Takeover-relative joystick control for the two S1 waist axes.

    Pitch moves only the uppermost pitch joint (SDK joint 3 / action index 2).
    Yaw moves only the dedicated top yaw joint (SDK joint 4 / action index 3).
    The first sample after takeover captures the measured robot pose once;
    later samples integrate joystick deltas from that reference.  A falling
    joystick edge reads the measured pose once more and latches it as the hold
    target.  No continuous measured-position rebasing, kinematics model, Mink
    task, or QP solver participates in this path.
    """

    _PITCH_INDEX = 2
    _YAW_INDEX = 3

    def __init__(
        self,
        *,
        torso_limits: tuple[Sequence[float], Sequence[float]],
        torso_velocity_limits: Sequence[float],
        yaw_left_speed_rad_s: float = 0.1,
        yaw_right_speed_rad_s: float = 0.1,
        yaw_direction: float = 1.0,
        pitch_speed_rad_s: float = 0.1,
        pitch_direction: float = -1.0,
        joystick_switch_threshold: float = 0.25,
        max_dt_seconds: float = 0.02,
    ) -> None:
        lower = np.asarray(torso_limits[0], dtype=float)
        upper = np.asarray(torso_limits[1], dtype=float)
        velocity = np.asarray(torso_velocity_limits, dtype=float)
        if (
            lower.shape != (4,)
            or upper.shape != (4,)
            or velocity.shape != (4,)
            or not np.isfinite(lower).all()
            or not np.isfinite(upper).all()
            or not np.isfinite(velocity).all()
            or np.any(lower >= upper)
            or np.any(velocity <= 0.0)
        ):
            raise ValueError("S1 torso limits must contain four finite joints")
        scalar_values = (
            yaw_left_speed_rad_s,
            yaw_right_speed_rad_s,
            yaw_direction,
            pitch_speed_rad_s,
            pitch_direction,
            joystick_switch_threshold,
            max_dt_seconds,
        )
        if not all(np.isfinite(value) for value in scalar_values):
            raise ValueError("direct torso parameters must be finite")
        if (
            yaw_left_speed_rad_s <= 0.0
            or yaw_right_speed_rad_s <= 0.0
            or pitch_speed_rad_s <= 0.0
            or not 0.0 < joystick_switch_threshold < 1.0
            or max_dt_seconds <= 0.0
        ):
            raise ValueError("invalid direct torso tuning")

        self.lower = lower
        self.upper = upper
        self.yaw_left_speed_rad_s = min(
            float(yaw_left_speed_rad_s), velocity[self._YAW_INDEX]
        )
        self.yaw_right_speed_rad_s = min(
            float(yaw_right_speed_rad_s), velocity[self._YAW_INDEX]
        )
        self.yaw_direction = float(yaw_direction)
        self.pitch_speed_rad_s = min(
            float(pitch_speed_rad_s), velocity[self._PITCH_INDEX]
        )
        self.pitch_direction = float(pitch_direction)
        self.joystick_switch_threshold = float(joystick_switch_threshold)
        self.max_dt_seconds = float(max_dt_seconds)
        self._target_q: np.ndarray | None = None
        self._head_hold: np.ndarray | None = None
        self._last_timestamp_ns: int | None = None
        self._yaw_active = False
        self._pitch_active = False

    @property
    def calibrated(self) -> bool:
        return self._target_q is not None

    def release(self) -> None:
        self._target_q = None
        self._head_hold = None
        self._last_timestamp_ns = None
        self._yaw_active = False
        self._pitch_active = False

    @staticmethod
    def _axis(payload: Mapping[str, Any], name: str) -> float:
        value = float(payload[name])
        if not np.isfinite(value):
            raise ValueError(f"QNexo {name} must be finite")
        return float(np.clip(value, -1.0, 1.0))

    def _switch(self, payload: Mapping[str, Any], name: str) -> float:
        """Quantize a joystick axis to a symmetric three-state switch."""
        value = self._axis(payload, name)
        if abs(value) < self.joystick_switch_threshold:
            return 0.0
        return float(np.sign(value))

    def update(
        self,
        payload: Mapping[str, Any],
        current_robot_joints: Mapping[str, Any],
        *,
        timestamp_ns: int,
    ) -> TeleopTarget:
        torso = np.asarray(current_robot_joints["astribot_torso"], dtype=float)
        head = np.asarray(current_robot_joints["astribot_head"], dtype=float)
        if (
            torso.shape != (4,)
            or head.shape != (2,)
            or not np.isfinite(torso).all()
            or not np.isfinite(head).all()
        ):
            raise ValueError("Astribot torso/head state has an invalid shape")
        timestamp_ns = int(timestamp_ns)
        if self._target_q is None:
            self._target_q = torso.copy()
            self._head_hold = head.copy()
            self._last_timestamp_ns = timestamp_ns
            return TeleopTarget(auxiliary_action_dict([*torso, *head]), True)

        assert self._head_hold is not None
        assert self._last_timestamp_ns is not None
        elapsed = max(0.0, (timestamp_ns - self._last_timestamp_ns) * 1e-9)
        dt = min(max(elapsed, 1e-4), self.max_dt_seconds)
        self._last_timestamp_ns = timestamp_ns

        yaw_switch = self._switch(payload, "left_joystick_x")
        pitch_switch = self._switch(payload, "right_joystick_y")
        yaw_active = yaw_switch != 0.0
        pitch_active = pitch_switch != 0.0

        # Stop on the measured release pose, then hold that fixed target.
        if self._yaw_active and not yaw_active:
            self._target_q[self._YAW_INDEX] = torso[self._YAW_INDEX]
        if self._pitch_active and not pitch_active:
            self._target_q[self._PITCH_INDEX] = torso[self._PITCH_INDEX]
        self._yaw_active = yaw_active
        self._pitch_active = pitch_active

        if yaw_active:
            yaw_speed = (
                self.yaw_right_speed_rad_s
                if yaw_switch > 0.0
                else self.yaw_left_speed_rad_s
            )
            self._target_q[self._YAW_INDEX] = np.clip(
                self._target_q[self._YAW_INDEX]
                + yaw_switch * self.yaw_direction * yaw_speed * dt,
                self.lower[self._YAW_INDEX],
                self.upper[self._YAW_INDEX],
            )
        if pitch_active:
            self._target_q[self._PITCH_INDEX] = np.clip(
                self._target_q[self._PITCH_INDEX]
                + pitch_switch
                * self.pitch_direction
                * self.pitch_speed_rad_s
                * dt,
                self.lower[self._PITCH_INDEX],
                self.upper[self._PITCH_INDEX],
            )

        return TeleopTarget(
            auxiliary_action_dict([*self._target_q, *self._head_hold]),
            False,
        )


def build_qnexo_torso_retargeter(
    profile: Mapping[str, Any],
    *,
    torso_limits: tuple[Sequence[float], Sequence[float]],
    torso_velocity_limits: Sequence[float],
) -> QnexoDirectTorsoRetargeter:
    """Build the takeover-relative torso controller shared by teleop and DAgger."""
    torso = profile["torso_teleop"]
    return QnexoDirectTorsoRetargeter(
        torso_limits=torso_limits,
        torso_velocity_limits=torso_velocity_limits,
        yaw_left_speed_rad_s=torso["yaw_left_speed_rad_s"],
        yaw_right_speed_rad_s=torso["yaw_right_speed_rad_s"],
        yaw_direction=torso["yaw_direction"],
        pitch_speed_rad_s=torso["pitch_speed_rad_s"],
        pitch_direction=torso["pitch_direction"],
        joystick_switch_threshold=torso["joystick_switch_threshold"],
        max_dt_seconds=torso["max_dt_seconds"],
    )
