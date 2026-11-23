from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

def pose_matrix(value: Any, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=float)
    if pose.shape == (16,):
        pose = pose.reshape(4, 4)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError(f"{name} must be a finite 4x4 transform")
    if not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError(f"{name} has an invalid homogeneous row")
    if not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5):
        raise ValueError(f"{name} rotation is not orthonormal")
    return pose


def pose6_matrix(value: Any, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=float)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError(f"{name} must contain six finite pose values")
    result = np.eye(4)
    result[:3, 3] = pose[:3]
    result[:3, :3] = Rotation.from_euler("xyz", pose[3:]).as_matrix()
    return result


def matrix_pose6(value: Any, name: str) -> tuple[float, ...]:
    pose = pose_matrix(value, name)
    rpy = Rotation.from_matrix(pose[:3, :3]).as_euler("xyz")
    return tuple(float(item) for item in (*pose[:3, 3], *rpy))


def smooth_pose(
    previous: Any,
    target: Any,
    *,
    translation_alpha: float,
    rotation_alpha: float,
) -> np.ndarray:
    """Low-pass an absolute pose without filtering Euler angles directly."""
    previous = pose_matrix(previous, "previous")
    target = pose_matrix(target, "target")
    if not 0.0 < translation_alpha <= 1.0:
        raise ValueError("translation_alpha must be in (0, 1]")
    if not 0.0 < rotation_alpha <= 1.0:
        raise ValueError("rotation_alpha must be in (0, 1]")
    result = previous.copy()
    result[:3, 3] += float(translation_alpha) * (
        target[:3, 3] - previous[:3, 3]
    )
    world_delta = target[:3, :3] @ previous[:3, :3].T
    filtered_delta = Rotation.from_rotvec(
        float(rotation_alpha) * Rotation.from_matrix(world_delta).as_rotvec()
    ).as_matrix()
    result[:3, :3] = filtered_delta @ previous[:3, :3]
    return result


def incremental_target(
    robot_reference: Any,
    exo_reference: Any,
    exo_current: Any,
    *,
    axis_map: Any,
    translation_scale: float,
    rotation_scale: float,
    max_translation_m: float | None,
    max_rotation_rad: float | None,
) -> np.ndarray:
    robot_reference = pose_matrix(robot_reference, "robot_reference")
    exo_reference = pose_matrix(exo_reference, "exo_reference")
    exo_current = pose_matrix(exo_current, "exo_current")
    axis_map = np.asarray(axis_map, dtype=float)
    if axis_map.shape != (3, 3) or not np.allclose(
        axis_map.T @ axis_map, np.eye(3), atol=1e-6
    ):
        raise ValueError("axis_map must be an orthonormal 3x3 matrix")
    # Express controller motion in its fixed takeover frame, then bridge the
    # QNexo endpoint basis to the Piper flange basis.  The two URDF frames are
    # not aligned (and the QNexo left/right endpoint frames differ), so using
    # ``axis_map`` directly on this local delta reverses physical directions.
    # ``axis_map`` is the optional world-frame semantic mapping; identity means
    # that +X/+Y/+Z motion in the QNexo URDF world remains +X/+Y/+Z in the
    # dual-Piper URDF world.
    exo_delta = np.linalg.inv(exo_reference) @ exo_current
    frame_map = (
        robot_reference[:3, :3].T
        @ axis_map
        @ exo_reference[:3, :3]
    )
    translation = frame_map @ exo_delta[:3, 3]
    translation *= float(translation_scale)
    if max_translation_m is not None:
        translation_limit = float(max_translation_m)
        if not np.isfinite(translation_limit) or translation_limit <= 0.0:
            raise ValueError("max_translation_m must be positive or None")
        distance = float(np.linalg.norm(translation))
        if distance > translation_limit:
            translation *= translation_limit / distance
    mapped_rotation = frame_map @ exo_delta[:3, :3] @ frame_map.T
    rotation = Rotation.from_matrix(mapped_rotation).as_rotvec()
    rotation *= float(rotation_scale)
    if max_rotation_rad is not None:
        rotation_limit = float(max_rotation_rad)
        if not np.isfinite(rotation_limit) or rotation_limit <= 0.0:
            raise ValueError("max_rotation_rad must be positive or None")
        angle = float(np.linalg.norm(rotation))
        if angle > rotation_limit:
            rotation *= rotation_limit / angle
    mapped_delta = np.eye(4)
    mapped_delta[:3, 3] = translation
    mapped_delta[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    return robot_reference @ mapped_delta


@dataclass(frozen=True, slots=True)
class PiperCartesianTarget:
    status: str
    left_pose: tuple[float, ...] | None = None
    right_pose: tuple[float, ...] | None = None
    left_delta_m: float = 0.0
    right_delta_m: float = 0.0
    left_gripper_m: float = 0.0
    right_gripper_m: float = 0.0


class PiperIncrementalSdkRetargeter:
    """Takeover-relative QNexo motion to controller-side Piper IK targets."""

    def __init__(
        self,
        profile: Mapping[str, Any],
    ) -> None:
        config = profile["shadow_teleop"]["sdk_cartesian"]
        self.translation_scale = float(config["translation_scale"])
        self.rotation_scale = float(config["rotation_scale"])
        self.max_translation_m = (
            None
            if config["max_translation_m"] is None
            else float(config["max_translation_m"])
        )
        self.max_rotation_rad = (
            None
            if config["max_rotation_rad"] is None
            else float(config["max_rotation_rad"])
        )
        self.translation_filter_alpha = float(config["translation_filter_alpha"])
        self.rotation_filter_alpha = float(config["rotation_filter_alpha"])
        self.axis_map_left = np.asarray(config["axis_map_left"], dtype=float)
        self.axis_map_right = np.asarray(config["axis_map_right"], dtype=float)
        self.active = False
        self.exo_reference: tuple[np.ndarray, np.ndarray] | None = None
        self.robot_reference: tuple[np.ndarray, np.ndarray] | None = None
        self.filtered_target: tuple[np.ndarray, np.ndarray] | None = None

    def set_takeover(self, active: bool) -> None:
        active = bool(active)
        if active != self.active:
            self.discard_anchor()
        self.active = active

    def discard_anchor(self) -> None:
        self.exo_reference = None
        self.robot_reference = None
        self.filtered_target = None

    def reset(self) -> None:
        self.active = False
        self.discard_anchor()

    def update(
        self,
        *,
        left_exo: Any,
        right_exo: Any,
        left_robot_flange: Any,
        right_robot_flange: Any,
        left_trigger: float,
        right_trigger: float,
    ) -> PiperCartesianTarget:
        if not self.active:
            return PiperCartesianTarget("INACTIVE")
        try:
            left_exo = pose_matrix(left_exo, "left_exo")
            right_exo = pose_matrix(right_exo, "right_exo")
            left_robot = pose6_matrix(left_robot_flange, "left_robot_flange")
            right_robot = pose6_matrix(right_robot_flange, "right_robot_flange")
            triggers = np.asarray([left_trigger, right_trigger], dtype=float)
            if (
                not np.isfinite(triggers).all()
                or np.any(triggers < 0.0)
                or np.any(triggers > 1.0)
            ):
                raise ValueError("QNexo triggers must be in [0, 1]")
        except (TypeError, ValueError):
            self.discard_anchor()
            return PiperCartesianTarget("HOLD_INVALID_INPUT")
        if self.exo_reference is None or self.robot_reference is None:
            self.exo_reference = left_exo.copy(), right_exo.copy()
            self.robot_reference = left_robot.copy(), right_robot.copy()
            self.filtered_target = left_robot.copy(), right_robot.copy()
            return PiperCartesianTarget("ANCHORED")
        left_target = incremental_target(
            self.robot_reference[0],
            self.exo_reference[0],
            left_exo,
            axis_map=self.axis_map_left,
            translation_scale=self.translation_scale,
            rotation_scale=self.rotation_scale,
            max_translation_m=self.max_translation_m,
            max_rotation_rad=self.max_rotation_rad,
        )
        right_target = incremental_target(
            self.robot_reference[1],
            self.exo_reference[1],
            right_exo,
            axis_map=self.axis_map_right,
            translation_scale=self.translation_scale,
            rotation_scale=self.rotation_scale,
            max_translation_m=self.max_translation_m,
            max_rotation_rad=self.max_rotation_rad,
        )
        if self.filtered_target is None:
            self.filtered_target = self.robot_reference
        left_target = smooth_pose(
            self.filtered_target[0],
            left_target,
            translation_alpha=self.translation_filter_alpha,
            rotation_alpha=self.rotation_filter_alpha,
        )
        right_target = smooth_pose(
            self.filtered_target[1],
            right_target,
            translation_alpha=self.translation_filter_alpha,
            rotation_alpha=self.rotation_filter_alpha,
        )
        self.filtered_target = left_target.copy(), right_target.copy()
        return PiperCartesianTarget(
            "READY_SDK_CARTESIAN",
            matrix_pose6(left_target, "left_target"),
            matrix_pose6(right_target, "right_target"),
            float(np.linalg.norm(left_target[:3, 3] - self.robot_reference[0][:3, 3])),
            float(np.linalg.norm(right_target[:3, 3] - self.robot_reference[1][:3, 3])),
            float(left_trigger * 0.1),
            float(right_trigger * 0.1),
        )
