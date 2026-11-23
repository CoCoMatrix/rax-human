from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .cartesian_retarget import incremental_target, smooth_pose
from .mapping import JOINT_NAMES, action_dict


def _pose(value: Any, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=float)
    # Dora/Arrow transports each transform as one flat 16-value list.
    if pose.shape == (16,):
        pose = pose.reshape(4, 4)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError(f"{name} must be a finite 4x4 transform")
    if not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError(f"{name} has an invalid homogeneous row")
    if not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5):
        raise ValueError(f"{name} rotation is not orthonormal")
    return pose


@dataclass(frozen=True, slots=True)
class PiperMinkConfig:
    solver: str
    control_hz: float
    max_iterations: int
    translation_scale: float
    rotation_scale: float
    max_translation_m: float | None
    max_rotation_rad: float | None
    translation_filter_alpha: float
    rotation_filter_alpha: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    posture_cost: float
    lm_damping: float
    position_cost: float
    orientation_cost: float
    max_solution_jump_rad: float
    measured_state_limit_tolerance_rad: float
    axis_map_left: np.ndarray
    axis_map_right: np.ndarray

    @classmethod
    def from_profile(cls, profile: Mapping[str, Any]) -> PiperMinkConfig:
        value = profile["shadow_teleop"]["mink"]
        return cls(
            solver=str(value["solver"]),
            control_hz=float(value["control_hz"]),
            max_iterations=int(value["max_iterations"]),
            translation_scale=float(value["translation_scale"]),
            rotation_scale=float(value["rotation_scale"]),
            max_translation_m=(
                None
                if value["max_translation_m"] is None
                else float(value["max_translation_m"])
            ),
            max_rotation_rad=(
                None
                if value["max_rotation_rad"] is None
                else float(value["max_rotation_rad"])
            ),
            translation_filter_alpha=float(value["translation_filter_alpha"]),
            rotation_filter_alpha=float(value["rotation_filter_alpha"]),
            position_tolerance_m=float(value["position_tolerance_m"]),
            orientation_tolerance_rad=float(value["orientation_tolerance_rad"]),
            posture_cost=float(value["posture_cost"]),
            lm_damping=float(value["lm_damping"]),
            position_cost=float(value["position_cost"]),
            orientation_cost=float(value["orientation_cost"]),
            max_solution_jump_rad=float(value["max_solution_jump_rad"]),
            measured_state_limit_tolerance_rad=float(
                profile["live_takeover"]["measured_state_limit_tolerance_rad"]
            ),
            axis_map_left=np.asarray(value["axis_map_left"], dtype=float),
            axis_map_right=np.asarray(value["axis_map_right"], dtype=float),
        )

@dataclass(frozen=True, slots=True)
class PiperMinkSolve:
    valid: bool
    joint_positions: tuple[float, ...] | None
    position_error_m: tuple[float, float] | None
    orientation_error_rad: tuple[float, float] | None
    iterations: int
    reason: str = ""


class DualPiperMinkIK:
    """Dual FrameTask IK over the repository-owned 12DoF Piper URDF."""

    def __init__(
        self,
        urdf_path: str | Path,
        endpoints: Mapping[str, str],
        config: PiperMinkConfig,
    ) -> None:
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(str(Path(urdf_path).resolve()))
        self.configuration = mink.Configuration(self.model)
        model_joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(self.model.njnt)
        ]
        if model_joint_names != list(JOINT_NAMES):
            raise ValueError(
                "Piper Mink model joint order does not match the 12D contract"
            )
        self.qpos_addresses = np.asarray(
            [self.model.jnt_qposadr[self.model.joint(name).id] for name in JOINT_NAMES]
        )
        self.lower = np.asarray(
            [self.model.jnt_range[self.model.joint(name).id, 0] for name in JOINT_NAMES]
        )
        self.upper = np.asarray(
            [self.model.jnt_range[self.model.joint(name).id, 1] for name in JOINT_NAMES]
        )
        self.endpoints = {
            "left": str(endpoints["left_arm"]),
            "right": str(endpoints["right_arm"]),
        }
        self.frame_tasks = {
            side: mink.FrameTask(
                frame_name=endpoint,
                frame_type="body",
                position_cost=config.position_cost,
                orientation_cost=config.orientation_cost,
                gain=1.0,
                lm_damping=config.lm_damping,
            )
            for side, endpoint in self.endpoints.items()
        }
        self.posture_task = mink.PostureTask(self.model, cost=config.posture_cost)
        self.limits = [mink.ConfigurationLimit(self.model)]

    def _model_q(self, joints: Any) -> np.ndarray:
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (12,) or not np.isfinite(joints).all():
            raise ValueError("Piper IK seed must contain 12 finite joints")
        tolerance = self.config.measured_state_limit_tolerance_rad
        if np.any(joints < self.lower - tolerance) or np.any(
            joints > self.upper + tolerance
        ):
            raise ValueError("Piper IK seed is outside URDF joint limits")
        joints = np.clip(joints, self.lower, self.upper)
        q = self.model.qpos0.copy()
        q[self.qpos_addresses] = joints
        return q

    def forward(self, joints: Any) -> tuple[np.ndarray, np.ndarray]:
        self.configuration.update(self._model_q(joints))
        return tuple(
            self.configuration.get_transform_frame_to_world(
                self.endpoints[side], "body"
            ).as_matrix()
            for side in ("left", "right")
        )

    @staticmethod
    def _errors(actual: np.ndarray, target: np.ndarray) -> tuple[float, float]:
        position = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
        orientation = float(
            np.linalg.norm(
                Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).as_rotvec()
            )
        )
        return position, orientation

    def solve(
        self,
        seed_joints: Any,
        left_target: Any,
        right_target: Any,
        *,
        max_iterations: int | None = None,
        require_convergence: bool = True,
    ) -> PiperMinkSolve:
        targets = {
            "left": _pose(left_target, "left_target"),
            "right": _pose(right_target, "right_target"),
        }
        iterations = 0
        try:
            q = self._model_q(seed_joints)
            self.configuration.update(q)
            self.posture_task.set_target(q)
            for side, task in self.frame_tasks.items():
                task.set_target(mink.SE3.from_matrix(targets[side]))
            tasks = [*self.frame_tasks.values(), self.posture_task]
            dt = 1.0 / self.config.control_hz
            iteration_limit = (
                self.config.max_iterations
                if max_iterations is None
                else int(max_iterations)
            )
            if iteration_limit < 1:
                raise ValueError("Piper IK max_iterations must be positive")
            for iterations in range(1, iteration_limit + 1):
                velocity = mink.solve_ik(
                    self.configuration,
                    tasks,
                    dt,
                    self.config.solver,
                    damping=1e-6,
                    safety_break=True,
                    limits=self.limits,
                )
                self.configuration.integrate_inplace(velocity, dt)
                errors = [
                    self._errors(
                        self.configuration.get_transform_frame_to_world(
                            self.endpoints[side], "body"
                        ).as_matrix(),
                        targets[side],
                    )
                    for side in ("left", "right")
                ]
                if all(
                    position <= self.config.position_tolerance_m
                    and orientation <= self.config.orientation_tolerance_rad
                    for position, orientation in errors
                ):
                    break
        # Solver backends do not share one exception base class. Any backend
        # failure must fail closed here instead of escaping into the Dora node.
        except Exception as exc:  # noqa: BLE001
            return PiperMinkSolve(
                False,
                None,
                None,
                None,
                iterations,
                f"{type(exc).__name__}: {exc}",
            )

        actual = [
            self.configuration.get_transform_frame_to_world(
                self.endpoints[side], "body"
            ).as_matrix()
            for side in ("left", "right")
        ]
        errors = [
            self._errors(value, targets[side])
            for value, side in zip(actual, ("left", "right"), strict=True)
        ]
        position_errors = tuple(value[0] for value in errors)
        orientation_errors = tuple(value[1] for value in errors)
        converged = all(
            position <= self.config.position_tolerance_m
            and orientation <= self.config.orientation_tolerance_rad
            for position, orientation in errors
        )
        joints = self.configuration.q[self.qpos_addresses]
        tolerance = self.config.measured_state_limit_tolerance_rad
        joints_within_tolerance = bool(
            np.all(joints >= self.lower - tolerance)
            and np.all(joints <= self.upper + tolerance)
        )
        bounded_joints = np.clip(joints, self.lower, self.upper)
        valid = (
            (converged or not require_convergence)
            and np.isfinite(joints).all()
            and joints_within_tolerance
        )
        return PiperMinkSolve(
            valid,
            tuple(float(value) for value in bounded_joints) if valid else None,
            position_errors,
            orientation_errors,
            iterations,
            (
                ""
                if converged
                else (
                    "tracking_residual"
                    if valid
                    else "ik_residual_exceeded"
                )
            ),
        )

@dataclass(frozen=True, slots=True)
class PiperRetargetResult:
    status: str
    action: dict[str, float] | None = None
    solve: PiperMinkSolve | None = None
    left_delta_m: float = 0.0
    right_delta_m: float = 0.0
    max_solution_step_rad: float = 0.0
    max_command_step_rad: float = 0.0


class PiperIncrementalMinkRetargeter:
    """Takeover-relative QNexo endpoint motion to no-send Piper joint targets."""

    def __init__(self, solver: DualPiperMinkIK, config: PiperMinkConfig) -> None:
        self.solver = solver
        self.config = config
        self.active = False
        self.exo_reference: tuple[np.ndarray, np.ndarray] | None = None
        self.robot_reference: tuple[np.ndarray, np.ndarray] | None = None
        self.filtered_target: tuple[np.ndarray, np.ndarray] | None = None
        self.last_solution: np.ndarray | None = None

    def set_takeover(self, active: bool) -> None:
        active = bool(active)
        if active != self.active:
            self.exo_reference = None
            self.robot_reference = None
            self.filtered_target = None
            self.last_solution = None
        self.active = active

    def reset(self) -> None:
        self.active = False
        self.exo_reference = None
        self.robot_reference = None
        self.filtered_target = None
        self.last_solution = None

    def discard_anchor(self) -> None:
        """Drop a bad/stale FK frame without changing the takeover request."""
        self.exo_reference = None
        self.robot_reference = None
        self.filtered_target = None
        self.last_solution = None

    def update(
        self,
        *,
        left_exo: Any,
        right_exo: Any,
        robot_state: Mapping[str, Any],
        left_trigger: float,
        right_trigger: float,
        max_gripper_width_m: float,
    ) -> PiperRetargetResult:
        if not self.active:
            return PiperRetargetResult("INACTIVE")
        try:
            left_exo = _pose(left_exo, "left_exo")
            right_exo = _pose(right_exo, "right_exo")
            state = action_dict(robot_state)
            joints = np.asarray([state[name] for name in JOINT_NAMES], dtype=float)
            triggers = np.asarray([left_trigger, right_trigger], dtype=float)
            if not np.isfinite(joints).all():
                raise ValueError("Piper state joints must be finite")
            if (
                not np.isfinite(triggers).all()
                or np.any(triggers < 0.0)
                or np.any(triggers > 1.0)
            ):
                raise ValueError("QNexo triggers must be finite values in [0, 1]")
        except (TypeError, ValueError):
            # A single corrupted transport/FK frame must never kill the formal
            # robot graph or preserve an anchor across an unknown pose gap.
            self.discard_anchor()
            return PiperRetargetResult("HOLD_INVALID_EXO")

        if self.exo_reference is None or self.robot_reference is None:
            self.exo_reference = left_exo.copy(), right_exo.copy()
            try:
                self.robot_reference = self.solver.forward(joints)
            except ValueError:
                self.discard_anchor()
                return PiperRetargetResult("HOLD_INVALID_ROBOT_STATE")
            self.last_solution = joints.copy()
            self.filtered_target = tuple(
                target.copy() for target in self.robot_reference
            )
            return PiperRetargetResult("ANCHORED")

        left_target = incremental_target(
            self.robot_reference[0],
            self.exo_reference[0],
            left_exo,
            axis_map=self.config.axis_map_left,
            translation_scale=self.config.translation_scale,
            rotation_scale=self.config.rotation_scale,
            max_translation_m=self.config.max_translation_m,
            max_rotation_rad=self.config.max_rotation_rad,
        )
        right_target = incremental_target(
            self.robot_reference[1],
            self.exo_reference[1],
            right_exo,
            axis_map=self.config.axis_map_right,
            translation_scale=self.config.translation_scale,
            rotation_scale=self.config.rotation_scale,
            max_translation_m=self.config.max_translation_m,
            max_rotation_rad=self.config.max_rotation_rad,
        )
        if self.filtered_target is None:
            self.filtered_target = tuple(
                target.copy() for target in self.robot_reference
            )
        left_target = smooth_pose(
            self.filtered_target[0],
            left_target,
            translation_alpha=self.config.translation_filter_alpha,
            rotation_alpha=self.config.rotation_filter_alpha,
        )
        right_target = smooth_pose(
            self.filtered_target[1],
            right_target,
            translation_alpha=self.config.translation_filter_alpha,
            rotation_alpha=self.config.rotation_filter_alpha,
        )
        self.filtered_target = left_target.copy(), right_target.copy()
        # Match AgileX questVR_ws: solve each absolute endpoint target to
        # convergence and warm-start from the previous IK solution. Robot
        # feedback is used only for the takeover anchor and final fail-closed
        # validation; it must not continuously rebuild the IK reference.
        seed = joints if self.last_solution is None else self.last_solution
        solve = self.solver.solve(seed, left_target, right_target)
        if not solve.valid or solve.joint_positions is None:
            return PiperRetargetResult("HOLD_IK", solve=solve)
        command = np.asarray(solve.joint_positions, dtype=float)
        solution_step = float(np.max(np.abs(command - seed)))
        if solution_step > self.config.max_solution_jump_rad:
            return PiperRetargetResult("HOLD_IK_JUMP", solve=solve)
        self.last_solution = command.copy()
        action = dict(state)
        action.update(zip(JOINT_NAMES, command, strict=True))
        action["left_gripper"] = float(
            np.clip(left_trigger * max_gripper_width_m, 0.0, max_gripper_width_m)
        )
        action["right_gripper"] = float(
            np.clip(right_trigger * max_gripper_width_m, 0.0, max_gripper_width_m)
        )
        return PiperRetargetResult(
            "READY_JOINT_TARGET",
            action=action_dict(action),
            solve=solve,
            left_delta_m=float(
                np.linalg.norm(left_target[:3, 3] - self.robot_reference[0][:3, 3])
            ),
            right_delta_m=float(
                np.linalg.norm(right_target[:3, 3] - self.robot_reference[1][:3, 3])
            ),
            max_solution_step_rad=solution_step,
            max_command_step_rad=solution_step,
        )
