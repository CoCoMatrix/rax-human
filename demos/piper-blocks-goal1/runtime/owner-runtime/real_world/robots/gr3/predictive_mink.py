from __future__ import annotations

import os
import time
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator
from XPolicyLab.real_world.robots.gr3.mapping import JOINT_NAMES

ACTION_DIM = len(JOINT_NAMES)
DEFAULT_GR3_URDF = Path(__file__).with_name("description") / "gr3_control.urdf"
HAND_ACTION_INDICES = tuple(
    index
    for index, name in enumerate(JOINT_NAMES)
    if name.startswith(("L_", "R_"))
)
MINK_ACTION_INDICES = tuple(
    index for index in range(ACTION_DIM) if index not in HAND_ACTION_INDICES
)
LOWER_BODY_JOINTS = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in (
        "hip_pitch",
        "hip_roll",
        "hip_yaw",
        "knee_pitch",
        "ankle_pitch",
        "ankle_roll",
    )
)
MIMIC_JOINTS = {
    f"{side}_thumb_distal_joint": (f"{side}_thumb_proximal_pitch_joint", 1.06)
    for side in ("L", "R")
} | {
    f"{side}_{finger}_intermediate_joint": (
        f"{side}_{finger}_proximal_joint",
        0.975,
    )
    for side in ("L", "R")
    for finger in ("index", "middle", "ring", "pinky")
}
WRIST_FRAMES = ("left_hand_roll_link", "right_hand_roll_link")


class ProjectionError(RuntimeError):
    pass


class ProjectionTimeout(ProjectionError):
    pass


def _limit_vector(text: str, name: str, default: float) -> np.ndarray:
    values = np.fromstring(text or str(default), sep=",", dtype=float)
    if values.size == 1:
        values = np.full(ACTION_DIM, values.item())
    if values.shape != (ACTION_DIM,) or not np.isfinite(values).all():
        raise ValueError(f"{name} must contain one or {ACTION_DIM} finite values")
    if np.any(values <= 0):
        raise ValueError(f"{name} values must be positive")
    return values


@dataclass
class ProjectorConfig:
    dt: float = 1.0 / 30.0
    max_velocity: np.ndarray = field(
        default_factory=lambda: np.full(ACTION_DIM, 1.5)
    )
    max_acceleration: np.ndarray = field(
        default_factory=lambda: np.full(ACTION_DIM, 8.0)
    )
    max_jerk: np.ndarray = field(
        default_factory=lambda: np.full(ACTION_DIM, 80.0)
    )
    state_timeout_seconds: float = 0.1
    solve_timeout_ms: float = 30.0
    posture_cost: float = 10.0
    encoder_sync_cost: float = 1.0
    damping_cost: float = 1e-3
    horizon_steps: int = 7
    solver: str = "daqp"

    @classmethod
    def from_env(cls) -> ProjectorConfig:
        config = cls(
            dt=1.0 / float(os.getenv("OUTPUT_ACTION_HZ", "30")),
            max_velocity=_limit_vector(
                os.getenv("PREDICTIVE_MINK_MAX_VELOCITY_RAD_S", ""),
                "PREDICTIVE_MINK_MAX_VELOCITY_RAD_S",
                1.5,
            ),
            max_acceleration=_limit_vector(
                os.getenv("PREDICTIVE_MINK_MAX_ACCELERATION_RAD_S2", ""),
                "PREDICTIVE_MINK_MAX_ACCELERATION_RAD_S2",
                8.0,
            ),
            max_jerk=_limit_vector(
                os.getenv("PREDICTIVE_MINK_MAX_JERK_RAD_S3", ""),
                "PREDICTIVE_MINK_MAX_JERK_RAD_S3",
                80.0,
            ),
            state_timeout_seconds=float(
                os.getenv("PREDICTIVE_MINK_STATE_TIMEOUT_MS", "100")
            )
            / 1000.0,
            solve_timeout_ms=float(
                os.getenv("PREDICTIVE_MINK_SOLVE_TIMEOUT_MS", "30")
            ),
            posture_cost=float(
                os.getenv("PREDICTIVE_MINK_ACTION_TRACKING_COST", "10.0")
            ),
            encoder_sync_cost=float(
                os.getenv("PREDICTIVE_MINK_ENCODER_SYNC_COST", "1.0")
            ),
            horizon_steps=int(os.getenv("PREDICTIVE_MINK_HORIZON_STEPS", "7")),
        )
        scalars = (
            config.dt,
            config.state_timeout_seconds,
            config.solve_timeout_ms,
            config.posture_cost,
            config.encoder_sync_cost,
            config.damping_cost,
        )
        if not np.isfinite(scalars).all() or min(scalars) <= 0:
            raise ValueError("Predictive Mink timing and task costs must be positive")
        if config.horizon_steps < 1:
            raise ValueError("Predictive Mink horizon must be positive")
        return config


def _max_second_difference(
    chunk: np.ndarray,
    previous_action: np.ndarray | None,
    previous_previous_action: np.ndarray | None,
) -> float:
    history = []
    if previous_action is not None:
        previous_previous_action = (
            previous_action
            if previous_previous_action is None
            else previous_previous_action
        )
        history.extend((previous_previous_action, previous_action))
    trajectory = np.vstack((*history, chunk))
    if len(trajectory) < 3:
        return 0.0
    return float(np.max(np.abs(np.diff(trajectory, n=2, axis=0))))


def resample_action_chunk(
    chunk: np.ndarray,
    *,
    previous_action: np.ndarray,
    factor: int,
) -> np.ndarray:
    """PCHIP-resample native-rate targets without overshooting adjacent knots."""
    chunk = np.asarray(chunk, dtype=np.float32)
    previous = np.asarray(previous_action, dtype=np.float32)
    if chunk.ndim != 2 or not len(chunk):
        raise ValueError(f"expected non-empty [T,D] action chunk, got {chunk.shape}")
    if previous.shape != (chunk.shape[1],):
        raise ValueError("previous action must match the chunk action dimension")
    if not np.isfinite(chunk).all() or not np.isfinite(previous).all():
        raise ValueError("spline inputs contain non-finite values")
    if factor < 1:
        raise ValueError("spline resample factor must be positive")
    if factor == 1:
        return chunk.copy()

    knots = np.arange(len(chunk) + 1, dtype=np.float64)
    samples = np.arange(1, len(chunk) * factor + 1, dtype=np.float64) / factor
    dense = PchipInterpolator(
        knots,
        np.vstack((previous, chunk)),
        axis=0,
        extrapolate=False,
    )(samples).astype(np.float32)
    if not np.isfinite(dense).all():
        raise ValueError("spline resampling produced non-finite actions")
    return dense


def smooth_action_chunk(
    chunk: np.ndarray,
    *,
    previous_action: np.ndarray | None = None,
    previous_previous_action: np.ndarray | None = None,
    old_overlap: np.ndarray | None = None,
    acceleration_weight: float = 0.25,
    overlap_weight: float = 4.0,
) -> tuple[np.ndarray, dict]:
    chunk = np.asarray(chunk, dtype=np.float32)
    if chunk.ndim != 2 or not len(chunk):
        raise ValueError(f"expected non-empty [T,D] action chunk, got {chunk.shape}")
    if not np.isfinite(chunk).all():
        raise ValueError("action chunk contains non-finite values")
    if (
        not np.isfinite(acceleration_weight)
        or not np.isfinite(overlap_weight)
        or acceleration_weight < 0
        or overlap_weight < 0
    ):
        raise ValueError("QP smoothing weights must be non-negative")

    steps, action_dim = chunk.shape
    previous = None
    previous_previous = None
    if previous_action is not None:
        previous = np.asarray(previous_action, dtype=np.float64)
        previous_previous = np.asarray(
            previous if previous_previous_action is None else previous_previous_action,
            dtype=np.float64,
        )
        if previous.shape != (action_dim,) or previous_previous.shape != (action_dim,):
            raise ValueError("previous actions must match the chunk action dimension")
        if not np.isfinite(previous).all() or not np.isfinite(previous_previous).all():
            raise ValueError("previous actions contain non-finite values")

    hessian = np.eye(steps, dtype=np.float64)
    rhs = chunk.astype(np.float64, copy=True)
    if acceleration_weight:
        difference = np.zeros((steps, steps), dtype=np.float64)
        target = np.zeros((steps, action_dim), dtype=np.float64)
        if previous is not None:
            difference[0, 0] = 1.0
            target[0] = 2.0 * previous - previous_previous
            if steps > 1:
                difference[1, :2] = (-2.0, 1.0)
                target[1] = -previous
        for index in range(2, steps):
            difference[index, index - 2 : index + 1] = (1.0, -2.0, 1.0)
        hessian += acceleration_weight * difference.T @ difference
        rhs += acceleration_weight * difference.T @ target

    overlap_steps = 0
    overlap = None
    if old_overlap is not None:
        overlap = np.asarray(old_overlap, dtype=np.float64)
        if overlap.ndim != 2 or overlap.shape[1] != action_dim:
            raise ValueError("old overlap must match the chunk action dimension")
        if not np.isfinite(overlap).all():
            raise ValueError("old overlap contains non-finite values")
        overlap_steps = min(len(overlap), steps)
        if overlap_steps and overlap_weight:
            progress = np.linspace(0.0, 1.0, overlap_steps)
            fade = (
                np.ones(1)
                if overlap_steps == 1
                else 1.0 - (3.0 * progress**2 - 2.0 * progress**3)
            )
            weights = overlap_weight * fade
            indices = np.arange(overlap_steps)
            hessian[indices, indices] += weights
            rhs[:overlap_steps] += weights[:, None] * overlap[:overlap_steps]

    smoothed = np.linalg.solve(hessian, rhs).astype(np.float32)
    if not np.isfinite(smoothed).all():
        raise ValueError("QP smoothing produced non-finite actions")
    info = {
        "enabled": True,
        "acceleration_weight": acceleration_weight,
        "overlap_weight": overlap_weight,
        "overlap_steps": overlap_steps,
        "boundary_jump_before_rad": (
            0.0 if previous is None else float(np.max(np.abs(chunk[0] - previous)))
        ),
        "boundary_jump_after_rad": (
            0.0
            if previous is None
            else float(np.max(np.abs(smoothed[0] - previous)))
        ),
        "second_difference_before_rad": _max_second_difference(
            chunk, previous, previous_previous
        ),
        "second_difference_after_rad": _max_second_difference(
            smoothed, previous, previous_previous
        ),
        "overlap_error_rms_before_rad": (
            0.0
            if not overlap_steps
            else float(np.sqrt(np.mean(np.square(chunk[:overlap_steps] - overlap[:overlap_steps]))))
        ),
        "overlap_error_rms_after_rad": (
            0.0
            if not overlap_steps
            else float(
                np.sqrt(
                    np.mean(
                        np.square(smoothed[:overlap_steps] - overlap[:overlap_steps])
                    )
                )
            )
        ),
    }
    return smoothed, info


def select_action(
    mode: str,
    raw_action: np.ndarray,
    projected_action: np.ndarray | None,
) -> np.ndarray:
    if mode in {"off", "shadow"}:
        return raw_action
    if mode == "enforce" and projected_action is not None:
        return projected_action
    if mode != "enforce":
        raise ValueError("Predictive Mink mode must be off, shadow, or enforce")
    raise ProjectionError("enforce mode requires a safe projected action")


def _validated_action(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (ACTION_DIM,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must be {ACTION_DIM} finite joint values")
    return array


def _history_array(command_history: Sequence[np.ndarray]) -> np.ndarray:
    if len(command_history) == 0:
        return np.empty((0, ACTION_DIM))
    history = np.asarray(command_history, dtype=float)
    if (
        history.ndim != 2
        or history.shape[1] != ACTION_DIM
        or not np.isfinite(history).all()
    ):
        raise ValueError("command history must contain finite 31D actions")
    return history[-3:]


def trajectory_motion_metrics(
    trajectory: np.ndarray,
    dt: float,
    joint_indices: Sequence[int] = tuple(range(ACTION_DIM)),
) -> dict:
    trajectory = np.asarray(trajectory, dtype=float)
    if (
        trajectory.ndim != 2
        or trajectory.shape[1] != ACTION_DIM
        or not len(trajectory)
        or not np.isfinite(trajectory).all()
    ):
        raise ValueError(f"trajectory must be non-empty [T,{ACTION_DIM}] finite values")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("trajectory dt must be positive and finite")
    trajectory = trajectory[:, list(joint_indices)]
    velocity = np.diff(trajectory, axis=0) / dt
    acceleration = np.diff(velocity, axis=0) / dt
    jerk = np.diff(acceleration, axis=0) / dt
    return {
        "peak_velocity_rad_s": (
            0.0 if not len(velocity) else float(np.max(np.abs(velocity)))
        ),
        "peak_acceleration_rad_s2": (
            0.0 if not len(acceleration) else float(np.max(np.abs(acceleration)))
        ),
        "peak_jerk_rad_s3": (
            0.0 if not len(jerk) else float(np.max(np.abs(jerk)))
        ),
        "squared_jerk": (
            0.0 if not len(jerk) else float(np.sum(np.square(jerk)))
        ),
    }


def projection_chunk_metrics(
    raw_steps: Sequence[np.ndarray],
    projected_steps: Sequence[np.ndarray | None],
    step_info: Sequence[dict],
    dt: float,
) -> dict:
    if not (len(raw_steps) == len(projected_steps) == len(step_info)):
        raise ValueError("projection trace arrays must have the same length")
    raw = np.asarray(raw_steps, dtype=float)
    raw_smoothness = trajectory_motion_metrics(raw, dt, MINK_ACTION_INDICES)
    successful = [
        (index, _validated_action(projected, "projected"))
        for index, projected in enumerate(projected_steps)
        if projected is not None
    ]
    optimized_indices = np.asarray(MINK_ACTION_INDICES)
    errors = (
        np.vstack(
            [
                (projected - raw[index])[optimized_indices]
                for index, projected in successful
            ]
        )
        if successful
        else np.empty((0, len(MINK_ACTION_INDICES)))
    )
    if len(errors):
        worst_flat = int(np.argmax(np.abs(errors)))
        worst_joint = JOINT_NAMES[
            MINK_ACTION_INDICES[worst_flat % len(MINK_ACTION_INDICES)]
        ]
        tracking_rmse = float(np.sqrt(np.mean(np.square(errors))))
        tracking_max = float(np.max(np.abs(errors)))
    else:
        worst_joint = None
        tracking_rmse = None
        tracking_max = None

    failure_reasons: dict[str, int] = {}
    for info in step_info:
        if info.get("status") == "error":
            reason = str(info.get("fallback_reason", "unknown"))
            failure_reasons[reason] = failure_reasons.get(reason, 0) + 1
    solve_ms = np.asarray(
        [info["solve_ms"] for info in step_info if "solve_ms" in info],
        dtype=float,
    )
    objective_build_ms = np.asarray(
        [
            info["objective_build_ms"]
            for info in step_info
            if "objective_build_ms" in info
        ],
        dtype=float,
    )
    qp_solver_ms = np.asarray(
        [info["qp_solver_ms"] for info in step_info if "qp_solver_ms" in info],
        dtype=float,
    )
    state_age_ms = np.asarray(
        [info["state_age_ms"] for info in step_info if "state_age_ms" in info],
        dtype=float,
    )
    projected_smoothness = (
        trajectory_motion_metrics(
            np.vstack([projected for _, projected in successful]),
            dt,
            MINK_ACTION_INDICES,
        )
        if len(successful) == len(raw_steps)
        else None
    )
    def error_summary(name: str) -> dict | None:
        values = np.asarray(
            [info[name] for info in step_info if name in info],
            dtype=float,
        )
        if not len(values):
            return None
        return {
            "mean": float(np.mean(values)),
            "max": float(np.max(values)),
            "final": float(values[-1]),
        }

    return {
        "steps": len(raw_steps),
        "projected_success_steps": len(successful),
        "projected_failure_steps": len(raw_steps) - len(successful),
        "tracking_rmse_rad": tracking_rmse,
        "tracking_max_error_rad": tracking_max,
        "tracking_worst_joint": worst_joint,
        "raw_smoothness": raw_smoothness,
        "projected_smoothness": projected_smoothness,
        "left_end_effector_error": {
            "position_m": error_summary("left_position_error_m"),
            "orientation_rad": error_summary("left_orientation_error_rad"),
        },
        "right_end_effector_error": {
            "position_m": error_summary("right_position_error_m"),
            "orientation_rad": error_summary("right_orientation_error_rad"),
        },
        "solve_ms_mean": None if not len(solve_ms) else float(np.mean(solve_ms)),
        "solve_ms_p95": (
            None if not len(solve_ms) else float(np.percentile(solve_ms, 95))
        ),
        "solve_ms_max": None if not len(solve_ms) else float(np.max(solve_ms)),
        "objective_build_ms_mean": (
            None
            if not len(objective_build_ms)
            else float(np.mean(objective_build_ms))
        ),
        "objective_build_ms_max": (
            None
            if not len(objective_build_ms)
            else float(np.max(objective_build_ms))
        ),
        "qp_solver_ms_mean": (
            None if not len(qp_solver_ms) else float(np.mean(qp_solver_ms))
        ),
        "qp_solver_ms_max": (
            None if not len(qp_solver_ms) else float(np.max(qp_solver_ms))
        ),
        "state_age_ms_mean": (
            None if not len(state_age_ms) else float(np.mean(state_age_ms))
        ),
        "state_age_ms_max": (
            None if not len(state_age_ms) else float(np.max(state_age_ms))
        ),
        "fallback_reasons": failure_reasons,
    }


def rate_limit_action(
    candidate: np.ndarray,
    q_enc: np.ndarray,
    command_history: Sequence[np.ndarray],
    position_lower: np.ndarray,
    position_upper: np.ndarray,
    config: ProjectorConfig,
) -> np.ndarray:
    candidate = _validated_action(candidate, "candidate")
    q_enc = _validated_action(q_enc, "q_enc")
    indices = np.asarray(MINK_ACTION_INDICES)
    limited_candidate = candidate[indices]
    q_enc = q_enc[indices]
    history = _history_array(command_history)[:, indices]
    position_lower = np.asarray(position_lower)[indices]
    position_upper = np.asarray(position_upper)[indices]
    max_velocity = config.max_velocity[indices]
    max_acceleration = config.max_acceleration[indices]
    max_jerk = config.max_jerk[indices]
    history = np.clip(history, position_lower, position_upper)

    anchor = history[-1] if len(history) else q_enc
    lower = np.maximum(position_lower, anchor - max_velocity * config.dt)
    upper = np.minimum(position_upper, anchor + max_velocity * config.dt)
    if len(history) >= 2:
        previous_velocity = (history[-1] - history[-2]) / config.dt
        velocity_lower = previous_velocity - max_acceleration * config.dt
        velocity_upper = previous_velocity + max_acceleration * config.dt
        lower = np.maximum(lower, history[-1] + velocity_lower * config.dt)
        upper = np.minimum(upper, history[-1] + velocity_upper * config.dt)
    if len(history) >= 3:
        older_velocity = (history[-2] - history[-3]) / config.dt
        previous_acceleration = (previous_velocity - older_velocity) / config.dt
        acceleration_lower = previous_acceleration - max_jerk * config.dt
        acceleration_upper = previous_acceleration + max_jerk * config.dt
        lower = np.maximum(
            lower,
            history[-1]
            + (previous_velocity + acceleration_lower * config.dt) * config.dt,
        )
        upper = np.minimum(
            upper,
            history[-1]
            + (previous_velocity + acceleration_upper * config.dt) * config.dt,
        )
    infeasible = lower > upper + 1e-9
    if np.any(infeasible):
        index = int(np.argmax(np.where(infeasible, lower - upper, -np.inf)))
        action_index = indices[index]
        raise ProjectionError(
            "command history leaves no feasible rate-limited action "
            f"for {JOINT_NAMES[action_index]} "
            f"(lower={lower[index]:.6f}, upper={upper[index]:.6f}, "
            f"gap={lower[index] - upper[index]:.6f})"
        )
    projected = candidate.copy()
    projected[indices] = np.clip(limited_candidate, lower, upper)
    return projected.astype(np.float32)


def _motion_metrics(
    action: np.ndarray,
    command_history: Sequence[np.ndarray],
    dt: float,
) -> dict:
    history = _history_array(command_history)
    return trajectory_motion_metrics(
        np.vstack((history, action)),
        dt,
        MINK_ACTION_INDICES,
    )


def _se3_error(current, target) -> tuple[float, float]:
    current_matrix = current.as_matrix()
    target_matrix = target.as_matrix()
    position = float(np.linalg.norm(current_matrix[:3, 3] - target_matrix[:3, 3]))
    rotation = current_matrix[:3, :3].T @ target_matrix[:3, :3]
    angle = float(np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)))
    return position, angle


def _validated_chunk(chunk: np.ndarray, name: str = "chunk") -> np.ndarray:
    chunk = np.asarray(chunk, dtype=float)
    if (
        chunk.ndim != 2
        or chunk.shape[1] != ACTION_DIM
        or not len(chunk)
        or not np.isfinite(chunk).all()
    ):
        raise ValueError(f"{name} must be non-empty [T,{ACTION_DIM}] finite values")
    return chunk


def _affine_difference(
    steps: int,
    joint_count: int,
    order: int,
    known: dict[int, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    coefficients = {
        1: (-1.0, 1.0),
        2: (1.0, -2.0, 1.0),
        3: (-1.0, 3.0, -3.0, 1.0),
    }[order]
    difference = np.zeros((steps, steps), dtype=float)
    offset = np.zeros((steps, joint_count), dtype=float)
    for row, time_index in enumerate(range(1, steps + 1)):
        for coefficient, sample_index in zip(
            coefficients,
            range(time_index - order, time_index + 1),
            strict=True,
        ):
            if sample_index > 0:
                difference[row, sample_index - 1] += coefficient
            else:
                offset[row] += coefficient * known[sample_index]
    return np.kron(difference, np.eye(joint_count)), offset.reshape(-1)


def _add_squared_cost(
    hessian: np.ndarray,
    linear: np.ndarray,
    matrix: np.ndarray,
    offset: np.ndarray,
    weight: float,
) -> None:
    hessian += 2.0 * weight * matrix.T @ matrix
    linear += 2.0 * weight * matrix.T @ offset


def _error_summary(values: Sequence[float]) -> dict:
    array = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(array)),
        "max": float(np.max(array)),
        "final": float(array[-1]),
    }


class PredictiveMinkProjector:
    def __init__(
        self,
        config: ProjectorConfig | None = None,
        *,
        model_path: str | None = None,
    ):
        try:
            import mink
            import mujoco
            from qpsolvers import solve_qp
        except ImportError as exc:
            raise RuntimeError(
                "Predictive Mink requires fastwam-policy[predictive]"
            ) from exc

        self.config = config or ProjectorConfig.from_env()
        if model_path is None:
            model_path = str(DEFAULT_GR3_URDF)
        self.mink = mink
        self.mujoco = mujoco
        self.solve_qp = solve_qp
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.configuration = mink.Configuration(self.model)
        self.target_configuration = mink.Configuration(self.model)

        model_names = [
            mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
            )
            for joint_id in range(self.model.njnt)
        ]
        if len(model_names) != len(set(model_names)):
            raise ValueError("GR3 model contains duplicate joint names")
        missing = [name for name in JOINT_NAMES if name not in model_names]
        if missing:
            raise ValueError(f"GR3 model is missing action joints: {missing}")
        missing_lower = [name for name in LOWER_BODY_JOINTS if name not in model_names]
        if missing_lower:
            raise ValueError(f"GR3 model is missing lower-body joints: {missing_lower}")
        missing_mimic = [
            name
            for follower, (leader, _) in MIMIC_JOINTS.items()
            for name in (follower, leader)
            if name not in model_names
        ]
        if missing_mimic:
            raise ValueError(f"GR3 model is missing mimic joints: {missing_mimic}")

        self.action_qpos = np.asarray(
            [self.model.jnt_qposadr[self.model.joint(name).id] for name in JOINT_NAMES]
        )
        self.action_dof = np.asarray(
            [self.model.jnt_dofadr[self.model.joint(name).id] for name in JOINT_NAMES]
        )
        self.lower_qpos = np.asarray(
            [
                self.model.jnt_qposadr[self.model.joint(name).id]
                for name in LOWER_BODY_JOINTS
            ]
        )
        self.position_lower = np.asarray(
            [self.model.jnt_range[self.model.joint(name).id, 0] for name in JOINT_NAMES]
        )
        self.position_upper = np.asarray(
            [self.model.jnt_range[self.model.joint(name).id, 1] for name in JOINT_NAMES]
        )
        self.optimized_action_indices = np.asarray(MINK_ACTION_INDICES)
        self.terminal_velocity = np.minimum(
            self.config.max_velocity[self.optimized_action_indices],
            0.2,
        )
        self.terminal_acceleration = np.minimum(
            self.config.max_acceleration[self.optimized_action_indices],
            self.config.max_jerk[self.optimized_action_indices] * self.config.dt,
        )
        self.terminal_velocity_time = (
            self.terminal_velocity / self.terminal_acceleration + self.config.dt
        )
        self.terminal_acceleration_time2 = (
            self.terminal_velocity
            * self.config.dt
            / self.terminal_acceleration
            + self.config.dt**2
        )
        self._warm_horizon: np.ndarray | None = None

    def _set_mimic(self, q: np.ndarray) -> None:
        for follower, (leader, multiplier) in MIMIC_JOINTS.items():
            follower_adr = self.model.jnt_qposadr[self.model.joint(follower).id]
            leader_adr = self.model.jnt_qposadr[self.model.joint(leader).id]
            q[follower_adr] = q[leader_adr] * multiplier

    def model_q(self, action: np.ndarray, *, clip: bool = True) -> np.ndarray:
        action = _validated_action(action, "action")
        q = self.model.qpos0.copy()
        q[self.action_qpos] = (
            np.clip(action, self.position_lower, self.position_upper)
            if clip
            else action
        )
        self._set_mimic(q)
        q[self.lower_qpos] = self.model.qpos0[self.lower_qpos]
        return q

    def reset(self) -> None:
        self._warm_horizon = None

    def _boundary(
        self,
        q_enc: np.ndarray,
        command_history: Sequence[np.ndarray],
    ) -> dict[int, np.ndarray]:
        indices = self.optimized_action_indices
        history = _history_array(command_history)[:, indices]
        q0 = history[-1] if len(history) else q_enc[indices]
        velocity = (
            np.zeros_like(q0)
            if len(history) < 2
            else (history[-1] - history[-2]) / self.config.dt
        )
        acceleration = (
            np.zeros_like(q0)
            if len(history) < 3
            else (
                velocity - (history[-2] - history[-3]) / self.config.dt
            )
            / self.config.dt
        )
        q_minus_one = q0 - velocity * self.config.dt
        q_minus_two = q_minus_one - (
            velocity - acceleration * self.config.dt
        ) * self.config.dt
        return {0: q0, -1: q_minus_one, -2: q_minus_two}

    def _constraint_system(
        self,
        steps: int,
        known: dict[int, np.ndarray],
    ) -> tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        joint_count = len(self.optimized_action_indices)
        first, first_offset = _affine_difference(steps, joint_count, 1, known)
        second, second_offset = _affine_difference(steps, joint_count, 2, known)
        third, third_offset = _affine_difference(steps, joint_count, 3, known)
        constraints = []
        bounds = []
        for matrix, offset, limit in (
            (
                first,
                first_offset,
                np.tile(
                    self.config.max_velocity[self.optimized_action_indices]
                    * self.config.dt,
                    steps,
                ),
            ),
            (
                second,
                second_offset,
                np.tile(
                    self.config.max_acceleration[self.optimized_action_indices]
                    * self.config.dt**2,
                    steps,
                ),
            ),
            (
                third,
                third_offset,
                np.tile(
                    self.config.max_jerk[self.optimized_action_indices]
                    * self.config.dt**3,
                    steps,
                ),
            ),
        ):
            constraints.extend((matrix, -matrix))
            bounds.extend((limit - offset, limit + offset))
        terminal = slice((steps - 1) * joint_count, steps * joint_count)
        for matrix, offset, limit in (
            (
                first,
                first_offset,
                self.terminal_velocity * self.config.dt,
            ),
            (
                second,
                second_offset,
                self.terminal_acceleration * self.config.dt**2,
            ),
        ):
            constraints.extend((matrix[terminal], -matrix[terminal]))
            bounds.extend(
                (
                    limit - offset[terminal],
                    limit + offset[terminal],
                )
            )
        terminal_position = np.zeros((joint_count, steps * joint_count))
        terminal_position[:, terminal] = np.eye(joint_count)
        terminal_position += (
            self.terminal_velocity_time[:, None]
            * first[terminal]
            / self.config.dt
        )
        terminal_position += (
            self.terminal_acceleration_time2[:, None]
            * second[terminal]
            / self.config.dt**2
        )
        terminal_position_offset = (
            self.terminal_velocity_time * first_offset[terminal] / self.config.dt
            + self.terminal_acceleration_time2
            * second_offset[terminal]
            / self.config.dt**2
        )
        terminal_lower = self.position_lower[self.optimized_action_indices]
        terminal_upper = self.position_upper[self.optimized_action_indices]
        constraints.extend((terminal_position, -terminal_position))
        bounds.extend(
            (
                terminal_upper - terminal_position_offset,
                -terminal_lower + terminal_position_offset,
            )
        )
        lower = np.tile(
            self.position_lower[self.optimized_action_indices],
            steps,
        )
        upper = np.tile(
            self.position_upper[self.optimized_action_indices],
            steps,
        )
        return (
            first,
            first_offset,
            second,
            second_offset,
            third,
            third_offset,
            np.vstack(constraints),
            np.concatenate(bounds),
            lower,
            upper,
        )

    def _initial_nominal(
        self,
        references: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> np.ndarray:
        nominal = references[:, self.optimized_action_indices].copy()
        if self._warm_horizon is not None and len(self._warm_horizon):
            shifted = np.vstack(
                (self._warm_horizon[1:], self._warm_horizon[-1:])
            )
            overlap = min(len(shifted), len(nominal))
            nominal[:overlap] = shifted[:overlap]
        return np.clip(nominal.reshape(-1), lower, upper).reshape(nominal.shape)

    def _end_effector_errors(
        self,
        optimized: np.ndarray,
        references: np.ndarray,
    ) -> dict[str, list[tuple[float, float]]]:
        errors = {"left": [], "right": [], "relative": []}
        for action, reference in zip(optimized, references, strict=True):
            self.configuration.update(self.model_q(action, clip=False))
            self.target_configuration.update(self.model_q(reference, clip=False))
            errors["left"].append(
                _se3_error(
                    self.configuration.get_transform_frame_to_world(
                        WRIST_FRAMES[0], "body"
                    ),
                    self.target_configuration.get_transform_frame_to_world(
                        WRIST_FRAMES[0], "body"
                    ),
                )
            )
            errors["right"].append(
                _se3_error(
                    self.configuration.get_transform_frame_to_world(
                        WRIST_FRAMES[1], "body"
                    ),
                    self.target_configuration.get_transform_frame_to_world(
                        WRIST_FRAMES[1], "body"
                    ),
                )
            )
            errors["relative"].append(
                _se3_error(
                    self.configuration.get_transform(
                        WRIST_FRAMES[1], "body", WRIST_FRAMES[0], "body"
                    ),
                    self.target_configuration.get_transform(
                        WRIST_FRAMES[1], "body", WRIST_FRAMES[0], "body"
                    ),
                )
            )
        return errors

    def project_horizon(
        self,
        references: np.ndarray,
        q_enc: np.ndarray,
        command_history: Sequence[np.ndarray] = (),
        *,
        state_age_seconds: float = 0.0,
    ) -> tuple[np.ndarray, dict]:
        references = _validated_chunk(references, "references")[
            : self.config.horizon_steps
        ]
        reference_steps = len(references)
        planning_references = references
        if reference_steps < self.config.horizon_steps:
            planning_references = np.vstack(
                (
                    references,
                    np.repeat(
                        references[-1:],
                        self.config.horizon_steps - reference_steps,
                        axis=0,
                    ),
                )
            )
        q_enc = _validated_action(q_enc, "q_enc")
        if (
            not np.isfinite(state_age_seconds)
            or state_age_seconds < 0
            or state_age_seconds > self.config.state_timeout_seconds
        ):
            raise ProjectionError(
                f"encoder state is stale ({state_age_seconds * 1000.0:.1f} ms)"
            )

        steps = len(planning_references)
        joint_count = len(self.optimized_action_indices)
        known = self._boundary(q_enc, command_history)
        (
            first,
            first_offset,
            _,
            _,
            _,
            _,
            inequality,
            inequality_bound,
            lower,
            upper,
        ) = self._constraint_system(steps, known)
        initial = self._initial_nominal(planning_references, lower, upper)
        solution = initial.reshape(-1)
        started = time.perf_counter()
        objective_started = time.perf_counter()
        variable_count = steps * joint_count
        hessian = np.eye(variable_count) * 1e-8
        linear = np.zeros(variable_count)
        q_enc_step = q_enc[self.optimized_action_indices]
        for index, reference in enumerate(planning_references):
            block = slice(index * joint_count, (index + 1) * joint_count)
            reference_step = reference[self.optimized_action_indices]
            block_hessian = np.zeros((joint_count, joint_count))
            block_linear = np.zeros(joint_count)
            if index < reference_steps:
                block_hessian += 2.0 * self.config.posture_cost * np.eye(joint_count)
                block_linear -= (
                    2.0 * self.config.posture_cost * reference_step
                )
            if index == 0:
                block_hessian += (
                    2.0 * self.config.encoder_sync_cost * np.eye(joint_count)
                )
                block_linear -= 2.0 * self.config.encoder_sync_cost * q_enc_step
            hessian[block, block] += block_hessian
            linear[block] += block_linear

        _add_squared_cost(
            hessian,
            linear,
            first,
            first_offset,
            self.config.damping_cost,
        )
        hessian = 0.5 * (hessian + hessian.T)
        objective_build_ms = (time.perf_counter() - objective_started) * 1000.0
        remaining_seconds = (
            self.config.solve_timeout_ms
            - (time.perf_counter() - started) * 1000.0
        ) / 1000.0
        if remaining_seconds <= 0:
            raise ProjectionTimeout(
                f"Horizon solve exceeded {self.config.solve_timeout_ms:.1f} ms"
            )
        qp_started = time.perf_counter()
        solution = self.solve_qp(
            hessian,
            linear,
            inequality,
            inequality_bound,
            lb=lower,
            ub=upper,
            solver=self.config.solver,
            initvals=solution,
            time_limit=remaining_seconds,
        )
        qp_solver_ms = (time.perf_counter() - qp_started) * 1000.0
        if solution is None or not np.isfinite(solution).all():
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            boundary_velocity = (known[0] - known[-1]) / self.config.dt
            boundary_acceleration = (
                known[0] - 2.0 * known[-1] + known[-2]
            ) / self.config.dt**2
            detail = (
                f"boundary_v={np.max(np.abs(boundary_velocity)):.3f} rad/s, "
                f"boundary_a={np.max(np.abs(boundary_acceleration)):.3f} rad/s^2"
            )
            if elapsed_ms >= 0.9 * self.config.solve_timeout_ms:
                raise ProjectionTimeout(
                    "Horizon QP did not finish within "
                    f"{self.config.solve_timeout_ms:.1f} ms ({detail})"
                )
            raise ProjectionError(f"Horizon QP is infeasible ({detail})")
        nominal = solution.reshape(steps, joint_count)

        solve_ms = (time.perf_counter() - started) * 1000.0
        if solve_ms > self.config.solve_timeout_ms:
            raise ProjectionTimeout(
                "Horizon solve exceeded "
                f"{self.config.solve_timeout_ms:.1f} ms "
                f"(elapsed={solve_ms:.3f} ms)"
            )
        violation = max(
            float(np.max(inequality @ solution - inequality_bound)),
            float(np.max(lower - solution)),
            float(np.max(solution - upper)),
        )
        if violation > 1e-6:
            raise ProjectionError(
                f"Horizon QP violates a hard constraint by {violation:.3e}"
            )

        optimized = planning_references.copy()
        optimized[:, self.optimized_action_indices] = nominal
        self._warm_horizon = nominal.copy()
        optimized = optimized[:reference_steps].astype(np.float32)

        errors = self._end_effector_errors(optimized, references)
        first_left = errors["left"][0]
        first_right = errors["right"][0]
        first_relative = errors["relative"][0]
        tracking_error = (
            optimized[0, self.optimized_action_indices]
            - references[0, self.optimized_action_indices]
        )
        joint_margin = np.minimum(
            nominal
            - self.position_lower[self.optimized_action_indices],
            self.position_upper[self.optimized_action_indices] - nominal,
        )
        command_boundary = q_enc.copy()
        command_boundary[self.optimized_action_indices] = known[0]
        planned_motion = trajectory_motion_metrics(
            np.vstack((command_boundary, optimized)),
            self.config.dt,
            MINK_ACTION_INDICES,
        )
        encoder_tracking_error = (
            known[0] - q_enc[self.optimized_action_indices]
        )
        horizon_error = {
            side: {
                "position_m": _error_summary([error[0] for error in errors[side]]),
                "orientation_rad": _error_summary(
                    [error[1] for error in errors[side]]
                ),
            }
            for side in ("left", "right", "relative")
        }
        info = {
            "status": "safe",
            "solver": self.config.solver,
            "solve_ms": solve_ms,
            "objective_build_ms": objective_build_ms,
            "qp_solver_ms": qp_solver_ms,
            "horizon_steps": reference_steps,
            "planning_steps": steps,
            "state_age_ms": state_age_seconds * 1000.0,
            "encoder_tracking_rmse_rad": float(
                np.sqrt(np.mean(np.square(encoder_tracking_error)))
            ),
            "encoder_tracking_max_error_rad": float(
                np.max(np.abs(encoder_tracking_error))
            ),
            "tracking_rmse_rad": float(
                np.sqrt(np.mean(np.square(tracking_error)))
            ),
            "tracking_max_error_rad": float(np.max(np.abs(tracking_error))),
            "left_position_error_m": first_left[0],
            "left_orientation_error_rad": first_left[1],
            "right_position_error_m": first_right[0],
            "right_orientation_error_rad": first_right[1],
            "relative_position_error_m": first_relative[0],
            "relative_orientation_error_rad": first_relative[1],
            "end_effector_error_definition": (
                "FK(q_optimized) relative to FK(q_action_chunk) at the same node"
            ),
            "horizon_end_effector_error": horizon_error,
            "horizon_smoothness": planned_motion,
            "min_joint_margin_rad": float(np.min(joint_margin)),
            "min_collision_distance_m": None,
            **_motion_metrics(optimized[0], command_history, self.config.dt),
        }
        return optimized, info

    def project_step(
        self,
        reference: np.ndarray,
        q_enc: np.ndarray,
        command_history: Sequence[np.ndarray] = (),
        *,
        state_age_seconds: float = 0.0,
    ) -> tuple[np.ndarray, dict]:
        optimized, info = self.project_horizon(
            np.asarray(reference, dtype=float)[None, :],
            q_enc,
            command_history,
            state_age_seconds=state_age_seconds,
        )
        return optimized[0], info

    def project_chunk(
        self,
        chunk: np.ndarray,
        q_enc: np.ndarray,
        command_history: Sequence[np.ndarray] = (),
    ) -> tuple[np.ndarray, dict]:
        chunk = _validated_chunk(chunk)
        current = _validated_action(q_enc, "q_enc")
        history = list(_history_array(command_history))
        projected = []
        step_info = []
        started = time.perf_counter()
        for index, reference in enumerate(chunk):
            try:
                horizon, info = self.project_horizon(
                    chunk[index:],
                    current,
                    history,
                )
            except ProjectionError as exc:
                raise ProjectionError(f"chunk step {index}: {exc}") from exc
            action = horizon[0]
            projected.append(action)
            step_info.append(info)
            history.append(action)
            history = history[-3:]
            current = action
        result = np.asarray(projected, dtype=np.float32)
        return result, {
            "status": "safe",
            "steps": len(result),
            "total_solve_ms": (time.perf_counter() - started) * 1000.0,
            "tracking_rmse_rad": float(
                np.sqrt(
                    np.mean(
                        np.square(
                            result[:, list(MINK_ACTION_INDICES)]
                            - chunk[:, list(MINK_ACTION_INDICES)]
                        )
                    )
                )
            ),
            "tracking_max_error_rad": float(
                np.max(
                    np.abs(
                        result[:, list(MINK_ACTION_INDICES)]
                        - chunk[:, list(MINK_ACTION_INDICES)]
                    )
                )
            ),
            "peak_velocity_rad_s": max(
                info["peak_velocity_rad_s"] for info in step_info
            ),
            "peak_acceleration_rad_s2": max(
                info["peak_acceleration_rad_s2"] for info in step_info
            ),
            "peak_jerk_rad_s3": max(
                info["peak_jerk_rad_s3"] for info in step_info
            ),
            "squared_jerk": sum(info["squared_jerk"] for info in step_info),
            "min_joint_margin_rad": min(
                info["min_joint_margin_rad"] for info in step_info
            ),
        }
