from __future__ import annotations

import time
from dataclasses import dataclass
from math import isfinite

import numpy as np

# Adapted from CoCoMatrix/robot_infra@6376e4d4efec9d873d10791b30e07fd3268fbb2b
# src/robot_infra/astribot/trajectory_smoothing.py. Numerical algorithm unchanged.
# XPolicyLab owns only ndarray validation and generic runtime integration.
FloatArray = np.ndarray


def as_float32(values):
    array = np.asarray(values, dtype=np.float32)
    if not np.isfinite(array).all():
        raise ValueError("tensor contains NaN or infinite values")
    return np.ascontiguousarray(array)

TERMINAL_SEAM_SECOND_DIFFERENCE_WEIGHT = 1.5


@dataclass(frozen=True)
class TrajectoryMetrics:
    max_step_rad: float
    acceleration_p95_rad_s2: float
    max_acceleration_rad_s2: float
    jerk_p95_rad_s3: float
    max_jerk_rad_s3: float


@dataclass(frozen=True)
class TrajectorySmoothingResult:
    values: FloatArray
    applied: bool
    prepare_ms: float
    max_target_deviation_rad: float
    before: TrajectoryMetrics
    after: TrajectoryMetrics
    fallback_reason: str | None = None


def trajectory_metrics(
    values: object,
    *,
    action_hz: float,
    continuous_indices: object | None = None,
) -> TrajectoryMetrics:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 2:
        raise ValueError("values must be a [T,D] array")
    if not isfinite(action_hz) or action_hz <= 0:
        raise ValueError("action_hz must be finite and positive")
    if continuous_indices is not None:
        array = array[:, np.asarray(continuous_indices, dtype=np.intp)]

    velocity = np.diff(array, axis=0) * action_hz
    acceleration = np.diff(velocity, axis=0) * action_hz
    jerk = np.diff(acceleration, axis=0) * action_hz
    return TrajectoryMetrics(
        max_step_rad=_max_abs(np.diff(array, axis=0)),
        acceleration_p95_rad_s2=_percentile_abs(acceleration, 95.0),
        max_acceleration_rad_s2=_max_abs(acceleration),
        jerk_p95_rad_s3=_percentile_abs(jerk, 95.0),
        max_jerk_rad_s3=_max_abs(jerk),
    )


def smooth_action_prefix(
    values: object,
    history: object,
    *,
    history_steps: int,
    smoothness_weight: float,
    fusion_decay: float,
    max_target_deviation_rad: float,
    action_hz: float,
    continuous_indices: object,
    preserve_first_target: bool = True,
) -> TrajectorySmoothingResult:
    """Fit a bounded smooth prefix while preserving all non-body dimensions.

    By default the first policy target is an exact seam anchor. Callers whose
    publisher already has a separate measured/accepted anchor may opt to fit
    that target too. Variable targets are solved jointly with a quadratic
    target-error and second-difference cost. A single global correction scale
    enforces the configured target-deviation cap without changing the direction
    of the deterministic solution.
    """

    started = time.perf_counter()
    raw = as_float32(values)
    if raw.ndim != 2 or raw.shape[0] == 0:
        raise ValueError("values must be a non-empty [T,D] array")
    indices = np.asarray(continuous_indices, dtype=np.intp).reshape(-1)
    if indices.size == 0 or np.any(indices < 0) or np.any(indices >= raw.shape[1]):
        raise ValueError("continuous_indices must select at least one valid dimension")
    history_array = as_float32(history)
    if history_array.ndim != 2 or history_array.shape[1] != raw.shape[1]:
        raise ValueError("history must be a [H,D] array with the same width as values")
    if type(history_steps) is not int or history_steps < 2:
        raise ValueError("history_steps must be an integer >= 2")
    for name, value, allow_zero in (
        ("smoothness_weight", smoothness_weight, True),
        ("fusion_decay", fusion_decay, True),
        ("max_target_deviation_rad", max_target_deviation_rad, False),
    ):
        if not isfinite(value) or value < 0 or (not allow_zero and value == 0):
            qualifier = "non-negative" if allow_zero else "positive"
            raise ValueError(f"{name} must be finite and {qualifier}")

    before = trajectory_metrics(raw, action_hz=action_hz, continuous_indices=indices)

    def finish(
        output: np.ndarray,
        *,
        applied: bool,
        deviation: float,
        fallback_reason: str | None = None,
    ) -> TrajectorySmoothingResult:
        return TrajectorySmoothingResult(
            values=output.astype(np.float32, copy=False),
            applied=applied,
            prepare_ms=(time.perf_counter() - started) * 1000.0,
            max_target_deviation_rad=deviation,
            before=before,
            after=trajectory_metrics(
                output,
                action_hz=action_hz,
                continuous_indices=indices,
            ),
            fallback_reason=fallback_reason,
        )

    # Two historical targets are the minimum needed to constrain acceleration.
    # With no usable history, returning the already-validated raw prefix keeps
    # startup behavior deterministic and avoids inventing a boundary velocity.
    retained_history = history_array[-history_steps:]
    if retained_history.shape[0] < 2 or raw.shape[0] < 2 or smoothness_weight == 0:
        return finish(raw.copy(), applied=False, deviation=0.0)

    body_history = retained_history[:, indices].astype(np.float64)
    body_raw = raw[:, indices].astype(np.float64)
    unheld_sequence = np.vstack((body_history, body_raw))
    if _max_abs(np.diff(unheld_sequence, n=2, axis=0)) <= 1e-7:
        return finish(raw.copy(), applied=False, deviation=0.0)
    # The accepted execution plan ends in a terminal hold. Including one
    # repeated terminal knot makes the fit reduce the velocity that would
    # otherwise be dropped abruptly at the next chunk boundary. The modest
    # fixed edge weight is part of the versioned rtg_v1 algorithm.
    fixed = np.vstack((body_history, body_raw, body_raw[-1:]))
    first_variable_offset = 1 if preserve_first_target else 0
    variable_count = raw.shape[0] - first_variable_offset
    mapping = np.zeros((fixed.shape[0], variable_count), dtype=np.float64)
    first_variable_row = body_history.shape[0] + first_variable_offset
    mapping[first_variable_row : first_variable_row + variable_count, :] = np.eye(
        variable_count, dtype=np.float64
    )
    mapping[-1, -1] = 1.0
    second_difference = np.diff(np.eye(fixed.shape[0], dtype=np.float64), n=2, axis=0)
    second_difference[-1] *= TERMINAL_SEAM_SECOND_DIFFERENCE_WEIGHT
    smooth_mapping = second_difference @ mapping
    target_weights = np.exp(
        -fusion_decay
        * np.arange(variable_count - 1, -1, -1, dtype=np.float64)
        / max(1, variable_count)
    )
    normal = np.diag(target_weights) + smoothness_weight * (smooth_mapping.T @ smooth_mapping)
    rhs = -smoothness_weight * (smooth_mapping.T @ (second_difference @ fixed))

    try:
        correction = np.linalg.solve(normal, rhs)
    except (FloatingPointError, np.linalg.LinAlgError, ValueError) as exc:
        return finish(
            raw.copy(),
            applied=False,
            deviation=0.0,
            fallback_reason=f"{type(exc).__name__}: {exc}",
        )
    if not np.all(np.isfinite(correction)):
        return finish(
            raw.copy(),
            applied=False,
            deviation=0.0,
            fallback_reason="non-finite quadratic solution",
        )

    maximum = _max_abs(correction)
    if maximum <= 1e-7:
        return finish(raw.copy(), applied=False, deviation=0.0)
    if maximum > max_target_deviation_rad:
        correction *= max_target_deviation_rad / maximum
    output = raw.copy()
    output[first_variable_offset:, indices] = (
        body_raw[first_variable_offset:] + correction
    ).astype(np.float32)
    if preserve_first_target:
        output[0] = raw[0]
    deviation = _max_abs(
        output[:, indices].astype(np.float64) - body_raw,
    )
    if not np.all(np.isfinite(output)) or deviation > max_target_deviation_rad + 1e-7:
        return finish(
            raw.copy(),
            applied=False,
            deviation=0.0,
            fallback_reason="bounded smoothing result failed finite/deviation checks",
        )
    return finish(
        output,
        applied=not np.array_equal(output, raw),
        deviation=deviation,
    )


def _max_abs(values: np.ndarray) -> float:
    return float(np.max(np.abs(values), initial=0.0))


def _percentile_abs(values: np.ndarray, percentile: float) -> float:
    if values.size == 0:
        return 0.0
    return float(np.percentile(np.abs(values), percentile))

