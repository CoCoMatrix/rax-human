from __future__ import annotations

import numpy as np


class LowPassFilter:
    """Stateful low-pass stage that never mutates its input action."""

    def __init__(self, alpha: float):
        self.alpha = float(alpha)
        if not np.isfinite(self.alpha) or not 0.0 < self.alpha <= 1.0:
            raise ValueError("low-pass alpha must be in (0, 1]")
        self._state: np.ndarray | None = None

    def reset(self) -> None:
        self._state = None

    def process(
        self,
        action: np.ndarray,
        *,
        initial: np.ndarray | None = None,
    ) -> np.ndarray:
        action = _vector(action, "action")
        if self._state is None:
            previous = action if initial is None else _vector(initial, "initial")
            if previous.shape != action.shape:
                raise ValueError("initial action must match the action dimension")
        else:
            previous = self._state
        result = previous + self.alpha * (action - previous)
        self._state = result.astype(np.float32)
        return self._state.copy()


def _vector(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.ndim != 1 or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite vector")
    return result


def resample_action_chunk(
    chunk: np.ndarray,
    *,
    previous_action: np.ndarray,
    factor: int,
) -> np.ndarray:
    """PCHIP-resample native-rate targets without changing the input chunk."""

    chunk = np.asarray(chunk, dtype=np.float32)
    previous = _vector(previous_action, "previous action")
    if chunk.ndim != 2 or not len(chunk):
        raise ValueError(f"expected non-empty [T,D] action chunk, got {chunk.shape}")
    if previous.shape != (chunk.shape[1],):
        raise ValueError("previous action must match the chunk action dimension")
    if not np.isfinite(chunk).all():
        raise ValueError("action chunk contains non-finite values")
    if factor < 1:
        raise ValueError("spline resample factor must be positive")
    if factor == 1:
        return chunk.copy()

    from scipy.interpolate import PchipInterpolator

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


def smooth_action_chunk(
    chunk: np.ndarray,
    *,
    previous_action: np.ndarray | None = None,
    previous_previous_action: np.ndarray | None = None,
    old_overlap: np.ndarray | None = None,
    acceleration_weight: float = 0.25,
    overlap_weight: float = 4.0,
) -> tuple[np.ndarray, dict]:
    """Apply the verified FastWAM chunk QP without changing the raw chunk."""

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
            else float(
                np.sqrt(
                    np.mean(np.square(chunk[:overlap_steps] - overlap[:overlap_steps]))
                )
            )
        ),
        "overlap_error_rms_after_rad": (
            0.0
            if not overlap_steps
            else float(
                np.sqrt(
                    np.mean(
                        np.square(
                            smoothed[:overlap_steps] - overlap[:overlap_steps]
                        )
                    )
                )
            )
        ),
    }
    return smoothed, info
