"""Robot-independent real-time action-chunk coordination.

This module owns the client-side part of real-time chunking: learning an
inference budget from measured latency and making a bounded transition from
the still-queued plan to a time-aligned replacement.  It deliberately knows
nothing about robot field names, model backends, hardware, or model-native RTC
guidance.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


class RtcMode(str, Enum):
    """Client-side real-time chunking mode."""

    DISABLED = "disabled"
    ALIGN = "align"
    BLEND = "blend"

    @classmethod
    def parse(cls, value: str | "RtcMode") -> "RtcMode":
        if isinstance(value, cls):
            return value
        aliases = {
            "off": cls.DISABLED.value,
            "none": cls.DISABLED.value,
            "0": cls.DISABLED.value,
            "false": cls.DISABLED.value,
            "prefetch": cls.ALIGN.value,
            "replace": cls.ALIGN.value,
            "on": cls.ALIGN.value,
            "1": cls.ALIGN.value,
            "true": cls.ALIGN.value,
            "enabled": cls.ALIGN.value,
            "merge": cls.BLEND.value,
        }
        raw = str(value).strip().lower()
        normalized = aliases.get(raw, raw)
        try:
            return cls(normalized)
        except ValueError as exc:
            raise ValueError("RTC mode must be disabled, align, or blend") from exc


class TransitionCurve(str, Enum):
    """Weight schedule used across the old/new chunk overlap."""

    LINEAR = "linear"
    SMOOTHSTEP = "smoothstep"
    COSINE = "cosine"

    @classmethod
    def parse(cls, value: str | "TransitionCurve") -> "TransitionCurve":
        if isinstance(value, cls):
            return value
        try:
            return cls(str(value).strip().lower())
        except ValueError as exc:
            raise ValueError(
                "RTC transition curve must be linear, smoothstep, or cosine"
            ) from exc


@dataclass(frozen=True, slots=True)
class RealTimeChunkingConfig:
    """Configuration shared by every robot-independent ActionChunk runtime.

    ``preserve_steps`` selects when the transition begins after a replacement
    arrives. ``transition_steps`` selects how many aligned old/new rows are
    merged. ``new_weight_start`` and ``new_weight_end`` select how strongly the
    replacement contributes across the chosen transition curve.

    ``request_remaining_steps`` is zero for latency-adaptive prefetching.  A
    positive value requests inference whenever that many queued actions remain,
    independently from the measured latency and blend length.  Latency is still
    measured and used to time-align the returned chunk.
    """

    mode: RtcMode | str = RtcMode.DISABLED
    latency_quantile: float = 0.95
    latency_margin_steps: int = 1
    latency_window_size: int = 32
    preserve_steps: int = 0
    transition_steps: int = 0
    transition_curve: TransitionCurve | str = TransitionCurve.SMOOTHSTEP
    new_weight_start: float = 0.0
    new_weight_end: float = 1.0
    request_remaining_steps: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "mode", RtcMode.parse(self.mode))
        object.__setattr__(self, "latency_quantile", float(self.latency_quantile))
        object.__setattr__(
            self,
            "latency_margin_steps",
            int(self.latency_margin_steps),
        )
        object.__setattr__(
            self,
            "latency_window_size",
            int(self.latency_window_size),
        )
        object.__setattr__(
            self,
            "request_remaining_steps",
            int(self.request_remaining_steps),
        )
        object.__setattr__(self, "preserve_steps", int(self.preserve_steps))
        object.__setattr__(self, "transition_steps", int(self.transition_steps))
        object.__setattr__(
            self,
            "transition_curve",
            TransitionCurve.parse(self.transition_curve),
        )
        object.__setattr__(self, "new_weight_start", float(self.new_weight_start))
        object.__setattr__(self, "new_weight_end", float(self.new_weight_end))
        if not 0.0 < self.latency_quantile <= 1.0:
            raise ValueError("RTC latency quantile must be in (0, 1]")
        if self.latency_margin_steps < 0:
            raise ValueError("RTC latency margin steps must be non-negative")
        if self.latency_window_size < 1:
            raise ValueError("RTC latency window size must be positive")
        if self.request_remaining_steps < 0:
            raise ValueError("RTC request remaining steps must be non-negative")
        if self.preserve_steps < 0 or self.transition_steps < 0:
            raise ValueError("RTC preserve/transition steps must be non-negative")
        start = self.new_weight_start
        end = self.new_weight_end
        if (
            not math.isfinite(start)
            or not math.isfinite(end)
            or not 0.0 <= start <= end <= 1.0
        ):
            raise ValueError(
                "RTC new-chunk weights must be finite and satisfy "
                "0 <= start <= end <= 1"
            )

    @classmethod
    def from_environ(
        cls,
        environ: Mapping[str, str],
        *,
        default: "RealTimeChunkingConfig | None" = None,
    ) -> "RealTimeChunkingConfig":
        """Load the common ``ACTION_RTC_*`` environment contract."""

        base = default or cls()

        def value(name: str, fallback: object) -> str:
            raw = environ.get(name)
            return str(fallback if raw is None or not str(raw).strip() else raw)

        return cls(
            mode=value("ACTION_RTC_MODE", base.mode.value),
            latency_quantile=float(
                value("ACTION_RTC_LATENCY_QUANTILE", base.latency_quantile)
            ),
            latency_margin_steps=int(
                value("ACTION_RTC_LATENCY_MARGIN_STEPS", base.latency_margin_steps)
            ),
            latency_window_size=int(
                value("ACTION_RTC_LATENCY_WINDOW_SIZE", base.latency_window_size)
            ),
            request_remaining_steps=int(
                value(
                    "ACTION_RTC_REQUEST_REMAINING_STEPS",
                    base.request_remaining_steps,
                )
            ),
            preserve_steps=int(
                value("ACTION_RTC_PRESERVE_STEPS", base.preserve_steps)
            ),
            transition_steps=int(
                value("ACTION_RTC_TRANSITION_STEPS", base.transition_steps)
            ),
            transition_curve=value(
                "ACTION_RTC_TRANSITION_CURVE", base.transition_curve.value
            ),
            new_weight_start=float(
                value("ACTION_RTC_NEW_WEIGHT_START", base.new_weight_start)
            ),
            new_weight_end=float(
                value("ACTION_RTC_NEW_WEIGHT_END", base.new_weight_end)
            ),
        )


@dataclass(frozen=True, slots=True)
class ChunkMerge:
    """A replacement chunk and evidence describing its transition."""

    values: np.ndarray
    applied: bool
    previous_steps: int
    preserved_steps: int
    transitioned_steps: int
    transition_curve: str
    boundary_linf_before: float | None
    boundary_linf_after: float | None

    def as_dict(self) -> dict[str, int | float | str | bool | None]:
        return {
            "rtc_merge_applied": self.applied,
            "rtc_previous_steps_at_accept": self.previous_steps,
            "rtc_preserved_steps": self.preserved_steps,
            "rtc_transition_steps": self.transitioned_steps,
            "rtc_transition_curve": self.transition_curve,
            "rtc_boundary_linf_before": self.boundary_linf_before,
            "rtc_boundary_linf_after": self.boundary_linf_after,
        }


def quintic_splice(previous: np.ndarray, replacement: np.ndarray, *,
                   blend_steps: int, continuous_indices) -> np.ndarray:
    """robot_infra quintic overlap transition; non-joint fields bypass it.

    Reference: CoCoMatrix/robot_infra@6376e4d4, async_inference.splice_action_prefix.
    Both inputs must already be aligned to the same execution cursor.
    """
    old = np.asarray(previous, dtype=np.float32)
    new = np.asarray(replacement, dtype=np.float32)
    if old.ndim != 2 or new.ndim != 2 or old.shape[1:] != new.shape[1:]:
        raise ValueError("Quintic splice requires matching [T,D] arrays")
    if not np.isfinite(old).all() or not np.isfinite(new).all():
        raise ValueError("Quintic splice requires finite inputs")
    indices = np.asarray(continuous_indices, dtype=np.intp).reshape(-1)
    if not len(indices) or np.any(indices < 0) or np.any(indices >= new.shape[1]) or len(np.unique(indices)) != len(indices):
        raise ValueError("Invalid quintic splice dimensions")
    if not 2 <= blend_steps <= min(len(old), len(new)):
        raise ValueError("Quintic blend steps must fit both prefixes")
    phase = np.linspace(0., 1., blend_steps, dtype=np.float64)
    weights = phase**3 * (10. + phase * (-15. + 6. * phase))
    output = new.copy()
    output[:blend_steps, indices] = (
        old[:blend_steps, indices] * (1. - weights[:, None])
        + new[:blend_steps, indices] * weights[:, None])
    return output


class RealTimeChunker:
    """Measured-latency trigger estimator and aligned chunk merger."""

    def __init__(self, config: RealTimeChunkingConfig):
        self.config = config
        self._latency_steps: deque[int] = deque(
            maxlen=config.latency_window_size
        )
        self.last_latency_quantile_steps = 0
        self.last_trigger_steps = 0

    @property
    def latency_samples_steps(self) -> tuple[int, ...]:
        return tuple(self._latency_steps)

    def observe_latency(
        self,
        delay_steps: int,
        *,
        horizon_steps: int,
        trigger_floor_steps: int = 0,
    ) -> int:
        """Return a prefetch depth learned from measured inference latency."""

        delay = int(delay_steps)
        horizon = int(horizon_steps)
        floor = int(trigger_floor_steps)
        if delay < 0 or horizon < 1 or floor < 0:
            raise ValueError("RTC delay/floor must be non-negative and horizon positive")
        self._latency_steps.append(delay)
        ordered = sorted(self._latency_steps)
        rank = max(
            0,
            int(math.ceil(self.config.latency_quantile * len(ordered))) - 1,
        )
        measured = ordered[rank]
        self.last_latency_quantile_steps = measured
        if self.config.request_remaining_steps:
            recommended = self.config.request_remaining_steps
        else:
            reserve = self.config.latency_margin_steps
            if self.config.mode is RtcMode.BLEND:
                reserve += self.config.preserve_steps + self.config.transition_steps
            recommended = measured + reserve
        recommended = min(max(0, horizon - 1), recommended)
        self.last_trigger_steps = max(floor, recommended)
        return self.last_trigger_steps

    def merge(self, previous: np.ndarray, replacement: np.ndarray) -> ChunkMerge:
        """Merge aligned ``[T,D]`` chunks without mutating either input."""

        candidate = self._chunk(replacement, "replacement", allow_empty=False)
        old = self._chunk(
            previous,
            "previous",
            action_dim=candidate.shape[1],
            allow_empty=True,
        )
        output = candidate.copy()
        curve = self.config.transition_curve.value
        if self.config.mode is not RtcMode.BLEND or not len(old):
            return ChunkMerge(output, False, len(old), 0, 0, curve, None, None)

        # Always leave at least one row owned entirely by the replacement.
        preserved = min(
            self.config.preserve_steps,
            len(old),
            max(0, len(output) - 1),
        )
        if preserved:
            output[:preserved] = old[:preserved]

        transitioned = min(
            self.config.transition_steps,
            max(0, len(old) - preserved),
            # Preserve at least one row that belongs wholly to the replacement.
            max(0, len(output) - preserved - 1),
        )
        before = None
        after = None
        if preserved < len(old) and preserved < len(output):
            before = float(
                np.max(np.abs(candidate[preserved] - old[preserved]))
            )

        if transitioned:
            progress = np.arange(1, transitioned + 1, dtype=np.float64) / (
                transitioned + 1
            )
            shaped = self._curve(progress)
            weights = self.config.new_weight_start + (
                self.config.new_weight_end - self.config.new_weight_start
            ) * shaped
            old_rows = old[preserved : preserved + transitioned]
            new_rows = candidate[preserved : preserved + transitioned]
            output[preserved : preserved + transitioned] = (
                (1.0 - weights[:, None]) * old_rows
                + weights[:, None] * new_rows
            ).astype(np.float32)

        if before is not None:
            after = float(
                np.max(np.abs(output[preserved] - old[preserved]))
            )
        applied = bool(preserved or transitioned)
        return ChunkMerge(
            output,
            applied,
            len(old),
            preserved,
            transitioned,
            curve,
            before,
            after,
        )

    def _curve(self, progress: np.ndarray) -> np.ndarray:
        if self.config.transition_curve is TransitionCurve.LINEAR:
            return progress
        if self.config.transition_curve is TransitionCurve.SMOOTHSTEP:
            return progress * progress * (3.0 - 2.0 * progress)
        return 0.5 - 0.5 * np.cos(np.pi * progress)

    @staticmethod
    def _chunk(
        value: np.ndarray,
        name: str,
        *,
        action_dim: int | None = None,
        allow_empty: bool,
    ) -> np.ndarray:
        result = np.asarray(value, dtype=np.float32)
        expected_dim = result.shape[1] if result.ndim == 2 else action_dim
        if (
            result.ndim != 2
            or (action_dim is not None and expected_dim != action_dim)
            or (not allow_empty and not len(result))
            or not np.isfinite(result).all()
        ):
            expected = "[T,D]" if action_dim is None else f"[T,{action_dim}]"
            raise ValueError(f"RTC {name} chunk must be finite {expected}")
        return result


__all__ = [
    "ChunkMerge",
    "RealTimeChunker",
    "RealTimeChunkingConfig",
    "RtcMode",
    "TransitionCurve",
]
