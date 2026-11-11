from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from ...output import LowPassFilter
from .mapping import LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES, action_dict


JOINT_NAMES = (*LEFT_JOINT_NAMES, *RIGHT_JOINT_NAMES)


@dataclass(frozen=True, slots=True)
class PiperModelFilterResult:
    action: dict[str, float]
    applied: bool
    initialized_from_measured: bool
    max_joint_adjustment_rad: float


class PiperModelLowPassFilter:
    """Stateful joint filter for RTC-coordinated dual-PiPER model actions.

    Gripper widths intentionally bypass this stage so filtering cannot delay a
    learned open/close event. The filter is initialized from fresh measured
    joints after every reset and remains continuous across action chunks.
    """

    def __init__(self, *, enabled: bool = True, alpha: float = 0.6) -> None:
        self.enabled = bool(enabled)
        self.alpha = float(alpha)
        # Construct even when disabled so invalid configuration always fails
        # closed instead of becoming a latent error when filtering is enabled.
        self._filter = LowPassFilter(self.alpha)
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    def reset(self) -> bool:
        was_initialized = self._initialized
        self._filter.reset()
        self._initialized = False
        return was_initialized

    def status(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "alpha": self.alpha,
            "fields": "arm_joints_only",
            "grippers": "bypass",
            "initialized": self._initialized,
            "placement": "post_rtc_post_boundary_pre_dispatch",
        }

    def process(
        self,
        action: Mapping[str, Any],
        *,
        measured_action: Mapping[str, Any],
    ) -> PiperModelFilterResult:
        target = action_dict(action)
        measured = action_dict(measured_action)
        if not self.enabled:
            return PiperModelFilterResult(
                action=target,
                applied=False,
                initialized_from_measured=False,
                max_joint_adjustment_rad=0.0,
            )

        target_joints = np.asarray(
            [target[name] for name in JOINT_NAMES], dtype=np.float32
        )
        measured_joints = np.asarray(
            [measured[name] for name in JOINT_NAMES], dtype=np.float32
        )
        initialized_from_measured = not self._initialized
        filtered_joints = self._filter.process(
            target_joints,
            initial=measured_joints,
        )
        self._initialized = True
        filtered = dict(target)
        for name, value in zip(JOINT_NAMES, filtered_joints, strict=True):
            filtered[name] = float(value)
        return PiperModelFilterResult(
            action=filtered,
            applied=True,
            initialized_from_measured=initialized_from_measured,
            max_joint_adjustment_rad=float(
                np.max(np.abs(filtered_joints - target_joints))
            ),
        )
