from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from XPolicyLab.real_world.robots.gr3.mapping import (
    ACTION_NAMES,
    BASE_ACTION_NAMES,
    JOINT_NAMES,
)


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    status: str
    action: dict[str, float] | None
    reasons: tuple[str, ...]

    @property
    def accepted(self) -> bool:
        return self.action is not None


class Gr3SafetyGuard:
    """Final fail-HOLD gate before any GR3 command writer."""

    def __init__(
        self,
        *,
        max_joint_step_rad: float,
        max_state_age_ms: float,
        joint_limits: Mapping[str, tuple[float, float]],
        require_joint_limits: bool = True,
    ):
        if max_joint_step_rad <= 0 or max_state_age_ms <= 0:
            raise ValueError("joint step and state age limits must be positive")
        unknown = set(joint_limits) - set(JOINT_NAMES)
        if unknown:
            raise ValueError(f"unknown GR3 joint limits: {sorted(unknown)}")
        if any(len(bounds) != 2 for bounds in joint_limits.values()):
            raise ValueError("each joint limit must contain lower and upper bounds")
        limits = {
            name: (float(bounds[0]), float(bounds[1]))
            for name, bounds in joint_limits.items()
        }
        if any(lower > upper for lower, upper in limits.values()):
            raise ValueError("joint limit lower bound exceeds upper bound")

        self.max_joint_step_rad = float(max_joint_step_rad)
        self.max_state_age_ns = int(float(max_state_age_ms) * 1_000_000)
        self.joint_limits = limits
        self.require_joint_limits = bool(require_joint_limits)

    @classmethod
    def from_profile(
        cls,
        path: str | Path,
        *,
        require_joint_limits: bool | None = None,
        max_joint_step_rad: float | None = None,
    ) -> Gr3SafetyGuard:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        safety = profile["safety"]
        limits = {
            name: tuple(bounds)
            for name, bounds in safety.get("joint_limits", {}).items()
        }
        return cls(
            max_joint_step_rad=(
                safety["max_joint_step_rad"]
                if max_joint_step_rad is None
                else max_joint_step_rad
            ),
            max_state_age_ms=safety["max_state_age_ms"],
            joint_limits=limits,
            require_joint_limits=(
                safety.get("require_joint_limits", True)
                if require_joint_limits is None
                else require_joint_limits
            ),
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
        generation: int,
        expected_generation: int,
        state_timestamp_ns: int,
        now_ns: int,
    ) -> SafetyDecision:
        if not enabled:
            return self._hold("control_disabled")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        if state_timestamp_ns <= 0 or now_ns < state_timestamp_ns:
            return self._hold("invalid_state_timestamp")
        if now_ns - state_timestamp_ns > self.max_state_age_ns:
            return self._hold("stale_robot_state")
        if set(action) != set(ACTION_NAMES):
            return self._hold("invalid_action_schema")
        if not set(JOINT_NAMES).issubset(robot_state):
            return self._hold("invalid_state_schema")

        try:
            target = {name: float(action[name]) for name in ACTION_NAMES}
            current = {name: float(robot_state[name]) for name in JOINT_NAMES}
        except (TypeError, ValueError):
            return self._hold("non_numeric_value")
        if not np.isfinite(tuple(target.values())).all():
            return self._hold("non_finite_action")
        if not np.isfinite(tuple(current.values())).all():
            return self._hold("non_finite_state")

        missing_limits = [name for name in JOINT_NAMES if name not in self.joint_limits]
        if self.require_joint_limits and missing_limits:
            return self._hold("missing_joint_limits")
        for name, (lower, upper) in self.joint_limits.items():
            if not lower <= current[name] <= upper:
                return self._hold("state_outside_joint_limits")

        safe = dict(target)
        reasons: list[str] = []
        for name in JOINT_NAMES:
            desired = target[name]
            if name in self.joint_limits:
                lower, upper = self.joint_limits[name]
                clipped = min(max(desired, lower), upper)
                if clipped != desired:
                    reasons.append("joint_limit_clipped")
                desired = clipped
            delta = desired - current[name]
            clipped_delta = min(
                max(delta, -self.max_joint_step_rad),
                self.max_joint_step_rad,
            )
            if clipped_delta != delta:
                reasons.append("joint_step_limited")
            safe[name] = current[name] + clipped_delta

        for name in BASE_ACTION_NAMES:
            if safe[name] != 0.0:
                reasons.append("base_disabled")
            safe[name] = 0.0

        unique_reasons = tuple(dict.fromkeys(reasons))
        return SafetyDecision(
            "ADJUSTED" if unique_reasons else "PASS",
            {name: safe[name] for name in ACTION_NAMES},
            unique_reasons,
        )
