"""Pure leader/follower-to-canonical-action conversion for dual PiPER.

The vendor SDK owns the leader/follower CAN protocol.  This module only turns
the leader feedback into the existing 14-dimensional Piper action contract;
it never opens a device or sends a command.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from .mapping import ACTION_NAMES, action_dict

ARM_ROLES = ("left_arm", "right_arm")
EXPERT_SOURCES = ("qnexo", "leader_follower")


@dataclass(frozen=True, slots=True)
class LeaderFollowerConfig:
    """Validated mapping for one physical leader/follower pair."""

    leader_role: str = "left_arm"
    follower_role: str = "right_arm"
    feedback_role: str = "right_arm"
    native_linkage: bool = False

    def __post_init__(self) -> None:
        for name, value in (
            ("leader_role", self.leader_role),
            ("follower_role", self.follower_role),
            ("feedback_role", self.feedback_role),
        ):
            if value not in ARM_ROLES:
                raise ValueError(f"{name} must be left_arm or right_arm")
        if self.leader_role == self.follower_role:
            raise ValueError("leader_role and follower_role must be different")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "LeaderFollowerConfig":
        data = dict(value or {})
        return cls(
            leader_role=str(data.get("leader_role", "left_arm")),
            follower_role=str(data.get("follower_role", "right_arm")),
            feedback_role=str(
                data.get("feedback_role", data.get("follower_role", "right_arm"))
            ),
            native_linkage=bool(data.get("native_linkage", False)),
        )


@dataclass(frozen=True, slots=True)
class LeaderJointFeedback:
    """A finite six-joint leader sample returned by the SDK."""

    joints_rad: tuple[float, ...]
    timestamp_s: float
    hz: float

    def __post_init__(self) -> None:
        if len(self.joints_rad) != 6 or not all(
            math.isfinite(value) for value in self.joints_rad
        ):
            raise ValueError("leader feedback must contain six finite joints")
        if not math.isfinite(self.timestamp_s) or not math.isfinite(self.hz):
            raise ValueError("leader feedback timestamp and rate must be finite")
        if self.hz < 0.0:
            raise ValueError("leader feedback rate must be non-negative")


def build_leader_follower_action(
    measured_state: Sequence[float] | Mapping[str, Any],
    leader_feedback: LeaderJointFeedback,
    leader_gripper_width_m: float,
    *,
    config: LeaderFollowerConfig,
) -> dict[str, float]:
    """Build a canonical absolute action without changing the leader arm.

    The current measured 14D state is copied first.  The leader's six joints
    and gripper are then placed in the configured follower slots.  This keeps
    the other arm at its measured pose and prevents an implicit left/right
    swap or a second delta-to-absolute conversion.
    """

    if isinstance(measured_state, Mapping):
        action = action_dict(measured_state)
    else:
        values = tuple(float(value) for value in measured_state)
        if len(values) != len(ACTION_NAMES):
            raise ValueError("measured_state must contain exactly 14 values")
        action = action_dict(dict(zip(ACTION_NAMES, values, strict=True)))
    width = float(leader_gripper_width_m)
    if not math.isfinite(width):
        raise ValueError("leader gripper width must be finite")
    side = config.follower_role.removesuffix("_arm")
    for index in range(1, 7):
        action[f"{side}_joint{index}"] = float(leader_feedback.joints_rad[index - 1])
    action[f"{side}_gripper"] = width
    return action


def parse_expert_source(value: Any) -> str:
    source = str(value or "qnexo").strip().lower()
    if source not in EXPERT_SOURCES:
        raise ValueError(
            f"PIPER_DAGGER_EXPERT_SOURCE must be one of {list(EXPERT_SOURCES)}"
        )
    return source
