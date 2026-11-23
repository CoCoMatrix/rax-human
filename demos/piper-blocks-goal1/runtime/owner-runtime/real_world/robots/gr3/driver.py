from __future__ import annotations

from collections.abc import Callable, Collection, Mapping
from dataclasses import dataclass
import math
from typing import Any

from XPolicyLab.real_world.robots.gr3.mapping import (
    ACTION_NAMES,
    BASE_ACTION_NAMES,
    JOINT_NAMES,
)


CONTROL_GROUPS = {
    "left_manipulator": (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint",
        "left_wrist_yaw_joint",
        "left_wrist_pitch_joint",
        "left_wrist_roll_joint",
    ),
    "right_manipulator": (
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "right_wrist_yaw_joint",
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint",
    ),
    "head": ("head_yaw_joint", "head_pitch_joint"),
    "waist": ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"),
    "left_hand": (
        "L_index_proximal_joint",
        "L_middle_proximal_joint",
        "L_ring_proximal_joint",
        "L_pinky_proximal_joint",
        "L_thumb_proximal_pitch_joint",
        "L_thumb_proximal_yaw_joint",
    ),
    "right_hand": (
        "R_index_proximal_joint",
        "R_middle_proximal_joint",
        "R_ring_proximal_joint",
        "R_pinky_proximal_joint",
        "R_thumb_proximal_pitch_joint",
        "R_thumb_proximal_yaw_joint",
    ),
}


def control_group_command(values: Mapping[str, float]) -> dict[str, list[float]]:
    command = {
        group: [values[name] for name in names]
        for group, names in CONTROL_GROUPS.items()
    }
    for hand in ("left_hand", "right_hand"):
        index, middle, ring, pinky, thumb_pitch, thumb_yaw = command[hand]
        command[hand] = [
            0.17 - index,
            0.17 - middle,
            0.17 - ring,
            0.17 - pinky,
            0.12 + thumb_pitch,
            -thumb_yaw,
        ]
    return command


@dataclass(frozen=True, slots=True)
class DriverDecision:
    status: str
    reasons: tuple[str, ...]
    wrote: bool = False


class Gr3AuroraDriver:
    """Offline-testable driver contract; not an installed hardware entry point."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        robot_id: int | None = None,
        limited_joints: Collection[str] = (),
        client_factory: Callable[[int], Any] | None = None,
    ) -> None:
        self.enabled = bool(enabled)
        self.robot_id = robot_id
        self.limits_complete = set(JOINT_NAMES).issubset(limited_joints)
        self._client_factory = client_factory or self._default_client_factory
        self._client: Any = None

    @staticmethod
    def _hold(reason: str) -> DriverDecision:
        return DriverDecision("HOLD", (reason,))

    def send(
        self,
        action: Mapping[str, Any],
        *,
        safety_status: str,
        operator_enabled: bool = False,
        robot_ready: bool = False,
        generation: int = -1,
        expected_generation: int = -2,
    ) -> DriverDecision:
        if not self.enabled:
            return self._hold("hardware_write_disabled")
        if self.robot_id is None:
            return self._hold("missing_robot_id")
        if not self.limits_complete:
            return self._hold("missing_joint_limits")
        if not operator_enabled:
            return self._hold("operator_not_enabled")
        if not robot_ready:
            return self._hold("robot_not_ready")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        if safety_status not in {"PASS", "ADJUSTED"}:
            return self._hold("action_not_safety_approved")
        if set(action) != set(ACTION_NAMES):
            return self._hold("invalid_action_schema")
        try:
            values = {name: float(action[name]) for name in ACTION_NAMES}
        except (TypeError, ValueError):
            return self._hold("non_numeric_action")
        if not all(math.isfinite(value) for value in values.values()):
            return self._hold("non_finite_action")
        if any(values[name] != 0.0 for name in BASE_ACTION_NAMES):
            return self._hold("base_action_not_zero")

        try:
            if self._client is None:
                self._client = self._client_factory(self.robot_id)
            self._client.set_joint_positions(control_group_command(values))
        except Exception:
            return DriverDecision("FAULT", ("aurora_write_failed",))
        return DriverDecision("SENT", (), True)

    @staticmethod
    def _default_client_factory(robot_id: int) -> Any:
        raise RuntimeError(
            "standalone GR3 driver is non-deployable; the formal teleop/Aurora owner "
            "is the only hardware writer"
        )
