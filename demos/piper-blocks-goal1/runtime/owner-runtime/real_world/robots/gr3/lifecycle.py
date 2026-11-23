from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import time
from typing import Any

from XPolicyLab.real_world.robots.gr3.driver import (
    CONTROL_GROUPS,
    control_group_command,
)
from XPolicyLab.real_world.robots.gr3.mapping import JOINT_NAMES


ACTIVE_LIFECYCLE = {
    "whole_body_fsm": "UserController_A",
    "upper_body_fsm": "RemoteState",
    "velocity_source": "Navigation",
}


@dataclass(frozen=True, slots=True)
class LifecycleDecision:
    status: str
    reasons: tuple[str, ...]
    lifecycle: dict[str, str]


class Gr3LifecycleController:
    """Apply the verified GR3 hardware lifecycle sequence."""

    def __init__(
        self,
        client: Any,
        *,
        transition_delay_seconds: float = 0.5,
        initialization_delay_seconds: float = 1.0,
        timeout_seconds: float = 5.0,
        poll_seconds: float = 0.05,
        idle_poses: Sequence[Mapping[str, float]] = (),
        idle_pose_settle_seconds: float = 1.0,
        idle_pose_duration_seconds: float = 1.0,
        idle_pose_pause_seconds: float = 0.5,
        command_frequency_hz: float = 60.0,
    ) -> None:
        self.client = client
        self.transition_delay_seconds = float(transition_delay_seconds)
        self.initialization_delay_seconds = float(initialization_delay_seconds)
        self.timeout_seconds = float(timeout_seconds)
        self.poll_seconds = float(poll_seconds)
        self.idle_poses = tuple(dict(pose) for pose in idle_poses)
        self.idle_pose_settle_seconds = float(idle_pose_settle_seconds)
        self.idle_pose_duration_seconds = float(idle_pose_duration_seconds)
        self.idle_pose_pause_seconds = float(idle_pose_pause_seconds)
        self.command_period_seconds = 1.0 / float(command_frequency_hz)
        if self.idle_poses and (
            self.idle_pose_settle_seconds < 0
            or self.idle_pose_duration_seconds <= 0
            or self.idle_pose_pause_seconds < 0
            or command_frequency_hz <= 0
            or any(set(pose) != set(JOINT_NAMES) for pose in self.idle_poses)
        ):
            raise ValueError("invalid GR3 idle pose configuration")

    def read(self) -> dict[str, str]:
        return {
            "whole_body_fsm": self.client.get_fsm_state(),
            "upper_body_fsm": self.client.get_upper_fsm_state(),
            "velocity_source": self.client.get_velocity_source(),
        }

    def _wait(self, expected: dict[str, str]) -> dict[str, str]:
        deadline = time.monotonic() + self.timeout_seconds
        current = self.read()
        while (
            any(current.get(name) != value for name, value in expected.items())
            and time.monotonic() < deadline
        ):
            time.sleep(self.poll_seconds)
            current = self.read()
        return current

    @staticmethod
    def _decision(
        expected: dict[str, str],
        lifecycle: dict[str, str],
    ) -> LifecycleDecision:
        reasons = tuple(
            f"{name}_mismatch"
            for name, value in expected.items()
            if lifecycle.get(name) != value
        )
        return LifecycleDecision("HOLD" if reasons else "READY", reasons, lifecycle)

    def enter_pdstand(self) -> LifecycleDecision:
        try:
            self.client.set_fsm_state(2)
            lifecycle = self._wait({"whole_body_fsm": "PdStand"})
        except Exception:
            return LifecycleDecision("HOLD", ("pdstand_command_failed",), {})
        return self._decision({"whole_body_fsm": "PdStand"}, lifecycle)

    def initialize_pdstand(self) -> LifecycleDecision:
        """Apply the GR3 Aurora startup state sequence."""
        try:
            time.sleep(self.initialization_delay_seconds)
            self.client.set_fsm_state(2)
            time.sleep(self.transition_delay_seconds)
            self.client.set_velocity_source(2)
            time.sleep(self.transition_delay_seconds)
            self.client.set_upper_fsm_state(2)
            time.sleep(self.initialization_delay_seconds)
            lifecycle = self._wait({"whole_body_fsm": "PdStand"})
        except Exception:
            return LifecycleDecision("HOLD", ("pdstand_command_failed",), {})
        return self._decision({"whole_body_fsm": "PdStand"}, lifecycle)

    def _joint_positions(self) -> dict[str, float]:
        joints: dict[str, float] = {}
        for group, names in CONTROL_GROUPS.items():
            values = [
                float(value)
                for value in self.client.get_group_state(group, "position")
            ]
            if len(values) != len(names):
                raise ValueError(f"invalid_{group}_length")
            if group.endswith("_hand"):
                values = [
                    0.17 - values[0],
                    0.17 - values[1],
                    0.17 - values[2],
                    0.17 - values[3],
                    values[4] - 0.12,
                    -values[5],
                ]
            joints.update(zip(names, values, strict=True))
        return joints

    def _move_to_idle_pose(self) -> None:
        current = self._joint_positions()
        for index, configured_pose in enumerate(self.idle_poses):
            target = {
                name: float(configured_pose[name]) for name in JOINT_NAMES
            }

            started = time.time()
            while (elapsed := time.time() - started) < self.idle_pose_duration_seconds:
                alpha = elapsed / self.idle_pose_duration_seconds
                pose = {
                    name: current[name] + (target[name] - current[name]) * alpha
                    for name in JOINT_NAMES
                }
                self.client.set_joint_positions(control_group_command(pose))
                time.sleep(self.command_period_seconds)
            current = self._joint_positions()
            if index + 1 < len(self.idle_poses):
                time.sleep(self.idle_pose_pause_seconds)

    def activate(self) -> LifecycleDecision:
        if self.read()["whole_body_fsm"] != "PdStand":
            return LifecycleDecision("HOLD", ("pdstand_required",), self.read())
        try:
            self.client.set_fsm_state(3)
            time.sleep(self.transition_delay_seconds)
            self.client.set_velocity_source(2)
            self.client.set_velocity(0.0, 0.0, 0.0)
            self.client.set_upper_fsm_state(2)
            lifecycle = self._wait(ACTIVE_LIFECYCLE)
            if (
                self.idle_poses
                and not self._decision(ACTIVE_LIFECYCLE, lifecycle).reasons
            ):
                time.sleep(self.idle_pose_settle_seconds)
                self._move_to_idle_pose()
                self.client.set_stand_pose(0.0, 0.0, 0.0)
                lifecycle = self._wait(ACTIVE_LIFECYCLE)
        except Exception:
            self.disengage()
            return LifecycleDecision("HOLD", ("activation_command_failed",), self.read())
        decision = self._decision(ACTIVE_LIFECYCLE, lifecycle)
        if decision.status != "READY":
            self.disengage()
        return decision

    def disengage(self) -> LifecycleDecision:
        try:
            self.client.set_velocity(0.0, 0.0, 0.0)
            self.client.set_fsm_state(2)
            lifecycle = self._wait({"whole_body_fsm": "PdStand"})
        except Exception:
            return LifecycleDecision("HOLD", ("disengage_command_failed",), {})
        return self._decision({"whole_body_fsm": "PdStand"}, lifecycle)
