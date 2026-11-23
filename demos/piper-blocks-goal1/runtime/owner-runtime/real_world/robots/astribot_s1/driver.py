from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ACTION_NAMES,
    ARM_JOINT_ACTION_NAMES,
    AUXILIARY_ACTION_NAMES,
    GRIPPER_ACTION_NAMES,
)
from XPolicyLab.real_world.robots.astribot_s1.sdk import AstribotS1SdkSession


@dataclass(frozen=True, slots=True)
class DriverDecision:
    status: str
    reasons: tuple[str, ...]
    wrote: bool = False


class AstribotS1Driver:
    """The only XPolicyLab boundary allowed to write an Astribot command."""

    def __init__(
        self,
        session: AstribotS1SdkSession,
        *,
        enabled: bool = False,
        control_way: str = "filter",
        use_wbc: bool = False,
        include_grippers: bool = True,
    ) -> None:
        if control_way not in {"filter", "direct"}:
            raise ValueError("control_way must be filter or direct")
        self.session = session
        self.enabled = bool(enabled)
        self.control_way = control_way
        self.use_wbc = bool(use_wbc)
        self.include_grippers = bool(include_grippers)

    @staticmethod
    def _hold(reason: str) -> DriverDecision:
        return DriverDecision("HOLD", (reason,))

    def send(
        self,
        action: Mapping[str, Any],
        *,
        safety_status: str,
        operator_enabled: bool,
        robot_ready: bool,
        generation: int,
        expected_generation: int,
    ) -> DriverDecision:
        if not self.enabled:
            return self._hold("hardware_write_disabled")
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
        try:
            writer = (
                self.session.set_cartesian_gripper
                if self.include_grippers
                else self.session.set_cartesian_arms
            )
            writer(
                values,
                control_way=self.control_way,
                use_wbc=self.use_wbc,
            )
        except Exception:
            return DriverDecision("FAULT", ("sdk_write_failed",))
        return DriverDecision("SENT", (), True)

    def stop_on_fault(self) -> DriverDecision:
        if not self.enabled:
            return self._hold("hardware_write_disabled")
        try:
            self.session.stop_robot()
        except Exception:
            return DriverDecision("FAULT", ("sdk_stop_failed",))
        return DriverDecision("STOPPED", (), True)


class AstribotS1JointDriver(AstribotS1Driver):
    """Arm-joint-only write boundary used by QNexo joint-delta teleop."""

    def send(
        self,
        action: Mapping[str, Any],
        *,
        safety_status: str,
        operator_enabled: bool,
        robot_ready: bool,
        generation: int,
        expected_generation: int,
    ) -> DriverDecision:
        if not self.enabled:
            return self._hold("hardware_write_disabled")
        if not operator_enabled:
            return self._hold("operator_not_enabled")
        if not robot_ready:
            return self._hold("robot_not_ready")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        if safety_status != "PASS":
            return self._hold("action_not_safety_approved")
        if set(action) != set(ARM_JOINT_ACTION_NAMES):
            return self._hold("invalid_action_schema")
        try:
            values = {name: float(action[name]) for name in ARM_JOINT_ACTION_NAMES}
        except (TypeError, ValueError):
            return self._hold("non_numeric_action")
        if not all(math.isfinite(value) for value in values.values()):
            return self._hold("non_finite_action")
        try:
            self.session.set_arm_joints(
                values,
                control_way=self.control_way,
                use_wbc=False,
            )
        except Exception:
            return DriverDecision("FAULT", ("sdk_write_failed",))
        return DriverDecision("SENT", (), True)


class AstribotS1GripperDriver(AstribotS1Driver):
    """Independent two-gripper write boundary for QNexo trigger control."""

    def send(
        self,
        action: Mapping[str, Any],
        *,
        safety_status: str,
        operator_enabled: bool,
        robot_ready: bool,
        generation: int,
        expected_generation: int,
    ) -> DriverDecision:
        if not self.enabled:
            return self._hold("hardware_write_disabled")
        if not operator_enabled:
            return self._hold("operator_not_enabled")
        if not robot_ready:
            return self._hold("robot_not_ready")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        if safety_status != "PASS":
            return self._hold("action_not_safety_approved")
        if set(action) != set(GRIPPER_ACTION_NAMES):
            return self._hold("invalid_action_schema")
        try:
            values = {name: float(action[name]) for name in GRIPPER_ACTION_NAMES}
        except (TypeError, ValueError):
            return self._hold("non_numeric_action")
        if not all(math.isfinite(value) for value in values.values()):
            return self._hold("non_finite_action")
        try:
            self.session.set_grippers(values, control_way=self.control_way)
        except Exception:
            return DriverDecision("FAULT", ("sdk_write_failed",))
        return DriverDecision("SENT", (), True)


class AstribotS1JointGripperDriver(AstribotS1Driver):
    """One fixed-schema SDK write for arms and independently gated grippers."""

    def send(
        self,
        arm_action: Mapping[str, Any],
        gripper_action: Mapping[str, Any],
        *,
        arm_safety_status: str,
        gripper_safety_status: str,
        operator_enabled: bool,
        robot_ready: bool,
        generation: int,
        expected_generation: int,
    ) -> DriverDecision:
        if not self.enabled:
            return self._hold("hardware_write_disabled")
        if not operator_enabled:
            return self._hold("operator_not_enabled")
        if not robot_ready:
            return self._hold("robot_not_ready")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        if arm_safety_status != "PASS":
            return self._hold("arm_action_not_safety_approved")
        if gripper_safety_status != "PASS":
            return self._hold("gripper_action_not_safety_approved")
        if set(arm_action) != set(ARM_JOINT_ACTION_NAMES):
            return self._hold("invalid_arm_action_schema")
        if set(gripper_action) != set(GRIPPER_ACTION_NAMES):
            return self._hold("invalid_gripper_action_schema")
        try:
            arms = {name: float(arm_action[name]) for name in ARM_JOINT_ACTION_NAMES}
            grippers = {
                name: float(gripper_action[name]) for name in GRIPPER_ACTION_NAMES
            }
        except (TypeError, ValueError):
            return self._hold("non_numeric_action")
        if not all(
            math.isfinite(value) for value in (*arms.values(), *grippers.values())
        ):
            return self._hold("non_finite_action")
        try:
            self.session.set_arm_joints_and_grippers(
                arms,
                grippers,
                control_way=self.control_way,
            )
        except Exception as error:
            return DriverDecision(
                "FAULT",
                ("sdk_write_failed", type(error).__name__, str(error)[:160]),
            )
        return DriverDecision("SENT", (), True)


class AstribotS1UpperBodyDriver(AstribotS1Driver):
    """Alternating filtered-arm and direct-torso SDK dispatch.

    The SDK exposes ``control_way`` per call rather than per part.  Arms,
    grippers and head therefore keep the smooth filter path while torso-only
    writes use the direct path proven by the vendor's realtime and torso-only
    examples.  Alternation preserves one set call per host control tick.
    """

    def __init__(
        self,
        session: AstribotS1SdkSession,
        *,
        torso_control_way: str = "direct",
        **kwargs: Any,
    ) -> None:
        super().__init__(session, **kwargs)
        if torso_control_way not in {"filter", "direct"}:
            raise ValueError("torso_control_way must be filter or direct")
        self.torso_control_way = torso_control_way
        self._send_torso_next = False

    def send(
        self,
        auxiliary_action: Mapping[str, Any],
        arm_action: Mapping[str, Any],
        gripper_action: Mapping[str, Any],
        *,
        auxiliary_safety_status: str,
        arm_safety_status: str,
        gripper_safety_status: str,
        operator_enabled: bool,
        robot_ready: bool,
        generation: int,
        expected_generation: int,
    ) -> DriverDecision:
        if not self.enabled:
            return self._hold("hardware_write_disabled")
        if not operator_enabled:
            return self._hold("operator_not_enabled")
        if not robot_ready:
            return self._hold("robot_not_ready")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        statuses = {
            "auxiliary": auxiliary_safety_status,
            "arm": arm_safety_status,
            "gripper": gripper_safety_status,
        }
        rejected = [
            name
            for name, status in statuses.items()
            if status != "PASS"
        ]
        if rejected:
            return self._hold(f"{rejected[0]}_action_not_safety_approved")
        if set(auxiliary_action) != set(AUXILIARY_ACTION_NAMES):
            return self._hold("invalid_auxiliary_action_schema")
        if set(arm_action) != set(ARM_JOINT_ACTION_NAMES):
            return self._hold("invalid_arm_action_schema")
        if set(gripper_action) != set(GRIPPER_ACTION_NAMES):
            return self._hold("invalid_gripper_action_schema")
        try:
            auxiliary = {
                name: float(auxiliary_action[name]) for name in AUXILIARY_ACTION_NAMES
            }
            arms = {name: float(arm_action[name]) for name in ARM_JOINT_ACTION_NAMES}
            grippers = {
                name: float(gripper_action[name]) for name in GRIPPER_ACTION_NAMES
            }
        except (TypeError, ValueError):
            return self._hold("non_numeric_action")
        if not all(
            math.isfinite(value)
            for value in (*auxiliary.values(), *arms.values(), *grippers.values())
        ):
            return self._hold("non_finite_action")
        try:
            if self._send_torso_next:
                self.session.set_torso_position(
                    auxiliary,
                    control_way=self.torso_control_way,
                )
            else:
                self.session.set_arm_gripper_head_positions(
                    auxiliary,
                    arms,
                    grippers,
                    control_way=self.control_way,
                )
            self._send_torso_next = not self._send_torso_next
        except Exception as error:
            return DriverDecision(
                "FAULT",
                ("sdk_write_failed", type(error).__name__, str(error)[:160]),
            )
        return DriverDecision("SENT", (), True)


class AstribotS1FullBodyDriver(AstribotS1Driver):
    """Full-body dispatch with no final action approval or value rewrite."""

    def __init__(
        self,
        session: AstribotS1SdkSession,
        *,
        torso_control_way: str = "direct",
        **kwargs: Any,
    ) -> None:
        super().__init__(session, **kwargs)
        if torso_control_way not in {"filter", "direct"}:
            raise ValueError("torso_control_way must be filter or direct")
        self.torso_control_way = torso_control_way
        self._send_torso_next = False

    def send(
        self,
        chassis_position: list[float],
        chassis_velocity: Mapping[str, Any],
        auxiliary_action: Mapping[str, Any],
        arm_action: Mapping[str, Any],
        gripper_action: Mapping[str, Any],
        *,
        chassis_safety_status: str,
        auxiliary_safety_status: str,
        arm_safety_status: str,
        gripper_safety_status: str,
        operator_enabled: bool,
        robot_ready: bool,
        generation: int,
        expected_generation: int,
    ) -> DriverDecision:
        if not self.enabled:
            return self._hold("hardware_write_disabled")
        if not operator_enabled:
            return self._hold("operator_not_enabled")
        if not robot_ready:
            return self._hold("robot_not_ready")
        if int(generation) != int(expected_generation):
            return self._hold("generation_mismatch")
        try:
            if self._send_torso_next:
                self.session.set_torso_position(
                    auxiliary_action,
                    control_way=self.torso_control_way,
                )
            else:
                self.session.set_chassis_arm_gripper_head_positions(
                    chassis_position,
                    auxiliary_action,
                    arm_action,
                    gripper_action,
                    control_way=self.control_way,
                )
            self._send_torso_next = not self._send_torso_next
        except Exception as error:
            return DriverDecision(
                "FAULT",
                ("sdk_write_failed", type(error).__name__, str(error)[:160]),
            )
        return DriverDecision("SENT", (), True)
