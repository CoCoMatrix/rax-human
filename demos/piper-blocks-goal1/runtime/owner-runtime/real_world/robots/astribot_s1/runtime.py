from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from XPolicyLab.real_world.contracts import ActionChunk
from XPolicyLab.real_world.inference import ActionQueue, PendingChunk
from XPolicyLab.real_world.robots.astribot_s1.driver import (
    AstribotS1Driver,
    AstribotS1FullBodyDriver,
    AstribotS1JointGripperDriver,
    DriverDecision,
)
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
    ASTRIBOT_S1_JOINT_GRIPPER_ACTION_SCHEMA,
    CHASSIS_VELOCITY_NAMES,
    full_body_action_dict,
    joint_gripper_action_dict,
    split_arm_gripper_action,
    split_full_body_action,
)
from XPolicyLab.real_world.robots.astribot_s1.safety import (
    AstribotS1AuxiliarySafetyGuard,
    AstribotS1CartesianSafetyGuard,
    AstribotS1ChassisVelocitySafetyGuard,
    AstribotS1GripperSafetyGuard,
    AstribotS1JointSafetyGuard,
    SafetyDecision,
)
from XPolicyLab.real_world.robots.astribot_s1.sdk import RobotSnapshot


@dataclass(frozen=True, slots=True)
class RuntimeDecision:
    """One vendor-rate joint-runtime decision with independent safety evidence."""

    arm_safety: SafetyDecision
    gripper_safety: SafetyDecision
    driver: DriverDecision
    generation: int
    chunk_step_index: int | None = None
    target: dict[str, float] | None = None
    stop: DriverDecision | None = None
    command_timestamp_ns: int = 0
    action_metadata: dict[str, Any] | None = None

    @property
    def wrote(self) -> bool:
        return self.driver.wrote


class AstribotS1Runtime:
    """Arm/gripper-only 16D compatibility scheduler for QNexo teleoperation.

    Model actions advance at ``action_frequency_hz``. The current absolute
    target is held and independently safety-checked on each SDK-rate tick, then
    arms and grippers are submitted through one fixed-schema SDK write.
    """

    _CRITICAL_SAFETY_REASONS = frozenset(
        {
            "robot_not_alive",
            "control_rights_missing",
            "stale_robot_state",
            "stale_command",
            "invalid_state_timestamp",
            "invalid_command_timestamp",
            "invalid_joint_state",
            "invalid_gripper_state",
            "joint_state_outside_sdk_limits",
            "gripper_state_outside_sdk_limits",
        }
    )

    def __init__(
        self,
        arm_guard: AstribotS1JointSafetyGuard,
        gripper_guard: AstribotS1GripperSafetyGuard,
        driver: AstribotS1JointGripperDriver,
        *,
        action_frequency_hz: float = 30.0,
        driver_frequency_hz: float = 250.0,
        trigger_steps: int = 0,
        model_auto_prefetch: bool = False,
        model_prefetch_quantile: float = 0.95,
        model_prefetch_margin_steps: int = 1,
        model_prefetch_window_size: int = 32,
    ) -> None:
        frequencies = (float(action_frequency_hz), float(driver_frequency_hz))
        if not all(math.isfinite(value) and value > 0 for value in frequencies):
            raise ValueError("Astribot action and driver frequencies must be positive")
        if trigger_steps < 0:
            raise ValueError("trigger_steps must be non-negative")
        self.arm_guard = arm_guard
        self.gripper_guard = gripper_guard
        self.driver = driver
        self.action_frequency_hz, self.driver_frequency_hz = frequencies
        self.model_auto_prefetch = bool(model_auto_prefetch)
        self.model_prefetch_quantile = float(model_prefetch_quantile)
        self.model_prefetch_margin_steps = int(model_prefetch_margin_steps)
        self.model_prefetch_window_size = int(model_prefetch_window_size)
        self.action_period_ns = max(1, round(1_000_000_000 / self.action_frequency_hz))
        self.driver_period_ns = max(1, round(1_000_000_000 / self.driver_frequency_hz))
        self.queue = ActionQueue(
            ASTRIBOT_S1_JOINT_GRIPPER_ACTION_SCHEMA.dim,
            trigger_steps,
        )
        self.generation = 0
        self.enabled = False
        self.single_chunk = False
        self._single_chunk_accepted = False
        self._current_target: dict[str, float] | None = None
        self._current_step_index: int | None = None
        self._current_command_timestamp_ns = 0
        self._next_action_ns = 0
        self._next_driver_ns = 0
        self._queued_action_metadata: dict[str, Any] = {}
        self._current_action_metadata: dict[str, Any] | None = None

    @classmethod
    def from_profile(
        cls,
        path: str | Path,
        driver: AstribotS1JointGripperDriver,
    ) -> AstribotS1Runtime:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        runtime = profile["runtime"]
        return cls(
            AstribotS1JointSafetyGuard.from_profile(path),
            AstribotS1GripperSafetyGuard.from_profile(path),
            driver,
            action_frequency_hz=profile["action_frequency_hz"],
            driver_frequency_hz=profile["sdk"]["frequency_hz"],
            trigger_steps=runtime.get("trigger_steps", 0),
            model_auto_prefetch=runtime.get("auto_prefetch", False),
            model_prefetch_quantile=runtime.get("prefetch_quantile", 0.95),
            model_prefetch_margin_steps=runtime.get("prefetch_margin_steps", 1),
            model_prefetch_window_size=runtime.get("prefetch_window_size", 32),
        )

    def _reset_execution(self) -> None:
        self._single_chunk_accepted = False
        self._current_target = None
        self._current_step_index = None
        self._current_command_timestamp_ns = 0
        self._next_action_ns = 0
        self._next_driver_ns = 0
        self._queued_action_metadata = {}
        self._current_action_metadata = None

    def control(self, command: str, generation: int) -> None:
        if command not in {"run", "step", "pause"}:
            raise ValueError(f"unsupported Astribot command: {command}")
        self.queue.reset(generation)
        self.generation = self.queue.generation
        self.enabled = command in {"run", "step"}
        self.single_chunk = command == "step"
        self._reset_execution()

    def begin_inference(self) -> PendingChunk:
        if not self.enabled:
            raise RuntimeError("Astribot runtime is paused")
        if self.single_chunk and self._single_chunk_accepted:
            raise RuntimeError("Astribot step already owns one action chunk")
        return self.queue.begin_inference()

    def accept(self, chunk: ActionChunk, pending: PendingChunk) -> int | None:
        if chunk.schema != ASTRIBOT_S1_JOINT_GRIPPER_ACTION_SCHEMA:
            raise ValueError(
                "expected action schema "
                f"{ASTRIBOT_S1_JOINT_GRIPPER_ACTION_SCHEMA.schema_id}, "
                f"got {chunk.schema.schema_id}"
            )
        if not math.isclose(chunk.frequency_hz, self.action_frequency_hz):
            raise ValueError(
                f"action frequency {chunk.frequency_hz:g} Hz does not match "
                f"Astribot runtime {self.action_frequency_hz:g} Hz"
            )
        if chunk.generation != pending.generation:
            return None
        consumed = self.queue.accept(chunk.values, pending)
        if consumed is not None:
            self._queued_action_metadata = dict(chunk.metadata)
        if consumed is not None and self.single_chunk:
            self._single_chunk_accepted = True
        return consumed

    def fail_inference(self, pending: PendingChunk) -> bool:
        return self.queue.fail_inference(pending)

    @property
    def should_infer(self) -> bool:
        if not self.enabled:
            return False
        if self.single_chunk and self._single_chunk_accepted:
            return False
        return self.queue.should_infer

    def _hold(
        self,
        *reasons: str,
        arm_safety: SafetyDecision | None = None,
        gripper_safety: SafetyDecision | None = None,
        critical: bool = False,
    ) -> RuntimeDecision:
        arm = arm_safety or SafetyDecision("HOLD", None, tuple(reasons))
        gripper = gripper_safety or SafetyDecision("HOLD", None, tuple(reasons))
        chunk_step_index = self._current_step_index
        target = self._current_target
        command_timestamp_ns = self._current_command_timestamp_ns
        action_metadata = self._current_action_metadata
        stop = self.driver.stop_on_fault() if critical else None
        if critical:
            self.enabled = False
            self.single_chunk = False
            self.queue.reset(self.generation)
            self._reset_execution()
        return RuntimeDecision(
            arm,
            gripper,
            self.driver._hold("action_not_safety_approved"),
            self.generation,
            chunk_step_index,
            target,
            stop,
            command_timestamp_ns,
            action_metadata,
        )

    def _driver_tick_due(self, now_ns: int) -> bool:
        if self._next_driver_ns and now_ns < self._next_driver_ns:
            return False
        if not self._next_driver_ns:
            self._next_driver_ns = now_ns
        while self._next_driver_ns <= now_ns:
            self._next_driver_ns += self.driver_period_ns
        return True

    def _activate(self, now_ns: int) -> bool:
        if self._current_target is None:
            queued = self.queue.pop()
            if queued is None:
                return False
            self._current_step_index, values = queued
            self._current_target = joint_gripper_action_dict(values)
            self._current_action_metadata = dict(self._queued_action_metadata)
            self._current_command_timestamp_ns = now_ns
            self._next_action_ns = now_ns + self.action_period_ns
            return True

        while now_ns >= self._next_action_ns:
            scheduled_ns = self._next_action_ns
            queued = self.queue.pop()
            if queued is None:
                self._current_target = None
                self._current_step_index = None
                self._current_command_timestamp_ns = 0
                self._current_action_metadata = None
                self._next_action_ns = 0
                if self.single_chunk:
                    self.enabled = False
                    self.single_chunk = False
                return False
            self._current_step_index, values = queued
            self._current_target = joint_gripper_action_dict(values)
            self._current_action_metadata = dict(self._queued_action_metadata)
            self._current_command_timestamp_ns = scheduled_ns
            self._next_action_ns += self.action_period_ns
        return True

    def tick(
        self,
        snapshot: RobotSnapshot,
        joint_limits: Mapping[str, Any],
        *,
        operator_enabled: bool,
        now_ns: int,
    ) -> RuntimeDecision:
        now_ns = int(now_ns)
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if not self._driver_tick_due(now_ns):
            return self._hold("driver_tick_not_due")
        if not self.enabled:
            return self._hold("control_disabled")
        if not operator_enabled:
            return self._hold("operator_not_enabled")
        if not self._activate(now_ns):
            return self._hold("action_queue_empty")

        assert self._current_target is not None
        arm_action, gripper_action = split_arm_gripper_action(self._current_target)
        common = {
            "current_joints": snapshot.joints,
            "joint_limits": joint_limits,
            "enabled": True,
            "robot_alive": snapshot.robot_alive,
            "have_control_rights": snapshot.have_control_rights,
            "generation": self.generation,
            "expected_generation": self.generation,
            "state_timestamp_ns": snapshot.timestamp_ns,
            "command_timestamp_ns": self._current_command_timestamp_ns,
            "now_ns": now_ns,
        }
        arm_safety = self.arm_guard.evaluate(arm_action, **common)
        gripper_safety = self.gripper_guard.evaluate(gripper_action, **common)
        if not arm_safety.accepted or not gripper_safety.accepted:
            reasons = (*arm_safety.reasons, *gripper_safety.reasons)
            critical = bool(self._CRITICAL_SAFETY_REASONS.intersection(reasons))
            return self._hold(
                "action_not_safety_approved",
                arm_safety=arm_safety,
                gripper_safety=gripper_safety,
                critical=critical,
            )

        driver = self.driver.send(
            arm_safety.action,
            gripper_safety.action,
            arm_safety_status=arm_safety.status,
            gripper_safety_status=gripper_safety.status,
            operator_enabled=True,
            robot_ready=(
                snapshot.robot_alive
                and snapshot.driver_alive
                and snapshot.have_control_rights
            ),
            generation=self.generation,
            expected_generation=self.generation,
        )
        stop = None
        chunk_step_index = self._current_step_index
        target = self._current_target
        command_timestamp_ns = self._current_command_timestamp_ns
        action_metadata = self._current_action_metadata
        if driver.status == "FAULT" or "robot_not_ready" in driver.reasons:
            stop = self.driver.stop_on_fault()
            self.enabled = False
            self.single_chunk = False
            self.queue.reset(self.generation)
            self._reset_execution()
        return RuntimeDecision(
            arm_safety,
            gripper_safety,
            driver,
            self.generation,
            chunk_step_index,
            target,
            stop,
            command_timestamp_ns,
            action_metadata,
        )


@dataclass(frozen=True, slots=True)
class FullBodyRuntimeDecision:
    chassis_safety: SafetyDecision
    auxiliary_safety: SafetyDecision
    arm_safety: SafetyDecision
    gripper_safety: SafetyDecision
    driver: DriverDecision
    generation: int
    chunk_step_index: int | None = None
    target: dict[str, float] | None = None
    stop: DriverDecision | None = None
    command_timestamp_ns: int = 0
    action_metadata: dict[str, Any] | None = None

    @property
    def wrote(self) -> bool:
        return self.driver.wrote


class AstribotS1FullBodyRuntime(AstribotS1Runtime):
    """25D scheduler that submits each selected target without a final action gate.

    Guard objects remain constructor-compatible for offline and legacy callers,
    but the formal full-body owner does not use them to reject or rewrite the
    Router-selected target.
    """

    def __init__(
        self,
        chassis_guard: AstribotS1ChassisVelocitySafetyGuard,
        auxiliary_guard: AstribotS1AuxiliarySafetyGuard,
        arm_guard: AstribotS1JointSafetyGuard,
        gripper_guard: AstribotS1GripperSafetyGuard,
        driver: AstribotS1FullBodyDriver,
        *,
        action_frequency_hz: float = 30.0,
        driver_frequency_hz: float = 250.0,
        trigger_steps: int = 0,
        model_auto_prefetch: bool = False,
        model_prefetch_quantile: float = 0.95,
        model_prefetch_margin_steps: int = 1,
        model_prefetch_window_size: int = 32,
    ) -> None:
        self.chassis_guard = chassis_guard
        self.auxiliary_guard = auxiliary_guard
        super().__init__(
            arm_guard,
            gripper_guard,
            driver,
            action_frequency_hz=action_frequency_hz,
            driver_frequency_hz=driver_frequency_hz,
            trigger_steps=trigger_steps,
            model_auto_prefetch=model_auto_prefetch,
            model_prefetch_quantile=model_prefetch_quantile,
            model_prefetch_margin_steps=model_prefetch_margin_steps,
            model_prefetch_window_size=model_prefetch_window_size,
        )
        self.queue = ActionQueue(
            ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.dim,
            trigger_steps,
        )
        self._chassis_position: list[float] | None = None
        self._last_chassis_tick_ns = 0

    @classmethod
    def from_profile(
        cls,
        path: str | Path,
        driver: AstribotS1FullBodyDriver,
    ) -> AstribotS1FullBodyRuntime:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        runtime = profile["runtime"]
        return cls(
            AstribotS1ChassisVelocitySafetyGuard.from_profile(path),
            AstribotS1AuxiliarySafetyGuard.from_profile(path),
            AstribotS1JointSafetyGuard.from_profile(path),
            AstribotS1GripperSafetyGuard.from_profile(path),
            driver,
            action_frequency_hz=profile["action_frequency_hz"],
            driver_frequency_hz=profile["sdk"]["frequency_hz"],
            trigger_steps=runtime.get("trigger_steps", 0),
            model_auto_prefetch=runtime.get("auto_prefetch", False),
            model_prefetch_quantile=runtime.get("prefetch_quantile", 0.95),
            model_prefetch_margin_steps=runtime.get("prefetch_margin_steps", 1),
            model_prefetch_window_size=runtime.get("prefetch_window_size", 32),
        )

    def _reset_execution(self) -> None:
        super()._reset_execution()
        self._chassis_position = None
        self._last_chassis_tick_ns = 0

    def accept(self, chunk: ActionChunk, pending: PendingChunk) -> int | None:
        if chunk.schema != ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA:
            raise ValueError(
                f"expected action schema {ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.schema_id}, "
                f"got {chunk.schema.schema_id}"
            )
        if not math.isclose(chunk.frequency_hz, self.action_frequency_hz):
            raise ValueError(
                f"action frequency {chunk.frequency_hz:g} Hz does not match "
                f"Astribot runtime {self.action_frequency_hz:g} Hz"
            )
        if chunk.generation != pending.generation:
            return None
        consumed = self.queue.accept(chunk.values, pending)
        if consumed is not None:
            self._queued_action_metadata = dict(chunk.metadata)
        if consumed is not None and self.single_chunk:
            self._single_chunk_accepted = True
        return consumed

    def _activate(self, now_ns: int) -> bool:
        if self._current_target is None:
            queued = self.queue.pop()
            if queued is None:
                return False
            self._current_step_index, values = queued
            self._current_target = full_body_action_dict(values)
            self._current_action_metadata = dict(self._queued_action_metadata)
            self._current_command_timestamp_ns = now_ns
            self._next_action_ns = now_ns + self.action_period_ns
            return True
        while now_ns >= self._next_action_ns:
            scheduled_ns = self._next_action_ns
            queued = self.queue.pop()
            if queued is None:
                self._current_target = None
                self._current_step_index = None
                self._current_command_timestamp_ns = 0
                self._current_action_metadata = None
                self._next_action_ns = 0
                self._chassis_position = None
                self._last_chassis_tick_ns = 0
                if self.single_chunk:
                    self.enabled = False
                    self.single_chunk = False
                return False
            self._current_step_index, values = queued
            self._current_target = full_body_action_dict(values)
            self._current_action_metadata = dict(self._queued_action_metadata)
            self._current_command_timestamp_ns = scheduled_ns
            self._next_action_ns += self.action_period_ns
        return True

    def _full_hold(
        self,
        *reasons: str,
        chassis: SafetyDecision | None = None,
        auxiliary: SafetyDecision | None = None,
        arm: SafetyDecision | None = None,
        gripper: SafetyDecision | None = None,
        critical: bool = False,
    ) -> FullBodyRuntimeDecision:
        default = SafetyDecision("HOLD", None, tuple(reasons))
        action_metadata = self._current_action_metadata
        stop = self.driver.stop_on_fault() if critical else None
        if critical:
            self.enabled = False
            self.single_chunk = False
            self.queue.reset(self.generation)
            self._reset_execution()
        return FullBodyRuntimeDecision(
            chassis or default,
            auxiliary or default,
            arm or default,
            gripper or default,
            self.driver._hold("action_not_safety_approved"),
            self.generation,
            self._current_step_index,
            self._current_target,
            stop,
            self._current_command_timestamp_ns,
            action_metadata,
        )

    def _integrate_chassis(
        self,
        snapshot: RobotSnapshot,
        velocity: Mapping[str, Any],
        now_ns: int,
    ) -> list[float]:
        if self._chassis_position is None:
            self._chassis_position = list(snapshot.joints["astribot_chassis"])
        dt_ns = (
            self.driver_period_ns
            if not self._last_chassis_tick_ns
            else max(
                0, min(now_ns - self._last_chassis_tick_ns, 2 * self.driver_period_ns)
            )
        )
        self._last_chassis_tick_ns = now_ns
        dt = dt_ns / 1_000_000_000
        for index, name in enumerate(CHASSIS_VELOCITY_NAMES):
            self._chassis_position[index] += float(velocity[name]) * dt
        return list(self._chassis_position)

    def tick(
        self,
        snapshot: RobotSnapshot,
        joint_limits: Mapping[str, Any],
        *,
        operator_enabled: bool,
        now_ns: int,
    ) -> FullBodyRuntimeDecision:
        now_ns = int(now_ns)
        if not self._driver_tick_due(now_ns):
            return self._full_hold("driver_tick_not_due")
        if not self.enabled:
            return self._full_hold("control_disabled")
        if not operator_enabled:
            return self._full_hold("operator_not_enabled")
        if not self._activate(now_ns):
            return self._full_hold("action_queue_empty")
        assert self._current_target is not None
        chassis_action, auxiliary_action, arm_action, gripper_action = (
            split_full_body_action(self._current_target)
        )
        chassis = SafetyDecision("PASS", dict(chassis_action), ())
        auxiliary = SafetyDecision("PASS", dict(auxiliary_action), ())
        arm = SafetyDecision("PASS", dict(arm_action), ())
        gripper = SafetyDecision("PASS", dict(gripper_action), ())
        chassis_position = self._integrate_chassis(
            snapshot,
            chassis_action,
            now_ns,
        )
        driver = self.driver.send(
            chassis_position,
            chassis_action,
            auxiliary_action,
            arm_action,
            gripper_action,
            chassis_safety_status="PASS",
            auxiliary_safety_status="PASS",
            arm_safety_status="PASS",
            gripper_safety_status="PASS",
            operator_enabled=True,
            robot_ready=(
                snapshot.robot_alive
                and snapshot.driver_alive
                and snapshot.have_control_rights
            ),
            generation=self.generation,
            expected_generation=self.generation,
        )
        stop = None
        if driver.status == "FAULT" or "robot_not_ready" in driver.reasons:
            stop = self.driver.stop_on_fault()
            self.enabled = False
            self.single_chunk = False
            self.queue.reset(self.generation)
            self._reset_execution()
        return FullBodyRuntimeDecision(
            chassis,
            auxiliary,
            arm,
            gripper,
            driver,
            self.generation,
            self._current_step_index,
            self._current_target,
            stop,
            self._current_command_timestamp_ns,
            self._current_action_metadata,
        )


@dataclass(frozen=True, slots=True)
class CartesianRuntimeDecision:
    safety: SafetyDecision
    driver: DriverDecision


class AstribotS1CartesianRuntime:
    """Legacy single-action Cartesian experiment; not used by default teleop."""

    def __init__(
        self,
        guard: AstribotS1CartesianSafetyGuard,
        driver: AstribotS1Driver,
    ) -> None:
        self.guard = guard
        self.driver = driver
        self.generation = 0
        self.enabled = False
        self.single_step = False
        self._action: dict[str, Any] | None = None
        self._action_timestamp_ns = 0
        self._action_generation = -1

    def control(self, command: str, generation: int) -> None:
        if command not in {"run", "step", "pause"}:
            raise ValueError(f"unsupported Astribot command: {command}")
        self.generation = int(generation)
        self.enabled = command in {"run", "step"}
        self.single_step = command == "step"
        self._action = None
        self._action_timestamp_ns = 0
        self._action_generation = -1

    def submit(
        self,
        action: Mapping[str, Any],
        *,
        timestamp_ns: int,
        generation: int,
    ) -> None:
        self._action = dict(action)
        self._action_timestamp_ns = int(timestamp_ns)
        self._action_generation = int(generation)

    def tick(
        self,
        snapshot: RobotSnapshot,
        joint_limits: Mapping[str, Any],
        *,
        operator_enabled: bool,
        now_ns: int,
    ) -> CartesianRuntimeDecision:
        if self._action is None:
            safety = SafetyDecision("HOLD", None, ("command_unavailable",))
            return CartesianRuntimeDecision(
                safety,
                self.driver._hold("action_not_safety_approved"),
            )
        safety = self.guard.evaluate(
            self._action,
            snapshot.arm_poses,
            snapshot.joints,
            joint_limits,
            enabled=self.enabled and operator_enabled,
            robot_alive=snapshot.robot_alive,
            have_control_rights=snapshot.have_control_rights,
            generation=self._action_generation,
            expected_generation=self.generation,
            state_timestamp_ns=snapshot.timestamp_ns,
            command_timestamp_ns=self._action_timestamp_ns,
            now_ns=int(now_ns),
        )
        if not safety.accepted:
            return CartesianRuntimeDecision(
                safety,
                self.driver._hold("action_not_safety_approved"),
            )
        driver = self.driver.send(
            safety.action,
            safety_status=safety.status,
            operator_enabled=operator_enabled,
            robot_ready=(
                snapshot.robot_alive
                and snapshot.driver_alive
                and snapshot.have_control_rights
            ),
            generation=self._action_generation,
            expected_generation=self.generation,
        )
        if self.single_step and driver.wrote:
            self.enabled = False
            self.single_step = False
        return CartesianRuntimeDecision(safety, driver)
