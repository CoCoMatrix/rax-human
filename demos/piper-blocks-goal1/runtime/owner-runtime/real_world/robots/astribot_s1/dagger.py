from __future__ import annotations

import math
import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from XPolicyLab.real_world.contracts import ActionChunk
from XPolicyLab.real_world.dagger import (
    DaggerActionProfile,
    DaggerRouter,
    Decision,
    Mode,
)
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
    AUXILIARY_ACTION_NAMES,
    chassis_velocity_action_dict,
    combine_arm_gripper_action,
    combine_full_body_action,
    current_arm_joint_action,
    current_auxiliary_action,
    current_full_body_hold_action,
    full_body_action_dict,
    full_body_action_vector,
)
from XPolicyLab.real_world.robots.astribot_s1.owner import (
    AstribotS1OwnerRuntime,
    OwnerTickDecision,
)
from XPolicyLab.real_world.robots.astribot_s1.qnexo_bridge import QnexoSample
from XPolicyLab.real_world.robots.astribot_s1.sdk import RobotSnapshot
from XPolicyLab.real_world.robots.astribot_s1.teleop import (
    QnexoGripperRetargeter,
    QnexoJointDeltaRetargeter,
    QnexoDirectTorsoRetargeter,
)
from XPolicyLab.real_world.teleop import (
    TeleopButtonCoordinator,
    TeleopButtonResult,
    TeleopControlTransition,
)

ASTRIBOT_S1_DAGGER_PROFILE = DaggerActionProfile(
    # The Astribot retargeters already emit takeover-relative absolute robot
    # targets.  The shared router owns arbitration and evidence only: a second
    # reference alignment or feedback-relative step limiter would reshape the
    # command that the operator actually produced.
    default_max_step=0.0,
    direct_expert_fields=frozenset(ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields),
)


@dataclass(frozen=True, slots=True)
class AstribotDaggerStep:
    decision: Decision
    model_step_index: int | None = None


@dataclass(frozen=True, slots=True)
class AstribotDaggerOperatorStep:
    transition: TeleopControlTransition | None = None
    control: Decision | None = None
    takeover: Decision | None = None
    expert: AstribotDaggerStep | None = None
    reset_status: str | None = None


class AstribotS1DaggerController:
    """25D full-body model/expert arbitration before the safety runtime."""

    def __init__(
        self,
        *,
        action_frequency_hz: float = 30.0,
        model_lowpass_alpha: float | None = None,
    ) -> None:
        frequency = float(action_frequency_hz)
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError("action_frequency_hz must be positive")
        self.action_frequency_hz = frequency
        self.router = DaggerRouter(
            ASTRIBOT_S1_DAGGER_PROFILE,
            model_lowpass_alpha=model_lowpass_alpha,
        )

    @property
    def generation(self) -> int:
        return self.router.generation

    @staticmethod
    def snapshot_action(snapshot: RobotSnapshot) -> dict[str, float]:
        return current_full_body_hold_action(snapshot.joints)

    def update_snapshot(self, snapshot: RobotSnapshot) -> None:
        self.router.update_robot(self.snapshot_action(snapshot))

    def update_robot(self, action: Any) -> None:
        self.router.update_robot(full_body_action_dict(action))

    def request_model(self, command: str) -> Decision | None:
        return self.router.request_model(command)

    def set_episode(self, active: bool) -> Decision:
        return self.router.set_episode(active)

    def set_takeover(self, pressed: bool) -> Decision:
        return self.router.set_takeover(pressed)

    def update_expert(self, action: Any) -> AstribotDaggerStep:
        decision = self.router.update_expert(full_body_action_dict(action))
        return AstribotDaggerStep(decision)

    def update_model_status(
        self,
        status: str,
        generation: int,
    ) -> AstribotDaggerStep | None:
        decision = self.router.update_model_status(status, generation)
        return None if decision is None else AstribotDaggerStep(decision)

    def update_model_action(
        self,
        action: Any,
        generation: int,
        *,
        model_step_index: int | None = None,
    ) -> AstribotDaggerStep | None:
        """Accept one step produced by an external ActionChunk runtime."""
        decision = self.router.update_model(
            full_body_action_dict(action),
            int(generation),
        )
        return (
            None
            if decision is None
            else AstribotDaggerStep(decision, model_step_index)
        )

class AstribotS1DaggerOwnerRuntime:
    """Bind the shared DAgger selection to one Astribot owner SDK session."""

    def __init__(
        self,
        owner: AstribotS1OwnerRuntime,
        controller: AstribotS1DaggerController | None = None,
    ) -> None:
        self.owner = owner
        self.controller = controller or AstribotS1DaggerController(
            action_frequency_hz=owner.action_runtime.action_frequency_hz,
        )
        if not math.isclose(
            self.controller.action_frequency_hz,
            owner.action_runtime.action_frequency_hz,
        ):
            raise ValueError("DAgger and owner action frequencies must match")
        self._execution_generation = -1
        self._execution_authorized = False

    def start(self, *, activate_cameras: bool = False) -> dict[str, Any]:
        return self.owner.start(activate_cameras=activate_cameras)

    def close(self) -> None:
        self.owner.close()

    def _pause_execution(self, generation: int) -> None:
        self.owner.control("pause", generation)
        self._execution_generation = int(generation)
        self._execution_authorized = False

    def _authorize_execution(self, generation: int) -> None:
        self.owner.control("run", generation)
        self._execution_generation = int(generation)
        self._execution_authorized = True

    def _apply_control(self, decision: Decision | None) -> None:
        if decision is None:
            return
        if decision.model_control == "pause" or decision.event in {
            "episode_end",
            "model_error",
            "model_paused",
            "model_starved",
        }:
            self._pause_execution(decision.generation)
        elif decision.model_control in {"bootstrap", "step"}:
            self._authorize_execution(decision.generation)

    def request_model(self, command: str) -> Decision | None:
        decision = self.controller.request_model(command)
        self._apply_control(decision)
        return decision

    def set_episode(self, active: bool) -> Decision:
        decision = self.controller.set_episode(active)
        self._apply_control(decision)
        return decision

    def set_takeover(self, pressed: bool) -> Decision:
        decision = self.controller.set_takeover(pressed)
        self._apply_control(decision)
        return decision

    def update_model_status(
        self,
        status: str,
        generation: int,
    ) -> AstribotDaggerStep | None:
        step = self.controller.update_model_status(status, generation)
        self._dispatch(step)
        return step

    def update_model_action(
        self,
        action: Any,
        generation: int,
        *,
        model_step_index: int | None = None,
    ) -> AstribotDaggerStep | None:
        step = self.controller.update_model_action(
            action,
            generation,
            model_step_index=model_step_index,
        )
        self._dispatch(step)
        return step

    def _dispatch(self, step: AstribotDaggerStep | None) -> int | None:
        if step is None:
            return None
        decision = step.decision
        self._apply_control(decision)
        if decision.action is None:
            return None
        if decision.mode in {Mode.MODEL, Mode.EXPERT} and (
            not self._execution_authorized
            or self._execution_generation != decision.generation
        ):
            self._authorize_execution(decision.generation)
        if (
            not self._execution_authorized
            or self._execution_generation != decision.generation
        ):
            return None
        pending = self.owner.begin_inference()
        chunk = ActionChunk(
            schema=ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
            values=full_body_action_vector(decision.action)[None, :],
            frequency_hz=self.owner.action_runtime.action_frequency_hz,
            generation=decision.generation,
            metadata={
                "source": decision.mode.value,
                "control_tick_id": decision.control_tick_id,
                "intervention_id": decision.intervention_id,
                "model_step_index": step.model_step_index,
                "takeover_active": decision.mode == Mode.EXPERT,
                "teleop_signal_valid": decision.teleop_signal_valid,
                "label_valid": decision.label_valid,
                "event": decision.event,
                "adjustment_reasons": decision.adjustment_reasons,
                "expert_proposed_action": (
                    None
                    if decision.expert_proposed_action is None
                    else full_body_action_vector(
                        decision.expert_proposed_action
                    ).tolist()
                ),
                "expert_aligned_unclipped_action": (
                    None
                    if decision.expert_aligned_unclipped_action is None
                    else full_body_action_vector(
                        decision.expert_aligned_unclipped_action
                    ).tolist()
                ),
                "expert_safe_action": (
                    None
                    if decision.expert_safe_action is None
                    else full_body_action_vector(decision.expert_safe_action).tolist()
                ),
            },
        )
        return self.owner.accept(chunk, pending)

    def update_expert(self, action: Any) -> AstribotDaggerStep:
        step = self.controller.update_expert(action)
        self._dispatch(step)
        return step

    def update_reset(self, action: Any) -> int | None:
        """Queue yesterday's joint-home target through the existing owner."""
        target = full_body_action_dict(action)
        generation = self.controller.generation
        if (
            not self._execution_authorized
            or self._execution_generation != generation
        ):
            self._authorize_execution(generation)
        pending = self.owner.begin_inference()
        chunk = ActionChunk(
            schema=ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
            values=full_body_action_vector(target)[None, :],
            frequency_hz=self.owner.action_runtime.action_frequency_hz,
            generation=generation,
            metadata={
                "source": "EXPERT",
                "event": "joint_home_reset",
                "takeover_active": False,
                "label_valid": False,
            },
        )
        return self.owner.accept(chunk, pending)

    def finish_reset(self) -> None:
        self._pause_execution(self.controller.generation)

    def tick(
        self,
        *,
        operator_enabled: bool,
        now_ns: int | None = None,
        snapshot: RobotSnapshot | None = None,
    ) -> OwnerTickDecision:
        snapshot = self.owner.session.snapshot() if snapshot is None else snapshot
        self.controller.update_snapshot(snapshot)
        return self.owner.tick(
            operator_enabled=operator_enabled,
            now_ns=now_ns,
            snapshot=snapshot,
        )


class AstribotS1DaggerTeleop:
    """Map shared QNexo buttons and footswitch into Astribot DAgger events."""

    def __init__(
        self,
        runtime: Any,
        *,
        joint_scales: Any,
        torso_retargeter: QnexoDirectTorsoRetargeter | None = None,
        home_arm_action: Mapping[str, Any] | None = None,
        home_auxiliary_action: Mapping[str, Any] | None = None,
        home_gripper_action: Mapping[str, Any] | None = None,
        reset_joint_tolerance_rad: float = 0.02,
        reset_auxiliary_tolerance_rad: float | None = None,
        reset_gripper_tolerance_percent: float = 2.0,
        reset_timeout_seconds: float = 10.0,
        waist_pitch_enabled: bool = True,
        waist_yaw_enabled: bool = True,
        left_end_effector_enabled: bool = True,
        right_end_effector_enabled: bool = True,
    ) -> None:
        self.runtime = runtime
        self.buttons = TeleopButtonCoordinator()
        self.lifecycle = self.buttons.lifecycle
        self.arm_retargeter = QnexoJointDeltaRetargeter(
            joint_scales=joint_scales,
        )
        self.gripper_retargeter = QnexoGripperRetargeter()
        self.torso_retargeter = torso_retargeter
        self.home_arm_action = (
            None if home_arm_action is None else dict(home_arm_action)
        )
        self.home_auxiliary_action = (
            None
            if home_auxiliary_action is None
            else dict(home_auxiliary_action)
        )
        self.home_gripper_action = (
            None
            if home_gripper_action is None
            else dict(home_gripper_action)
        )
        self.reset_joint_tolerance_rad = float(reset_joint_tolerance_rad)
        self.reset_auxiliary_tolerance_rad = float(
            reset_joint_tolerance_rad
            if reset_auxiliary_tolerance_rad is None
            else reset_auxiliary_tolerance_rad
        )
        self.reset_gripper_tolerance_percent = float(
            reset_gripper_tolerance_percent
        )
        self.reset_timeout_seconds = float(reset_timeout_seconds)
        self.waist_pitch_enabled = bool(waist_pitch_enabled)
        self.waist_yaw_enabled = bool(waist_yaw_enabled)
        self.left_end_effector_enabled = bool(left_end_effector_enabled)
        self.right_end_effector_enabled = bool(right_end_effector_enabled)
        if self.reset_joint_tolerance_rad <= 0.0:
            raise ValueError("reset_joint_tolerance_rad must be positive")
        if self.reset_auxiliary_tolerance_rad <= 0.0:
            raise ValueError("reset_auxiliary_tolerance_rad must be positive")
        if self.reset_gripper_tolerance_percent <= 0.0:
            raise ValueError("reset_gripper_tolerance_percent must be positive")
        if self.reset_timeout_seconds <= 0.0:
            raise ValueError("reset_timeout_seconds must be positive")
        initial_pose_parts = (
            self.home_arm_action,
            self.home_auxiliary_action,
            self.home_gripper_action,
        )
        if any(part is None for part in initial_pose_parts) and any(
            part is not None for part in initial_pose_parts
        ):
            raise ValueError(
                "fixed initial arm, auxiliary and gripper actions must be "
                "configured together"
            )
        self._robot_reset_active = False
        self.reset_started_at = 0.0
        self.takeover_pressed = False

    @property
    def resetting(self) -> bool:
        return self._robot_reset_active

    @property
    def reset_event(self) -> str | None:
        return self.buttons.boundary_event if self.resetting else None

    @property
    def pending_prepare(self) -> bool:
        return self.buttons.pending_prepare

    @property
    def pending_model_event(self) -> str | None:
        return self.buttons.pending_model_event

    @property
    def controller(self) -> AstribotS1DaggerController:
        if hasattr(self.runtime, "runtime"):
            return self.runtime.runtime.controller
        return self.runtime.controller

    def _model_control(self, command: str) -> Decision | None:
        if not self.controller.router.active_episode:
            self.runtime.request_model(command)
            return self.runtime.set_episode(True)
        return self.runtime.request_model(command)

    def _release_retargeters(self) -> None:
        self.arm_retargeter.release()
        if self.torso_retargeter is not None:
            self.torso_retargeter.release()

    def _begin_reset(self, event: str) -> bool:
        self._release_retargeters()
        self.takeover_pressed = False
        if self.home_arm_action is None:
            return False
        if self.buttons.boundary_event != str(event):
            raise RuntimeError(
                "robot reset must match the active shared button boundary"
            )
        self._robot_reset_active = True
        self.reset_started_at = time.monotonic()
        return True

    def _stop_reset_motion(self) -> None:
        if self._robot_reset_active:
            self.runtime.finish_reset()
        self._robot_reset_active = False
        self._release_retargeters()

    def _apply_button_result(
        self,
        result: TeleopButtonResult,
    ) -> tuple[TeleopControlTransition | None, Decision | None]:
        transition = result.transition
        if result.cancelled_boundary is not None:
            self._stop_reset_motion()
        if not transition.accepted:
            return transition, None

        event = transition.event
        if event == "prepare":
            if not self._begin_reset(event):
                replayed = self.buttons.complete_boundary(success=True)
                if replayed is not None:
                    return self._apply_button_result(replayed)
            return transition, None
        if event == "start":
            return transition, self._model_control("run")
        if event == "step":
            return transition, self._model_control("step")
        if event == "stop":
            self._stop_reset_motion()
            self.takeover_pressed = False
            if self.controller.router.active_episode:
                return transition, self.runtime.set_episode(False)
            return transition, None
        if event == "finish":
            self.takeover_pressed = False
            control = (
                self.runtime.set_episode(False)
                if self.controller.router.active_episode
                else None
            )
            if not self._begin_reset(event):
                replayed = self.buttons.complete_boundary(success=True)
                if replayed is not None:
                    replayed_transition, replayed_control = (
                        self._apply_button_result(replayed)
                    )
                    return replayed_transition, replayed_control or control
            return transition, control
        return transition, None

    def _handle_button(
        self,
        event: str | None,
        *,
        operator_enabled: bool,
    ) -> tuple[TeleopControlTransition | None, Decision | None]:
        if event is None:
            return None, None
        return self._apply_button_result(
            self.buttons.handle(
                event,
                takeover_active=bool(operator_enabled),
            )
        )

    @staticmethod
    def _full_expert_action(
        arm_gripper: Mapping[str, Any],
        auxiliary_action: Mapping[str, Any],
    ) -> dict[str, float]:
        return combine_full_body_action(
            chassis_velocity_action_dict((0.0, 0.0, 0.0)),
            auxiliary_action,
            arm_gripper,
        )

    def _capability_gated_payload(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        gated = dict(payload)
        if not self.waist_yaw_enabled:
            gated["left_joystick_x"] = 0.0
        if not self.waist_pitch_enabled:
            gated["right_joystick_y"] = 0.0
        return gated

    def _gripper_action(
        self,
        payload: Mapping[str, Any],
        snapshot: RobotSnapshot,
    ) -> dict[str, float]:
        target = self.gripper_retargeter.update(payload)
        if not self.left_end_effector_enabled:
            target["astribot_gripper_left.joint_0"] = float(
                snapshot.joints["astribot_gripper_left"][0]
            )
        if not self.right_end_effector_enabled:
            target["astribot_gripper_right.joint_0"] = float(
                snapshot.joints["astribot_gripper_right"][0]
            )
        return target

    def _update_reset(
        self,
        snapshot: RobotSnapshot,
    ) -> tuple[str | None, TeleopButtonResult | None]:
        if not self.resetting:
            return None, None
        assert self.home_arm_action is not None
        assert self.home_auxiliary_action is not None
        assert self.home_gripper_action is not None
        current_arms = current_arm_joint_action(snapshot.joints)
        arm_error = max(
            abs(float(self.home_arm_action[name]) - current_arms[name])
            for name in current_arms
        )
        current_auxiliary = current_auxiliary_action(snapshot.joints)
        torso_error = max(
            abs(float(self.home_auxiliary_action[name]) - current_auxiliary[name])
            for name in AUXILIARY_ACTION_NAMES[:4]
        )
        current_grippers = {
            "astribot_gripper_left.joint_0": float(
                snapshot.joints["astribot_gripper_left"][0]
            ),
            "astribot_gripper_right.joint_0": float(
                snapshot.joints["astribot_gripper_right"][0]
            ),
        }
        gripper_error = max(
            abs(float(self.home_gripper_action[name]) - current_grippers[name])
            for name in current_grippers
        )
        if (
            arm_error <= self.reset_joint_tolerance_rad
            and torso_error <= self.reset_auxiliary_tolerance_rad
            and gripper_error <= self.reset_gripper_tolerance_percent
        ):
            self._stop_reset_motion()
            replayed = self.buttons.complete_boundary(success=True)
            return "JOINT_HOME_RESET_COMPLETE", replayed
        if time.monotonic() - self.reset_started_at > self.reset_timeout_seconds:
            self._stop_reset_motion()
            replayed = self.buttons.complete_boundary(success=False)
            return "JOINT_HOME_RESET_TIMEOUT", replayed
        self.runtime.update_reset(
            self._full_expert_action(
                combine_arm_gripper_action(
                    self.home_arm_action,
                    self.home_gripper_action,
                ),
                self.home_auxiliary_action,
            )
        )
        return "JOINT_HOME_RESET_RUNNING", None

    def update(
        self,
        sample: QnexoSample,
        snapshot: RobotSnapshot,
    ) -> AstribotDaggerOperatorStep:
        self.controller.update_snapshot(snapshot)
        transition, control = self._handle_button(
            sample.control_event,
            operator_enabled=bool(sample.operator_enabled),
        )

        reset_status, replayed = self._update_reset(snapshot)
        if reset_status is not None:
            if replayed is not None:
                transition, control = self._apply_button_result(replayed)
            return AstribotDaggerOperatorStep(
                transition,
                control,
                reset_status=reset_status,
            )

        expert = None
        target = None
        torso_target = None
        gated_payload = self._capability_gated_payload(sample.payload)
        pressed = bool(sample.operator_enabled)
        takeover_changed = pressed != self.takeover_pressed

        # The torso adapter emits takeover-relative deltas, just like the arm
        # adapter.  Recalibrate it before the expert reference is captured so
        # robot_reference == expert_reference on the two controlled waist axes.
        # Without this edge reset, a pre-takeover joystick lead is subtracted
        # from every later command and produces asymmetric motion and rebound.
        if takeover_changed and pressed and self.torso_retargeter is not None:
            self.torso_retargeter.release()
        if self.lifecycle.prepared and not self.lifecycle.finishing:
            target = self.arm_retargeter.update(gated_payload, snapshot.joints)
            if self.torso_retargeter is not None:
                torso_target = self.torso_retargeter.update(
                    gated_payload,
                    snapshot.joints,
                    timestamp_ns=sample.timestamp_ns,
                )

        auxiliary_action = (
            torso_target.action
            if torso_target is not None
            else current_auxiliary_action(snapshot.joints)
        )

        takeover = None
        dispatched_after_takeover = False
        if takeover_changed:
            expert_action = None
            if pressed and target is not None:
                expert_action = self._full_expert_action(
                    combine_arm_gripper_action(
                        target.action,
                        self._gripper_action(gated_payload, snapshot),
                    ),
                    auxiliary_action,
                )
                # Populate expert_now before set_takeover captures references.
                expert = self.runtime.update_expert(expert_action)
            if self.controller.router.active_episode:
                takeover = self.runtime.set_takeover(pressed)
                if pressed and expert_action is not None:
                    # Emit the calibrated first expert target in the new
                    # takeover generation.  The first command is an exact
                    # measured-pose hold; joystick integration begins on the
                    # next source sample.
                    expert = self.runtime.update_expert(expert_action)
                    dispatched_after_takeover = True
            self.takeover_pressed = pressed
            if not pressed and self.torso_retargeter is not None:
                self.torso_retargeter.release()

        if (
            not dispatched_after_takeover
            and target is not None
            and not target.calibrated
            and (torso_target is None or not torso_target.calibrated)
        ):
            expert = self.runtime.update_expert(
                self._full_expert_action(
                    combine_arm_gripper_action(
                        target.action,
                        self._gripper_action(gated_payload, snapshot),
                    ),
                    auxiliary_action,
                )
            )
        return AstribotDaggerOperatorStep(
            transition,
            control,
            takeover,
            expert,
            reset_status,
        )


__all__ = [
    "ASTRIBOT_S1_DAGGER_PROFILE",
    "AstribotDaggerOperatorStep",
    "AstribotDaggerStep",
    "AstribotS1DaggerController",
    "AstribotS1DaggerOwnerRuntime",
    "AstribotS1DaggerTeleop",
]
