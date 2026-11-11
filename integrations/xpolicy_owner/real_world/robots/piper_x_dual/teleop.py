from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np
from XPolicyLab.real_world.dagger import DaggerActionProfile, DaggerRouter, Decision
from XPolicyLab.real_world.teleop import (
    TeleopButtonCoordinator,
    TeleopButtonResult,
    TeleopControlTransition,
)

from .inventory import DualPiperSnapshot
from .mapping import ACTION_NAMES, action_dict

PIPER_SHADOW_DAGGER_PROFILE = DaggerActionProfile(
    # The Mink target is already takeover-relative in Cartesian space, so the
    # common router must neither realign nor step-limit the absolute joint and
    # gripper command. Validation and rejection live at the SDK owner boundary.
    default_max_step=0.0,
    direct_expert_fields=frozenset(ACTION_NAMES),
)


@dataclass(frozen=True, slots=True)
class PiperShadowTarget:
    action: dict[str, float]
    arm_mapping_qualified: bool = False
    waist_pitch: str = "disabled"
    waist_yaw: str = "disabled"


class PiperShadowRetargeter:
    """No-send expert candidate: measured arm hold plus trigger grippers."""

    def __init__(
        self,
        *,
        gripper_max_width_m: float = 0.1,
        left_end_effector_enabled: bool = True,
        right_end_effector_enabled: bool = True,
    ) -> None:
        if not np.isfinite(gripper_max_width_m) or gripper_max_width_m <= 0.0:
            raise ValueError("gripper maximum width must be positive")
        self.gripper_max_width_m = float(gripper_max_width_m)
        self.end_effector_enabled = {
            "left": bool(left_end_effector_enabled),
            "right": bool(right_end_effector_enabled),
        }

    @staticmethod
    def snapshot_action(snapshot: DualPiperSnapshot) -> dict[str, float]:
        return action_dict(snapshot.state_14d)

    def update(
        self,
        payload: Mapping[str, Any],
        snapshot: DualPiperSnapshot,
    ) -> PiperShadowTarget:
        triggers = np.asarray(
            [payload["left_trigger"], payload["right_trigger"]], dtype=float
        )
        if triggers.shape != (2,) or not np.isfinite(triggers).all():
            raise ValueError("QNexo triggers must contain two finite values")
        if np.any(triggers < 0.0) or np.any(triggers > 1.0):
            raise ValueError("QNexo triggers must be in [0, 1]")
        action = self.snapshot_action(snapshot)
        for side, trigger in zip(("left", "right"), triggers, strict=True):
            name = f"{side}_gripper"
            if self.end_effector_enabled[side]:
                # Shared QNexo semantics: 1=open, 0=closed. Piper uses width.
                action[name] = float(trigger * self.gripper_max_width_m)
        return PiperShadowTarget(action)


@dataclass(frozen=True, slots=True)
class PiperShadowControlStep:
    transition: TeleopControlTransition | None = None
    control: Decision | None = None
    takeover: Decision | None = None
    expert: Decision | None = None


class PiperShadowController:
    """Shared lifecycle/router integration; hardware dispatch stays in the owner."""

    def __init__(
        self,
        retargeter: PiperShadowRetargeter | None = None,
        *,
        prepare_requires_confirmation: bool = False,
        expert_only: bool = False,
    ) -> None:
        self.buttons = TeleopButtonCoordinator()
        self.expert_only = bool(expert_only)
        self.router = DaggerRouter(
            PIPER_SHADOW_DAGGER_PROFILE,
            expert_only=self.expert_only,
        )
        self.retargeter = retargeter or PiperShadowRetargeter()
        self.takeover_active = False
        self.prepare_requires_confirmation = bool(prepare_requires_confirmation)

    @property
    def lifecycle_state(self) -> str:
        return self.buttons.lifecycle.state

    @property
    def generation(self) -> int:
        return self.router.generation

    @staticmethod
    def snapshot_action(snapshot: DualPiperSnapshot) -> dict[str, float]:
        return action_dict(snapshot.state_14d)

    def update_snapshot(self, snapshot: DualPiperSnapshot) -> None:
        self.router.update_robot(self.snapshot_action(snapshot))

    def _model_control(self, command: str) -> Decision | None:
        if not self.router.active_episode:
            self.router.request_model(command)
            return self.router.set_episode(True)
        return self.router.request_model(command)

    def _apply_button_result(
        self, result: TeleopButtonResult
    ) -> tuple[TeleopControlTransition, Decision | None]:
        transition = result.transition
        if not transition.accepted:
            return transition, None
        event = transition.event
        if event == "prepare":
            if self.prepare_requires_confirmation:
                return transition, None
            replayed = self.buttons.complete_boundary(success=True)
            if replayed is not None:
                return self._apply_button_result(replayed)
            return transition, None
        if event == "start":
            return transition, self._model_control("run")
        if event == "step":
            return transition, self._model_control("step")
        if event == "stop":
            self.takeover_active = False
            control = (
                self.router.set_episode(False) if self.router.active_episode else None
            )
            return transition, control
        if event in {"finish", "discard"}:
            self.takeover_active = False
            control = (
                self.router.set_episode(False) if self.router.active_episode else None
            )
            replayed = self.buttons.complete_boundary(success=True)
            if replayed is not None:
                replayed_transition, replayed_control = self._apply_button_result(
                    replayed
                )
                return replayed_transition, replayed_control or control
            return transition, control
        return transition, None

    def complete_prepare(
        self, *, success: bool
    ) -> tuple[TeleopControlTransition | None, Decision | None]:
        """Complete the common asynchronous prepare boundary after motion settles."""
        replayed = self.buttons.complete_boundary(success=bool(success))
        if replayed is None:
            return None, None
        return self._apply_button_result(replayed)

    def pause_model_rollout(self) -> PiperShadowControlStep:
        """Hold the current pose and retain preparation for the next ACT chunk."""

        result = self.buttons.pause_running()
        transition = result.transition
        control = None
        if transition.accepted:
            self.takeover_active = False
            control = (
                self.router.set_episode(False)
                if self.router.active_episode
                else None
            )
        return PiperShadowControlStep(transition=transition, control=control)

    def handle_button(
        self, event: str | None
    ) -> tuple[TeleopControlTransition | None, Decision | None]:
        if event is None:
            return None, None
        return self._apply_button_result(
            self.buttons.handle(event, takeover_active=self.takeover_active)
        )

    def set_takeover(self, active: bool) -> Decision:
        active = bool(active)
        decision = self.router.set_takeover(active)
        self.takeover_active = active
        return decision

    def update(
        self,
        *,
        payload: Mapping[str, Any] | None,
        expert_action: Mapping[str, Any] | None = None,
        button_event: str | None,
        takeover_active: bool,
        snapshot: DualPiperSnapshot,
    ) -> PiperShadowControlStep:
        self.update_snapshot(snapshot)
        transition, control = self.handle_button(button_event)
        takeover = None
        if bool(takeover_active) != self.takeover_active:
            takeover = self.set_takeover(bool(takeover_active))
        if payload is not None and expert_action is not None:
            raise ValueError(
                "provide either raw shadow payload or external expert action"
            )
        expert = None
        if expert_action is not None and self.buttons.lifecycle.prepared:
            expert = self.router.update_expert(action_dict(expert_action))
        elif payload is not None and self.buttons.lifecycle.prepared:
            target = self.retargeter.update(payload, snapshot)
            expert = self.router.update_expert(target.action)
        return PiperShadowControlStep(
            transition=transition,
            control=control,
            takeover=takeover,
            expert=expert,
        )

    def update_model(
        self, action: Mapping[str, Any], generation: int
    ) -> Decision | None:
        return self.router.update_model(action_dict(action), int(generation))

    def update_model_status(self, status: str, generation: int) -> Decision | None:
        return self.router.update_model_status(str(status), int(generation))

    def fail_closed(self) -> None:
        self.buttons.fail_closed()
        if self.router.active_episode:
            self.router.set_episode(False)
        self.takeover_active = False
