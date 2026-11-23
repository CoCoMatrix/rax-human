from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum
import math
from types import MappingProxyType


class Mode(str, Enum):
    HOLD = "HOLD"
    MODEL = "MODEL"
    EXPERT = "EXPERT"
    SAFETY_STOP = "SAFETY_STOP"


@dataclass(frozen=True, slots=True)
class DaggerActionProfile:
    """Robot-specific action semantics consumed by the shared router."""

    default_max_step: float = 0.15
    max_step_by_field: Mapping[str, float] = field(default_factory=dict)
    direct_expert_fields: frozenset[str] = frozenset()
    frozen_fields: frozenset[str] = frozenset()
    disabled_fields: Mapping[str, float] = field(default_factory=dict)
    step_limit_reason: str = "velocity_limited"
    frozen_reason: str = "frozen_field"
    disabled_reason: str = "disabled_field"

    def __post_init__(self) -> None:
        default = float(self.default_max_step)
        if not math.isfinite(default) or default < 0:
            raise ValueError("default_max_step must be finite and non-negative")
        limits = {str(name): float(value) for name, value in self.max_step_by_field.items()}
        if any(not math.isfinite(value) or value < 0 for value in limits.values()):
            raise ValueError("per-field max steps must be finite and non-negative")
        disabled = {str(name): float(value) for name, value in self.disabled_fields.items()}
        if any(not math.isfinite(value) for value in disabled.values()):
            raise ValueError("disabled-field values must be finite")
        object.__setattr__(self, "default_max_step", default)
        object.__setattr__(self, "max_step_by_field", MappingProxyType(limits))
        object.__setattr__(
            self,
            "direct_expert_fields",
            frozenset(map(str, self.direct_expert_fields)),
        )
        object.__setattr__(self, "frozen_fields", frozenset(map(str, self.frozen_fields)))
        object.__setattr__(self, "disabled_fields", MappingProxyType(disabled))
        for attribute in ("step_limit_reason", "frozen_reason", "disabled_reason"):
            if not str(getattr(self, attribute)):
                raise ValueError(f"{attribute} must be non-empty")
            object.__setattr__(self, attribute, str(getattr(self, attribute)))

    def max_step(self, field_name: str) -> float:
        return self.max_step_by_field.get(field_name, self.default_max_step)


@dataclass
class Decision:
    mode: Mode
    generation: int
    action: dict[str, float] | None = None
    expert_proposed_action: dict[str, float] | None = None
    expert_aligned_unclipped_action: dict[str, float] | None = None
    expert_safe_action: dict[str, float] | None = None
    teleop_signal_valid: bool = False
    label_valid: bool = False
    model_control: str | None = None
    event: str | None = None
    adjustment_reasons: tuple[str, ...] = ()
    intervention_id: int = 0
    control_tick_id: int | None = None


class DaggerRouter:
    """Robot-independent model/expert arbitration and takeover alignment."""

    def __init__(
        self,
        profile: DaggerActionProfile | None = None,
        *,
        model_lowpass_alpha: float | None = None,
        expert_only: bool = False,
    ) -> None:
        if model_lowpass_alpha is not None and not 0.0 < model_lowpass_alpha <= 1.0:
            raise ValueError("model_lowpass_alpha must be in (0, 1]")
        self.profile = profile or DaggerActionProfile()
        self.model_lowpass_alpha = model_lowpass_alpha
        self.expert_only = bool(expert_only)
        self.model_filter_state: dict[str, float] | None = None
        self.mode = Mode.HOLD
        self.generation = 0
        self.intervention_id = 0
        self.active_episode = False
        self.takeover = False
        self.takeover_started = False
        self.robot_state: dict[str, float] | None = None
        self.frozen_reference: dict[str, float] | None = None
        self.expert_now: dict[str, float] | None = None
        self.robot_reference: dict[str, float] | None = None
        self.expert_reference: dict[str, float] | None = None
        self.pending_model_control: str | None = None
        self.resume_model_control: str | None = None
        self.model_busy = False
        self._next_control_tick_id = 0

    def _allocate_control_tick(self) -> int:
        control_tick_id = self._next_control_tick_id
        self._next_control_tick_id += 1
        return control_tick_id

    @staticmethod
    def _coerce_action(action: Mapping[str, float]) -> dict[str, float]:
        values = {str(name): float(value) for name, value in action.items()}
        if any(not math.isfinite(value) for value in values.values()):
            raise ValueError("DAgger actions must contain only finite values")
        return values

    def update_robot(self, state: Mapping[str, float]) -> None:
        self.robot_state = self._coerce_action(state)
        if (
            self.profile.frozen_fields
            and self.active_episode
            and self.frozen_reference is None
        ):
            self.frozen_reference = {
                name: self.robot_state[name]
                for name in self.profile.frozen_fields
                if name in self.robot_state
            }

    def set_episode(self, active: bool) -> Decision:
        if bool(active) == self.active_episode:
            return Decision(
                self.mode,
                self.generation,
                intervention_id=self.intervention_id,
            )
        self.active_episode = bool(active)
        self.takeover = self.expert_only and self.active_episode
        self.takeover_started = False
        self.robot_reference = None
        self.expert_reference = None
        self.frozen_reference = None
        self.model_filter_state = None
        self.generation += 1
        if self.active_episode:
            self._next_control_tick_id = 0
            if self.profile.frozen_fields and self.robot_state is not None:
                self.frozen_reference = {
                    name: self.robot_state[name]
                    for name in self.profile.frozen_fields
                    if name in self.robot_state
                }
            if self.expert_only:
                self._capture_references()
            self.mode = Mode.HOLD
            model_control = None if self.expert_only else self.pending_model_control
            self.pending_model_control = None
            self.model_busy = model_control == "step"
            return Decision(
                self.mode,
                self.generation,
                model_control=model_control,
                event="episode_start",
                intervention_id=self.intervention_id,
            )
        self.pending_model_control = None
        self.resume_model_control = None
        self.model_busy = False
        self.mode = Mode.HOLD
        return Decision(
            self.mode,
            self.generation,
            model_control="pause",
            event="episode_end",
            intervention_id=self.intervention_id,
        )

    def set_takeover(self, pressed: bool) -> Decision:
        pressed = bool(pressed)
        if self.expert_only:
            return Decision(
                self.mode,
                self.generation,
                intervention_id=self.intervention_id,
            )
        if not self.active_episode or pressed == self.takeover:
            return Decision(
                self.mode,
                self.generation,
                intervention_id=self.intervention_id,
            )
        self.takeover = pressed
        self.model_filter_state = None
        self.generation += 1
        if pressed:
            self.intervention_id += 1
            self.model_busy = False
            self._capture_references()
            self.takeover_started = False
            self.mode = Mode.HOLD
            return Decision(
                self.mode,
                self.generation,
                model_control="pause",
                event="takeover_request",
                intervention_id=self.intervention_id,
            )
        self.takeover_started = False
        self.robot_reference = None
        self.expert_reference = None
        self.mode = Mode.HOLD
        return Decision(
            self.mode,
            self.generation,
            model_control=(
                self.resume_model_control
                if self.resume_model_control == "bootstrap"
                else None
            ),
            event="takeover_end",
            intervention_id=self.intervention_id,
        )

    def request_model(self, command: str) -> Decision | None:
        if self.expert_only:
            return None
        model_control = {"run": "bootstrap", "step": "step"}.get(str(command))
        if model_control is None or self.takeover:
            return None
        if command == "step" and self.model_busy:
            return None
        self.resume_model_control = model_control
        if not self.active_episode:
            self.pending_model_control = model_control
            return None
        self.model_filter_state = None
        self.generation += 1
        self.model_busy = command == "step"
        self.mode = Mode.HOLD
        return Decision(
            self.mode,
            self.generation,
            model_control=model_control,
            event="model_step" if command == "step" else None,
            intervention_id=self.intervention_id,
        )

    def update_expert(self, action: Mapping[str, float]) -> Decision:
        self.expert_now = self._coerce_action(action)
        proposed = dict(self.expert_now)
        # Frozen fields constrain model output.  The operator must still be
        # able to move those fields during an intervention.
        shadow_safe, shadow_reasons = self._limit(
            self.expert_now,
            apply_frozen_fields=False,
        )
        if not self.active_episode:
            return Decision(
                self.mode,
                self.generation,
                action=shadow_safe,
                expert_proposed_action=proposed,
                teleop_signal_valid=True,
                label_valid=False,
                adjustment_reasons=shadow_reasons,
                intervention_id=self.intervention_id,
            )
        if not self.takeover:
            return Decision(
                self.mode,
                self.generation,
                expert_proposed_action=proposed,
                teleop_signal_valid=True,
                label_valid=False,
                adjustment_reasons=shadow_reasons,
                intervention_id=self.intervention_id,
            )
        self._capture_references()
        if self.robot_reference is None or self.expert_reference is None:
            self.mode = Mode.HOLD
            return Decision(
                self.mode,
                self.generation,
                expert_proposed_action=proposed,
                teleop_signal_valid=True,
                label_valid=False,
                adjustment_reasons=shadow_reasons,
                intervention_id=self.intervention_id,
            )
        aligned = {
            name: (
                value
                if name in self.profile.direct_expert_fields
                else self.robot_reference.get(name, value)
                + value
                - self.expert_reference.get(name, value)
            )
            for name, value in self.expert_now.items()
        }
        safe, reasons = self._limit(aligned, apply_frozen_fields=False)
        takeover_event = (
            None if self.takeover_started or self.expert_only else "takeover_start"
        )
        self.takeover_started = True
        self.mode = Mode.EXPERT
        return Decision(
            self.mode,
            self.generation,
            control_tick_id=self._allocate_control_tick(),
            action=safe,
            expert_proposed_action=proposed,
            expert_aligned_unclipped_action=aligned,
            expert_safe_action=safe,
            teleop_signal_valid=True,
            label_valid=True,
            event=takeover_event,
            adjustment_reasons=reasons,
            intervention_id=self.intervention_id,
        )

    def update_model(
        self,
        action: Mapping[str, float],
        generation: int,
    ) -> Decision | None:
        if (
            self.expert_only
            or not self.active_episode
            or self.takeover
            or int(generation) != self.generation
        ):
            return None
        target = self._coerce_action(action)
        reasons: list[str] = []
        if self.model_lowpass_alpha is not None:
            previous = self.model_filter_state or self.robot_state or target
            alpha = self.model_lowpass_alpha
            target = {
                name: previous.get(name, value)
                + alpha * (value - previous.get(name, value))
                for name, value in target.items()
            }
            reasons.append("controller_smoothing")
        target, limit_reasons = self._limit(target)
        reasons.extend(limit_reasons)
        if self.model_lowpass_alpha is not None:
            self.model_filter_state = dict(target)
        self.mode = Mode.MODEL
        return Decision(
            self.mode,
            self.generation,
            control_tick_id=self._allocate_control_tick(),
            action=target,
            adjustment_reasons=tuple(dict.fromkeys(reasons)),
            intervention_id=self.intervention_id,
        )

    def update_model_status(self, status: str, generation: int) -> Decision | None:
        if self.expert_only:
            return None
        if int(generation) != self.generation or not self.active_episode or self.takeover:
            return None
        if status == "paused":
            self.model_busy = False
        if status in {"loading", "starved", "error", "paused"}:
            self.mode = Mode.HOLD
            return Decision(
                self.mode,
                self.generation,
                event=f"model_{status}",
                intervention_id=self.intervention_id,
            )
        return None

    def _capture_references(self) -> None:
        if (
            self.robot_reference is None
            and self.robot_state is not None
            and self.expert_now is not None
        ):
            self.robot_reference = dict(self.robot_state)
            self.expert_reference = dict(self.expert_now)

    def _limit(
        self,
        target: Mapping[str, float],
        *,
        apply_frozen_fields: bool = True,
    ) -> tuple[dict[str, float], tuple[str, ...]]:
        limited = dict(target)
        reasons: list[str] = []
        if self.robot_state is not None:
            for name, value in target.items():
                current = self.robot_state.get(name)
                max_step = self.profile.max_step(name)
                if current is None or max_step <= 0:
                    continue
                clipped = current + max(-max_step, min(value - current, max_step))
                if clipped != value:
                    reasons.append(self.profile.step_limit_reason)
                limited[name] = clipped
        if apply_frozen_fields and self.frozen_reference is not None:
            for name, value in self.frozen_reference.items():
                if limited.get(name) != value:
                    reasons.append(self.profile.frozen_reason)
                limited[name] = value
        for name, value in self.profile.disabled_fields.items():
            if name in limited and limited[name] != value:
                reasons.append(self.profile.disabled_reason)
            if name in limited:
                limited[name] = value
        return limited, tuple(dict.fromkeys(reasons))
