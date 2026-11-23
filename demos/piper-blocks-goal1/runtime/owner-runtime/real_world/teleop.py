from __future__ import annotations

from collections.abc import Collection, Mapping, Sequence
from dataclasses import dataclass
from enum import Enum, auto
import errno
import os
import struct
import time


DEFAULT_FOOTSWITCH_DEVICE = (
    "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd"
)
DEFAULT_FOOTSWITCH_LEFT_KEY_CODE = 16
DEFAULT_FOOTSWITCH_MIDDLE_KEY_CODE = 57
DEFAULT_FOOTSWITCH_RIGHT_KEY_CODE = 45
FOOTSWITCH_EVENT = struct.Struct("@llHHi")
FOOTSWITCH_EV_KEY = 1
FOOTSWITCH_RESET_EVENTS = frozenset({"prepare", "stop", "finish", "discard"})


class FootSwitch:
    """Non-blocking Linux input switch with hold or press-toggle semantics."""

    def __init__(
        self,
        path: str = DEFAULT_FOOTSWITCH_DEVICE,
        key_code: int = DEFAULT_FOOTSWITCH_RIGHT_KEY_CODE,
        *,
        toggle_on_press: bool = False,
        reconnect_interval_s: float = 0.25,
    ) -> None:
        self.path = str(path)
        self.key_code = int(key_code)
        self.toggle_on_press = bool(toggle_on_press)
        self.reconnect_interval_s = float(reconnect_interval_s)
        if self.reconnect_interval_s < 0.0:
            raise ValueError("footswitch reconnect interval must be non-negative")
        # A takeover pedal is optional at process startup. Keep takeover false
        # and retry the stable input path so QNexo lifecycle buttons continue
        # to work and a later pedal hotplug does not require restarting Dora.
        self._fd: int | None = None
        self._next_reconnect_at = 0.0
        self._pressed_keys: set[int] = set()
        self._toggle_state = False
        self._reconnect_if_due()

    @property
    def connected(self) -> bool:
        return self._fd is not None

    def _disconnect(self) -> None:
        fd, self._fd = self._fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        self._pressed_keys.clear()
        self._toggle_state = False
        self._next_reconnect_at = time.monotonic() + self.reconnect_interval_s

    def _reconnect_if_due(self) -> bool:
        if self._fd is not None:
            return True
        if time.monotonic() < self._next_reconnect_at:
            return False
        try:
            self._fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            self._next_reconnect_at = time.monotonic() + self.reconnect_interval_s
            return False
        return True

    def poll(self) -> bool:
        if not self._reconnect_if_due():
            return False
        while True:
            try:
                assert self._fd is not None
                data = os.read(self._fd, FOOTSWITCH_EVENT.size * 32)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in {errno.ENODEV, errno.ENXIO, errno.EIO, errno.EBADF}:
                    self._disconnect()
                    return False
                raise
            if not data:
                break
            for _, _, event_type, code, value in FOOTSWITCH_EVENT.iter_unpack(data):
                if event_type != FOOTSWITCH_EV_KEY:
                    continue
                if value == 1:
                    self._pressed_keys.add(code)
                    if self.toggle_on_press and code == self.key_code:
                        self._toggle_state = not self._toggle_state
                elif value == 0:
                    self._pressed_keys.discard(code)
        return self._toggle_state if self.toggle_on_press else self.is_pressed(
            self.key_code
        )

    def is_pressed(self, key_code: int) -> bool:
        return int(key_code) in self._pressed_keys

    def reset_toggle(self) -> None:
        self._toggle_state = False

    def reset_for_control_event(self, event: str) -> bool:
        """Fail closed at every lifecycle boundary that starts/ends a run."""
        if str(event) not in FOOTSWITCH_RESET_EVENTS:
            return False
        self.reset_toggle()
        return True

    def close(self) -> None:
        self._disconnect()


class Button:
    """Robot-independent press/release/hold edge state."""

    class State(Enum):
        IDLE = auto()
        PRESSED = auto()
        RELEASED = auto()
        HOLDING = auto()

    def __init__(self, name: str, debounce_time: float = 0.2) -> None:
        self.name = str(name)
        self._press_time = 0.0
        self._release_time = 0.0
        self._last_obs = False
        self._debounce_time = float(debounce_time)
        self._state = self.State.IDLE

    @property
    def is_pressed(self) -> bool:
        return self._state == self.State.PRESSED

    @property
    def is_released(self) -> bool:
        return self._state == self.State.RELEASED

    @property
    def is_holding(self) -> bool:
        return self._state == self.State.HOLDING

    @property
    def hold_time(self) -> float:
        if self._state in {self.State.PRESSED, self.State.HOLDING}:
            return time.time() - self._press_time
        return 0.0

    @property
    def release_time(self) -> float:
        if self._state in {self.State.RELEASED, self.State.IDLE}:
            return time.time() - self._release_time
        return 0.0

    @property
    def hold_time_before_release(self) -> float:
        if self._state in {self.State.RELEASED, self.State.IDLE}:
            if self._release_time > self._press_time:
                return self._release_time - self._press_time
        return 0.0

    def update(self, state: bool) -> None:
        previous = self._last_obs
        self._last_obs = bool(state)
        if state and not previous:
            now = time.time()
            # Match the GR3 button contract: one physical press emits one
            # lifecycle edge. Ignore a release/re-press pulse inside the
            # existing debounce window instead of producing a second command.
            if (
                self._release_time > 0.0
                and now - self._release_time < self._debounce_time
            ):
                self._state = self.State.HOLDING
            else:
                self._press_time = now
                self._state = self.State.PRESSED
        elif not state and previous:
            self._release_time = time.time()
            self._state = self.State.RELEASED
        elif state:
            self._state = self.State.HOLDING
        else:
            self._state = self.State.IDLE

    def __repr__(self) -> str:
        return f"<Button {self.name}: {self._state} ({self.hold_time:.2f}s)>"


@dataclass(frozen=True, slots=True)
class TeleopControlEvent:
    command: str
    button: str
    timestamp_ns: int


class BilateralButtonBank:
    """Map bilateral controller button edges into ordered control commands."""

    def __init__(
        self,
        button_names: Sequence[str],
        command_bindings: Sequence[tuple[str, str]],
    ) -> None:
        self.buttons = {str(name): Button(str(name)) for name in button_names}
        self.command_bindings = tuple(
            (str(button), str(command)) for button, command in command_bindings
        )

    def update(
        self,
        left_pressed: Collection[str],
        right_pressed: Collection[str],
        *,
        timestamp_ns: int | None = None,
    ) -> TeleopControlEvent | None:
        left = set(left_pressed)
        right = set(right_pressed)
        for button in self.buttons.values():
            side, name = button.name.split("_", 1)
            button.update(name in (left if side == "left" else right))
        return requested_control_event(
            self.buttons,
            self.command_bindings,
            timestamp_ns=timestamp_ns,
        )

    @property
    def pressed(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, button in self.buttons.items()
            if button.is_pressed or button.is_holding
        )


def requested_control_event(
    buttons: Mapping[str, Button],
    command_bindings: Sequence[tuple[str, str]],
    *,
    timestamp_ns: int | None = None,
) -> TeleopControlEvent | None:
    for button_name, command in command_bindings:
        if buttons[button_name].is_pressed:
            return TeleopControlEvent(
                command,
                button_name,
                time.time_ns() if timestamp_ns is None else int(timestamp_ns),
            )
    return None


QNEXO_BUTTON_NAMES = (
    "left_red",
    "left_blue",
    "left_white",
    "left_black",
    "left_joystick",
    "right_red",
    "right_blue",
    "right_white",
    "right_black",
)

# Shared QNexo data-side vocabulary. Current GR3 hardware validation shows that
# the serial bridge preserves the operator's physical left/right button side.
# The collector workflow therefore binds data-side right white/blue/red to the
# physical-right prepare/start/finish controls. Data-side left buttons have no
# lifecycle command; in particular physical left red is intentionally disabled.
STANDARD_TELEOP_BUTTON_COMMANDS = (
    ("right_red", "finish"),
    ("right_white", "prepare"),
    ("right_blue", "start"),
)

DATA_COLLECTION_BUTTON_COMMANDS = (
    ("right_red", "finish"),
    ("left_red", "discard"),
    ("right_white", "prepare"),
    ("right_blue", "start"),
)

TELEOP_CONTRACT = "xpolicylab.teleop.operator_eval.v2"
TELEOP_CAPABILITY_KEYS = (
    "waist_pitch",
    "waist_yaw",
    "left_end_effector",
    "right_end_effector",
)


def validate_teleop_profile(profile: Mapping[str, object]) -> dict[str, bool]:
    """Validate the shared operator semantics before robot-specific mapping."""
    contract = profile.get("teleop_contract")
    if contract != TELEOP_CONTRACT:
        raise ValueError(
            f"teleop_contract must be {TELEOP_CONTRACT}, got {contract!r}"
        )
    capabilities = profile.get("teleop_capabilities")
    if not isinstance(capabilities, Mapping):
        raise ValueError("teleop_capabilities must be a mapping")
    result: dict[str, bool] = {}
    for name in TELEOP_CAPABILITY_KEYS:
        value = capabilities.get(name)
        if type(value) is not bool:
            raise ValueError(f"teleop capability {name} must be an explicit boolean")
        result[name] = value
    return result


def effective_teleop_capabilities(
    capabilities: Mapping[str, bool],
) -> dict[str, str]:
    """Convert validated profile flags into the status contract vocabulary."""
    return {
        name: "enabled" if capabilities[name] else "disabled"
        for name in TELEOP_CAPABILITY_KEYS
    }


class QnexoButtonBank(BilateralButtonBank):
    def __init__(self, collection_mode: str = "MIXED") -> None:
        mode = str(collection_mode).strip().upper()
        if mode not in {"DATA_COLLECTION", "MIXED"}:
            raise ValueError("collection_mode must be DATA_COLLECTION or MIXED")
        bindings = (
            DATA_COLLECTION_BUTTON_COMMANDS
            if mode == "DATA_COLLECTION"
            else STANDARD_TELEOP_BUTTON_COMMANDS
        )
        super().__init__(QNEXO_BUTTON_NAMES, bindings)


@dataclass(frozen=True, slots=True)
class QnexoControlState:
    """One robot-independent QNexo button and footswitch update."""

    takeover: bool
    control_event: TeleopControlEvent | None = None


@dataclass(slots=True)
class ReviewJoystickNavigator:
    """Turn either left-stick axis into reliable, rate-limited review edges.

    A direction change is accepted immediately.  A held direction repeats at
    a slow UI cadence, so a calibrated stick that does not return through the
    narrow hardware center cannot leave review navigation permanently latched.
    """

    enter_threshold: float = 0.55
    exit_threshold: float = 0.40
    repeat_interval_s: float = 0.75
    _direction: int = 0
    _last_emitted_at: float | None = None

    def __post_init__(self) -> None:
        if not 0.0 < self.exit_threshold < self.enter_threshold <= 1.0:
            raise ValueError(
                "review joystick thresholds must satisfy "
                "0 < exit_threshold < enter_threshold <= 1"
            )
        if self.repeat_interval_s <= 0.0:
            raise ValueError("review joystick repeat interval must be positive")

    def update(
        self,
        left_x: float,
        left_y: float,
        *,
        now: float | None = None,
    ) -> str | None:
        """Return one ``left_up``/``left_down`` navigation event, if due."""

        axis_x = float(left_x)
        axis_y = float(left_y)
        dominant = axis_y if abs(axis_y) >= abs(axis_x) else axis_x
        if abs(dominant) <= self.exit_threshold:
            self._direction = 0
            return None
        if abs(dominant) < self.enter_threshold:
            return None

        direction = -1 if dominant < 0.0 else 1
        timestamp = time.monotonic() if now is None else float(now)
        changed = direction != self._direction
        repeat_due = (
            self._last_emitted_at is not None
            and timestamp - self._last_emitted_at >= self.repeat_interval_s
        )
        self._direction = direction
        if not changed and not repeat_due:
            return None
        self._last_emitted_at = timestamp
        return "left_up" if direction < 0 else "left_down"


class QnexoControlCoordinator:
    """Shared QNexo lifecycle buttons and takeover pedal."""

    def __init__(
        self,
        footswitch: FootSwitch,
        *,
        button_bank: QnexoButtonBank | None = None,
    ) -> None:
        self.footswitch = footswitch
        self.button_bank = button_bank or QnexoButtonBank()

    def update(
        self,
        left_pressed: Collection[str],
        right_pressed: Collection[str],
        *,
        timestamp_ns: int,
    ) -> QnexoControlState:
        event = self.button_bank.update(
            left_pressed,
            right_pressed,
            timestamp_ns=int(timestamp_ns),
        )
        takeover = self.footswitch.poll()
        if event is not None and self.footswitch.reset_for_control_event(
            event.command
        ):
            takeover = False
        return QnexoControlState(
            takeover=bool(takeover),
            control_event=event,
        )

    def fail_closed(self) -> None:
        self.footswitch.reset_toggle()

    def close(self) -> None:
        self.footswitch.close()


# Backward-compatible import name.  New integrations should use the explicit
# coordinator name so buttons and both pedals are treated as one control plane.
QnexoControlInput = QnexoControlCoordinator


@dataclass(frozen=True, slots=True)
class TeleopControlTransition:
    event: str
    accepted: bool
    state: str
    generation: int
    reason: str = ""


class TeleopControlLifecycle:
    """Robot-independent operator lifecycle for direct teleoperation."""

    def __init__(
        self,
        *,
        safe_reprepare_states: Collection[str] = (),
    ) -> None:
        self.state = "idle"
        self.generation = 0
        self.step_pending = False
        self.safe_reprepare_states = frozenset(
            str(state) for state in safe_reprepare_states
        )
        unsupported = self.safe_reprepare_states - {"prepared"}
        if unsupported:
            raise ValueError(
                "safe reprepare states must be declared standard safe states, "
                f"got {sorted(unsupported)!r}"
            )

    @property
    def prepared(self) -> bool:
        return self.state in {"prepared", "running", "finishing"}

    @property
    def running(self) -> bool:
        return self.state == "running"

    @property
    def finishing(self) -> bool:
        return self.state == "finishing"

    @property
    def motion_requested(self) -> bool:
        return self.running or self.step_pending

    def handle(self, event: str) -> TeleopControlTransition:
        event = str(event)
        if event not in {
            "prepare", "start", "step", "stop", "finish", "discard"
        }:
            raise ValueError(f"unsupported teleop control event: {event}")
        self.generation += 1
        accepted = True
        reason = ""
        if event == "prepare":
            if (
                self.state != "idle"
                and self.state not in self.safe_reprepare_states
            ):
                accepted = False
                reason = "prepare_not_allowed"
            else:
                self.state = "prepared"
                self.step_pending = False
        elif event == "start":
            if self.running:
                accepted = False
                reason = "continuous_mode_active"
            elif self.state != "prepared":
                accepted = False
                reason = "prepare_required"
            else:
                self.state = "running"
                self.step_pending = False
        elif event == "step":
            if not self.prepared or self.finishing:
                accepted = False
                reason = "prepare_required"
            elif self.running:
                accepted = False
                reason = "continuous_mode_active"
            else:
                self.step_pending = True
        elif event == "stop":
            self.state = "idle"
            self.step_pending = False
        elif event in {"finish", "discard"}:
            if self.state not in {"prepared", "running"}:
                accepted = False
                reason = "prepare_required"
            else:
                self.state = "finishing"
                self.step_pending = False
        return TeleopControlTransition(
            event,
            accepted,
            self.state,
            self.generation,
            reason,
        )

    def consume_step(self) -> bool:
        if not self.step_pending:
            return False
        self.step_pending = False
        return True

    def pause_running(self) -> TeleopControlTransition:
        """Stop model execution while retaining a resumable prepared state.

        This is an internal bounded-rollout boundary, not an operator button.
        Unlike ``stop``, it deliberately preserves preparation so a supervised
        policy can continue from the measured pose without another home move.
        """

        self.generation += 1
        accepted = self.state == "running"
        if accepted:
            self.state = "prepared"
            self.step_pending = False
        return TeleopControlTransition(
            "pause",
            accepted,
            self.state,
            self.generation,
            "" if accepted else "continuous_mode_inactive",
        )

    def finish_complete(self) -> None:
        if not self.finishing:
            raise RuntimeError("teleop lifecycle is not finishing")
        self.state = "idle"
        self.step_pending = False

    def prepare_failed(self) -> None:
        """Fail a robot-specific asynchronous prepare back to retryable IDLE."""
        if self.state != "prepared":
            raise RuntimeError("teleop lifecycle is not preparing or prepared")
        self.state = "idle"
        self.step_pending = False


@dataclass(frozen=True, slots=True)
class TeleopButtonResult:
    """One normalized button decision before robot-specific side effects.

    ``cancelled_boundary`` tells an adapter to stop an in-flight robot motion
    before applying the accepted replacement event.  ``replayed`` identifies
    an event that was received during an asynchronous boundary and is being
    delivered after that boundary completed.
    """

    transition: TeleopControlTransition
    replayed: bool = False
    cancelled_boundary: str | None = None


class TeleopButtonCoordinator:
    """Shared white/blue/red state machine for every robot adapter.

    The lifecycle itself is robot independent.  A robot adapter only starts
    and completes its own ``prepare``/``finish`` motion.  While such a motion
    is active, this coordinator preserves the operator-visible GR3 ordering:
    one white event after finish and one blue event are queued, then replayed
    in that order.  Stop always cancels the boundary and clears the queue.
    """

    EVENTS = frozenset(
        {"prepare", "start", "step", "stop", "finish", "discard"}
    )

    def __init__(
        self,
        lifecycle: TeleopControlLifecycle | None = None,
    ) -> None:
        self.lifecycle = lifecycle or TeleopControlLifecycle()
        self._boundary_event: str | None = None
        self._pending_prepare = False
        self._pending_model_event: str | None = None

    @property
    def boundary_event(self) -> str | None:
        return self._boundary_event

    @property
    def boundary_active(self) -> bool:
        return self._boundary_event is not None

    @property
    def pending_prepare(self) -> bool:
        return self._pending_prepare

    @property
    def pending_model_event(self) -> str | None:
        return self._pending_model_event

    def _rejected(
        self,
        event: str,
        reason: str,
        *,
        replayed: bool = False,
    ) -> TeleopButtonResult:
        self.lifecycle.generation += 1
        return TeleopButtonResult(
            TeleopControlTransition(
                event,
                False,
                self.lifecycle.state,
                self.lifecycle.generation,
                reason,
            ),
            replayed=replayed,
        )

    def reject(self, event: str, reason: str) -> TeleopButtonResult:
        """Report a robot-specific readiness rejection without state drift."""
        event = str(event)
        if event not in self.EVENTS:
            raise ValueError(f"unsupported teleop control event: {event}")
        return self._rejected(event, str(reason))

    def _clear_pending(self) -> None:
        self._pending_prepare = False
        self._pending_model_event = None

    def pause_running(self) -> TeleopButtonResult:
        """Apply the internal resumable boundary outside the button plane."""

        if self.boundary_active:
            return self._rejected("pause", "transition_in_progress")
        self._clear_pending()
        return TeleopButtonResult(self.lifecycle.pause_running())

    def handle(
        self,
        event: str,
        *,
        takeover_active: bool = False,
        replayed: bool = False,
    ) -> TeleopButtonResult:
        event = str(event)
        if event not in self.EVENTS:
            raise ValueError(f"unsupported teleop control event: {event}")

        if event == "step" and takeover_active:
            return self._rejected(
                event,
                "takeover_active",
                replayed=replayed,
            )

        cancelled_boundary = None
        if self.boundary_active:
            if event == "stop":
                cancelled_boundary = self._boundary_event
                self._boundary_event = None
                self._clear_pending()
            elif event in {"finish", "discard"} and self._boundary_event == "prepare":
                # A red finish supersedes an unfinished white prepare.  The
                # adapter cancels the prepare motion and starts its finish
                # motion from the current measured pose.
                cancelled_boundary = self._boundary_event
                self._boundary_event = None
                self._clear_pending()
            elif event == "prepare":
                if self._boundary_event == "finish":
                    self._pending_prepare = True
                    return self._rejected(
                        event,
                        "queued_after_transition",
                        replayed=replayed,
                    )
                return self._rejected(
                    event,
                    "transition_in_progress",
                    replayed=replayed,
                )
            elif event in {"start", "step"}:
                if self._pending_model_event is None:
                    self._pending_model_event = event
                return self._rejected(
                    event,
                    "queued_after_transition",
                    replayed=replayed,
                )
            else:
                return self._rejected(
                    event,
                    "transition_in_progress",
                    replayed=replayed,
                )

        transition = self.lifecycle.handle(event)
        if transition.accepted:
            if event in {"prepare", "finish", "discard"}:
                self._boundary_event = event
            elif event == "step":
                self.lifecycle.consume_step()
        return TeleopButtonResult(
            transition,
            replayed=replayed,
            cancelled_boundary=cancelled_boundary,
        )

    def complete_boundary(
        self,
        *,
        success: bool,
    ) -> TeleopButtonResult | None:
        """Complete a robot motion and replay the next queued button event."""
        event = self._boundary_event
        if event is None:
            raise RuntimeError("no teleop boundary is active")
        self._boundary_event = None
        if event == "prepare":
            if not success:
                self.lifecycle.prepare_failed()
        elif event in {"finish", "discard"}:
            # Finish timeout/failure still ends the episode and returns the
            # lifecycle to IDLE.  The adapter reports the robot motion failure
            # separately and must not accidentally leave model control armed.
            self.lifecycle.finish_complete()
        else:  # pragma: no cover - protected by handle()
            raise RuntimeError(f"unsupported teleop boundary: {event}")

        if self._pending_prepare:
            self._pending_prepare = False
            return self.handle("prepare", replayed=True)
        if self._pending_model_event is not None:
            pending = self._pending_model_event
            self._pending_model_event = None
            return self.handle(pending, replayed=True)
        return None

    def fail_closed(self) -> str | None:
        """Clear all pending work and force an idle, non-motion state."""
        cancelled = self._boundary_event
        self._boundary_event = None
        self._clear_pending()
        self.lifecycle.state = "idle"
        self.lifecycle.step_pending = False
        return cancelled
