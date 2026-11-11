from __future__ import annotations

import base64
import binascii
from collections import deque
import hashlib
import html
import json
import os
import queue
import secrets
import shutil
import subprocess
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping

from XPolicyLab.real_world.rtc import RealTimeChunkingConfig, RtcMode
from XPolicyLab.policy.Pi_05 import web_deployment_intake

from .deployment_switch import DeploymentSwitchError, DeploymentSwitcher
from .leader_follower import LeaderFollowerConfig, parse_expert_source
from .mapping import ACTION_NAMES
from .operator_video import DEFAULT_OPERATOR_VIDEO_ROOT, OperatorVideoRecorder
from .recording_folder import recording_folder, open_recording_folder


COMMANDS = frozenset(
    {"prepare", "start", "stop", "finish", "discard", "pause", "open_grippers", "open_left_gripper"}
)
CAMERA_NAMES = ("left", "head", "right")
OPERATOR_SOURCES = frozenset({"keyboard_ui", "pigey"})
MAX_SUPERVISOR_ROLLOUT_STEPS = 10_000
MAX_PIGEY_STATUS_BYTES = 1_400_000
MAX_PIGEY_OBSERVATION_BYTES = 1_000_000
DEFAULT_HUMAN_DEMO_ROOT = (
    Path(__file__).resolve().parents[3] / "piper_human_demos"
)
DEFAULT_HUMAN_DEMO_FPS = 5.0
DEFAULT_HUMAN_DEMO_MAX_SECONDS = 600.0
PIGEY_TERMINAL_PHASES = frozenset(
    {"ALL_DONE", "SAFE_ABORT", "PREFLIGHT_FAILED", "CLOSED"}
)
DESKTOP_CLEANUP_TASKS = (
    "Close the laptop.",
    "Pick up one drink container and place it on the coaster.",
    "Pick up one mouse and place it on the mouse pad.",
    "Pick up one paper ball and place it in the trash bin.",
    "Pick up one pen and place it in the pen holder.",
)


def load_piper_task_catalog(
    path: str | os.PathLike[str] | None,
    *,
    fallback: tuple[str, ...] = DESKTOP_CLEANUP_TASKS,
) -> tuple[str, ...]:
    """Load a bundle-owned exact prompt catalog without guessing synonyms."""
    if path is None or not str(path).strip():
        return tuple(fallback)
    catalog_path = Path(path).expanduser().resolve()
    value = json.loads(catalog_path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("Piper task catalog must be a JSON object")

    # Deployment bundles may keep their authoritative prompts directly in
    # contract.json. Accept that contract representation so a new checkpoint
    # does not need a second, manually synchronized task file.
    canonical_prompts = value.get("canonical_prompts")
    if canonical_prompts is not None:
        if not isinstance(canonical_prompts, list) or not canonical_prompts:
            raise ValueError(
                "Piper bundle contract canonical_prompts must be a non-empty list"
            )
        tasks = tuple(
            prompt.strip()
            for prompt in canonical_prompts
            if isinstance(prompt, str) and prompt.strip()
        )
        if len(tasks) != len(canonical_prompts) or len(tasks) != len(set(tasks)):
            raise ValueError(
                "Piper bundle contract canonical prompts must be unique strings"
            )
        return tasks

    if value.get("exact_match_required") is not True:
        raise ValueError("Piper task catalog must require exact prompt matches")
    rows = value.get("tasks")
    if not isinstance(rows, list) or not rows:
        raise ValueError("Piper task catalog must contain a non-empty tasks list")
    tasks: list[str] = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict) or row.get("task_index") != index:
            raise ValueError("Piper task catalog indices must be consecutive from zero")
        task = row.get("task")
        if not isinstance(task, str) or not task.strip():
            raise ValueError("Piper task catalog prompts must be non-empty strings")
        tasks.append(task.strip())
    if len(tasks) != len(set(tasks)):
        raise ValueError("Piper task catalog prompts must be unique")
    return tuple(tasks)


def load_pigey_sequence_presets(
    raw: str | None,
    tasks: tuple[str, ...],
) -> tuple[dict[str, Any], ...]:
    """Validate launcher-supplied preset IDs against the exact task catalog."""
    if raw is None or not raw.strip():
        return ()
    value = json.loads(raw)
    if not isinstance(value, list) or not value:
        raise ValueError("Pigey sequence presets must be a non-empty JSON list")
    allowed = set(tasks)
    result: list[dict[str, Any]] = []
    ids: set[str] = set()
    sequences: set[tuple[str, ...]] = set()
    for row in value:
        if not isinstance(row, dict) or set(row) != {"id", "label", "prompts"}:
            raise ValueError(
                "each Pigey sequence preset requires only id, label, and prompts"
            )
        preset_id = str(row["id"]).strip()
        label = str(row["label"]).strip()
        prompts = row["prompts"]
        if (
            not preset_id
            or len(preset_id) > 64
            or any(not (char.isalnum() or char in {"_", "-"}) for char in preset_id)
        ):
            raise ValueError("Pigey sequence preset id is invalid")
        if preset_id in ids:
            raise ValueError("Pigey sequence preset ids must be unique")
        if not label or len(label) > 160:
            raise ValueError("Pigey sequence preset label is invalid")
        if not isinstance(prompts, list) or not prompts:
            raise ValueError("Pigey sequence preset prompts must be non-empty")
        normalized = tuple(
            prompt.strip()
            for prompt in prompts
            if isinstance(prompt, str) and prompt.strip()
        )
        if len(normalized) != len(prompts) or len(normalized) != len(set(normalized)):
            raise ValueError("Pigey sequence preset prompts must be unique strings")
        if any(prompt not in allowed for prompt in normalized):
            raise ValueError(
                "Pigey sequence preset contains a prompt outside the task catalog"
            )
        if normalized in sequences:
            raise ValueError("Pigey sequence preset sequences must be unique")
        ids.add(preset_id)
        sequences.add(normalized)
        result.append(
            {"id": preset_id, "label": label, "prompts": list(normalized)}
        )
    return tuple(result)


def open_operator_page(url: str, opener: Any = webbrowser.open) -> bool:
    """Open the loopback dashboard without blocking the Dora event loop."""
    try:
        opened = bool(opener(str(url), new=2))
    except Exception as exc:
        print(f"PIPER_OPERATOR_UI_AUTO_OPEN_ERROR {exc!r}", flush=True)
        return False
    print(
        f"PIPER_OPERATOR_UI_AUTO_OPEN {'OPENED' if opened else 'NO_BROWSER'} {url}",
        flush=True,
    )
    return opened


def operator_ui_token(configured: str | None = None) -> str:
    """Resolve a bounded session token, allowing supervised graph replacement."""

    value = (
        os.getenv("PIPER_OPERATOR_UI_TOKEN", "")
        if configured is None
        else str(configured)
    ).strip()
    if not value:
        return secrets.token_urlsafe(24)
    if not 32 <= len(value) <= 256 or not value.isascii() or any(
        character.isspace() for character in value
    ):
        raise ValueError(
            "PIPER_OPERATOR_UI_TOKEN must be 32-256 non-whitespace ASCII characters"
        )
    return value


class PiperOperatorState:
    """Thread-safe handoff from the loopback HTTP UI to the Dora node."""

    def __init__(
        self,
        *,
        task: str,
        write_enabled: bool,
        control_source: str,
        expert_source: str = "qnexo",
        leader_follower: Mapping[str, Any] | None = None,
        expert_max_age_ms: float = 100.0,
        tasks: tuple[str, ...] = DESKTOP_CLEANUP_TASKS,
        action_horizon: int = 10,
        default_execute_chunk: int = 10,
        execution_mode: str = "auto",
        rtc_config: RealTimeChunkingConfig | None = None,
        policy_label: str = "Pi0.5 policy",
        deployment_id: str = "",
        collection_mode: str = "OFF",
        recorder_enabled: bool = False,
        record_dir: str = "",
        pigey_expected: bool = False,
        pigey_sequence_presets: tuple[Mapping[str, Any], ...] = (),
        human_demo_required: bool = False,
        human_demo_root: str | os.PathLike[str] = DEFAULT_HUMAN_DEMO_ROOT,
        human_demo_fps: float = DEFAULT_HUMAN_DEMO_FPS,
        human_demo_max_seconds: float = DEFAULT_HUMAN_DEMO_MAX_SECONDS,
        operator_video_root: str | os.PathLike[str] = DEFAULT_OPERATOR_VIDEO_ROOT,
        operator_video_fps: float = 10.0,
        operator_video_max_seconds: float = DEFAULT_HUMAN_DEMO_MAX_SECONDS,
        camera_max_age_seconds: float = 2.0,
    ) -> None:
        self.tasks = tuple(tasks)
        if not self.tasks:
            raise ValueError("Piper operator task catalog cannot be empty")
        self.task = str(task).strip()
        if self.task not in self.tasks:
            raise ValueError("Piper operator task is not supported by this checkpoint")
        self.write_enabled = bool(write_enabled)
        self.control_source = str(control_source)
        self.expert_source = parse_expert_source(expert_source)
        self.leader_follower = LeaderFollowerConfig.from_mapping(leader_follower)
        self.expert_max_age_ms = float(expert_max_age_ms)
        if self.expert_max_age_ms <= 0.0:
            raise ValueError("Piper leader/follower feedback age must be positive")
        self.action_horizon = int(action_horizon)
        self.default_execute_chunk = int(default_execute_chunk)
        if self.action_horizon < 1:
            raise ValueError("Piper policy action horizon must be positive")
        if not 1 <= self.default_execute_chunk <= self.action_horizon:
            raise ValueError("Piper default execution chunk must fit the action horizon")
        self.execution_mode = str(execution_mode).strip().lower()
        if self.execution_mode not in {"auto", "fixed"}:
            raise ValueError("Piper execution mode must be auto or fixed")
        self.fixed_execute_chunk = self.default_execute_chunk
        self.rtc_config = rtc_config or RealTimeChunkingConfig()
        if (
            self.rtc_config.mode is not RtcMode.DISABLED
            and self.rtc_config.request_remaining_steps >= self.action_horizon
        ):
            raise ValueError(
                "RTC request remaining steps must be smaller than the action horizon"
            )
        if (
            self.rtc_config.mode is RtcMode.BLEND
            and self.rtc_config.transition_steps >= self.action_horizon
        ):
            raise ValueError("RTC blend steps must leave one fully new action step")
        self.policy_label = str(policy_label).strip() or "Pi0.5 policy"
        self.deployment_id = str(deployment_id).strip()
        self.collection_mode = str(collection_mode).strip().upper()
        if self.collection_mode not in {"OFF", "DATA_COLLECTION", "MIXED"}:
            raise ValueError(
                "Piper collection mode must be OFF, DATA_COLLECTION, or MIXED"
            )
        self.recorder_enabled = bool(recorder_enabled)
        self.record_dir = str(record_dir).strip()
        self.human_demo_required = bool(human_demo_required)
        self.pigey_expected = bool(pigey_expected or human_demo_required)
        self.human_demo_root = Path(human_demo_root).expanduser().resolve()
        self.human_demo_fps = float(human_demo_fps)
        self.human_demo_max_seconds = float(human_demo_max_seconds)
        if not 0 < self.human_demo_fps <= 30:
            raise ValueError("Piper human-demo FPS must be in (0, 30]")
        if not 1 <= self.human_demo_max_seconds <= 3600:
            raise ValueError("Piper human-demo maximum duration must be in [1, 3600]")
        self.operator_video = OperatorVideoRecorder(
            root=operator_video_root,
            fps=operator_video_fps,
            max_seconds=operator_video_max_seconds,
        )
        self.camera_max_age_seconds = float(camera_max_age_seconds)
        if not 0.1 <= self.camera_max_age_seconds <= 60.0:
            raise ValueError("Piper UI camera maximum age must be in [0.1, 60] seconds")
        self.pigey_sequence_presets = tuple(
            {
                "id": str(preset["id"]),
                "label": str(preset["label"]),
                "prompts": list(preset["prompts"]),
            }
            for preset in pigey_sequence_presets
        )
        self._commands: queue.SimpleQueue[tuple[str, int, str]] = queue.SimpleQueue()
        self._task_updates: queue.SimpleQueue[tuple[str, int, str]] = queue.SimpleQueue()
        self._execution_updates: queue.SimpleQueue[
            tuple[dict[str, Any], int]
        ] = queue.SimpleQueue()
        self._rtc_updates: queue.SimpleQueue[
            tuple[dict[str, Any], int]
        ] = queue.SimpleQueue()
        self._expert_config_updates: queue.SimpleQueue[
            tuple[dict[str, Any], int]
        ] = queue.SimpleQueue()
        self._rollout_updates: queue.SimpleQueue[
            tuple[dict[str, int], int, str]
        ] = queue.SimpleQueue()
        self._transition_updates: queue.SimpleQueue[
            tuple[dict[str, str], int, str]
        ] = queue.SimpleQueue()
        self._lock = threading.Lock()
        self._robot_status: dict[str, Any] = {"status": "WAITING_FOR_ROBOT_GRAPH"}
        self._robot_status_at = 0.0
        self._policy_status: dict[str, Any] = {"status": "WAITING_FOR_POLICY"}
        self._model_info: dict[str, Any] = {}
        self._robot_state: dict[str, float] = {}
        self._robot_state_metadata = {}
        self.rollout_max_control_steps: int | None = None
        self._last_command = ""
        self._last_command_at = 0.0
        self._camera_frames: dict[str, bytes] = {}
        self._camera_metadata: dict[str, dict[str, Any]] = {}
        self._camera_received_monotonic_ns: dict[str, int] = {}
        self._human_demo: dict[str, Any] = {
            "status": "idle",
            "active": False,
            "camera": "head",
            "record_fps": self.human_demo_fps,
            "max_seconds": self.human_demo_max_seconds,
            "root": str(self.human_demo_root),
            "frame_count": 0,
        }
        self._human_demo_entries: list[dict[str, Any]] = []
        self._human_demo_last_frame_monotonic_ns = 0
        if self.human_demo_required:
            self._pigey_status: dict[str, Any] = {
                "phase": "WAITING_FOR_HUMAN_DEMO",
                "session_id": "launcher-recording",
                "message": (
                    "Record the human task sequence below, then use Finish recording "
                    "& run Pigey for the automatic handoff."
                ),
                "server_timestamp_ns": time.time_ns(),
            }
        elif pigey_expected:
            self._pigey_status = {
                "phase": "WAITING_FOR_PIGEY",
                "session_id": "launcher-expected",
                "message": "Combined launcher is waiting for Pigey preflight.",
                "server_timestamp_ns": time.time_ns(),
            }
        else:
            self._pigey_status = {}
        self._pigey_observation: bytes | None = None
        self._pigey_observation_metadata: dict[str, Any] = {}
        self._pigey_operator_control: dict[str, Any] = {
            "sequence": 0,
            "command": None,
            "requested_timestamp_ns": 0,
        }
        # The latest command remains available for older Pigey clients.  The
        # bounded history is authoritative for new clients so a fast X -> I ->
        # S sequence cannot collapse into only the final Start command before
        # the supervisor polls the UI.
        self._pigey_operator_controls: deque[dict[str, Any]] = deque(maxlen=32)
        self._pigey_sequence_selection: dict[str, Any] = {
            "sequence": 0,
            "preset_id": None,
            "selected_timestamp_ns": 0,
        }

    @staticmethod
    def _operator_source(source: Any) -> str:
        normalized = str(source).strip().lower()
        if normalized not in OPERATOR_SOURCES:
            raise ValueError(f"unsupported Piper operator source: {source!r}")
        return normalized

    def _pigey_active_locked(self) -> bool:
        if not self._pigey_status:
            return False
        phase = str(self._pigey_status.get("phase", "")).upper()
        return phase not in PIGEY_TERMINAL_PHASES or bool(
            self._pigey_status.get("session_continues", False)
        )

    def _lifecycle_controls_locked(self) -> bool:
        lifecycle = str(self._robot_status.get("lifecycle_state", "")).lower()
        return lifecycle in {"running", "finishing"} or bool(
            self._robot_status.get("recording", False)
        )

    def _human_demo_active_locked(self) -> bool:
        return bool(self._human_demo.get("active"))

    def _pigey_recording_mode_locked(self) -> bool:
        """Return whether the shared record buttons must keep Pigey semantics."""
        return bool(self.pigey_expected or self._pigey_status)

    def _gripper_open_rejection_locked(self) -> str:
        if not self.write_enabled or self.control_source not in {"keyboard", "both"}:
            return "夹爪控制需要可写 Web"
        robot = self._robot_status
        if robot.get("status") != "READY_LIVE_TAKEOVER":
            return "请先等待机器人就绪"
        if (robot.get("lifecycle_state") not in {"idle", "prepared"}
                or robot.get("boundary_event") or robot.get("takeover")
                or robot.get("recording") or self._policy_status.get("status") != "paused"):
            return "请先按 Hold，等待机器人暂停"
        if (self._pigey_active_locked() or self._human_demo_active_locked()
                or self.operator_video.snapshot().get("active") or self.collection_mode != "OFF"):
            return "请先结束监督任务或录制"
        if not self._commands.empty() or time.monotonic() - self._last_command_at < 2.0:
            return "请等待上一条指令处理完成"
        return ""

    def enqueue(self, command: str, *, source: str = "keyboard_ui") -> None:
        normalized = str(command).strip().lower()
        if normalized not in COMMANDS:
            raise ValueError(f"unsupported Piper operator command: {command!r}")
        if normalized == "discard" and self.collection_mode != "DATA_COLLECTION":
            raise ValueError("discard is only available in expert-only data mode")
        operator_source = self._operator_source(source)
        if normalized == "pause" and operator_source != "pigey":
            raise ValueError("pause is reserved for the Pigey task boundary")
        timestamp_ns = time.time_ns()
        with self._lock:
            if normalized in {"open_grippers", "open_left_gripper"}:
                reason = self._gripper_open_rejection_locked()
                if operator_source != "keyboard_ui" or reason:
                    raise ValueError(reason or "manual grippers require the operator Web")
            if (
                operator_source == "keyboard_ui"
                and normalized != "stop"
                and self._human_demo_active_locked()
            ):
                raise ValueError(
                    "human-demo recording is active; only Hold/stop is available"
                )
            if operator_source == "keyboard_ui" and self._pigey_active_locked():
                sequence = int(self._pigey_operator_control["sequence"]) + 1
                self._pigey_operator_control = {
                    "sequence": sequence,
                    "command": normalized,
                    "requested_timestamp_ns": timestamp_ns,
                }
                self._pigey_operator_controls.append(
                    dict(self._pigey_operator_control)
                )
                self._last_command = normalized
                self._last_command_at = time.monotonic()
                # Hold, Prepare, and Finish must reach the sole hardware owner
                # as an immediate stop. Pigey retains the requested semantic
                # command and performs prepare/finish after synchronizing any
                # in-flight VLM response. Start remains supervisor-only.
                if normalized in {"stop", "prepare", "finish"}:
                    self._commands.put(("stop", timestamp_ns, operator_source))
                return
            self._commands.put((normalized, timestamp_ns, operator_source))
            self._last_command = normalized
            self._last_command_at = time.monotonic()

    def drain(self) -> list[tuple[str, int, str]]:
        result: list[tuple[str, int, str]] = []
        while True:
            try:
                result.append(self._commands.get_nowait())
            except queue.Empty:
                return result

    def select_task(self, task: str, *, source: str = "keyboard_ui") -> None:
        selected = str(task).strip()
        custom_gpt_task = (os.getenv('PIPER_ALLOW_CUSTOM_TASK') == '1'
                           and bool(selected) and len(selected) <= 4000)
        if selected not in self.tasks and not custom_gpt_task:
            raise ValueError("task is not supported by this checkpoint")
        operator_source = self._operator_source(source)
        with self._lock:
            if operator_source == "keyboard_ui" and self._pigey_active_locked():
                raise ValueError(
                    "Pigey supervisor is active; task selection is locked"
                )
            if (
                operator_source == "keyboard_ui"
                and self._human_demo_active_locked()
            ):
                raise ValueError(
                    "human-demo recording is active; task selection is locked"
                )
            if self._lifecycle_controls_locked():
                raise ValueError("stop or finish the current episode before changing task")
            self.task = selected
        self._task_updates.put((selected, time.time_ns(), operator_source))

    def drain_task_updates(self) -> list[tuple[str, int, str]]:
        result: list[tuple[str, int, str]] = []
        while True:
            try:
                result.append(self._task_updates.get_nowait())
            except queue.Empty:
                return result

    def select_pigey_sequence(
        self, preset_id: str, *, source: str = "keyboard_ui"
    ) -> dict[str, Any]:
        """Select one deployment-validated sequence without commanding motion."""
        operator_source = self._operator_source(source)
        if operator_source != "keyboard_ui":
            raise ValueError("Pigey sequence selection is reserved for the operator UI")
        selected_id = str(preset_id).strip()
        with self._lock:
            phase = str(self._pigey_status.get("phase", "")).upper()
            selection_enabled = phase == "WAITING_FOR_SEQUENCE" or (
                phase == "ROUND_READY"
                and bool(self._pigey_status.get("sequence_selection_enabled", False))
            )
            if not selection_enabled:
                raise ValueError("Pigey is not waiting for a sequence selection")
            selected = next(
                (
                    preset
                    for preset in self.pigey_sequence_presets
                    if preset["id"] == selected_id
                ),
                None,
            )
            if selected is None:
                raise ValueError("unknown Pigey sequence preset")
            sequence = int(self._pigey_sequence_selection["sequence"]) + 1
            self._pigey_sequence_selection = {
                "sequence": sequence,
                "preset_id": selected_id,
                "label": selected["label"],
                "selected_timestamp_ns": time.time_ns(),
            }
            self._pigey_status["phase"] = "SEQUENCE_QUEUED"
            self._pigey_status["sequence_selection_enabled"] = False
            self._pigey_status["message"] = (
                f"{selected['label']} queued; waiting for Pigey acknowledgement"
            )
            self._pigey_status["server_timestamp_ns"] = time.time_ns()
            return dict(self._pigey_sequence_selection)

    def _controls_locked(self) -> bool:
        return (
            self._lifecycle_controls_locked()
            or self._pigey_active_locked()
            or self._human_demo_active_locked()
        )

    def select_execution(self, mode: str, steps: Any = None) -> None:
        selected_mode = str(mode).strip().lower()
        if selected_mode not in {"auto", "fixed"}:
            raise ValueError("execution mode must be auto or fixed")
        selected_steps = self.fixed_execute_chunk
        if selected_mode == "fixed":
            try:
                selected_steps = int(steps)
            except (TypeError, ValueError) as exc:
                raise ValueError("fixed execution steps must be an integer") from exc
            if not 1 <= selected_steps <= self.action_horizon:
                raise ValueError(
                    f"fixed execution steps must be in [1, {self.action_horizon}]"
                )
        effective_steps = (
            selected_steps
            if selected_mode == "fixed"
            else self.default_execute_chunk
        )
        if os.getenv("PIPER_TRAINING_RTC_ENABLED", "0") == "1" and (
            selected_mode != "fixed" or selected_steps != self.default_execute_chunk
        ):
            raise ValueError("Training-time RTC uses the deployment's fixed commit interval")
        if (
            self.rtc_config.mode is not RtcMode.DISABLED
            and self.rtc_config.request_remaining_steps
            >= effective_steps
        ):
            raise ValueError(
                "execution chunk must be larger than the RTC request-remaining "
                f"setting ({self.rtc_config.request_remaining_steps})"
            )
        with self._lock:
            if self._controls_locked():
                raise ValueError(
                    "stop or finish the current episode before changing execution"
                )
            self.execution_mode = selected_mode
            self.fixed_execute_chunk = selected_steps
        payload = {
            "mode": selected_mode,
            "steps": selected_steps if selected_mode == "fixed" else None,
        }
        self._execution_updates.put((payload, time.time_ns()))

    def drain_execution_updates(self) -> list[tuple[dict[str, Any], int]]:
        result: list[tuple[dict[str, Any], int]] = []
        while True:
            try:
                result.append(self._execution_updates.get_nowait())
            except queue.Empty:
                return result

    def select_rtc(
        self,
        mode: Any,
        request_remaining_steps: Any = 0,
        transition_steps: Any = 0,
        transition_curve: Any = "smoothstep",
    ) -> None:
        current = self.rtc_config
        try:
            request_remaining = int(request_remaining_steps)
            transition = int(transition_steps)
        except (TypeError, ValueError) as exc:
            raise ValueError("RTC request/blend steps must be integers") from exc
        config = RealTimeChunkingConfig(
            mode=str(mode),
            latency_quantile=current.latency_quantile,
            latency_margin_steps=current.latency_margin_steps,
            latency_window_size=current.latency_window_size,
            request_remaining_steps=request_remaining,
            preserve_steps=0,
            transition_steps=transition,
            transition_curve=str(transition_curve),
            new_weight_start=current.new_weight_start,
            new_weight_end=current.new_weight_end,
        )
        execution_steps = (
            self.fixed_execute_chunk
            if self.execution_mode == "fixed"
            else self.default_execute_chunk
        )
        if os.getenv("PIPER_TRAINING_RTC_ENABLED", "0") == "1" and config.mode is not RtcMode.DISABLED:
            raise ValueError("Training-time RTC is enabled in the model; leave posthoc blending Off")
        if (
            config.mode is not RtcMode.DISABLED
            and config.request_remaining_steps >= execution_steps
        ):
            raise ValueError(
                "RTC request remaining steps must be smaller than the execution "
                f"chunk ({execution_steps}); select a larger fixed execution chunk first"
            )
        if config.mode is RtcMode.BLEND and config.transition_steps >= execution_steps:
            raise ValueError(
                "RTC blend steps must be smaller than the execution chunk"
            )
        with self._lock:
            if self._controls_locked():
                raise ValueError("stop the current episode before changing RTC")
            self.rtc_config = config
        payload = {
            "mode": config.mode.value,
            "request_remaining_steps": config.request_remaining_steps,
            # The Piper UI performs immediate aligned blending after return.
            # Retain the shared compatibility field but never add an extra
            # post-inference preserve delay.
            "preserve_steps": 0,
            "transition_steps": config.transition_steps,
            "transition_curve": config.transition_curve.value,
        }
        self._rtc_updates.put((payload, time.time_ns()))

    def drain_rtc_updates(self) -> list[tuple[dict[str, Any], int]]:
        result: list[tuple[dict[str, Any], int]] = []
        while True:
            try:
                result.append(self._rtc_updates.get_nowait())
            except queue.Empty:
                return result

    def select_expert_config(
        self,
        source: Any,
        leader_role: Any = "left_arm",
        follower_role: Any = "right_arm",
        feedback_role: Any = "right_arm",
        max_age_ms: Any = 100.0,
    ) -> None:
        """Queue a source/mapping change for the sole owner before motion.

        The UI is intentionally unable to change expert ownership while an
        episode, Pigey handoff, or human-demo capture is active.  This keeps
        the source switch atomic at the owner boundary and avoids mixing
        QNexo and SDK leader samples in one recorded trajectory.
        """
        selected_source = parse_expert_source(source)
        config = LeaderFollowerConfig(
            leader_role=str(leader_role),
            follower_role=str(follower_role),
            feedback_role=str(feedback_role),
            native_linkage=False,
        )
        try:
            selected_age = float(max_age_ms)
        except (TypeError, ValueError) as exc:
            raise ValueError("leader/follower feedback age must be numeric") from exc
        if not 1.0 <= selected_age <= 1000.0:
            raise ValueError("leader/follower feedback age must be in [1, 1000] ms")
        with self._lock:
            if self._controls_locked():
                raise ValueError(
                    "stop or finish the current episode before changing expert control"
                )
            self.expert_source = selected_source
            self.leader_follower = config
            self.expert_max_age_ms = selected_age
        payload = {
            "source": selected_source,
            "leader_role": config.leader_role,
            "follower_role": config.follower_role,
            "feedback_role": config.feedback_role,
            "native_linkage": False,
            "max_age_ms": selected_age,
        }
        self._expert_config_updates.put((payload, time.time_ns()))

    def drain_expert_config_updates(self) -> list[tuple[dict[str, Any], int]]:
        result: list[tuple[dict[str, Any], int]] = []
        while True:
            try:
                result.append(self._expert_config_updates.get_nowait())
            except queue.Empty:
                return result

    def select_rollout(
        self,
        max_control_steps: Any,
        *,
        source: str = "pigey",
    ) -> None:
        operator_source = self._operator_source(source)
        try:
            steps = int(max_control_steps)
        except (TypeError, ValueError) as exc:
            raise ValueError("rollout max_control_steps must be an integer") from exc
        if not 1 <= steps <= MAX_SUPERVISOR_ROLLOUT_STEPS:
            raise ValueError(
                "rollout max_control_steps must be in "
                f"[1, {MAX_SUPERVISOR_ROLLOUT_STEPS}]"
            )
        with self._lock:
            if self._lifecycle_controls_locked() or (
                operator_source != "pigey" and self._pigey_active_locked()
            ) or self._human_demo_active_locked():
                raise ValueError("stop the current episode before configuring a rollout")
            self.rollout_max_control_steps = steps
        self._rollout_updates.put(
            ({"max_control_steps": steps}, time.time_ns(), operator_source)
        )

    def drain_rollout_updates(self) -> list[tuple[dict[str, int], int, str]]:
        result: list[tuple[dict[str, int], int, str]] = []
        while True:
            try:
                result.append(self._rollout_updates.get_nowait())
            except queue.Empty:
                return result

    def select_transition_pose(
        self,
        pose_id: Any,
        task_prompt: Any,
        library_sha256: Any,
        reason: Any,
        request_id: Any,
        *,
        source: str = "pigey",
    ) -> None:
        operator_source = self._operator_source(source)
        if operator_source != "pigey":
            raise ValueError("transition poses are reserved for Pigey")
        pose = str(pose_id).strip()
        task = str(task_prompt).strip()
        digest = str(library_sha256).strip().lower()
        normalized_reason = str(reason).strip().lower()
        transition_request_id = str(request_id).strip()
        if not pose or len(pose) > 128 or not pose.isascii():
            raise ValueError("transition pose id is invalid")
        if task not in self.tasks:
            raise ValueError("transition pose task is not in the checkpoint catalog")
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise ValueError("transition pose library SHA-256 is invalid")
        if normalized_reason not in {"task_transition", "next_object", "recovery"}:
            raise ValueError("transition pose reason is invalid")
        if (
            not transition_request_id
            or len(transition_request_id) > 64
            or not transition_request_id.isascii()
        ):
            raise ValueError("transition pose request id is invalid")
        with self._lock:
            if not self._pigey_active_locked():
                raise ValueError("Pigey supervisor is not active")
            lifecycle = str(
                self._robot_status.get("lifecycle_state", "")
            ).lower()
            boundary = self._robot_status.get("boundary_event")
            if lifecycle != "prepared" or boundary not in {None, ""}:
                raise ValueError(
                    "transition pose requires a completed prepared boundary"
                )
        payload = {
            "pose_id": pose,
            "task_prompt": task,
            "library_sha256": digest,
            "reason": normalized_reason,
            "request_id": transition_request_id,
        }
        self._transition_updates.put((payload, time.time_ns(), operator_source))

    def drain_transition_updates(
        self,
    ) -> list[tuple[dict[str, str], int, str]]:
        result: list[tuple[dict[str, str], int, str]] = []
        while True:
            try:
                result.append(self._transition_updates.get_nowait())
            except queue.Empty:
                return result

    def update_robot_status(self, value: Any) -> None:
        if isinstance(value, str):
            try:
                parsed = json.loads(value)
            except json.JSONDecodeError:
                parsed = {"status": value}
        elif isinstance(value, dict):
            parsed = dict(value)
        else:
            parsed = {"status": str(value)}
        with self._lock:
            self._robot_status = parsed
            self._robot_status_at = time.monotonic()

    def update_policy_status(self, value: Any) -> None:
        parsed = self._status_dict(value)
        with self._lock:
            self._policy_status = parsed
            rtc = parsed.get("rtc")
            if isinstance(rtc, dict):
                try:
                    self.rtc_config = RealTimeChunkingConfig(
                        mode=rtc["mode"],
                        latency_quantile=rtc["latency_quantile"],
                        latency_margin_steps=rtc["latency_margin_steps"],
                        latency_window_size=rtc["latency_window_size"],
                        request_remaining_steps=rtc.get(
                            "request_remaining_steps", 0
                        ),
                        preserve_steps=rtc["preserve_steps"],
                        transition_steps=rtc["transition_steps"],
                        transition_curve=rtc["transition_curve"],
                        new_weight_start=rtc["new_weight_start"],
                        new_weight_end=rtc["new_weight_end"],
                    )
                except (KeyError, TypeError, ValueError):
                    pass

    def update_model_info(self, value: Any) -> None:
        parsed = self._status_dict(value)
        with self._lock:
            self._model_info = parsed

    def update_robot_state(self, value: Mapping[str, Any], metadata: Mapping[str, Any] | None = None) -> None:
        if set(value) != set(ACTION_NAMES):
            raise ValueError("Piper UI robot state does not match the 14D schema")
        state = {name: float(value[name]) for name in ACTION_NAMES}
        with self._lock:
            self._robot_state = state
            self._robot_state_metadata = {k:v for k,v in (metadata or {}).items() if k in ("source_timestamp_ns", "timestamp_ns", "maximum_age_ms", "inter_arm_skew_ms")}

    @staticmethod
    def _status_dict(value: Any) -> dict[str, Any]:
        if isinstance(value, str):
            try:
                parsed = json.loads(value)
            except json.JSONDecodeError:
                return {"status": value}
            return parsed if isinstance(parsed, dict) else {"status": str(parsed)}
        if isinstance(value, dict):
            return dict(value)
        return {"status": str(value)}

    def update_camera(
        self, name: str, jpeg: bytes, metadata: dict[str, Any] | None = None
    ) -> None:
        camera_name = str(name).strip().lower()
        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"unsupported Piper UI camera: {name!r}")
        if not isinstance(jpeg, bytes) or not jpeg:
            raise ValueError("Piper UI camera frame must be non-empty JPEG bytes")
        raw_detail = dict(metadata or {})
        detail: dict[str, Any] = {}
        for key in (
            "width",
            "height",
            "timestamp_ns",
            "source_timestamp_ns",
            "device_time_ns",
            "device_frame_index",
        ):
            if key in raw_detail:
                detail[key] = int(raw_detail[key])
        for key in ("encoding", "timestamp_domain", "camera_role", "camera_serial"):
            if key in raw_detail:
                detail[key] = str(raw_detail[key])
        detail["jpeg_bytes"] = len(jpeg)
        with self._lock:
            self._camera_frames[camera_name] = jpeg
            self._camera_metadata[camera_name] = detail
            self._camera_received_monotonic_ns[camera_name] = time.monotonic_ns()
            if camera_name == "head" and self.operator_video.snapshot().get("active"):
                self.operator_video.update(
                    camera_name,
                    frames=dict(self._camera_frames),
                    metadata=dict(self._camera_metadata),
                )
            if camera_name == "head" and self._human_demo.get("active"):
                self._record_human_demo_frame_locked(jpeg, detail)

    def _camera_status_locked(
        self, name: str, *, now_ns: int | None = None
    ) -> dict[str, Any]:
        if name not in self._camera_frames:
            return {"available": False, "status": "waiting"}
        received_ns = self._camera_received_monotonic_ns.get(name)
        if received_ns is None:
            return {"available": False, "status": "stale"}
        if now_ns is None:
            now_ns = time.monotonic_ns()
        age_seconds = max(0.0, (now_ns - received_ns) / 1e9)
        available = age_seconds <= self.camera_max_age_seconds
        return {
            "available": available,
            "status": "live" if available else "stale",
            "age_seconds": round(age_seconds, 3),
            **dict(self._camera_metadata.get(name, {})),
        }

    def _record_human_demo_frame_locked(
        self, jpeg: bytes, metadata: Mapping[str, Any]
    ) -> None:
        now_monotonic_ns = time.monotonic_ns()
        period_ns = int(1_000_000_000 / self.human_demo_fps)
        if (
            self._human_demo_last_frame_monotonic_ns
            and now_monotonic_ns - self._human_demo_last_frame_monotonic_ns
            < period_ns
        ):
            return
        maximum_frames = max(
            2, int(self.human_demo_fps * self.human_demo_max_seconds)
        )
        if len(self._human_demo_entries) >= maximum_frames:
            self._human_demo["active"] = False
            self._human_demo["status"] = "limit_reached"
            self._human_demo["message"] = (
                "Maximum recording duration reached; press Stop & Save."
            )
            return
        frame_index = len(self._human_demo_entries) + 1
        relative_path = Path("frames") / f"frame_{frame_index:06d}.jpg"
        frame_path = Path(self._human_demo["directory"]) / relative_path
        try:
            frame_path.write_bytes(jpeg)
        except OSError as exc:
            self._human_demo["active"] = False
            self._human_demo["status"] = "error"
            self._human_demo["message"] = f"Head-camera frame write failed: {exc}"
            raise ValueError(self._human_demo["message"]) from exc
        recorded_timestamp_ns = time.time_ns()
        entry = {
            "path": str(relative_path),
            "recorded_timestamp_ns": recorded_timestamp_ns,
        }
        for key in (
            "source_timestamp_ns",
            "device_time_ns",
            "device_frame_index",
            "width",
            "height",
        ):
            if key in metadata:
                entry[key] = int(metadata[key])
        self._human_demo_entries.append(entry)
        self._human_demo_last_frame_monotonic_ns = now_monotonic_ns
        self._human_demo["frame_count"] = frame_index
        self._human_demo["duration_seconds"] = round(
            max(
                0.0,
                (recorded_timestamp_ns - self._human_demo["started_timestamp_ns"])
                / 1e9,
            ),
            3,
        )

    def start_human_demo_recording(self) -> dict[str, Any]:
        """Start a bounded head-camera recording without issuing robot commands."""

        with self._lock:
            pigey_phase = str(self._pigey_status.get("phase", "")).upper()
            if (
                self._pigey_active_locked()
                and not (
                    pigey_phase == "WAITING_FOR_HUMAN_DEMO"
                    or (
                        pigey_phase == "ROUND_READY"
                        and bool(
                            self._pigey_status.get(
                                "demo_rerecord_enabled", False
                            )
                        )
                    )
                )
            ):
                raise ValueError("stop Pigey before recording a new human demo")
            if self._human_demo.get("active") or self._human_demo.get(
                "status"
            ) in {"finalizing", "limit_reached"}:
                raise ValueError(
                    "a human-demo recording is active or awaiting finalization"
                )
            lifecycle = str(self._robot_status.get("lifecycle_state", "")).lower()
            if lifecycle != "idle":
                raise ValueError("human-demo recording requires the robot lifecycle idle")
            head = self._camera_frames.get("head")
            if head is None or not self._camera_status_locked("head")["available"]:
                raise ValueError("head/top camera is not ready")
            started_timestamp_ns = time.time_ns()
            session_id = (
                time.strftime("%Y%m%d-%H%M%S")
                + f"-{started_timestamp_ns % 1_000_000_000:09d}"
            )
            directory = self.human_demo_root / session_id
            frames_dir = directory / "frames"
            try:
                frames_dir.mkdir(parents=True, exist_ok=False)
            except OSError as exc:
                raise ValueError(f"cannot create human-demo directory: {exc}") from exc
            self._human_demo_entries = []
            self._human_demo_last_frame_monotonic_ns = 0
            self._human_demo = {
                "status": "recording",
                "active": True,
                "camera": "head",
                "record_fps": self.human_demo_fps,
                "max_seconds": self.human_demo_max_seconds,
                "root": str(self.human_demo_root),
                "session_id": session_id,
                "directory": str(directory),
                "started_timestamp_ns": started_timestamp_ns,
                "frame_count": 0,
                "duration_seconds": 0.0,
                "message": "Recording the head/top camera only.",
            }
            self._record_human_demo_frame_locked(
                head, self._camera_metadata.get("head", {})
            )
            return dict(self._human_demo)

    def start_ui_recording(self) -> dict[str, Any]:
        """Start the recorder selected by the current deployment mode.

        Pigey keeps its legacy single head-camera input. Normal deployments use
        the same buttons for a read-only recording of all three UI previews.
        """
        with self._lock:
            if self._pigey_recording_mode_locked():
                use_pigey_recorder = True
            else:
                use_pigey_recorder = False
                frames = dict(self._camera_frames)
                metadata = dict(self._camera_metadata)
                deployment = str(
                    os.getenv("PIPER_DEPLOYMENT", "unregistered-deployment")
                )
                task = self.task
                policy_label = self.policy_label
                model_info = dict(self._model_info)
        if use_pigey_recorder:
            return {"recording": self.start_human_demo_recording()}
        return self.operator_video.start(
            frames=frames,
            metadata=metadata,
            deployment=deployment,
            task=task,
            policy_label=policy_label,
            model_info=model_info,
        )

    def stop_ui_recording(self) -> dict[str, Any]:
        with self._lock:
            use_pigey_recorder = self._pigey_recording_mode_locked()
        if use_pigey_recorder:
            return self.stop_human_demo_recording()
        return self.operator_video.stop()

    def recording_folder_status(self) -> dict[str, Any]:
        with self._lock:
            if self._pigey_recording_mode_locked():
                recording, root, deployment = dict(self._human_demo), self.human_demo_root, None
            else:
                recording = self.operator_video.snapshot()
                root, deployment = self.operator_video.root, self.deployment_id
        try:
            return {"available": True, **recording_folder(recording, root=root, deployment=deployment)}
        except (OSError, ValueError) as exc:
            return {"available": False, "error": str(exc)}

    def open_ui_recording_folder(self) -> dict[str, Any]:
        folder = self.recording_folder_status()
        if not folder["available"]:
            raise ValueError(folder["error"])
        return open_recording_folder(folder)

    @staticmethod
    def _encode_human_demo_mp4(
        frames_dir: Path, output_path: Path, fps: float
    ) -> str | None:
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            return "ffmpeg is unavailable; use the generated manifest or frame directory"
        try:
            subprocess.run(
                [
                    ffmpeg,
                    "-nostdin",
                    "-v",
                    "error",
                    "-y",
                    "-framerate",
                    f"{fps:.8f}",
                    "-i",
                    str(frames_dir / "frame_%06d.jpg"),
                    "-c:v",
                    "libx264",
                    "-preset",
                    "veryfast",
                    "-vf",
                    "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                    "-pix_fmt",
                    "yuv420p",
                    "-movflags",
                    "+faststart",
                    str(output_path),
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=180,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return f"MP4 encoding failed: {exc}"
        return None

    def stop_human_demo_recording(self) -> dict[str, Any]:
        """Finalize a Pigey-ready manifest and an MP4 from recorded head frames."""

        with self._lock:
            if not self._human_demo.get("active") and self._human_demo.get(
                "status"
            ) != "limit_reached":
                raise ValueError("no human-demo recording is active")
            self._human_demo["active"] = False
            self._human_demo["status"] = "finalizing"
            stopped_timestamp_ns = time.time_ns()
            entries = [dict(entry) for entry in self._human_demo_entries]
            directory = Path(self._human_demo["directory"])
            started_timestamp_ns = int(self._human_demo["started_timestamp_ns"])
            session_id = str(self._human_demo["session_id"])
        if len(entries) < 2:
            with self._lock:
                self._human_demo["status"] = "error"
                self._human_demo["message"] = (
                    "Record long enough to capture at least two head-camera frames."
                )
            raise ValueError(self._human_demo["message"])

        duration_seconds = max(
            0.0, (stopped_timestamp_ns - started_timestamp_ns) / 1e9
        )
        manifest_path = directory / "demo_manifest.json"
        manifest = {
            "schema_version": "xpolicy.pigey_human_demo/v1",
            "session_id": session_id,
            "camera": "head",
            "source": "xpolicy.operator_ui",
            "started_timestamp_ns": started_timestamp_ns,
            "stopped_timestamp_ns": stopped_timestamp_ns,
            "duration_seconds": round(duration_seconds, 3),
            "record_fps": self.human_demo_fps,
            "entries": entries,
        }
        temporary_manifest = manifest_path.with_suffix(".json.tmp")
        try:
            temporary_manifest.write_text(json.dumps(manifest, indent=2) + "\n")
            temporary_manifest.replace(manifest_path)
        except OSError as exc:
            with self._lock:
                self._human_demo["status"] = "error"
                self._human_demo["message"] = f"Demo manifest write failed: {exc}"
            raise ValueError(self._human_demo["message"]) from exc
        video_path = directory / "human_demo.mp4"
        video_error = self._encode_human_demo_mp4(
            directory / "frames", video_path, self.human_demo_fps
        )
        with self._lock:
            self._human_demo.update(
                {
                    "status": "ready",
                    "active": False,
                    "stopped_timestamp_ns": stopped_timestamp_ns,
                    "duration_seconds": round(duration_seconds, 3),
                    "frame_count": len(entries),
                    "manifest_path": str(manifest_path),
                    "video_path": str(video_path) if video_error is None else None,
                    "pigey_input_path": (
                        str(video_path) if video_error is None else str(manifest_path)
                    ),
                    "video_error": video_error,
                    "message": (
                        "Saved; the launcher is validating and loading this into Pigey."
                        if self.human_demo_required
                        else "Ready for Pigey multi-task inference."
                        if video_error is None
                        else "Frames and manifest are ready; MP4 encoding failed."
                    ),
                }
            )
            return dict(self._human_demo)

    def camera_frame(self, name: str) -> tuple[bytes, dict[str, Any]] | None:
        camera_name = str(name).strip().lower()
        if camera_name not in CAMERA_NAMES:
            return None
        with self._lock:
            frame = self._camera_frames.get(camera_name)
            if frame is None or not self._camera_status_locked(camera_name)[
                "available"
            ]:
                return None
            return frame, dict(self._camera_metadata.get(camera_name, {}))

    def update_pigey_status(self, value: Mapping[str, Any]) -> None:
        """Store bounded supervisor telemetry without adding a Dora control path."""
        status = dict(value)
        status.pop("source", None)
        phase = str(status.get("phase", "")).strip().upper()
        session_id = str(status.get("session_id", "")).strip()
        if not phase or len(phase) > 64:
            raise ValueError("Pigey status phase must be a non-empty bounded string")
        if not session_id or len(session_id) > 128:
            raise ValueError("Pigey status session_id must be a non-empty bounded string")
        decision = status.get("decision")
        if decision is not None and str(decision).upper() not in {
            "ACT",
            "NEXT_OBJECT",
            "DONE",
        }:
            raise ValueError(
                "Pigey decision must be ACT, NEXT_OBJECT, DONE, or null"
            )
        observation_b64 = status.pop("observation_jpeg_base64", None)
        observation: bytes | None = None
        if observation_b64 is not None:
            if not isinstance(observation_b64, str):
                raise ValueError("Pigey observation JPEG must be base64 text")
            try:
                observation = base64.b64decode(observation_b64, validate=True)
            except (binascii.Error, ValueError) as exc:
                raise ValueError("Pigey observation JPEG is not valid base64") from exc
            if not 0 < len(observation) <= MAX_PIGEY_OBSERVATION_BYTES:
                raise ValueError("Pigey observation JPEG exceeds the bounded size")
            if not observation.startswith(b"\xff\xd8"):
                raise ValueError("Pigey observation payload is not a JPEG")
            digest = hashlib.sha256(observation).hexdigest()
            declared_digest = str(status.get("observation_sha256", "")).lower()
            if declared_digest and not secrets.compare_digest(digest, declared_digest):
                raise ValueError("Pigey observation SHA-256 does not match the JPEG")
            status["observation_sha256"] = digest
            status["observation_bytes"] = len(observation)
        status["phase"] = phase
        status["session_id"] = session_id
        status["server_timestamp_ns"] = time.time_ns()
        with self._lock:
            if observation is not None:
                self._pigey_observation = observation
                self._pigey_observation_metadata = {
                    "sha256": status["observation_sha256"],
                    "bytes": len(observation),
                }
            elif self._pigey_observation_metadata:
                status.setdefault(
                    "observation_sha256",
                    self._pigey_observation_metadata["sha256"],
                )
                status.setdefault(
                    "observation_bytes",
                    self._pigey_observation_metadata["bytes"],
                )
            self._pigey_status = status

    def pigey_observation_frame(self) -> tuple[bytes, dict[str, Any]] | None:
        with self._lock:
            if self._pigey_observation is None:
                return None
            return self._pigey_observation, dict(self._pigey_observation_metadata)

    def snapshot(self) -> dict[str, Any]:
        folder_status = self.recording_folder_status()
        with self._lock:
            camera_now_ns = time.monotonic_ns()
            effective_steps = self._model_info.get("execution_chunk_steps")
            if effective_steps is None:
                effective_steps = self._policy_status.get(
                    "execution_chunk_effective_steps"
                )
            pigey_status = dict(self._pigey_status)
            pigey_age_seconds = None
            if pigey_status:
                pigey_age_seconds = max(
                    0.0,
                    (time.time_ns() - int(pigey_status["server_timestamp_ns"])) / 1e9,
                )
            return {
                "task": self.task,
                "tasks": list(self.tasks),
                "write_enabled": self.write_enabled,
                "control_source": self.control_source,
                "dagger_expert_source": self.expert_source,
                "expert": {
                    "source": self.expert_source,
                    "leader_role": self.leader_follower.leader_role,
                    "follower_role": self.leader_follower.follower_role,
                    "feedback_role": self.leader_follower.feedback_role,
                    "native_linkage": self.leader_follower.native_linkage,
                    "max_age_ms": self.expert_max_age_ms,
                    "locked": self._controls_locked(),
                },
                "last_command": self._last_command,
                "gripper_control": {
                    "supports_open_left_only": True,
                    "can_open": not bool(self._gripper_open_rejection_locked()),
                    "reason": self._gripper_open_rejection_locked(),
                    "last_request": dict(self._robot_status.get("gripper_control", {})),
                },
                "command_pending": not self._commands.empty() or time.monotonic() - self._last_command_at < 2.0,
                "robot_status": dict(self._robot_status),
                "robot_status_age_seconds": time.monotonic() - self._robot_status_at if self._robot_status_at else None,
                "policy_status": dict(self._policy_status),
                "model_info": dict(self._model_info),
                "policy_label": self.policy_label,
                "deployment": {
                    "id": self.deployment_id,
                    "label": self.policy_label,
                },
                "robot_state": {
                    "available": bool(self._robot_state),
                    "feedback_metadata": dict(self._robot_state_metadata),
                    "schema_id": "xpolicy.piper_x_dual.state.v1",
                    "names": list(ACTION_NAMES),
                    "values": [self._robot_state[name] for name in ACTION_NAMES]
                    if self._robot_state
                    else [],
                    "units": ["rad"] * 6 + ["m"] + ["rad"] * 6 + ["m"],
                },
                "rollout": {
                    "configured_max_control_steps": self.rollout_max_control_steps,
                    "locked": self._controls_locked(),
                    "owner": dict(self._robot_status.get("rollout", {})),
                },
                "execution": {
                    "mode": self.execution_mode,
                    "fixed_steps": self.fixed_execute_chunk,
                    "default_steps": self.default_execute_chunk,
                    "action_horizon": self.action_horizon,
                    "effective_steps": effective_steps,
                    "locked": self._controls_locked(),
                },
                "rtc": {
                    "mode": self.rtc_config.mode.value,
                    "request_remaining_steps": (
                        self.rtc_config.request_remaining_steps
                    ),
                    "transition_steps": self.rtc_config.transition_steps,
                    "transition_curve": self.rtc_config.transition_curve.value,
                    "locked": self._controls_locked(),
                },
                "collection": {
                    "mode": self.collection_mode,
                    "locked": True,
                    "recorder_enabled": self.recorder_enabled,
                    "record_dir": self.record_dir,
                    "episode_active": bool(self._robot_status.get("recording", False)),
                    "selected_action_source": self._robot_status.get(
                        "selected_action_source", "HOLD"
                    ),
                    "label_valid": bool(self._robot_status.get("label_valid", False)),
                    "lifecycle_state": self._robot_status.get(
                        "lifecycle_state", "idle"
                    ),
                },
                "pigey": {
                    **pigey_status,
                    "operator_control": dict(self._pigey_operator_control),
                    "operator_controls": [
                        dict(control) for control in self._pigey_operator_controls
                    ],
                    "available": bool(pigey_status),
                    "active": self._pigey_active_locked(),
                    "age_seconds": pigey_age_seconds,
                    "observation_available": self._pigey_observation is not None,
                    "sequence_presets": [
                        {
                            "id": preset["id"],
                            "label": preset["label"],
                            "prompts": list(preset["prompts"]),
                        }
                        for preset in self.pigey_sequence_presets
                    ],
                    "sequence_selection": dict(self._pigey_sequence_selection),
                },
                "human_demo": dict(self._human_demo),
                "operator_recording": self.operator_video.snapshot(),
                "recording_folder": folder_status,
                "cameras": {
                    name: self._camera_status_locked(name, now_ns=camera_now_ns)
                    for name in CAMERA_NAMES
                },
            }


def operator_page(
    state: PiperOperatorState,
    token: str,
    deployment_switch: DeploymentSwitcher | None = None,
) -> bytes:
    mode = "LIVE WRITES ENABLED" if state.write_enabled else "SHADOW / NO WRITE"
    mode_class = "live" if state.write_enabled else "shadow"
    token_json = json.dumps(token)
    session_json = json.dumps(hashlib.sha256(token.encode()).hexdigest())
    keyboard_checked = " checked" if state.control_source == "keyboard" else ""
    task_options = "".join(
        f'<option value="{html.escape(task, quote=True)}"'
        f'{" selected" if task == state.task else ""}>'
        f'{html.escape(task)}</option>'
        for task in state.tasks
    )
    sequence_options = "".join(
        f'<option value="{html.escape(str(preset["id"]), quote=True)}">'
        f'{html.escape(str(preset["label"]))}</option>'
        for preset in state.pigey_sequence_presets
    )
    execution_options = "".join(
        f'<option value="{steps}"'
        f'{" selected" if steps == state.fixed_execute_chunk else ""}>'
        f'{steps} step{"s" if steps != 1 else ""}</option>'
        for steps in range(1, state.action_horizon + 1)
    )
    rtc_request_options = (
        '<option value="0">Auto latency</option>'
        + "".join(
            f'<option value="{steps}">{steps} left</option>'
            for steps in range(1, state.action_horizon)
        )
    )
    rtc_blend_options = "".join(
        f'<option value="{steps}">'
        f'{"No blend" if steps == 0 else f"{steps} blend"}</option>'
        for steps in range(state.action_horizon)
    )
    expert_source_options = "".join(
        f'<option value="{source}"'
        f'{" selected" if source == state.expert_source else ""}>'
        f'{"QNexo / retarget" if source == "qnexo" else "Leader → follower SDK bridge"}</option>'
        for source in ("qnexo", "leader_follower")
    )
    role_options = {
        role: "".join(
            f'<option value="{candidate}"'
            f'{" selected" if candidate == role else ""}>{candidate.replace("_", " ")}</option>'
            for candidate in ("left_arm", "right_arm")
        )
        for role in (
            state.leader_follower.leader_role,
            state.leader_follower.follower_role,
            state.leader_follower.feedback_role,
        )
    }
    expert_age_values = sorted({50, 75, 100, 150, 200, 300, int(state.expert_max_age_ms)})
    expert_age_options = "".join(
        f'<option value="{age}"'
        f'{" selected" if abs(age - state.expert_max_age_ms) < 1e-6 else ""}>{age} ms</option>'
        for age in expert_age_values
    )
    expert_panel_hidden = " hidden" if state.collection_mode == "OFF" else ""
    policy_label = html.escape(state.policy_label)
    if os.getenv("PIPER_NATIVE_TRAJECTORY_PROFILE"):
        policy_label += " · 异步轨迹平滑试验"
    if os.getenv("PIPER_NATIVE_VELOCITY_SEAM_PROFILE"):
        policy_label += " · 速度衔接试验"
    if os.getenv("PIPER_NATIVE_ONE_EURO_PROFILE"):
        policy_label += " · One Euro 限幅试验"
    if os.getenv("PIPER_NATIVE_JOINT_CLIP_PROFILE"):
        policy_label += " · 左J4裁切试验（93°内→89°）"
    switch_panel = ""
    if deployment_switch is not None:
        deployment_buttons = "".join(
            "<button class=\"deployment-switch"
            f"{' current' if option.name == deployment_switch.deployment else ''}\" "
            f"data-deployment=\"{html.escape(option.name, quote=True)}\" "
            f"aria-current=\"{'true' if option.name == deployment_switch.deployment else 'false'}\" "
            f"{'disabled' if option.name == deployment_switch.deployment else ''}>"
            f"{'当前运行中 · ' if option.name == deployment_switch.deployment else ''}"
            f"{html.escape(option.label)}</button>"
            for option in deployment_switch.catalog
        )
        switch_panel = f"""
  <section class=\"panel deployment-panel\">
    <div class=\"deployment-head\"><div><div class=\"deployment-title\">个人部署包 · 一键切换</div><div class=\"deployment-help\"><span id=\"deployment-count\">{len(deployment_switch.catalog)}</span> 个本机已登记包 · 点击后重启 Web 和模型，自动重新连接。不会自动 Prepare 或 Start。</div></div><span id=\"deployment-current\" class=\"deployment-current\">{html.escape(deployment_switch.deployment)}</span></div>
    <div class=\"deployment-buttons\">{deployment_buttons}</div>
    <div class="deployment-help">每段执行步数 · 预测长度仍为 50 步。修改后自动重启，切换模型时保留所选步数。</div>
    <div class="execution-presets" role="group" aria-label="每段执行步数">
      {''.join(f'<button class="deployment-switch" data-execution-steps="{steps}" disabled>{steps} 步</button>' for steps in (10, 15, 20, 25))}
    </div>
    <div id=\"deployment-status\" class=\"field-note\" role=\"status\" aria-live=\"polite\">仅在策略暂停、未运行且未录制时可切换。</div>
    <details class=\"deployment-intake\"><summary>＋ 接入部署包</summary>
      <p class=\"deployment-help\">填写服务器上的个人部署包路径，先检查再登记。完整验证通过后，按钮会自动出现；登记不会切换当前模型。</p>
      <form id=\"intake-form\" class=\"intake-fields\">
        <label>部署包目录<input id=\"intake-bundle\" required autocomplete=\"off\" placeholder=\"/home/ubuntu/code_junqi/model-store/models/pi05/新包目录\"></label>
        <label>部署名（可选，自动匹配）<input id=\"intake-name\" autocomplete=\"off\" placeholder=\"junqi-…\"></label>
        <label>按钮名称（可选）<input id=\"intake-label\" maxlength=\"120\" autocomplete=\"off\" placeholder=\"例如：新模型 EMA+4000\"></label>
        <div class=\"intake-actions\"><button id=\"intake-inspect\" type=\"submit\">检查部署包</button><button id=\"intake-register\" type=\"button\" disabled>加入切换列表</button></div>
      </form>
      <div id=\"intake-status\" class=\"field-note\" role=\"status\" aria-live=\"polite\">还没有新包时无需操作。支持与当前机器人、Native RTC 配置兼容的个人包。</div>
      <pre id=\"intake-command\" hidden></pre>
    </details>
  </section>"""
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Dual-Piper Operator Controls</title>
  <style>
    :root {{ color-scheme: dark; font-family: Inter, ui-sans-serif, system-ui, sans-serif; background: #090b0e; color: #f5f7fa; }}
    * {{ box-sizing: border-box; }}
    body {{ max-width: 1840px; margin: 0 auto; padding: 22px clamp(16px, 2vw, 34px) 32px; background: #090b0e; }}
    header {{ display: flex; align-items: center; justify-content: space-between; gap: 16px; margin-bottom: 20px; }}
    h1 {{ margin: 0; font-size: 1.36rem; font-weight: 650; letter-spacing: -.035em; }}
    .subtle {{ margin-top: 4px; color: #7f8792; font-size: .82rem; }}
    .mode {{ padding: 7px 11px; border: 1px solid #303640; border-radius: 999px; font-size: .72rem; font-weight: 800; letter-spacing: .06em; }}
    .shadow {{ color: #8ec5ff; background: #101b2a; }} .live {{ color: #ffb4b4; background: #2a1114; border-color: #6b252d; }}
    .panel {{ background: #111419; border: 1px solid #222831; border-radius: 14px; padding: 15px; margin-bottom: 14px; }}
    .setup-grid {{ display: grid; grid-template-columns: minmax(340px, 1.45fr) minmax(260px, .75fr) minmax(470px, 1.2fr) auto; align-items: end; gap: 14px; }}
    .field {{ min-width: 0; }}
    label, .label {{ display: block; margin-bottom: 7px; color: #858e9a; font-size: .7rem; font-weight: 750; text-transform: uppercase; letter-spacing: .085em; }}
    select {{ width: 100%; min-height: 44px; padding: 0 38px 0 12px; color: #f4f4f5; background: #0a0d11; border: 1px solid #303743; border-radius: 9px; font: inherit; }}
    select:disabled {{ opacity: .55; }}
    .execution-pair {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }}
    .rtc-pair {{ display: grid; grid-template-columns: 1.15fr .9fr .9fr 1.15fr; gap: 8px; }}
    .deployment-panel {{ border-color: #4a3c23; background: linear-gradient(135deg, #1b1710, #111419 62%); }}
    .deployment-head {{ display: flex; align-items: start; justify-content: space-between; gap: 14px; margin-bottom: 12px; }}
    .deployment-title {{ color: #f0d9a7; font-size: .95rem; font-weight: 750; }}
    .deployment-help {{ color: #a9997c; font-size: .74rem; margin-top: 3px; }}
    .deployment-current {{ max-width: 50%; overflow-wrap: anywhere; color: #d9c496; font: .73rem ui-monospace, SFMono-Regular, Menlo, monospace; text-align: right; }}
    .deployment-buttons {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }}
    .deployment-switch {{ min-height: 44px; font-size: .78rem; }}
    .deployment-switch.current {{ color: #f0d9a7; background: #2b2415; border-color: #8b6c2a; }}
    .deployment-intake {{ margin-top: 16px; border-top: 1px solid #423724; padding-top: 12px; }}
    .deployment-intake summary {{ cursor: pointer; color: #f0d9a7; font-weight: 650; }}
    .intake-fields {{ display: grid; gap: 10px; margin: 12px 0; }}
    .intake-fields label {{ display: grid; gap: 5px; font-size: .8rem; color: #c5cbd2; }}
    .intake-fields input {{ width: 100%; min-width: 0; padding: 11px; border: 1px solid #363e49; border-radius: 8px; color: #eee; background: #0c1015; }}
    .intake-actions {{ display: flex; flex-wrap: wrap; gap: 8px; }}
    #intake-command {{ white-space: pre-wrap; overflow-wrap: anywhere; user-select: all; }}
    .expert-panel {{ border-color: #30445d; background: linear-gradient(135deg, #101923, #111419 62%); }}
    .expert-head {{ display: flex; align-items: baseline; justify-content: space-between; gap: 12px; margin-bottom: 12px; }}
    .expert-title {{ color: #dce9f8; font-size: .95rem; font-weight: 750; }}
    .expert-help {{ color: #718196; font-size: .74rem; }}
    .expert-grid {{ display: grid; grid-template-columns: 1.3fr repeat(3, 1fr) .8fr auto; align-items: end; gap: 10px; }}
    .expert-grid button {{ min-height: 44px; white-space: nowrap; }}
    .expert-note {{ margin-top: 10px; color: #7d8ea2; font-size: .74rem; line-height: 1.4; }}
    .field-note {{ min-height: 17px; margin-top: 6px; color: #68717d; font-size: .73rem; }}
    .runtime {{ min-width: 145px; padding: 0 8px 11px; color: #aeb4be; font-size: .82rem; white-space: nowrap; }}
    .dot {{ display: inline-block; width: 8px; height: 8px; margin-right: 7px; border-radius: 50%; background: #64748b; }}
    .dot.ready {{ background: #34d399; box-shadow: 0 0 0 3px #123c31; }}
    .cameras {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; padding: 0; background: transparent; border: 0; }}
    figure {{ margin: 0; overflow: hidden; background: #05070a; border: 1px solid #222831; border-radius: 13px; }}
    figure img {{ display: block; width: 100%; aspect-ratio: 16 / 9; object-fit: contain; background: #05070a; }}
    figure img.unavailable {{ opacity: 0; }}
    figcaption {{ display: flex; align-items: center; justify-content: space-between; padding: 10px 12px; color: #d7dae0; font-size: .79rem; }}
    .camera-label {{ font-weight: 700; letter-spacing: .02em; }}
    .camera-contract {{ margin-left: 7px; color: #646d79; font-size: .7rem; font-weight: 500; }}
    .camera-state {{ color: #737b87; font-size: .72rem; }}
    .buttons {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }}
    button {{ min-height: 50px; padding: 0 14px; color: #f4f4f5; background: #20252d; border: 1px solid #343a45; border-radius: 10px; font-size: .9rem; font-weight: 750; cursor: pointer; transition: background .12s, transform .08s; }}
    button:hover {{ background: #2a303a; }} button:active {{ transform: translateY(1px); }}
    button:disabled {{ opacity: .48; cursor: not-allowed; }} button:disabled:hover {{ background: #20252d; }}
    button[data-command=start] {{ background: #1769e0; border-color: #2379ee; }}
    button[data-command=start]:hover {{ background: #2275ec; }}
    button[data-command=stop] {{ color: #ffd89a; border-color: #6f5020; }}
    button[data-command=finish] {{ color: #ffb4b4; border-color: #6b252d; }}
    .control-meta {{ display: flex; justify-content: space-between; gap: 18px; margin-top: 13px; color: #9299a5; font-size: .8rem; }}
    .shortcut {{ display: inline; margin: 0; text-transform: none; letter-spacing: 0; cursor: pointer; }}
    .warning {{ color: #d9a8a8; text-align: right; }}
    .collection-panel {{ border-color: #314457; background: linear-gradient(135deg, #101821, #111419 62%); }}
    .collection-grid {{ display: grid; grid-template-columns: minmax(210px, .7fr) repeat(3, minmax(150px, .55fr)) minmax(280px, 1.4fr) auto; align-items: end; gap: 12px; }}
    .collection-value {{ min-height: 44px; display: flex; align-items: center; padding: 0 12px; border: 1px solid #293441; border-radius: 9px; background: #0a0e13; color: #dce4ed; font-size: .82rem; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }}
    .collection-actions {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }}
    .collection-actions button {{ min-height: 44px; white-space: nowrap; }}
    button[data-command=discard] {{ color: #ffb4b4; border-color: #6b252d; }}
    [hidden] {{ display: none !important; }}
    .pigey-panel {{ border-color: #245849; background: linear-gradient(135deg, #101c1a, #111419 58%); }}
    .pigey-head {{ display: flex; align-items: center; justify-content: space-between; gap: 14px; margin-bottom: 13px; }}
    .pigey-title {{ display: flex; align-items: center; gap: 9px; font-size: 1rem; font-weight: 750; }}
    .pigey-badge {{ padding: 5px 9px; border: 1px solid #2b6c57; border-radius: 999px; color: #75e0ba; background: #102c24; font-size: .7rem; font-weight: 800; letter-spacing: .06em; }}
    .pigey-grid {{ display: grid; grid-template-columns: minmax(280px, .85fr) minmax(460px, 1.5fr); gap: 14px; }}
    .pigey-proof {{ margin: 0; border-color: #285344; }}
    .pigey-proof img {{ aspect-ratio: 16 / 9; }}
    .pigey-metrics {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 9px; }}
    .pigey-metric {{ min-width: 0; padding: 11px; border: 1px solid #26333b; border-radius: 10px; background: #0a1013; }}
    .pigey-value {{ overflow: hidden; color: #f5f7fa; font-size: .92rem; font-weight: 700; text-overflow: ellipsis; white-space: nowrap; }}
    .pigey-value.act {{ color: #70b7ff; }} .pigey-value.next_object {{ color: #f5c66f; }} .pigey-value.done {{ color: #75e0ba; }}
    .pigey-wide {{ grid-column: span 2; }}
    .pigey-evidence {{ margin-top: 10px; color: #7f8c94; font: .72rem/1.45 ui-monospace, SFMono-Regular, Menlo, monospace; overflow-wrap: anywhere; }}
    .pigey-sequence {{ display: grid; grid-template-columns: minmax(220px, 1fr) auto; gap: 10px; align-items: end; margin: 0 0 14px; padding: 12px; border: 1px solid #285344; border-radius: 10px; background: #0a1714; }}
    .pigey-sequence .field-note {{ grid-column: 1 / -1; }}
    .demo-panel {{ border-color: #3f4651; background: linear-gradient(135deg, #15171c, #111419 62%); }}
    .demo-layout {{ display: grid; grid-template-columns: minmax(230px, .55fr) minmax(420px, 1.45fr); align-items: center; gap: 14px; }}
    .demo-buttons {{ display: grid; grid-template-columns: 1fr 1fr; gap: 9px; }}
    .demo-buttons button {{ min-height: 44px; }}
    #demo-start {{ color: #ffb4b4; border-color: #71303a; }}
    #demo-stop {{ color: #a7f3d0; border-color: #28604c; }}
    .demo-status {{ color: #c8cdd5; font-size: .84rem; line-height: 1.5; }}
    .demo-path {{ display: block; margin-top: 7px; color: #8ec5ff; font: .72rem/1.45 ui-monospace, SFMono-Regular, Menlo, monospace; overflow-wrap: anywhere; user-select: all; }}
    details {{ color: #9299a5; font-size: .82rem; }} summary {{ cursor: pointer; }}
    pre {{ max-height: 260px; margin: 12px 0 0; padding: 12px; overflow: auto; color: #cbd5e1; background: #080a0d; border-radius: 9px; }}
    @media (max-width: 1320px) {{ .setup-grid, .collection-grid {{ grid-template-columns: 1fr 1fr; }} .deployment-buttons {{ grid-template-columns: repeat(3, minmax(0, 1fr)); }} .expert-grid {{ grid-template-columns: repeat(3, 1fr); }} .expert-grid button {{ grid-column: span 1; }} .runtime {{ grid-column: 1 / -1; padding: 0; }} }}
    @media (max-width: 900px) {{ .cameras, .pigey-grid, .demo-layout {{ grid-template-columns: 1fr; }} .deployment-buttons {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }} .pigey-metrics {{ grid-template-columns: repeat(2, 1fr); }} .expert-grid {{ grid-template-columns: 1fr 1fr; }} }}
    @media (max-width: 700px) {{ .setup-grid {{ grid-template-columns: 1fr; }} .deployment-head {{ display: block; }} .deployment-current {{ display: block; max-width: none; margin-top: 6px; text-align: left; }} .deployment-buttons {{ grid-template-columns: 1fr; }} .expert-grid {{ grid-template-columns: 1fr; }} .buttons {{ grid-template-columns: repeat(2, 1fr); }} .control-meta {{ display: block; }} .warning {{ margin-top: 10px; text-align: left; }} }}
  </style>
</head>
<body>
  <header>
    <div><h1>Dual-PiPER Operator</h1><div class="subtle">{policy_label}</div></div>
    <span class="mode {mode_class}">{mode}</span>
  </header>
  <section class="panel setup-grid">
    <div class="field">
      <label for="task">Task prompt</label>
      <select id="task">{task_options}</select>
      <div class="field-note">Exact prompt from this model bundle</div>
    </div>
    <div class="field">
      <label for="execution-mode">Execution chunk</label>
      <div class="execution-pair">
        <select id="execution-mode">
          <option value="auto"{" selected" if state.execution_mode == "auto" else ""}>Auto</option>
          <option value="fixed"{" selected" if state.execution_mode == "fixed" else ""}>Fixed</option>
        </select>
        <select id="execution-steps">{execution_options}</select>
      </div>
      <div id="execution-note" class="field-note">Auto uses the model recommendation</div>
    </div>
    <div class="field">
      <label for="rtc-mode">Real-time chunking</label>
      <div class="rtc-pair">
        <select id="rtc-mode">
          <option value="disabled"{" selected" if state.rtc_config.mode.value == "disabled" else ""}>Off</option>
          <option value="align"{" selected" if state.rtc_config.mode.value == "align" else ""}>Align</option>
          <option value="blend"{" selected" if state.rtc_config.mode.value == "blend" else ""}>Aligned blend</option>
        </select>
        <select id="rtc-request" title="Start inference with this many queued actions remaining">{rtc_request_options}</select>
        <select id="rtc-transition" title="Aligned actions to blend after inference returns">{rtc_blend_options}</select>
        <select id="rtc-curve" title="Blend curve">
          <option value="linear">Linear</option>
          <option value="smoothstep">Smooth</option>
          <option value="cosine">Cosine</option>
        </select>
      </div>
      <div id="rtc-note" class="field-note">Off · finish the current chunk before inferring again</div>
    </div>
    <div class="runtime"><span id="runtime-dot" class="dot"></span><span id="lifecycle">Connecting</span></div>
  </section>
  {switch_panel}
  <section class="panel expert-panel"{expert_panel_hidden}>
    <div class="expert-head"><div class="expert-title">DAgger expert control</div><div class="expert-help">Apply while idle or prepared; locked during an episode</div></div>
    <div class="expert-grid">
      <div class="field"><label for="expert-source">Source</label><select id="expert-source">{expert_source_options}</select></div>
      <div class="field"><label for="expert-leader">Leader</label><select id="expert-leader">{role_options[state.leader_follower.leader_role]}</select></div>
      <div class="field"><label for="expert-follower">Follower</label><select id="expert-follower">{role_options[state.leader_follower.follower_role]}</select></div>
      <div class="field"><label for="expert-feedback">Joint feedback</label><select id="expert-feedback">{role_options[state.leader_follower.feedback_role]}</select></div>
      <div class="field"><label for="expert-age">Max age</label><select id="expert-age">{expert_age_options}</select></div>
      <button id="expert-apply">Apply</button>
    </div>
    <div class="expert-note">QNexo keymap: right white = Prepare · right blue = Start · right red = Finish · right footswitch = takeover toggle · left red = Discard (expert-only data mode). The source selector changes only the continuous expert action path; lifecycle and safety ownership stay in the owner.</div>
    <div id="expert-note" class="expert-note">Leader → follower reads the selected SDK leader feedback and dispatches only the follower arm; firmware-native linkage remains disabled.</div>
  </section>
  <section id="collection-panel" class="panel collection-panel" hidden>
    <div class="collection-grid">
      <div class="field">
        <label for="collection-mode">DAgger mode</label>
        <select id="collection-mode" disabled>
          <option value="MIXED">Mixed · model + takeover</option>
          <option value="DATA_COLLECTION">Data · expert only</option>
        </select>
        <div class="field-note">Launch parameter · restart to change</div>
      </div>
      <div><span class="label">Recorder</span><div id="collection-recorder" class="collection-value">Waiting</div></div>
      <div><span class="label">Episode</span><div id="collection-episode" class="collection-value">Idle</div></div>
      <div><span class="label">Applied source</span><div id="collection-source" class="collection-value">HOLD</div></div>
      <div><span class="label">Expert source</span><div id="collection-expert-source" class="collection-value">{html.escape(state.expert_source)}</div></div>
      <div><span class="label">Output</span><div id="collection-output" class="collection-value">—</div></div>
      <div class="collection-actions">
        <button id="collection-save" data-command="finish">Save</button>
        <button id="collection-discard" data-command="discard">Discard</button>
      </div>
    </div>
  </section>
  <section class="cameras panel">
    <figure><img id="camera-left" alt=""><figcaption><span><span class="camera-label">Left wrist</span><span class="camera-contract">camera_left</span></span><span id="camera-left-state" class="camera-state">Waiting</span></figcaption></figure>
    <figure><img id="camera-head" alt=""><figcaption><span><span class="camera-label">Head</span><span class="camera-contract">camera_head</span></span><span id="camera-head-state" class="camera-state">Waiting</span></figcaption></figure>
    <figure><img id="camera-right" alt=""><figcaption><span><span class="camera-label">Right wrist</span><span class="camera-contract">camera_right</span></span><span id="camera-right-state" class="camera-state">Waiting</span></figcaption></figure>
  </section>
  <section class="panel" aria-label="硬件连接与故障">
    <button id="reconnect-hardware" disabled>重新连接硬件和相机</button>
    <span id="reconnect-feedback" class="field-note" role="status" aria-live="polite">暂停且未录制时可重连；保留当前模型和读写模式。</span>
    <div class="deployment-buttons" style="margin-top:12px">
      <div><strong>左臂</strong><div id="health-left_arm" role="status">等待反馈</div></div>
      <div><strong>右臂</strong><div id="health-right_arm" role="status">等待反馈</div></div>
      <div><strong>左夹爪</strong><div id="health-left_gripper" role="status">等待反馈</div></div>
      <div><strong>右夹爪</strong><div id="health-right_gripper" role="status">等待反馈</div></div>
    </div>
  </section>
  <section id="human-demo-panel" class="panel demo-panel">
    <div class="demo-layout">
      <div>
        <div id="recording-title" class="label">Operator video · left/head/right UI views</div>
        <div class="demo-buttons">
          <button id="demo-start">● Start recording</button>
          <button id="demo-stop" disabled>■ Stop &amp; save</button>
          <button id="demo-open-folder" disabled>打开录像文件夹</button>
        </div>
      </div>
      <div class="demo-status">
        <div id="demo-status">Idle · records vision only and never commands the robot.</div>
        <code id="demo-path" class="demo-path">Saved MP4/manifest path will appear here.</code>
        <div id="demo-folder-feedback" class="field-note" role="status" aria-live="polite"></div>
      </div>
    </div>
  </section>
  <section id="pigey-panel" class="panel pigey-panel" hidden>
    <div class="pigey-head">
      <div class="pigey-title"><span id="pigey-dot" class="dot"></span>Pigey visual supervisor</div>
      <span id="pigey-phase" class="pigey-badge">WAITING</span>
    </div>
    <div id="pigey-sequence-picker" class="pigey-sequence" hidden>
      <div class="field">
        <label for="pigey-sequence-select">Fixed task sequence</label>
        <select id="pigey-sequence-select">{sequence_options}</select>
      </div>
      <button id="pigey-sequence-run">Run selected sequence</button>
      <div id="pigey-sequence-note" class="field-note">Demo induction is off. Live VLM completion checks remain enabled.</div>
    </div>
    <div class="pigey-grid">
      <figure class="pigey-proof">
        <img id="pigey-observation" class="unavailable" alt="Exact image submitted to Qwen">
        <figcaption><span class="camera-label">VLM input</span><span id="pigey-image-state" class="camera-state">Waiting</span></figcaption>
      </figure>
      <div>
        <div class="pigey-metrics">
          <div class="pigey-metric"><span class="label">Rule</span><div id="pigey-rule" class="pigey-value">—</div></div>
          <div class="pigey-metric"><span class="label">Decision</span><div id="pigey-decision" class="pigey-value">—</div></div>
          <div class="pigey-metric"><span class="label">Transition proof</span><div id="pigey-done" class="pigey-value">0 / 2</div></div>
          <div class="pigey-metric"><span class="label">Current object ACT</span><div id="pigey-actions" class="pigey-value">0</div></div>
          <div class="pigey-metric"><span class="label">Objects completed</span><div id="pigey-objects" class="pigey-value">0</div></div>
          <div class="pigey-metric pigey-wide"><span class="label">Policy prompt</span><div id="pigey-prompt" class="pigey-value">—</div></div>
          <div class="pigey-metric"><span class="label">Model steps</span><div id="pigey-rollout" class="pigey-value">0</div></div>
          <div class="pigey-metric"><span class="label">Task steps</span><div id="pigey-task-steps" class="pigey-value">0</div></div>
          <div class="pigey-metric"><span class="label">VLM request</span><div id="pigey-request" class="pigey-value">—</div></div>
        </div>
        <div id="pigey-evidence" class="pigey-evidence">Waiting for authenticated Pigey telemetry.</div>
      </div>
    </div>
  </section>
  <section class="panel">
    <div class="buttons">
      <button data-command="prepare">I · Prepare</button>
      <button data-command="start">S · Start</button>
      <button data-command="stop">X · Hold</button>
      <button data-command="finish">F · Finish</button>
    </div>
    <div class="control-meta">
      <button id="open-grippers" data-command="open_grippers" disabled>张开左右夹爪</button>
      <span id="gripper-feedback" class="warning" aria-live="polite">暂停时张开双夹爪；按 Prepare 回初始姿态并闭合。</span>
    </div>
    <div class="control-meta">
      <label class="shortcut"><input id="keyboard" type="checkbox"{keyboard_checked}> Enable keyboard shortcuts</label>
      <span class="warning">Prepare moves to the configured initial pose in live mode.</span>
      <span id="pigey-control-lock" class="warning" hidden>I/S/X/F control Pigey: X holds immediately · I initializes and pauses · S resumes the cached sequence · F exits.</span>
      <span id="command-feedback" class="warning" aria-live="polite"></span>
      <span id="hardware-feedback" class="warning" aria-live="polite"></span>
    </div>
  </section>
  <section class="panel"><details><summary>Runtime details</summary><pre id="status">Waiting…</pre></details></section>
  <script>
    const token = {token_json};
    const operatorSession = {session_json};
    const commandFeedback = document.getElementById('command-feedback');
    let reconnectError = '';
    async function post(path, payload) {{
      const response = await fetch(path, {{
        method: 'POST',
        headers: {{'Content-Type': 'application/json', 'X-XPolicy-Operator-Token': token}},
        body: JSON.stringify(payload)
      }});
      const data = await response.json();
      if (!response.ok) {{
        const error = new Error(data.error || 'Request failed');
        error.sessionChanged = response.status === 403 && data.error === 'invalid operator token';
        throw error;
      }}
      return data;
    }}
    async function command(value) {{
      commandFeedback.textContent = `${{value.toUpperCase()}} requested…`;
      try {{
        await post('/api/command', {{command: value}});
        commandFeedback.textContent = `${{value.toUpperCase()}} 已提交；是否执行请查看机器人状态及反馈原因`;
        await refresh();
      }} catch (error) {{
        if (error.sessionChanged) {{
          commandFeedback.textContent = '服务已重启，正在刷新页面；请确认状态后重新点击。';
          window.location.reload();
          return; // Never replay a hardware command across service sessions.
        }}
        commandFeedback.textContent = `${{value.toUpperCase()}} 失败：${{error.message}}`;
        if (value === 'open_grippers') document.getElementById('gripper-feedback').textContent = `张开请求失败：${{error.message}}`;
      }}
    }}
    const commandButtons = Array.from(document.querySelectorAll('button[data-command]'));
    let demoRecordingActive = false;
    commandButtons.forEach(
      button => button.addEventListener('click', () => command(button.dataset.command))
    );
    let deploymentButtons = Array.from(document.querySelectorAll('button[data-deployment]'));
    const deploymentContainer = document.querySelector('.deployment-buttons');
    let deploymentListSignature = '';
    const deploymentStatus = document.getElementById('deployment-status');
    const deploymentCurrent = document.getElementById('deployment-current');
    const executionPresetButtons = Array.from(document.querySelectorAll('button[data-execution-steps]'));
    let activeDeployment = '';
    let deploymentBusy = false;
    let deploymentSubmitting = false;
    function updateDeploymentSwitch(deployment) {{
      if (!deploymentStatus || !deploymentCurrent) return;
      const signature = JSON.stringify(deployment.deployments || []);
      if (signature !== deploymentListSignature) {{
        deploymentListSignature = signature;
        deploymentContainer.replaceChildren();
        (deployment.deployments || []).forEach(option => {{
          const button = document.createElement('button');
          button.className = 'deployment-switch';
          button.dataset.deployment = option.name;
          button.dataset.label = option.label;
          button.textContent = option.label;
          deploymentContainer.appendChild(button);
        }});
        deploymentButtons = Array.from(deploymentContainer.querySelectorAll('button'));
        document.getElementById('deployment-count').textContent = String(deploymentButtons.length);
      }}
      const status = deployment.status || {{phase: 'idle'}};
      const phase = String(status.phase || 'idle');
      const busy = ['queued', 'validating', 'stopping', 'starting', 'rollback_starting'].includes(phase);
      const reconnectButton = document.getElementById('reconnect-hardware');
      reconnectButton.disabled = busy || deploymentSubmitting || !deployment.eligible;
      if (status.operation === 'reconnect') {{
        const messages = {{queued:'重连已排队', validating:'正在检查当前配置', stopping:'正在释放硬件和相机连接', starting:'正在重新连接硬件、相机与模型', ready:'重连完成，当前策略已暂停', failed:'重连未完成'}};
        document.getElementById('reconnect-feedback').textContent = (messages[phase] || phase) + (status.error ? `：${{status.error}}` : '');
      }} else if (!deployment.eligible && deployment.reason) {{
        document.getElementById('reconnect-feedback').textContent = `暂不可重连：${{deployment.reason}}`;
      }} else {{
        document.getElementById('reconnect-feedback').textContent = '重连会重启当前运行时；保留模型、执行步数和读写模式。';
      }}
      if (reconnectError) document.getElementById('reconnect-feedback').textContent = reconnectError;
      deploymentBusy = busy || deploymentSubmitting;
      const message = {{
        idle: '仅在策略暂停、未运行且未录制时可切换。',
        queued: '切换请求已接收，正在校验目标包。',
        validating: '正在校验目标部署包。',
        stopping: '目标包已通过校验，正在停止当前运行时。',
        starting: '正在启动目标部署包并等待就绪确认。',
        ready: '切换已确认生效：运行时、相机和策略均已就绪。',
        rollback_starting: '目标启动未确认，正在恢复切换前的部署包。',
        rolled_back: '目标未能就绪，已确认恢复到切换前的部署包。',
        failed: '切换没有完成；请查看运行时日志后再操作。'
      }}[phase] || `切换状态：${{phase}}`;
      deploymentStatus.textContent = status.error ? `${{message}} 原因：${{status.error}}` : message;
      if (!busy && !deployment.eligible && deployment.reason) deploymentStatus.textContent = '暂不可切换：请先 Hold 暂停，并结束录制或等待当前操作完成。';
      deploymentCurrent.textContent = String(deployment.current || '未登记');
      activeDeployment = deployment.current || '';
      executionPresetButtons.forEach(button => {{
        const steps = Number(button.dataset.executionSteps);
        const isCurrent = steps === Number(deployment.execution_steps);
        button.classList.toggle('current', isCurrent);
        button.setAttribute('aria-pressed', String(isCurrent));
        button.textContent = `${{steps}} 步${{isCurrent ? ' · 当前' : ''}}`;
        button.disabled = deploymentBusy || !deployment.eligible || isCurrent
          || !(deployment.execution_options || []).includes(steps);
      }});
      deploymentButtons.forEach(button => {{
        const isCurrent = button.dataset.deployment === deployment.current;
        button.classList.toggle('current', isCurrent);
        button.setAttribute('aria-current', String(isCurrent));
        button.textContent = (isCurrent ? '当前运行中 · ' : '') + button.dataset.label;
        button.disabled = deploymentBusy || !deployment.eligible || isCurrent;
      }});
    }}
    async function switchDeployment(target, executeSteps = null) {{
      if (!deploymentStatus) return;
      if (deploymentBusy) return;
      deploymentSubmitting = true;
      deploymentBusy = true;
      deploymentStatus.textContent = '正在提交部署切换请求；当前运行时会重启。';
      deploymentButtons.forEach(button => {{ button.disabled = true; }});
      executionPresetButtons.forEach(button => {{ button.disabled = true; }});
      try {{
        const payload = {{deployment: target}};
        if (executeSteps !== null) payload.execute_steps = executeSteps;
        await post('/api/deployments/switch', payload);
        deploymentStatus.textContent = '已提交。正在校验、重启并确认新部署。';
      }} catch (error) {{
        if (error.sessionChanged) {{ window.location.reload(); return; }}
        deploymentStatus.textContent = error.message;
      }} finally {{
        deploymentSubmitting = false;
        await refresh();
      }}
    }}
    deploymentContainer?.addEventListener('click', event => {{
      const button = event.target.closest('button[data-deployment]');
      if (button && !button.disabled) switchDeployment(button.dataset.deployment);
    }});
    executionPresetButtons.forEach(button => button.addEventListener('click', () => {{
      if (!button.disabled) switchDeployment(activeDeployment, Number(button.dataset.executionSteps));
    }}));
    document.getElementById('reconnect-hardware').addEventListener('click', async () => {{
      if (deploymentBusy) return;
      const feedback = document.getElementById('reconnect-feedback');
      deploymentSubmitting = true;
      document.getElementById('reconnect-hardware').disabled = true;
      reconnectError = '';
      feedback.textContent = '正在提交硬件与相机重连请求…';
      try {{
        await post('/api/hardware/reconnect', {{}});
        feedback.textContent = '已提交，Web 重启后会自动重新连接。';
      }} catch (error) {{
        if (error.sessionChanged) {{ window.location.reload(); return; }}
        reconnectError = `重连失败：${{error.message}}`;
        feedback.textContent = reconnectError;
      }} finally {{ deploymentSubmitting = false; }}
    }});
    const intakeForm = document.getElementById('intake-form');
    const intakeRegister = document.getElementById('intake-register');
    const intakeInspect = document.getElementById('intake-inspect');
    const intakeStatus = document.getElementById('intake-status');
    const intakeCommand = document.getElementById('intake-command');
    let intakeBusy = false;
    let intakeRevision = 0;
    function intakePayload() {{
      return {{bundle: document.getElementById('intake-bundle').value.trim(),
        deployment: document.getElementById('intake-name').value.trim(),
        label: document.getElementById('intake-label').value.trim()}};
    }}
    intakeForm?.addEventListener('input', () => {{
      intakeRevision += 1;
      intakeRegister.disabled = true;
      intakeCommand.hidden = true;
      intakeStatus.textContent = '路径或设置已变更，请重新检查。';
    }});
    async function runIntake(action) {{
      if (intakeBusy || deploymentBusy) return;
      intakeBusy = true;
      const revision = intakeRevision;
      intakeInspect.disabled = true;
      intakeRegister.disabled = true;
      intakeCommand.hidden = true;
      intakeStatus.textContent = action === 'inspect' ? '正在检查包与部署配置…' : '正在登记已验证的部署包…';
      try {{
        const result = await post(`/api/deployments/${{action}}`, intakePayload());
        if (revision !== intakeRevision) return;
        intakeStatus.textContent = result.message;
        intakeRegister.disabled = !result.can_register;
        intakeCommand.textContent = (result.commands || []).join('\\n');
        intakeCommand.hidden = !intakeCommand.textContent;
        if (action === 'register') await refresh();
      }} catch (error) {{
        if (error.sessionChanged) {{ window.location.reload(); return; }}
        if (revision === intakeRevision) intakeStatus.textContent = error.message;
      }} finally {{
        intakeBusy = false;
        intakeInspect.disabled = false;
      }}
    }}
    intakeForm?.addEventListener('submit', event => {{ event.preventDefault(); runIntake('inspect'); }});
    intakeRegister?.addEventListener('click', () => runIntake('register'));
    const keys = {{KeyI: 'prepare', KeyS: 'start', KeyF: 'finish', KeyX: 'stop'}};
    window.addEventListener('keydown', event => {{
      if (event.target.closest('input, textarea, select, [contenteditable="true"]')) return;
      const value = keys[event.code];
      if (!value || event.repeat) return;
      if (!document.getElementById('keyboard').checked) return;
      if ((demoRecordingActive || deploymentBusy) && value !== 'stop') return;
      event.preventDefault();
      command(value);
    }});
    const taskSelect = document.getElementById('task');
    taskSelect.addEventListener('change', async () => {{
      try {{ await post('/api/task', {{task: taskSelect.value}}); }}
      catch (error) {{ alert(error.message); refresh(); }}
    }});
    const executionMode = document.getElementById('execution-mode');
    const executionSteps = document.getElementById('execution-steps');
    const executionNote = document.getElementById('execution-note');
    async function updateExecution() {{
      try {{
        await post('/api/execution', {{
          mode: executionMode.value,
          steps: Number(executionSteps.value)
        }});
      }} catch (error) {{
        alert(error.message);
        refresh();
      }}
    }}
    executionMode.addEventListener('change', updateExecution);
    executionSteps.addEventListener('change', updateExecution);
    const rtcMode = document.getElementById('rtc-mode');
    const rtcRequest = document.getElementById('rtc-request');
    const rtcTransition = document.getElementById('rtc-transition');
    const rtcCurve = document.getElementById('rtc-curve');
    const rtcNote = document.getElementById('rtc-note');
    const expertSource = document.getElementById('expert-source');
    const expertLeader = document.getElementById('expert-leader');
    const expertFollower = document.getElementById('expert-follower');
    const expertFeedback = document.getElementById('expert-feedback');
    const expertAge = document.getElementById('expert-age');
    const expertApply = document.getElementById('expert-apply');
    const expertNote = document.getElementById('expert-note');
    const pigeyPanel = document.getElementById('pigey-panel');
    const collectionPanel = document.getElementById('collection-panel');
    const collectionMode = document.getElementById('collection-mode');
    const collectionSave = document.getElementById('collection-save');
    const collectionDiscard = document.getElementById('collection-discard');
    const pigeyObservation = document.getElementById('pigey-observation');
    const pigeyControlLock = document.getElementById('pigey-control-lock');
    const pigeySequencePicker = document.getElementById('pigey-sequence-picker');
    const pigeySequenceSelect = document.getElementById('pigey-sequence-select');
    const pigeySequenceRun = document.getElementById('pigey-sequence-run');
    const pigeySequenceNote = document.getElementById('pigey-sequence-note');
    const humanDemoPanel = document.getElementById('human-demo-panel');
    const demoStart = document.getElementById('demo-start');
    const demoStop = document.getElementById('demo-stop');
    const demoStatus = document.getElementById('demo-status');
    const demoPath = document.getElementById('demo-path');
    const demoOpenFolder = document.getElementById('demo-open-folder');
    const demoFolderFeedback = document.getElementById('demo-folder-feedback');
    let folderOpening = false;
    demoOpenFolder.addEventListener('click', async () => {{
      if (folderOpening) return;
      folderOpening = true;
      demoOpenFolder.disabled = true;
      demoFolderFeedback.textContent = '正在打开录像文件夹…';
      try {{
        const result = await post('/api/demo/open-folder', {{}});
        demoFolderFeedback.textContent = result.folder.message;
        demoPath.textContent = result.folder.directory;
      }} catch (error) {{
        demoFolderFeedback.textContent = `打开失败：${{error.message}}`;
        if (error.sessionChanged) window.location.reload();
      }} finally {{ folderOpening = false; }}
    }});
    const recordingTitle = document.getElementById('recording-title');
    async function demoRecording(action) {{
      try {{ await post(`/api/demo/${{action}}`, {{}}); await refresh(); }}
      catch (error) {{ alert(error.message); await refresh(); }}
    }}
    demoStart.addEventListener('click', () => demoRecording('start'));
    demoStop.addEventListener('click', () => demoRecording('stop'));
    pigeySequenceRun.addEventListener('click', async () => {{
      try {{
        const selected = await post('/api/pigey/sequence', {{preset_id: pigeySequenceSelect.value}});
        pigeySequenceNote.textContent = `${{selected.label}} selected · waiting for Pigey`;
        await refresh();
      }} catch (error) {{
        pigeySequenceNote.textContent = error.message;
        await refresh();
      }}
    }});
    async function updateRtc() {{
      try {{
        await post('/api/rtc', {{
          mode: rtcMode.value,
          request_remaining_steps: Number(rtcRequest.value),
          transition_steps: Number(rtcTransition.value),
          transition_curve: rtcCurve.value
        }});
      }} catch (error) {{
        alert(error.message);
        refresh();
      }}
    }}
    [rtcMode, rtcRequest, rtcTransition, rtcCurve].forEach(
      control => control.addEventListener('change', updateRtc)
    );
    async function updateExpertConfig() {{
      try {{
        await post('/api/expert-config', {{
          source: expertSource.value,
          leader_role: expertLeader.value,
          follower_role: expertFollower.value,
          feedback_role: expertFeedback.value,
          max_age_ms: Number(expertAge.value)
        }});
        expertNote.textContent = 'Applied. The owner will use this mapping on the next takeover.';
        await refresh();
      }} catch (error) {{
        expertNote.textContent = error.message;
        await refresh();
      }}
    }}
    expertApply.addEventListener('click', updateExpertConfig);
    async function refresh() {{
      try {{
        const response = await fetch('/api/state', {{cache: 'no-store'}});
        const serverSession = response.headers.get('X-XPolicy-Operator-Session');
        if (serverSession && serverSession !== operatorSession) {{
          window.location.reload();
          return;
        }}
        const state = await response.json();
        const health = state.robot_status.component_health || {{}};
        const freshRobot = state.robot_status_age_seconds !== null && state.robot_status_age_seconds < 3;
        ['left_arm', 'right_arm', 'left_gripper', 'right_gripper'].forEach(name => {{
          const item = health[name] || {{}};
          const element = document.getElementById(`health-${{name}}`);
          element.textContent = freshRobot ? (item.message || '等待设备反馈') + (Number.isFinite(item.width_m) ? ` · 开度 ${{(item.width_m*1000).toFixed(1)}} mm` : '') : '机器人反馈中断：等待控制进程重新连接';
          element.style.color = freshRobot && item.ok ? '#98d9a8' : '#ffb4b4';
        }});
        document.getElementById('status').textContent = JSON.stringify(state, null, 2);
        if (state.deployment_switch) updateDeploymentSwitch(state.deployment_switch);
        const robotStatus = String(state.robot_status.status || '');
        const lifecycle = robotStatus.startsWith('HOLD_') ? robotStatus
          : state.robot_status.lifecycle_state || robotStatus || 'Connecting';
        document.getElementById('hardware-feedback').textContent =
          robotStatus.startsWith('HOLD_')
            ? state.robot_status.feedback_error || state.robot_status.startup_feedback_error || ''
            : '';
        document.getElementById('lifecycle').textContent = lifecycle.replaceAll('_', ' ');
        const ready = String(state.robot_status.status || '').includes('READY');
        document.getElementById('runtime-dot').classList.toggle('ready', ready);
        const pigey = state.pigey || {{}};
        const pigeyActive = Boolean(pigey.active);
        const pigeyPhase = String(pigey.phase || '').toUpperCase();
        const sequencePresets = Array.isArray(pigey.sequence_presets)
          ? pigey.sequence_presets : [];
        const sequenceReady = pigeyPhase === 'WAITING_FOR_SEQUENCE'
          || (pigeyPhase === 'ROUND_READY' && Boolean(pigey.sequence_selection_enabled));
        const demoCaptureExpected = pigeyPhase === 'WAITING_FOR_HUMAN_DEMO'
          || (pigeyPhase === 'ROUND_READY' && Boolean(pigey.demo_rerecord_enabled));
        const humanDemo = state.human_demo || {{status: 'idle', active: false}};
        const operatorRecording = state.operator_recording || {{status: 'idle', active: false}};
        const pigeyRecording = Boolean(pigey.available);
        const recording = pigeyRecording ? humanDemo : operatorRecording;
        const folder = state.recording_folder || {{}};
        demoOpenFolder.disabled = folderOpening || deploymentBusy || !folder.available;
        demoOpenFolder.title = folder.directory || folder.error || '暂无录像目录';
        demoRecordingActive = pigeyRecording && Boolean(humanDemo.active);
        const controlsLocked = Boolean(state.execution.locked);
        const expert = state.expert || {{}};
        const collection = state.collection || {{mode: 'OFF'}};
        collectionPanel.hidden = collection.mode === 'OFF';
        collectionMode.value = collection.mode;
        document.getElementById('collection-recorder').textContent = collection.recorder_enabled ? 'Armed' : 'Disabled';
        document.getElementById('collection-episode').textContent = collection.episode_active ? 'Recording' : String(collection.lifecycle_state || 'Idle');
        document.getElementById('collection-source').textContent = `${{collection.selected_action_source || 'HOLD'}} · label ${{collection.label_valid ? 'valid' : 'off'}}`;
        document.getElementById('collection-output').textContent = collection.record_dir || 'Not configured';
        taskSelect.disabled = controlsLocked;
        commandButtons.forEach(button => {{
          button.disabled = (demoRecordingActive || deploymentBusy) && button.dataset.command !== 'stop';
        }});
        const gripperControl = state.gripper_control || {{}};
        document.getElementById('open-grippers').disabled = deploymentBusy || !gripperControl.can_open;
        const gripperRequest = gripperControl.last_request || {{}};
        document.getElementById('gripper-feedback').textContent = gripperRequest.accepted === false
          ? `张开请求未完成：${{gripperRequest.reason}}`
          : gripperRequest.accepted === true
            ? '张开指令已下发；按 Prepare 回初始姿态并闭合。'
            : gripperControl.reason || '暂停时张开双夹爪；按 Prepare 回初始姿态并闭合。';
        collectionSave.disabled = !collection.episode_active;
        collectionDiscard.disabled = collection.mode !== 'DATA_COLLECTION' || !collection.episode_active;
        pigeyControlLock.hidden = !pigeyActive;
        pigeySequencePicker.hidden = sequencePresets.length === 0;
        pigeySequenceSelect.disabled = !sequenceReady;
        pigeySequenceRun.disabled = !sequenceReady;
        humanDemoPanel.hidden = sequencePresets.length > 0
          && !demoCaptureExpected && !demoRecordingActive;
        const allCamerasAvailable = cameraNames.every(name => Boolean(state.cameras[name]?.available));
        const pigeyCaptureBlocked = pigeyRecording && state.robot_status.lifecycle_state !== 'idle';
        demoStart.disabled = Boolean(recording.active) || (pigeyActive && !demoCaptureExpected)
          || pigeyCaptureBlocked || (pigeyRecording
            ? !Boolean(state.cameras.head?.available) : !allCamerasAvailable);
        demoStop.disabled = !Boolean(recording.active) && recording.status !== 'limit_reached';
        recordingTitle.textContent = pigeyRecording
          ? 'Human demonstration · head/top camera'
          : 'Operator video · left/head/right UI views';
        demoStop.textContent = demoCaptureExpected
          ? '■ Finish recording & run Pigey'
          : '■ Stop & save';
        const demoDuration = Number(recording.duration_seconds || 0).toFixed(1);
        const sampleLabel = pigeyRecording ? 'head/top frames' : 'three-camera samples';
        demoStatus.textContent = recording.active
          ? `Recording ${{sampleLabel}} · ${{recording.frame_count || 0}} · ${{demoDuration}} s`
          : `${{recording.message || 'Idle · records vision only and never commands the robot.'}} ${{recording.frame_count ? `· ${{recording.frame_count}} samples · ${{demoDuration}} s` : ''}}`;
        demoPath.textContent = recording.pigey_input_path || recording.manifest_path || recording.directory || folder.directory || 'Saved MP4/manifest path will appear here.';
        executionMode.disabled = controlsLocked;
        executionSteps.disabled = controlsLocked || state.execution.mode === 'auto';
        const expertLocked = Boolean(expert.locked);
        expertSource.disabled = expertLocked;
        expertLeader.disabled = expertLocked;
        expertFollower.disabled = expertLocked;
        expertFeedback.disabled = expertLocked;
        expertAge.disabled = expertLocked;
        expertApply.disabled = expertLocked;
        if (!expertLocked && document.activeElement !== expertSource) expertSource.value = expert.source || state.dagger_expert_source || 'qnexo';
        if (!expertLocked && document.activeElement !== expertLeader) expertLeader.value = expert.leader_role || 'left_arm';
        if (!expertLocked && document.activeElement !== expertFollower) expertFollower.value = expert.follower_role || 'right_arm';
        if (!expertLocked && document.activeElement !== expertFeedback) expertFeedback.value = expert.feedback_role || 'right_arm';
        if (!expertLocked && document.activeElement !== expertAge) expertAge.value = String(expert.max_age_ms || 100);
        document.getElementById('collection-expert-source').textContent = expert.source || state.dagger_expert_source || 'qnexo';
        const nativeRtc = state.policy_status.training_time_rtc?.enabled === true;
        const rtcLocked = Boolean(state.rtc.locked) || nativeRtc;
        if (nativeRtc) {{
          executionMode.disabled = true; executionSteps.disabled = true;
          executionMode.value = 'fixed'; executionSteps.value = String(state.execution.default_steps);
          rtcMode.value = 'disabled'; rtcRequest.value = '0'; rtcTransition.value = '0';
        }}
        const rtcEnabled = state.rtc.mode !== 'disabled';
        const rtcBlend = state.rtc.mode === 'blend';
        rtcMode.disabled = rtcLocked;
        rtcRequest.disabled = rtcLocked || !rtcEnabled;
        rtcTransition.disabled = rtcLocked || !rtcBlend;
        rtcCurve.disabled = rtcLocked || !rtcBlend;
        if (document.activeElement !== taskSelect) taskSelect.value = state.task;
        if (!controlsLocked && document.activeElement !== executionMode) executionMode.value = state.execution.mode;
        if (!controlsLocked && document.activeElement !== executionSteps) executionSteps.value = String(state.execution.fixed_steps);
        if (!rtcLocked && document.activeElement !== rtcMode) rtcMode.value = state.rtc.mode;
        if (!rtcLocked && document.activeElement !== rtcRequest) rtcRequest.value = String(state.rtc.request_remaining_steps);
        if (!rtcLocked && document.activeElement !== rtcTransition) rtcTransition.value = String(state.rtc.transition_steps);
        if (!rtcLocked && document.activeElement !== rtcCurve) rtcCurve.value = state.rtc.transition_curve;
        const effective = state.execution.effective_steps;
        executionNote.textContent = state.execution.mode === 'auto'
          ? `Model recommendation · ${{effective || state.execution.default_steps}} / ${{state.execution.action_horizon}} steps`
          : `Fixed prefix · ${{state.execution.fixed_steps}} / ${{state.execution.action_horizon}} steps`;
        const trajectoryScale = Number(state.policy_status.execution_timebase?.trajectory_time_scale ?? 1);
        if (trajectoryScale !== 1) {{
          executionNote.textContent += ` · 轨迹速度 ${{Math.round(trajectoryScale * 100)}}%（手臂与夹爪同步）`;
        }}
        const requestLabel = state.rtc.request_remaining_steps
          ? `request with ${{state.rtc.request_remaining_steps}} left`
          : 'automatic request timing';
        const measuredDelay = Number(state.model_info.inference_delay_steps || 0);
        const alignedSkip = Number(state.model_info.aligned_offset_steps || 0);
        const measuredLabel = measuredDelay
          ? ` · last latency ${{measuredDelay}} steps, skipped ${{alignedSkip}}`
          : '';
        rtcNote.textContent = nativeRtc
          ? `Native training-prefix RTC · commit ${{state.policy_status.training_time_rtc.commit_steps}} · future ${{state.policy_status.training_time_rtc.future_budget_steps}} · prefix ${{state.model_info?.native_prefix_length ?? 'waiting'}} · consumed ${{state.model_info?.native_prefix_consumed === true ? 'yes' : 'not verified'}} · exact ${{state.model_info?.native_prefix_exact === true ? 'yes' : 'not verified'}} · no output blending`
          : state.rtc.mode === 'disabled'
          ? 'Off · finish the current chunk before inferring again'
          : state.rtc.mode === 'align'
            ? `Align · ${{requestLabel}} · skip elapsed steps${{measuredLabel}}`
            : `Blend · ${{requestLabel}} · skip elapsed, blend ${{state.rtc.transition_steps}} · ${{state.rtc.transition_curve}}${{measuredLabel}}`;
        pigeyPanel.hidden = !pigey.available;
        if (pigey.available) {{
          document.getElementById('pigey-phase').textContent = String(pigey.phase || 'UNKNOWN').replaceAll('_', ' ');
          document.getElementById('pigey-dot').classList.toggle('ready', Boolean(pigey.active));
          document.getElementById('pigey-rule').textContent = pigey.rule_index
            ? `${{pigey.rule_index}} / ${{pigey.rule_count || '?'}}`
            : 'Induction';
          const decision = pigey.decision || '—';
          const decisionNode = document.getElementById('pigey-decision');
          decisionNode.textContent = decision;
          decisionNode.className = `pigey-value ${{String(decision).toLowerCase()}}`;
          document.getElementById('pigey-done').textContent = `${{pigey.transition_confirmation || pigey.done_confirmation || 0}} / ${{pigey.transition_confirmations_required || pigey.done_confirmations || 2}}`;
          document.getElementById('pigey-actions').textContent = `${{pigey.actions_for_rule || 0}} / ${{pigey.max_actions_per_rule || '?'}}`;
          document.getElementById('pigey-objects').textContent = String(pigey.completed_objects_for_rule || 0);
          document.getElementById('pigey-prompt').textContent = pigey.policy_prompt || '—';
          const ownerRollout = state.rollout?.owner || {{}};
          const maximum = ownerRollout.active_max_control_steps || ownerRollout.last?.max_control_steps || pigey.max_control_steps || 0;
          const current = ownerRollout.model_steps || ownerRollout.last?.model_steps || 0;
          document.getElementById('pigey-rollout').textContent = maximum ? `${{current}} / ${{maximum}}` : String(current);
          const taskStepLimit = Number(pigey.max_task_control_steps || 0);
          const taskSteps = Number(pigey.task_control_steps || 0);
          document.getElementById('pigey-task-steps').textContent = taskStepLimit
            ? `${{taskSteps}} / ${{taskStepLimit}}`
            : String(taskSteps);
          document.getElementById('pigey-request').textContent = pigey.vlm_request_id
            ? String(pigey.vlm_request_id).slice(0, 12)
            : (pigey.phase === 'VLM_SUBMITTED' ? 'submitted…' : '—');
          const digest = pigey.observation_sha256 || '';
          const visualEvidence = pigey.visual_evidence || {{}};
          const executionContext = pigey.execution_context || {{}};
          const completionGuard = pigey.completion_guard || {{}};
          document.getElementById('pigey-image-state').textContent = pigey.observation_available ? 'Submitted' : 'Waiting';
          document.getElementById('pigey-evidence').textContent = [
            pigey.vlm_model ? `model=${{pigey.vlm_model}}` : null,
            digest ? `image_sha256=${{digest}}` : null,
            pigey.observation_views?.length ? `current_views=${{pigey.observation_views.map(view => view.camera).join(',')}}` : null,
            pigey.attempt_baseline_views?.length ? `baseline_views=${{pigey.attempt_baseline_views.map(view => view.camera).join(',')}}` : null,
            Number.isInteger(visualEvidence.baseline_satisfied_count) ? `baseline_satisfied=${{visualEvidence.baseline_satisfied_count}}` : null,
            Number.isInteger(visualEvidence.current_satisfied_count) ? `current_satisfied=${{visualEvidence.current_satisfied_count}}` : null,
            Number.isInteger(visualEvidence.current_unsatisfied_count) ? `current_unsatisfied=${{visualEvidence.current_unsatisfied_count}}` : null,
            typeof visualEvidence.active_manipulation === 'boolean' ? `active_manipulation=${{visualEvidence.active_manipulation}}` : null,
            typeof visualEvidence.relation_ambiguous === 'boolean' ? `relation_ambiguous=${{visualEvidence.relation_ambiguous}}` : null,
            pigey.transition_candidate ? `candidate=${{pigey.transition_candidate}} ${{pigey.transition_confirmation || 0}}/${{pigey.transition_confirmations_required || 2}}` : null,
            pigey.vlm_request_id ? `request_id=${{pigey.vlm_request_id}}` : null,
            pigey.demo_frames?.length ? `demo_frames=${{pigey.demo_frames.length}}` : null,
            pigey.task_sequence?.length ? `sequence=${{pigey.task_sequence.join(' → ')}}` : null,
            executionContext.phase ? `task_phase=${{executionContext.phase}}` : null,
            executionContext.previous_task_completed ? 'previous_task=confirmed' : null,
            completionGuard.accepted === false ? `completion_guard=${{completionGuard.reason || 'rejected'}}` : null,
            pigey.total_actions_for_rule ? `total_acts=${{pigey.total_actions_for_rule}}` : null,
            pigey.message || null,
          ].filter(Boolean).join(' · ') || 'Authenticated Pigey session is active.';
          if (pigey.observation_available && pigeyObservation.dataset.digest !== digest) {{
            pigeyObservation.dataset.digest = digest;
            pigeyObservation.src = `/api/pigey/observation.jpg?t=${{Date.now()}}`;
          }}
        }}
        cameraNames.forEach(name => {{
          const camera = state.cameras[name] || {{}};
          const available = Boolean(camera.available);
          const label = camera.status === 'stale'
            ? `Stale (${{Number(camera.age_seconds || 0).toFixed(1)}}s)`
            : 'Waiting';
          const details = state.robot_status.camera_details?.[`camera_${{name}}`] || {{}};
          const cameraLabel = document.getElementById(`camera-${{name}}-state`);
          cameraLabel.textContent = details.error ? `${{details.status || '相机异常'}}：${{details.error}}` : available ? 'Live' : `${{label}}：相机画面未到达，请检查 USB 和连接状态`;
          cameraLabel.style.color = available && !details.error ? '#98d9a8' : '#ffb4b4';
        }});
      }} catch (error) {{
        document.getElementById('status').textContent = String(error);
        demoOpenFolder.disabled = true;
        document.getElementById('reconnect-hardware').disabled = true;
        document.getElementById('reconnect-feedback').textContent = 'Web 连接中断或正在重启，自动重连中…';
        ['left', 'head', 'right'].forEach(name => {{
          document.getElementById(`camera-${{name}}-state`).textContent = 'Web 连接中断，画面状态未知';
        }});
        ['left_arm', 'right_arm', 'left_gripper', 'right_gripper'].forEach(name => {{
          const element = document.getElementById(`health-${{name}}`);
          element.textContent = 'Web 连接中断，设备状态未知';
          element.style.color = '#ffb4b4';
        }});
        deploymentButtons.forEach(button => {{ button.disabled = true; }});
        commandButtons.forEach(button => {{ button.disabled = true; }});
        document.getElementById('runtime-dot').classList.remove('ready');
        document.getElementById('lifecycle').textContent = '重新连接中';
        if (deploymentStatus) deploymentStatus.textContent = deploymentBusy
          ? 'Web 与模型重启中，正在自动重连；模型加载最多等待 300 秒。'
          : 'Web 连接中断，正在自动重连。';
      }}
    }}
    const cameraNames = ['left', 'head', 'right'];
    pigeyObservation.addEventListener('load', () => pigeyObservation.classList.remove('unavailable'));
    pigeyObservation.addEventListener('error', () => pigeyObservation.classList.add('unavailable'));
    cameraNames.forEach(name => {{
      const frame = document.getElementById(`camera-${{name}}`);
      frame.addEventListener('load', () => frame.classList.remove('unavailable'));
      frame.addEventListener('error', () => frame.classList.add('unavailable'));
    }});
    function refreshCameras() {{
      const stamp = Date.now();
      cameraNames.forEach(name => {{
        document.getElementById(`camera-${{name}}`).src = `/api/camera/${{name}}.jpg?t=${{stamp}}`;
      }});
    }}
    setInterval(refreshCameras, 150); refreshCameras();
    setInterval(refresh, 500); refresh();
  </script>
</body>
</html>""".encode()


def _handler_type(
    state: PiperOperatorState,
    token: str,
    deployment_switch: DeploymentSwitcher | None = None,
) -> type[BaseHTTPRequestHandler]:
    page = operator_page(state, token, deployment_switch)
    session_id = hashlib.sha256(token.encode()).hexdigest()
    request_lock = threading.Lock()

    def runtime_snapshot() -> dict[str, Any]:
        snapshot = state.snapshot()
        if deployment_switch is not None:
            snapshot["deployment_switch"] = deployment_switch.snapshot(snapshot)
        return snapshot

    class Handler(BaseHTTPRequestHandler):
        server_version = "XPolicyLabPiperOperator/1"

        def log_message(self, format: str, *args: Any) -> None:
            if not self.path.startswith(
                ("/api/state", "/api/camera/", "/api/pigey/observation.jpg")
            ):
                print(f"PIPER_OPERATOR_UI {format % args}", flush=True)

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            try:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-XPolicy-Operator-Session", session_id)
                self.send_header("X-Frame-Options", "DENY")
                self.send_header("Content-Security-Policy", "frame-ancestors 'none'")
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                # Browsers routinely cancel an old preview request when the
                # next camera frame replaces it. That is not a UI failure.
                return

        def _json(self, status: int, value: Any) -> None:
            self._send(
                status,
                json.dumps(value, sort_keys=True).encode(),
                "application/json; charset=utf-8",
            )

        def do_GET(self) -> None:
            path = self.path.partition("?")[0]
            if path == "/":
                self._send(HTTPStatus.OK, page, "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(HTTPStatus.OK, runtime_snapshot())
            elif path == "/api/deployments":
                if deployment_switch is None:
                    self._json(
                        HTTPStatus.NOT_FOUND,
                        {"ok": False, "error": "deployment switching is unavailable"},
                    )
                else:
                    self._json(HTTPStatus.OK, deployment_switch.snapshot(runtime_snapshot()))
            elif path.startswith("/api/camera/") and path.endswith(".jpg"):
                camera_name = path.removeprefix("/api/camera/").removesuffix(".jpg")
                frame = state.camera_frame(camera_name)
                if frame is None:
                    self._json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"ok": False, "error": "camera frame not ready"},
                    )
                    return
                self._send(HTTPStatus.OK, frame[0], "image/jpeg")
            elif path == "/api/pigey/observation.jpg":
                frame = state.pigey_observation_frame()
                if frame is None:
                    self._json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"ok": False, "error": "Pigey observation not ready"},
                    )
                    return
                self._send(HTTPStatus.OK, frame[0], "image/jpeg")
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:
            with request_lock:
                self._do_POST()

        def _do_POST(self) -> None:
            path = self.path.partition("?")[0]
            if path not in {
                "/api/command",
                "/api/task",
                "/api/execution",
                "/api/rtc",
                "/api/expert-config",
                "/api/rollout",
                "/api/pigey/transition-pose",
                "/api/pigey/status",
                "/api/pigey/sequence",
                "/api/demo/start",
                "/api/demo/stop",
                "/api/demo/open-folder",
                "/api/deployments/switch",
                "/api/hardware/reconnect",
                "/api/deployments/inspect",
                "/api/deployments/register",
            }:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            if not secrets.compare_digest(
                self.headers.get("X-XPolicy-Operator-Token", ""), token
            ):
                self._json(
                    HTTPStatus.FORBIDDEN,
                    {"ok": False, "error": "invalid operator token"},
                )
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                maximum_length = (
                    MAX_PIGEY_STATUS_BYTES
                    if path == "/api/pigey/status"
                    else 4096
                )
                if not 0 < length <= maximum_length:
                    raise ValueError("invalid request size")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError("request must be a JSON object")
                if deployment_switch is not None and deployment_switch.busy():
                    if not (path == "/api/command" and payload.get("command") == "stop"):
                        raise DeploymentSwitchError("部署切换中，请等待 Web 重新连接后操作")
                source = payload.get("source", "keyboard_ui")
                response: dict[str, Any] = {"ok": True}
                if path == "/api/command":
                    command = str(payload.get("command", ""))
                    state.enqueue(command, source=source)
                    response["command"] = command.strip().lower()
                    print(
                        "PIPER_OPERATOR_UI_REQUEST "
                        f"{response['command']} source={source}",
                        flush=True,
                    )
                elif path == "/api/task":
                    state.select_task(str(payload.get("task", "")), source=source)
                elif path == "/api/execution":
                    state.select_execution(
                        payload.get("mode", ""), payload.get("steps")
                    )
                elif path == "/api/rtc":
                    state.select_rtc(
                        payload.get("mode", ""),
                        payload.get("request_remaining_steps", 0),
                        payload.get("transition_steps", 0),
                        payload.get("transition_curve", "smoothstep"),
                    )
                elif path == "/api/expert-config":
                    state.select_expert_config(
                        payload.get("source", "qnexo"),
                        payload.get("leader_role", "left_arm"),
                        payload.get("follower_role", "right_arm"),
                        payload.get("feedback_role", "right_arm"),
                        payload.get("max_age_ms", 100.0),
                    )
                    response["expert"] = state.snapshot()["expert"]
                elif path == "/api/pigey/status":
                    if source != "pigey":
                        raise ValueError("Pigey status requires source=pigey")
                    state.update_pigey_status(payload)
                elif path == "/api/pigey/sequence":
                    response.update(
                        state.select_pigey_sequence(
                            payload.get("preset_id", ""), source=source
                        )
                    )
                elif path == "/api/pigey/transition-pose":
                    state.select_transition_pose(
                        payload.get("pose_id"),
                        payload.get("task_prompt"),
                        payload.get("library_sha256"),
                        payload.get("reason", "task_transition"),
                        payload.get("request_id"),
                        source=source,
                    )
                elif path == "/api/demo/start":
                    response["recording"] = state.start_ui_recording()
                elif path == "/api/demo/stop":
                    response["recording"] = state.stop_ui_recording()
                elif path == "/api/demo/open-folder":
                    response["folder"] = state.open_ui_recording_folder()
                elif path in {"/api/deployments/inspect", "/api/deployments/register"}:
                    if deployment_switch is None:
                        raise DeploymentSwitchError("当前部署未启用个人包接入")
                    bundle = str(payload.get("bundle", ""))
                    name = str(payload.get("deployment", ""))
                    if path.endswith("/inspect"):
                        response.update(web_deployment_intake.inspect_bundle(bundle, name))
                    else:
                        if not deployment_switch._is_eligible(runtime_snapshot()):
                            raise DeploymentSwitchError("请先暂停策略并结束录制，再登记新部署包")
                        response.update(web_deployment_intake.register(bundle, name, str(payload.get("label", ""))))
                elif path == "/api/hardware/reconnect":
                    if deployment_switch is None:
                        raise DeploymentSwitchError("当前部署不支持硬件重连")
                    response.update(deployment_switch.request(
                        deployment_switch.deployment, runtime_snapshot(), reconnect=True,
                    ))
                elif path == "/api/deployments/switch":
                    if deployment_switch is None:
                        raise DeploymentSwitchError(
                            "deployment switching is unavailable for this package"
                        )
                    response.update(
                        deployment_switch.request(
                            str(payload.get("deployment", "")), runtime_snapshot(),
                            execute_steps=payload.get("execute_steps")
                        )
                    )
                else:
                    state.select_rollout(
                        payload.get("max_control_steps"), source=source
                    )
            except (ValueError, OSError) as exc:
                self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                return
            self._json(HTTPStatus.OK, response)

    return Handler


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _named_robot_state(value: Any) -> dict[str, float]:
    named = {
        str(item["name"].as_py()): float(item["value"].as_py()) for item in value
    }
    if set(named) != set(ACTION_NAMES):
        raise ValueError("Piper UI robot state does not match the 14D schema")
    return {name: named[name] for name in ACTION_NAMES}


def encode_camera_preview(
    value: Any,
    metadata: dict[str, Any],
    *,
    max_width: int,
    jpeg_quality: int,
) -> bytes:
    import cv2
    import numpy as np

    width = int(metadata["width"])
    height = int(metadata["height"])
    if width < 1 or height < 1 or max_width < 1:
        raise ValueError("Piper UI camera dimensions must be positive")
    if not 1 <= jpeg_quality <= 95:
        raise ValueError("Piper UI camera JPEG quality must be in [1, 95]")
    frame = np.asarray(
        value.to_numpy(zero_copy_only=False), dtype=np.uint8
    ).reshape(height, width, 3)
    encoding = str(metadata.get("encoding", "")).lower()
    if encoding == "rgb8":
        frame = frame[..., ::-1]
    elif encoding != "bgr8":
        raise ValueError(f"unsupported Piper UI camera encoding: {encoding!r}")
    if width > max_width:
        frame = cv2.resize(
            frame,
            (max_width, round(height * max_width / width)),
            interpolation=cv2.INTER_AREA,
        )
    ok, encoded = cv2.imencode(
        ".jpg", frame, (cv2.IMWRITE_JPEG_QUALITY, jpeg_quality)
    )
    if not ok:
        raise ValueError("Piper UI camera JPEG encoding failed")
    return encoded.tobytes()


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("Piper operator UI requires dora-rs and pyarrow") from exc

    host = os.getenv("PIPER_OPERATOR_UI_HOST", "127.0.0.1")
    if host not in {"127.0.0.1", "localhost", "::1"}:
        raise SystemExit("Piper operator UI only binds to loopback")
    port = int(os.getenv("PIPER_OPERATOR_UI_PORT", "8091"))
    if not 1 <= port <= 65535:
        raise ValueError("PIPER_OPERATOR_UI_PORT must be in [1, 65535]")
    camera_fps = float(os.getenv("PIPER_OPERATOR_UI_CAMERA_FPS", "10"))
    camera_max_width = int(os.getenv("PIPER_OPERATOR_UI_CAMERA_MAX_WIDTH", "960"))
    camera_jpeg_quality = int(os.getenv("PIPER_OPERATOR_UI_CAMERA_JPEG_QUALITY", "76"))
    if not 0 < camera_fps <= 30:
        raise ValueError("PIPER_OPERATOR_UI_CAMERA_FPS must be in (0, 30]")
    if camera_max_width < 1 or not 1 <= camera_jpeg_quality <= 95:
        raise ValueError("Piper operator UI camera width/quality is invalid")
    tasks = load_piper_task_catalog(os.getenv("PIPER_TASK_CATALOG_PATH"))
    sequence_presets = load_pigey_sequence_presets(
        os.getenv("PIPER_PIGEY_SEQUENCE_PRESETS_JSON"), tasks
    )
    rtc_config = RealTimeChunkingConfig.from_environ(os.environ)
    leader_follower_defaults = {
        "leader_role": os.getenv("PIPER_LF_LEADER_ROLE", "left_arm"),
        "follower_role": os.getenv("PIPER_LF_FOLLOWER_ROLE", "right_arm"),
        "feedback_role": os.getenv("PIPER_LF_FEEDBACK_ROLE", "right_arm"),
        "native_linkage": False,
    }
    state = PiperOperatorState(
        task=os.getenv("TASK", ""),
        write_enabled=os.getenv("ENABLE_WRITE", "0") == "1",
        control_source=os.getenv("PIPER_CONTROL_SOURCE", "both"),
        expert_source=os.getenv("PIPER_DAGGER_EXPERT_SOURCE", "qnexo"),
        leader_follower=leader_follower_defaults,
        expert_max_age_ms=float(os.getenv("PIPER_LF_MAX_AGE_MS", "100")),
        tasks=tasks,
        action_horizon=int(os.getenv("PIPER_POLICY_ACTION_HORIZON", "10")),
        default_execute_chunk=int(
            os.getenv("POLICY_EXECUTE_CHUNK_SIZE", "10")
        ),
        execution_mode=os.getenv("POLICY_EXECUTE_CHUNK_MODE", "auto"),
        rtc_config=rtc_config,
        policy_label=os.getenv("PIPER_POLICY_LABEL", "Pi0.5 policy"),
        deployment_id=os.getenv("PIPER_DEPLOYMENT", ""),
        collection_mode=os.getenv("RECAP_COLLECTION_MODE", "OFF"),
        recorder_enabled=os.getenv("ENABLE_DAGGER_RECORDING", "0") == "1",
        record_dir=os.getenv("DAGGER_RECORD_DIR", ""),
        pigey_expected=os.getenv("PIPER_PIGEY_EXPECTED", "0").strip().lower()
        in {"1", "true", "yes", "on"},
        pigey_sequence_presets=sequence_presets,
        human_demo_required=os.getenv(
            "PIPER_PIGEY_RECORD_DEMO", "0"
        ).strip().lower()
        in {"1", "true", "yes", "on"},
        human_demo_root=os.getenv(
            "PIPER_OPERATOR_DEMO_ROOT", str(DEFAULT_HUMAN_DEMO_ROOT)
        ),
        human_demo_fps=float(
            os.getenv("PIPER_OPERATOR_DEMO_FPS", str(DEFAULT_HUMAN_DEMO_FPS))
        ),
        human_demo_max_seconds=float(
            os.getenv(
                "PIPER_OPERATOR_DEMO_MAX_SECONDS",
                str(DEFAULT_HUMAN_DEMO_MAX_SECONDS),
            )
        ),
        operator_video_root=os.getenv(
            "PIPER_OPERATOR_VIDEO_ROOT", str(DEFAULT_OPERATOR_VIDEO_ROOT)
        ),
        operator_video_fps=float(
            os.getenv("PIPER_OPERATOR_VIDEO_FPS", str(camera_fps))
        ),
        operator_video_max_seconds=float(
            os.getenv(
                "PIPER_OPERATOR_VIDEO_MAX_SECONDS",
                str(DEFAULT_HUMAN_DEMO_MAX_SECONDS),
            )
        ),
        camera_max_age_seconds=float(
            os.getenv("PIPER_OPERATOR_UI_CAMERA_MAX_AGE_SECONDS", "2.0")
        ),
    )
    token = operator_ui_token()
    deployment_switch = DeploymentSwitcher.from_environment()
    server = ThreadingHTTPServer((host, port), _handler_type(state, token, deployment_switch))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    ui_url = f"http://{host}:{port}"
    print(f"PIPER_OPERATOR_UI_READY {ui_url}", flush=True)
    if os.getenv("PIPER_OPERATOR_UI_AUTO_OPEN", "1").strip().lower() not in {
        "0",
        "false",
        "no",
        "off",
    }:
        threading.Thread(
            target=open_operator_page,
            args=(ui_url,),
            daemon=True,
        ).start()

    node = Node(os.getenv("DORA_NODE_ID", "piper_operator_ui"))
    last_status_at = 0.0
    last_camera_at = dict.fromkeys(CAMERA_NAMES, 0.0)
    camera_period = 1.0 / camera_fps
    try:
        for event in node:
            if event is None or event.get("type") == "STOP":
                break
            if event.get("type") != "INPUT":
                continue
            event_id = str(event["id"])
            if event_id == "robot_status":
                state.update_robot_status(_scalar(event["value"]))
                continue
            if event_id == "policy_status":
                state.update_policy_status(_scalar(event["value"]))
                continue
            if event_id == "model_info":
                state.update_model_info(_scalar(event["value"]))
                continue
            if event_id == "state_robot":
                state.update_robot_state(_named_robot_state(event["value"]), dict(event.get("metadata", {})))
                continue
            if event_id.startswith("image_"):
                camera_name = event_id.removeprefix("image_")
                now = time.monotonic()
                if camera_name not in last_camera_at:
                    continue
                if now - last_camera_at[camera_name] < camera_period:
                    continue
                last_camera_at[camera_name] = now
                metadata = dict(event.get("metadata", {}))
                try:
                    jpeg = encode_camera_preview(
                        event["value"],
                        metadata,
                        max_width=camera_max_width,
                        jpeg_quality=camera_jpeg_quality,
                    )
                    state.update_camera(camera_name, jpeg, metadata)
                except (KeyError, TypeError, ValueError) as exc:
                    print(
                        f"PIPER_OPERATOR_CAMERA_ERROR {camera_name} {exc}",
                        flush=True,
                    )
                continue
            if event_id != "tick":
                continue
            for task, timestamp_ns, operator_source in state.drain_task_updates():
                now_ns = time.time_ns()
                node.send_output(
                    "task",
                    pa.array([task]),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": timestamp_ns,
                        "target_execution_timestamp_ns": now_ns,
                        "operator_source": operator_source,
                    },
                )
                print(f"PIPER_OPERATOR_TASK {task}", flush=True)
            for configuration, timestamp_ns in state.drain_execution_updates():
                now_ns = time.time_ns()
                node.send_output(
                    "execution_config",
                    pa.array([json.dumps(configuration, sort_keys=True)]),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": timestamp_ns,
                        "target_execution_timestamp_ns": now_ns,
                        "operator_source": "keyboard_ui",
                    },
                )
                print(
                    "PIPER_OPERATOR_EXECUTION "
                    + json.dumps(configuration, sort_keys=True),
                    flush=True,
                )
            for configuration, timestamp_ns in state.drain_rtc_updates():
                now_ns = time.time_ns()
                node.send_output(
                    "rtc_config",
                    pa.array([json.dumps(configuration, sort_keys=True)]),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": timestamp_ns,
                        "target_execution_timestamp_ns": now_ns,
                        "operator_source": "keyboard_ui",
                    },
                )
                print(
                    "PIPER_OPERATOR_RTC "
                    + json.dumps(configuration, sort_keys=True),
                    flush=True,
                )
            for configuration, timestamp_ns in state.drain_expert_config_updates():
                now_ns = time.time_ns()
                node.send_output(
                    "expert_config",
                    pa.array([json.dumps(configuration, sort_keys=True)]),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": timestamp_ns,
                        "target_execution_timestamp_ns": now_ns,
                        "operator_source": "keyboard_ui",
                    },
                )
                print(
                    "PIPER_OPERATOR_EXPERT_CONFIG "
                    + json.dumps(configuration, sort_keys=True),
                    flush=True,
                )
            for configuration, timestamp_ns, operator_source in (
                state.drain_rollout_updates()
            ):
                now_ns = time.time_ns()
                node.send_output(
                    "rollout_config",
                    pa.array([json.dumps(configuration, sort_keys=True)]),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": timestamp_ns,
                        "target_execution_timestamp_ns": now_ns,
                        "operator_source": operator_source,
                    },
                )
                print(
                    "PIPER_OPERATOR_ROLLOUT "
                    + json.dumps(configuration, sort_keys=True),
                    flush=True,
                )
            for configuration, timestamp_ns, operator_source in (
                state.drain_transition_updates()
            ):
                now_ns = time.time_ns()
                node.send_output(
                    "transition_command",
                    pa.array([json.dumps(configuration, sort_keys=True)]),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": timestamp_ns,
                        "target_execution_timestamp_ns": now_ns,
                        "operator_source": operator_source,
                    },
                )
                print(
                    "PIPER_OPERATOR_TRANSITION_POSE "
                    + json.dumps(configuration, sort_keys=True),
                    flush=True,
                )
            for command, timestamp_ns, operator_source in state.drain():
                now_ns = time.time_ns()
                node.send_output(
                    "button_command",
                    pa.array([command]),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": timestamp_ns,
                        "target_execution_timestamp_ns": now_ns,
                        "operator_source": operator_source,
                    },
                )
                print(
                    f"PIPER_OPERATOR_COMMAND {command} source={operator_source}",
                    flush=True,
                )
            now = time.monotonic()
            if now - last_status_at >= 1.0:
                node.send_output(
                    "status",
                    pa.array(
                        [
                            json.dumps(
                                {"status": "READY", "url": ui_url},
                                sort_keys=True,
                            )
                        ]
                    ),
                    {"timestamp_ns": time.time_ns()},
                )
                last_status_at = now
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
