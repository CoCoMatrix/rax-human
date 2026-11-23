from __future__ import annotations

from dataclasses import dataclass, field
from collections.abc import Mapping
import json
import os
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid
from typing import Any

import numpy as np

from XPolicyLab.real_world.contracts import ObservationFrame
from XPolicyLab.real_world.inference import (
    ActionChunkPolicyRuntime,
    ActionChunkRuntime,
    AsyncPolicyInference,
    NoActionResult,
)
from XPolicyLab.real_world.rtc import (
    RealTimeChunkingConfig,
    RtcMode,
)

from .mapping import PIPER_X_DUAL_ACTION_SCHEMA, PIPER_X_DUAL_STATE_SCHEMA
from .operator_ui_node import DESKTOP_CLEANUP_TASKS, load_piper_task_catalog
from .review_release import release_step_limit, validate_review_runtime


CAMERA_NAMES = ("camera_head", "camera_left", "camera_right")
DEFAULT_INSTRUCTION = (
    "Stack the second cube on top of the first cube, then place the cone on top "
    "of the second cube."
)


def no_action_enabled(environ: Mapping[str, str] = os.environ) -> bool:
    """Cloud receipts are opt-in and require an explicit no-write owner."""
    return (
        environ.get("XPOLICY_ALLOW_NO_ACTION") == "1"
        and environ.get("ENABLE_WRITE") == "0"
    )


def no_action_status(result: NoActionResult, count: int) -> dict[str, Any]:
    """Keep the existing paused -> HOLD status contract for transport receipts."""
    return {
        "reason": "cloud_no_action",
        "no_action_received_count": count,
        "request_id": result.request_id,
        "transport_receipt": dict(result.transport_receipt),
        "inference_executed": result.inference_executed,
        "action_steps": 0,
    }


def policy_runtime_parameters(
    environ: Mapping[str, str] = os.environ,
) -> tuple[float, int, bool, str]:
    """Read the policy cadence independently from the robot owner cadence."""
    frequency_hz = float(environ.get("POLICY_ACTION_FREQUENCY_HZ", "30"))
    execute_chunk_size = int(environ.get("POLICY_EXECUTE_CHUNK_SIZE", "15"))
    execute_chunk_mode = environ.get("POLICY_EXECUTE_CHUNK_MODE", "auto").strip().lower()
    prefetch = environ.get("POLICY_PREFETCH_ENABLED", "1").strip().lower()
    if not np.isfinite(frequency_hz) or frequency_hz <= 0:
        raise ValueError("POLICY_ACTION_FREQUENCY_HZ must be positive")
    if execute_chunk_size < 1:
        raise ValueError("POLICY_EXECUTE_CHUNK_SIZE must be positive")
    if execute_chunk_mode not in {"auto", "fixed"}:
        raise ValueError("POLICY_EXECUTE_CHUNK_MODE must be auto or fixed")
    if prefetch not in {"0", "1", "false", "true", "no", "yes", "off", "on"}:
        raise ValueError("POLICY_PREFETCH_ENABLED must be a boolean")
    return (
        frequency_hz,
        execute_chunk_size,
        prefetch in {"1", "true", "yes", "on"},
        execute_chunk_mode,
    )


def rtc_config_update(
    current: RealTimeChunkingConfig,
    value: Any,
) -> RealTimeChunkingConfig:
    """Apply the operator-visible RTC fields without losing advanced defaults."""
    if not isinstance(value, dict):
        raise ValueError("RTC configuration must be a JSON object")
    allowed = {
        "mode",
        "request_remaining_steps",
        "preserve_steps",
        "transition_steps",
        "transition_curve",
    }
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(f"unsupported RTC configuration fields: {sorted(unknown)}")
    return RealTimeChunkingConfig(
        mode=value.get("mode", current.mode.value),
        latency_quantile=current.latency_quantile,
        latency_margin_steps=current.latency_margin_steps,
        latency_window_size=current.latency_window_size,
        request_remaining_steps=value.get(
            "request_remaining_steps", current.request_remaining_steps
        ),
        preserve_steps=value.get("preserve_steps", current.preserve_steps),
        transition_steps=value.get("transition_steps", current.transition_steps),
        transition_curve=value.get(
            "transition_curve", current.transition_curve.value
        ),
        new_weight_start=current.new_weight_start,
        new_weight_end=current.new_weight_end,
    )


def policy_trajectory_time_scale(environ: Mapping[str, str] = os.environ) -> float:
    from XPolicyLab.real_world.action_timebase import ActionTimebase
    scale = ActionTimebase(float(environ.get("ACTION_TRAJECTORY_TIME_SCALE", "1"))).scale
    incompatible = ("PIPER_TRAINING_RTC_ENABLED", "XPOLICY_ALLOW_NO_ACTION",
                    "PIPER_BOUNDED_REVIEW_RELEASE", "PIPER_CONTINUOUS_REVIEW",
                    "PIPER_GPT_TRAJECTORY_RELEASE")
    if scale != 1 and any(environ.get(name) == "1" for name in incompatible):
        raise ValueError("Retiming requires the standard absolute-position policy path")
    return scale


def rtc_status(config: RealTimeChunkingConfig) -> dict[str, Any]:
    return {
        "mode": config.mode.value,
        "latency_quantile": config.latency_quantile,
        "latency_margin_steps": config.latency_margin_steps,
        "latency_window_size": config.latency_window_size,
        "request_remaining_steps": config.request_remaining_steps,
        "preserve_steps": config.preserve_steps,
        "transition_steps": config.transition_steps,
        "transition_curve": config.transition_curve.value,
        "new_weight_start": config.new_weight_start,
        "new_weight_end": config.new_weight_end,
    }


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _named_vector(value: Any) -> np.ndarray:
    named = {
        str(item["name"].as_py()): float(item["value"].as_py())
        for item in value
    }
    if set(named) != set(PIPER_X_DUAL_STATE_SCHEMA.fields):
        raise ValueError("Piper policy state does not match its declared schema")
    return np.asarray(
        [named[name] for name in PIPER_X_DUAL_STATE_SCHEMA.fields],
        dtype=np.float32,
    )


def _named_vector_array(pa: Any, values: Any) -> Any:
    return pa.StructArray.from_arrays(
        [
            pa.array(PIPER_X_DUAL_ACTION_SCHEMA.fields),
            pa.array(values, type=pa.float32()),
        ],
        names=["name", "value"],
    )


def _dora_metadata(value: Mapping[str, Any]) -> dict[str, Any]:
    """Drop JSON-null diagnostics that Dora cannot encode as metadata."""

    return {str(key): item for key, item in value.items() if item is not None}


def _image(value: Any, metadata: dict[str, Any]) -> np.ndarray:
    width = int(metadata["width"])
    height = int(metadata["height"])
    image = np.asarray(
        value.to_numpy(zero_copy_only=False), dtype=np.uint8
    ).reshape(height, width, 3)
    encoding = str(metadata.get("encoding", ""))
    if encoding == "bgr8":
        image = image[..., ::-1].copy()
    elif encoding != "rgb8":
        raise ValueError(f"unsupported Piper camera encoding: {encoding!r}")
    return image


@dataclass
class PiperPolicyObservationAdapter:
    instruction: str
    robot_id: str
    state: np.ndarray | None = None
    images: dict[str, np.ndarray] = field(default_factory=dict)
    timestamp_ns: int = 0
    sequence: int = 0
    camera_frames: dict[str, dict[str, Any]] = field(default_factory=dict)
    session_id: str = field(default_factory=lambda: uuid.uuid4().hex)

    def update_state(self, value: Any, metadata: dict[str, Any]) -> None:
        self.state = _named_vector(value)
        self.timestamp_ns = int(
            metadata.get("source_timestamp_ns")
            or metadata.get("timestamp_ns")
            or time.time_ns()
        )
        self.sequence = int(metadata.get("sequence", self.sequence + 1))

    def update_image(
        self, camera_name: str, value: Any, metadata: dict[str, Any]
    ) -> None:
        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"unsupported Piper camera: {camera_name}")
        self.images[camera_name] = _image(value, metadata)
        self.camera_frames[camera_name] = {
            key: metadata[key] for key in ('source_timestamp_ns', 'timestamp_ns',
                'device_time_ns', 'device_frame_index', 'timestamp_domain', 'camera_serial')
            if key in metadata
        }

    def build(self, queue: Any) -> ObservationFrame | None:
        if self.state is None or any(name not in self.images for name in CAMERA_NAMES):
            return None
        queued = np.stack(tuple(queue.actions)) if queue.actions else None
        return ObservationFrame(
            state_schema=PIPER_X_DUAL_STATE_SCHEMA,
            state=self.state,
            images={name: self.images[name] for name in CAMERA_NAMES},
            instruction=self.instruction,
            timestamp_ns=self.timestamp_ns,
            sequence=self.sequence,
            robot_type="piper_x_dual",
            robot_id=self.robot_id,
            metadata={
                "camera_frames": {name: dict(value) for name, value in self.camera_frames.items()},
                "policy_session_id": self.session_id,
                "owner_generation": queue.generation,
                "inference_context": {
                    "queued_actions": queued,
                    "execution_step": queue.execution_step,
                }
            },
        )


class ReadOnlyObservationSnapshots:
    """Bounded, lossless reader of existing policy observations; never a writer.

    Files are for external shadow consumers. Enabling this does not enable
    inference or change HOLD, generations, action queues, or hardware ownership.
    """

    def __init__(self, directory: str, *, interval_s: float = 0.5, keep: int = 8):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.interval_s = interval_s
        self.keep = keep
        self._next_at = 0.0
        self._pending: queue.Queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, daemon=True,
                                        name="piper-read-only-snapshots")
        self._worker.start()

    @classmethod
    def from_environment(cls):
        directory = os.getenv("PIPER_OBSERVATION_SNAPSHOT_DIR", "").strip()
        return cls(directory) if directory else None

    def offer(self, frame: ObservationFrame | None) -> None:
        if frame is None or self._stop.is_set():
            return
        now = time.monotonic()
        if now < self._next_at:
            return
        self._next_at = now + self.interval_s
        try:
            # ObservationFrame owns immutable arrays; no encoding/disk I/O on
            # the Dora thread and no dependency on scheduler.enabled.
            self._pending.put_nowait(frame)
        except queue.Full:
            pass

    @staticmethod
    def _plain(value):
        if isinstance(value, Mapping):
            return {str(key): ReadOnlyObservationSnapshots._plain(item)
                    for key, item in value.items()}
        if isinstance(value, (tuple, list)):
            return [ReadOnlyObservationSnapshots._plain(item) for item in value]
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        return value

    def _write(self, frame: ObservationFrame) -> None:
        name = f"observation-{frame.timestamp_ns:020d}-{frame.sequence:012d}-{uuid.uuid4().hex[:8]}"
        folder = self.directory / name
        temporary = self.directory / ("." + name + ".tmp")
        temporary.mkdir()
        record = {
            "schema_version": "junqi.piper.read_only_observation_snapshot.v1",
            "source": "xpolicy.piper_policy_observation_adapter",
            "read_only_snapshot": True,
            "hardware_commands_sent_by_exporter": 0,
            "directory": name, "npz": "observation.npz",
            "timestamp_ns": frame.timestamp_ns, "sequence": frame.sequence,
            "exported_timestamp_ns": time.time_ns(),
            "instruction": frame.instruction,
            "robot_type": frame.robot_type, "robot_id": frame.robot_id,
            "state_schema": frame.state_schema.to_wire(),
            "state14": frame.state.tolist(),
            "metadata": self._plain(frame.metadata),
        }
        try:
            np.savez(temporary / "observation.npz", state14=frame.state,
                     **{name: frame.images[name] for name in CAMERA_NAMES})
            data = json.dumps(record, ensure_ascii=False, indent=2) + "\n"
            (temporary / "observation.json").write_text(data)
            temporary.rename(folder)
            pointer = self.directory / (".latest-" + uuid.uuid4().hex + ".tmp")
            pointer.write_text(data)
            pointer.replace(self.directory / "latest.json")
            # Keep the pointer target even if a paused/stale source repeats its
            # timestamp and sequence (the UUID is deliberately unordered).
            captures = [folder] + sorted(
                (item for item in self.directory.glob("observation-*") if item != folder),
                key=lambda item: item.stat().st_mtime_ns, reverse=True)
            for old in captures[self.keep:]:
                if old.is_dir():
                    shutil.rmtree(old)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)

    def _run(self):
        while not self._stop.is_set():
            try:
                frame = self._pending.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._write(frame)
            except Exception as exc:
                # Export failures must not alter the formal control lifecycle.
                print(f"PIPER_READ_ONLY_SNAPSHOT_ERROR {type(exc).__name__}: {exc}",
                      flush=True)

    def close(self):
        self._stop.set()
        self._worker.join(timeout=1.0)


def main() -> None:
    collection_mode = os.getenv("RECAP_COLLECTION_MODE", "MIXED").strip().upper()
    if collection_mode not in {"OFF", "DATA_COLLECTION", "MIXED"}:
        raise ValueError(
            "RECAP_COLLECTION_MODE must be OFF, DATA_COLLECTION, or MIXED"
        )
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Piper policy node requires dora-rs and pyarrow"
        ) from exc

    policy_url = os.getenv("POLICY_URL", "ws://127.0.0.1:19003")
    robot_id = os.getenv("ROBOT_ID", "piper_x_dual_01")
    instruction = os.getenv("TASK", DEFAULT_INSTRUCTION)
    allowed_tasks = load_piper_task_catalog(
        os.getenv("PIPER_TASK_CATALOG_PATH"),
        fallback=(DEFAULT_INSTRUCTION, *DESKTOP_CLEANUP_TASKS),
    )
    if instruction not in allowed_tasks:
        raise ValueError("Piper task is not supported by the selected checkpoint")
    node = Node(os.getenv("DORA_NODE_ID", "piper_x_dual_policy"))
    if collection_mode == "DATA_COLLECTION":
        payload = json.dumps(
            {
                "status": "disabled_data_collection",
                "generation": 0,
                "task": instruction,
                "collection_mode": collection_mode,
            },
            sort_keys=True,
        )
        now_ns = time.time_ns()
        node.send_output(
            "status",
            pa.array([payload]),
            {"timestamp_ns": now_ns, "source_timestamp_ns": now_ns, "generation": 0},
        )
        for event in node:
            if event is None or event.get("type") == "STOP":
                break
        return

    try:
        from client_server.ws.model_client import WsModelClient
    except ModuleNotFoundError as exc:
        raise RuntimeError("Piper mixed mode requires the policy client") from exc
    action_frequency_hz, execute_chunk_size, prefetch_enabled, execute_chunk_mode = (
        policy_runtime_parameters()
    )
    rtc_default = RealTimeChunkingConfig(
        mode=RtcMode.ALIGN if prefetch_enabled else RtcMode.DISABLED,
        latency_quantile=0.95,
        latency_margin_steps=1,
        latency_window_size=32,
    )
    allow_no_action = no_action_enabled()
    bounded_review = os.getenv("PIPER_BOUNDED_REVIEW_RELEASE") == "1"
    continuous_review = os.getenv("PIPER_CONTINUOUS_REVIEW") == "1"
    trajectory_review = os.getenv("PIPER_GPT_TRAJECTORY_RELEASE") == "1"
    review_profile = os.getenv("PIPER_REVIEW_PROFILE", "legacy")
    validate_review_runtime(
        profile=review_profile, bounded_review=bounded_review,
        continuous_review=continuous_review, trajectory_review=trajectory_review,
        allow_no_action=allow_no_action, prefetch_enabled=prefetch_enabled,
        execute_chunk_size=execute_chunk_size, execute_chunk_mode=execute_chunk_mode,
        action_frequency_hz=action_frequency_hz,
    )
    scheduler = ActionChunkRuntime(
        PIPER_X_DUAL_ACTION_SCHEMA,
        action_frequency_hz=action_frequency_hz,
        action_chunk_size=execute_chunk_size,
        auto_action_chunk_size=execute_chunk_mode == "auto",
        trigger_steps=0,
        rtc_config=RealTimeChunkingConfig.from_environ(
            os.environ,
            default=rtc_default,
        ),
        allow_no_action=allow_no_action,
        allow_review_wait=continuous_review,
        trajectory_profile=os.environ.get("ACTION_TRAJECTORY_PROFILE") or None,
        trajectory_time_scale=policy_trajectory_time_scale(),
    )
    from XPolicyLab.policy.Pi_05.image_transport import for_deployment
    image_transport = for_deployment(os.getenv("PIPER_DEPLOYMENT", ""))

    def make_policy_client():
        client = WsModelClient(
            url=policy_url,
            evaluation_id=f"piper-policy-{robot_id}",
            trial_id=f"piper-policy-{robot_id}",
        )
        return image_transport.wrap(client) if image_transport is not None else client

    policy = ActionChunkPolicyRuntime(
        scheduler,
        AsyncPolicyInference(
            make_policy_client,
            allow_no_action=allow_no_action,
            allow_review_wait=continuous_review,
        ),
        retry_seconds=float(os.getenv("MODEL_RETRY_SECONDS", "0.5")),
    )
    training_prefix_enabled = os.getenv("PIPER_TRAINING_RTC_ENABLED", "0") == "1"
    if any(os.environ.get(k) for k in ("PIPER_NATIVE_PRECOMMIT_PROFILE", "PIPER_NATIVE_TRAJECTORY_PROFILE", "PIPER_NATIVE_VELOCITY_SEAM_PROFILE", "PIPER_NATIVE_ONE_EURO_PROFILE", "PIPER_NATIVE_JOINT_CLIP_PROFILE")) and not training_prefix_enabled:
        raise ValueError("Native suffix smoothing requires training-prefix RTC")
    if training_prefix_enabled:
        if os.getenv("PIPER_MODEL_FILTER_ENABLED", "1") != "0":
            raise ValueError("Native prefix candidate requires optional owner model low-pass disabled")
        if os.environ.get("ACTION_TRAJECTORY_PROFILE"):
            raise ValueError("Native prefix validation requires optional RTG disabled")
        if allow_no_action or bounded_review or continuous_review or trajectory_review:
            raise ValueError("Training-prefix RTC requires the standard executable policy path")
        from XPolicyLab.real_world.training_time_rtc import TrainingPrefixRuntime, TrainingPrefixPolicyRuntime, TrainingPrefixInference
        scheduler = TrainingPrefixRuntime(
            PIPER_X_DUAL_ACTION_SCHEMA, action_frequency_hz=action_frequency_hz,
            action_chunk_size=execute_chunk_size, auto_action_chunk_size=execute_chunk_mode == "auto",
            rtc_config=RealTimeChunkingConfig.from_environ(os.environ, default=rtc_default),
            prediction_horizon=int(os.getenv("PIPER_POLICY_ACTION_HORIZON", "50")),
            commit_steps=execute_chunk_size,
            future_budget_steps=int(os.getenv("PIPER_NATIVE_RTC_FUTURE_STEPS", "5")),
            max_prefix_steps=int(os.getenv("PIPER_NATIVE_RTC_MAX_PREFIX_STEPS", "6")),
            enforce_wallclock_deadline=os.getenv("PIPER_NATIVE_RTC_WALLCLOCK_DEADLINE", "0") == "1",
            native_trajectory_profile=os.environ.get("PIPER_NATIVE_TRAJECTORY_PROFILE") or None,
            velocity_seam_profile=os.environ.get("PIPER_NATIVE_VELOCITY_SEAM_PROFILE") or None,
            one_euro_profile=os.environ.get("PIPER_NATIVE_ONE_EURO_PROFILE") or None,
            joint_clip_profile=os.environ.get("PIPER_NATIVE_JOINT_CLIP_PROFILE") or None,
            precommit_profile=os.environ.get("PIPER_NATIVE_PRECOMMIT_PROFILE") or None,
        )
        client_factory = policy.inference._client_factory
        policy.inference.close()  # No requests have been submitted during startup.
        policy = TrainingPrefixPolicyRuntime(scheduler, TrainingPrefixInference(
            client_factory, suffix_processor=scheduler.prepare_prefix_result
            if scheduler.native_trajectory or scheduler.velocity_seam or scheduler.one_euro or scheduler.joint_clip else None))
    observation = PiperPolicyObservationAdapter(instruction, robot_id)
    snapshots = ReadOnlyObservationSnapshots.from_environment()
    if bounded_review and scheduler.queue.rtc_config.mode != RtcMode.DISABLED:
        raise ValueError("bounded reviewed prefix requires RTC disabled")
    generation = 0
    chunk_id = 0
    no_action_count = 0
    active_release = None
    active_request_id = ""
    chunk_log_interval = max(
        1, int(os.getenv("PIPER_POLICY_CHUNK_LOG_INTERVAL", "25"))
    )
    last_status = ""

    def send_status(status: str, **detail: Any) -> None:
        nonlocal last_status
        payload = {
            "status": status,
            "generation": generation,
            "task": observation.instruction,
            "review_profile": review_profile,
            "image_transport": image_transport.status if image_transport is not None else {"enabled": False},
            "execution_chunk_mode": scheduler.action_chunk_mode,
            "execution_chunk_default_steps": scheduler.default_action_chunk_size,
            "execution_chunk_fixed_steps": scheduler.action_chunk_size,
            "execution_chunk_effective_steps": (
                scheduler.last_effective_action_chunk_size
            ),
            "rtc": rtc_status(scheduler.queue.rtc_config),
            "execution_timebase": scheduler.timebase.status(action_frequency_hz),
            "execution_dispatch_steps": scheduler.last_dispatch_chunk_size,
            "training_time_rtc": ({"enabled": True, "commit_steps": scheduler.commit_steps,
                                   "future_budget_steps": scheduler.future_budget_steps,
                                   "max_prefix_steps": scheduler.max_prefix_steps,
                                   "trajectory_smoothing": (scheduler.native_trajectory.config if scheduler.native_trajectory else None),
                                   "velocity_seam": (scheduler.velocity_seam.config if scheduler.velocity_seam else None),
                                   "one_euro": (scheduler.one_euro.config if scheduler.one_euro else None),
                                   "joint_clip": (scheduler.joint_clip.config if scheduler.joint_clip else None),
                                   "precommit": (scheduler.precommit.config if scheduler.precommit else None),
                                   "trajectory_report": ({k:v for k,v in scheduler.last_native_info.items() if k.startswith('trajectory_')})}
                                  if training_prefix_enabled else None),
            **detail,
        }
        signature = json.dumps(payload, sort_keys=True, default=str)
        if signature == last_status:
            return
        last_status = signature
        now_ns = time.time_ns()
        node.send_output(
            "status",
            pa.array([signature]),
            {
                "timestamp_ns": now_ns,
                "source_timestamp_ns": now_ns,
                "generation": generation,
            },
        )

    # Publish an accurate stopped-policy state before blocking on Dora input.
    # The combined Pigey launcher waits for this acknowledgement so it cannot
    # race a policy node that has not completed startup.
    send_status("paused")

    try:
        for event in node:
            if event is None or event.get("type") == "STOP":
                break
            if event.get("type") != "INPUT":
                continue
            event_id = str(event["id"])
            metadata = dict(event.get("metadata", {}))
            if event_id == "state_robot":
                observation.update_state(event["value"], metadata)
                continue
            if event_id.startswith("image_"):
                observation.update_image(
                    event_id.removeprefix("image_"), event["value"], metadata
                )
                continue
            if event_id == "task":
                selected_task = str(_scalar(event["value"])).strip()
                custom_gpt_task = (os.getenv('PIPER_ALLOW_CUSTOM_TASK') == '1'
                                   and bool(selected_task) and len(selected_task) <= 4000)
                if selected_task not in allowed_tasks and not custom_gpt_task:
                    raise ValueError(
                        "Piper UI task is not supported by the selected checkpoint"
                    )
                observation.instruction = selected_task
                send_status("paused" if not scheduler.enabled else "ready")
                print(f"PIPER_X_DUAL_TASK {selected_task}", flush=True)
                continue
            if event_id == "execution_config":
                configuration = json.loads(str(_scalar(event["value"])))
                if not isinstance(configuration, dict):
                    raise ValueError("execution configuration must be a JSON object")
                mode = str(configuration.get("mode", "")).strip().lower()
                steps = configuration.get("steps")
                try:
                    if bounded_review and (mode != ("auto" if continuous_review else "fixed") or (not continuous_review and steps != execute_chunk_size)):
                        raise ValueError("reviewed prefix length is fixed")
                    scheduler.configure_action_chunk(mode, steps)
                except (RuntimeError, ValueError) as exc:
                    send_status("configuration_error", error=str(exc))
                    print(
                        f"PIPER_X_DUAL_EXECUTION_CONFIG_REJECTED {exc}",
                        flush=True,
                    )
                    continue
                send_status("paused")
                print(
                    "PIPER_X_DUAL_EXECUTION_CONFIG "
                    + json.dumps(
                        {
                            "mode": scheduler.action_chunk_mode,
                            "steps": scheduler.action_chunk_size,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                continue
            if event_id == "rtc_config":
                try:
                    configuration = json.loads(str(_scalar(event["value"])))
                    if bounded_review and configuration.get("mode") != "disabled":
                        raise ValueError("reviewed prefix requires RTC disabled")
                    scheduler.configure_rtc(
                        rtc_config_update(scheduler.queue.rtc_config, configuration)
                    )
                except (
                    RuntimeError,
                    TypeError,
                    ValueError,
                    json.JSONDecodeError,
                ) as exc:
                    send_status("configuration_error", error=str(exc))
                    print(f"PIPER_X_DUAL_RTC_CONFIG_REJECTED {exc}", flush=True)
                    continue
                send_status("paused")
                print(
                    "PIPER_X_DUAL_RTC_CONFIG "
                    + json.dumps(
                        rtc_status(scheduler.queue.rtc_config), sort_keys=True
                    ),
                    flush=True,
                )
                continue
            if event_id == "control":
                control = json.loads(str(_scalar(event["value"])))
                generation = int(control["generation"])
                policy.control(str(control["command"]), generation)
                active_release = None
                active_request_id = ""
                send_status("loading" if scheduler.enabled else "paused")
                continue
            if event_id != "tick":
                continue

            frame = observation.build(scheduler.queue)
            if snapshots is not None:
                snapshots.offer(frame)
            tick = policy.tick(
                None if frame is None else frame.to_policy_observation(),
                observation_timestamp_ns=0 if frame is None else frame.timestamp_ns,
                accepted_timestamp_ns=time.time_ns(),
            )
            result = tick.inference
            if result is not None:
                if result.error is not None:
                    send_status("error", error=result.error)
                elif tick.no_action_accepted:
                    assert result.no_action is not None and tick.step is None
                    no_action_count += 1
                    detail = no_action_status(result.no_action, no_action_count)
                    send_status("paused", roundtrip_ms=result.roundtrip_ms, **detail)
                    if no_action_count == 1 or no_action_count % chunk_log_interval == 0:
                        print(
                            "PIPER_X_DUAL_NO_ACTION "
                            + json.dumps(detail, sort_keys=True),
                            flush=True,
                        )
                elif result.chunk is not None:
                    alignment = scheduler.queue.last_alignment
                    if alignment is not None and alignment.accepted:
                        if bounded_review:
                            active_release = result.chunk.metadata.get("review_release")
                            active_request_id = result.chunk.request_id
                            if (not isinstance(active_release, dict)
                                    or not 1 <= len(result.chunk.values) <= execute_chunk_size
                                    or (not continuous_review and len(result.chunk.values) != execute_chunk_size)
                                    or active_release.get("steps") != len(result.chunk.values)):
                                raise ValueError("exact reviewed prefix and release required")
                            maximum = release_step_limit(
                                active_release, profile=review_profile,
                                allow_trajectory=trajectory_review,
                            )
                            if len(result.chunk.values) > maximum:
                                raise ValueError(f"reviewed {active_release.get('kind', 'legacy')} prefix exceeds {maximum} steps")
                            if (active_release.get("execution_observation_timestamp_ns") != result.pending.observation_timestamp_ns
                                    or active_release.get("execution_session_id") != observation.session_id
                                    or active_release.get("generation") != generation):
                                raise ValueError("review release does not bind this execution observation")
                        chunk_id += 1
                        info = {
                            "model_chunk_id": chunk_id,
                            "model_action_horizon": len(result.chunk.values),
                            "execution_chunk_mode": scheduler.action_chunk_mode,
                            "execution_chunk_steps": (
                                scheduler.last_effective_action_chunk_size
                            ),
                            "roundtrip_ms": result.roundtrip_ms,
                            **scheduler.timebase.status(action_frequency_hz),
                            "execution_dispatch_steps": scheduler.last_dispatch_chunk_size,
                            **alignment.as_dict(),
                            **(scheduler.queue.trajectory_processor.last_report
                               if scheduler.queue.trajectory_processor is not None else {}),
                        }
                        if training_prefix_enabled:
                            info.update(scheduler.last_native_info)
                        node.send_output(
                            "model_info",
                            pa.array([json.dumps(info, sort_keys=True)]),
                            _dora_metadata({
                                "timestamp_ns": result.ended_ns,
                                "source_timestamp_ns": observation.timestamp_ns,
                                "generation": generation,
                                **info,
                            }),
                        )
                        if chunk_id == 1 or chunk_id % chunk_log_interval == 0:
                            print(
                                "PIPER_X_DUAL_ACTION_CHUNK "
                                + json.dumps(info, sort_keys=True),
                                flush=True,
                            )

            if tick.step is not None:
                now_ns = time.time_ns()
                release_metadata = {}
                if scheduler.timebase.scale != 1:
                    release_metadata.update(
                        trajectory_time_scale=scheduler.timebase.scale,
                        trajectory_execution_tick_index=tick.step.execution_tick_index,
                        trajectory_source_phase=tick.step.source_phase,
                    )
                if training_prefix_enabled and (scheduler.native_trajectory is not None or scheduler.velocity_seam is not None or scheduler.one_euro is not None or scheduler.joint_clip is not None):
                    release_metadata.update(
                        trajectory_raw_action=json.dumps(scheduler.last_raw_action.tolist()),
                        **{k: v for k, v in scheduler.last_native_info.items() if k.startswith('trajectory_')},
                    )
                if bounded_review:
                    if active_release is None or now_ns >= active_release["expires_ns"]:
                        raise ValueError("review release missing or expired before policy emission")
                    release_metadata = {"review_release": json.dumps(active_release, sort_keys=True),
                                        "policy_request_id": active_request_id}
                if training_prefix_enabled and scheduler.precommit is not None:
                    release_metadata['native_precommit_version'] = 1
                node.send_output(
                    "action_robot",
                    _named_vector_array(pa, tick.step.values),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": observation.timestamp_ns,
                        "generation": tick.step.generation,
                        "model_generation": tick.step.generation,
                        "model_chunk_id": chunk_id,
                        "model_chunk_step_index": tick.step.chunk_step_index,
                        "queue_depth": len(scheduler.queue.actions),
                        **release_metadata,
                    },
                )
                send_status("ready", model_chunk_id=chunk_id)
            elif scheduler.enabled and policy.inference.busy:
                send_status("loading" if chunk_id == 0 else "starved")
    finally:
        policy.close()
        if snapshots is not None:
            snapshots.close()


if __name__ == "__main__":
    main()
