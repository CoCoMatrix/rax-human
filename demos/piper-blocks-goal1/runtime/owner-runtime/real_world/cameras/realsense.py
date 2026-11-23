from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
import json
from pathlib import Path
import threading
import time
import uuid
from typing import Any, Mapping

import numpy as np


class RealSenseError(RuntimeError):
    """A configured RealSense stream cannot satisfy its runtime contract."""


@dataclass(frozen=True, slots=True)
class RealSenseFrame:
    role: str
    serial: str
    image: np.ndarray
    frame_number: int
    device_timestamp_ms: float
    received_timestamp_ns: int
    timestamp_domain: str


@dataclass(frozen=True, slots=True)
class RealSenseStartupReport:
    role: str
    serial: str
    firmware_version: str
    usb_type: str
    physical_port: str
    startup_attempts: int
    startup_frame_count: int


@dataclass(slots=True)
class _StreamState:
    role: str
    serial: str
    pipeline: Any
    config: Any
    stop: threading.Event
    runtime_ready: threading.Event
    future: Future[None]
    frames: int = 0
    dropped_frames: int = 0
    last_frame_number: int = -1
    latest: RealSenseFrame | None = None
    error: str = ""
    timeout_count: int = 0
    restart_count: int = 0
    recovery_attempts: int = 0
    fatal: bool = False
    owner_generation: str = field(default_factory=lambda: uuid.uuid4().hex)
    received_framesets: int = 0


def _import_realsense() -> Any:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RealSenseError("pyrealsense2 is not installed in this runtime") from exc
    return rs


def _device_info(device: Any, rs: Any) -> dict[str, str]:
    fields = {
        "serial": rs.camera_info.serial_number,
        "firmware_version": rs.camera_info.firmware_version,
        "usb_type": rs.camera_info.usb_type_descriptor,
        "physical_port": rs.camera_info.physical_port,
    }
    return {name: str(device.get_info(field)) for name, field in fields.items()}


def _validate_identity(
    role: str, expected: Mapping[str, Any], actual: Mapping[str, str]
) -> None:
    for field in ("serial", "firmware_version", "usb_type"):
        configured = expected.get(field)
        if configured is not None and str(configured) != actual[field]:
            raise RealSenseError(
                f"{role} {field} mismatch: {actual[field]!r} != {str(configured)!r}"
            )


class MultiRealSenseSource:
    """Own multiple D4xx pipelines with strict staged startup and latest-frame caches."""

    def __init__(
        self,
        cameras: Mapping[str, Mapping[str, Any]],
        *,
        width: int,
        height: int,
        fps: int,
        startup_max_attempts: int = 3,
        startup_retry_delay_s: float = 0.5,
        startup_readiness_s: float = 0.5,
        frame_timeout_ms: int = 3000,
        max_frame_age_ms: float = 1000.0,
        recovery_max_attempts: int = 3,
        recovery_retry_delay_s: float = 1.0,
        head_stereo_sink: Any | None = None,
        stereo_sinks: Mapping[str, Any] | None = None,
        diagnostics: bool = False,
        sdk_log_file: str | None = None,
    ) -> None:
        if not cameras:
            raise RealSenseError("at least one RealSense camera must be configured")
        serials = [str(camera["serial"]) for camera in cameras.values()]
        if len(serials) != len(set(serials)):
            raise RealSenseError("RealSense serials must be unique")
        if width <= 0 or height <= 0 or fps <= 0:
            raise RealSenseError("image dimensions and fps must be positive")
        if (
            startup_max_attempts < 1
            or startup_retry_delay_s <= 0.0
            or startup_readiness_s <= 0.0
            or frame_timeout_ms < 1
            or max_frame_age_ms <= 0.0
            or recovery_max_attempts < 1
            or recovery_retry_delay_s <= 0.0
        ):
            raise RealSenseError("RealSense startup/recovery settings must be positive")
        self.cameras = {role: dict(camera) for role, camera in cameras.items()}
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.startup_max_attempts = int(startup_max_attempts)
        self.startup_retry_delay_s = float(startup_retry_delay_s)
        self.startup_readiness_s = float(startup_readiness_s)
        self.frame_timeout_ms = int(frame_timeout_ms)
        self.max_frame_age_ms = float(max_frame_age_ms)
        self.recovery_max_attempts = int(recovery_max_attempts)
        self.recovery_retry_delay_s = float(recovery_retry_delay_s)
        self._lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=len(cameras))
        self._streams: dict[str, _StreamState] = {}
        self._closed = False
        self.diagnostics = bool(diagnostics)
        self.sdk_log_file = str(Path(sdk_log_file).resolve()) if sdk_log_file else None
        self.head_stereo_sink = head_stereo_sink  # Existing callers remain compatible.
        self.stereo_sinks = dict(stereo_sinks or {})
        if head_stereo_sink is not None:
            if "camera_head" in self.stereo_sinks:raise RealSenseError("Duplicate head stereo sink")
            self.stereo_sinks["camera_head"] = head_stereo_sink
        for role,sink in self.stereo_sinks.items():
            if role not in self.cameras:raise RealSenseError("Stereo sink camera missing: "+role)
            sink.validate_camera(str(self.cameras[role]["serial"]), self.width,self.height,self.fps)

    def _diagnostic(self, stage: str, **details: Any) -> None:
        if self.diagnostics:
            print("REALSENSE_DIAGNOSTIC " + json.dumps(
                {"stage": stage, "host_unix_ns": time.time_ns(), **details},
                sort_keys=True, default=str,
            ), flush=True)

    def _log_profile(self, profile: Any, **details: Any) -> None:
        if not self.diagnostics:
            return
        try:
            streams = []
            for p in profile.get_streams():
                video = p.as_video_stream_profile()
                streams.append({"stream": str(p.stream_type()), "index": p.stream_index(),
                                "format": str(p.format()), "fps": p.fps(),
                                "width": video.width(), "height": video.height()})
            self._diagnostic("pipeline_started", streams=streams, **details)
        except Exception as exc:
            self._diagnostic("pipeline_started_profile_diagnostic_error", error=str(exc), **details)

    def _recover(self, state: _StreamState, detail: str) -> bool:
        """Restart one failed pipeline without interrupting healthy cameras."""
        first_attempt = state.recovery_attempts + 1
        for attempt in range(first_attempt, self.recovery_max_attempts + 1):
            with self._lock:
                state.recovery_attempts = attempt
                state.error = (
                    f"{detail}; restart attempt {attempt}/"
                    f"{self.recovery_max_attempts}"
                )
            try:
                self._diagnostic("recovery_stop_begin", role=state.role, serial=state.serial, attempt=attempt)
                state.pipeline.stop()
                self._diagnostic("recovery_stop_done", role=state.role, serial=state.serial, attempt=attempt)
            except RuntimeError:
                pass
            if state.stop.wait(self.recovery_retry_delay_s):
                return False
            try:
                self._diagnostic("recovery_start_begin", role=state.role, serial=state.serial, attempt=attempt)
                profile = state.pipeline.start(state.config)
                self._log_profile(profile, role=state.role, serial=state.serial, attempt=attempt)
            except RuntimeError as exc:
                detail = f"{type(exc).__name__}: {exc}"
                continue
            with self._lock:
                state.restart_count += 1
                state.owner_generation = uuid.uuid4().hex
                state.received_framesets = 0
                state.error = (
                    "waiting for frames after restart attempt "
                    f"{attempt}/{self.recovery_max_attempts}"
                )
            return True
        with self._lock:
            state.fatal = True
            state.error = (
                f"recovery exhausted after {self.recovery_max_attempts} attempts: "
                f"{detail}"
            )
        return False

    def _consume(self, state: _StreamState) -> None:
        while not state.stop.is_set():
            try:
                frameset = state.pipeline.wait_for_frames(self.frame_timeout_ms)
                received_timestamp_ns = time.time_ns()
                state.received_framesets += 1
                color = frameset.get_color_frame()
                if state.received_framesets == 1:
                    self._diagnostic("first_frameset", role=state.role, serial=state.serial,
                                     owner_generation=state.owner_generation,
                                     received_timestamp_ns=received_timestamp_ns,
                                     has_color=bool(color))
            except RuntimeError as exc:
                if state.stop.is_set():
                    return
                detail = f"{type(exc).__name__}: {exc}"
                self._diagnostic("wait_frames_error", role=state.role, serial=state.serial,
                                 received_framesets=state.received_framesets,
                                 rgb_frames=state.frames, error=detail)
                with self._lock:
                    state.timeout_count += 1
                    state.error = detail
                # Startup has its own staged retry loop. Runtime recovery begins
                # only after that camera has passed the readiness frame gate.
                if not state.runtime_ready.is_set() or not self._recover(
                    state, detail
                ):
                    return
                continue
            if not color:
                continue
            frame_number = int(color.get_frame_number())
            image = np.asanyarray(color.get_data()).copy()
            if image.dtype != np.uint8 or image.shape != (
                self.height,
                self.width,
                3,
            ):
                with self._lock:
                    state.error = f"invalid BGR frame shape/dtype: {image.shape}/{image.dtype}"
                    state.fatal = True
                return
            domain = str(color.get_frame_timestamp_domain()).split(".")[-1]
            frame = RealSenseFrame(
                role=state.role,
                serial=state.serial,
                image=image,
                frame_number=frame_number,
                device_timestamp_ms=float(color.get_timestamp()),
                received_timestamp_ns=received_timestamp_ns,
                timestamp_domain=domain,
            )
            with self._lock:
                if state.last_frame_number >= 0:
                    state.dropped_frames += max(
                        0, frame_number - state.last_frame_number - 1
                    )
                state.last_frame_number = frame_number
                state.frames += 1
                state.latest = frame
                state.error = ""
                state.recovery_attempts = 0
                state.fatal = False
            if state.role in self.stereo_sinks:
                # The existing owner hands off one copied, paired frameset;
                # the sink never creates a pipeline and writes files off-thread.
                self.stereo_sinks[state.role].capture(
                    frameset, serial=state.serial,
                    owner_generation=state.owner_generation,
                    host_received_unix_ns=received_timestamp_ns,
                )

    def start(self) -> dict[str, RealSenseStartupReport]:
        if self._streams or self._closed:
            raise RealSenseError("RealSense source can only be started once")
        rs = _import_realsense()
        if self.sdk_log_file:
            if Path(self.sdk_log_file).exists():
                raise RealSenseError("SDK diagnostic log path already exists")
            rs.log_to_file(rs.log_severity.debug, self.sdk_log_file)
            self._diagnostic("sdk_log_enabled", path=self.sdk_log_file)
        context = rs.context()
        discovered = {
            str(device.get_info(rs.camera_info.serial_number)): device
            for device in context.query_devices()
        }
        missing = sorted(
            str(camera["serial"])
            for camera in self.cameras.values()
            if str(camera["serial"]) not in discovered
        )
        if missing:
            raise RealSenseError(f"configured RealSense serials are missing: {missing}")
        required_frames = max(3, int(round(self.startup_readiness_s * self.fps)))
        reports: dict[str, RealSenseStartupReport] = {}
        try:
            for role, expected in self.cameras.items():
                serial = str(expected["serial"])
                actual = _device_info(discovered[serial], rs)
                _validate_identity(role, expected, actual)
                last_detail = "pipeline did not start"
                for attempt in range(1, self.startup_max_attempts + 1):
                    pipeline = rs.pipeline(context)
                    config = rs.config()
                    config.enable_device(serial)
                    config.enable_stream(
                        rs.stream.color,
                        self.width,
                        self.height,
                        rs.format.bgr8,
                        self.fps,
                    )
                    if role in self.stereo_sinks:
                        self.stereo_sinks[role].configure_streams(config, rs)
                    try:
                        self._diagnostic("startup_begin", role=role, serial=serial,
                                         attempt=attempt, width=self.width, height=self.height,
                                         fps=self.fps, head_ir_enabled=(role in self.stereo_sinks))
                        pipeline_profile = pipeline.start(config)
                        self._log_profile(pipeline_profile, role=role, serial=serial, attempt=attempt)
                    except RuntimeError as exc:
                        last_detail = f"{type(exc).__name__}: {exc}"
                        self._diagnostic("startup_error", role=role, serial=serial,
                                         attempt=attempt, error=last_detail)
                        if attempt < self.startup_max_attempts:
                            time.sleep(self.startup_retry_delay_s)
                        continue
                    stop = threading.Event()
                    runtime_ready = threading.Event()
                    placeholder: Future[None] = Future()
                    state = _StreamState(
                        role=role,
                        serial=serial,
                        pipeline=pipeline,
                        config=config,
                        stop=stop,
                        runtime_ready=runtime_ready,
                        future=placeholder,
                    )
                    state.future = self._executor.submit(self._consume, state)
                    deadline = time.monotonic() + max(
                        4.0, self.startup_readiness_s + 3.0
                    )
                    while time.monotonic() < deadline:
                        with self._lock:
                            count = state.frames
                            error = state.error
                        if error or count >= required_frames:
                            break
                        time.sleep(0.05)
                    with self._lock:
                        count = state.frames
                        error = state.error
                    if not error and count >= required_frames:
                        self._diagnostic("readiness_pass", role=role, serial=serial,
                                         attempt=attempt, rgb_frames=count)
                        self._streams[role] = state
                        state.runtime_ready.set()
                        reports[role] = RealSenseStartupReport(
                            role=role,
                            serial=serial,
                            firmware_version=actual["firmware_version"],
                            usb_type=actual["usb_type"],
                            physical_port=actual["physical_port"],
                            startup_attempts=attempt,
                            startup_frame_count=count,
                        )
                        break
                    last_detail = error or f"only {count} startup frames"
                    self._diagnostic("readiness_failed", role=role, serial=serial,
                                     attempt=attempt, rgb_frames=count,
                                     received_framesets=state.received_framesets,
                                     error=last_detail)
                    stop.set()
                    try:
                        self._diagnostic("startup_cleanup_stop_begin", role=role, serial=serial, attempt=attempt)
                        pipeline.stop()
                        self._diagnostic("startup_cleanup_stop_done", role=role, serial=serial, attempt=attempt)
                    except RuntimeError:
                        pass
                    state.future.result(timeout=4.0)
                    if attempt < self.startup_max_attempts:
                        time.sleep(self.startup_retry_delay_s)
                else:
                    raise RealSenseError(
                        f"{role} staged readiness failed after "
                        f"{self.startup_max_attempts} attempts: {last_detail}"
                    )
            return reports
        except Exception:
            self.close()
            raise

    def latest(self, role: str, *, after_frame_number: int = -1) -> RealSenseFrame | None:
        with self._lock:
            state = self._streams[role]
            if state.error:
                return None
            frame = state.latest
            if frame is None or frame.frame_number <= after_frame_number:
                return None
            age_ms = (time.time_ns() - frame.received_timestamp_ns) / 1_000_000
            if age_ms < 0.0 or age_ms > self.max_frame_age_ms:
                return None
            return frame

    def health(self) -> dict[str, dict[str, Any]]:
        with self._lock:
            now_ns = time.time_ns()
            health: dict[str, dict[str, Any]] = {}
            for role, state in self._streams.items():
                frame_age_ms = (
                    None
                    if state.latest is None
                    else max(
                        0.0,
                        (now_ns - state.latest.received_timestamp_ns) / 1_000_000,
                    )
                )
                if state.fatal or (
                    state.future.done() and not state.stop.is_set()
                ):
                    status = "ERROR"
                elif state.error:
                    status = "RECOVERING"
                elif frame_age_ms is None:
                    status = "STARTING"
                elif frame_age_ms > self.max_frame_age_ms:
                    status = "STALE"
                else:
                    status = "STREAMING"
                health[role] = {
                    "status": status,
                    "serial": state.serial,
                    "frames": state.frames,
                    "dropped_frames": state.dropped_frames,
                    "last_frame_number": state.last_frame_number,
                    "latest_frame_age_ms": frame_age_ms,
                    "error": state.error,
                    "timeout_count": state.timeout_count,
                    "restart_count": state.restart_count,
                    "recovery_attempts": state.recovery_attempts,
                }
                if role in self.stereo_sinks:
                    health[role]["stereo_export"] = self.stereo_sinks[role].status()
                    if role == "camera_head":health[role]["head_stereo_export"] = health[role]["stereo_export"]
            return health

    @staticmethod
    def overall_status(health: Mapping[str, Mapping[str, Any]]) -> str:
        statuses = {str(stream.get("status")) for stream in health.values()}
        if statuses == {"STREAMING"}:
            return "READY_STREAMING"
        if "ERROR" in statuses:
            return "ERROR"
        return "DEGRADED"

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        streams = list(self._streams.values())
        for state in streams:
            state.stop.set()
        for state in streams:
            try:
                self._diagnostic("shutdown_stop_begin", role=state.role, serial=state.serial)
                state.pipeline.stop()
                self._diagnostic("shutdown_stop_done", role=state.role, serial=state.serial)
            except RuntimeError:
                pass
        for state in streams:
            try:
                state.future.result(timeout=4.0)
            except Exception:
                pass
        self._streams.clear()
        self._executor.shutdown(wait=True, cancel_futures=True)

    def __enter__(self) -> MultiRealSenseSource:
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()
