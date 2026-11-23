from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import threading
import time
from typing import Any, Mapping

from .inventory import InventoryError


@dataclass(frozen=True)
class CameraStreamProbe:
    role: str
    serial: str
    usb_descriptor_serial: str
    firmware_version: str
    usb_type: str
    physical_port: str
    startup_attempts: int
    startup_frame_count: int
    frame_count: int
    first_frame_number: int
    last_frame_number: int
    dropped_frame_count: int
    measured_fps: float
    width: int
    height: int
    fps: int


def _import_realsense() -> Any:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise InventoryError(
            "pyrealsense2 is unavailable; install the pinned Piper runtime dependency"
        ) from exc
    return rs


def _device_info(device: Any, rs: Any) -> dict[str, str]:
    fields = {
        "serial": rs.camera_info.serial_number,
        "firmware_version": rs.camera_info.firmware_version,
        "usb_type": rs.camera_info.usb_type_descriptor,
        "physical_port": rs.camera_info.physical_port,
    }
    return {name: str(device.get_info(field)) for name, field in fields.items()}


def _validate_device_info(
    role: str, expected: Mapping[str, Any], actual: Mapping[str, str]
) -> None:
    for field in ("serial", "firmware_version", "usb_type"):
        if str(expected[field]) != actual[field]:
            raise InventoryError(
                f"{role} {field} mismatch: {actual[field]!r} != {str(expected[field])!r}"
            )


def _consume_color_frames(
    role: str,
    pipeline: Any,
    *,
    stop: threading.Event,
    lock: threading.Lock,
    frames: dict[str, list[tuple[int, float]]],
    errors: dict[str, str],
    timeout_ms: int,
) -> None:
    while not stop.is_set():
        try:
            frame = pipeline.wait_for_frames(timeout_ms).get_color_frame()
        except RuntimeError as exc:
            if not stop.is_set():
                with lock:
                    errors[role] = f"{type(exc).__name__}: {exc}"
            return
        if frame:
            sample = (int(frame.get_frame_number()), float(frame.get_timestamp()))
            with lock:
                frames[role].append(sample)


def probe_rgb_streams(
    cameras: Mapping[str, Mapping[str, Any]],
    *,
    width: int,
    height: int,
    fps: int,
    duration_s: float,
    startup_max_attempts: int,
    startup_retry_delay_s: float,
    startup_readiness_s: float,
) -> dict[str, CameraStreamProbe]:
    """Strictly stage D435 startup while continuously consuming active streams."""
    if duration_s <= 0.0 or fps <= 0 or width <= 0 or height <= 0:
        raise InventoryError("camera probe dimensions, fps and duration must be positive")
    if startup_max_attempts < 1 or startup_retry_delay_s <= 0.0 or startup_readiness_s <= 0.0:
        raise InventoryError("camera staged-start settings must be positive")
    rs = _import_realsense()
    context = rs.context()
    discovered = {
        str(device.get_info(rs.camera_info.serial_number)): device
        for device in context.query_devices()
    }
    expected_serials = {str(camera["serial"]) for camera in cameras.values()}
    missing = sorted(expected_serials.difference(discovered))
    if missing:
        raise InventoryError(f"configured RealSense serials are missing: {missing}")

    pipelines: dict[str, Any] = {}
    device_infos: dict[str, dict[str, str]] = {}
    startup_frames_required = max(3, int(round(startup_readiness_s * fps)))
    steady_frames_required = max(2, int(round(duration_s * fps * 0.9)))
    frames: dict[str, list[tuple[int, float]]] = {role: [] for role in cameras}
    errors: dict[str, str] = {}
    startup_counts: dict[str, int] = {}
    startup_attempts: dict[str, int] = {}
    baselines: dict[str, int] = {}
    lock = threading.Lock()
    stop = threading.Event()
    executor = ThreadPoolExecutor(max_workers=len(cameras))
    futures = []
    try:
        # Each active stream is drained before the next pipeline starts. D435
        # pipelines can otherwise report start success while an earlier stream
        # stops delivering frames during multi-camera startup.
        for role, expected in cameras.items():
            serial = str(expected["serial"])
            actual = _device_info(discovered[serial], rs)
            _validate_device_info(role, expected, actual)
            last_detail = "pipeline did not start"
            for attempt in range(1, startup_max_attempts + 1):
                with lock:
                    frames[role].clear()
                    errors.pop(role, None)
                pipeline = rs.pipeline(context)
                config = rs.config()
                config.enable_device(serial)
                config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
                try:
                    pipeline.start(config)
                except RuntimeError as exc:
                    last_detail = f"{type(exc).__name__}: {exc}"
                    if attempt < startup_max_attempts:
                        time.sleep(startup_retry_delay_s)
                    continue
                future = executor.submit(
                    _consume_color_frames,
                    role,
                    pipeline,
                    stop=stop,
                    lock=lock,
                    frames=frames,
                    errors=errors,
                    timeout_ms=3000,
                )
                deadline = time.monotonic() + max(4.0, startup_readiness_s + 3.0)
                while time.monotonic() < deadline:
                    with lock:
                        count = len(frames[role])
                        error = errors.get(role)
                    if error is not None or count >= startup_frames_required:
                        break
                    time.sleep(0.05)
                with lock:
                    count = len(frames[role])
                    error = errors.get(role)
                if error is None and count >= startup_frames_required:
                    pipelines[role] = pipeline
                    device_infos[role] = actual
                    futures.append(future)
                    startup_counts[role] = count
                    startup_attempts[role] = attempt
                    break
                last_detail = error or f"only {count} startup frames"
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass
                future.result(timeout=4.0)
                if attempt < startup_max_attempts:
                    time.sleep(startup_retry_delay_s)
            else:
                raise InventoryError(
                    f"{role} staged readiness failed after {startup_max_attempts} attempts: "
                    f"{last_detail}"
                )

        with lock:
            baselines = {role: len(samples) for role, samples in frames.items()}
        time.sleep(duration_s)
        stop.set()
        for future in futures:
            future.result(timeout=4.0)

        with lock:
            errors_after_start = dict(errors)
            frames_by_role = {
                role: list(samples[baselines[role] :])
                for role, samples in frames.items()
            }
        if errors_after_start:
            role, error = next(iter(errors_after_start.items()))
            raise InventoryError(f"{role} stream failed after staged startup: {error}")

        reports: dict[str, CameraStreamProbe] = {}
        for role in cameras:
            frames = frames_by_role[role]
            if len(frames) < steady_frames_required:
                raise InventoryError(
                    f"{role} produced {len(frames)} steady frames; "
                    f"expected at least {steady_frames_required}"
                )
            first_number, first_timestamp = frames[0]
            last_number, last_timestamp = frames[-1]
            sequence_span = last_number - first_number + 1
            dropped = max(0, sequence_span - len(frames))
            elapsed_s = (last_timestamp - first_timestamp) / 1000.0
            measured_fps = (len(frames) - 1) / elapsed_s if elapsed_s > 0.0 else 0.0
            if dropped:
                raise InventoryError(f"{role} dropped {dropped} color frames")
            if measured_fps < fps * 0.9:
                raise InventoryError(
                    f"{role} measured {measured_fps:.3f} fps; expected at least {fps * 0.9:.3f}"
                )
            actual = device_infos[role]
            reports[role] = CameraStreamProbe(
                role=role,
                serial=actual["serial"],
                usb_descriptor_serial=str(cameras[role]["usb_descriptor_serial"]),
                firmware_version=actual["firmware_version"],
                usb_type=actual["usb_type"],
                physical_port=actual["physical_port"],
                startup_attempts=startup_attempts[role],
                startup_frame_count=startup_counts[role],
                frame_count=len(frames),
                first_frame_number=first_number,
                last_frame_number=last_number,
                dropped_frame_count=dropped,
                measured_fps=measured_fps,
                width=width,
                height=height,
                fps=fps,
            )
        return reports
    finally:
        stop.set()
        for pipeline in pipelines.values():
            try:
                pipeline.stop()
            except RuntimeError:
                pass
        executor.shutdown(wait=True, cancel_futures=True)
