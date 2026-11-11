from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Any, Mapping

import yaml

from .realsense import MultiRealSenseSource
from .head_stereo_export import sink_from_env


def load_runtime_config(
    profile_path: str | Path,
    device_path: str | Path,
    camera_profile_path: str | Path | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    profile = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
    device = yaml.safe_load(Path(device_path).read_text(encoding="utf-8"))
    cameras = dict(device["cameras"])
    if camera_profile_path is None:
        camera_profile = dict(profile["cameras"])
    else:
        selected = yaml.safe_load(
            Path(camera_profile_path).read_text(encoding="utf-8")
        )
        if (
            not isinstance(selected, dict)
            or selected.get("schema") != "xpolicy.realsense-capture-profile/v1"
            or not isinstance(selected.get("cameras"), dict)
        ):
            raise ValueError("invalid RealSense capture profile")
        camera_profile = dict(selected["cameras"])
    required = tuple(camera_profile["required_roles"])
    if set(cameras) != set(required) or len(cameras) != len(required):
        raise ValueError(
            f"camera role mismatch: {tuple(cameras)!r} != {required!r}"
        )
    return camera_profile, cameras


def output_name(role: str, camera: Mapping[str, Any]) -> str:
    configured = camera.get("dora_output")
    return str(configured) if configured else role.replace("camera_", "image_", 1)


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("RealSense Dora node requires dora-rs and pyarrow") from exc

    profile_path = Path(os.environ["REALSENSE_PROFILE"]).resolve()
    device_path = Path(os.environ["REALSENSE_DEVICE_INSTANCE"]).resolve()
    raw_camera_profile_path = os.getenv("REALSENSE_CAMERA_PROFILE", "").strip()
    camera_profile_path = (
        Path(raw_camera_profile_path).resolve() if raw_camera_profile_path else None
    )
    camera_profile, cameras = load_runtime_config(
        profile_path, device_path, camera_profile_path
    )
    color = camera_profile["color"]
    startup = camera_profile["startup"]
    recovery = camera_profile.get("recovery", {})
    if not isinstance(recovery, Mapping):
        raise ValueError("invalid RealSense recovery profile")
    node = Node(os.getenv("DORA_NODE_ID", "realsense_multi"))
    stereo_sink = sink_from_env()
    left_env=dict(os.environ)
    for suffix in ('EXPORT_DIR','CALIBRATION'):
        left_env['REALSENSE_STEREO_'+suffix]=os.getenv('REALSENSE_LEFT_STEREO_'+suffix,'')
    left_env['REALSENSE_STEREO_INTERVAL_MS']=os.getenv('REALSENSE_LEFT_STEREO_INTERVAL_MS','500')
    left_sink=sink_from_env(left_env)
    source = MultiRealSenseSource(
        cameras,
        width=int(color["width"]),
        height=int(color["height"]),
        fps=int(color["fps"]),
        startup_max_attempts=int(startup["max_attempts"]),
        startup_retry_delay_s=float(startup["retry_delay_seconds"]),
        startup_readiness_s=float(startup["readiness_seconds"]),
        frame_timeout_ms=int(recovery.get("frame_timeout_ms", 3000)),
        max_frame_age_ms=float(recovery.get("max_frame_age_ms", 1000.0)),
        recovery_max_attempts=int(
            recovery.get("max_attempts", startup["max_attempts"])
        ),
        recovery_retry_delay_s=float(
            recovery.get("retry_delay_seconds", startup["retry_delay_seconds"])
        ),
        head_stereo_sink=stereo_sink,
        stereo_sinks={"camera_left":left_sink} if left_sink is not None else None,
        diagnostics=os.getenv("REALSENSE_DIAGNOSTICS", "0") == "1",
        sdk_log_file=os.getenv("REALSENSE_SDK_LOG_FILE", "").strip() or None,
    )
    sent_frame_numbers = dict.fromkeys(cameras, -1)
    last_status_at = 0.0
    try:
        reports = source.start()
    except Exception as exc:
        # Publish the staged-start failure before stopping the node. Without
        # this, the owner only sees WAITING_CAMERAS and the useful serial-level
        # error is available solely in the daemon traceback.
        failure = {
            "status": "ERROR",
            "error": f"{type(exc).__name__}: {exc}",
            "cameras": {
                role: {"serial": str(camera.get("serial", ""))}
                for role, camera in cameras.items()
            },
        }
        print("REALSENSE_MULTI " + json.dumps(failure, sort_keys=True), flush=True)
        try:
            node.send_output(
                "status",
                pa.array([json.dumps(failure, sort_keys=True)]),
                {"timestamp_ns": time.time_ns(), "camera_count": len(cameras)},
            )
        except Exception:
            # Preserve the original startup failure if Dora is already
            # stopping and cannot accept the diagnostic status.
            pass
        if stereo_sink is not None:
            stereo_sink.close()
        if left_sink is not None:
            left_sink.close()
        raise
    ready = {
        "status": "READY_STREAMING",
        "cameras": {
            role: {
                "serial": report.serial,
                "startup_attempts": report.startup_attempts,
                "startup_frame_count": report.startup_frame_count,
                "output": output_name(role, cameras[role]),
            }
            for role, report in reports.items()
        },
    }
    print("REALSENSE_MULTI " + json.dumps(ready, sort_keys=True), flush=True)
    last_stream_status = "READY_STREAMING"
    node.send_output(
        "status",
        pa.array([json.dumps(ready, sort_keys=True)]),
        {"timestamp_ns": time.time_ns(), "camera_count": len(cameras)},
    )
    try:
        for event in node:
            if event is None or event.get("type") == "STOP":
                break
            if event.get("type") != "INPUT" or event["id"] != "tick":
                continue
            for role, camera in cameras.items():
                frame = source.latest(
                    role, after_frame_number=sent_frame_numbers[role]
                )
                if frame is None:
                    continue
                sent_frame_numbers[role] = frame.frame_number
                node.send_output(
                    output_name(role, camera),
                    pa.array(frame.image.ravel()),
                    {
                        "width": frame.image.shape[1],
                        "height": frame.image.shape[0],
                        "encoding": "bgr8",
                        "timestamp_ns": frame.received_timestamp_ns,
                        "source_timestamp_ns": frame.received_timestamp_ns,
                        "target_execution_timestamp_ns": frame.received_timestamp_ns,
                        "device_time_ns": int(frame.device_timestamp_ms * 1_000_000),
                        "device_frame_index": frame.frame_number,
                        "timestamp_domain": frame.timestamp_domain,
                        "camera_role": role,
                        "camera_serial": frame.serial,
                    },
                )
            now = time.monotonic()
            if now - last_status_at >= 1.0:
                health = source.health()
                stream_status = source.overall_status(health)
                status = {"status": stream_status, "cameras": health}
                node.send_output(
                    "status",
                    pa.array([json.dumps(status, sort_keys=True)]),
                    {"timestamp_ns": time.time_ns(), "camera_count": len(cameras)},
                )
                if stream_status != last_stream_status:
                    print(
                        "REALSENSE_MULTI "
                        + json.dumps(status, sort_keys=True),
                        flush=True,
                    )
                    last_stream_status = stream_status
                last_status_at = now
    finally:
        health = source.health()
        source.close()
        if stereo_sink is not None:
            stereo_sink.close()
        if left_sink is not None:
            left_sink.close()
        stopped = {"status": "STOPPED", "cameras": health}
        print("REALSENSE_MULTI " + json.dumps(stopped, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
