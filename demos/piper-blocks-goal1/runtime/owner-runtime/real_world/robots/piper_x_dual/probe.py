from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any, Callable

from .camera import CameraStreamProbe, probe_rgb_streams
from .inventory import DualPiperSnapshot, InventoryError, ReadOnlyDualPiperOwner
from .profile import (
    ProfileError,
    load_device_instance,
    load_profile,
    validate_device_instance,
)
from .sdk import DualPiperSdkSession


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PROFILE = Path(__file__).with_name("profile.yml")


def _default_device_path(profile: dict[str, Any]) -> Path:
    return REPOSITORY_ROOT / str(profile["device_instance"]["default_path"])


def collect_snapshots(
    owner: ReadOnlyDualPiperOwner,
    *,
    sample_count: int,
    sample_interval_s: float,
    clock: Callable[[], float] = time.time,
    sleep: Callable[[float], None] = time.sleep,
) -> list[DualPiperSnapshot]:
    if sample_count < 2:
        raise InventoryError("probe requires at least two samples")
    if sample_interval_s <= 0.0:
        raise InventoryError("sample interval must be positive")
    snapshots: list[DualPiperSnapshot] = []
    for index in range(sample_count):
        snapshots.append(owner.snapshot(now_s=clock()))
        if index + 1 < sample_count:
            sleep(sample_interval_s)
    for role in ("left", "right"):
        timestamps = [getattr(snapshot, role).latest_timestamp_s for snapshot in snapshots]
        if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
            raise InventoryError(f"{role}_arm feedback timestamps are not advancing")
    return snapshots


def build_report(
    profile: dict[str, Any],
    device: dict[str, Any],
    snapshots: list[DualPiperSnapshot],
    camera_probes: dict[str, CameraStreamProbe],
) -> dict[str, Any]:
    final = snapshots[-1]
    return {
        "robot_id": device["robot_id"],
        "robot_type": device["robot_type"],
        "profile_id": profile["profile_id"],
        "readiness": "READY_NO_WRITE",
        "healthy": True,
        "read_only": True,
        "write_enabled": False,
        "sample_count": len(snapshots),
        "state_order": [
            *profile["body"]["arms"]["left_arm"]["joint_names"],
            "left_gripper",
            *profile["body"]["arms"]["right_arm"]["joint_names"],
            "right_gripper",
        ],
        "state_14d": list(final.state_14d),
        "maximum_age_ms": max(snapshot.maximum_age_ms for snapshot in snapshots),
        "maximum_inter_arm_skew_ms": max(
            snapshot.sample_skew_ms for snapshot in snapshots
        ),
        "arms": {
            role: {
                "can_adapter_serial": device["arms"][role]["can_adapter_serial"],
                "netdev": device["arms"][role]["netdev"],
                "firmware_driver": device["arms"][role]["firmware"]["sdk_driver"],
                "controller_mode": getattr(final, role.removesuffix("_arm")).controller_mode,
                "controller_status": getattr(final, role.removesuffix("_arm")).controller_status,
                "arm_status_hz": getattr(final, role.removesuffix("_arm")).arm_status_hz,
                "gripper_status_hz": getattr(final, role.removesuffix("_arm")).gripper_status_hz,
            }
            for role in ("left_arm", "right_arm")
        },
        "cameras": {
            role: {
                "serial": probe.serial,
                "usb_descriptor_serial": probe.usb_descriptor_serial,
                "firmware_version": probe.firmware_version,
                "usb_type": probe.usb_type,
                "physical_port": probe.physical_port,
                "startup_attempts": probe.startup_attempts,
                "startup_frame_count": probe.startup_frame_count,
                "frame_count": probe.frame_count,
                "first_frame_number": probe.first_frame_number,
                "last_frame_number": probe.last_frame_number,
                "dropped_frame_count": probe.dropped_frame_count,
                "measured_fps": probe.measured_fps,
                "width": probe.width,
                "height": probe.height,
                "fps": probe.fps,
                "stream_validated": True,
            }
            for role, probe in camera_probes.items()
        },
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Passive dual-Piper inventory probe; never enables or commands an actuator."
    )
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--device-instance", type=Path)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--sample-interval", type=float, default=0.05)
    parser.add_argument("--camera-duration", type=float, default=2.0)
    parser.add_argument("--json", action="store_true", dest="as_json")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        profile = load_profile(args.profile.resolve())
        device_path = (
            args.device_instance.resolve()
            if args.device_instance is not None
            else _default_device_path(profile)
        )
        device = load_device_instance(device_path)
        validate_device_instance(profile, device)
        if args.settle_seconds < 0.0:
            raise InventoryError("settle time must be non-negative")
        with DualPiperSdkSession(profile, device) as owner:
            time.sleep(args.settle_seconds)
            color = profile["cameras"]["color"]
            startup = profile["cameras"]["startup"]
            camera_probes = probe_rgb_streams(
                device["cameras"],
                width=int(color["width"]),
                height=int(color["height"]),
                fps=int(color["fps"]),
                duration_s=args.camera_duration,
                startup_max_attempts=int(startup["max_attempts"]),
                startup_retry_delay_s=float(startup["retry_delay_seconds"]),
                startup_readiness_s=float(startup["readiness_seconds"]),
            )
            snapshots = collect_snapshots(
                owner,
                sample_count=args.samples,
                sample_interval_s=args.sample_interval,
            )
        report = build_report(profile, device, snapshots, camera_probes)
        if args.as_json:
            print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        else:
            print(f"Robot              : {report['robot_id']}")
            print("Readiness          : READY_NO_WRITE")
            for role, arm in report["arms"].items():
                print(
                    f"{role:19}: {arm['controller_mode']}/{arm['controller_status']} "
                    f"arm={arm['arm_status_hz']:.1f}Hz gripper={arm['gripper_status_hz']:.1f}Hz "
                    f"netdev={arm['netdev']}"
                )
            print(f"Max state age      : {report['maximum_age_ms']:.3f} ms")
            print(f"Max inter-arm skew : {report['maximum_inter_arm_skew_ms']:.3f} ms")
            for role, camera in report["cameras"].items():
                print(
                    f"{role:19}: {camera['frame_count']} frames "
                    f"{camera['width']}x{camera['height']}@{camera['measured_fps']:.2f} "
                    f"serial={camera['serial']} attempts={camera['startup_attempts']}"
                )
            print("Probe mode         : READ ONLY (write interface unavailable)")
        return 0
    except (ProfileError, InventoryError, KeyError, OSError) as exc:
        print(f"BLOCKED: {exc}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
