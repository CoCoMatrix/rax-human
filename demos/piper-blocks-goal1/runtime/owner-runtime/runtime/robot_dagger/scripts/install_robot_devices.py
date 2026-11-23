#!/usr/bin/python3
"""Install and verify host device identities from a robot instance profile."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised on an unprepared host
    raise SystemExit(
        "PyYAML is required. On Ubuntu, install it with: sudo apt install python3-yaml"
    ) from exc


SCHEMA_VERSION = "xpolicy.robot_devices.v1"
HEX_SERIAL_RE = re.compile(r"^[0-9A-Fa-f]+$")
ROLE_RE = re.compile(r"^[a-z][a-z0-9_]*$")
NETDEV_RE = re.compile(r"^can_px_[a-z0-9]+$")

SCRIPT_DIR = Path(__file__).resolve().parent
RUNTIME_ROOT = SCRIPT_DIR.parent
SYSTEMD_CONFIG = RUNTIME_ROOT / "config" / "systemd"
UDEV_CONFIG = RUNTIME_ROOT / "config" / "udev"
LINK_TEMPLATE = SYSTEMD_CONFIG / "10-xpolicylab-piper-can.link.in"
CAN_SERVICE = SYSTEMD_CONFIG / "xpolicylab-piper-can@.service"
CAN_UDEV_RULE = UDEV_CONFIG / "90-xpolicylab-piper-can.rules"
REALSENSE_UDEV_RULE = UDEV_CONFIG / "80-xpolicylab-realsense.rules"


class ProfileError(ValueError):
    """Raised when a robot device profile is incomplete or ambiguous."""


def _required_mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict) or not value:
        raise ProfileError(f"{path} must be a non-empty mapping")
    return value


def load_profile(path: Path) -> dict[str, Any]:
    try:
        profile = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ProfileError(f"profile does not exist: {path}") from exc
    except yaml.YAMLError as exc:
        raise ProfileError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(profile, dict):
        raise ProfileError("profile root must be a mapping")
    validate_profile(profile)
    return profile


def validate_profile(profile: dict[str, Any]) -> None:
    if profile.get("schema_version") != SCHEMA_VERSION:
        raise ProfileError(f"schema_version must be {SCHEMA_VERSION!r}")

    for key in ("robot_id", "robot_type"):
        value = profile.get(key)
        if not isinstance(value, str) or not ROLE_RE.fullmatch(value):
            raise ProfileError(f"{key} must match {ROLE_RE.pattern}")

    arms = _required_mapping(profile.get("arms"), "arms")
    can_serials: set[str] = set()
    netdevs: set[str] = set()
    for role, raw_arm in arms.items():
        if not ROLE_RE.fullmatch(str(role)):
            raise ProfileError(f"invalid arm role: {role!r}")
        arm = _required_mapping(raw_arm, f"arms.{role}")
        serial = str(arm.get("can_adapter_serial", "")).upper()
        netdev = arm.get("netdev")
        bitrate = arm.get("bitrate")
        if not HEX_SERIAL_RE.fullmatch(serial):
            raise ProfileError(f"arms.{role}.can_adapter_serial must be hexadecimal")
        if not isinstance(netdev, str) or not NETDEV_RE.fullmatch(netdev) or len(netdev) > 15:
            raise ProfileError(
                f"arms.{role}.netdev must match {NETDEV_RE.pattern} and be at most 15 characters"
            )
        if bitrate != 1_000_000 or isinstance(bitrate, bool):
            raise ProfileError(f"arms.{role}.bitrate must be 1000000 for Piper")
        if serial in can_serials:
            raise ProfileError(f"duplicate CAN adapter serial: {serial}")
        if netdev in netdevs:
            raise ProfileError(f"duplicate CAN netdev: {netdev}")
        firmware = arm.get("firmware")
        if firmware is not None:
            firmware = _required_mapping(firmware, f"arms.{role}.firmware")
            if firmware.get("sdk_driver") not in {"default", "v183", "v188", "v189"}:
                raise ProfileError(
                    f"arms.{role}.firmware.sdk_driver must be default, v183, v188, or v189"
                )
            for field in (
                "hardware_version",
                "motor_ratio_and_batch",
                "node_type",
                "software_version",
                "production_date",
                "node_number",
            ):
                if not isinstance(firmware.get(field), str) or not firmware[field]:
                    raise ProfileError(f"arms.{role}.firmware.{field} must be a non-empty string")
        end_effector = arm.get("end_effector")
        if end_effector is not None:
            end_effector = _required_mapping(end_effector, f"arms.{role}.end_effector")
            if end_effector.get("type") != "agx_gripper":
                raise ProfileError(f"arms.{role}.end_effector.type must be agx_gripper")
            if end_effector.get("mode") != "width":
                raise ProfileError(f"arms.{role}.end_effector.mode must be width")
            max_range = end_effector.get("max_range_m")
            if not isinstance(max_range, (int, float)) or isinstance(max_range, bool) or max_range <= 0:
                raise ProfileError(f"arms.{role}.end_effector.max_range_m must be positive")
        can_serials.add(serial)
        netdevs.add(netdev)

    cameras = _required_mapping(profile.get("cameras"), "cameras")
    camera_serials: set[str] = set()
    for role, raw_camera in cameras.items():
        if not ROLE_RE.fullmatch(str(role)):
            raise ProfileError(f"invalid camera role: {role!r}")
        camera = _required_mapping(raw_camera, f"cameras.{role}")
        serial = str(camera.get("serial", ""))
        if not serial or not serial.isdigit():
            raise ProfileError(f"cameras.{role}.serial must be a numeric USB serial string")
        if serial in camera_serials:
            raise ProfileError(f"duplicate camera serial: {serial}")
        camera_serials.add(serial)
        usb_serial = str(camera.get("usb_descriptor_serial", serial))
        if not usb_serial.isdigit():
            raise ProfileError(
                f"cameras.{role}.usb_descriptor_serial must be a numeric USB serial string"
            )


def render_link(serial: str, netdev: str) -> str:
    template = LINK_TEMPLATE.read_text(encoding="utf-8")
    return template.replace("@CAN_SERIAL@", serial).replace("@CAN_NAME@", netdev)


def _run(command: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=check, text=True, capture_output=True)


def install_profile(profile: dict[str, Any], *, dry_run: bool) -> None:
    targets: list[tuple[Path, str | Path]] = []
    for arm in profile["arms"].values():
        serial = str(arm["can_adapter_serial"]).upper()
        netdev = str(arm["netdev"])
        target = Path("/etc/systemd/network") / f"10-xpolicylab-piper-{netdev}.link"
        targets.append((target, render_link(serial, netdev)))
    targets.extend(
        [
            (Path("/etc/systemd/system") / CAN_SERVICE.name, CAN_SERVICE),
            (Path("/etc/udev/rules.d") / CAN_UDEV_RULE.name, CAN_UDEV_RULE),
            (Path("/etc/udev/rules.d") / REALSENSE_UDEV_RULE.name, REALSENSE_UDEV_RULE),
        ]
    )

    if dry_run:
        print(f"profile={profile['robot_id']}")
        for target, source in targets:
            source_description = source if isinstance(source, Path) else "generated from profile"
            print(f"would install {target} <- {source_description}")
        return

    if os.geteuid() != 0:
        raise ProfileError("installation requires root; rerun with sudo")

    Path("/etc/systemd/network").mkdir(parents=True, exist_ok=True)
    Path("/etc/systemd/system").mkdir(parents=True, exist_ok=True)
    Path("/etc/udev/rules.d").mkdir(parents=True, exist_ok=True)
    for target, source in targets:
        if isinstance(source, Path):
            shutil.copyfile(source, target)
        else:
            target.write_text(source, encoding="utf-8")
        target.chmod(0o644)

    _run(["systemctl", "daemon-reload"])
    _run(["udevadm", "control", "--reload-rules"])
    print(f"Installed host device rules for {profile['robot_id']}.")
    print("Replug CAN adapters to apply stable names; cameras are selected by serial at runtime.")


def _udev_properties(path: Path) -> dict[str, str]:
    result = _run(["udevadm", "info", "--query=property", f"--path={path}"], check=False)
    properties: dict[str, str] = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            properties[key] = value
    return properties


def _camera_serials_online() -> set[str]:
    serials: set[str] = set()
    video_root = Path("/sys/class/video4linux")
    if not video_root.exists():
        return serials
    for video_device in video_root.glob("video*"):
        serial = _udev_properties(video_device).get("ID_SERIAL_SHORT")
        if serial:
            serials.add(serial)
    return serials


def verify_profile(profile: dict[str, Any]) -> bool:
    ok = True
    print(f"robot_id={profile['robot_id']}")
    for role, arm in profile["arms"].items():
        netdev = str(arm["netdev"])
        expected_serial = str(arm["can_adapter_serial"]).upper()
        net_path = Path("/sys/class/net") / netdev
        if not net_path.exists():
            print(f"CAN {role}: MISSING expected={netdev} serial={expected_serial}")
            ok = False
            continue
        actual_serial = _udev_properties(net_path).get("ID_SERIAL_SHORT", "").upper()
        details = _run(["ip", "-details", "link", "show", "dev", netdev], check=False).stdout
        is_up = "<" in details and "UP" in details.partition(">")[0]
        bitrate_match = re.search(r"\bbitrate\s+(\d+)\b", details)
        actual_bitrate = int(bitrate_match.group(1)) if bitrate_match else None
        matches = (
            actual_serial == expected_serial
            and is_up
            and actual_bitrate == int(arm["bitrate"])
        )
        status = "OK" if matches else "MISMATCH"
        print(
            f"CAN {role}: {status} netdev={netdev} serial={actual_serial or '-'} "
            f"up={is_up} bitrate={actual_bitrate or '-'}"
        )
        ok = ok and matches

    online_cameras = _camera_serials_online()
    for role, camera in profile["cameras"].items():
        serial = str(camera["serial"])
        usb_serial = str(camera.get("usb_descriptor_serial", serial))
        present = usb_serial in online_cameras
        print(
            f"CAMERA {role}: {'OK' if present else 'MISSING'} "
            f"serial={serial} usb_serial={usb_serial}"
        )
        ok = ok and present
    return ok


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Install or verify device identities declared by a robot instance profile."
    )
    parser.add_argument("--profile", required=True, type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="print installation targets")
    mode.add_argument("--verify-only", action="store_true", help="perform no system writes")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        profile = load_profile(args.profile.resolve())
        if args.verify_only:
            return 0 if verify_profile(profile) else 1
        install_profile(profile, dry_run=args.dry_run)
        return 0
    except (ProfileError, OSError, subprocess.CalledProcessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
