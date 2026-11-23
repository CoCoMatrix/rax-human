from __future__ import annotations

import argparse
import ctypes
import json
from typing import Any

from .sdk import (
    CONTROLLER_IP_ENV,
    Dcss,
    StatusTimeoutError,
    TianjiSdk,
    controller_ip_from_env,
    versions_compatible,
)


ARM_STATE_NAMES = {
    0: "IDLE",
    1: "POSITION",
    2: "PVT",
    3: "TORQUE",
    4: "RELEASE",
    100: "ERROR",
    101: "TRANS_TO_POSITION",
    102: "TRANS_TO_PVT",
    103: "TRANS_TO_TORQUE",
    104: "TRANS_TO_RELEASE",
    109: "TRANS_TO_IDLE",
}

ARM_ERROR_NAMES = {
    0: "NONE",
    1: "BUS_PHYSICAL_ABNORMAL",
    2: "SERVO_ERROR",
    3: "INVALID_PVT",
    4: "REQUEST_POSITION_MODE_FAILED",
    5: "POSITION_MODE_FAILED",
    6: "REQUEST_TORQUE_MODE_FAILED",
    7: "TORQUE_MODE_FAILED",
    8: "REQUEST_ENABLE_SERVO_FAILED",
    9: "ENABLE_SERVO_FAILED",
    10: "REQUEST_DISABLE_SERVO_FAILED",
    11: "DISABLE_SERVO_FAILED",
    12: "INTERNAL_ERROR",
    13: "EMERGENCY_STOP",
    14: "FLOATING_BASE_IMU_MISSING",
    15: "PDO_ABNORMAL",
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only Tianji MARVIN SDK/connection/status probe."
    )
    parser.add_argument(
        "--controller-ip",
        help=f"controller IPv4 address; defaults to ${CONTROLLER_IP_ENV}",
    )
    parser.add_argument(
        "--sdk-library",
        help="exact libMarvinSDK.so path; otherwise use SDK environment discovery",
    )
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--json", action="store_true", dest="as_json")
    return parser


def _arm_text(arm: Any) -> str:
    state_name = ARM_STATE_NAMES.get(arm.state, "UNKNOWN")
    error_name = ARM_ERROR_NAMES.get(arm.error_code, "UNKNOWN")
    positions = " ".join(f"{value:.6f}" for value in arm.joint_position_rad)
    return (
        f"{arm.logical_side} (SDK {arm.sdk_arm}): state={arm.state}/{state_name}, "
        f"error={arm.error_code}/{error_name}, frame={arm.frame_serial}, "
        f"stopped={int(arm.stopped)}\n"
        f"  joint_position_rad: {positions}"
    )


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    controller_ip = controller_ip_from_env(args.controller_ip)
    with TianjiSdk(args.sdk_library) as sdk:
        sdk_version = sdk.sdk_version
        sdk.connect(controller_ip)
        controller_version = sdk.controller_version()
        try:
            snapshot = sdk.wait_for_live_state(timeout_s=args.timeout)
            frames_advancing = True
        except StatusTimeoutError:
            # Preserve the valid last controller payload in diagnostics even if
            # its realtime counter is static.
            snapshot = sdk.wait_for_live_state(
                timeout_s=min(args.timeout, 1.0),
                require_advancing_frames=False,
            )
            frames_advancing = False
        servo_errors = sdk.servo_errors()
        version_match = versions_compatible(sdk_version, controller_version)
        health_issues = []
        if not version_match:
            health_issues.append("SDK/controller major versions do not match")
        if not frames_advancing:
            health_issues.append("both-arm realtime frame counters are static")
        for arm in (snapshot.left, snapshot.right):
            if arm.state == 100 or arm.error_code != 0:
                error_name = ARM_ERROR_NAMES.get(arm.error_code, "UNKNOWN")
                health_issues.append(
                    f"{arm.logical_side} arm error {arm.error_code}/{error_name}"
                )
        for side, errors in servo_errors.items():
            if any(errors):
                health_issues.append(f"{side} arm has nonzero servo errors")

        result = {
            "controller_ip": controller_ip,
            "sdk_library": sdk.library_path,
            "sdk_version": sdk_version,
            "controller_version": controller_version,
            "major_version_match": version_match,
            "dcss_size": ctypes.sizeof(Dcss),
            "realtime_frames_advancing": frames_advancing,
            "state": snapshot.to_dict(),
            "servo_errors": servo_errors,
            "healthy": not health_issues,
            "health_issues": health_issues,
            "read_only": True,
        }
        if args.as_json:
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        else:
            print(f"SDK library       : {sdk.library_path}")
            print(f"SDK version       : {sdk_version}")
            print(f"Controller version: {controller_version}")
            print(f"Version compatible: {result['major_version_match']}")
            print(f"Realtime counters : {'advancing' if frames_advancing else 'static'}")
            print(_arm_text(snapshot.left))
            print(_arm_text(snapshot.right))
            print("Servo errors left :", " ".join(map(str, servo_errors["left"])))
            print("Servo errors right:", " ".join(map(str, servo_errors["right"])))
            print("Health             :", "PASS" if not health_issues else "BLOCKED")
            for issue in health_issues:
                print("  -", issue)
            print("Probe mode        : READ ONLY")
    return 0 if not health_issues else 4


if __name__ == "__main__":
    raise SystemExit(main())
