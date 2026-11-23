from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from .cartesian_retarget import pose6_matrix
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


def _arm_joints(snapshot: DualPiperSnapshot, role: str) -> np.ndarray:
    sample = snapshot.left if role == "left_arm" else snapshot.right
    return np.asarray(sample.joint_position_rad, dtype=float)


def bounded_canary_targets(
    measured: Any,
    limits: Any,
    *,
    joint_index: int,
    delta_rad: float,
    measured_limit_tolerance_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the small test and recoverable return targets from live feedback."""
    measured = np.asarray(measured, dtype=float)
    limits = np.asarray(limits, dtype=float)
    if measured.shape != (6,) or limits.shape != (6, 2):
        raise InventoryError("Piper canary requires six measured joints and limits")
    if not np.isfinite(measured).all() or not np.isfinite(limits).all():
        raise InventoryError("Piper canary state and limits must be finite")
    if joint_index not in range(1, 7):
        raise InventoryError("Piper canary joint index must be in [1, 6]")
    if delta_rad <= 0.0 or not np.isfinite(delta_rad):
        raise InventoryError("Piper canary delta must be positive")
    lower, upper = limits[:, 0], limits[:, 1]
    outside = np.maximum(lower - measured, measured - upper)
    if np.any(outside > measured_limit_tolerance_rad):
        raise InventoryError("measured Piper state is too far outside profile limits")
    return_target = np.clip(measured, lower, upper)
    test_target = return_target.copy()
    index = joint_index - 1
    positive = test_target[index] + delta_rad
    negative = test_target[index] - delta_rad
    if positive <= upper[index]:
        test_target[index] = positive
    elif negative >= lower[index]:
        test_target[index] = negative
    else:
        raise InventoryError("Piper canary joint has no room for the configured delta")
    return test_target, return_target


def wait_for_target(
    owner: ReadOnlyDualPiperOwner,
    role: str,
    target: Any,
    *,
    tolerance_rad: float,
    timeout_seconds: float,
) -> tuple[DualPiperSnapshot, float]:
    target = np.asarray(target, dtype=float)
    started = time.monotonic()
    time.sleep(0.25)
    last_state_error = ""
    while True:
        try:
            snapshot = owner.snapshot(now_s=time.time())
            error = float(np.max(np.abs(_arm_joints(snapshot, role) - target)))
            last_state_error = ""
            if error <= tolerance_rad:
                return snapshot, error
        except InventoryError as exc:
            # Mode/config changes can reset the SDK rolling Hz counter for one
            # sample. Do not send another command until feedback recovers.
            last_state_error = str(exc)
            error = float("inf")
        if time.monotonic() - started >= timeout_seconds:
            if last_state_error:
                raise InventoryError(
                    f"{role} target wait lost valid feedback: {last_state_error}"
                )
            raise InventoryError(f"{role} target timeout: max error {error:.6f} rad")
        time.sleep(0.05)


def wait_for_cartesian_target(
    session: DualPiperSdkSession,
    role: str,
    target: Any,
    *,
    position_tolerance_m: float,
    rotation_tolerance_rad: float,
    timeout_seconds: float,
) -> tuple[Any, float, float]:
    target_matrix = pose6_matrix(target, "cartesian_canary_target")
    started = time.monotonic()
    time.sleep(0.25)
    while True:
        feedback = session.read_cartesian_feedback(role)
        current = pose6_matrix(feedback.flange_pose, "cartesian_canary_feedback")
        position_error = float(
            np.linalg.norm(current[:3, 3] - target_matrix[:3, 3])
        )
        rotation_error = float(
            np.linalg.norm(
                Rotation.from_matrix(
                    current[:3, :3] @ target_matrix[:3, :3].T
                ).as_rotvec()
            )
        )
        if (
            position_error <= position_tolerance_m
            and rotation_error <= rotation_tolerance_rad
            and feedback.ik_joint_angles is not None
        ):
            return feedback, position_error, rotation_error
        if time.monotonic() - started >= timeout_seconds:
            raise InventoryError(
                f"{role} Cartesian target timeout: position={position_error:.6f} m, "
                f"rotation={rotation_error:.6f} rad, "
                f"ik_feedback={feedback.ik_joint_angles is not None}"
            )
        time.sleep(0.05)


def wait_for_gripper_target(
    owner: ReadOnlyDualPiperOwner,
    role: str,
    target_m: float,
    *,
    tolerance_m: float,
    timeout_seconds: float,
) -> tuple[DualPiperSnapshot, float]:
    started = time.monotonic()
    time.sleep(0.2)
    while True:
        snapshot = owner.snapshot(now_s=time.time())
        sample = snapshot.left if role == "left_arm" else snapshot.right
        error = abs(sample.gripper_width_m - target_m)
        if error <= tolerance_m:
            return snapshot, error
        if time.monotonic() - started >= timeout_seconds:
            raise InventoryError(
                f"{role} gripper target timeout: error={error:.6f} m"
            )
        time.sleep(0.05)


def run_canary(
    profile: dict[str, Any],
    device: dict[str, Any],
    *,
    role: str,
    mode: str,
) -> dict[str, Any]:
    canary = profile["write_canary"]
    if role not in tuple(canary["arm_order"]):
        raise InventoryError(f"unsupported Piper canary role: {role}")
    speed = int(canary["speed_percent"])
    tolerance = float(canary["position_tolerance_rad"])
    timeout = float(canary["timeout_seconds"])
    limits_by_name = profile["body"]["joint_limits_rad"]
    names = profile["body"]["arms"][role]["joint_names"]
    limits = np.asarray([limits_by_name[name] for name in names], dtype=float)

    session = DualPiperSdkSession(
        profile,
        device,
        supervised_write_authorized=True,
    )
    with session as owner:
        time.sleep(float(profile["read_only_probe"]["status_timeout_seconds"]))
        start_snapshot = owner.snapshot(now_s=time.time())
        measured = _arm_joints(start_snapshot, role)
        if mode == "step":
            target, return_target = bounded_canary_targets(
                measured,
                limits,
                joint_index=int(canary["test_joint_index"]),
                delta_rad=float(canary["test_delta_rad"]),
                measured_limit_tolerance_rad=float(
                    canary["measured_state_limit_tolerance_rad"]
                ),
            )
        elif mode == "prepare":
            target = np.asarray(profile["prepare"]["initial_pose"][role], dtype=float)
            return_target = None
            speed = int(profile["prepare"]["speed_percent"])
            tolerance = float(profile["prepare"]["position_tolerance_rad"])
            timeout = float(profile["prepare"]["timeout_seconds"])
        elif mode == "cartesian":
            start_feedback = session.read_cartesian_feedback(role)
            return_pose = np.asarray(start_feedback.flange_pose, dtype=float)
            target = return_pose.copy()
            axis = {"x": 0, "y": 1, "z": 2}[canary["cartesian_test_axis"]]
            delta = float(canary["cartesian_test_delta_m"])
            lower, upper = profile["shadow_teleop"]["sdk_cartesian"][
                "workspace_xyz_m"
            ][canary["cartesian_test_axis"]]
            target[axis] += delta if target[axis] + delta <= upper else -delta
            if not lower <= target[axis] <= upper:
                raise InventoryError("Piper Cartesian canary has no workspace margin")
            enabled = session.enable_arm_for_canary(role)
            session.move_arm_cartesian_for_canary(
                role, target, speed_percent=speed
            )
            reached, position_error, rotation_error = wait_for_cartesian_target(
                session,
                role,
                target,
                position_tolerance_m=float(
                    canary["cartesian_position_tolerance_m"]
                ),
                rotation_tolerance_rad=float(
                    canary["cartesian_rotation_tolerance_rad"]
                ),
                timeout_seconds=timeout,
            )
            session.move_arm_cartesian_for_canary(
                role, return_pose, speed_percent=speed
            )
            returned, return_position_error, return_rotation_error = (
                wait_for_cartesian_target(
                    session,
                    role,
                    return_pose,
                    position_tolerance_m=float(
                        canary["cartesian_position_tolerance_m"]
                    ),
                    rotation_tolerance_rad=float(
                        canary["cartesian_rotation_tolerance_rad"]
                    ),
                    timeout_seconds=timeout,
                )
            )
            return {
                "status": "PASS",
                "mode": mode,
                "robot_id": device["robot_id"],
                "profile_id": profile["profile_id"],
                "arm": role,
                "enabled_joints": list(enabled),
                "speed_percent": speed,
                "measured_start_pose": return_pose.tolist(),
                "target_pose": target.tolist(),
                "target_position_error_m": position_error,
                "target_rotation_error_rad": rotation_error,
                "target_ik_joint_angles_rad": list(reached.ik_joint_angles),
                "return_position_error_m": return_position_error,
                "return_rotation_error_rad": return_rotation_error,
                "return_ik_joint_angles_rad": list(returned.ik_joint_angles),
                "measured_final_pose": list(returned.flange_pose),
                "other_arm_command_count": 0,
                "gripper_command_count": 0,
            }
        elif mode == "gripper":
            sample = start_snapshot.left if role == "left_arm" else start_snapshot.right
            return_width = float(sample.gripper_width_m)
            delta = float(canary["gripper_test_delta_m"])
            target = return_width + delta if return_width + delta <= 0.1 else return_width - delta
            if not 0.0 <= target <= 0.1:
                raise InventoryError("Piper gripper canary has no range margin")
            enabled = session.enable_arm_for_canary(role)
            session.move_gripper_for_canary(
                role,
                target,
                force_n=float(canary["gripper_force_n"]),
            )
            reached, target_error = wait_for_gripper_target(
                owner,
                role,
                target,
                tolerance_m=float(canary["gripper_position_tolerance_m"]),
                timeout_seconds=timeout,
            )
            session.move_gripper_for_canary(
                role,
                return_width,
                force_n=float(canary["gripper_force_n"]),
            )
            returned, return_error = wait_for_gripper_target(
                owner,
                role,
                return_width,
                tolerance_m=float(canary["gripper_position_tolerance_m"]),
                timeout_seconds=timeout,
            )
            reached_sample = reached.left if role == "left_arm" else reached.right
            returned_sample = returned.left if role == "left_arm" else returned.right
            return {
                "status": "PASS",
                "mode": mode,
                "robot_id": device["robot_id"],
                "profile_id": profile["profile_id"],
                "arm": role,
                "enabled_joints": list(enabled),
                "measured_start_m": return_width,
                "target_m": target,
                "measured_target_m": reached_sample.gripper_width_m,
                "target_error_m": target_error,
                "measured_final_m": returned_sample.gripper_width_m,
                "return_error_m": return_error,
                "other_arm_command_count": 0,
                "arm_motion_command_count": 0,
            }
        else:
            raise InventoryError(
                "Piper canary mode must be step, prepare, cartesian or gripper"
            )

        enabled = session.enable_arm_for_canary(role)
        session.move_arm_for_canary(role, target, speed_percent=speed)
        reached, target_error = wait_for_target(
            owner,
            role,
            target,
            tolerance_rad=tolerance,
            timeout_seconds=timeout,
        )

        return_error = None
        final_snapshot = reached
        if return_target is not None and canary["return_to_measured_start"]:
            session.move_arm_for_canary(role, return_target, speed_percent=speed)
            final_snapshot, return_error = wait_for_target(
                owner,
                role,
                return_target,
                tolerance_rad=tolerance,
                timeout_seconds=timeout,
            )

        return {
            "status": "PASS",
            "mode": mode,
            "robot_id": device["robot_id"],
            "profile_id": profile["profile_id"],
            "arm": role,
            "enabled_joints": list(enabled),
            "speed_percent": speed,
            "measured_start_rad": measured.tolist(),
            "target_rad": target.tolist(),
            "target_max_error_rad": target_error,
            "return_target_rad": (
                None if return_target is None else return_target.tolist()
            ),
            "return_max_error_rad": return_error,
            "measured_final_rad": _arm_joints(final_snapshot, role).tolist(),
            "other_arm_command_count": 0,
            "gripper_command_count": 0,
        }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Supervised one-arm Piper X SDK write qualification"
    )
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--device-instance", type=Path)
    parser.add_argument("--arm", choices=("left_arm", "right_arm"), default="left_arm")
    parser.add_argument(
        "--mode",
        choices=("step", "prepare", "cartesian", "gripper"),
        default="step",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        profile = load_profile(args.profile.resolve())
        device_path = (
            args.device_instance.resolve()
            if args.device_instance is not None
            else REPOSITORY_ROOT / profile["device_instance"]["default_path"]
        )
        device = load_device_instance(device_path)
        validate_device_instance(profile, device)
        report = run_canary(profile, device, role=args.arm, mode=args.mode)
        print("PIPER_X_WRITE_CANARY " + json.dumps(report, sort_keys=True))
        return 0
    except (ProfileError, InventoryError, KeyError, OSError, ValueError) as exc:
        print(f"PIPER_X_WRITE_CANARY BLOCKED: {exc}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
