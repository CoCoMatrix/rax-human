from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml

EXPECTED_ARMS = ("left_arm", "right_arm")
EXPECTED_CAMERAS = ("camera_head", "camera_left", "camera_right")
EXPECTED_FIRMWARE_DRIVERS = ("default", "v183", "v188", "v189")
CAN_SERIAL_RE = re.compile(r"^[0-9A-F]+$")


class ProfileError(ValueError):
    """The fixed dual-Piper profile is unsafe or internally inconsistent."""


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProfileError(f"{path} must be a mapping")
    return value


def _number_list(value: Any, path: str, length: int) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ProfileError(f"{path} must contain exactly {length} values")
    if any(
        not isinstance(item, (int, float)) or isinstance(item, bool) for item in value
    ):
        raise ProfileError(f"{path} must contain only numbers")
    return [float(item) for item in value]


def load_profile(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ProfileError("profile root must be a mapping")
    validate_profile(value)
    return value


def load_device_instance(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ProfileError("device instance root must be a mapping")
    return value


def validate_device_instance(
    profile: Mapping[str, Any], device_instance: Mapping[str, Any]
) -> None:
    """Validate the physical instance before opening either CAN channel."""
    validate_profile(profile)
    if device_instance.get("schema_version") != "xpolicy.robot_devices.v1":
        raise ProfileError("device instance schema_version mismatch")
    if device_instance.get("robot_type") != profile["robot_type"]:
        raise ProfileError("device instance robot_type mismatch")
    robot_id = device_instance.get("robot_id")
    if not isinstance(robot_id, str) or not robot_id:
        raise ProfileError("device instance robot_id must be non-empty")

    arms = _mapping(device_instance.get("arms"), "device_instance.arms")
    if tuple(arms) != EXPECTED_ARMS:
        raise ProfileError("device instance arms must be left_arm then right_arm")
    can_serials: set[str] = set()
    netdevs: set[str] = set()
    for role in EXPECTED_ARMS:
        arm = _mapping(arms[role], f"device_instance.arms.{role}")
        serial = str(arm.get("can_adapter_serial", "")).upper()
        netdev = arm.get("netdev")
        if not CAN_SERIAL_RE.fullmatch(serial):
            raise ProfileError(
                f"device_instance.arms.{role}.can_adapter_serial is invalid"
            )
        if not isinstance(netdev, str) or netdev != f"can_px_{serial[:8].lower()}":
            raise ProfileError(
                f"device_instance.arms.{role}.netdev does not match its CAN serial"
            )
        if arm.get("bitrate") != profile["sdk"]["can_bitrate"]:
            raise ProfileError(f"device_instance.arms.{role}.bitrate mismatch")
        if serial in can_serials or netdev in netdevs:
            raise ProfileError("device instance CAN bindings must be unique")
        can_serials.add(serial)
        netdevs.add(netdev)

        firmware = _mapping(
            arm.get("firmware"), f"device_instance.arms.{role}.firmware"
        )
        if firmware.get("sdk_driver") not in EXPECTED_FIRMWARE_DRIVERS:
            raise ProfileError(
                f"device_instance.arms.{role}.firmware.sdk_driver is invalid"
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
                raise ProfileError(
                    f"device_instance.arms.{role}.firmware.{field} must be non-empty"
                )

        end_effector = _mapping(
            arm.get("end_effector"), f"device_instance.arms.{role}.end_effector"
        )
        if (
            end_effector.get("type") != "agx_gripper"
            or end_effector.get("mode") != "width"
        ):
            raise ProfileError(f"device_instance.arms.{role}.end_effector mismatch")
        expected_gripper = profile["body"]["grippers"][
            f"{role.removesuffix('_arm')}_gripper"
        ]
        if float(end_effector.get("max_range_m", -1.0)) != float(
            expected_gripper["limits"][1]
        ):
            raise ProfileError(f"device_instance.arms.{role}.gripper range mismatch")

    cameras = _mapping(device_instance.get("cameras"), "device_instance.cameras")
    if tuple(cameras) != ("camera_left", "camera_head", "camera_right"):
        raise ProfileError("device instance cameras must be left, head, right")
    camera_serials: set[str] = set()
    camera_usb_serials: set[str] = set()
    for role in EXPECTED_CAMERAS:
        camera = _mapping(cameras.get(role), f"device_instance.cameras.{role}")
        serial = str(camera.get("serial", ""))
        if not serial.isdigit() or not serial:
            raise ProfileError(f"device_instance.cameras.{role}.serial is invalid")
        if serial in camera_serials:
            raise ProfileError("device instance camera bindings must be unique")
        camera_serials.add(serial)
        usb_serial = str(camera.get("usb_descriptor_serial", ""))
        if not usb_serial.isdigit() or not usb_serial:
            raise ProfileError(
                f"device_instance.cameras.{role}.usb_descriptor_serial is invalid"
            )
        if usb_serial in camera_usb_serials:
            raise ProfileError("device instance camera USB bindings must be unique")
        camera_usb_serials.add(usb_serial)
        if camera.get("model") != "Intel RealSense D435":
            raise ProfileError(f"device_instance.cameras.{role}.model mismatch")
        for field in ("firmware_version", "usb_type"):
            if not isinstance(camera.get(field), str) or not camera[field]:
                raise ProfileError(
                    f"device_instance.cameras.{role}.{field} must be non-empty"
                )


def validate_profile(profile: Mapping[str, Any]) -> None:
    if profile.get("profile_id") != "piper_x_dual_takeover_canary_v1":
        raise ProfileError("unexpected profile_id")
    if profile.get("robot_type") != "piper_x_dual":
        raise ProfileError("robot_type must be piper_x_dual")
    if profile.get("maturity") != "supervised_takeover_canary":
        raise ProfileError("profile maturity must be supervised_takeover_canary")
    forbidden_live_fields = {
        "state_schema",
        "action_schema",
        "action_frequency_hz",
        "data_collection",
    }
    present = sorted(forbidden_live_fields.intersection(profile))
    if present:
        raise ProfileError(f"inventory profile cannot declare live fields: {present}")

    device_instance = _mapping(profile.get("device_instance"), "device_instance")
    if device_instance.get("schema_version") != "xpolicy.robot_devices.v1":
        raise ProfileError("device_instance.schema_version mismatch")

    sdk = _mapping(profile.get("sdk"), "sdk")
    if sdk.get("bundled_root") != "third_party/robot_sdks/pyagxarm":
        raise ProfileError("sdk.bundled_root must select the repository SDK snapshot")
    if sdk.get("root_env") != "PIPER_X_SDK_ROOT":
        raise ProfileError("sdk.root_env must be PIPER_X_SDK_ROOT")
    if sdk.get("required_commit") != "8cd90f9106219a156c3c0d7e58ee36d838a89baf":
        raise ProfileError("sdk.required_commit does not match the bundled pyAgxArm")
    if sdk.get("robot_model") != "piper_x":
        raise ProfileError("sdk.robot_model must be piper_x")
    if sdk.get("firmware_driver_source") != "robot_instance":
        raise ProfileError("firmware driver must come from robot_instance")
    if tuple(sdk.get("allowed_firmware_drivers", ())) != EXPECTED_FIRMWARE_DRIVERS:
        raise ProfileError("sdk.allowed_firmware_drivers is incomplete or reordered")
    if sdk.get("can_interface") != "socketcan" or sdk.get("can_bitrate") != 1_000_000:
        raise ProfileError("Piper requires socketcan at 1000000 bps")
    for field in ("enable_timeout_seconds", "enable_retry_interval_seconds"):
        value = sdk.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise ProfileError(f"sdk.{field} must be positive")
    if float(sdk["enable_retry_interval_seconds"]) > float(
        sdk["enable_timeout_seconds"]
    ):
        raise ProfileError("SDK enable retry interval must not exceed its timeout")

    geometry = _mapping(profile.get("geometry"), "geometry")
    if geometry.get("root_frame") != "dual_base_link":
        raise ProfileError("geometry.root_frame must be dual_base_link")
    if float(geometry.get("arm_separation_m", -1.0)) != 0.6:
        raise ProfileError("geometry.arm_separation_m must be 0.60")
    if geometry.get("urdf_path") != "description/dual_piper_x_mink.urdf":
        raise ProfileError("geometry.urdf_path must select the repository Mink model")
    if geometry.get("urdf_kind") != "kinematics_only":
        raise ProfileError("Piper Mink URDF must remain explicitly kinematics-only")
    source_sha256 = geometry.get("urdf_source_sha256")
    if not isinstance(source_sha256, str) or not re.fullmatch(
        r"[0-9a-f]{64}", source_sha256
    ):
        raise ProfileError("geometry.urdf_source_sha256 must be a lowercase SHA256")
    end_effectors = _mapping(
        geometry.get("end_effector_links"), "geometry.end_effector_links"
    )
    if dict(end_effectors) != {
        "left_arm": "left_link6",
        "right_arm": "right_link6",
    }:
        raise ProfileError("geometry.end_effector_links must select both link6 frames")
    left_xyz = _number_list(
        geometry.get("left_base_xyz_m"), "geometry.left_base_xyz_m", 3
    )
    right_xyz = _number_list(
        geometry.get("right_base_xyz_m"), "geometry.right_base_xyz_m", 3
    )
    if left_xyz != [0.0, 0.3, 0.008] or right_xyz != [0.0, -0.3, 0.008]:
        raise ProfileError("arm bases must be separated along Y without mirroring")

    body = _mapping(profile.get("body"), "body")
    if tuple(body.get("arm_order", ())) != EXPECTED_ARMS:
        raise ProfileError("body.arm_order must be left_arm, right_arm")
    arms = _mapping(body.get("arms"), "body.arms")
    if tuple(arms) != EXPECTED_ARMS:
        raise ProfileError("body.arms must contain left_arm then right_arm")

    ordered_joints: list[str] = []
    for role in EXPECTED_ARMS:
        arm = _mapping(arms.get(role), f"body.arms.{role}")
        expected_names = [f"{role.removesuffix('_arm')}_joint{i}" for i in range(1, 7)]
        if arm.get("joint_names") != expected_names:
            raise ProfileError(f"body.arms.{role}.joint_names mismatch")
        if arm.get("sdk_joint_names") != [f"joint{i}" for i in range(1, 7)]:
            raise ProfileError(f"body.arms.{role}.sdk_joint_names mismatch")
        directions = _number_list(
            arm.get("direction"), f"body.arms.{role}.direction", 6
        )
        if directions != [1.0] * 6:
            raise ProfileError("dual Piper arms are translated, not mirrored")
        ordered_joints.extend(expected_names)

    limits = _mapping(body.get("joint_limits_rad"), "body.joint_limits_rad")
    if set(limits) != set(ordered_joints):
        raise ProfileError("joint_limits_rad must exactly cover both six-joint arms")
    for joint in ordered_joints:
        lower, upper = _number_list(limits[joint], f"body.joint_limits_rad.{joint}", 2)
        if lower >= upper:
            raise ProfileError(f"invalid joint limit ordering for {joint}")

    grippers = _mapping(body.get("grippers"), "body.grippers")
    if tuple(grippers) != ("left_gripper", "right_gripper"):
        raise ProfileError("body.grippers must contain left_gripper then right_gripper")
    for side in ("left", "right"):
        gripper = _mapping(grippers[f"{side}_gripper"], f"body.grippers.{side}_gripper")
        if gripper.get("arm") != f"{side}_arm" or gripper.get("unit") != "m":
            raise ProfileError(f"{side} gripper role/unit mismatch")
        if _number_list(
            gripper.get("limits"), f"body.grippers.{side}_gripper.limits", 2
        ) != [0.0, 0.1]:
            raise ProfileError(f"{side} gripper limits must be [0.0, 0.1] m")

    cameras = _mapping(profile.get("cameras"), "cameras")
    if tuple(cameras.get("required_roles", ())) != EXPECTED_CAMERAS:
        raise ProfileError("camera roles must be head, left, right")
    if cameras.get("bind_by") != "serial":
        raise ProfileError("cameras must bind by serial")
    startup = _mapping(cameras.get("startup"), "cameras.startup")
    max_attempts = startup.get("max_attempts")
    if (
        not isinstance(max_attempts, int)
        or isinstance(max_attempts, bool)
        or max_attempts < 1
    ):
        raise ProfileError("cameras.startup.max_attempts must be a positive integer")
    for field in ("retry_delay_seconds", "readiness_seconds"):
        value = startup.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise ProfileError(f"cameras.startup.{field} must be positive")

    probe = _mapping(profile.get("read_only_probe"), "read_only_probe")
    if (
        probe.get("require_both_arms") is not True
        or probe.get("require_advancing_frames") is not True
    ):
        raise ProfileError(
            "read_only_probe must require both arms and advancing frames"
        )
    for field in (
        "status_timeout_seconds",
        "minimum_arm_status_hz",
        "minimum_gripper_status_hz",
        "max_state_age_ms",
    ):
        value = probe.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise ProfileError(f"read_only_probe.{field} must be positive")
    skew = probe.get("max_inter_arm_skew_ms")
    if not isinstance(skew, (int, float)) or isinstance(skew, bool) or skew < 0:
        raise ProfileError("read_only_probe.max_inter_arm_skew_ms must be non-negative")
    gripper_tolerance = probe.get("gripper_feedback_tolerance_m")
    if (
        not isinstance(gripper_tolerance, (int, float))
        or isinstance(gripper_tolerance, bool)
        or not 0.0 <= gripper_tolerance <= 0.01
    ):
        raise ProfileError(
            "read_only_probe.gripper_feedback_tolerance_m must be in [0.0, 0.01]"
        )

    prepare = _mapping(profile.get("prepare"), "prepare")
    initial_pose = _mapping(prepare.get("initial_pose"), "prepare.initial_pose")
    if tuple(initial_pose) != (
        "left_arm",
        "right_arm",
        "left_gripper",
        "right_gripper",
    ):
        raise ProfileError("prepare.initial_pose must cover both arms and grippers")
    for role in EXPECTED_ARMS:
        values = _number_list(initial_pose.get(role), f"prepare.initial_pose.{role}", 6)
        names = body["arms"][role]["joint_names"]
        for name, value in zip(names, values, strict=True):
            lower, upper = limits[name]
            if not float(lower) <= value <= float(upper):
                raise ProfileError(f"prepare target is outside limits for {name}")
    for side in ("left", "right"):
        value = initial_pose.get(f"{side}_gripper")
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ProfileError(f"prepare.initial_pose.{side}_gripper must be numeric")
        if not 0.0 <= float(value) <= 0.1:
            raise ProfileError(f"prepare.initial_pose.{side}_gripper is outside limits")
    if prepare.get("command") != "move_j":
        raise ProfileError("Piper prepare must use the SDK-smoothed move_j command")
    if not isinstance(prepare.get("source"), str) or not prepare["source"]:
        raise ProfileError("prepare.source must record the vendor reference")
    for field in (
        "speed_percent",
        "position_tolerance_rad",
        "gripper_tolerance_m",
        "timeout_seconds",
    ):
        value = prepare.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise ProfileError(f"prepare.{field} must be positive")
    if not 1 <= int(prepare["speed_percent"]) <= 100:
        raise ProfileError("prepare.speed_percent must be in [1, 100]")

    transition = _mapping(profile.get("transition_pose"), "transition_pose")
    direct_transition_bound = transition.get("max_joint_delta_rad")
    if (
        not isinstance(direct_transition_bound, (int, float))
        or isinstance(direct_transition_bound, bool)
        or not 0.0 < float(direct_transition_bound) <= 3.141592653589793
    ):
        raise ProfileError("transition_pose.max_joint_delta_rad must be in (0, pi]")
    if not isinstance(transition.get("segmented_transition_enabled"), bool):
        raise ProfileError(
            "transition_pose.segmented_transition_enabled must be a boolean"
        )
    segmented_bound = transition.get("segmented_max_joint_delta_rad")
    if transition["segmented_transition_enabled"]:
        if (
            not isinstance(segmented_bound, (int, float))
            or isinstance(segmented_bound, bool)
            or not float(direct_transition_bound)
            < float(segmented_bound)
            <= 3.141592653589793
        ):
            raise ProfileError(
                "transition_pose.segmented_max_joint_delta_rad must exceed the "
                "direct bound and remain within pi"
            )
        max_waypoints = transition.get("max_segmented_waypoints")
        if (
            not isinstance(max_waypoints, int)
            or isinstance(max_waypoints, bool)
            or max_waypoints < 2
        ):
            raise ProfileError(
                "transition_pose.max_segmented_waypoints must be at least two"
            )
    home_fallback_enabled = transition.get("home_fallback_enabled")
    if not isinstance(home_fallback_enabled, bool):
        raise ProfileError(
            "transition_pose.home_fallback_enabled must be a boolean"
        )
    home_fallback_max_waypoints = transition.get("home_fallback_max_waypoints")
    if (
        not isinstance(home_fallback_max_waypoints, int)
        or isinstance(home_fallback_max_waypoints, bool)
        or home_fallback_max_waypoints < 2
    ):
        raise ProfileError(
            "transition_pose.home_fallback_max_waypoints must be at least two"
        )

    canary = _mapping(profile.get("write_canary"), "write_canary")
    if tuple(canary.get("arm_order", ())) != EXPECTED_ARMS:
        raise ProfileError("write_canary.arm_order must be left then right")
    if canary.get("installation_position") != "horizontal":
        raise ProfileError("Piper write canary requires horizontal installation")
    if canary.get("return_to_measured_start") is not True:
        raise ProfileError("Piper write canary must return to its measured start")
    if canary.get("test_joint_index") not in range(1, 7):
        raise ProfileError("write_canary.test_joint_index must be in [1, 6]")
    for field in (
        "speed_percent",
        "test_delta_rad",
        "cartesian_test_delta_m",
        "cartesian_position_tolerance_m",
        "cartesian_rotation_tolerance_rad",
        "gripper_test_delta_m",
        "gripper_position_tolerance_m",
        "gripper_force_n",
        "measured_state_limit_tolerance_rad",
        "position_tolerance_rad",
        "timeout_seconds",
    ):
        value = canary.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise ProfileError(f"write_canary.{field} must be positive")
    if not 1 <= int(canary["speed_percent"]) <= 100:
        raise ProfileError("write_canary.speed_percent must be in [1, 100]")
    if float(canary["test_delta_rad"]) > 0.05:
        raise ProfileError("write_canary.test_delta_rad must not exceed 0.05 rad")
    if canary.get("cartesian_test_axis") not in ("x", "y", "z"):
        raise ProfileError("write_canary.cartesian_test_axis must be x, y or z")
    if float(canary["cartesian_test_delta_m"]) > 0.01:
        raise ProfileError("write_canary Cartesian delta must not exceed 0.01 m")
    if float(canary["gripper_test_delta_m"]) > 0.01:
        raise ProfileError("write_canary gripper delta must not exceed 0.01 m")

    live = _mapping(profile.get("live_takeover"), "live_takeover")
    if live.get("qualification") != "supervised_external_takeover":
        raise ProfileError("live_takeover qualification mismatch")
    if live.get("command") != "move_j":
        raise ProfileError("live takeover must stream host-IK joint targets")
    if not isinstance(live.get("gripper_write_enabled"), bool):
        raise ProfileError("gripper_write_enabled must be a boolean qualification")
    control_hz = live.get("control_hz")
    if not isinstance(control_hz, int) or isinstance(control_hz, bool) or control_hz != 100:
        raise ProfileError("live_takeover.control_hz must match the 10 ms Dora timer")
    for field in (
        "speed_percent",
        "max_state_age_ms",
        "max_command_age_ms",
        "measured_state_limit_tolerance_rad",
    ):
        value = live.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise ProfileError(f"live_takeover.{field} must be positive")
    if not 1 <= int(live["speed_percent"]) <= 100:
        raise ProfileError("live_takeover.speed_percent must be in [1, 100]")
    if float(live["measured_state_limit_tolerance_rad"]) > 0.02:
        raise ProfileError("measured state limit tolerance must not exceed 0.02 rad")
    boundary = _mapping(
        live.get("model_boundary_normalization"),
        "live_takeover.model_boundary_normalization",
    )
    if boundary.get("enabled") is not True:
        raise ProfileError("model boundary normalization must be explicitly enabled")
    joint_boundary_tolerance = boundary.get("joint_tolerance_rad")
    if (
        not isinstance(joint_boundary_tolerance, (int, float))
        or isinstance(joint_boundary_tolerance, bool)
        or not 0.0 < float(joint_boundary_tolerance) <= 0.03
    ):
        raise ProfileError(
            "model joint boundary tolerance must be in (0, 0.03] rad"
        )
    if boundary.get("gripper_mode") != "clamp_physical_range":
        raise ProfileError(
            "model gripper boundary mode must be clamp_physical_range"
        )

    safety = _mapping(profile.get("safety"), "safety")
    for field in ("auto_clear_errors", "auto_enable", "auto_move", "write_enabled"):
        if safety.get(field) is not False:
            raise ProfileError(f"safety.{field} must be false in inventory profile")

    shadow = _mapping(profile.get("shadow_teleop"), "shadow_teleop")
    if shadow.get("qualification") != "supervised_takeover_canary":
        raise ProfileError("shadow teleop qualification mismatch")
    if shadow.get("arm_mapping") != "host_urdf_ik_joint_stream":
        raise ProfileError("Piper takeover must use host URDF IK joint streaming")
    if shadow.get("gripper_mapping") != "trigger_to_absolute_width":
        raise ProfileError("unexpected Piper shadow gripper mapping")
    if shadow.get("waist_pitch") != "disabled" or shadow.get("waist_yaw") != "disabled":
        raise ProfileError("Piper shadow teleop must disable both waist capabilities")
    if float(shadow.get("max_gripper_width_m", -1.0)) != 0.1:
        raise ProfileError(
            "Piper shadow gripper width must match the measured 0.1 m range"
        )
    if float(shadow.get("gripper_force_n", 0.0)) <= 0.0:
        raise ProfileError("Piper gripper force must be positive")
    sdk_cartesian = _mapping(
        shadow.get("sdk_cartesian"), "shadow_teleop.sdk_cartesian"
    )
    if sdk_cartesian.get("qualification") != "offline_firmware_canary_only":
        raise ProfileError("Piper SDK Cartesian path must remain offline-only")
    if sdk_cartesian.get("command") != "move_p":
        raise ProfileError("Piper Cartesian takeover must use SDK move_p")
    if sdk_cartesian.get("ik_feedback") != "get_ik_joint_angles":
        raise ProfileError("Piper Cartesian takeover must record firmware IK")
    if sdk_cartesian.get("flange_feedback") != "get_flange_pose":
        raise ProfileError("Piper Cartesian takeover requires flange feedback")
    for field in (
        "translation_scale",
        "rotation_scale",
        "translation_filter_alpha",
        "rotation_filter_alpha",
    ):
        value = sdk_cartesian.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise ProfileError(f"shadow_teleop.sdk_cartesian.{field} must be positive")
    translation_limit = sdk_cartesian.get("max_translation_m")
    if translation_limit is not None and (
        not isinstance(translation_limit, (int, float))
        or isinstance(translation_limit, bool)
        or translation_limit <= 0
    ):
        raise ProfileError(
            "shadow_teleop.sdk_cartesian.max_translation_m must be positive or null"
        )
    rotation_limit = sdk_cartesian.get("max_rotation_rad")
    if rotation_limit is not None and (
        not isinstance(rotation_limit, (int, float))
        or isinstance(rotation_limit, bool)
        or rotation_limit <= 0
    ):
        raise ProfileError(
            "shadow_teleop.sdk_cartesian.max_rotation_rad must be positive or null"
        )
    for field in ("translation_filter_alpha", "rotation_filter_alpha"):
        value = sdk_cartesian.get(field)
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not 0.0 < float(value) <= 1.0
        ):
            raise ProfileError(
                f"shadow_teleop.sdk_cartesian.{field} must be in (0, 1]"
            )
    workspace = _mapping(
        sdk_cartesian.get("workspace_xyz_m"),
        "shadow_teleop.sdk_cartesian.workspace_xyz_m",
    )
    if tuple(workspace) != ("x", "y", "z"):
        raise ProfileError("Piper Cartesian workspace must define x, y, z")
    for axis in ("x", "y", "z"):
        lower, upper = _number_list(
            workspace[axis],
            f"shadow_teleop.sdk_cartesian.workspace_xyz_m.{axis}",
            2,
        )
        if lower >= upper:
            raise ProfileError(f"invalid Piper Cartesian {axis} workspace")
    expected_identity = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    for side in ("left", "right"):
        rows = sdk_cartesian.get(f"axis_map_{side}")
        if rows != expected_identity:
            raise ProfileError(
                "Piper SDK axis maps must preserve translated arm frames"
            )

    # Host-side continuous IK contract.
    mink = _mapping(shadow.get("mink"), "shadow_teleop.mink")
    if mink.get("solver") != "daqp" or mink.get("control_hz") != control_hz:
        raise ProfileError("Piper live Mink must use the qualified solver cadence")
    iterations = mink.get("max_iterations")
    if (
        not isinstance(iterations, int)
        or isinstance(iterations, bool)
        or iterations < 1
    ):
        raise ProfileError("shadow_teleop.mink.max_iterations must be positive")
    for field in (
        "translation_scale",
        "rotation_scale",
        "position_tolerance_m",
        "orientation_tolerance_rad",
        "position_cost",
        "orientation_cost",
        "posture_cost",
        "lm_damping",
        "max_solution_jump_rad",
    ):
        value = mink.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise ProfileError(f"shadow_teleop.mink.{field} must be positive")
    translation_limit = mink.get("max_translation_m")
    if translation_limit is not None and (
        not isinstance(translation_limit, (int, float))
        or isinstance(translation_limit, bool)
        or translation_limit <= 0
    ):
        raise ProfileError(
            "shadow_teleop.mink.max_translation_m must be positive or null"
        )
    rotation_limit = mink.get("max_rotation_rad")
    if rotation_limit is not None and (
        not isinstance(rotation_limit, (int, float))
        or isinstance(rotation_limit, bool)
        or rotation_limit <= 0
    ):
        raise ProfileError(
            "shadow_teleop.mink.max_rotation_rad must be positive or null"
        )
    for field in ("translation_filter_alpha", "rotation_filter_alpha"):
        if float(mink[field]) > 1.0:
            raise ProfileError(f"shadow_teleop.mink.{field} must be at most 1")
    for side in ("left", "right"):
        rows = mink.get(f"axis_map_{side}")
        if not isinstance(rows, list) or len(rows) != 3:
            raise ProfileError(f"shadow_teleop.mink.axis_map_{side} must be 3x3")
        matrix = [
            _number_list(row, f"shadow_teleop.mink.axis_map_{side}", 3) for row in rows
        ]
        expected_identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        if matrix != expected_identity:
            raise ProfileError(
                "unqualified Piper axis maps must remain explicit identity"
            )


def validate_urdf(
    profile: Mapping[str, Any], urdf_path: Path, *, tolerance: float = 1e-6
) -> None:
    """Validate the external dual-Piper URDF without importing a robot runtime."""
    validate_profile(profile)
    root = ET.parse(urdf_path).getroot()
    geometry = _mapping(profile["geometry"], "geometry")
    if root.tag != "robot" or root.get("name") != geometry["expected_urdf_robot_name"]:
        raise ProfileError("URDF robot name mismatch")

    joints = {element.get("name"): element for element in root.findall("joint")}
    links = {element.get("name") for element in root.findall("link")}
    for role, name in geometry["end_effector_links"].items():
        if name not in links:
            raise ProfileError(f"URDF missing {role} endpoint link {name}")
    body = _mapping(profile["body"], "body")
    expected_limits = _mapping(body["joint_limits_rad"], "body.joint_limits_rad")
    for name, bounds in expected_limits.items():
        element = joints.get(name)
        if element is None or element.get("type") != "revolute":
            raise ProfileError(f"URDF missing revolute joint {name}")
        limit = element.find("limit")
        if limit is None:
            raise ProfileError(f"URDF joint {name} has no limit")
        actual = [float(limit.get("lower", "nan")), float(limit.get("upper", "nan"))]
        expected = [float(bounds[0]), float(bounds[1])]
        if any(abs(a - b) > tolerance for a, b in zip(actual, expected)):
            raise ProfileError(
                f"URDF/profile limits differ for {name}: {actual} != {expected}"
            )

    expected_mounts = {
        "left_mount_joint": geometry["left_base_xyz_m"],
        "right_mount_joint": geometry["right_base_xyz_m"],
    }
    for name, expected_xyz in expected_mounts.items():
        element = joints.get(name)
        origin = None if element is None else element.find("origin")
        if element is None or element.get("type") != "fixed" or origin is None:
            raise ProfileError(f"URDF missing fixed mount {name}")
        actual_xyz = [float(value) for value in origin.get("xyz", "").split()]
        if len(actual_xyz) != 3 or any(
            abs(a - float(b)) > tolerance for a, b in zip(actual_xyz, expected_xyz)
        ):
            raise ProfileError(f"URDF/profile mount differs for {name}")
