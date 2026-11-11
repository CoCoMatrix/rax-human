from __future__ import annotations

import math
import time
from collections import OrderedDict
from collections.abc import Mapping
from dataclasses import dataclass
from types import TracebackType
from typing import Any

from .inventory import ArmSample, InventoryError, ReadOnlyDualPiperOwner
from .leader_follower import LeaderJointFeedback

FIRMWARE_DRIVERS = ("default", "v183", "v188", "v189")


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.split("(", 1)[0]


@dataclass
class PiperSdkArmReader:
    """Read-only view over one pyAgxArm driver and AGX gripper."""

    role: str
    arm: Any
    gripper: Any
    minimum_arm_status_hz: float
    minimum_gripper_status_hz: float

    def read_sample(self) -> ArmSample:
        arm_status = self.arm.get_arm_status()
        joint_angles = self.arm.get_joint_angles()
        gripper_status = self.gripper.get_gripper_status()
        missing = [
            name
            for name, value in (
                ("arm_status", arm_status),
                ("joint_angles", joint_angles),
                ("gripper_status", gripper_status),
            )
            if value is None
        ]
        if missing:
            raise InventoryError(
                f"{self.role} missing SDK feedback: {', '.join(missing)}"
            )

        arm_hz = min(float(arm_status.hz), float(joint_angles.hz))
        gripper_hz = float(gripper_status.hz)
        if (
            not math.isfinite(arm_hz)
            or not math.isfinite(gripper_hz)
            or arm_hz < 0.0
            or gripper_hz < 0.0
        ):
            raise InventoryError(f"{self.role} SDK feedback rate is invalid")
        # pyAgxArm's ``hz`` is a diagnostic count from the most recently
        # completed 100 ms bucket. Sampling exactly as that bucket rolls over
        # can legitimately expose 0 Hz while newer timestamped frames are
        # already cached. Do not turn that estimator race into a robot HOLD.
        # ReadOnlyDualPiperOwner gates the actual arm/gripper CAN timestamps
        # against max_state_age_ms, so genuinely stopped feedback still fails
        # closed within the configured 100 ms deadline.
        gripper_mode = str(getattr(gripper_status.msg, "mode", ""))
        if gripper_mode != "width":
            raise InventoryError(
                f"{self.role} gripper feedback mode is {gripper_mode or 'missing'}, "
                "expected width"
            )

        return ArmSample(
            role=self.role,
            joint_position_rad=tuple(float(value) for value in joint_angles.msg),
            gripper_width_m=float(gripper_status.msg.value),
            arm_timestamp_s=min(
                float(arm_status.timestamp), float(joint_angles.timestamp)
            ),
            gripper_timestamp_s=float(gripper_status.timestamp),
            arm_status_hz=arm_hz,
            gripper_status_hz=gripper_hz,
            controller_mode=_enum_name(arm_status.msg.ctrl_mode),
            controller_status=_enum_name(arm_status.msg.arm_status),
        )


@dataclass(frozen=True, slots=True)
class DualPiperDispatch:
    left_timestamp_ns: int
    right_timestamp_ns: int

    @property
    def skew_ms(self) -> float:
        return abs(self.right_timestamp_ns - self.left_timestamp_ns) / 1_000_000.0


@dataclass(frozen=True, slots=True)
class PiperCartesianFeedback:
    role: str
    flange_pose: tuple[float, ...]
    flange_timestamp_s: float
    flange_hz: float
    ik_joint_angles: tuple[float, ...] | None
    ik_timestamp_s: float | None
    ik_hz: float | None


def _pyagx_firmware(value: str, piper_fw: Any) -> Any:
    mapping = {
        "default": piper_fw.DEFAULT,
        "v183": piper_fw.V183,
        "v188": piper_fw.V188,
        "v189": piper_fw.V189,
    }
    try:
        return mapping[value]
    except KeyError as exc:
        raise InventoryError(f"unsupported Piper firmware driver: {value}") from exc


def create_read_only_sdk_arm(*, channel: str, firmware_driver: str) -> tuple[Any, Any]:
    """Create and passively connect one SDK reader; no control API is called."""
    try:
        from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
    except ImportError as exc:
        raise InventoryError(
            "pyAgxArm is unavailable; run `uv sync --extra piper-x-dual` to "
            "install third_party/robot_sdks/pyagxarm"
        ) from exc

    config = create_agx_arm_config(
        robot=ArmModel.PIPER_X,
        firmeware_version=_pyagx_firmware(firmware_driver, PiperFW),
        interface="socketcan",
        channel=channel,
        bitrate=1_000_000,
    )
    arm = AgxArmFactory.create_arm(config)
    gripper = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
    arm.connect()
    return arm, gripper


def disconnect_read_only_sdk_arm(arm: Any) -> None:
    # pyAgxArm's receive timeout is 1 s. A longer join avoids closing the CAN
    # socket while its reader thread is still inside recv().
    arm.disconnect(join_timeout=2.5)


class DualPiperSdkSession:
    """Own both SDK connections; writes require an explicit supervised session."""

    def __init__(
        self,
        profile: Mapping[str, Any],
        device_instance: Mapping[str, Any],
        *,
        supervised_write_authorized: bool = False,
    ) -> None:
        probe = profile["read_only_probe"]
        self._profile = profile
        self._device_instance = device_instance
        self._minimum_arm_hz = float(probe["minimum_arm_status_hz"])
        self._minimum_gripper_hz = float(probe["minimum_gripper_status_hz"])
        self._max_state_age_ms = float(probe["max_state_age_ms"])
        self._max_inter_arm_skew_ms = float(probe["max_inter_arm_skew_ms"])
        self._gripper_feedback_tolerance_m = float(
            probe.get("gripper_feedback_tolerance_m", 0.0)
        )
        sdk = profile["sdk"]
        self._enable_timeout_s = float(sdk["enable_timeout_seconds"])
        self._enable_retry_interval_s = float(
            sdk["enable_retry_interval_seconds"]
        )
        self._arms: OrderedDict[str, Any] = OrderedDict()
        self._grippers: OrderedDict[str, Any] = OrderedDict()
        self._supervised_write_authorized = bool(supervised_write_authorized)
        self._enabled_canary_arms: set[str] = set()
        self._live_control_enabled = False
        self._live_speed_percent: int | None = None
        self._live_motion_mode: str | None = None

    def __enter__(self) -> ReadOnlyDualPiperOwner:
        readers: OrderedDict[str, PiperSdkArmReader] = OrderedDict()
        try:
            for role in ("left_arm", "right_arm"):
                arm_config = self._device_instance["arms"][role]
                arm, gripper = create_read_only_sdk_arm(
                    channel=str(arm_config["netdev"]),
                    firmware_driver=str(arm_config["firmware"]["sdk_driver"]),
                )
                self._arms[role] = arm
                self._grippers[role] = gripper
                readers[role] = PiperSdkArmReader(
                    role=role,
                    arm=arm,
                    gripper=gripper,
                    minimum_arm_status_hz=self._minimum_arm_hz,
                    minimum_gripper_status_hz=self._minimum_gripper_hz,
                )
        except Exception:
            self.close()
            raise
        return ReadOnlyDualPiperOwner(
            readers,
            max_state_age_ms=self._max_state_age_ms,
            max_inter_arm_skew_ms=self._max_inter_arm_skew_ms,
            gripper_feedback_tolerance_m=self._gripper_feedback_tolerance_m,
        )

    def _canary_arm(self, role: str) -> Any:
        if not self._supervised_write_authorized:
            raise InventoryError("Piper SDK write was not authorized for this session")
        return self._connected_arm(role)

    def _connected_arm(self, role: str) -> Any:
        if role not in ("left_arm", "right_arm"):
            raise InventoryError(f"unknown Piper arm role: {role}")
        try:
            return self._arms[role]
        except KeyError as exc:
            raise InventoryError("Piper SDK session is not connected") from exc

    def _enable_arm(self, role: str) -> tuple[bool, ...]:
        arm = self._canary_arm(role)
        enabled: tuple[bool, ...] = ()
        enable_attempts = 0
        started_at = time.monotonic()
        deadline = started_at + self._enable_timeout_s
        next_enable_at = started_at
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_enable_at:
                # The pinned SDK's return value samples the feedback bits
                # immediately after sending. Those bits can still be false
                # while the controller is processing the request, so retry
                # the vendor command and gate on feedback below.
                arm.enable()
                enable_attempts += 1
                next_enable_at = now + self._enable_retry_interval_s
            values = arm.get_joints_enable_status_list()
            if values is not None:
                enabled = tuple(bool(value) for value in values)
                if len(enabled) == 6 and all(enabled):
                    break
            time.sleep(0.02)
        if len(enabled) != 6 or not all(enabled):
            netdev = str(self._device_instance["arms"][role]["netdev"])
            feedback_hz: float | None = None
            get_fps = getattr(arm, "get_fps", None)
            if callable(get_fps):
                try:
                    feedback_hz = float(get_fps())
                except (TypeError, ValueError):
                    feedback_hz = None
            if feedback_hz is not None and feedback_hz <= 0.0:
                raise InventoryError(
                    f"{role} received no CAN feedback on {netdev} while enabling; "
                    "check robot power, emergency stop, and CAN cable"
                )
            raise InventoryError(
                f"{role} did not report all six joints enabled after "
                f"{self._enable_timeout_s:.1f}s/{enable_attempts} requests: {enabled}"
            )
        arm.set_joint_limits_enabled(True)
        arm.set_installation_pos(arm.OPTIONS.INSTALLATION_POS.HORIZONTAL)
        time.sleep(0.3)
        return enabled

    def release_software_stop(self, role: str) -> dict[str, Any]:
        """Send one vendor 0x150 resume request to a disabled arm only.

        This maintenance operation never enables motors or sends pose/gripper
        targets. A NORMAL reply confirms observed status, not an ACK that a
        previously unreported emergency stop was cleared.
        """
        arm = self._canary_arm(role)
        if self._live_control_enabled or self._enabled_canary_arms:
            raise InventoryError("software-stop release requires a maintenance session")

        def fresh_status() -> Any:
            status = arm.get_arm_status()
            if status is None:
                raise InventoryError(f"{role} missing arm status before software-stop release")
            age_ms = (time.time() - float(status.timestamp)) * 1000.0
            if not math.isfinite(age_ms) or not 0 <= age_ms <= self._max_state_age_ms:
                raise InventoryError(f"{role} stale arm status during software-stop release")
            return status

        before = fresh_status()
        before_status = int(before.msg.arm_status)
        before_mode = _enum_name(before.msg.ctrl_mode)
        if before_status not in (0, 1):
            raise InventoryError(f"{role} has another controller fault: {before_status}")
        bits = arm.get_joints_enable_status_list()
        if bits is None or len(bits) != 6 or any(bits):
            raise InventoryError(f"{role} software-stop release requires six disabled joints")
        # Check individual driver cache timestamps: SDK False can also mean
        # missing feedback and must not be treated as a disabled motor.
        for index in range(1, 7):
            driver = arm.get_driver_states(index)
            age_ms = float("inf") if driver is None else (
                time.time() - float(driver.timestamp)
            ) * 1000.0
            if not math.isfinite(age_ms) or not 0 <= age_ms <= self._max_state_age_ms:
                raise InventoryError(f"{role} joint {index} missing/stale driver feedback")
            if driver.msg.foc_status.driver_enable_status:
                raise InventoryError(f"{role} joint {index} is enabled")
        sent_at = time.time()
        arm.reset()  # Pinned pyAgxArm: ArmMsgMotionCtrl(2), CAN 0x150.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            after = fresh_status()
            if float(after.timestamp) > sent_at and int(after.msg.arm_status) == 0:
                return {
                    "role": role,
                    "resume_requests": 1,
                    "before_arm_status": before_status,
                    "before_control_mode": before_mode,
                    "after_arm_status": int(after.msg.arm_status),
                    "after_control_mode": _enum_name(after.msg.ctrl_mode),
                    "after_timestamp_s": float(after.timestamp),
                    "joint_enable_status": list(arm.get_joints_enable_status_list()),
                    "enable_requests": 0,
                    "motion_targets": 0,
                }
            time.sleep(0.02)
        raise InventoryError(f"{role} software-stop release sent once; NORMAL feedback not confirmed")

    def enable_arm_for_canary(self, role: str) -> tuple[bool, ...]:
        """Enable exactly one arm after installing the SDK's safety settings."""
        self._canary_arm(role)
        other = "right_arm" if role == "left_arm" else "left_arm"
        if other in self._enabled_canary_arms:
            raise InventoryError("supervised canary permits only one enabled arm")
        # SDK 1.0.0 sends the enable frame and immediately samples feedback;
        # its boolean can therefore be false even when the controller enables
        # a few milliseconds later. The six feedback bits are authoritative.
        enabled = self._enable_arm(role)
        self._enabled_canary_arms.add(role)
        return enabled

    def enable_for_live_control(self) -> dict[str, tuple[bool, ...]]:
        """Enable both arms for the formal supervised owner session."""
        if not self._supervised_write_authorized:
            raise InventoryError("Piper SDK write was not authorized for this session")
        enabled = {role: self._enable_arm(role) for role in ("left_arm", "right_arm")}
        # Live commands select speed/mode through the cached helper below.
        # Disabling the SDK's automatic mode switch prevents move_p from
        # resetting the firmware trajectory planner on every 20 ms target.
        for arm in self._arms.values():
            arm.set_auto_set_motion_mode_enabled(False)
        self._live_speed_percent = None
        self._live_motion_mode = None
        self._live_control_enabled = True
        return enabled

    def configure_leader_follower(
        self,
        *,
        leader_role: str,
        follower_role: str,
    ) -> None:
        """Put one connected Piper pair into the vendor LF modes.

        This is deliberately owned by the existing dual-Piper owner.  No
        second process may open CAN or send mode frames.  The caller must have
        explicitly authorized live writes and enabled both arms first.
        """

        if leader_role == follower_role:
            raise InventoryError("leader and follower roles must be different")
        if leader_role not in ("left_arm", "right_arm") or follower_role not in (
            "left_arm",
            "right_arm",
        ):
            raise InventoryError("leader/follower roles must be left_arm or right_arm")
        self._canary_arm(leader_role).set_leader_mode()
        self._canary_arm(follower_role).set_follower_mode()

    def read_leader_joint_feedback(
        self, role: str
    ) -> LeaderJointFeedback | None:
        """Read the vendor leader-angle stream from one connected CAN channel."""

        arm = self._connected_arm(role)
        getter = getattr(arm, "get_leader_joint_angles", None)
        if not callable(getter):
            raise InventoryError(
                "bundled pyAgxArm driver does not expose get_leader_joint_angles()"
            )
        message = getter()
        if message is None:
            return None
        try:
            joints = tuple(float(value) for value in message.msg)
            timestamp_s = float(message.timestamp)
            hz = float(message.hz)
        except (AttributeError, TypeError, ValueError) as exc:
            raise InventoryError("leader joint feedback is malformed") from exc
        try:
            return LeaderJointFeedback(joints, timestamp_s, hz)
        except ValueError as exc:
            raise InventoryError(str(exc)) from exc

    def read_leader_gripper_width(self, role: str) -> tuple[float, float] | None:
        """Read the width feedback used for the follower gripper target."""

        status = self._grippers[role].get_gripper_status()
        if status is None:
            return None
        try:
            value = float(status.msg.value)
            timestamp_s = float(status.timestamp)
        except (AttributeError, TypeError, ValueError) as exc:
            raise InventoryError("leader gripper feedback is malformed") from exc
        if not math.isfinite(value) or not math.isfinite(timestamp_s):
            raise InventoryError("leader gripper feedback is non-finite")
        return value, timestamp_s

    def _configure_live_motion(
        self,
        mode: str,
        speed_percent: int,
        *,
        refresh_mode: bool = False,
    ) -> None:
        if not self._live_control_enabled:
            raise InventoryError("both Piper arms must be enabled for live control")
        if (
            not isinstance(speed_percent, int)
            or isinstance(speed_percent, bool)
            or not 1 <= speed_percent <= 100
        ):
            raise InventoryError("Piper live speed_percent must be in [1, 100]")
        if mode not in ("j", "p"):
            raise InventoryError(f"unsupported Piper live motion mode: {mode}")
        if self._live_speed_percent != speed_percent:
            for arm in self._arms.values():
                arm.set_speed_percent(speed_percent)
            self._live_speed_percent = speed_percent
        if refresh_mode or self._live_motion_mode != mode:
            for arm in self._arms.values():
                arm.set_motion_mode(
                    arm.OPTIONS.MOTION_MODE.J
                    if mode == "j"
                    else arm.OPTIONS.MOTION_MODE.P
                )
            self._live_motion_mode = mode

    def _validated_target(self, role: str, target_rad: Any) -> tuple[float, ...]:
        try:
            values = tuple(float(value) for value in target_rad)
        except (TypeError, ValueError) as exc:
            raise InventoryError("Piper target must be a numeric sequence") from exc
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise InventoryError("Piper target must contain six finite joints")
        names = self._profile["body"]["arms"][role]["joint_names"]
        limits = self._profile["body"]["joint_limits_rad"]
        for name, value in zip(names, values, strict=True):
            lower, upper = map(float, limits[name])
            if not lower <= value <= upper:
                raise InventoryError(f"Piper target is outside {name} limits")
        return values

    def move_arm_for_canary(
        self,
        role: str,
        target_rad: Any,
        *,
        speed_percent: int,
    ) -> tuple[float, ...]:
        """Send one SDK-smoothed joint target to the currently qualified arm."""
        arm = self._canary_arm(role)
        if role not in self._enabled_canary_arms:
            raise InventoryError(
                f"{role} must be enabled through the canary gate first"
            )
        if (
            not isinstance(speed_percent, int)
            or isinstance(speed_percent, bool)
            or not 1 <= speed_percent <= 100
        ):
            raise InventoryError("Piper canary speed_percent must be in [1, 100]")
        values = self._validated_target(role, target_rad)
        arm.set_speed_percent(speed_percent)
        arm.move_j(list(values))
        return values

    def move_arm_cartesian_for_canary(
        self,
        role: str,
        pose: Any,
        *,
        speed_percent: int,
    ) -> tuple[float, ...]:
        """Send one bounded flange target through the qualified arm's move_p."""
        arm = self._canary_arm(role)
        if role not in self._enabled_canary_arms:
            raise InventoryError(
                f"{role} must be enabled through the canary gate first"
            )
        if (
            not isinstance(speed_percent, int)
            or isinstance(speed_percent, bool)
            or not 1 <= speed_percent <= 100
        ):
            raise InventoryError("Piper canary speed_percent must be in [1, 100]")
        values = self._validated_pose(role, pose)
        arm.set_speed_percent(speed_percent)
        arm.move_p(list(values))
        return values

    @staticmethod
    def _validated_gripper_width(value: Any) -> float:
        try:
            width = float(value)
        except (TypeError, ValueError) as exc:
            raise InventoryError("Piper gripper target must be numeric") from exc
        if not math.isfinite(width) or not 0.0 <= width <= 0.1:
            raise InventoryError("Piper gripper target must be in [0.0, 0.1] m")
        return width

    def move_gripper_for_canary(
        self,
        role: str,
        width_m: Any,
        *,
        force_n: float,
    ) -> float:
        self._canary_arm(role)
        if role not in self._enabled_canary_arms:
            raise InventoryError(
                f"{role} must be enabled through the canary gate first"
            )
        width = self._validated_gripper_width(width_m)
        force = float(force_n)
        if not math.isfinite(force) or force <= 0.0:
            raise InventoryError("Piper gripper force must be positive")
        self._grippers[role].move_gripper_m(value=width, force=force)
        return width

    def initialize_gripper_width_mode(
        self,
        role: str,
        width_m: Any,
        *,
        force_n: float,
        timeout_s: float = 5.0,
        tolerance_m: float = 0.003,
    ) -> dict[str, float | str]:
        """Select width mode with one supervised move, then verify feedback."""
        self._canary_arm(role)
        width = self._validated_gripper_width(width_m)
        force = float(force_n)
        timeout = float(timeout_s)
        tolerance = float(tolerance_m)
        if not math.isfinite(force) or force <= 0.0:
            raise InventoryError("Piper gripper force must be positive")
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise InventoryError("Piper gripper initialization timeout must be positive")
        if not math.isfinite(tolerance) or tolerance < 0.0:
            raise InventoryError(
                "Piper gripper initialization tolerance must be non-negative"
            )

        gripper = self._grippers[role]
        feedback_deadline = time.monotonic() + timeout
        initial = gripper.get_gripper_status()
        while initial is None and time.monotonic() < feedback_deadline:
            time.sleep(0.02)
            initial = gripper.get_gripper_status()
        if initial is None:
            raise InventoryError(
                f"{role} missing gripper feedback before initialization after "
                f"{timeout:.3f} s"
            )
        initial_mode = str(getattr(initial.msg, "mode", ""))
        initial_value = float(initial.msg.value)
        initial_timestamp = float(initial.timestamp)
        if not math.isfinite(initial_value):
            raise InventoryError(f"{role} gripper feedback is non-finite")
        age = time.time() - initial_timestamp
        if not math.isfinite(age) or not 0 <= age <= self._max_state_age_ms / 1000:
            raise InventoryError(f"{role} gripper feedback is stale before initialization")

        # pyAgxArm has no mode-only selector. Its documented width command both
        # selects width feedback and moves the gripper to the requested stroke.
        gripper.move_gripper_m(value=width, force=force)
        deadline = time.monotonic() + timeout
        last_mode = initial_mode
        last_value = initial_value
        while time.monotonic() < deadline:
            status = gripper.get_gripper_status()
            if status is not None:
                last_mode = str(getattr(status.msg, "mode", ""))
                last_value = float(status.msg.value)
                if (
                    float(status.timestamp) > initial_timestamp
                    and last_mode == "width"
                    and math.isfinite(last_value)
                    and abs(last_value - width) <= tolerance
                ):
                    return {
                        "role": role,
                        "initial_mode": initial_mode or "missing",
                        "initial_value": initial_value,
                        "mode": last_mode,
                        "width_m": last_value,
                        "target_width_m": width,
                        "force_n": force,
                    }
            time.sleep(0.02)
        raise InventoryError(
            f"{role} gripper width initialization timed out: "
            f"mode={last_mode or 'missing'} value={last_value!r} "
            f"target={width:.6f} m"
        )

    def move_dual_grippers_for_live_control(
        self,
        left_width_m: Any,
        right_width_m: Any,
        *,
        force_n: float,
    ) -> DualPiperDispatch:
        if not self._live_control_enabled:
            raise InventoryError("both Piper arms must be enabled for live control")
        left = float(left_width_m)
        right = float(right_width_m)
        force = float(force_n)
        if not math.isfinite(force) or force <= 0.0:
            raise InventoryError("Piper gripper force must be positive")
        self._grippers["left_arm"].move_gripper_m(value=left, force=force)
        left_ns = time.time_ns()
        self._grippers["right_arm"].move_gripper_m(value=right, force=force)
        right_ns = time.time_ns()
        return DualPiperDispatch(left_ns, right_ns)

    def move_single_arm_for_live_control(
        self,
        role: str,
        target_rad: Any,
        *,
        speed_percent: int,
    ) -> DualPiperDispatch:
        """Send one arm target while leaving the other arm untouched.

        Leader/follower feedback uses this path so a physically draggable
        leader is never overwritten by a repeated measured-pose command.
        """

        if not self._live_control_enabled:
            raise InventoryError("both Piper arms must be enabled for live control")
        target = self._validated_target(role, target_rad)
        self._configure_live_motion("j", speed_percent, refresh_mode=True)
        self._arms[role].move_j(list(target))
        timestamp_ns = time.time_ns()
        return DualPiperDispatch(timestamp_ns, timestamp_ns)

    def move_single_gripper_for_live_control(
        self,
        role: str,
        width_m: Any,
        *,
        force_n: float,
    ) -> float:
        """Send one gripper width without touching the paired arm."""

        if not self._live_control_enabled:
            raise InventoryError("both Piper arms must be enabled for live control")
        width = self._validated_gripper_width(width_m)
        force = float(force_n)
        if not math.isfinite(force) or force <= 0.0:
            raise InventoryError("Piper gripper force must be positive")
        self._grippers[role].move_gripper_m(value=width, force=force)
        return width

    def move_dual_for_live_control(
        self,
        left_target_rad: Any,
        right_target_rad: Any,
        *,
        speed_percent: int,
    ) -> DualPiperDispatch:
        """Send the owner-selected joint targets unchanged to both arms."""
        left = tuple(map(float, left_target_rad))
        right = tuple(map(float, right_target_rad))
        # Match AgileX's QuestVR streaming order: send the J/CAN mode frame
        # immediately before each joint target batch. With SDK automatic mode
        # switching disabled, caching this frame can leave the controller
        # accepting gripper traffic while ignoring later arm targets.
        self._configure_live_motion("j", speed_percent, refresh_mode=True)
        self._arms["left_arm"].move_j(list(left))
        left_ns = time.time_ns()
        self._arms["right_arm"].move_j(list(right))
        right_ns = time.time_ns()
        return DualPiperDispatch(left_ns, right_ns)

    def _validated_pose(self, role: str, pose: Any) -> tuple[float, ...]:
        try:
            values = tuple(float(value) for value in pose)
        except (TypeError, ValueError) as exc:
            raise InventoryError("Piper Cartesian target must be numeric") from exc
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise InventoryError("Piper Cartesian target must contain six finite values")
        workspace = self._profile["shadow_teleop"]["sdk_cartesian"][
            "workspace_xyz_m"
        ]
        for axis, value in zip(("x", "y", "z"), values[:3], strict=True):
            lower, upper = map(float, workspace[axis])
            if not lower <= value <= upper:
                raise InventoryError(
                    f"{role} Cartesian target is outside {axis} workspace"
                )
        if not -math.pi <= values[3] <= math.pi:
            raise InventoryError(f"{role} Cartesian roll is outside [-pi, pi]")
        if not -math.pi / 2 <= values[4] <= math.pi / 2:
            raise InventoryError(f"{role} Cartesian pitch is outside [-pi/2, pi/2]")
        if not -math.pi <= values[5] <= math.pi:
            raise InventoryError(f"{role} Cartesian yaw is outside [-pi, pi]")
        return values

    def move_dual_cartesian_for_live_control(
        self,
        left_pose: Any,
        right_pose: Any,
        *,
        speed_percent: int,
    ) -> DualPiperDispatch:
        """Atomically validate, then send both targets through firmware move_p IK."""
        left = self._validated_pose("left_arm", left_pose)
        right = self._validated_pose("right_arm", right_pose)
        self._configure_live_motion("p", speed_percent)
        self._arms["left_arm"].move_p(list(left))
        left_ns = time.time_ns()
        self._arms["right_arm"].move_p(list(right))
        right_ns = time.time_ns()
        return DualPiperDispatch(left_ns, right_ns)

    def read_cartesian_feedback(self, role: str) -> PiperCartesianFeedback:
        arm = self._connected_arm(role)
        flange = arm.get_flange_pose()
        if flange is None:
            raise InventoryError(f"{role} missing flange pose feedback")
        flange_pose = tuple(float(value) for value in flange.msg)
        if len(flange_pose) != 6 or not all(
            math.isfinite(value) for value in flange_pose
        ):
            raise InventoryError(f"{role} flange pose feedback is invalid")
        ik = arm.get_ik_joint_angles()
        ik_joints = None if ik is None else tuple(float(value) for value in ik.msg)
        if ik_joints is not None and (
            len(ik_joints) != 6
            or not all(math.isfinite(value) for value in ik_joints)
        ):
            raise InventoryError(f"{role} firmware IK feedback is invalid")
        return PiperCartesianFeedback(
            role=role,
            flange_pose=flange_pose,
            flange_timestamp_s=float(flange.timestamp),
            flange_hz=float(flange.hz),
            ik_joint_angles=ik_joints,
            ik_timestamp_s=None if ik is None else float(ik.timestamp),
            ik_hz=None if ik is None else float(ik.hz),
        )

    def close(self) -> None:
        self._enabled_canary_arms.clear()
        self._live_control_enabled = False
        self._live_speed_percent = None
        self._live_motion_mode = None
        for role in reversed(tuple(self._arms)):
            arm = self._arms.pop(role)
            self._grippers.pop(role, None)
            disconnect_read_only_sdk_arm(arm)

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
