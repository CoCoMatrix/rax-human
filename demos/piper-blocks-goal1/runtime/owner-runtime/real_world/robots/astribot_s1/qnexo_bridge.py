from __future__ import annotations

from dataclasses import dataclass
import errno
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

from XPolicyLab.real_world.teleop import FootSwitch, QnexoButtonBank


DEFAULT_QNEXO_SOURCE = (
    Path(__file__).parents[3]
    / "runtime/qnexo/source/src"
)


@dataclass(frozen=True, slots=True)
class QnexoSample:
    payload: dict[str, Any]
    timestamp_ns: int
    operator_enabled: bool
    control_event: str | None = None
    control_button: str | None = None
    buttons: tuple[str, ...] = ()


def physical_button_name(logical_name: str) -> str:
    """Return the physical name preserved by the calibrated QNexo bridge."""
    return str(logical_name)


class QnexoEndEffectorSource:
    """Reuse QNexo serial, footswitch and exoskeleton FK without Dora."""

    def __init__(
        self,
        *,
        port: str = "/dev/qnbot",
        footswitch_device: str = (
            "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd"
        ),
        footswitch_key_code: int = 45,
        footswitch_toggle_on_press: bool = False,
        button_deadman: bool = False,
        qnexo_source: str | Path = DEFAULT_QNEXO_SOURCE,
        urdf_path: str | Path | None = None,
    ) -> None:
        source = str(Path(qnexo_source).resolve())
        if source not in sys.path:
            sys.path.insert(0, source)
        from qnexo.ee_kinematics import DEFAULT_EXO_URDF, ExoskeletonFK
        from qnexo.exo import ExoConnector, parse_joystick_data

        self._parse_joystick_data = parse_joystick_data
        self._connector = ExoConnector(port, queue_size=1)
        self._FootSwitch = FootSwitch
        self._footswitch_device = str(footswitch_device)
        self._footswitch_key_code = int(footswitch_key_code)
        self._footswitch_toggle_on_press = bool(footswitch_toggle_on_press)
        self._footswitch_retry_at = 0.0
        self._footswitch = (
            FootSwitch(
                footswitch_device,
                footswitch_key_code,
                toggle_on_press=self._footswitch_toggle_on_press,
            )
            if Path(footswitch_device).exists()
            else None
        )
        self._button_deadman = bool(button_deadman)
        if self._footswitch is None and not self._button_deadman:
            raise RuntimeError(
                "footswitch is missing; use button_deadman explicitly for "
                "the unassigned QNexo left-white hold button"
            )
        self._fk = ExoskeletonFK(urdf_path or DEFAULT_EXO_URDF)
        self._buttons = QnexoButtonBank()
        self._last_timestamp_ns = 0

    @classmethod
    def for_dagger(cls, **options: Any) -> QnexoEndEffectorSource:
        """Build the shared source with GR3-compatible click-toggle takeover."""
        options.setdefault("footswitch_toggle_on_press", True)
        return cls(**options)

    def _poll_footswitch(self) -> bool:
        """Fail closed and reopen the event fd after USB re-enumeration."""
        if self._footswitch is None:
            now = time.monotonic()
            if now < self._footswitch_retry_at:
                return False
            self._footswitch_retry_at = now + 0.25
            try:
                self._footswitch = self._FootSwitch(
                    self._footswitch_device,
                    self._footswitch_key_code,
                    toggle_on_press=self._footswitch_toggle_on_press,
                )
            except OSError as error:
                if error.errno in {errno.ENOENT, errno.ENODEV, errno.EIO}:
                    return False
                raise
        try:
            return self._footswitch.poll()
        except OSError as error:
            if error.errno not in {errno.ENOENT, errno.ENODEV, errno.EIO}:
                raise
            try:
                self._footswitch.close()
            except OSError:
                pass
            self._footswitch = None
            self._footswitch_retry_at = time.monotonic() + 0.25
            return False

    def start(self) -> None:
        if not self._connector.start():
            raise RuntimeError("could not open QNexo serial port")

    @staticmethod
    def _hand_vector(open_fraction: float) -> list[float]:
        # Matches qnexo.main.map_gripper_to_6dof's first finger convention.
        first_four = -2.0 + 2.0 * float(np.clip(open_fraction, 0.0, 1.0))
        return [first_four] * 4 + [0.0, -1.57]

    def poll(self) -> QnexoSample | None:
        data = self._connector.get_latest_data()
        if data is None:
            return None
        timestamp_ns = int(data.timestamp * 1e9)
        if timestamp_ns <= self._last_timestamp_ns:
            return None
        self._last_timestamp_ns = timestamp_ns
        left_pose, right_pose = self._fk.forward(
            np.asarray(data.arm_joint_left_rad[:7]),
            np.asarray(data.arm_joint_right_rad[:7]),
        )
        left = self._parse_joystick_data(*data.joystick_left)
        right = self._parse_joystick_data(*data.joystick_right)
        control = self._buttons.update(
            left["button"],
            right["button"],
            timestamp_ns=timestamp_ns,
        )
        operator_enabled = (
            self._poll_footswitch()
            if self._footswitch is not None or not self._button_deadman
            else "white" in left["button"]
        )
        if control is not None and (
            self._footswitch is not None
            and self._footswitch.reset_for_control_event(control.command)
        ):
            operator_enabled = False
        return QnexoSample(
            payload={
                "left_pose": left_pose.ravel().tolist(),
                "right_pose": right_pose.ravel().tolist(),
                "left_arm_joints": list(map(float, data.arm_joint_left_rad[:7])),
                "right_arm_joints": list(map(float, data.arm_joint_right_rad[:7])),
                "left_trigger": float(left["trigger"]),
                "right_trigger": float(right["trigger"]),
                "left_joystick_x": float(left["x"]),
                "left_joystick_y": float(left["y"]),
                "right_joystick_x": float(right["x"]),
                "right_joystick_y": float(right["y"]),
                "left_hand": self._hand_vector(left["trigger"]),
                "right_hand": self._hand_vector(right["trigger"]),
            },
            timestamp_ns=timestamp_ns,
            operator_enabled=operator_enabled,
            control_event=control.command if control is not None else None,
            control_button=(
                physical_button_name(control.button)
                if control is not None
                else None
            ),
            buttons=tuple(
                physical_button_name(button) for button in self._buttons.pressed
            ),
        )

    def close(self) -> None:
        self._connector.stop()
        if self._footswitch is not None:
            self._footswitch.close()

    def __enter__(self) -> QnexoEndEffectorSource:
        self.start()
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
