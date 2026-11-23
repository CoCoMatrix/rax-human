from __future__ import annotations

import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ARM_NAMES,
    GRIPPER_NAMES,
    PART_DOFS,
    PART_NAMES,
    arm_command_lists,
    arm_joint_action_dict,
    arm_joint_command_lists,
    arm_joint_gripper_command_lists,
    auxiliary_action_dict,
    command_lists,
    gripper_action_dict,
    gripper_command_lists,
)


@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    joints: dict[str, tuple[float, ...]]
    desired_grippers: dict[str, tuple[float, ...]]
    arm_poses: dict[str, tuple[float, ...]]
    robot_alive: bool
    have_control_rights: bool
    driver_alive: bool
    timestamp_ns: int
    desired_arm_poses: dict[str, tuple[float, ...]] = field(default_factory=dict)


class AstribotS1SdkSession:
    """Small, injectable boundary around the external ROS2 SDK."""

    def __init__(
        self,
        *,
        frequency_hz: float = 250.0,
        high_control_rights: bool = False,
        client_factory: Callable[..., Any] | None = None,
    ) -> None:
        if frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        factory = client_factory or self._default_client_factory
        self.client = factory(
            frequency_hz=float(frequency_hz),
            high_control_rights=bool(high_control_rights),
        )
        self._closed = False
        self._validate_identity()
        self.joint_limits = self._read_joint_limits()
        self.joint_velocity_limits = self._read_joint_velocity_limits()

    @staticmethod
    def _default_client_factory(
        *, frequency_hz: float, high_control_rights: bool
    ) -> Any:
        from astribot_sdk.core.astribot_api.astribot_client import Astribot

        return Astribot(
            freq=frequency_hz,
            high_control_rights=high_control_rights,
            node_name="xpolicylab_astribot_s1",
        )

    def _validate_identity(self) -> None:
        names = tuple(map(str, self.client.whole_body_names))
        if names != PART_NAMES:
            raise RuntimeError(
                f"SDK robot parts do not match Astribot S1 profile: {names}"
            )
        dofs = tuple(map(int, self.client.get_dof(list(PART_NAMES))))
        expected = tuple(PART_DOFS.values())
        if dofs != expected:
            raise RuntimeError(f"SDK DoF mismatch: expected {expected}, got {dofs}")

    def _read_joint_limits(
        self,
    ) -> dict[str, tuple[tuple[float, ...], tuple[float, ...]]]:
        result = self.client.get_joints_position_limit(list(PART_NAMES))
        if not isinstance(result, Sequence) or len(result) != 2:
            raise RuntimeError("SDK returned an invalid joint-position limit payload")
        lower_parts, upper_parts = result
        limits = {}
        for index, part in enumerate(PART_NAMES):
            lower = tuple(map(float, lower_parts[index]))
            upper = tuple(map(float, upper_parts[index]))
            expected = PART_DOFS[part]
            if len(lower) != expected or len(upper) != expected:
                raise RuntimeError(f"SDK returned invalid limits for {part}")
            limits[part] = (lower, upper)
        return limits

    def _read_joint_velocity_limits(self) -> dict[str, tuple[float, ...]]:
        reader = getattr(self.client, "get_joints_velocity_limit", None)
        if reader is None:
            return {
                part: tuple([float("inf")] * dof) for part, dof in PART_DOFS.items()
            }
        result = reader(list(PART_NAMES))
        if not isinstance(result, Sequence) or len(result) != len(PART_NAMES):
            raise RuntimeError("SDK returned invalid joint-velocity limits")
        limits = {}
        for part, dof, values in zip(
            PART_NAMES,
            PART_DOFS.values(),
            result,
            strict=True,
        ):
            limit = tuple(map(float, values))
            if len(limit) != dof or any(value <= 0 for value in limit):
                raise RuntimeError(f"SDK returned invalid velocity limits for {part}")
            limits[part] = limit
        return limits

    def snapshot(self) -> RobotSnapshot:
        joint_values = self.client.get_current_joints_position(list(PART_NAMES))
        gripper_values = self.client.get_desired_joints_position(list(GRIPPER_NAMES))
        pose_values = self.client.get_current_cartesian_pose(
            names=list(ARM_NAMES),
            frame=self.client.chassis_frame_name,
        )
        desired_pose_values = self.client.get_desired_cartesian_pose(
            names=list(ARM_NAMES),
            frame=self.client.chassis_frame_name,
        )
        joints = {
            part: tuple(map(float, joint_values[index]))
            for index, part in enumerate(PART_NAMES)
        }
        arm_poses = {
            arm: tuple(map(float, pose_values[index]))
            for index, arm in enumerate(ARM_NAMES)
        }
        desired_arm_poses = {
            arm: tuple(map(float, desired_pose_values[index]))
            for index, arm in enumerate(ARM_NAMES)
        }
        desired_grippers = {
            gripper: tuple(map(float, gripper_values[index]))
            for index, gripper in enumerate(GRIPPER_NAMES)
        }
        interface = self.client.astribot_interface
        driver_alive = bool(getattr(interface, "flag_robot_driver_alive", True))
        robot_alive = bool(interface.is_alive()) and driver_alive
        return RobotSnapshot(
            joints=joints,
            desired_grippers=desired_grippers,
            arm_poses=arm_poses,
            robot_alive=robot_alive,
            have_control_rights=bool(self.client.get_control_rights_status()),
            driver_alive=driver_alive,
            timestamp_ns=time.time_ns(),
            desired_arm_poses=desired_arm_poses,
        )

    def set_cartesian_gripper(
        self,
        action: Mapping[str, Any],
        *,
        control_way: str = "filter",
        use_wbc: bool = False,
    ) -> None:
        arm_names, _, arm_commands = arm_command_lists(action)
        _, _, commands = command_lists(action)
        self.client.set_cartesian_pose(
            arm_names,
            arm_commands,
            control_way=control_way,
            use_wbc=use_wbc,
            add_default_torso=False,
            remote_wbc_option=False,
        )
        self.client.set_joints_position(
            list(GRIPPER_NAMES),
            [commands[1], commands[3]],
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def set_cartesian_arms(
        self,
        action: Mapping[str, Any],
        *,
        control_way: str = "filter",
        use_wbc: bool = False,
    ) -> None:
        names, _, commands = arm_command_lists(action)
        self.client.set_cartesian_pose(
            names,
            commands,
            control_way=control_way,
            use_wbc=use_wbc,
            add_default_torso=False,
            remote_wbc_option=False,
        )

    def set_arm_joints(
        self,
        action: Mapping[str, Any],
        *,
        control_way: str = "filter",
        use_wbc: bool = False,
    ) -> None:
        names, commands = arm_joint_command_lists(action)
        self.client.set_joints_position(
            names,
            commands,
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def set_grippers(
        self,
        action: Mapping[str, Any],
        *,
        control_way: str = "filter",
    ) -> None:
        names, commands = gripper_command_lists(action)
        self.client.set_joints_position(
            names,
            commands,
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def set_arm_joints_and_grippers(
        self,
        arm_action: Mapping[str, Any],
        gripper_action: Mapping[str, Any],
        *,
        control_way: str = "filter",
    ) -> None:
        names, commands = arm_joint_gripper_command_lists(arm_action, gripper_action)
        self.client.set_joints_position(
            names,
            commands,
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def set_arm_gripper_head_positions(
        self,
        auxiliary_action: Mapping[str, Any],
        arm_action: Mapping[str, Any],
        gripper_action: Mapping[str, Any],
        *,
        control_way: str = "filter",
    ) -> None:
        """Write the filtered non-torso upper body as one SDK command group."""
        auxiliary = auxiliary_action_dict(auxiliary_action)
        arms = arm_joint_action_dict(arm_action)
        grippers = gripper_action_dict(gripper_action)
        names = [
            "astribot_arm_left",
            "astribot_gripper_left",
            "astribot_arm_right",
            "astribot_gripper_right",
            "astribot_head",
        ]
        commands = [
            [
                arms[f"astribot_arm_left.joint_{index}"]
                for index in range(PART_DOFS["astribot_arm_left"])
            ],
            [grippers["astribot_gripper_left.joint_0"]],
            [
                arms[f"astribot_arm_right.joint_{index}"]
                for index in range(PART_DOFS["astribot_arm_right"])
            ],
            [grippers["astribot_gripper_right.joint_0"]],
            [
                auxiliary[f"astribot_head.joint_{index}"]
                for index in range(PART_DOFS["astribot_head"])
            ],
        ]
        self.client.set_joints_position(
            names,
            commands,
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def set_torso_position(
        self,
        auxiliary_action: Mapping[str, Any],
        *,
        control_way: str = "direct",
    ) -> None:
        """Write only the torso, as in the vendor's torso-only examples."""
        auxiliary = auxiliary_action_dict(auxiliary_action)
        command = [
            auxiliary[f"astribot_torso.joint_{index}"]
            for index in range(PART_DOFS["astribot_torso"])
        ]
        self.client.set_joints_position(
            ["astribot_torso"],
            [command],
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def set_upper_body_positions(
        self,
        auxiliary_action: Mapping[str, Any],
        arm_action: Mapping[str, Any],
        gripper_action: Mapping[str, Any],
        *,
        control_way: str = "filter",
    ) -> None:
        """Write the SDK's canonical chassis-free whole-body joint group."""
        auxiliary = auxiliary_action_dict(auxiliary_action)
        arms = arm_joint_action_dict(arm_action)
        grippers = gripper_action_dict(gripper_action)
        names = [
            "astribot_torso",
            "astribot_arm_left",
            "astribot_gripper_left",
            "astribot_arm_right",
            "astribot_gripper_right",
            "astribot_head",
        ]
        commands = [
            [
                auxiliary[f"astribot_torso.joint_{index}"]
                for index in range(PART_DOFS["astribot_torso"])
            ],
            [
                arms[f"astribot_arm_left.joint_{index}"]
                for index in range(PART_DOFS["astribot_arm_left"])
            ],
            [grippers["astribot_gripper_left.joint_0"]],
            [
                arms[f"astribot_arm_right.joint_{index}"]
                for index in range(PART_DOFS["astribot_arm_right"])
            ],
            [grippers["astribot_gripper_right.joint_0"]],
            [
                auxiliary[f"astribot_head.joint_{index}"]
                for index in range(PART_DOFS["astribot_head"])
            ],
        ]
        self.client.set_joints_position(
            names,
            commands,
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def set_full_body_positions(
        self,
        chassis_position: Sequence[float],
        auxiliary_action: Mapping[str, Any],
        arm_action: Mapping[str, Any],
        gripper_action: Mapping[str, Any],
        *,
        control_way: str = "filter",
    ) -> None:
        chassis = list(map(float, chassis_position))
        if len(chassis) != PART_DOFS["astribot_chassis"]:
            raise ValueError("Astribot chassis position must contain 3 values")
        auxiliary = auxiliary_action_dict(auxiliary_action)
        arms = arm_joint_action_dict(arm_action)
        grippers = gripper_action_dict(gripper_action)
        commands = [
            chassis,
            [
                auxiliary[f"astribot_torso.joint_{index}"]
                for index in range(PART_DOFS["astribot_torso"])
            ],
            [
                arms[f"astribot_arm_left.joint_{index}"]
                for index in range(PART_DOFS["astribot_arm_left"])
            ],
            [grippers["astribot_gripper_left.joint_0"]],
            [
                arms[f"astribot_arm_right.joint_{index}"]
                for index in range(PART_DOFS["astribot_arm_right"])
            ],
            [grippers["astribot_gripper_right.joint_0"]],
            [
                auxiliary[f"astribot_head.joint_{index}"]
                for index in range(PART_DOFS["astribot_head"])
            ],
        ]
        self.client.set_joints_position(
            list(PART_NAMES),
            commands,
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def set_chassis_arm_gripper_head_positions(
        self,
        chassis_position: Sequence[float],
        auxiliary_action: Mapping[str, Any],
        arm_action: Mapping[str, Any],
        gripper_action: Mapping[str, Any],
        *,
        control_way: str = "filter",
    ) -> None:
        """Write every full-body position target except the independently driven torso."""
        chassis = list(map(float, chassis_position))
        if len(chassis) != PART_DOFS["astribot_chassis"]:
            raise ValueError("Astribot chassis position must contain 3 values")
        auxiliary = auxiliary_action_dict(auxiliary_action)
        arms = arm_joint_action_dict(arm_action)
        grippers = gripper_action_dict(gripper_action)
        names = [
            "astribot_chassis",
            "astribot_arm_left",
            "astribot_gripper_left",
            "astribot_arm_right",
            "astribot_gripper_right",
            "astribot_head",
        ]
        commands = [
            chassis,
            [
                arms[f"astribot_arm_left.joint_{index}"]
                for index in range(PART_DOFS["astribot_arm_left"])
            ],
            [grippers["astribot_gripper_left.joint_0"]],
            [
                arms[f"astribot_arm_right.joint_{index}"]
                for index in range(PART_DOFS["astribot_arm_right"])
            ],
            [grippers["astribot_gripper_right.joint_0"]],
            [
                auxiliary[f"astribot_head.joint_{index}"]
                for index in range(PART_DOFS["astribot_head"])
            ],
        ]
        self.client.set_joints_position(
            names,
            commands,
            control_way=control_way,
            use_wbc=False,
            add_default_torso=False,
        )

    def activate_cameras(self) -> bool:
        """Activate the vendor camera module without changing robot motion."""
        return bool(self.client.activate_camera())

    def get_cameras_info(self) -> dict[str, Any]:
        result = self.client.get_cameras_info()
        if not isinstance(result, Mapping):
            raise RuntimeError("SDK returned an invalid camera-info payload")
        return {str(name): value for name, value in result.items()}

    def register_image_callback(
        self,
        camera_name: str,
        image_type: str,
        callback: Callable[..., None],
        *,
        need_decode: bool,
    ) -> Any:
        return self.client.register_image_callback(
            str(camera_name),
            str(image_type),
            callback,
            need_decode=bool(need_decode),
        )

    def unregister_image_callback(self, subscription: Any) -> None:
        """Destroy an SDK-created ROS subscription when the node supports it."""
        interface = self.client.astribot_interface
        node = getattr(interface, "node", None)
        destroy = getattr(node, "destroy_subscription", None)
        if destroy is not None:
            destroy(subscription)
            return
        destroy = getattr(subscription, "destroy", None)
        if destroy is not None:
            destroy()

    def stop_robot(self) -> None:
        self.client.stop_robot()

    def restart_robot(self) -> None:
        self.client.restart_robot()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        shutdown = getattr(self.client.astribot_interface, "shutdown", None)
        if shutdown is not None:
            shutdown()

    def __enter__(self) -> AstribotS1SdkSession:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
