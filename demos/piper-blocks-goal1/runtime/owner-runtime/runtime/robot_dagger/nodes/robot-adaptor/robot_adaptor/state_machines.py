import logging
import time

import numpy as np
import pyarrow as pa
from scipy.spatial.transform import Rotation as R
from transitions import Machine
from transitions.extensions import HierarchicalMachine

from farther_core.utils import DictInterpolator
from robot_adaptor.aurora import AuroraWrapper, PdState
from robot_adaptor.controller import RobotBasePoseController

logging.basicConfig(level=logging.DEBUG)
# Set transitions' log level to INFO; DEBUG messages will be omitted
logging.getLogger("transitions").setLevel(logging.INFO)

logger = logging.getLogger(__name__)

ROBOT_NAME_MAPPING = {"gr2": "gr2t2v2", "gr3": "PPWAVEV2"}

# STATES = [
#     "idle",
#     "engaging",
#     "engaged",
#     "disengaged",
# ]

# TRANSITIONS = [
#     {"trigger": "next", "source": "idle", "dest": "engaging"},
#     {"trigger": "next", "source": "engaging", "dest": "engaged"},
#     {"trigger": "next", "source": "engaged", "dest": "disengaged"},
#     {"trigger": "next", "source": "disengaged", "dest": "idle"},
#     {"trigger": "pause", "source": "*", "dest": "disengaged"},
#     {"trigger": "stop", "source": "*", "dest": "idle"},
# ]


def remap(
    x: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
    clip=True,
) -> float:
    tmp = (x - old_min) / (old_max - old_min)
    if clip:
        tmp = np.clip(tmp, new_min, new_max)
    return new_min + tmp * (new_max - new_min)


class DataCollectionMachine:
    def __init__(self, node):
        self.episode_index = -1
        self.node = node

        self.machine = Machine(
            model=self,
            states=["collect", "idle"],
            transitions=[
                {
                    "trigger": "next",
                    "source": "idle",
                    "dest": "collect",
                    "before": "increment_episode",
                    "after": "pub_current_state",
                },
                {
                    "trigger": "next",
                    "source": "collect",
                    "dest": "idle",
                    "before": "finish_episode",
                    "after": "pub_current_state",
                },
                {
                    "trigger": "abort",
                    "source": "collect",
                    "dest": "idle",
                    "before": "abort_episode",
                    "after": "pub_current_state",
                },
                {"trigger": "stop", "source": "*", "dest": "idle"},
            ],
            initial="idle",
            ignore_invalid_triggers=True,
        )

        self.logger = logging.getLogger(__name__ + ".DataCollection")

    def pub_current_state(self):
        self.logger.debug(f"Collection state: {self.state}_{self.episode_index:03d}")

    def increment_episode(self):
        self.episode_index += 1
        self.node.send_output("episode_index", pa.array([self.episode_index]))
        self.logger.info(f"Episode {self.episode_index} started")

    def finish_episode(self):
        self.logger.info(f"Episode {self.episode_index} finished")
        self.node.send_output("episode_index", pa.array([-1]))

    def abort_episode(self):
        self.node.send_output("episode_failed", pa.array([self.episode_index]))
        self.logger.info(f"Episode {self.episode_index} failed")


class AuroraMachine(AuroraWrapper):
    def __init__(
        self,
        robot_name: str,
        robot_id: int,
        controlled_joints: dict[str, list[str]],
        base_limits: dict[str, float],
        idle_pose: dict[str, float],
        idle_poses_sequence: list[dict[str, float]] = None,
        *,
        write_enabled: bool,
    ):
        super().__init__(
            robot_name,
            robot_id,
            controlled_joints,
            base_limits,
            idle_pose,
            write_enabled=write_enabled,
        )
        self.idle_poses_sequence = idle_poses_sequence or [idle_pose]
        self.current_idle_pose_index = 0

        STATES = [
            "idle",
            "preengage",
            "engaging",
            {
                "name": "engaged",
                "children": [
                    {"name": "standing", "children": ["one", "two", "transition"]},
                    {"name": "walking", "children": ["one", "two"]},
                ],
            },
            "disengaged",
        ]

        TRANSITIONS = [
            {"trigger": "next", "source": "idle", "dest": "preengage", "after": "pub_current_state"},
            {"trigger": "ready", "source": "preengage", "dest": "engaging", "after": "pub_current_state"},
            {"trigger": "engaged", "source": "engaging", "dest": "engaged_standing_one", "after": "pub_current_state"},
            # {"trigger": "walk", "source": "engaged_standing_one", "dest": "engaged_standing_transition"},
            # {"trigger": "walk", "source": "engaged_standing_two", "dest": "engaged_standing_transition"},
            # {"trigger": "walk", "source": "engaged_standing_transition", "dest": "engaged_walking_one"},
            {
                "trigger": "clutch",
                "source": "engaged_walking_one",
                "dest": "engaged_walking_two",
                "after": "pub_current_state",
            },
            {
                "trigger": "clutch",
                "source": "engaged_walking_two",
                "dest": "engaged_walking_one",
                "after": "pub_current_state",
            },
            # {"trigger": "stand", "source": "engaged_walking", "dest": "engaged_standing_one", "before": "_stop_walking"},
            {
                "trigger": "clutch",
                "source": "engaged_standing_one",
                "dest": "engaged_standing_two",
                "after": "pub_current_state",
            },
            {
                "trigger": "clutch",
                "source": "engaged_standing_two",
                "dest": "engaged_standing_one",
                "after": "pub_current_state",
            },
            {
                "trigger": "toggle_pd",
                "source": "engaged_standing",
                "dest": "engaged_walking_one",
                "after": "pub_current_state",
            },
            {
                "trigger": "toggle_pd",
                "source": "engaged_walking",
                "dest": "engaged_standing_one",
                "before": "_stop_walking",
                "after": "pub_current_state",
            },
            {"trigger": "next", "source": "engaged", "dest": "disengaged", "after": "pub_current_state"},
            {"trigger": "next", "source": "disengaged", "dest": "preengage", "after": "pub_current_state"},
            {"trigger": "stop", "source": "*", "dest": "disengaged", "after": "pub_current_state"},
            {"trigger": "stop", "source": "disengaged", "dest": "idle", "after": "pub_current_state"},
        ]

        self.machine = HierarchicalMachine(
            model=self, states=STATES, transitions=TRANSITIONS, initial="idle", ignore_invalid_triggers=True
        )

        self.engaged = False
        self.engage_target: DictInterpolator | None = None
        self.engage_duration = 1.0
        self.preserve_pd_state_on_disengage = False

        self.controller = RobotBasePoseController(self.frequency, limits=self.base_limits)

    # --- 👇 Engaging👇---
    def set_engage_target(self, target: dict[str, float]):
        self.engage_target = DictInterpolator(self.joint_positions, target, self.engage_duration, 1 / 60)

    def step_engage(self, t: float | None = None):
        if self.engage_target is None:
            print("⚠️ Engage target is not set.")
            self.trigger("stop")
            return

        t = time.time() if t is None else t

        target = self.engage_target.update(t)
        if target is not None:
            self.joint_positions = target
        else:
            print("🕹️ Robot fully engaged.")
            self.trigger("engaged")
            self.pub_current_state()
            self.engage_target = None

    # --- 👆 Engaging 👆---

    def pub_current_state(self):
        logger.debug(f"FSM state: {self.state}")

    def on_enter_idle(self):
        logger.info(f"[FSM] {self.state}")
        self.pub_current_state()

    def on_enter_disengaged(self):
        logger.info(f"[FSM] {self.state}")
        if self.preserve_pd_state_on_disengage:
            self.preserve_pd_state_on_disengage = False
        else:
            self.set_state(PdState.STAND)

        current_pose = self.joint_positions
        self.joint_positions = current_pose

    def on_enter_preengage(self):
        logger.info(f"[FSM] {self.state}")
        # Prepare Aurora ownership once, but leave the actual joint motion to
        # the asynchronous engaging phase.  Previously move_to_idle_pose()
        # interpolated to the idle pose here and set_engage_target() immediately
        # interpolated to the same pose again.
        self.prepare_for_engage()

    def prepare_for_engage(self):
        """Put Aurora in the mode required for one engage interpolation."""
        self.client.set_upper_fsm_state(2)

        if "gr2" in self.robot_name:
            self.set_state(PdState.STAND)
        elif "gr3" in self.robot_name:
            self.set_state(PdState.WALK)
        else:
            raise ValueError(f"Unknown robot name: {self.robot_name}")

        time.sleep(1.0)

    def move_to_idle_pose(self, duration: float = 1.0):
        self.prepare_for_engage()

        current_pose = self.joint_positions
        print(f"{current_pose=}")

        # Execute all idle poses in sequence
        logger.info(f"Executing {len(self.idle_poses_sequence)} idle poses in sequence")

        for i, target_pose in enumerate(self.idle_poses_sequence):
            for key, value in current_pose.items():
                if "elbow" in key and value != 0 and target_pose[key] == 0:
                    target_pose[key] = value

            # real_target_pos = current_pose.copy()

            # for key, value in target_pose.items():
            #     real_target_pos[key] = value if value != 0 else real_target_pos[key]
            logger.info(f"Moving to idle pose {i + 1}/{len(self.idle_poses_sequence)}")
            print(f"Target pose {i + 1}: {target_pose}")

            # Interpolate from current pose to target pose
            for pose in DictInterpolator(current_pose, target_pose, duration, 1 / 60):
                self.joint_positions = pose

            # Update current_pose for next iteration
            current_pose = self.joint_positions

            # Small delay between poses for smoother transitions
            if i < len(self.idle_poses_sequence) - 1:  # Don't delay after the last pose
                time.sleep(0.5)

        logger.info("Completed idle poses sequence")

    def on_enter_engaging(self):
        logger.info(f"[FSM] {self.state}")
        if self.engage_target is None:
            logger.error("Engage target is None")
            self.trigger("stop")
            return

    def on_enter_engaged(self):
        logger.info(f"[FSM] {self.state}")
        self.client.set_upper_fsm_state(2)

    def on_enter_engaged_standing(self):
        logger.info(f"[FSM] {self.state}")
        if "gr2" in self.robot_name:
            self.set_state(PdState.STAND)
        elif "gr3" in self.robot_name:
            self.set_state(PdState.WALK)
        else:
            raise ValueError(f"Unknown robot name: {self.robot_name}")

    def on_enter_engaged_walking(self):
        logger.info(f"[FSM] {self.state}")
        if "gr2" in self.robot_name:
            self.set_state(PdState.WALK)
        elif "gr3" in self.robot_name:
            pass
        else:
            raise ValueError(f"Unknown robot name: {self.robot_name}")

    def pd_command(self, x, y, z):
        DEADZONE = 0.3
        Z_DEADZONE = 0.3
        Y_THREADHOLD = 0.5
        BASE_HEIGHT_VEL = 0.00015 * 400 / self.frequency  # gr3: -0.5~0.04; gr2: -0.2~0.025
        BASE_PITCH_VEL = 0.0004 * 400 / self.frequency  # gr3: -0.3~0.5; gr2: -0.3~0.45
        BASE_YAW_VEL = 0.0005 * 400 / self.frequency  # gr2: -0.5~0.5

        X_VEL_COEFF = 0.8
        REVERSE_X_VEL_COEFF = 0.3
        YAW_VEL_COEFF = 0.8
        SIDE_VEL_COEFF = 1.0

        if self.state.startswith("engaged_standing"):
            if self.get_state() != PdState.STAND and "gr2" in self.robot_name:
                self.set_state(PdState.STAND)
                return None

            # if y > Y_THREADHOLD:  # y serve as a brake
            #     return

            cmd_x = -1 if x > DEADZONE else 0
            cmd_y = 1 if y > DEADZONE else 0
            cmd_z = 1 if z >= Z_DEADZONE else -1 if z <= -Z_DEADZONE else 0
            if self.state == "engaged_standing_one":
                # Mode 1: X controls height, Yaw controls pitch
                self.stand_cmd[0] += cmd_x * BASE_HEIGHT_VEL + cmd_y * BASE_PITCH_VEL
                self.stand_cmd[1] += cmd_z * BASE_PITCH_VEL

            elif self.state == "engaged_standing_two":
                # Mode 2: X controls height, Yaw controls yaw
                self.stand_cmd[0] += cmd_x * BASE_HEIGHT_VEL + cmd_y * BASE_PITCH_VEL
                self.stand_cmd[2] += cmd_z * BASE_YAW_VEL
            else:
                raise ValueError(f"Unknown state: {self.state}")
            for i in range(3):
                self.stand_cmd[i] = np.clip(
                    self.stand_cmd[i], self.base_limits["lower"][i], self.base_limits["upper"][i]
                )
            self.set_stand_pose(self.stand_cmd[0], self.stand_cmd[1], self.stand_cmd[2])

            return {
                "base_height": self.stand_cmd[0],
                "base_pitch": self.stand_cmd[1],
                "base_yaw": self.stand_cmd[2],
                "vel_x": 0.0,
                "vel_y": 0.0,
                "vel_yaw": 0.0,
            }

        elif self.state.startswith("engaged_walking"):
            if self.get_state() != PdState.WALK:
                self.set_state(PdState.WALK)
                return None

            vel = [0, 0, 0]

            def map_x_vel(x, y):
                diff = x - y
                if abs(diff) < 0.1:
                    return 0
                return diff * (X_VEL_COEFF if diff > 0 else REVERSE_X_VEL_COEFF)

            if self.state == "engaged_walking_one":
                vel = [map_x_vel(x, y), 0.0, z * YAW_VEL_COEFF]
                self.set_velocity(*vel)
            elif self.state == "engaged_walking_two":
                side_vel = z * SIDE_VEL_COEFF
                if abs(side_vel) < 0.1:
                    side_vel = 0
                elif abs(side_vel) < 0.3:
                    side_vel = 0.3
                vel = [map_x_vel(x, y), side_vel, 0.0]
                self.set_velocity(*vel)

            return {
                "base_height": self.stand_cmd[0],
                "base_pitch": self.stand_cmd[1],
                "base_yaw": self.stand_cmd[2],
                "vel_x": vel[0],
                "vel_y": vel[1],
                "vel_yaw": vel[2],
            }

        return None

    def base_command(self, xyzquat: np.ndarray):
        if "gr3" not in self.robot_name:
            raise NotImplementedError(f"Unsupported robot: {self.robot_name}")
        DEADZONE = 0.3

        if not self.state.startswith("engaged"):
            return None

        if self.get_state() != PdState.WALK:
            self.set_state(PdState.WALK)
            return None

        rpy = R.from_quat(xyzquat[3:]).as_euler("xyz")

        base_state = self.base_state

        cmd = {
            "base_height": self.controller.update(
                "base_height", xyzquat[2], base_state["stand_pose"][0], 1 / self.frequency
            ),
            "base_pitch": self.controller.update(
                "base_pitch", rpy[1], base_state["base"]["rpy"][1], 1 / self.frequency
            ),
            "base_yaw": 0.0,
            "vel_x": self.controller.update("vel_x", xyzquat[0], base_state["base"]["position"][0], 1 / self.frequency),
            "vel_y": self.controller.update("vel_y", xyzquat[1], base_state["base"]["position"][1], 1 / self.frequency),
            "vel_yaw": self.controller.update("vel_yaw", rpy[2], base_state["base"]["rpy"][2], 1 / self.frequency),
        }
        # print(f"target: {xyzquat[2]} state: {base_state['stand_pose'][0]} cmd: {cmd['base_height']}")

        self.set_stand_pose(
            cmd["base_height"] + base_state["stand_pose"][0],
            cmd["base_pitch"] + base_state["base"]["rpy"][1],
            cmd["base_yaw"],
        )
        self.set_velocity(cmd["vel_x"], cmd["vel_y"], cmd["vel_yaw"])
        return cmd

    def raw_base_command(self, cmd: dict):
        if "gr3" not in self.robot_name:
            raise NotImplementedError(f"Unsupported robot: {self.robot_name}")

        if not self.state.startswith("engaged"):
            return None

        if self.get_state() != PdState.WALK:
            self.set_state(PdState.WALK)
            return None

        base_state = self.base_state

        cmd_out = {}

        if "base_height" in cmd:
            cmd_out["base_height"] = cmd.get("base_height", base_state["stand_pose"][0])
        elif "vel_height" in cmd:
            if not hasattr(self, "base_height_cmd"):
                self.base_height_cmd = 0.0
            self.base_height_cmd += cmd.get("vel_height", 0.0)
            self.base_height_cmd = np.clip(
                self.base_height_cmd, self.base_limits["lower"][0], self.base_limits["upper"][0]
            )
            cmd_out["base_height"] = self.base_height_cmd
        else:
            raise ValueError(f"Unknown command: {cmd}")

        cmd_out["base_height"] = np.clip(
            cmd_out["base_height"], self.base_limits["lower"][0], self.base_limits["upper"][0]
        )

        if "base_pitch" in cmd:
            cmd_out["base_pitch"] = cmd.get("base_pitch", base_state["base"]["rpy"][1])
        elif "vel_pitch" in cmd:  # for wbc
            if not hasattr(self, "base_pitch_cmd"):
                self.base_pitch_cmd = 0.0
            self.base_pitch_cmd += cmd.get("vel_pitch", 0.0)
            self.base_pitch_cmd = np.clip(
                self.base_pitch_cmd, self.base_limits["lower"][1], self.base_limits["upper"][1]
            )
            cmd_out["base_pitch"] = self.base_pitch_cmd
        else:
            raise ValueError(f"Unknown command: {cmd}")

        cmd_out["base_pitch"] = np.clip(
            cmd_out["base_pitch"], self.base_limits["lower"][1], self.base_limits["upper"][1]
        )

        cmd_out["base_yaw"] = cmd.get("base_yaw", base_state["base"]["rpy"][2])
        cmd_out["base_yaw"] = np.clip(cmd_out["base_yaw"], self.base_limits["lower"][2], self.base_limits["upper"][2])

        cmd_out["vel_x"] = cmd.get("vel_x", 0.0)
        cmd_out["vel_y"] = cmd.get("vel_y", 0.0)
        cmd_out["vel_yaw"] = cmd.get("vel_yaw", 0.0)

        self.set_stand_pose(cmd_out["base_height"], cmd_out["base_pitch"], cmd_out["base_yaw"])
        self.set_velocity(cmd_out["vel_x"], cmd_out["vel_y"], cmd_out["vel_yaw"])
        return cmd_out


class AuroraMachineMinimal(AuroraWrapper):
    def __init__(
        self,
        robot_name: str,
        robot_id: int,
        controlled_joints: dict[str, list[str]],
        base_limits: dict[str, float],
        idle_pose: dict[str, float],
        *,
        write_enabled: bool,
    ):
        super().__init__(
            robot_name,
            robot_id,
            controlled_joints,
            base_limits,
            idle_pose,
            write_enabled=write_enabled,
        )

        self.machine = Machine(
            model=self,
            states=["idle", "engaging", "engaged", "disengaged"],
            transitions=[
                {"trigger": "next", "source": "idle", "dest": "engaging"},
                {"trigger": "next", "source": "engaging", "dest": "engaged"},
                {"trigger": "next", "source": "engaged", "dest": "disengaged"},
                {"trigger": "next", "source": "disengaged", "dest": "engaging"},
                {"trigger": "stop", "source": "*", "dest": "disengaged"},
                {"trigger": "stop", "source": "disengaged", "dest": "idle"},
            ],
            initial="idle",
            ignore_invalid_triggers=True,
        )

    def pub_current_state(self):
        logger.debug(f"VR FSM state: {self.state}")

    def on_enter_idle(self):
        logger.info(f"[FSM] {self.state}")
        # self.to_pd_stand()

        current_pose = self.joint_positions

        for pose in DictInterpolator(current_pose, self.idle_pose, 1.0, 1 / 60):
            self.joint_positions = pose

    def on_enter_disengaged(self):
        logger.info(f"[FSM] {self.state}")

        current_pose = self.joint_positions
        self.joint_positions = current_pose

    def on_enter_engaging(self):
        logger.info(f"[FSM] {self.state}")
        current_pose = self.joint_positions

        print(f"{current_pose=}")
        print(f"{self.idle_pose=}")
        for pose in DictInterpolator(current_pose, self.idle_pose, 1.0, 1 / 60):
            self.joint_positions = pose

    def on_enter_engaged(self):
        logger.info(f"[FSM] {self.state}")
