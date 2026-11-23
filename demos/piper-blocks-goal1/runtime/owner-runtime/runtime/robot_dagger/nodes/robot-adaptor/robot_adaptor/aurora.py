import logging
import os
import time
from enum import IntEnum
from typing import TypeAlias

import numpy as np

from farther_core.utils import Interpolator
from fourier_aurora_client import AuroraClient
from robot_adaptor.write_authorization import (
    WriteAuthorizedClient,
    require_hardware_write_authorization,
)

logging.basicConfig(level=logging.DEBUG)
# Set transitions' log level to INFO; DEBUG messages will be omitted
logging.getLogger("transitions").setLevel(logging.INFO)

logger = logging.getLogger(__name__)


def parse_robot_name(name: str):
    if name.startswith("gr2"):
        return "gr2t2v2"
    elif name.startswith("gr3"):
        return "gr3"


WBC_WALK = os.getenv("WBC_WALK", "0") == "1"
UPPER_ONLY = os.getenv("UPPER_ONLY", "0") == "1"


class WholeBodyPdState(IntEnum):
    STAND = 2
    WALK = 4 if WBC_WALK else 3


class UpperPdState(IntEnum):
    STAND = 11
    WALK = 11


if UPPER_ONLY:
    PdState: TypeAlias = UpperPdState
else:
    PdState: TypeAlias = WholeBodyPdState


class AuroraWrapper:
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
        require_hardware_write_authorization(write_enabled)
        retries = int(os.getenv("AURORA_INIT_RETRIES", "15"))
        base_delay = float(os.getenv("AURORA_INIT_RETRY_DELAY", "2.0"))
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                raw_client = AuroraClient.get_instance(
                    robot_id, robot_name=parse_robot_name(robot_name)
                )
                if raw_client:
                    self.client = WriteAuthorizedClient(
                        raw_client,
                        enabled=write_enabled,
                    )
                    time.sleep(1)
                    logger.info(
                        f"AuroraClient 初始化成功 robot={robot_name} id={robot_id} 尝试={attempt}"
                    )
                    break
            except Exception as e:
                last_exc = e
                logger.error(
                    f"AuroraClient 初始化失败 robot={robot_name} id={robot_id} 尝试={attempt}/{retries}: {e}",
                    exc_info=True,
                )
                if attempt < retries:
                    time.sleep(base_delay * attempt)
        if not hasattr(self, "client"):
            logger.critical(
                f"AuroraClient 初始化失败已重试 {retries} 次 robot={robot_name} id={robot_id}"
            )
            raise last_exc or RuntimeError("AuroraClient.get_instance returned no client")
        self.robot_name = robot_name
        self.controlled_joints = controlled_joints
        self.idle_pose = idle_pose
        self.base_limits = base_limits
        self.setting_state_time: float | None = None
        self.setting_state_target: PdState | int | None = None
        self.setting_duration = 0.5
        self.frequency = 60
        time.sleep(1)

        self.client.set_fsm_state(PdState.STAND)
        time.sleep(0.5)
        self.client.set_velocity_source(2)
        time.sleep(0.5)
        self.client.set_upper_fsm_state(2)

        self.stand_cmd = [0.0, 0.0, 0.0]

        time.sleep(1)

    @property
    def pd_state(self):
        return "walk" if self.get_state() == PdState.WALK else "stand"

    def setting_state(self):
        if self.setting_state_time is not None:
            if time.time() - self.setting_state_time < self.setting_duration:
                return True
        self.setting_state_time = None
        return False

    def set_state(self, state: PdState | int):
        if self.setting_state():
            return
        self.setting_state_target = state
        self.client.set_fsm_state(state)

        time.sleep(0.5)
        self.client.set_velocity_source(2)
        self.client.set_upper_fsm_state(2)
        self.setting_state_time = time.time()
        self.stand_cmd = [0.0, 0.0, 0.0]

    def get_state(self):
        if self.setting_state_target is not None:
            return self.setting_state_target
        pd_state = self.client.get_fsm_state()
        if pd_state == "PdStand":
            return PdState.STAND
        elif pd_state == "UserController_A":
            return PdState.WALK

    def get_stand_pose(self):
        h, p, y, _ = self.client.get_stand_pose()
        return h, p, y

    def set_stand_pose(self, height: float, pitch: float, yaw: float):
        if self.setting_state():
            return
        height = np.clip(height, self.base_limits["lower"][0], self.base_limits["upper"][0])
        pitch = np.clip(pitch, self.base_limits["lower"][1], self.base_limits["upper"][1])
        yaw = np.clip(yaw, self.base_limits["lower"][2], self.base_limits["upper"][2])
        self.client.set_stand_pose(height, pitch, yaw)

    def set_velocity(self, x: float, y: float, yaw: float):
        if self.setting_state():
            return
        # print(f"velocity: {x:.2f}, {y:.2f}, {yaw:.2f}")
        self.client.set_velocity(x, y, yaw)

    def _restore_stand(self, duration: float = 1.0):
        if self.get_state() != PdState.STAND:
            return
        delta_z, delta_pitch, delta_yaw = self.get_stand_pose()
        # interpolate to zero
        it = Interpolator(np.array([delta_z, delta_pitch, delta_yaw]), np.zeros(3), duration, 0.05)
        for pose in it:
            self.set_stand_pose(*pose)
        self.set_stand_pose(0.0, 0.0, 0.0)

    def _stop_walking(self):
        if self.get_state() != PdState.WALK:
            return
        self.set_velocity(0, 0, 0)

    # ------ end walking machine ------

    @property
    def joint_positions(self):
        q_dict = {}
        for key, val in self.controlled_joints.items():
            pos_array = self.client.get_group_state(key, "position")
            if "hand" in key:
                if len(pos_array) == 6:
                    pos_array = [
                        -pos_array[0] + 0.17,
                        -pos_array[1] + 0.17,
                        -pos_array[2] + 0.17,
                        -pos_array[3] + 0.17,
                        pos_array[4] - 0.12,
                        -pos_array[5],
                    ]
                elif len(pos_array) == 12:
                    pos_array = [
                        0.2 - pos_array[0],
                        0.01 - pos_array[1],
                        pos_array[2] + 0.04,
                        0.2 - pos_array[3],
                        0.01 - pos_array[4],
                        0.2 - pos_array[5],
                        0.01 - pos_array[6],
                        0.2 - pos_array[7],
                        0.01 - pos_array[8],
                        pos_array[9] + 0.02,
                        pos_array[10] - 0.14,
                        0.2 - pos_array[11],
                    ]
            for i, joint_name in enumerate(val):
                q_dict[joint_name] = pos_array[i]

        return q_dict

    @joint_positions.setter
    def joint_positions(self, q_dict: dict[str, float]):
        position_dict = {}
        for key, val in self.controlled_joints.items():
            try:
                position_dict[key] = [q_dict[joint_name] for joint_name in val]
            except KeyError:
                continue
            if "hand" in key:
                pos_array = position_dict[key]
                if len(pos_array) == 6:
                    position_dict[key] = [
                        0.17 - pos_array[0],
                        0.17 - pos_array[1],
                        0.17 - pos_array[2],
                        0.17 - pos_array[3],
                        0.12 + pos_array[4],
                        -pos_array[5],
                    ]
                elif len(pos_array) == 12:
                    position_dict[key] = [
                        0.2 - pos_array[0],
                        0.01 - pos_array[1],
                        pos_array[2] - 0.04,
                        0.2 - pos_array[3],
                        0.01 - pos_array[4],
                        0.2 - pos_array[5],
                        0.01 - pos_array[6],
                        0.2 - pos_array[7],
                        0.01 - pos_array[8],
                        pos_array[9] - 0.02,
                        pos_array[10] + 0.14,
                        0.2 - pos_array[11],
                    ]
        self.client.set_joint_positions(position_dict)

    @property
    def base_state(self):
        state = self.get_state()
        stand_pose = self.client.get_stand_pose()  # [delta_z, delta_pitch, delta_yaw, stable_level]
        base_rpy = self.client.get_base_data("rpy")
        base_quat = self.client.get_base_data("quat_wxyz")
        base_pos = self.client.get_base_data("pos_W")
        omega_B = self.client.get_base_data("omega_B")
        acc_B = self.client.get_base_data("acc_B")

        return {
            "state": state,
            "stand_pose": stand_pose,
            "base": {"rpy": base_rpy, "position": base_pos, "quat": base_quat},
            "imu": {"omega_B": omega_B, "acc_B": acc_B},
        }
