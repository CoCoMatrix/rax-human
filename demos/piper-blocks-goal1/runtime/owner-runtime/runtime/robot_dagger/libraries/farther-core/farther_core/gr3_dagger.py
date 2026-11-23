from __future__ import annotations

from collections.abc import Mapping

import numpy as np


SCHEMA_VERSION = "gr3_dagger_v2"
RETARGET_MAPPING_VERSION = "gr3_retarget_v4"
CONTROLLER_VERSION = "gr3_dagger_router_v2"

JOINT_NAMES = (
    "head_yaw_joint",
    "head_pitch_joint",
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_yaw_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_wrist_yaw_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "L_pinky_proximal_joint",
    "L_ring_proximal_joint",
    "L_middle_proximal_joint",
    "L_index_proximal_joint",
    "L_thumb_proximal_pitch_joint",
    "L_thumb_proximal_yaw_joint",
    "R_pinky_proximal_joint",
    "R_ring_proximal_joint",
    "R_middle_proximal_joint",
    "R_index_proximal_joint",
    "R_thumb_proximal_pitch_joint",
    "R_thumb_proximal_yaw_joint",
)
STATE_NAMES = JOINT_NAMES + ("base_height", "base_pitch")
BASE_ACTION_NAMES = (
    "vel_height",
    "vel_pitch",
    "base_yaw",
    "vel_x",
    "vel_y",
    "vel_yaw",
)
ACTION_NAMES = JOINT_NAMES + BASE_ACTION_NAMES
STATE_DIM = len(STATE_NAMES)
ACTION_DIM = len(ACTION_NAMES)


def canonical_robot_action(
    joints: Mapping[str, float], base: Mapping[str, float]
) -> dict[str, float]:
    values = {**joints, **base}
    missing = [name for name in ACTION_NAMES if name not in values]
    if missing:
        raise ValueError(f"GR3 action is missing: {missing}")
    action = {name: float(values[name]) for name in ACTION_NAMES}
    if not np.isfinite(tuple(action.values())).all():
        raise ValueError("GR3 action contains NaN or Inf")
    return action


def canonical_robot_state(
    joints: Mapping[str, float], base_state: Mapping
) -> dict[str, float]:
    try:
        base = base_state["base"]
        values = {
            **joints,
            "base_height": float(base["position"][2]),
            "base_pitch": float(base["rpy"][1]),
        }
    except (KeyError, IndexError, TypeError) as exc:
        raise ValueError(f"invalid GR3 base state: {exc}") from exc
    missing = [name for name in STATE_NAMES if name not in values]
    if missing:
        raise ValueError(f"GR3 state is missing: {missing}")
    state = {name: float(values[name]) for name in STATE_NAMES}
    if not np.isfinite(tuple(state.values())).all():
        raise ValueError("GR3 state contains NaN or Inf")
    return state


def action_vector(action: Mapping[str, float]) -> np.ndarray:
    missing = [name for name in ACTION_NAMES if name not in action]
    if missing:
        raise ValueError(f"GR3 action is missing: {missing}")
    vector = np.asarray([action[name] for name in ACTION_NAMES], dtype=np.float32)
    if vector.shape != (ACTION_DIM,) or not np.isfinite(vector).all():
        raise ValueError(f"GR3 action must be finite ({ACTION_DIM},)")
    return vector


def state_vector(state: Mapping[str, float]) -> np.ndarray:
    missing = [name for name in STATE_NAMES if name not in state]
    if missing:
        raise ValueError(f"GR3 state is missing: {missing}")
    vector = np.asarray([state[name] for name in STATE_NAMES], dtype=np.float32)
    if vector.shape != (STATE_DIM,) or not np.isfinite(vector).all():
        raise ValueError(f"GR3 state must be finite ({STATE_DIM},)")
    return vector
