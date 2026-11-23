from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from XPolicyLab.real_world.contracts import ObservationFrame, VectorSchema


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

GR3_STATE_SCHEMA = VectorSchema(
    schema_id="gr3_dagger_v2.state",
    fields=STATE_NAMES,
    units=("rad",) * len(JOINT_NAMES) + ("m", "rad"),
    semantics=("absolute_position",) * len(STATE_NAMES),
)
GR3_JOINT_STATE_SCHEMA = VectorSchema(
    schema_id="gr3_joint_v1.state",
    fields=JOINT_NAMES,
    units=("rad",) * len(JOINT_NAMES),
    semantics=("absolute_position",) * len(JOINT_NAMES),
)
GR3_ACTION_SCHEMA = VectorSchema(
    schema_id="gr3_dagger_v2.action",
    fields=ACTION_NAMES,
    units=("rad",) * len(JOINT_NAMES)
    + ("m/s", "rad/s", "rad", "m/s", "m/s", "rad/s"),
    semantics=("absolute_position",) * len(JOINT_NAMES)
    + (
        "velocity",
        "velocity",
        "absolute_position",
        "velocity",
        "velocity",
        "velocity",
    ),
)


def state_vector(
    joints: Mapping[str, float], base_state: Mapping[str, Any]
) -> np.ndarray:
    try:
        base = base_state["base"]
        values = [
            *(joints[name] for name in JOINT_NAMES),
            base["position"][2],
            base["rpy"][1],
        ]
    except (KeyError, IndexError, TypeError) as exc:
        raise ValueError(f"invalid GR3 state: {exc}") from exc
    return GR3_STATE_SCHEMA.coerce(values, name="GR3 state", ndim=1)


def joint_state_vector(joints: Mapping[str, float]) -> np.ndarray:
    try:
        values = [joints[name] for name in JOINT_NAMES]
    except KeyError as exc:
        raise ValueError(f"invalid GR3 joint state: missing {exc.args[0]}") from exc
    return GR3_JOINT_STATE_SCHEMA.coerce(
        values, name="GR3 joint state", ndim=1
    )


def action_vector(action: Mapping[str, float]) -> np.ndarray:
    try:
        values = [action[name] for name in ACTION_NAMES]
    except KeyError as exc:
        raise ValueError(f"GR3 action is missing {exc.args[0]}") from exc
    return GR3_ACTION_SCHEMA.coerce(values, name="GR3 action", ndim=1)


def action_dict(action: np.ndarray) -> dict[str, float]:
    vector = GR3_ACTION_SCHEMA.coerce(action, name="GR3 action", ndim=1)
    return dict(zip(ACTION_NAMES, map(float, vector), strict=True))


def build_observation(
    joints: Mapping[str, float],
    base_state: Mapping[str, Any],
    image: np.ndarray,
    instruction: str,
    *,
    timestamp_ns: int,
    sequence: int,
    robot_id: str,
    bgr: bool = True,
    rotate_180: bool = False,
    model_state_dim: int = GR3_STATE_SCHEMA.dim,
    metadata: Mapping[str, Any] | None = None,
) -> ObservationFrame:
    color = np.asarray(image)
    if color.ndim != 3 or color.shape[-1] != 3:
        raise ValueError(f"GR3 camera image must be HxWx3, got {color.shape}")
    if bgr:
        color = color[..., ::-1]
    if rotate_180:
        color = np.rot90(color, 2)
    if model_state_dim == GR3_JOINT_STATE_SCHEMA.dim:
        state_schema = GR3_JOINT_STATE_SCHEMA
        state = joint_state_vector(joints)
    elif model_state_dim == GR3_STATE_SCHEMA.dim:
        state_schema = GR3_STATE_SCHEMA
        state = state_vector(joints, base_state)
    else:
        raise ValueError("GR3 model state dimension must be 31 or 33")
    return ObservationFrame(
        state_schema=state_schema,
        state=state,
        images={"cam_head": np.ascontiguousarray(color, dtype=np.uint8)},
        instruction=instruction,
        timestamp_ns=timestamp_ns,
        sequence=sequence,
        robot_type="gr3",
        robot_id=robot_id,
        metadata={"camera_source": "camera_top", **dict(metadata or {})},
    )
