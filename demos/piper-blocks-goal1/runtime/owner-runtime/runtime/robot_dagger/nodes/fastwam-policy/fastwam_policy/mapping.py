from __future__ import annotations

from collections.abc import Mapping

import numpy as np


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

BASE_ACTION_NAMES = (
    "vel_height",
    "vel_pitch",
    "base_yaw",
    "vel_x",
    "vel_y",
    "vel_yaw",
)

ACTION_DIM = len(JOINT_NAMES)


def build_fastwam_observation(
    state: Mapping[str, float],
    base_state: Mapping,
    image: np.ndarray,
    task: str,
    *,
    bgr: bool = True,
) -> dict:
    try:
        joints = [state[name] for name in JOINT_NAMES]
        stand_pose = base_state["stand_pose"]
        values = joints + [stand_pose[0], stand_pose[1]]
    except (KeyError, IndexError, TypeError) as exc:
        raise ValueError(f"invalid GR3 state: {exc}") from exc

    image = np.asarray(image)
    if image.ndim != 3 or image.shape[-1] != 3:
        raise ValueError(f"FastWAM expects HxWx3 image, got {image.shape}")
    if bgr:
        image = image[..., ::-1]

    return {
        "video": {"top": np.ascontiguousarray(image, dtype=np.uint8)},
        "state": {"default": np.asarray(values, dtype=np.float32)},
        "language": {"task": [[str(task)]]},
    }


def flatten_fastwam_action_chunk(action: Mapping[str, np.ndarray]) -> np.ndarray:
    try:
        chunk = np.asarray(action["default"], dtype=np.float32)
    except (KeyError, TypeError) as exc:
        raise ValueError("FastWAM action must contain 'default'") from exc
    if chunk.ndim == 3:
        if chunk.shape[0] != 1:
            raise ValueError(f"FastWAM action expects batch size 1, got {chunk.shape}")
        chunk = chunk[0]
    if chunk.ndim != 2 or chunk.shape[1] != ACTION_DIM or not len(chunk):
        raise ValueError(f"FastWAM action expected non-empty (T,{ACTION_DIM}), got {chunk.shape}")
    if not np.isfinite(chunk).all():
        raise ValueError("FastWAM action contains non-finite values")
    return chunk


def joint_action(step: np.ndarray) -> dict[str, float]:
    step = np.asarray(step, dtype=np.float32)
    if step.shape != (ACTION_DIM,):
        raise ValueError(f"expected {ACTION_DIM} joint values, got {step.shape}")
    return dict(zip(JOINT_NAMES, map(float, step), strict=True))


def base_action() -> dict[str, float]:
    return dict.fromkeys(BASE_ACTION_NAMES, 0.0)
