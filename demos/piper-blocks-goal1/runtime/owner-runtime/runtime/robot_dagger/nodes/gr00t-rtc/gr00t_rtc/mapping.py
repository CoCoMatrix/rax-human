from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from farther_core.gr3_dagger import (
    ACTION_DIM,
    BASE_ACTION_NAMES,
    JOINT_NAMES,
)


def _state_vector(state: Mapping[str, float], base_state: Mapping) -> np.ndarray:
    try:
        joints = [state[name] for name in JOINT_NAMES]
        robot_base = base_state["base"]
        base = [robot_base["position"][2], robot_base["rpy"][1]]
    except (KeyError, IndexError, TypeError) as exc:
        raise ValueError(f"invalid GR3 state: {exc}") from exc
    return np.asarray(joints + base, dtype=np.float32)


def build_jiaxin_observation(
    state: Mapping[str, float],
    base_state: Mapping,
    image: np.ndarray,
    task: str,
    *,
    bgr: bool = True,
    rotate_180: bool = False,
) -> dict:
    """Convert the verified Coconut GR3 contract to Jiaxin's batched schema."""
    values = _state_vector(state, base_state)
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[-1] != 3:
        raise ValueError(f"Jiaxin expects HxWx3 image, got {image.shape}")
    if bgr:
        image = image[..., ::-1]
    if rotate_180:
        image = np.rot90(image, 2)
    image = np.ascontiguousarray(image, dtype=np.uint8)

    def shaped(part):
        return np.asarray(part, dtype=np.float32).reshape(1, 1, -1)

    return {
        "video": {"top": image.reshape(1, 1, *image.shape)},
        "state": {
            "head": shaped(values[0:2]),
            "waist": shaped(values[2:5]),
            "left_arm": shaped(values[5:12]),
            "right_arm": shaped(values[12:19]),
            "left_hand_6dof": shaped(values[19:25]),
            "right_hand_6dof": shaped(values[25:31]),
            "base_state": shaped(values[31:33]),
        },
        "language": {"task": [[str(task)]]},
    }


def _time_rows(value, width: int, key: str) -> np.ndarray:
    value = np.asarray(value, dtype=np.float32)
    if value.ndim == 3:
        if value.shape[0] != 1:
            raise ValueError(f"Jiaxin action {key} expects batch size 1, got {value.shape}")
        value = value[0]
    elif value.ndim == 1:
        value = value.reshape(1, -1)
    if value.ndim != 2 or value.shape[1] != width:
        raise ValueError(f"Jiaxin action {key} expected (T,{width}), got {value.shape}")
    return value


def flatten_jiaxin_action_chunk(action: Mapping[str, np.ndarray]) -> np.ndarray:
    """Return every model step in canonical Dora order; never discard the horizon."""
    parts = [
        _time_rows(action["head"], 2, "head"),
        _time_rows(action["waist"], 3, "waist"),
        _time_rows(action["left_arm"], 7, "left_arm"),
        _time_rows(action["right_arm"], 7, "right_arm"),
        _time_rows(action["left_hand_6dof"], 6, "left_hand_6dof"),
        _time_rows(action["right_hand_6dof"], 6, "right_hand_6dof"),
        _time_rows(action["base_command"], 6, "base_command"),
    ]
    horizons = {part.shape[0] for part in parts}
    if len(horizons) != 1:
        raise ValueError(f"Jiaxin action horizons differ: {sorted(horizons)}")
    return np.concatenate(parts, axis=1)


def split_jiaxin_action_chunk(chunk: np.ndarray) -> dict[str, np.ndarray]:
    """Convert canonical physical actions back to Jiaxin groups for RTC inpainting."""
    chunk = np.asarray(chunk, dtype=np.float32)
    if chunk.ndim != 2 or chunk.shape[1] != ACTION_DIM:
        raise ValueError(f"expected (T,{ACTION_DIM}) action chunk, got {chunk.shape}")
    return {
        "head": chunk[:, 0:2],
        "waist": chunk[:, 2:5],
        "left_arm": chunk[:, 5:12],
        "right_arm": chunk[:, 12:19],
        "left_hand_6dof": chunk[:, 19:25],
        "right_hand_6dof": chunk[:, 25:31],
        "base_command": chunk[:, 31:37],
    }


def joint_action(step: np.ndarray) -> dict[str, float]:
    step = np.asarray(step, dtype=np.float32)
    if step.shape != (ACTION_DIM,):
        raise ValueError(f"expected {ACTION_DIM} action values, got {step.shape}")
    return dict(zip(JOINT_NAMES, map(float, step[: len(JOINT_NAMES)]), strict=True))


def base_action(step: np.ndarray) -> dict[str, float]:
    step = np.asarray(step, dtype=np.float32)
    if step.shape != (ACTION_DIM,):
        raise ValueError(f"expected {ACTION_DIM} action values, got {step.shape}")
    return dict(zip(BASE_ACTION_NAMES, map(float, step[len(JOINT_NAMES) :]), strict=True))
