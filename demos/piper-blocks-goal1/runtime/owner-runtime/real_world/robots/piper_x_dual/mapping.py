from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from XPolicyLab.real_world.contracts import VectorSchema


LEFT_JOINT_NAMES = tuple(f"left_joint{index}" for index in range(1, 7))
RIGHT_JOINT_NAMES = tuple(f"right_joint{index}" for index in range(1, 7))
JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
GRIPPER_NAMES = ("left_gripper", "right_gripper")
ACTION_NAMES = (
    *LEFT_JOINT_NAMES,
    "left_gripper",
    *RIGHT_JOINT_NAMES,
    "right_gripper",
)

PIPER_X_DUAL_STATE_SCHEMA = VectorSchema(
    schema_id="xpolicy.piper_x_dual.state.v1",
    fields=ACTION_NAMES,
    units=("rad",) * 6 + ("m",) + ("rad",) * 6 + ("m",),
    semantics=("absolute_joint",) * 6
    + ("absolute_gripper_width",)
    + ("absolute_joint",) * 6
    + ("absolute_gripper_width",),
)
PIPER_X_DUAL_ACTION_SCHEMA = VectorSchema(
    schema_id="xpolicy.piper_x_dual.joint_position.v1",
    fields=ACTION_NAMES,
    units=PIPER_X_DUAL_STATE_SCHEMA.units,
    semantics=PIPER_X_DUAL_STATE_SCHEMA.semantics,
)


def action_vector(values: Mapping[str, Any] | Sequence[float]) -> np.ndarray:
    if isinstance(values, Mapping):
        if set(values) != set(ACTION_NAMES):
            raise ValueError("invalid Piper X dual action schema")
        result = np.asarray([values[name] for name in ACTION_NAMES], dtype=float)
    else:
        result = np.asarray(values, dtype=float)
    if result.shape != (len(ACTION_NAMES),):
        raise ValueError("Piper X dual action must contain exactly 14 values")
    return result


def action_dict(values: Mapping[str, Any] | Sequence[float]) -> dict[str, float]:
    vector = action_vector(values)
    return {name: float(value) for name, value in zip(ACTION_NAMES, vector, strict=True)}
