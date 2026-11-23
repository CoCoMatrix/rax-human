import json
from pathlib import Path

import numpy as np

from farther_core.gr3_dagger import (
    ACTION_DIM,
    ACTION_NAMES,
    BASE_ACTION_NAMES,
    JOINT_NAMES,
    STATE_DIM,
    STATE_NAMES,
    action_vector,
    canonical_robot_action,
)


def test_gr3_schema_and_runtime_contract_match():
    schema_path = Path(__file__).parents[3] / "schemas" / "gr3_dagger_v2.json"
    schema = json.loads(schema_path.read_text())

    assert STATE_DIM == schema["state_dim"] == 33
    assert ACTION_DIM == schema["action_dim"] == 37
    assert list(STATE_NAMES) == schema["state_order"]
    assert list(ACTION_NAMES) == schema["action_order"]


def test_robot_action_is_reordered_by_name_not_input_order():
    joints = dict.fromkeys(reversed(JOINT_NAMES), 0.0)
    joints["left_wrist_pitch_joint"] = 1.0
    joints["left_wrist_roll_joint"] = 2.0
    base = dict.fromkeys(reversed(BASE_ACTION_NAMES), 0.0)

    action = canonical_robot_action(joints, base)
    vector = action_vector(action)

    assert list(action) == list(ACTION_NAMES)
    assert vector[ACTION_NAMES.index("left_wrist_roll_joint")] == np.float32(2.0)
    assert vector[ACTION_NAMES.index("left_wrist_pitch_joint")] == np.float32(1.0)
