import numpy as np
import pytest

from gr00t_rtc.client import JiaxinInferenceClient
from gr00t_rtc.mapping import (
    ACTION_DIM,
    JOINT_NAMES,
    base_action,
    build_jiaxin_observation,
    flatten_jiaxin_action_chunk,
    split_jiaxin_action_chunk,
)
from gr00t_rtc.rtc import RtcQueue, frozen_steps
from gr00t_rtc import node as rtc_node


def test_mapping_preserves_full_horizon():
    state = dict(zip(JOINT_NAMES, range(len(JOINT_NAMES)), strict=True))
    obs = build_jiaxin_observation(
        state,
        {
            "stand_pose": [31.0, 32.0, 0.0],
            "base": {
                "position": [0.0, 0.0, 0.835],
                "rpy": [0.0, 0.033, 0.0],
            },
        },
        np.zeros((4, 6, 3), dtype=np.uint8),
        "test",
        bgr=False,
    )
    assert obs["state"]["left_arm"].shape == (1, 1, 7)
    assert obs["state"]["left_arm"][0, 0].tolist() == [5, 6, 7, 8, 9, 10, 11]
    np.testing.assert_allclose(obs["state"]["base_state"][0, 0], [0.835, 0.033])

    chunk = np.arange(40 * ACTION_DIM, dtype=np.float32).reshape(40, ACTION_DIM)
    grouped = split_jiaxin_action_chunk(chunk)
    assert grouped["left_arm"][0].tolist() == [5, 6, 7, 8, 9, 10, 11]
    assert grouped["right_arm"][0].tolist() == [12, 13, 14, 15, 16, 17, 18]
    rebuilt = flatten_jiaxin_action_chunk(grouped)
    np.testing.assert_array_equal(rebuilt, chunk)
    assert base_action(chunk[0]) == {
        "vel_height": 31.0,
        "vel_pitch": 32.0,
        "base_yaw": 33.0,
        "vel_x": 34.0,
        "vel_y": 35.0,
        "vel_yaw": 36.0,
    }


def test_rtc_discards_steps_consumed_during_inference():
    queue = RtcQueue(trigger_steps=4)
    first = np.arange(10 * ACTION_DIM, dtype=np.float32).reshape(10, ACTION_DIM)
    queue.replace(first)
    for _ in range(6):
        assert queue.pop() is not None
    snapshot, pending = queue.begin_inference()
    assert len(snapshot) == 4
    assert queue.pop() is not None
    assert queue.pop() is not None

    replacement = np.full((10, ACTION_DIM), 7.0, dtype=np.float32)
    replacement[:, 0] = np.arange(10)
    assert queue.accept(replacement, pending) == 2
    step_index, action = queue.pop_with_index()
    assert step_index == 2
    assert action[0] == 2


def test_generation_and_starvation_are_safe():
    queue = RtcQueue()
    _, old = queue.begin_inference()
    queue.reset(2)
    assert queue.accept(np.ones((2, ACTION_DIM)), old) is None
    assert queue.pop() is None
    assert frozen_steps([150.0, 220.0], 30.0, 1, 12) == 8


def test_zero_trigger_disables_overlap_inference():
    queue = RtcQueue(trigger_steps=0)
    queue.replace(np.ones((2, ACTION_DIM), dtype=np.float32))
    assert not queue.should_infer
    assert queue.pop() is not None
    assert not queue.should_infer
    assert queue.pop() is not None
    assert queue.should_infer
    previous, _ = queue.begin_inference()
    assert previous.shape == (0, ACTION_DIM)


def test_jiaxin_request_contains_no_rtc_options():
    client = object.__new__(JiaxinInferenceClient)
    captured = {}
    expected = np.zeros((3, ACTION_DIM), dtype=np.float32)

    def request(endpoint, data):
        captured.update(endpoint=endpoint, data=data)
        return split_jiaxin_action_chunk(expected), {}

    client._request = request
    actual, _ = client.infer({"language": {"task": [["pick"]]}})

    assert captured == {
        "endpoint": "get_action",
        "data": {
            "observation": {"language": {"task": [["pick"]]}},
            "options": {"instruction": "pick"},
        },
    }
    np.testing.assert_array_equal(actual, expected)


def test_step_control_runs_exactly_one_chunk():
    assert rtc_node.control_state("step") == (True, True)
    assert rtc_node.control_state("run") == (True, False)
    assert rtc_node.control_state("bootstrap") == (True, False)
    assert rtc_node.control_state("pause") == (False, False)


def test_action_chunk_size_limits_model_output():
    chunk = np.arange(40 * ACTION_DIM, dtype=np.float32).reshape(40, ACTION_DIM)
    np.testing.assert_array_equal(rtc_node.limit_action_chunk(chunk, 20), chunk[:20])
    np.testing.assert_array_equal(rtc_node.limit_action_chunk(chunk, 40), chunk)
    with pytest.raises(ValueError, match="positive"):
        rtc_node.limit_action_chunk(chunk, 0)
