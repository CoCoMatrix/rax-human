from collections import deque

import numpy as np
import pytest

from fastwam_policy import node as policy_node
from fastwam_policy.client import FastWamInferenceClient
from fastwam_policy.mapping import (
    BASE_ACTION_NAMES,
    JOINT_NAMES,
    base_action,
    build_fastwam_observation,
    flatten_fastwam_action_chunk,
    joint_action,
)
from XPolicyLab.real_world.output import (
    resample_action_chunk,
    smooth_action_chunk,
)
from XPolicyLab.real_world.robots.gr3.output_node import advance_execution_grid
from XPolicyLab.real_world.robots.gr3.predictive_mink import (
    HAND_ACTION_INDICES,
    LOWER_BODY_JOINTS,
    MINK_ACTION_INDICES,
    MIMIC_JOINTS,
    PredictiveMinkProjector,
    ProjectionError,
    ProjectorConfig,
    projection_chunk_metrics,
    rate_limit_action,
    select_action,
)


EXPECTED_JOINT_NAMES = (
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


def test_observation_matches_fastwam_training_order():
    assert JOINT_NAMES == EXPECTED_JOINT_NAMES
    state = dict(zip(JOINT_NAMES, range(31), strict=True))
    image = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)

    observation = build_fastwam_observation(
        state,
        {"stand_pose": [31.0, 32.0, 0.0, 1.0]},
        image,
        "pick",
        bgr=True,
    )

    assert observation["state"]["default"].tolist() == list(range(33))
    assert observation["video"]["top"].shape == (1, 2, 3)
    assert observation["video"]["top"][0, 0].tolist() == [3, 2, 1]
    assert observation["language"]["task"] == [["pick"]]


def test_fastwam_action_is_absolute_canonical_joint_order():
    expected = np.arange(2 * 31, dtype=np.float32).reshape(2, 31)
    chunk = flatten_fastwam_action_chunk({"default": expected[np.newaxis, ...]})

    np.testing.assert_array_equal(chunk, expected)
    assert joint_action(chunk[1]) == dict(
        zip(JOINT_NAMES, map(float, expected[1]), strict=True)
    )
    assert tuple(base_action()) == BASE_ACTION_NAMES == (
        "vel_height",
        "vel_pitch",
        "base_yaw",
        "vel_x",
        "vel_y",
        "vel_yaw",
    )


def test_fastwam_action_rejects_invalid_or_nonfinite_output():
    with pytest.raises(ValueError, match="31"):
        flatten_fastwam_action_chunk({"default": np.zeros((2, 30))})

    invalid = np.zeros((1, 31), dtype=np.float32)
    invalid[0, 5] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        flatten_fastwam_action_chunk({"default": invalid})


def test_client_uses_native_fastwam_endpoints():
    client = object.__new__(FastWamInferenceClient)
    captured = []
    expected = np.zeros((2, 31), dtype=np.float32)

    def request(endpoint, data=None):
        captured.append((endpoint, data))
        if endpoint == "reset":
            return {"status": "ok"}
        return [{"default": expected[np.newaxis, ...]}, {"replan_steps": 32}]

    client._request = request
    observation = {"language": {"task": [["pick"]]}}

    chunk, info = client.infer(observation)
    assert info == {"replan_steps": 32}
    np.testing.assert_array_equal(chunk, expected)
    assert client.reset() == {"status": "ok"}
    assert captured == [
        (
            "get_action",
            {
                "observation": observation,
                "options": {"instruction": "pick", "action_space": "selected"},
            },
        ),
        ("reset", {"options": None}),
    ]


def test_client_sends_rtc_prefix_in_selected_action_space():
    client = object.__new__(FastWamInferenceClient)
    captured = []
    expected = np.zeros((32, 31), dtype=np.float32)

    def request(endpoint, data=None):
        captured.append((endpoint, data))
        return [{"default": expected[np.newaxis, ...]}, {}]

    client._request = request
    prefix = np.ones((16, 31), dtype=np.float32)
    client.infer(
        {"language": {"task": [["pick"]]}},
        rtc_prefix=prefix,
        rtc_inference_delay=10,
        rtc_prefix_horizon=16,
        rtc_prefix_attention_schedule="exp",
        rtc_max_guidance_weight=5.0,
    )

    options = captured[0][1]["options"]
    np.testing.assert_array_equal(options.pop("rtc_prev_action_chunk"), prefix)
    assert options == {
        "instruction": "pick",
        "action_space": "selected",
        "rtc_inference_delay": 10,
        "rtc_prefix_horizon": 16,
        "rtc_prefix_attention_schedule": "exp",
        "rtc_max_guidance_weight": 5.0,
    }


def test_step_control_and_chunk_limit_match_verified_behavior():
    assert policy_node.control_state("step") == (True, True)
    assert policy_node.control_state("bootstrap") == (True, False)
    assert policy_node.control_state("pause") == (False, False)

    chunk = np.arange(32 * 31, dtype=np.float32).reshape(32, 31)
    np.testing.assert_array_equal(policy_node.limit_action_chunk(chunk, 25), chunk[:25])
    np.testing.assert_array_equal(
        policy_node.limit_action_chunk(chunk, 32, require_exact=True),
        chunk,
    )
    with pytest.raises(ValueError, match="exactly 32"):
        policy_node.limit_action_chunk(chunk[:31], 32, require_exact=True)
    with pytest.raises(ValueError, match="positive"):
        policy_node.limit_action_chunk(chunk, 0)


def test_rtc_replaces_queue_after_actual_executed_delay():
    chunk = np.arange(32 * 31, dtype=np.float32).reshape(32, 31)
    remaining, delay = policy_node.replace_rtc_chunk(
        chunk,
        action_index_before_inference=16,
        action_index_now=26,
    )

    assert delay == 10
    np.testing.assert_array_equal(remaining, chunk[10:])


def test_rtc_replans_every_prefix_horizon_while_inference_overlaps_execution():
    old_chunk = np.zeros((32, 31), dtype=np.float32)
    new_chunk = np.ones((32, 31), dtype=np.float32)
    queue = list(old_chunk)
    action_index = 0

    for _ in range(16):
        queue.pop(0)
        action_index += 1
    request_index = action_index
    assert len(queue) == 16

    for _ in range(10):
        queue.pop(0)
        action_index += 1
    replacement, delay = policy_node.replace_rtc_chunk(
        new_chunk,
        action_index_before_inference=request_index,
        action_index_now=action_index,
    )
    queue = list(replacement)
    assert delay == 10
    assert len(queue) == 22

    for _ in range(6):
        queue.pop(0)
        action_index += 1
    assert len(queue) == 16
    assert action_index - request_index == 16


def test_single_chunk_qp_reduces_action_spike():
    chunk = np.zeros((7, 31), dtype=np.float32)
    chunk[3] = 1.0

    smoothed, info = smooth_action_chunk(
        chunk,
        previous_action=np.zeros(31, dtype=np.float32),
        acceleration_weight=1.0,
        overlap_weight=0.0,
    )

    assert smoothed.shape == chunk.shape
    assert info["overlap_steps"] == 0
    assert info["second_difference_after_rad"] < info["second_difference_before_rad"]


def test_rtc_qp_fuses_aligned_old_and_new_overlap():
    old_overlap = np.zeros((6, 31), dtype=np.float32)
    new_chunk = np.ones((22, 31), dtype=np.float32)

    fused, info = smooth_action_chunk(
        new_chunk,
        old_overlap=old_overlap,
        acceleration_weight=0.0,
        overlap_weight=4.0,
    )

    assert info["overlap_steps"] == 6
    np.testing.assert_allclose(fused[0], 0.2)
    np.testing.assert_allclose(fused[5:], 1.0)
    assert info["overlap_error_rms_after_rad"] < info["overlap_error_rms_before_rad"]


def test_pchip_high_rate_resampling_preserves_native_rtc_grid():
    previous = np.zeros(31, dtype=np.float32)
    chunk = np.zeros((32, 31), dtype=np.float32)
    chunk[:, 0] = np.sin(np.linspace(0.0, 2.0, len(chunk)))
    chunk[:, 1] = np.linspace(-0.5, 0.5, len(chunk))

    dense = resample_action_chunk(chunk, previous_action=previous, factor=3)

    assert dense.shape == (96, 31)
    np.testing.assert_allclose(dense[2::3], chunk, atol=1e-7)
    starts = np.vstack((previous, chunk[:-1]))
    for index in range(len(chunk)):
        interval = dense[index * 3 : (index + 1) * 3]
        lower = np.minimum(starts[index], chunk[index])
        upper = np.maximum(starts[index], chunk[index])
        assert np.all(interval >= lower - 1e-7)
        assert np.all(interval <= upper + 1e-7)

    # RTC stays on the native 30 Hz grid after 16 * 3 high-rate commands.
    np.testing.assert_allclose(dense[16 * 3 + 2 :: 3], chunk[16:], atol=1e-7)
    assert len(dense) / 90.0 == pytest.approx(len(chunk) / 30.0)


def test_high_rate_execution_leaves_native_rtc_prefix_after_16_steps():
    chunk = np.arange(32 * 31, dtype=np.float32).reshape(32, 31)
    native = deque(row.copy() for row in chunk)
    phase = 0
    completed = 0

    for _ in range(16 * 3):
        phase, native_step = advance_execution_grid(native, phase, 3)
        completed += native_step

    assert phase == 0
    assert completed == 16
    np.testing.assert_array_equal(np.stack(native), chunk[16:])


def test_pchip_resampling_rejects_invalid_inputs():
    chunk = np.zeros((2, 31), dtype=np.float32)
    with pytest.raises(ValueError, match="positive"):
        resample_action_chunk(chunk, previous_action=np.zeros(31), factor=0)
    with pytest.raises(ValueError, match="dimension"):
        resample_action_chunk(chunk, previous_action=np.zeros(30), factor=3)


def test_projector_config_rejects_invalid_per_joint_limits(monkeypatch):
    monkeypatch.setenv("PREDICTIVE_MINK_MAX_VELOCITY_RAD_S", "1,2")
    with pytest.raises(ValueError, match="one or 31"):
        ProjectorConfig.from_env()


def test_projector_config_reads_positive_action_tracking_cost(monkeypatch):
    monkeypatch.setenv("PREDICTIVE_MINK_ACTION_TRACKING_COST", "12.5")
    assert ProjectorConfig.from_env().posture_cost == 12.5

    monkeypatch.setenv("PREDICTIVE_MINK_ACTION_TRACKING_COST", "0")
    with pytest.raises(ValueError, match="positive"):
        ProjectorConfig.from_env()


def test_projector_config_reads_horizon_and_encoder_cost(monkeypatch):
    monkeypatch.setenv("PREDICTIVE_MINK_HORIZON_STEPS", "6")
    monkeypatch.setenv("PREDICTIVE_MINK_ENCODER_SYNC_COST", "1.5")

    config = ProjectorConfig.from_env()

    assert config.horizon_steps == 6
    assert config.encoder_sync_cost == 1.5


def test_gr3_model_mapping_locks_legs_and_applies_hand_mimic():
    projector = PredictiveMinkProjector(ProjectorConfig(solve_timeout_ms=1000.0))
    action = np.zeros(31)
    action[JOINT_NAMES.index("L_index_proximal_joint")] = -1.0
    action[JOINT_NAMES.index("L_thumb_proximal_pitch_joint")] = 0.5
    q = projector.model_q(action)

    assert len(projector.action_qpos) == 31
    for name in LOWER_BODY_JOINTS:
        address = projector.model.jnt_qposadr[projector.model.joint(name).id]
        assert q[address] == projector.model.qpos0[address]
    for follower, (leader, multiplier) in MIMIC_JOINTS.items():
        follower_address = projector.model.jnt_qposadr[
            projector.model.joint(follower).id
        ]
        leader_address = projector.model.jnt_qposadr[
            projector.model.joint(leader).id
        ]
        assert q[follower_address] == pytest.approx(q[leader_address] * multiplier)
    with pytest.raises(ValueError, match="31"):
        projector.model_q(np.zeros(30))


def test_gr3_model_mapping_rejects_missing_action_joint(tmp_path):
    model_path = tmp_path / "missing.xml"
    model_path.write_text(
        """
<mujoco>
  <worldbody>
    <body>
      <joint name="waist_yaw_joint" type="hinge" range="-1 1"/>
      <geom type="sphere" size="0.01" mass="1"/>
    </body>
  </worldbody>
</mujoco>
""".strip()
    )
    with pytest.raises(ValueError, match="missing action joints"):
        PredictiveMinkProjector(model_path=str(model_path))


def test_rate_projection_enforces_body_limits_and_bypasses_hands():
    config = ProjectorConfig(
        dt=0.1,
        max_velocity=np.ones(31),
        max_acceleration=np.full(31, 2.0),
        max_jerk=np.full(31, 10.0),
    )
    zero = np.zeros(31)
    projected = rate_limit_action(
        np.ones(31),
        zero,
        [zero, zero, zero],
        np.full(31, -10.0),
        np.full(31, 10.0),
        config,
    )

    np.testing.assert_allclose(projected[list(MINK_ACTION_INDICES)], 0.01, atol=1e-7)
    np.testing.assert_allclose(projected[list(HAND_ACTION_INDICES)], 1.0)
    velocity = (
        np.diff(
            np.vstack((zero, zero, zero, projected))[:, MINK_ACTION_INDICES],
            axis=0,
        )
        / config.dt
    )
    acceleration = np.diff(velocity, axis=0) / config.dt
    jerk = np.diff(acceleration, axis=0) / config.dt
    assert np.max(np.abs(velocity)) <= 1.0 + 1e-6
    assert np.max(np.abs(acceleration)) <= 2.0 + 1e-6
    assert np.max(np.abs(jerk)) <= 10.0 + 1e-5


def test_rate_projection_follows_projected_history_despite_encoder_lag():
    config = ProjectorConfig(
        dt=0.1,
        max_velocity=np.ones(31),
        max_acceleration=np.full(31, 2.0),
        max_jerk=np.full(31, 10.0),
    )
    history = [
        np.zeros(31),
        np.full(31, 0.01),
        np.full(31, 0.03),
    ]

    projected = rate_limit_action(
        np.full(31, 0.06),
        np.full(31, -0.1),
        history,
        np.full(31, -10.0),
        np.full(31, 10.0),
        config,
    )

    np.testing.assert_allclose(projected, 0.06, atol=1e-7)


def test_mink_projection_keeps_command_boundary_during_encoder_lag():
    projector = PredictiveMinkProjector(ProjectorConfig(solve_timeout_ms=1000.0))
    reference = np.zeros(31)
    joint_index = JOINT_NAMES.index("left_shoulder_pitch_joint")
    reference[joint_index] = 0.3
    history = [np.zeros(31)] * 3
    first, first_info = projector.project_step(reference, np.zeros(31), history)
    history = [*history, first][-3:]
    observed = np.zeros(31)
    observed[joint_index] = 0.1
    second, second_info = projector.project_step(reference, observed, history)

    assert first_info["status"] == second_info["status"] == "safe"
    jerk_limited_step = 80.0 * (1.0 / 30.0) ** 3
    assert first[joint_index] == pytest.approx(jerk_limited_step, abs=1e-7)
    assert second[joint_index] < observed[joint_index]
    assert second_info["encoder_tracking_max_error_rad"] == pytest.approx(
        0.1 - first[joint_index]
    )
    with pytest.raises(ProjectionError, match="stale"):
        projector.project_step(
            reference,
            observed,
            history,
            state_age_seconds=1.0,
        )


def test_mink_projection_passes_hand_reference_through():
    projector = PredictiveMinkProjector(ProjectorConfig(solve_timeout_ms=1000.0))
    reference = np.zeros(31)
    reference[list(HAND_ACTION_INDICES)] = np.linspace(-0.5, 0.5, 12)

    projected, info = projector.project_step(reference, np.zeros(31))

    assert len(MINK_ACTION_INDICES) == 19
    assert len(HAND_ACTION_INDICES) == 12
    np.testing.assert_array_equal(
        projected[list(HAND_ACTION_INDICES)],
        reference[list(HAND_ACTION_INDICES)].astype(np.float32),
    )
    assert info["status"] == "safe"


def test_valid_chunk_stays_close_and_finite():
    projector = PredictiveMinkProjector(ProjectorConfig(solve_timeout_ms=1000.0))
    chunk = np.zeros((4, 31))
    joint_index = JOINT_NAMES.index("left_shoulder_pitch_joint")
    chunk[:, joint_index] = np.arange(1, len(chunk) + 1) * 1e-4

    projected, info = projector.project_chunk(chunk, np.zeros(31))

    assert np.isfinite(projected).all()
    assert info["status"] == "safe"
    assert info["tracking_rmse_rad"] < 1e-5


def test_horizon_tracks_requested_upper_body_joint_motion():
    config = ProjectorConfig(
        max_velocity=np.full(31, 100.0),
        max_acceleration=np.full(31, 1000.0),
        max_jerk=np.full(31, 10000.0),
        solve_timeout_ms=1000.0,
    )
    projector = PredictiveMinkProjector(config)
    reference = np.zeros(31)
    targets = {
        "waist_yaw_joint": 0.25,
        "left_shoulder_pitch_joint": 0.5,
        "left_shoulder_roll_joint": 0.3,
        "left_elbow_pitch_joint": -0.7,
        "right_shoulder_pitch_joint": -0.4,
        "right_shoulder_roll_joint": -0.3,
        "right_elbow_pitch_joint": -0.6,
    }
    for name, value in targets.items():
        reference[JOINT_NAMES.index(name)] = value

    projected, _ = projector.project_horizon(
        np.tile(reference, (config.horizon_steps, 1)),
        np.zeros(31),
        [np.zeros(31)] * 3,
    )

    waist_index = JOINT_NAMES.index("waist_yaw_joint")
    assert projected[-1, waist_index] > 0.15
    assert abs(projected[-1, waist_index] - reference[waist_index]) < 0.1
    for name, value in targets.items():
        if name != "waist_yaw_joint":
            assert projected[-1, JOINT_NAMES.index(name)] == pytest.approx(
                value,
                abs=1e-6,
            )


def test_horizon_does_not_modify_unrequested_joints():
    config = ProjectorConfig(solve_timeout_ms=1000.0)
    projector = PredictiveMinkProjector(config)
    reference = np.zeros(31)
    targets = {
        "left_shoulder_pitch_joint": 0.5,
        "left_shoulder_roll_joint": 0.3,
        "left_elbow_pitch_joint": -0.7,
        "right_shoulder_pitch_joint": -0.4,
        "right_shoulder_roll_joint": -0.3,
        "right_elbow_pitch_joint": -0.6,
    }
    for name, value in targets.items():
        reference[JOINT_NAMES.index(name)] = value

    projected, _ = projector.project_horizon(
        np.tile(reference, (config.horizon_steps, 1)),
        np.zeros(31),
        [np.zeros(31)] * 3,
    )

    unrequested = [
        index for index, name in enumerate(JOINT_NAMES) if name not in targets
    ]
    np.testing.assert_allclose(projected[:, unrequested], 0.0, atol=1e-5)


def test_horizon_reports_fk_error_from_joint_projection():
    config = ProjectorConfig(
        max_velocity=np.full(31, 100.0),
        max_acceleration=np.full(31, 1000.0),
        max_jerk=np.full(31, 10000.0),
        solve_timeout_ms=1000.0,
    )
    projector = PredictiveMinkProjector(config)
    reference = np.zeros(31)
    reference[JOINT_NAMES.index("left_shoulder_pitch_joint")] = 0.5
    reference[JOINT_NAMES.index("left_elbow_pitch_joint")] = -0.7
    reference[JOINT_NAMES.index("right_shoulder_pitch_joint")] = -0.4
    reference[JOINT_NAMES.index("right_elbow_pitch_joint")] = -0.6

    projected, info = projector.project_horizon(
        np.tile(reference, (config.horizon_steps, 1)),
        np.zeros(31),
        [np.zeros(31)] * 3,
    )

    error = info["horizon_end_effector_error"]
    assert error["left"]["position_m"]["final"] < 1e-6
    assert error["right"]["position_m"]["final"] < 1e-6
    assert error["left"]["orientation_rad"]["final"] < 1e-6
    assert error["right"]["orientation_rad"]["final"] < 1e-6
    assert info["end_effector_error_definition"] == (
        "FK(q_optimized) relative to FK(q_action_chunk) at the same node"
    )
    projector.configuration.update(projector.model_q(projected[-1], clip=False))
    projector.target_configuration.update(
        projector.model_q(reference, clip=False)
    )
    optimized_position = projector.configuration.get_transform_frame_to_world(
        "left_hand_roll_link", "body"
    ).as_matrix()[:3, 3]
    reference_position = (
        projector.target_configuration.get_transform_frame_to_world(
            "left_hand_roll_link", "body"
        )
        .as_matrix()[:3, 3]
    )
    assert error["left"]["position_m"]["final"] == pytest.approx(
        np.linalg.norm(optimized_position - reference_position)
    )


def test_horizon_brakes_before_future_joint_limit_and_keeps_hard_rates():
    projector = PredictiveMinkProjector(
        ProjectorConfig(
            max_velocity=np.full(31, 1.0),
            max_acceleration=np.full(31, 3.0),
            max_jerk=np.full(31, 30.0),
            solve_timeout_ms=1000.0,
        )
    )
    joint_index = JOINT_NAMES.index("left_shoulder_pitch_joint")
    q_enc = np.zeros(31)
    q_enc[joint_index] = 2.925
    history = []
    for value in (2.905, 2.915, 2.925):
        action = q_enc.copy()
        action[joint_index] = value
        history.append(action)
    chunk = np.tile(q_enc, (projector.config.horizon_steps, 1))
    chunk[:, joint_index] = projector.position_upper[joint_index]

    projected, _ = projector.project_horizon(chunk, q_enc, history)

    greedy_first = q_enc[joint_index] + 0.3 * projector.config.dt
    assert projected[0, joint_index] < greedy_first
    trajectory = np.concatenate(
        ([2.905, 2.915, 2.925], projected[:, joint_index])
    )
    velocity = np.diff(trajectory) / projector.config.dt
    acceleration = np.diff(velocity) / projector.config.dt
    jerk = np.diff(acceleration) / projector.config.dt
    assert np.max(np.abs(velocity)) <= np.max(projector.config.max_velocity) + 1e-5
    assert np.max(np.abs(acceleration)) <= np.max(projector.config.max_acceleration) + 1e-5
    assert np.max(np.abs(jerk)) <= np.max(projector.config.max_jerk) + 3e-2
    assert np.max(projected[:, joint_index]) <= (
        projector.position_upper[joint_index] + 1e-7
    )


def test_horizon_remains_feasible_near_narrow_waist_limit():
    projector = PredictiveMinkProjector(
        ProjectorConfig(solve_timeout_ms=1000.0)
    )
    joint_index = JOINT_NAMES.index("waist_yaw_joint")
    q = np.zeros(31, dtype=np.float32)
    history = [q.copy(), q.copy(), q.copy()]

    for _ in range(20):
        references = np.tile(q, (projector.config.horizon_steps, 1))
        references[:, joint_index] = projector.position_upper[joint_index]
        projected, _ = projector.project_horizon(references, q, history)
        q = projected[0]
        history = (history + [q.copy()])[-3:]

    projected, info = projector.project_horizon(references[:1], q, history)

    assert np.isfinite(projected).all()
    assert info["planning_steps"] == projector.config.horizon_steps
    assert q[joint_index] > 0.0


def test_horizon_passes_every_hand_reference_through():
    projector = PredictiveMinkProjector(ProjectorConfig(solve_timeout_ms=1000.0))
    chunk = np.zeros((projector.config.horizon_steps, 31))
    chunk[:, list(HAND_ACTION_INDICES)] = np.linspace(
        -0.5,
        0.5,
        projector.config.horizon_steps * len(HAND_ACTION_INDICES),
    ).reshape(projector.config.horizon_steps, len(HAND_ACTION_INDICES))

    projected, _ = projector.project_horizon(
        chunk,
        np.zeros(31),
        [np.zeros(31)] * 3,
    )

    np.testing.assert_array_equal(
        projected[:, list(HAND_ACTION_INDICES)],
        chunk[:, list(HAND_ACTION_INDICES)].astype(np.float32),
    )


def test_horizon_rejects_solve_timeout():
    projector = PredictiveMinkProjector(
        ProjectorConfig(solve_timeout_ms=1e-9)
    )
    with pytest.raises(ProjectionError, match="exceeded"):
        projector.project_horizon(
            np.zeros((projector.config.horizon_steps, 31)),
            np.zeros(31),
            [np.zeros(31)] * 3,
        )


def test_projection_chunk_metrics_compare_tracking_and_smoothness():
    raw = np.zeros((4, 31))
    raw[:, 0] = (0.0, 0.1, 0.3, 0.6)
    raw[:, HAND_ACTION_INDICES[0]] = (0.0, 10.0, -10.0, 10.0)
    projected = np.zeros((4, 31))
    projected[:, 0] = (0.0, 0.05, 0.15, 0.3)
    projected[:, HAND_ACTION_INDICES[0]] = raw[:, HAND_ACTION_INDICES[0]]
    info = [
        {
            "status": "safe",
            "solve_ms": value,
            "objective_build_ms": value / 5.0,
            "qp_solver_ms": value * 0.75,
            "state_age_ms": value * 2.0,
            "left_position_error_m": value / 1000.0,
            "left_orientation_error_rad": value / 100.0,
            "right_position_error_m": value / 500.0,
            "right_orientation_error_rad": value / 50.0,
        }
        for value in (1.0, 2.0, 3.0, 4.0)
    ]

    metrics = projection_chunk_metrics(raw, list(projected), info, 0.1)

    assert metrics["projected_success_steps"] == 4
    assert metrics["projected_failure_steps"] == 0
    assert metrics["tracking_max_error_rad"] == pytest.approx(0.3)
    assert metrics["tracking_worst_joint"] == "head_yaw_joint"
    assert metrics["raw_smoothness"]["peak_velocity_rad_s"] == pytest.approx(3.0)
    assert metrics["projected_smoothness"]["peak_velocity_rad_s"] == pytest.approx(
        1.5
    )
    assert metrics["solve_ms_mean"] == pytest.approx(2.5)
    assert metrics["solve_ms_p95"] == pytest.approx(3.85)
    assert metrics["objective_build_ms_mean"] == pytest.approx(0.5)
    assert metrics["objective_build_ms_max"] == pytest.approx(0.8)
    assert metrics["qp_solver_ms_mean"] == pytest.approx(1.875)
    assert metrics["qp_solver_ms_max"] == pytest.approx(3.0)
    assert metrics["left_end_effector_error"]["position_m"] == {
        "mean": pytest.approx(0.0025),
        "max": pytest.approx(0.004),
        "final": pytest.approx(0.004),
    }
    assert metrics["right_end_effector_error"]["orientation_rad"]["max"] == (
        pytest.approx(0.08)
    )

    failed = info.copy()
    failed[-1] = {"status": "error", "fallback_reason": "infeasible"}
    metrics = projection_chunk_metrics(
        raw, [*projected[:3], None], failed, 0.1
    )
    assert metrics["projected_failure_steps"] == 1
    assert metrics["projected_smoothness"] is None
    assert metrics["fallback_reasons"] == {"infeasible": 1}


def test_projector_modes_keep_shadow_raw_and_require_safe_enforce():
    raw = np.zeros(31)
    projected = np.ones(31)
    np.testing.assert_array_equal(select_action("off", raw, None), raw)
    np.testing.assert_array_equal(select_action("shadow", raw, projected), raw)
    np.testing.assert_array_equal(select_action("enforce", raw, projected), projected)
    with pytest.raises(ProjectionError, match="safe projected"):
        select_action("enforce", raw, None)


def test_command_history_remains_feasible_during_encoder_lag():
    projector = PredictiveMinkProjector(ProjectorConfig(solve_timeout_ms=1000.0))
    q_enc = np.zeros(31)
    history = deque([q_enc.copy() for _ in range(3)], maxlen=3)
    references = np.zeros((projector.config.horizon_steps, 31))
    references[:, JOINT_NAMES.index("left_shoulder_pitch_joint")] = 0.3

    emitted = [q_enc.copy(), q_enc.copy(), q_enc.copy()]
    for _ in range(12):
        projected, info = projector.project_horizon(references, q_enc, history)
        history.append(projected[0].copy())
        emitted.append(projected[0].copy())

    assert info["status"] == "safe"
    trajectory = np.asarray(emitted)[:, list(MINK_ACTION_INDICES)]
    velocity = np.diff(trajectory, axis=0) / projector.config.dt
    acceleration = np.diff(velocity, axis=0) / projector.config.dt
    jerk = np.diff(acceleration, axis=0) / projector.config.dt
    assert np.max(np.abs(velocity)) <= np.max(projector.config.max_velocity) + 1e-5
    assert np.max(np.abs(acceleration)) <= np.max(projector.config.max_acceleration) + 1e-5
    assert np.max(np.abs(jerk)) <= np.max(projector.config.max_jerk) + 3e-2
