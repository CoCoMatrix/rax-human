import math

from dagger_router.core import DaggerRouter, Decision, Mode
from dagger_router.node import decision_metadata


def test_episode_model_takeover_and_handback():
    router = DaggerRouter(max_joint_step=0.15)
    router.update_robot({"joint": 1.0})

    shadow = router.update_expert({"joint": 10.0})
    assert shadow.mode == Mode.HOLD
    assert shadow.action == {"joint": 1.15}
    assert shadow.expert_safe_action is None
    assert shadow.teleop_signal_valid
    assert not shadow.label_valid

    router.request_model("run")
    start = router.set_episode(True)
    assert (start.mode, start.model_control, start.generation) == (Mode.HOLD, "bootstrap", 1)
    assert router.update_model({"joint": 0.0}, 0) is None
    assert router.update_model({"joint": 1.1}, 1).mode == Mode.MODEL

    takeover = router.set_takeover(True)
    assert (
        takeover.mode,
        takeover.event,
        takeover.model_control,
        takeover.generation,
    ) == (Mode.HOLD, "takeover_request", "pause", 2)
    corrected = router.update_expert({"joint": 10.1})
    assert corrected.mode == Mode.EXPERT
    assert corrected.event == "takeover_start"
    assert corrected.teleop_signal_valid
    assert corrected.label_valid
    assert corrected.control_tick_id == 1
    assert math.isclose(corrected.action["joint"], 1.1)
    assert math.isclose(corrected.expert_aligned_unclipped_action["joint"], 1.1)
    assert router.set_takeover(True).intervention_id == 1
    assert router.set_episode(True).intervention_id == 1

    handback = router.set_takeover(False)
    assert (handback.mode, handback.model_control, handback.generation) == (
        Mode.HOLD,
        "bootstrap",
        3,
    )
    assert router.update_model({"joint": 1.2}, 2) is None
    assert router.update_model({"joint": 1.2}, 3).mode == Mode.MODEL

    end = router.set_episode(False)
    assert (end.mode, end.model_control) == (Mode.HOLD, "pause")


def test_takeover_uses_absolute_hand_targets_after_model_closes_hand():
    router = DaggerRouter(max_joint_step=0)
    hand = "L_index_proximal_joint"
    router.update_robot({"arm": 1.0, hand: -2.0})
    router.update_expert({"arm": 10.0, hand: 0.0})
    router.request_model("run")
    router.set_episode(True)
    router.update_model({"arm": 1.0, hand: -2.0}, router.generation)
    router.set_takeover(True)

    decision = router.update_expert({"arm": 10.0, hand: 0.0})

    assert decision.action["arm"] == 1.0
    assert decision.action[hand] == 0.0


def test_each_step_request_gets_one_new_model_generation():
    router = DaggerRouter()
    router.request_model("step")

    first = router.set_episode(True)
    assert (first.mode, first.model_control, first.generation) == (
        Mode.HOLD,
        "step",
        1,
    )

    assert router.request_model("step") is None
    assert router.generation == 1
    router.update_model_status("paused", 1)

    second = router.request_model("step")
    assert (second.mode, second.model_control, second.generation, second.event) == (
        Mode.HOLD,
        "step",
        2,
        "model_step",
    )


def test_step_mode_handback_waits_for_another_left_blue_click():
    router = DaggerRouter()
    router.request_model("step")
    router.set_episode(True)
    router.set_takeover(True)

    handback = router.set_takeover(False)

    assert handback.mode == Mode.HOLD
    assert handback.model_control is None


def test_left_blue_is_ignored_while_foot_takeover_is_held():
    router = DaggerRouter()
    router.request_model("step")
    router.set_episode(True)
    router.set_takeover(True)

    assert router.request_model("step") is None
    assert not router.model_busy
    router.set_takeover(False)

    next_step = router.request_model("step")
    assert next_step.model_control == "step"


def test_model_starvation_holds_without_repeating_action():
    router = DaggerRouter()
    router.set_episode(True)
    router.update_model({"joint": 0.0}, 1)
    decision = router.update_model_status("starved", 1)
    assert decision.mode == Mode.HOLD
    assert decision.action is None


def test_freeze_waist_holds_episode_start_angles():
    router = DaggerRouter(freeze_waist=True)
    waist = {
        "waist_yaw_joint": 0.1,
        "waist_roll_joint": 0.2,
        "waist_pitch_joint": 0.3,
    }
    router.update_robot({**waist, "joint": 1.0})
    router.set_episode(True)
    router.update_robot({name: value + 0.1 for name, value in waist.items()})

    decision = router.update_model(
        {name: -0.5 for name in waist} | {"joint": 1.1}, router.generation
    )
    action = decision.action
    assert {name: action[name] for name in waist} == waist
    assert "waist_frozen" in decision.adjustment_reasons


def test_freeze_selected_waist_joints_allows_yaw():
    router = DaggerRouter(
        max_joint_step=0,
        freeze_waist=True,
        frozen_waist_joints=("waist_roll_joint",),
    )
    router.update_robot(
        {
            "waist_yaw_joint": 0.1,
            "waist_roll_joint": 0.2,
            "waist_pitch_joint": 0.3,
        }
    )
    router.set_episode(True)

    action = router.update_model(
        {
            "waist_yaw_joint": 0.4,
            "waist_roll_joint": 0.5,
            "waist_pitch_joint": 0.6,
        },
        router.generation,
    ).action

    assert action == {
        "waist_yaw_joint": 0.4,
        "waist_roll_joint": 0.2,
        "waist_pitch_joint": 0.6,
    }


def test_freeze_waist_applies_to_model_not_expert():
    router = DaggerRouter(
        max_joint_step=0,
        freeze_waist=True,
        frozen_waist_joints=("waist_pitch_joint",),
    )
    router.update_robot({"waist_pitch_joint": 0.3})
    router.set_episode(True)
    router.update_expert({"waist_pitch_joint": 0.2})
    router.set_takeover(True)

    decision = router.update_expert({"waist_pitch_joint": 0.4})

    assert math.isclose(decision.action["waist_pitch_joint"], 0.5)
    assert math.isclose(
        decision.expert_aligned_unclipped_action["waist_pitch_joint"],
        0.5,
    )


def test_waist_joystick_delta_uses_measured_robot_pose_like_dual_arms():
    router = DaggerRouter(max_joint_step=0)
    router.update_robot({"waist_pitch_joint": 0.2})
    router.set_episode(True)
    router.update_expert({"waist_pitch_joint": 0.0})
    router.set_takeover(True)

    decision = router.update_expert({"waist_pitch_joint": 0.01})

    assert decision.mode == Mode.EXPERT
    assert math.isclose(decision.action["waist_pitch_joint"], 0.21)


def test_gr3_facade_preserves_base_disabled_reason():
    router = DaggerRouter(max_joint_step=0, execute_base=False)
    router.set_episode(True)

    decision = router.update_model({"joint": 0.0, "vel_x": 0.4}, router.generation)

    assert decision.action["vel_x"] == 0.0
    assert decision.adjustment_reasons == ("base_disabled",)


def test_router_action_metadata_identifies_the_control_mode():
    metadata = decision_metadata(
        Decision(Mode.HOLD, 6, control_tick_id=12),
        {"generation": 99, "source": "exo"},
    )

    assert metadata["generation"] == 6
    assert metadata["control_mode"] == "HOLD"
    assert metadata["selected_action_source"] == "HOLD"
    assert metadata["control_tick_id"] == 12
    assert metadata["source"] == "exo"


def test_shadow_teleop_is_debug_only_without_taking_control():
    router = DaggerRouter(max_joint_step=0)
    router.update_robot({"joint": 0.0})
    router.set_episode(True)
    router.update_model({"joint": 0.1}, router.generation)

    shadow = router.update_expert({"joint": 0.2})

    assert shadow.mode == Mode.MODEL
    assert shadow.action is None
    assert shadow.expert_proposed_action == {"joint": 0.2}
    assert shadow.expert_safe_action is None
    assert shadow.teleop_signal_valid
    assert not shadow.label_valid
    assert shadow.control_tick_id is None


def test_control_tick_ids_cover_only_selected_episode_actions():
    router = DaggerRouter(max_joint_step=0)
    router.update_robot({"joint": 0.0})
    router.update_expert({"joint": 10.0})
    router.set_episode(True)

    first = router.update_model({"joint": 0.1}, router.generation)
    second = router.update_model({"joint": 0.2}, router.generation)
    router.set_takeover(True)
    takeover = router.update_expert({"joint": 10.1})

    assert first.control_tick_id == 0
    assert second.control_tick_id == 1
    assert takeover.control_tick_id == 2

    router.set_episode(False)
    router.set_episode(True)
    restarted = router.update_model({"joint": 0.3}, router.generation)
    assert restarted.control_tick_id == 0


def test_model_lowpass_is_switchable_and_resets_after_takeover():
    plain = DaggerRouter(max_joint_step=0)
    plain.update_robot({"joint": 0.0})
    plain.set_episode(True)
    assert plain.update_model({"joint": 1.0}, plain.generation).action == {"joint": 1.0}

    filtered = DaggerRouter(max_joint_step=0, model_lowpass_alpha=0.5)
    filtered.update_robot({"joint": 0.0})
    filtered.set_episode(True)
    assert filtered.update_model({"joint": 1.0}, filtered.generation).action == {
        "joint": 0.5
    }
    assert filtered.update_model({"joint": 1.0}, filtered.generation).action == {
        "joint": 0.75
    }

    filtered.set_takeover(True)
    filtered.update_robot({"joint": 0.2})
    filtered.set_takeover(False)
    resumed = filtered.update_model({"joint": 1.0}, filtered.generation)
    assert math.isclose(resumed.action["joint"], 0.6)
