from robot_adaptor.teleop_node import (
    begin_engage,
    configured_fixed_initial_pose,
    configured_gr3_driver,
    engaged_action_allowed,
    events_until_shutdown,
    finish_and_stop,
    finish_home_and_stop,
    prepare_allowed,
    routed_action_allowed,
    shutdown_teleop,
    split_gr3_action,
    step_allowed,
    start_allowed,
)


def test_gr3_fixed_initial_pose_requires_every_controlled_joint():
    pose = configured_fixed_initial_pose(
        {"prepare": {"initial_pose": {"joint_a": 1, "joint_b": -2.0}}},
        ("joint_a", "joint_b"),
    )
    assert pose == {"joint_a": 1.0, "joint_b": -2.0}


def test_gr3_fixed_initial_pose_rejects_implicit_zero_fill():
    import pytest

    with pytest.raises(ValueError, match=r"missing=\['joint_b'\]"):
        configured_fixed_initial_pose(
            {"prepare": {"initial_pose": {"joint_a": 1.0}}},
            ("joint_a", "joint_b"),
        )


def test_gr3_driver_and_initial_pose_are_one_profile_contract():
    profile = {
        "driver": {
            "controlled_joints": {
                "arm": ["joint_a"],
                "head": ["joint_b"],
            },
            "base_limits": {
                "lower": [-0.5, -0.3, 0.0],
                "upper": [0.04, 0.5, 0.0],
            },
        },
        "prepare": {"initial_pose": {"joint_a": 1.0, "joint_b": 2.0}},
    }

    groups, limits, pose = configured_gr3_driver(profile)

    assert groups == {"arm": ["joint_a"], "head": ["joint_b"]}
    assert limits["lower"] == [-0.5, -0.3, 0.0]
    assert pose == {"joint_a": 1.0, "joint_b": 2.0}


def test_gr3_driver_rejects_pose_that_does_not_match_group_membership():
    import pytest

    profile = {
        "driver": {
            "controlled_joints": {"arm": ["joint_a", "joint_b"]},
            "base_limits": {
                "lower": [-0.5, -0.3, 0.0],
                "upper": [0.04, 0.5, 0.0],
            },
        },
        "prepare": {"initial_pose": {"joint_a": 1.0}},
    }
    with pytest.raises(ValueError, match="missing"):
        configured_gr3_driver(profile)


def test_dora_stop_event_terminates_loop_and_runs_shutdown():
    calls = []
    first_input = {"type": "INPUT", "id": "tick"}
    ignored_input = {"type": "INPUT", "id": "action"}

    class Collection:
        state = "idle"

    class Client:
        def set_velocity(self, *_):
            calls.append("velocity")

        def set_fsm_state(self, _state):
            calls.append("fsm")

        def get_fsm_state(self):
            return "PdStand"

        def close(self):
            calls.append("close")

    class Machine:
        client = Client()

    events = [first_input, {"type": "STOP"}, ignored_input]

    assert list(events_until_shutdown(events, Machine(), Collection())) == [first_input]
    assert calls == ["velocity", "fsm", "close"]


def test_dora_shutdown_finishes_collection_enters_pdstand_and_closes_client():
    calls = []

    class Collection:
        state = "collect"

        def trigger(self, name):
            calls.append(("collection", name))

    class Client:
        def set_velocity(self, x, y, yaw):
            calls.append(("velocity", x, y, yaw))

        def set_fsm_state(self, state):
            calls.append(("fsm", state))

        def get_fsm_state(self):
            calls.append(("get_fsm",))
            return "PdStand"

        def close(self):
            calls.append(("close",))

    class Machine:
        client = Client()

    result = shutdown_teleop(Machine(), Collection(), timeout_seconds=0)

    assert result == {"status": "READY", "reasons": [], "pd_state": "PdStand"}
    assert calls == [
        ("collection", "next"),
        ("velocity", 0.0, 0.0, 0.0),
        ("fsm", 2),
        ("get_fsm",),
        ("close",),
    ]


def test_dora_shutdown_closes_client_when_pdstand_command_fails():
    calls = []

    class Collection:
        state = "idle"

    class Client:
        def set_velocity(self, *_):
            calls.append(("velocity",))

        def set_fsm_state(self, _state):
            calls.append(("fsm",))
            raise RuntimeError("sdk unavailable")

        def close(self):
            calls.append(("close",))

    class Machine:
        client = Client()

    result = shutdown_teleop(Machine(), Collection(), timeout_seconds=0)

    assert result["status"] == "HOLD"
    assert result["reasons"] == ["pdstand_command_failed:sdk unavailable"]
    assert calls == [("velocity",), ("fsm",), ("close",)]


def test_white_prepares_with_idle_pose():
    calls = []

    class Machine:
        def set_engage_target(self, target):
            calls.append(("target", target))

        def set_stand_pose(self, *pose):
            calls.append(("stand", pose))

        def trigger(self, name):
            calls.append(("trigger", name))

    begin_engage(Machine(), {"joint": 1.0})

    assert calls == [
        ("target", {"joint": 1.0}),
        ("stand", (0, 0, 0)),
        ("trigger", "ready"),
    ]


def test_preengage_prepares_aurora_without_repeating_idle_pose_motion():
    import robot_adaptor.state_machines as state_machines

    calls = []

    class Machine:
        state = "preengage"

        def prepare_for_engage(self):
            calls.append("prepare_aurora")

        def move_to_idle_pose(self):
            calls.append("move_to_idle_pose")

    state_machines.AuroraMachine.on_enter_preengage(Machine())

    assert calls == ["prepare_aurora"]


def test_white_and_blue_are_state_gated():
    assert prepare_allowed("idle")
    assert prepare_allowed("disengaged")
    assert not prepare_allowed("engaged_standing_one")
    assert start_allowed("engaged_standing_one", "idle")
    assert not start_allowed("engaging", "idle")
    assert not start_allowed("engaged_standing_one", "collect")


def test_left_blue_only_steps_after_prepare_and_outside_continuous_mode():
    assert step_allowed("engaged_standing_one", "idle", None, True)
    assert step_allowed("engaged_standing_one", "collect", "step", True)
    assert not step_allowed("engaged_standing_one", "idle", None, False)
    assert not step_allowed("idle", "idle", None, True)
    assert not step_allowed("engaged_standing_one", "collect", "run", True)


def test_red_finishes_episode_before_stopping_robot():
    calls = []

    class Collection:
        state = "collect"

        def trigger(self, name):
            calls.append(("collection", name))

    class Machine:
        def trigger(self, name):
            calls.append(("robot", name, self.preserve_pd_state_on_disengage))

        def pub_current_state(self):
            calls.append(("robot", "publish"))

    finish_and_stop(Machine(), Collection())

    assert calls == [
        ("collection", "next"),
        ("robot", "stop", True),
        ("robot", "publish"),
    ]


def test_left_red_finishes_homes_then_stops_robot():
    calls = []

    class Collection:
        state = "collect"

        def trigger(self, name):
            calls.append(("collection", name))

    class Machine:
        def move_to_idle_pose(self, duration):
            calls.append(("robot", "home", duration))

        def trigger(self, name):
            calls.append(("robot", name, self.preserve_pd_state_on_disengage))

        def pub_current_state(self):
            calls.append(("robot", "publish"))

    finish_home_and_stop(Machine(), Collection())

    assert calls == [
        ("collection", "next"),
        ("robot", "home", 2.0),
        ("robot", "stop", True),
        ("robot", "publish"),
    ]


def test_red_disengage_preserves_rlstand_but_fault_stop_uses_pdstand():
    import robot_adaptor.state_machines as state_machines

    calls = []

    class Machine:
        state = "disengaged"
        joint_positions = {}
        preserve_pd_state_on_disengage = True

        def set_state(self, state):
            calls.append(state)

    machine = Machine()
    state_machines.AuroraMachine.on_enter_disengaged(machine)
    assert calls == []
    assert not machine.preserve_pd_state_on_disengage

    state_machines.AuroraMachine.on_enter_disengaged(machine)
    assert calls == [state_machines.PdState.STAND]


def test_engaged_robot_ignores_actions_until_episode_starts():
    assert not engaged_action_allowed("idle")
    assert engaged_action_allowed("idle", allow_without_episode=True)
    assert engaged_action_allowed("collect")


def test_model_episode_rejects_manual_action_during_start_race():
    assert routed_action_allowed("MODEL", True)
    assert routed_action_allowed("EXPERT", True)
    assert not routed_action_allowed("MANUAL", True)
    assert not routed_action_allowed(None, True)
    assert routed_action_allowed(None, False)


def test_gr3_base_fields_are_not_sent_to_joint_controller():
    joints, base = split_gr3_action(
        {"head_yaw_joint": 0.1, "vel_x": 0.2, "vel_yaw": 0.3}
    )

    assert joints == {"head_yaw_joint": 0.1}
    assert base["vel_x"] == 0.2
    assert base["vel_yaw"] == 0.3


def test_preengage_only_advances_when_action_is_ready(monkeypatch):
    import robot_adaptor.state_machines as state_machines

    def fake_aurora_init(
        self,
        robot_name,
        _robot_id,
        _joints,
        base_limits,
        _pose,
        *,
        write_enabled,
    ):
        assert write_enabled is True
        self.robot_name = robot_name
        self.base_limits = base_limits
        self.frequency = 60

    monkeypatch.setattr(state_machines.AuroraWrapper, "__init__", fake_aurora_init)
    monkeypatch.setattr(state_machines.AuroraMachine, "on_enter_preengage", lambda self: None)
    monkeypatch.setattr(state_machines.AuroraMachine, "on_enter_engaged", lambda self: None)
    monkeypatch.setattr(
        state_machines.AuroraMachine, "on_enter_engaged_standing", lambda self: None
    )

    machine = state_machines.AuroraMachine(
        "gr3",
        115,
        {},
        {"lower": [0, 0, 0], "upper": [0, 0, 0]},
        {},
        write_enabled=True,
    )
    machine.trigger("next")
    assert machine.state == "preengage"

    machine.engage_target = object()
    machine.trigger("ready")
    assert machine.state == "engaging"
    assert machine.trigger("next") is False
    assert machine.state == "engaging"

    machine.trigger("engaged")
    assert machine.state == "engaged_standing_one"
