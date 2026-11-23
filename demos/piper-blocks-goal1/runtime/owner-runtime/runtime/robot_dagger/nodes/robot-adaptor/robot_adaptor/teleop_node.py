import argparse
import json
import math
import os
import time
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from pathlib import Path

import pyarrow as pa
import yaml
from dora import Node

from farther_core.gr3_dagger import (
    BASE_ACTION_NAMES,
    SCHEMA_VERSION,
    canonical_robot_action,
    canonical_robot_state,
)
from farther_core.serde import JointStateSerde
from robot_adaptor.aurora import PdState
from robot_adaptor.state_machines import AuroraMachine, DataCollectionMachine
from XPolicyLab.real_world.teleop import (
    TeleopButtonCoordinator,
    TeleopButtonResult,
    effective_teleop_capabilities,
    validate_teleop_profile,
)

try:
    from datetime import UTC
except ImportError:
    UTC = timezone.utc

ENABLE_BASE_COMMAND = os.getenv("ENABLE_BASE_COMMAND", "0") == "1"
ENABLE_MODEL_STEP = os.getenv("ENABLE_MODEL_STEP", "0") == "1"
REQUIRE_CONTROL_MODE = os.getenv("REQUIRE_CONTROL_MODE", "0") == "1"
ALLOW_ACTION_WITHOUT_EPISODE = os.getenv("ALLOW_ACTION_WITHOUT_EPISODE", "0") == "1"
FINISH_HOME_DURATION_SECONDS = float(os.getenv("FINISH_HOME_DURATION_SECONDS", "2.0"))


def strict_write_authorization() -> bool:
    value = os.getenv("GR3_ENABLE_WRITE", "0")
    if value not in {"0", "1"}:
        raise ValueError("GR3_ENABLE_WRITE must be exactly 0 or 1")
    return value == "1"


def configured_fixed_initial_pose(
    profile: Mapping[str, object],
    controlled_joint_names: Sequence[str],
) -> dict[str, float]:
    """Load one complete GR3 prepare target from the robot profile."""

    initial = profile.get("prepare", {})
    initial = initial.get("initial_pose") if isinstance(initial, Mapping) else None
    if not isinstance(initial, Mapping):
        raise ValueError("GR3 prepare.initial_pose must be configured")
    expected = tuple(controlled_joint_names)
    if len(expected) != len(set(expected)):
        raise ValueError("GR3 controlled joint names must be unique")
    if set(initial) != set(expected):
        missing = sorted(set(expected) - set(initial))
        extra = sorted(set(initial) - set(expected))
        raise ValueError(
            "GR3 prepare.initial_pose must contain exactly all controlled joints; "
            f"missing={missing!r}, extra={extra!r}"
        )
    pose: dict[str, float] = {}
    for name in expected:
        value = initial[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"GR3 initial pose for {name} must be numeric")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"GR3 initial pose for {name} must be finite")
        pose[name] = value
    return pose


def configured_gr3_driver(
    profile: Mapping[str, object],
) -> tuple[dict[str, list[str]], dict[str, list[float]], dict[str, float]]:
    """Load and cross-check the complete formal GR3 driver contract."""

    driver = profile.get("driver")
    if not isinstance(driver, Mapping):
        raise ValueError("GR3 profile.driver must be configured")
    groups = driver.get("controlled_joints")
    limits = driver.get("base_limits")
    if not isinstance(groups, Mapping) or not groups:
        raise ValueError("GR3 driver.controlled_joints must be configured")
    controlled: dict[str, list[str]] = {}
    ordered_names: list[str] = []
    for group, names in groups.items():
        if not isinstance(group, str) or not isinstance(names, list) or not names:
            raise ValueError("GR3 controlled joint groups must be non-empty lists")
        if any(not isinstance(name, str) or not name for name in names):
            raise ValueError("GR3 controlled joint names must be non-empty strings")
        controlled[group] = list(names)
        ordered_names.extend(names)
    if len(ordered_names) != len(set(ordered_names)):
        raise ValueError("GR3 controlled joint names must be unique")
    if not isinstance(limits, Mapping) or set(limits) != {"lower", "upper"}:
        raise ValueError("GR3 driver.base_limits requires lower and upper")
    base_limits: dict[str, list[float]] = {}
    for boundary in ("lower", "upper"):
        values = limits[boundary]
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError(f"GR3 base_limits.{boundary} must contain three values")
        parsed = []
        for value in values:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"GR3 base_limits.{boundary} must be numeric")
            parsed_value = float(value)
            if not math.isfinite(parsed_value):
                raise ValueError(f"GR3 base_limits.{boundary} must be finite")
            parsed.append(parsed_value)
        base_limits[boundary] = parsed
    if any(
        lower > upper
        for lower, upper in zip(
            base_limits["lower"], base_limits["upper"], strict=True
        )
    ):
        raise ValueError("GR3 base limit lower bounds must not exceed upper bounds")
    pose = configured_fixed_initial_pose(profile, ordered_names)
    return controlled, base_limits, pose


def split_gr3_action(action):
    joints = dict(action)
    base = {name: float(joints.pop(name, 0.0)) for name in BASE_ACTION_NAMES}
    return joints, base


def begin_engage(machine, target):
    machine.set_engage_target(target)
    machine.set_stand_pose(0, 0, 0)
    machine.trigger("ready")


def finish_and_stop(machine, collection):
    if collection.state.startswith("collect"):
        collection.trigger("next")
    machine.preserve_pd_state_on_disengage = True
    machine.trigger("stop")
    machine.pub_current_state()


def finish_home_and_stop(machine, collection):
    if collection.state.startswith("collect"):
        collection.trigger("next")
    machine.move_to_idle_pose(FINISH_HOME_DURATION_SECONDS)
    machine.preserve_pd_state_on_disengage = True
    machine.trigger("stop")
    machine.pub_current_state()


def shutdown_teleop(machine, collection, timeout_seconds=3.0, poll_seconds=0.05):
    """Finish collection, restore PdStand, and release the Aurora client.

    Red-button episode boundaries deliberately preserve RLStand. This function
    is only for the Dora node/process boundary, where GR3 must return to the
    startup-safe PdStand state and all SDK background threads must be closed.
    """
    reasons = []
    pd_state = None

    if collection.state.startswith("collect"):
        try:
            collection.trigger("next")
        except Exception as error:
            reasons.append(f"collection_stop_failed:{error}")

    try:
        machine.client.set_velocity(0.0, 0.0, 0.0)
        machine.client.set_fsm_state(PdState.STAND)

        deadline = time.monotonic() + max(0.0, timeout_seconds)
        while True:
            pd_state = machine.client.get_fsm_state()
            if pd_state == "PdStand" or time.monotonic() >= deadline:
                break
            time.sleep(max(0.0, poll_seconds))
        if pd_state != "PdStand":
            reasons.append(f"pdstand_not_reached:{pd_state}")
    except Exception as error:
        reasons.append(f"pdstand_command_failed:{error}")
    finally:
        try:
            machine.client.close()
        except Exception as error:
            reasons.append(f"client_close_failed:{error}")

    return {
        "status": "HOLD" if reasons else "READY",
        "reasons": reasons,
        "pd_state": pd_state,
    }


def events_until_shutdown(node, machine, collection):
    try:
        for event in node:
            if event is None or event.get("type") == "STOP":
                break
            yield event
    finally:
        result = shutdown_teleop(machine, collection)
        print("GR3_TELEOP_SHUTDOWN", json.dumps(result, sort_keys=True), flush=True)


def engaged_action_allowed(collection_state, allow_without_episode=False):
    return allow_without_episode or collection_state.startswith("collect")


def prepare_allowed(robot_state):
    return robot_state in {"idle", "disengaged"}


def start_allowed(robot_state, collection_state):
    return robot_state.startswith("engaged") and collection_state == "idle"


def step_allowed(robot_state, collection_state, model_mode, enabled):
    return enabled and robot_state.startswith("engaged") and (
        collection_state == "idle" or model_mode == "step"
    )


def routed_action_allowed(control_mode, required):
    return not required or control_mode in {"MODEL", "EXPERT"}


def publish_episode_result(node, task_outcome, confirmed, termination_reason):
    timestamp_ns = time.time_ns()
    node.send_output(
        "episode_result",
        pa.array(
            [
                json.dumps(
                    {
                        "task_outcome": task_outcome,
                        "outcome_confirmed_by_operator": confirmed,
                        "termination_reason": termination_reason,
                        "failure_reason": None,
                    }
                )
            ]
        ),
        {
            "timestamp_ns": timestamp_ns,
            "source_timestamp_ns": timestamp_ns,
            "target_execution_timestamp_ns": timestamp_ns,
        },
    )


def main():
    parser = argparse.ArgumentParser(description="Robot Teleop Node")

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="teleop",
    )
    parser.add_argument(
        "--robot-name",
        type=str,
        required=False,
        help="The name of the robot.",
        default="gr2",
    )
    parser.add_argument(
        "--robot-id",
        type=int,
        required=False,
        help="The ID of the robot.",
        default=135,
    )

    args = parser.parse_args()

    ROBOT_NAME = os.getenv("ROBOT_NAME", args.robot_name)
    ROBOT_ID = int(os.getenv("ROBOT_ID", args.robot_id))

    robot_profile_path = os.environ.get("ROBOT_PROFILE")
    if not robot_profile_path:
        raise RuntimeError("ROBOT_PROFILE is required for robot teleoperation")
    with open(robot_profile_path, encoding="utf-8") as profile_stream:
        robot_profile = yaml.safe_load(profile_stream)
    teleop_capabilities = validate_teleop_profile(robot_profile)
    effective_capabilities = effective_teleop_capabilities(teleop_capabilities)

    if ROBOT_NAME == "gr3":
        CONTROLLED_JOINTS, BASE_LIMITS, IDLE_POSE = configured_gr3_driver(
            robot_profile
        )
    else:
        # Legacy robot-adaptor variants keep their existing local configs until
        # they receive a robot-owned profile and formal graph.
        config_path = Path(__file__).parent / "configs" / f"{ROBOT_NAME}.yml"
        with open(config_path, encoding="utf-8") as config_stream:
            config = yaml.safe_load(config_stream)
        CONTROLLED_JOINTS = config["controlled_joints"]
        controlled_joint_names = [
            item for group in CONTROLLED_JOINTS.values() for item in group
        ]
        IDLE_POSE = dict.fromkeys(controlled_joint_names, 0.0)
        IDLE_POSE.update(config["idle_pose"])
        BASE_LIMITS = config["base_limits"]

    last_err = ""
    while True:
        try:
            node = Node(
                args.name,
            )  # provide the name to connect to the dataflow if dynamic node
            break
        except RuntimeError as err:
            if err != last_err:
                print(err)
                last_err = err
            print("Waiting for dataflow to be spawned")
            time.sleep(1)

    # c = FrequencyCounter(1)
    gr3_write_enabled = strict_write_authorization()
    if ROBOT_NAME == "gr3" and not gr3_write_enabled:
        raise PermissionError(
            "GR3 teleop refuses Aurora SDK connection without GR3_ENABLE_WRITE=1"
        )
    m = AuroraMachine(
        ROBOT_NAME,
        ROBOT_ID,
        CONTROLLED_JOINTS,
        BASE_LIMITS,
        IDLE_POSE,
        write_enabled=(gr3_write_enabled if ROBOT_NAME == "gr3" else True),
    )
    dc = DataCollectionMachine(node)
    buttons = TeleopButtonCoordinator()
    time.sleep(1)
    pedal_input = (0.0, 0.0, 0.0)
    base_cmd_dict = None
    model_mode = None
    last_robot_status_ns = 0

    def send_model_command(command, metadata):
        node.send_output(
            "model_command",
            pa.array([command]),
            metadata,
        )

    def apply_button_result(result: TeleopButtonResult, metadata):
        """Apply shared semantics through GR3-specific SDK/FSM operations."""
        nonlocal model_mode
        transition = result.transition
        print(
            "GR3_BUTTON_TRANSITION",
            json.dumps(
                {
                    "event": transition.event,
                    "accepted": transition.accepted,
                    "state": transition.state,
                    "generation": transition.generation,
                    "reason": transition.reason,
                    "replayed": result.replayed,
                    "cancelled_boundary": result.cancelled_boundary,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        if not transition.accepted:
            return

        command = transition.event
        if command == "prepare":
            m.trigger("next")
            begin_engage(m, IDLE_POSE)
            return
        if command == "start":
            dc.trigger("next")
            model_mode = "run"
            send_model_command(model_mode, metadata)
            print(f"Data Collection start: {dc.state=}, {dc.episode_index=}")
            return
        if command == "step":
            if dc.state == "idle":
                dc.trigger("next")
            model_mode = "step"
            send_model_command(model_mode, metadata)
            print(f"Model single-step: {dc.state=}, {dc.episode_index=}")
            return
        if command == "stop":
            model_mode = None
            publish_episode_result(node, "UNKNOWN", False, "operator_stop")
            finish_and_stop(m, dc)
            return
        if command == "finish":
            model_mode = None
            publish_episode_result(
                node,
                "UNKNOWN",
                False,
                "operator_finish_pending_review",
            )
            finish_home_and_stop(m, dc)
            replayed = buttons.complete_boundary(success=True)
            if replayed is not None:
                apply_button_result(replayed, metadata)

    def normalized_button_result(command):
        """Apply robot readiness as an extra gate around the common FSM."""
        if not buttons.boundary_active:
            if command == "prepare" and not prepare_allowed(m.state):
                return buttons.reject(command, "robot_not_ready")
            if command == "start" and not start_allowed(m.state, dc.state):
                return buttons.reject(command, "robot_not_ready")
            if command == "step" and not step_allowed(
                m.state,
                dc.state,
                model_mode,
                ENABLE_MODEL_STEP,
            ):
                return buttons.reject(command, "robot_not_ready")
        return buttons.handle(command)

    for event in events_until_shutdown(node, m, dc):
        if event["type"] != "INPUT":
            continue
        if event["id"] == "tick":
            q_dict = m.joint_positions
            base_state = m.base_state
            timestamp_ns = time.time_ns()
            node.send_output("joint_state", JointStateSerde.serialize(q_dict), {"timestamp_ns": timestamp_ns})
            node.send_output("base_state", pa.array([base_state]), {"timestamp_ns": timestamp_ns})
            if ROBOT_NAME == "gr3":
                state_robot = canonical_robot_state(q_dict, base_state)
                node.send_output(
                    "state_robot",
                    JointStateSerde.serialize(state_robot),
                    {
                        "timestamp_ns": timestamp_ns,
                        "source_timestamp_ns": timestamp_ns,
                        "target_execution_timestamp_ns": timestamp_ns,
                        "schema_version": SCHEMA_VERSION,
                        "state_dim": len(state_robot),
                    },
                )
            if timestamp_ns - last_robot_status_ns >= 250_000_000:
                last_robot_status_ns = timestamp_ns
                try:
                    pd_state = m.client.get_fsm_state()
                except Exception as error:
                    pd_state = f"unavailable: {error}"
                node.send_output(
                    "robot_status",
                    pa.array(
                        [
                            json.dumps(
                                {
                                    "fsm_state": m.state,
                                    "pd_state": pd_state,
                                    "collection_state": dc.state,
                                    "episode_index": dc.episode_index,
                                    "model_mode": model_mode,
                                    "control_state": buttons.lifecycle.state,
                                    "control_boundary": buttons.boundary_event,
                                    "teleop_contract": robot_profile[
                                        "teleop_contract"
                                    ],
                                    "teleop_capabilities": effective_capabilities,
                                }
                            )
                        ]
                    ),
                    {"timestamp_ns": timestamp_ns},
                )

        elif event["id"] == "action":
            if m.state.startswith("engaged"):
                if not engaged_action_allowed(dc.state, ALLOW_ACTION_WITHOUT_EPISODE):
                    continue
                control_mode = event.get("metadata", {}).get("control_mode")
                if hasattr(control_mode, "as_py"):
                    control_mode = control_mode.as_py()
                if not routed_action_allowed(control_mode, REQUIRE_CONTROL_MODE):
                    continue
                q_dict = JointStateSerde.deserialize(event["value"])
                action_base = {}
                if ROBOT_NAME == "gr3":
                    q_dict, action_base = split_gr3_action(q_dict)

                execution_metadata = dict(event.get("metadata", {}))
                executed = dict(q_dict)
                m.joint_positions = executed
                execution_metadata.update(
                    target_execution_timestamp_ns=time.time_ns(),
                    action_applied=True,
                    owner_dispatch="EXACT_ROUTED_ACTION",
                )
                node.send_output(
                    "executed_action",
                    JointStateSerde.serialize(
                        canonical_robot_action(executed, action_base)
                        if ROBOT_NAME == "gr3"
                        else executed
                    ),
                    execution_metadata,
                )

            elif m.state.startswith("engaging"):
                m.step_engage()
                if (
                    buttons.boundary_event == "prepare"
                    and not m.state.startswith("engaging")
                ):
                    replayed = buttons.complete_boundary(
                        success=m.state.startswith("engaged")
                    )
                    if replayed is not None:
                        apply_button_result(
                            replayed,
                            dict(event.get("metadata", {})),
                        )

        elif event["id"] == "base_pose":
            # c.update(event["id"])
            xyzquat = event["value"].to_pylist()

            base_cmd_dict = m.base_command(xyzquat)
            if base_cmd_dict is None:
                continue

            base_cmd_array = JointStateSerde.serialize(base_cmd_dict)
            node.send_output("base_command", base_cmd_array, {"timestamp_ns": int(datetime.now(UTC).timestamp() * 1e9)})

        elif event["id"] == "base_command":
            if not ENABLE_BASE_COMMAND or not m.state.startswith("engaged"):
                continue
            cmd = JointStateSerde.deserialize(event["value"])
            m.raw_base_command(cmd)
            node.send_output(
                "base_command",
                JointStateSerde.serialize(cmd),
                {"timestamp_ns": int(datetime.now(UTC).timestamp() * 1e9)},
            )

        elif event["id"] == "pedal":
            pedal_input = event["value"].to_pylist()
            # print(f"{pedal_input=} {m.state=}")
            base_cmd_dict = m.pd_command(*pedal_input)  # base_height, base_pitch, base_yaw, vel_x, vel_y, vel_yaw
            if base_cmd_dict is None:
                continue

            base_cmd_array = JointStateSerde.serialize(base_cmd_dict)
            node.send_output("base_command", base_cmd_array, {"timestamp_ns": int(datetime.now(UTC).timestamp() * 1e9)})

        elif event["id"] == "command":
            value = event["value"][0].as_py()
            print(f"COMMAND: {value=}")
            if value in TeleopButtonCoordinator.EVENTS:
                apply_button_result(
                    normalized_button_result(str(value)),
                    dict(event.get("metadata", {})),
                )
                continue
            if value in ["toggle_pd", "clutch", "abort", "reset"]:
                if value == "abort" and m.state.startswith("engaged"):
                    buttons.fail_closed()
                    publish_episode_result(node, "ABORTED", True, "operator_abort")
                    dc.trigger(value)
                    print(f"Data Collection {value}: {dc.state=}, {dc.episode_index=}")
                    continue
                m.trigger(value)
                m.pub_current_state()


if __name__ == "__main__":
    main()
