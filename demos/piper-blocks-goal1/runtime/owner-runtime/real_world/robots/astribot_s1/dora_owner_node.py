from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time
from typing import Any, Callable, Mapping

import yaml

from XPolicyLab.real_world.robots.astribot_s1.dagger import (
    AstribotS1DaggerOwnerRuntime,
    AstribotS1DaggerTeleop,
)
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
    ASTRIBOT_S1_STATE_SCHEMA,
    AUXILIARY_ACTION_NAMES,
    arm_joint_action_dict,
    full_body_action_vector,
    gripper_action_dict,
)
from XPolicyLab.real_world.robots.astribot_s1.owner import AstribotS1OwnerRuntime
from XPolicyLab.real_world.robots.astribot_s1.qnexo_bridge import QnexoSample
from XPolicyLab.real_world.robots.astribot_s1.rollout import (
    AstribotS1RolloutRecorder,
)
from XPolicyLab.real_world.robots.astribot_s1.sdk import AstribotS1SdkSession
from XPolicyLab.real_world.robots.astribot_s1.teleop import (
    build_qnexo_torso_retargeter,
)
from XPolicyLab.real_world.teleop import (
    effective_teleop_capabilities,
    validate_teleop_profile,
)


DEFAULT_PROFILE = Path(__file__).with_name("profile.yml")


def build_astribot_qnexo_payload(
    raw: Mapping[str, Any],
    parse_joystick_data: Callable[[int, int, int, int], Mapping[str, Any]],
) -> dict[str, Any]:
    """Keep GR3's raw QNexo transport while producing the S1 retarget input."""
    left_arm = list(map(float, raw["arm_joint_left_rad"][:7]))
    right_arm = list(map(float, raw["arm_joint_right_rad"][:7]))
    if len(left_arm) != 7 or len(right_arm) != 7:
        raise ValueError("QNexo raw arm payload must contain at least 7 joints per side")
    left = parse_joystick_data(*map(int, raw["joystick_left"]))
    right = parse_joystick_data(*map(int, raw["joystick_right"]))
    return {
        "left_arm_joints": left_arm,
        "right_arm_joints": right_arm,
        "left_trigger": float(left["trigger"]),
        "right_trigger": float(right["trigger"]),
        "left_joystick_x": float(left["x"]),
        "left_joystick_y": float(left["y"]),
        "right_joystick_x": float(right["x"]),
        "right_joystick_y": float(right["y"]),
    }


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _truthy(name: str, default: str = "0") -> bool:
    return os.getenv(name, default).strip().lower() in {"1", "true", "yes", "on"}


def _evaluation_task_id() -> str:
    task_id = os.getenv("DAQ_TASK_ID", "").strip()
    if task_id:
        return task_id
    if os.getenv("XPOLICY_EVAL_RUN_ID", "").strip():
        raise ValueError("RealEval Astribot run requires DAQ_TASK_ID")
    return "prototype"


def _named_vector_array(pa: Any, names: tuple[str, ...], values: Any) -> Any:
    """Encode robot vectors with the same named Arrow struct used by GR3."""
    return pa.StructArray.from_arrays(
        [pa.array(names), pa.array(values, type=pa.float32())],
        names=["name", "value"],
    )


def _named_vector(value: Any, names: tuple[str, ...]) -> list[float]:
    named = {
        str(item["name"].as_py()): float(item["value"].as_py())
        for item in value
    }
    if set(named) != set(names):
        raise ValueError("named robot action does not match Astribot schema")
    return [named[name] for name in names]


def configured_fixed_initial_pose(
    profile: Mapping[str, Any],
) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
    """Load the reviewed, immutable prepare pose from the robot profile."""
    initial = profile.get("prepare", {}).get("initial_pose")
    if not isinstance(initial, Mapping):
        raise ValueError("Astribot prepare.initial_pose must be configured")
    expected = {
        "torso",
        "arm_left",
        "gripper_left",
        "arm_right",
        "gripper_right",
        "head",
    }
    if set(initial) != expected:
        raise ValueError(
            "Astribot prepare.initial_pose must contain exactly "
            f"{sorted(expected)!r}"
        )
    try:
        auxiliary_values = [
            *map(float, initial["torso"]),
            *map(float, initial["head"]),
        ]
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "Astribot prepare.initial_pose torso/head must be numeric sequences"
        ) from exc
    if len(auxiliary_values) != len(AUXILIARY_ACTION_NAMES) or not all(
        math.isfinite(value) for value in auxiliary_values
    ):
        raise ValueError(
            "Astribot prepare.initial_pose must contain 4 finite torso and "
            "2 finite head values"
        )
    return (
        arm_joint_action_dict([*initial["arm_left"], *initial["arm_right"]]),
        dict(zip(AUXILIARY_ACTION_NAMES, auxiliary_values, strict=True)),
        gripper_action_dict(
            [initial["gripper_left"], initial["gripper_right"]]
        ),
    )


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
        from qnexo.exo import parse_joystick_data
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Astribot Dora owner requires dora-rs, pyarrow and the copied "
            "GR3 qnexo package"
        ) from exc

    if not _truthy("ASTRIBOT_SDK_CONNECT_ACKNOWLEDGED"):
        raise RuntimeError(
            "Astribot SDK construction is only available through the live launcher"
        )
    enable_write = _truthy("ENABLE_WRITE")

    profile_path = Path(os.getenv("ASTRIBOT_PROFILE", str(DEFAULT_PROFILE))).resolve()
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if profile.get("robot_type") != "astribot_s1":
        raise ValueError("ASTRIBOT_PROFILE is not an Astribot S1 profile")
    teleop_capabilities = validate_teleop_profile(profile)
    effective_capabilities = effective_teleop_capabilities(teleop_capabilities)

    instruction = os.getenv("TASK", "pick the tomato")
    if instruction != "pick the tomato":
        raise ValueError("the selected Astribot checkpoint requires 'pick the tomato'")
    robot_id = os.getenv("ROBOT_ID", "astribot-s1")
    report_root = Path(
        os.getenv("ROLLOUT_REPORT_DIR", str(profile["rollout"]["output_root"]))
    ).expanduser()
    if not report_root.is_absolute():
        report_root = Path(__file__).resolve().parents[3] / report_root
    source_timeout_ns = int(float(profile["teleop"]["source_timeout_ms"]) * 1e6)

    node = Node(os.getenv("DORA_NODE_ID", "astribot_owner"))
    session = AstribotS1SdkSession(
        frequency_hz=float(profile["sdk"]["frequency_hz"]),
        high_control_rights=_truthy("ASTRIBOT_HIGH_CONTROL_RIGHTS"),
    )
    owner = AstribotS1OwnerRuntime.from_profile(
        profile_path,
        session,
        enable_write=enable_write,
        robot_id=robot_id,
        instruction=instruction,
    )
    dagger_owner = AstribotS1DaggerOwnerRuntime(owner)
    recorder = AstribotS1RolloutRecorder(
        report_root,
        mode="hardware" if enable_write else "shadow",
        robot_id=robot_id,
        instruction=instruction,
        generation=1,
        profile_id=str(profile["profile_id"]),
        state_schema=ASTRIBOT_S1_STATE_SCHEMA,
        action_schema=ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
        rollout_id=os.getenv("ROLLOUT_ID") or None,
    )
    (
        home_arm_action,
        home_auxiliary_action,
        home_gripper_action,
    ) = configured_fixed_initial_pose(profile)
    startup_snapshot = session.snapshot()
    # The owner performs exactly one SDK state read per robot-rate tick. QNexo
    # and model events consume this cache instead of multiplying blocking SDK
    # reads beyond the vendor's configured 250 Hz host-control loop.
    latest_snapshot = startup_snapshot
    torso_retargeter = (
        build_qnexo_torso_retargeter(
            profile,
            torso_limits=session.joint_limits["astribot_torso"],
            torso_velocity_limits=session.joint_velocity_limits["astribot_torso"],
        )
        if teleop_capabilities["waist_pitch"]
        or teleop_capabilities["waist_yaw"]
        else None
    )
    teleop = AstribotS1DaggerTeleop(
        dagger_owner,
        joint_scales=profile["joint_teleop"]["joint_scales"],
        torso_retargeter=torso_retargeter,
        home_arm_action=home_arm_action,
        home_auxiliary_action=home_auxiliary_action,
        home_gripper_action=home_gripper_action,
        reset_joint_tolerance_rad=profile["joint_teleop"][
            "reset_joint_tolerance_rad"
        ],
        reset_auxiliary_tolerance_rad=profile["joint_teleop"][
            "reset_auxiliary_tolerance_rad"
        ],
        reset_gripper_tolerance_percent=profile["teleop"][
            "reset_gripper_tolerance_percent"
        ],
        reset_timeout_seconds=profile["joint_teleop"]["reset_timeout_seconds"],
        waist_pitch_enabled=teleop_capabilities["waist_pitch"],
        waist_yaw_enabled=teleop_capabilities["waist_yaw"],
        left_end_effector_enabled=teleop_capabilities["left_end_effector"],
        right_end_effector_enabled=teleop_capabilities["right_end_effector"],
    )
    latest_payload: dict[str, Any] | None = None
    latest_source_ns = 0
    takeover = False
    control_events: set[str] = set()
    takeover_events: set[str] = set()
    inference_chunks = 0
    model_steps = 0
    expert_steps = 0
    observations = 0
    hardware_writes = 0
    finalized = False
    last_status: str | None = None
    last_reset_status: str | None = None
    episode_index = -1
    recording = False
    latest_model_chunk_id = 0
    latest_model_horizon = 0
    latest_inference_started_ns = 0
    latest_inference_ended_ns = 0
    latest_alignment_info: dict[str, Any] = {}
    evaluation_task_id = _evaluation_task_id()

    def metadata(source_timestamp_ns: int | None = None) -> dict[str, Any]:
        now_ns = time.time_ns()
        source_ns = int(source_timestamp_ns or now_ns)
        return {
            "timestamp_ns": now_ns,
            "source_timestamp_ns": source_ns,
            "target_execution_timestamp_ns": now_ns,
            "hardware_write_enabled": enable_write,
        }

    def send(name: str, payload: Mapping[str, Any], source_ns: int = 0) -> None:
        node.send_output(
            name,
            pa.array([json.dumps(dict(payload), sort_keys=True, default=str)]),
            metadata(source_ns),
        )

    def send_scalar(
        name: str,
        value: Any,
        *,
        source_ns: int = 0,
        detail: Mapping[str, Any] | None = None,
    ) -> None:
        output_metadata = metadata(source_ns)
        if detail:
            output_metadata.update(dict(detail))
        node.send_output(name, pa.array([value]), output_metadata)

    def send_named_vector(
        name: str,
        names: tuple[str, ...],
        values: Any,
        *,
        source_ns: int,
        detail: Mapping[str, Any] | None = None,
    ) -> None:
        output_metadata = metadata(source_ns)
        if detail:
            output_metadata.update(dict(detail))
        node.send_output(
            name,
            _named_vector_array(pa, names, values),
            output_metadata,
        )

    def publish_episode_result(event: str) -> None:
        outcome = "UNKNOWN"
        send_scalar(
            "episode_result",
            json.dumps(
                {
                    "task_outcome": outcome,
                    "outcome_confirmed_by_operator": False,
                    "termination_reason": (
                        "operator_finish_pending_review"
                        if event == "finish"
                        else "operator_stop"
                    ),
                    "failure_reason": None,
                },
                sort_keys=True,
            ),
            source_ns=latest_source_ns,
        )

    def sample(*, control_event: str | None = None) -> QnexoSample:
        if latest_payload is None or latest_source_ns <= 0:
            raise RuntimeError("QNexo control arrived before the first raw sample")
        return QnexoSample(
            payload=latest_payload,
            timestamp_ns=latest_source_ns,
            operator_enabled=takeover,
            control_event=control_event,
        )

    def update_operator(*, control_event: str | None = None) -> None:
        nonlocal expert_steps, last_reset_status
        nonlocal episode_index, recording
        if latest_payload is None:
            send(
                "status",
                {"status": "WAITING_FOR_QNEXO", "event": control_event},
            )
            return
        result = teleop.update(
            sample(control_event=control_event),
            latest_snapshot,
        )

        def publish_model_control(decision: Any) -> None:
            if decision is None or decision.model_control is None:
                return
            send_scalar(
                "model_control",
                json.dumps(
                    {
                        "command": decision.model_control,
                        "generation": decision.generation,
                    },
                    sort_keys=True,
                ),
                source_ns=latest_source_ns,
                detail={"generation": decision.generation},
            )

        publish_model_control(result.control)
        publish_model_control(result.takeover)
        if result.transition is not None:
            transition = result.transition
            if transition.accepted:
                control_events.add(transition.event)
            recorder.record_lifecycle(
                "control_event",
                event=transition.event,
                accepted=transition.accepted,
                state=transition.state,
                reason=transition.reason,
            )
            send_scalar(
                "control_event",
                transition.event,
                source_ns=latest_source_ns,
                detail={
                    "accepted": transition.accepted,
                    "state": transition.state,
                    "reason": transition.reason,
                },
            )
            if transition.accepted and transition.event == "start":
                episode_index += 1
                recording = True
                send_scalar(
                    "episode_index",
                    episode_index,
                    source_ns=latest_source_ns,
                    detail={"task_id": evaluation_task_id},
                )
                recorder.record_lifecycle(
                    "episode_recording",
                    status="RECORDING_STARTED",
                    episode_index=episode_index,
                )
            if transition.accepted and transition.event in {"stop", "finish"}:
                if recording:
                    publish_episode_result(transition.event)
                    send_scalar("episode_index", -1, source_ns=latest_source_ns)
                    recording = False
                recorder.record_lifecycle(
                    "episode_recording",
                    status="ENDED_AUTO_RETAIN",
                    episode_index=episode_index,
                )
        if result.takeover is not None:
            event = str(result.takeover.event)
            takeover_events.add(event)
            recorder.record_lifecycle("takeover_event", event=event)
            send_scalar(
                "takeover_event",
                event,
                source_ns=latest_source_ns,
                detail={"mode": result.takeover.mode.value},
            )
        if result.expert is not None:
            expert_steps += 1
        if (
            result.reset_status is not None
            and result.reset_status != last_reset_status
        ):
            recorder.record_lifecycle(
                "joint_home_reset",
                status=result.reset_status,
                event=teleop.reset_event,
            )
            last_reset_status = result.reset_status

    try:
        camera_info = dagger_owner.start(
            activate_cameras=_truthy("ACTIVATE_CAMERAS", "1")
        )
        recorder.record_lifecycle(
            "astribot_dagger_started",
            camera_info=camera_info,
            graph_nodes=[
                "exo_driver_node",
                "qnexo_controls",
                "astribot_owner",
                "astribot_policy",
            ],
            hardware_write_enabled=enable_write,
            startup_pose_timestamp_ns=startup_snapshot.timestamp_ns,
            startup_joints=startup_snapshot.joints,
        )
        send(
            "status",
            {
                "status": "HARDWARE_STARTED" if enable_write else "SHADOW_STARTED",
                "camera_streams": list(profile["cameras"]["streams"]),
                "hardware_write_enabled": enable_write,
                "report_dir": str(recorder.rollout_dir),
            },
        )

        for event in node:
            if event is None or event.get("type") == "STOP":
                break
            if event.get("type") != "INPUT":
                continue
            event_id = str(event["id"])
            event_metadata = event.get("metadata", {})

            if event_id == "exo_data":
                raw = _scalar(event["value"])
                latest_payload = build_astribot_qnexo_payload(
                    raw,
                    parse_joystick_data,
                )
                latest_source_ns = int(
                    event_metadata.get("source_timestamp_ns")
                    or event_metadata.get("timestamp_ns")
                    or time.time_ns()
                )
                update_operator()
                continue
            if event_id == "button_command":
                update_operator(control_event=str(_scalar(event["value"])))
                continue
            if event_id == "takeover":
                pressed = bool(_scalar(event["value"]))
                if pressed != takeover:
                    takeover = pressed
                    update_operator()
                continue
            if event_id == "model_action":
                action = _named_vector(
                    event["value"],
                    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields,
                )
                generation_value = int(
                    event_metadata.get("model_generation")
                    or event_metadata.get("generation")
                    or 0
                )
                step_index = int(
                    event_metadata.get("model_chunk_step_index", 0)
                )
                dagger_owner.controller.update_snapshot(latest_snapshot)
                selected = dagger_owner.update_model_action(
                    action,
                    generation_value,
                    model_step_index=step_index,
                )
                if selected is not None:
                    model_steps += 1
                latest_model_chunk_id = int(
                    event_metadata.get("model_chunk_id", latest_model_chunk_id)
                )
                latest_alignment_info = dict(event_metadata)
                continue
            if event_id == "model_info":
                info = json.loads(str(_scalar(event["value"])))
                inference_chunks = max(
                    inference_chunks,
                    int(info.get("model_chunk_id", inference_chunks)),
                )
                latest_model_chunk_id = int(
                    info.get("model_chunk_id", latest_model_chunk_id)
                )
                latest_model_horizon = int(
                    info.get("model_action_horizon", latest_model_horizon)
                )
                latest_inference_started_ns = int(
                    info.get(
                        "inference_start_timestamp_ns",
                        latest_inference_started_ns,
                    )
                )
                latest_inference_ended_ns = int(
                    info.get(
                        "inference_end_timestamp_ns",
                        latest_inference_ended_ns,
                    )
                )
                latest_alignment_info = dict(info)
                send("model_info", info, latest_inference_ended_ns)
                continue
            if event_id == "model_status":
                status = json.loads(str(_scalar(event["value"])))
                model_status = str(status.get("status", ""))
                model_generation = int(status.get("generation", 0))
                if model_status in {"loading", "starved", "error", "paused"}:
                    dagger_owner.update_model_status(
                        model_status,
                        model_generation,
                    )
                continue
            if event_id != "tick":
                continue

            latest_snapshot = session.snapshot()
            snapshot = latest_snapshot
            # The SDK adapter timestamps a snapshot after its state reads. Take
            # the scheduler time afterwards so safety never sees future state.
            now_ns = max(time.time_ns(), snapshot.timestamp_ns)
            source_fresh = bool(
                latest_source_ns
                and 0 <= now_ns - latest_source_ns <= source_timeout_ns
            )
            tick = dagger_owner.tick(
                operator_enabled=source_fresh,
                now_ns=now_ns,
                snapshot=snapshot,
            )
            recorder.record_tick(tick)
            hardware_writes += int(tick.wrote)
            if tick.wrote and not enable_write:
                raise RuntimeError("Astribot shadow graph unexpectedly wrote hardware")
            if tick.status == "FAULT":
                raise RuntimeError(f"Astribot owner fault: {tick.reasons}")
            if tick.observation is not None:
                observations += 1
            # Publish the same named, asynchronous streams consumed by the GR3
            # recorder.  Robot dimensions and camera count remain profile data.
            if tick.observation is not None:
                observation = tick.observation
                observation_detail = {
                    "schema_version": "astribot_s1_dagger_v1",
                    "sequence": observation.sequence,
                    "robot_type": "astribot_s1",
                    "robot_id": robot_id,
                }
                send_named_vector(
                    "observation_state_robot",
                    ASTRIBOT_S1_STATE_SCHEMA.fields,
                    observation.state,
                    source_ns=observation.timestamp_ns,
                    detail=observation_detail,
                )
                for camera_name, image in observation.images.items():
                    image_metadata = {
                        **observation_detail,
                        "width": int(image.shape[1]),
                        "height": int(image.shape[0]),
                        "encoding": "rgb8",
                        "device_frame_index": observation.sequence,
                    }
                    node.send_output(
                        f"observation_image_{camera_name}",
                        pa.array(image.reshape(-1), type=pa.uint8()),
                        {**metadata(observation.timestamp_ns), **image_metadata},
                    )

            target = tick.action.target
            if target is not None:
                action_metadata = dict(tick.action.action_metadata or {})
                source = str(action_metadata.get("source", "UNKNOWN"))
                action_values = full_body_action_vector(target)
                action_source_ns = int(
                    tick.action.command_timestamp_ns or now_ns
                )
                common_action_metadata = {
                    "schema_version": "astribot_s1_dagger_v1",
                    "generation": tick.action.generation,
                    "model_generation": tick.action.generation,
                    "control_tick_id": action_metadata.get("control_tick_id"),
                    "selected_action_source": source,
                    "control_mode": source,
                    "intervention_id": int(
                        action_metadata.get("intervention_id", 0)
                    ),
                    "expert_action_valid": bool(
                        action_metadata.get("label_valid", source == "EXPERT")
                    ),
                    "train_mask": bool(
                        action_metadata.get("label_valid", source == "EXPERT")
                    ),
                    "model_chunk_id": latest_model_chunk_id,
                    "model_chunk_step_index": int(
                        action_metadata.get(
                            "model_step_index",
                            tick.action.chunk_step_index or 0,
                        )
                        or 0
                    ),
                    "model_action_horizon": latest_model_horizon,
                    "queue_depth": int(
                        latest_alignment_info.get("queue_depth", 0)
                    ),
                    "inference_start_timestamp_ns": latest_inference_started_ns,
                    "inference_end_timestamp_ns": latest_inference_ended_ns,
                    **latest_alignment_info,
                    "predicted_execution_timestamp_ns": action_source_ns,
                    "controller_adjustment_reason": ",".join(
                        map(str, action_metadata.get("adjustment_reasons", ()))
                    ),
                    "owner_dispatch": "EXACT_ROUTED_ACTION",
                }
                send_named_vector(
                    "action_executed_robot",
                    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields,
                    action_values,
                    source_ns=action_source_ns,
                    detail=common_action_metadata,
                )
                send_scalar(
                    "control_mode",
                    source,
                    source_ns=action_source_ns,
                    detail=common_action_metadata,
                )
                send_scalar(
                    "selected_action_source",
                    source,
                    source_ns=action_source_ns,
                    detail=common_action_metadata,
                )
                send_scalar(
                    "label_valid",
                    common_action_metadata["expert_action_valid"],
                    source_ns=action_source_ns,
                    detail=common_action_metadata,
                )
                if source == "MODEL":
                    send_named_vector(
                        "action_model_robot",
                        ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields,
                        action_values,
                        source_ns=action_source_ns,
                        detail=common_action_metadata,
                    )
                if source == "EXPERT":
                    proposed = action_metadata.get("expert_proposed_action")
                    aligned = action_metadata.get("expert_aligned_unclipped_action")
                    safe = action_metadata.get("expert_safe_action") or target
                    if proposed is not None:
                        send_named_vector(
                            "action_expert_proposed_robot",
                            ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields,
                            full_body_action_vector(proposed),
                            source_ns=action_source_ns,
                            detail=common_action_metadata,
                        )
                    if aligned is not None:
                        send_named_vector(
                            "action_expert_aligned_unclipped_robot",
                            ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields,
                            full_body_action_vector(aligned),
                            source_ns=action_source_ns,
                            detail=common_action_metadata,
                        )
                    send_named_vector(
                        "action_expert_safe_robot",
                        ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields,
                        full_body_action_vector(safe),
                        source_ns=action_source_ns,
                        detail=common_action_metadata,
                    )

            status_payload = {
                "status": tick.status,
                "reasons": tick.reasons,
                "teleop_contract": profile["teleop_contract"],
                "teleop_capabilities": effective_capabilities,
                "source_fresh": source_fresh,
                "control_state": teleop.lifecycle.state,
                "takeover": takeover,
                "observations": observations,
                "inference_chunks": inference_chunks,
                "model_steps": model_steps,
                "expert_steps": expert_steps,
                "hardware_write_count": hardware_writes,
                "recording": recording,
                "evidence_retention": "AUTO_RETAIN_ALL_EVAL",
                "resetting": teleop.resetting,
            }
            signature = json.dumps(status_payload, sort_keys=True, default=str)
            if signature != last_status:
                send("status", status_payload, latest_source_ns)
                last_status = signature

        passed = (
            {"prepare", "start"}.issubset(control_events)
            and {"takeover_request", "takeover_end"}.issubset(takeover_events)
            and observations > 0
            and inference_chunks > 0
            and model_steps > 0
            and expert_steps > 0
        )
        summary = recorder.finalize(
            outcome="pass" if passed else "timeout",
            termination_reason=(
                "astribot_dagger_complete"
                if passed
                else "astribot_dagger_incomplete"
            ),
        )
        finalized = True
        send(
            "model_info",
            {
                "status": "PASS" if passed else "INCOMPLETE",
                "summary": summary,
                "control_events": sorted(control_events),
                "takeover_events": sorted(takeover_events),
                "report_dir": str(recorder.rollout_dir),
            },
        )
    except KeyboardInterrupt:
        if not finalized:
            recorder.finalize(outcome="aborted", termination_reason="operator_interrupt")
            finalized = True
    except Exception as exc:
        if not finalized:
            recorder.finalize(
                outcome="fault",
                termination_reason=f"{type(exc).__name__}: {str(exc)[:160]}",
            )
            finalized = True
        raise
    finally:
        if not finalized:
            recorder.finalize(
                outcome="aborted",
                termination_reason="dora_owner_closed_without_result",
            )
        if enable_write:
            try:
                dagger_owner.owner.control(
                    "stop",
                    dagger_owner.controller.generation,
                )
            except Exception:
                pass
        try:
            dagger_owner.close()
        finally:
            session.close()


if __name__ == "__main__":
    main()
