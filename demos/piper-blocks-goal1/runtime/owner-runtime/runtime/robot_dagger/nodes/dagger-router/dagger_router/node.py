from __future__ import annotations

import json
import os
import time

import pyarrow as pa
from dora import Node
from farther_core.gr3_dagger import (
    BASE_ACTION_NAMES,
    CONTROLLER_VERSION,
    SCHEMA_VERSION,
)
from farther_core.serde import JointStateSerde

from .core import DaggerRouter, Decision, Mode, WAIST_JOINTS


ZERO_BASE = {
    "vel_height": 0.0,
    "vel_pitch": 0.0,
    "base_yaw": 0.0,
    "vel_x": 0.0,
    "vel_y": 0.0,
    "vel_yaw": 0.0,
}


def _scalar(value):
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _generation(event) -> int:
    value = event.get("metadata", {}).get("generation", -1)
    return int(value.as_py() if hasattr(value, "as_py") else value)


def decision_metadata(decision: Decision, source_metadata=None) -> dict:
    now_ns = time.time_ns()
    metadata = {"timestamp_ns": now_ns, "source_timestamp_ns": now_ns}
    if isinstance(source_metadata, dict):
        metadata.update(source_metadata)
    metadata.update(
        generation=decision.generation,
        control_mode=decision.mode.value,
        selected_action_source=(
            "SAFETY" if decision.mode.value == "SAFETY_STOP" else decision.mode.value
        ),
        intervention_id=decision.intervention_id,
        action_adjustment_reason=",".join(decision.adjustment_reasons),
        router_timestamp_ns=now_ns,
        controller_version=CONTROLLER_VERSION,
        schema_version=SCHEMA_VERSION,
    )
    if decision.control_tick_id is not None:
        metadata["control_tick_id"] = decision.control_tick_id
    return metadata


def main() -> None:
    node = Node()
    lowpass_enabled = os.getenv("ENABLE_MODEL_ACTION_LOWPASS", "0")
    if lowpass_enabled not in {"0", "1"}:
        raise ValueError("ENABLE_MODEL_ACTION_LOWPASS must be 0 or 1")
    frozen_waist_joints = tuple(
        name.strip()
        for name in os.getenv(
            "FREEZE_WAIST_JOINTS", ",".join(WAIST_JOINTS)
        ).split(",")
        if name.strip()
    )
    router = DaggerRouter(
        float(os.getenv("MAX_JOINT_STEP_RAD", "0.15")),
        os.getenv("FREEZE_WAIST", "0") == "1",
        (
            float(os.getenv("MODEL_ACTION_LOWPASS_ALPHA", "0.6"))
            if lowpass_enabled == "1"
            else None
        ),
        os.getenv("ENABLE_BASE_COMMAND", "0") == "1",
        frozen_waist_joints=frozen_waist_joints,
    )

    def emit(decision: Decision | None, source_metadata=None) -> None:
        if decision is None:
            return
        metadata = decision_metadata(decision, source_metadata)
        if decision.event or decision.model_control:
            print(
                "GR3_DAGGER_ROUTER",
                json.dumps(
                    {
                        "event": decision.event,
                        "model_control": decision.model_control,
                        "generation": decision.generation,
                        "control_mode": decision.mode.value,
                        "selected_action_source": metadata[
                            "selected_action_source"
                        ],
                        "teleop_signal_valid": decision.teleop_signal_valid,
                        "label_valid": decision.label_valid,
                        "intervention_id": decision.intervention_id,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
        if decision.model_control:
            node.send_output(
                "model_control",
                pa.array(
                    [
                        json.dumps(
                            {
                                "command": decision.model_control,
                                "generation": decision.generation,
                            }
                        )
                    ]
                ),
                metadata,
            )
        if decision.event:
            node.send_output("takeover_event", pa.array([decision.event]), metadata)
            node.send_output("control_event", pa.array([decision.event]), metadata)
        node.send_output("control_mode", pa.array([decision.mode.value]), metadata)
        node.send_output(
            "selected_action_source",
            pa.array([metadata["selected_action_source"]]),
            metadata,
        )
        node.send_output("label_valid", pa.array([decision.label_valid]), metadata)
        expert_metadata = {
            **metadata,
            "teleop_signal_valid": decision.teleop_signal_valid,
            "expert_action_valid": decision.label_valid,
            "expert_invalid_reason": (
                ""
                if decision.label_valid
                else (
                    "takeover_inactive"
                    if decision.teleop_signal_valid
                    else "teleop_signal_invalid"
                )
            ),
            "train_mask": decision.label_valid,
        }
        if decision.expert_proposed_action is not None:
            node.send_output(
                "expert_proposed_action",
                JointStateSerde.serialize(decision.expert_proposed_action),
                expert_metadata,
            )
        if decision.expert_aligned_unclipped_action is not None:
            node.send_output(
                "expert_aligned_unclipped_action",
                JointStateSerde.serialize(
                    decision.expert_aligned_unclipped_action
                ),
                expert_metadata,
            )
        if decision.expert_safe_action is not None:
            serialized = JointStateSerde.serialize(decision.expert_safe_action)
            node.send_output(
                "expert_safe_action",
                serialized,
                expert_metadata,
            )
            legacy_expert = {
                name: value
                for name, value in decision.expert_safe_action.items()
                if name not in BASE_ACTION_NAMES
            }
            node.send_output(
                "expert_action",
                JointStateSerde.serialize(legacy_expert),
                expert_metadata,
            )
        if decision.action is not None:
            execution_metadata = {
                **metadata,
                "target_execution_timestamp_ns": time.time_ns(),
                "train_mask": decision.mode.value == "EXPERT" and decision.label_valid,
            }
            joint_action = {
                name: value
                for name, value in decision.action.items()
                if name not in BASE_ACTION_NAMES
            }
            node.send_output("action", JointStateSerde.serialize(joint_action), execution_metadata)
            node.send_output(
                "action_robot",
                JointStateSerde.serialize(decision.action),
                execution_metadata,
            )
            if decision.mode is Mode.MODEL:
                node.send_output(
                    "model_action_robot",
                    JointStateSerde.serialize(decision.action),
                    execution_metadata,
                )
            base = {
                name: decision.action.get(name, ZERO_BASE[name])
                for name in BASE_ACTION_NAMES
            }
            node.send_output(
                "base_command",
                JointStateSerde.serialize(base),
                execution_metadata,
            )

    for event in node:
        if event.get("type") != "INPUT":
            continue
        event_id = event["id"]
        if event_id == "robot_state":
            router.update_robot(JointStateSerde.deserialize(event["value"]))
        elif event_id == "expert_action":
            emit(
                router.update_expert(JointStateSerde.deserialize(event["value"])),
                event.get("metadata"),
            )
        elif event_id == "model_action":
            emit(
                router.update_model(
                    JointStateSerde.deserialize(event["value"]), _generation(event)
                ),
                event.get("metadata"),
            )
        elif event_id == "takeover":
            emit(
                router.set_takeover(bool(_scalar(event["value"]))),
                event.get("metadata"),
            )
        elif event_id == "model_command":
            emit(router.request_model(str(_scalar(event["value"]))))
        elif event_id == "episode_index":
            emit(router.set_episode(int(_scalar(event["value"])) >= 0))
        elif event_id == "episode_failed":
            emit(router.set_episode(False))
        elif event_id == "model_status":
            status = json.loads(str(_scalar(event["value"])))
            emit(
                router.update_model_status(
                    str(status["status"]), int(status["generation"])
                ),
                event.get("metadata"),
            )


if __name__ == "__main__":
    main()
