from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

import pyarrow as pa
from dora import Node

from .mapping import ACTION_NAMES, action_dict
from .mink_retarget import (
    DualPiperMinkIK,
    PiperIncrementalMinkRetargeter,
    PiperMinkConfig,
)
from .profile import load_profile

DEFAULT_PROFILE = Path(__file__).with_name("profile.yml")


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _named_vector(value: Any) -> dict[str, float]:
    named = {
        str(item["name"].as_py()): float(item["value"].as_py()) for item in value
    }
    return action_dict(named)


def _named_vector_array(action: Any) -> Any:
    values = action_dict(action)
    return pa.StructArray.from_arrays(
        [
            pa.array(ACTION_NAMES),
            pa.array([values[name] for name in ACTION_NAMES], type=pa.float64()),
        ],
        names=["name", "value"],
    )


def main() -> None:
    # Always keep the QNexo FK/IK stream alive.  The owner selects between this
    # stream and the SDK leader/follower bridge, so the UI can switch source
    # while idle without restarting the Dora graph.
    profile_path = Path(os.getenv("PIPER_PROFILE", str(DEFAULT_PROFILE))).resolve()
    profile = load_profile(profile_path)
    config = PiperMinkConfig.from_profile(profile)
    urdf_path = profile_path.parent / str(profile["geometry"]["urdf_path"])
    solver = DualPiperMinkIK(
        urdf_path,
        profile["geometry"]["end_effector_links"],
        config,
    )
    retarget = PiperIncrementalMinkRetargeter(solver, config)
    node = Node(os.getenv("DORA_NODE_ID", "piper_x_dual_retarget"))
    collection_mode = os.getenv("RECAP_COLLECTION_MODE", "MIXED").strip().upper()
    if collection_mode not in {"DATA_COLLECTION", "MIXED"}:
        raise ValueError("RECAP_COLLECTION_MODE must be DATA_COLLECTION or MIXED")
    expert_only = collection_mode == "DATA_COLLECTION"
    robot_state: dict[str, float] | None = None
    takeover = expert_only
    retarget.set_takeover(takeover)
    last_exo_at = 0.0
    latest_exo: dict[str, Any] | None = None
    latest_exo_metadata: dict[str, Any] = {}
    latest_exo_source_ns = 0
    exo_sequence = 0
    processed_exo_sequence = 0
    last_reported_status = ""
    next_status_report_at = 0.0
    timeout = float(os.getenv("EXO_TIMEOUT_SECONDS", "0.2"))
    if timeout <= 0.0:
        raise ValueError("EXO_TIMEOUT_SECONDS must be positive")

    def send_status(status: str, source_ns: int = 0, **detail: Any) -> None:
        now_ns = time.time_ns()
        payload = {
            "status": status,
            "hardware_dispatched": False,
            "control_frame_left": profile["geometry"]["end_effector_links"][
                "left_arm"
            ],
            "control_frame_right": profile["geometry"]["end_effector_links"][
                "right_arm"
            ],
            **detail,
        }
        node.send_output(
            "status",
            pa.array([json.dumps(payload, sort_keys=True)]),
            {
                "timestamp_ns": now_ns,
                "source_timestamp_ns": int(source_ns or now_ns),
                "hardware_dispatched": False,
            },
        )

    send_status("WAITING_INPUT")
    try:
        for event in node:
            if event is None or event.get("type") == "STOP":
                break
            if event.get("type") != "INPUT":
                continue
            event_id = str(event["id"])
            metadata = dict(event.get("metadata", {}))
            source_ns = int(
                metadata.get("source_timestamp_ns")
                or metadata.get("timestamp_ns")
                or time.time_ns()
            )
            if event_id == "robot_state":
                robot_state = _named_vector(event["value"])
                continue
            if event_id == "takeover":
                takeover = expert_only or bool(_scalar(event["value"]))
                retarget.set_takeover(takeover)
                latest_exo = None
                last_exo_at = 0.0
                send_status("WAITING_ANCHOR" if takeover else "INACTIVE", source_ns)
                continue
            if event_id == "ee_pose":
                if expert_only and not takeover:
                    takeover = True
                    retarget.set_takeover(True)
                latest_exo = _scalar(event["value"])
                latest_exo_metadata = metadata
                latest_exo_source_ns = source_ns
                last_exo_at = time.monotonic()
                exo_sequence += 1
                continue
            if event_id != "tick":
                continue
            if takeover and last_exo_at and time.monotonic() - last_exo_at > timeout:
                retarget.set_takeover(False)
                takeover = False
                send_status("HOLD_EXO_STALE", source_ns)
            if (
                not takeover
                or robot_state is None
                or latest_exo is None
                or exo_sequence == processed_exo_sequence
            ):
                continue
            processed_exo_sequence = exo_sequence
            data = latest_exo
            source_ns = latest_exo_source_ns
            metadata = latest_exo_metadata
            result = retarget.update(
                left_exo=data["left_pose"],
                right_exo=data["right_pose"],
                robot_state=robot_state,
                left_trigger=float(data["left_trigger"]),
                right_trigger=float(data["right_trigger"]),
                max_gripper_width_m=float(
                    profile["shadow_teleop"]["max_gripper_width_m"]
                ),
            )
            status_changed = result.status != last_reported_status
            if status_changed:
                print(
                    "PIPER_X_DUAL_RETARGET "
                    + json.dumps(
                        {"status": result.status, "takeover": takeover},
                        sort_keys=True,
                    ),
                    flush=True,
                )
                last_reported_status = result.status
            solve = result.solve
            detail = {
                "left_delta_m": result.left_delta_m,
                "right_delta_m": result.right_delta_m,
                "controller_ik": "host_mink_warm_start",
                "ik_iterations": 0 if solve is None else solve.iterations,
                "ik_reason": "" if solve is None else solve.reason,
                "max_solution_step_rad": result.max_solution_step_rad,
                "max_command_step_rad": result.max_command_step_rad,
            }
            now_monotonic = time.monotonic()
            if status_changed or now_monotonic >= next_status_report_at:
                send_status(result.status, source_ns, **detail)
                next_status_report_at = now_monotonic + 1.0
            if result.action is None:
                continue
            node.send_output(
                "expert_action",
                _named_vector_array(result.action),
                {
                    **metadata,
                    "timestamp_ns": time.time_ns(),
                    "source_timestamp_ns": source_ns,
                    "target_execution_timestamp_ns": time.time_ns(),
                    "retarget_mapping_version": "piper_mink_warm_start_joint_stream_v1",
                    "hardware_dispatched": False,
                    **detail,
                },
            )
    finally:
        retarget.reset()


if __name__ == "__main__":
    main()
