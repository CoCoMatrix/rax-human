from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Any

from XPolicyLab.real_world.robots.gr3.mapping import BASE_ACTION_NAMES, JOINT_NAMES
from XPolicyLab.real_world.robots.gr3.safety import Gr3SafetyGuard


def _bool_env(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    if value not in {"0", "1"}:
        raise ValueError(f"{name} must be 0 or 1")
    return value == "1"


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _metadata_int(metadata: dict[str, Any], key: str, default: int) -> int:
    value = metadata.get(key, default)
    return int(value.as_py() if hasattr(value, "as_py") else value)


def _deserialize(value: Any) -> dict[str, float]:
    result: dict[str, float] = {}
    for item in value:
        name = str(item["name"].as_py())
        if name in result:
            raise ValueError(f"duplicate action field: {name}")
        result[name] = float(item["value"].as_py())
    return result


def _serialize(pa: Any, values: dict[str, float]) -> Any:
    return pa.StructArray.from_arrays(
        [pa.array(list(values)), pa.array(list(values.values()), type=pa.float32())],
        names=["name", "value"],
    )


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("GR3 safety node requires dora-rs and pyarrow") from exc

    profile_path = Path(
        os.getenv("GR3_PROFILE", Path(__file__).with_name("profile.yml"))
    )
    guard = Gr3SafetyGuard.from_profile(
        profile_path,
        require_joint_limits=_bool_env("REQUIRE_JOINT_LIMITS", True),
        max_joint_step_rad=(
            float(os.environ["SAFETY_MAX_JOINT_STEP_RAD"])
            if "SAFETY_MAX_JOINT_STEP_RAD" in os.environ
            else None
        ),
    )
    node = Node()
    robot_state: dict[str, float] = {}
    state_timestamp_ns = 0
    enabled = False
    generation = 0

    def send_status(status: str, reasons: tuple[str, ...]) -> None:
        timestamp_ns = time.time_ns()
        node.send_output(
            "status",
            pa.array(
                [
                    json.dumps(
                        {
                            "status": status,
                            "reasons": list(reasons),
                            "generation": generation,
                        },
                        sort_keys=True,
                    )
                ]
            ),
            {
                "timestamp_ns": timestamp_ns,
                "generation": generation,
                "safety_status": status,
                "safety_reasons": ",".join(reasons),
            },
        )

    for event in node:
        if event.get("type") != "INPUT":
            continue
        event_id = event["id"]
        if event_id == "state":
            try:
                robot_state = _deserialize(event["value"])
            except (KeyError, TypeError, ValueError):
                robot_state = {}
                state_timestamp_ns = 0
                send_status("HOLD", ("invalid_state_payload",))
                continue
            metadata = event.get("metadata", {})
            state_timestamp_ns = _metadata_int(
                metadata,
                "source_timestamp_ns",
                _metadata_int(metadata, "timestamp_ns", time.time_ns()),
            )
            continue
        if event_id == "control":
            try:
                control = json.loads(str(_scalar(event["value"])))
                generation = int(control["generation"])
                command = str(control["command"])
            except (IndexError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                enabled = False
                send_status("HOLD", ("invalid_control_payload",))
                continue
            enabled = command in {"bootstrap", "run", "step"}
            send_status("ARMED" if enabled else "HOLD", () if enabled else ("control_disabled",))
            continue
        if event_id != "action":
            continue

        metadata = dict(event.get("metadata", {}))
        try:
            action = _deserialize(event["value"])
        except (KeyError, TypeError, ValueError):
            send_status("HOLD", ("invalid_action_payload",))
            continue
        decision = guard.evaluate(
            action,
            robot_state,
            enabled=enabled,
            generation=_metadata_int(metadata, "generation", -1),
            expected_generation=generation,
            state_timestamp_ns=state_timestamp_ns,
            now_ns=time.time_ns(),
        )
        send_status(decision.status, decision.reasons)
        if not decision.accepted:
            continue

        safe_action = decision.action
        assert safe_action is not None
        output_metadata = {
            **metadata,
            "timestamp_ns": time.time_ns(),
            "generation": generation,
            "safety_status": decision.status,
            "safety_reasons": ",".join(decision.reasons),
        }
        node.send_output("safe_action", _serialize(pa, safe_action), output_metadata)
        node.send_output(
            "safe_base_action",
            _serialize(
                pa,
                {name: safe_action[name] for name in BASE_ACTION_NAMES},
            ),
            output_metadata,
        )
