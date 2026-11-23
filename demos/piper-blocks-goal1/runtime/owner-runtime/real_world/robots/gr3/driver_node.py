from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path
import time
from typing import Any

import yaml

from XPolicyLab.real_world.robots.gr3.driver import Gr3AuroraDriver
from XPolicyLab.real_world.robots.gr3.safety_node import _bool_env, _deserialize


def _metadata_value(metadata: dict[str, Any], key: str, default: Any) -> Any:
    value = metadata.get(key, default)
    return value.as_py() if hasattr(value, "as_py") else value


def main() -> None:
    raise RuntimeError(
        "xpolicylab-gr3-driver is retired; use the formal teleop/Aurora owner graph"
    )


def _retired_main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("GR3 driver node requires dora-rs and pyarrow") from exc

    profile = yaml.safe_load(
        Path(os.getenv("GR3_PROFILE", Path(__file__).with_name("profile.yml"))).read_text(
            encoding="utf-8"
        )
    )
    robot_id = os.getenv("GR3_ROBOT_ID")
    driver = Gr3AuroraDriver(
        enabled=_bool_env("GR3_ENABLE_WRITE", False),
        robot_id=int(robot_id) if robot_id else None,
        limited_joints=profile["safety"].get("joint_limits", {}),
    )
    readiness_max_age_ns = int(
        float(profile["readiness"]["max_age_ms"]) * 1_000_000
    )
    node = Node()
    operator_enabled = False
    generation = 0
    readiness_status = "HOLD"
    readiness_timestamp_ns = 0
    written_actions = 0
    last_decision_signature: tuple[str, tuple[str, ...]] | None = None

    for event in node:
        if event.get("type") != "INPUT":
            continue
        event_id = event["id"]
        if event_id == "control":
            try:
                control = json.loads(str(event["value"][0].as_py()))
                command = str(control["command"])
                generation = int(control["generation"])
                operator_enabled = command in {"bootstrap", "run", "step"}
                print(
                    "GR3_DRIVER_CONTROL",
                    json.dumps(
                        {
                            "command": command,
                            "generation": generation,
                            "operator_enabled": operator_enabled,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
            except (IndexError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                operator_enabled = False
            continue
        if event_id == "readiness":
            try:
                readiness = json.loads(str(event["value"][0].as_py()))
                readiness_status = str(readiness["status"])
                readiness_timestamp_ns = int(readiness["timestamp_ns"])
            except (IndexError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                readiness_status = "HOLD"
                readiness_timestamp_ns = 0
            continue
        if event_id != "action":
            continue

        metadata = dict(event.get("metadata", {}))
        try:
            action = _deserialize(event["value"])
        except (KeyError, TypeError, ValueError):
            decision = driver._hold("invalid_action_payload")
        else:
            now_ns = time.time_ns()
            readiness_age_ns = now_ns - readiness_timestamp_ns
            decision = driver.send(
                action,
                safety_status=str(_metadata_value(metadata, "safety_status", "")),
                operator_enabled=operator_enabled,
                robot_ready=(
                    readiness_status == "READY"
                    and 0 <= readiness_age_ns <= readiness_max_age_ns
                ),
                generation=int(_metadata_value(metadata, "generation", -1)),
                expected_generation=generation,
            )
        if decision.wrote:
            written_actions += 1
        signature = (decision.status, tuple(decision.reasons))
        if (
            signature != last_decision_signature
            or decision.wrote
            and (written_actions == 1 or written_actions % 30 == 0)
        ):
            print(
                "GR3_DRIVER_STATUS",
                json.dumps(
                    {
                        **asdict(decision),
                        "written_actions": written_actions,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            last_decision_signature = signature
        node.send_output(
            "status",
            pa.array([json.dumps(asdict(decision), sort_keys=True)]),
            {
                "timestamp_ns": time.time_ns(),
                "driver_status": decision.status,
                "driver_reasons": ",".join(decision.reasons),
                "wrote": decision.wrote,
            },
        )


if __name__ == "__main__":
    main()
