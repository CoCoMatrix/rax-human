from __future__ import annotations

import json
import os
import time
from typing import Any

from XPolicyLab.real_world.teleop import (
    TeleopButtonCoordinator,
    TeleopButtonResult,
    TeleopControlTransition,
)


def control_for_event(
    event: str,
    *,
    allow_continuous: bool = False,
    prepared: bool = True,
) -> tuple[str, str]:
    if event == "step":
        return ("step", "") if prepared else ("pause", "prepare_required")
    if event == "start":
        return (
            ("run", "")
            if allow_continuous
            else ("pause", "continuous_model_disabled")
        )
    if event in {"stop", "finish", "prepare", "takeover"}:
        return "pause", ""
    raise ValueError(f"unsupported operator event: {event}")


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def lifecycle_command_for_event(event: str) -> str | None:
    """Keep red-button episode boundaries in GR3 RLStand.

    Starting and shutting down the Dora graph still own the PdStand
    transitions. A takeover remains a safety disengage, while stop/finish
    only pause policy dispatch so the next episode can reuse RLStand.
    """
    if event == "prepare":
        return "prepare"
    if event == "takeover":
        return "disengage"
    return None


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("GR3 operator node requires dora-rs and pyarrow") from exc

    node = Node()
    allow_continuous = os.getenv("ALLOW_CONTINUOUS_MODEL", "0") == "1"
    require_prepare = os.getenv("REQUIRE_PREPARE", "1") == "1"
    buttons = TeleopButtonCoordinator()
    if not require_prepare:
        # Compatibility for the legacy standalone node only.  Production
        # robot graphs require the standard white-button prepare boundary.
        buttons.lifecycle.state = "prepared"
    takeover = False

    def emit(result: TeleopButtonResult) -> str:
        transition = result.transition
        if transition.accepted:
            command, reason = control_for_event(
                transition.event,
                allow_continuous=allow_continuous,
                prepared=buttons.lifecycle.prepared,
            )
        else:
            command, reason = "pause", transition.reason
        timestamp_ns = time.time_ns()
        metadata = {
            "timestamp_ns": timestamp_ns,
            "source_timestamp_ns": timestamp_ns,
            "generation": transition.generation,
            "operator_event": transition.event,
        }
        status_payload = {
            "event": transition.event,
            "command": command,
            "accepted": transition.accepted,
            "state": transition.state,
            "generation": transition.generation,
            "reason": reason,
            "replayed": result.replayed,
        }
        node.send_output(
            "control",
            pa.array(
                [
                    json.dumps(
                        {
                            "command": command,
                            "generation": transition.generation,
                        },
                        sort_keys=True,
                    )
                ]
            ),
            metadata,
        )
        node.send_output(
            "status",
            pa.array([json.dumps(status_payload, sort_keys=True)]),
            metadata,
        )
        print(
            "GR3_OPERATOR_EVENT",
            json.dumps(status_payload, sort_keys=True),
            flush=True,
        )
        return command

    def emit_lifecycle(command: str, metadata: dict[str, Any]) -> None:
        node.send_output("lifecycle", pa.array([command]), metadata)

    for event in node:
        if event.get("type") != "INPUT":
            continue
        if event["id"] == "button_command":
            try:
                command_event = str(_scalar(event["value"]))
                result = (
                    buttons.reject(command_event, "continuous_model_disabled")
                    if command_event == "start" and not allow_continuous
                    else buttons.handle(command_event)
                )
                emit(result)
                lifecycle_command = (
                    lifecycle_command_for_event(command_event)
                    if result.transition.accepted
                    else None
                )
                if lifecycle_command is not None:
                    emit_lifecycle(
                        lifecycle_command,
                        dict(event.get("metadata", {})),
                    )
                if result.transition.accepted and command_event in {
                    "prepare",
                    "finish",
                }:
                    buttons.complete_boundary(success=True)
            except (IndexError, TypeError, ValueError):
                continue
        elif event["id"] == "takeover":
            pressed = bool(_scalar(event["value"]))
            if pressed and not takeover:
                buttons.lifecycle.generation += 1
                emit(
                    TeleopButtonResult(
                        TeleopControlTransition(
                            "takeover",
                            True,
                            buttons.lifecycle.state,
                            buttons.lifecycle.generation,
                        )
                    )
                )
                emit_lifecycle("disengage", dict(event.get("metadata", {})))
            takeover = pressed


if __name__ == "__main__":
    main()
