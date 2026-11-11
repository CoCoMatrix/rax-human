from __future__ import annotations

import json
import os
import time
from typing import Any


CONTROL_SOURCES = frozenset({"both", "qnexo", "keyboard"})
BUTTON_COMMANDS = frozenset(
    {"prepare", "start", "stop", "finish", "discard", "pause", "open_grippers", "open_left_gripper"}
)


class PiperControlMux:
    """Merge operator lifecycle sources without granting either hardware access."""

    def __init__(self, control_source: str = "both") -> None:
        source = str(control_source).strip().lower()
        if source not in CONTROL_SOURCES:
            raise ValueError(
                f"PIPER_CONTROL_SOURCE must be one of {sorted(CONTROL_SOURCES)}"
            )
        self.control_source = source
        self.qnexo_takeover = False
        self._last_takeover: bool | None = None

    def accepts(self, source: str) -> bool:
        return self.control_source == "both" or self.control_source == source

    def button(
        self,
        source: str,
        command: Any,
        *,
        operator_source: str = "",
    ) -> str | None:
        if not self.accepts(source):
            return None
        normalized = str(command).strip().lower()
        if normalized not in BUTTON_COMMANDS:
            raise ValueError(f"unsupported Piper lifecycle command: {command!r}")
        if normalized in {"open_grippers", "open_left_gripper"} and not (
            source == "keyboard" and operator_source == "keyboard_ui"
        ):
            raise ValueError("manual grippers require the operator Web")
        if normalized == "pause" and not (
            source == "keyboard" and str(operator_source).lower() == "pigey"
        ):
            raise ValueError("pause is reserved for the Pigey task boundary")
        return normalized

    def update_qnexo_takeover(self, active: Any) -> bool | None:
        self.qnexo_takeover = bool(active) if self.accepts("qnexo") else False
        return self.takeover_if_changed()

    def takeover_if_changed(self) -> bool | None:
        current = self.qnexo_takeover if self.accepts("qnexo") else False
        if current == self._last_takeover:
            return None
        self._last_takeover = current
        return current


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("Piper control mux requires dora-rs and pyarrow") from exc

    mux = PiperControlMux(os.getenv("PIPER_CONTROL_SOURCE", "both"))
    node = Node(os.getenv("DORA_NODE_ID", "piper_control_mux"))

    def send_takeover(value: bool, metadata: dict[str, Any] | None = None) -> None:
        now_ns = time.time_ns()
        detail = dict(metadata or {})
        detail.setdefault("source_timestamp_ns", now_ns)
        detail["timestamp_ns"] = now_ns
        detail["target_execution_timestamp_ns"] = now_ns
        node.send_output("takeover", pa.array([bool(value)]), detail)

    for event in node:
        if event is None or event.get("type") == "STOP":
            break
        if event.get("type") != "INPUT":
            continue
        event_id = str(event["id"])
        metadata = dict(event.get("metadata", {}))
        if event_id == "qnexo_takeover":
            takeover = mux.update_qnexo_takeover(_scalar(event["value"]))
            if takeover is not None:
                send_takeover(takeover, metadata)
            continue
        if event_id == "tick":
            takeover = mux.takeover_if_changed()
            if takeover is not None:
                send_takeover(takeover, metadata)
            continue
        source = {
            "qnexo_button_command": "qnexo",
            "keyboard_button_command": "keyboard",
        }.get(event_id)
        if source is None:
            continue
        declared_source = str(metadata.get("operator_source", "")).strip().lower()
        operator_source = (
            declared_source
            if source == "keyboard" and declared_source in {"keyboard_ui", "pigey"}
            else source
        )
        command = mux.button(
            source,
            _scalar(event["value"]),
            operator_source=operator_source,
        )
        if command is None:
            continue
        now_ns = time.time_ns()
        metadata.setdefault("source_timestamp_ns", now_ns)
        metadata["timestamp_ns"] = now_ns
        metadata["target_execution_timestamp_ns"] = now_ns
        metadata["operator_source"] = operator_source
        node.send_output("button_command", pa.array([command]), metadata)
        print(
            "PIPER_CONTROL_MUX "
            + json.dumps(
                {
                    "command": command,
                    "control_channel": source,
                    "operator_source": operator_source,
                },
                sort_keys=True,
            ),
            flush=True,
        )


if __name__ == "__main__":
    main()
