from __future__ import annotations

import json
import os
import time
from typing import Any, Mapping

from XPolicyLab.real_world.teleop import (
    DEFAULT_FOOTSWITCH_DEVICE,
    DEFAULT_FOOTSWITCH_RIGHT_KEY_CODE,
    FootSwitch,
    QnexoControlCoordinator,
    QnexoButtonBank,
    ReviewJoystickNavigator,
)


def _metadata(source: Mapping[str, Any] | None = None) -> dict[str, Any]:
    result = dict(source or {})
    now_ns = time.time_ns()
    result.setdefault("timestamp_ns", now_ns)
    result.setdefault("source_timestamp_ns", now_ns)
    result["target_execution_timestamp_ns"] = now_ns
    return result


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
        from qnexo.exo import parse_joystick_data
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "QNexo control node requires dora-rs, pyarrow, and qnexo.exo"
        ) from exc

    timeout_seconds = float(os.getenv("EXO_TIMEOUT_SECONDS", "0.2"))
    if timeout_seconds <= 0:
        raise ValueError("EXO_TIMEOUT_SECONDS must be positive")
    switch = FootSwitch(
        os.getenv("FOOTSWITCH_DEVICE", DEFAULT_FOOTSWITCH_DEVICE),
        int(
            os.getenv(
                "FOOTSWITCH_TAKEOVER_KEY_CODE",
                str(DEFAULT_FOOTSWITCH_RIGHT_KEY_CODE),
            )
        ),
        toggle_on_press=True,
    )
    collection_mode = os.getenv("RECAP_COLLECTION_MODE", "MIXED")
    controls = QnexoControlCoordinator(
        switch,
        button_bank=QnexoButtonBank(collection_mode),
    )
    if not switch.connected:
        print(
            "QNEXO_CONTROLS "
            + json.dumps(
                {
                    "footswitch": "UNAVAILABLE",
                    "footswitch_path": switch.path,
                    "takeover": False,
                },
                sort_keys=True,
            ),
            flush=True,
        )
    node = Node()
    last_exo_at: float | None = None
    stale_reported = False
    last_takeover = False
    last_pressed: tuple[tuple[str, ...], tuple[str, ...]] = ((), ())
    review_navigator = ReviewJoystickNavigator()

    def send(name: str, value: Any, metadata: Mapping[str, Any]) -> None:
        node.send_output(name, pa.array([value]), dict(metadata))

    def send_status(status: str, metadata: Mapping[str, Any], **detail: Any) -> None:
        send(
            "status",
            json.dumps({"status": status, **detail}, sort_keys=True),
            metadata,
        )

    try:
        for event in node:
            if event is None or event.get("type") == "STOP":
                break
            if event.get("type") != "INPUT":
                continue
            event_id = str(event["id"])
            if event_id == "tick":
                stale = (
                    last_exo_at is None
                    or time.monotonic() - last_exo_at > timeout_seconds
                )
                if stale and not stale_reported:
                    controls.fail_closed()
                    event_metadata = _metadata(event.get("metadata"))
                    send("takeover", False, event_metadata)
                    send_status("STALE", event_metadata, reason="exo_timeout")
                    stale_reported = True
                continue
            if event_id != "exo_data":
                continue

            event_metadata = _metadata(event.get("metadata"))
            if float(event_metadata.get("latest_data_delay_s", 0.0)) > timeout_seconds:
                controls.fail_closed()
                send("takeover", False, event_metadata)
                send_status("STALE", event_metadata, reason="exo_data_delay")
                stale_reported = True
                continue

            raw = event["value"][0].as_py()
            left = parse_joystick_data(*map(int, raw["joystick_left"]))
            right = parse_joystick_data(*map(int, raw["joystick_right"]))
            source_timestamp_ns = int(
                event_metadata.get("source_timestamp_ns")
                or event_metadata.get("timestamp_ns")
                or time.time_ns()
            )
            state = controls.update(
                left["button"],
                right["button"],
                timestamp_ns=source_timestamp_ns,
            )
            pressed = (
                tuple(map(str, left["button"])),
                tuple(map(str, right["button"])),
            )
            if pressed != last_pressed:
                print(
                    "QNEXO_CONTROLS "
                    + json.dumps(
                        {
                            "left_pressed": pressed[0],
                            "right_pressed": pressed[1],
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                last_pressed = pressed
            if state.control_event is not None:
                print(
                    "QNEXO_CONTROLS "
                    + json.dumps(
                        {
                            "button": state.control_event.button,
                            "command": state.control_event.command,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                review_button_events = {
                    "right_blue": "right_blue",
                    "right_red": "right_red",
                }
                if review_event := review_button_events.get(
                    state.control_event.button
                ):
                    send("review_input", review_event, event_metadata)
            review_event = review_navigator.update(
                float(left["x"]),
                float(left["y"]),
            )
            if review_event is not None:
                send("review_input", review_event, event_metadata)
            if state.takeover != last_takeover:
                print(
                    "QNEXO_CONTROLS "
                    + json.dumps({"takeover": state.takeover}, sort_keys=True),
                    flush=True,
                )
                last_takeover = state.takeover
            last_exo_at = time.monotonic()
            if stale_reported:
                send_status(
                    "READY",
                    event_metadata,
                    footswitch_connected=switch.connected,
                )
                stale_reported = False
            send("takeover", state.takeover, event_metadata)
            if state.control_event is not None:
                send(
                    "button_command",
                    state.control_event.command,
                    event_metadata,
                )
    finally:
        controls.fail_closed()
        controls.close()


if __name__ == "__main__":
    main()
