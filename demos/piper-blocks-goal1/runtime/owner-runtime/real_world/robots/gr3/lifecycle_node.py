from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path
import time
from typing import Any

import yaml

from XPolicyLab.real_world.robots.gr3.lifecycle import Gr3LifecycleController


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
        from fourier_aurora_client import AuroraClient
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "GR3 lifecycle node requires Dora and the compatible Aurora SDK"
        ) from exc

    node = Node()
    client: Any = None
    controller: Gr3LifecycleController | None = None

    def get_controller() -> Gr3LifecycleController:
        nonlocal client, controller
        if controller is None:
            profile = yaml.safe_load(
                Path(
                    os.getenv(
                        "GR3_PROFILE",
                        Path(__file__).with_name("profile.yml"),
                    )
                ).read_text(encoding="utf-8")
            )
            prepare = profile["prepare"]
            client = AuroraClient.get_instance(
                int(os.getenv("GR3_ROBOT_ID", "115")),
                robot_name="gr3",
                is_ros_compatible=False,
            )
            if client is None:
                raise RuntimeError("AuroraClient initialization failed")
            controller = Gr3LifecycleController(
                client,
                idle_poses=(prepare["initial_pose"],),
                idle_pose_settle_seconds=prepare["idle_pose_settle_seconds"],
                idle_pose_duration_seconds=prepare["idle_pose_duration_seconds"],
                idle_pose_pause_seconds=prepare["idle_pose_pause_seconds"],
                command_frequency_hz=prepare["command_frequency_hz"],
            )
        return controller

    def send_status(phase: str, decision: Any) -> None:
        timestamp_ns = time.time_ns()
        node.send_output(
            "status",
            pa.array(
                [
                    json.dumps(
                        {"phase": phase, **asdict(decision)},
                        sort_keys=True,
                    )
                ]
            ),
            {
                "timestamp_ns": timestamp_ns,
                "source_timestamp_ns": timestamp_ns,
                "lifecycle_phase": phase,
                "lifecycle_status": decision.status,
            },
        )

    try:
        # Bringing up the robot chain immediately places the whole-body
        # controller in PdStand.
        # Preparing the arm pose remains an explicit operator command.
        startup_pdstand = get_controller().initialize_pdstand()
        send_status("pdstand", startup_pdstand)
        if startup_pdstand.status != "READY":
            raise RuntimeError(
                "GR3 startup PdStand failed: "
                + ",".join(startup_pdstand.reasons)
            )

        for event in node:
            if event.get("type") != "INPUT" or event["id"] != "command":
                continue
            command = str(_scalar(event["value"]))
            lifecycle = get_controller()
            if command == "prepare":
                pdstand = lifecycle.enter_pdstand()
                send_status("pdstand", pdstand)
                if pdstand.status == "READY":
                    send_status("active", lifecycle.activate())
            elif command == "disengage":
                send_status("pdstand", lifecycle.disengage())
    finally:
        if controller is not None:
            decision = controller.disengage()
            print(
                "GR3_LIFECYCLE_SHUTDOWN",
                json.dumps(asdict(decision), sort_keys=True),
                flush=True,
            )
        if client is not None:
            client.close()


if __name__ == "__main__":
    main()
