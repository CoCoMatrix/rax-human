from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any

import yaml

from XPolicyLab.real_world.robots.gr3.driver import CONTROL_GROUPS
from XPolicyLab.real_world.robots.gr3.safety_node import _serialize


@dataclass(frozen=True, slots=True)
class AuroraSnapshot:
    timestamp_ns: int
    status: str
    reasons: tuple[str, ...]
    joints: dict[str, float] | None
    base_state: dict[str, Any] | None
    lifecycle: dict[str, str]


class Gr3AuroraDdsReadClient:
    """Minimal Aurora DDS client with state subscribers and no publishers."""

    def __init__(self, domain_id: int = 115) -> None:
        from fourier_aurora_client.aurora_state_map import (
            UpperBodyFsmState,
            VelocityCommandSource,
            WholeBodyFsmState,
        )
        from fourier_aurora_client.dds_interface import DDSInterface
        from fourier_aurora_client.qos_profile import SubscriberQosProfile
        import fourier_msgs.msg.AuroraState as AuroraState
        import fourier_msgs.msg.BaseData as BaseData
        import fourier_msgs.msg.MotionControlState as MotionControlState

        self._lock = threading.Lock()
        self._lifecycle: dict[str, str] = {}
        self._groups: dict[str, list[float]] = {}
        self._base: dict[str, list[float]] = {}
        self._timestamps: dict[str, int] = {}
        self._dds = DDSInterface(int(domain_id), is_ros_compatible=False)
        qos = SubscriberQosProfile.best_effort()

        def on_lifecycle(data: Any) -> None:
            with self._lock:
                self._lifecycle = {
                    "whole_body_fsm": WholeBodyFsmState(
                        data.whole_body_fsm_state()
                    ).name,
                    "upper_body_fsm": UpperBodyFsmState(
                        data.upper_body_fsm_state()
                    ).name,
                    "velocity_source": VelocityCommandSource(
                        data.velocity_command_source()
                    ).name,
                }
                self._timestamps["lifecycle"] = time.time_ns()

        def on_groups(data: Any) -> None:
            groups = {
                str(group.group_name()): [
                    float(value) for value in group.joint_position()
                ]
                for group in data.group_state()
            }
            with self._lock:
                self._groups = groups
                self._timestamps["groups"] = time.time_ns()

        def on_base(data: Any) -> None:
            with self._lock:
                self._base = {
                    "rpy": [float(value) for value in data.rpy()],
                    "pos_W": [float(value) for value in data.pos_W()],
                }
                self._timestamps["base"] = time.time_ns()

        self._subscribers = [
            self._dds.create_subscriber(
                "aurora_state",
                AuroraState.AuroraStatePubSubType(),
                AuroraState.AuroraState(),
                on_lifecycle,
                qos,
            ),
            self._dds.create_subscriber(
                "robot_control_group_state",
                MotionControlState.RobotControlGroupStatePubSubType(),
                MotionControlState.RobotControlGroupState(),
                on_groups,
                qos,
            ),
            self._dds.create_subscriber(
                "base_data_pub",
                BaseData.BaseDataPubSubType(),
                BaseData.BaseData(),
                on_base,
                qos,
            ),
        ]

    def get_fsm_state(self) -> str:
        with self._lock:
            return self._lifecycle["whole_body_fsm"]

    def get_upper_fsm_state(self) -> str:
        with self._lock:
            return self._lifecycle["upper_body_fsm"]

    def get_velocity_source(self) -> str:
        with self._lock:
            return self._lifecycle["velocity_source"]

    def get_group_state(self, group: str, key: str = "position") -> list[float]:
        if key != "position":
            raise KeyError(key)
        with self._lock:
            return list(self._groups[group])

    def get_base_data(self, key: str) -> list[float]:
        with self._lock:
            return list(self._base[key])

    def get_state_timestamp_ns(self) -> int:
        with self._lock:
            if set(self._timestamps) != {"lifecycle", "groups", "base"}:
                return 0
            return min(self._timestamps.values())

    def close(self) -> None:
        (getattr(self._dds, "close", None) or self._dds._close)()


class Gr3AuroraStateSource:
    """Read-only GR3 Aurora adapter; it never changes robot lifecycle state."""

    def __init__(
        self,
        client_factory: Callable[[], Any],
        *,
        expected_lifecycle: dict[str, str],
        max_state_age_ms: float = 100.0,
        retry_seconds: float = 2.0,
    ) -> None:
        self._client_factory = client_factory
        self.expected_lifecycle = expected_lifecycle
        self.max_state_age_ns = int(float(max_state_age_ms) * 1_000_000)
        self.retry_seconds = float(retry_seconds)
        self._client: Any = None
        self._retry_at = 0.0

    @staticmethod
    def _call(client: Any, names: tuple[str, ...], *args: Any) -> Any:
        for name in names:
            method = getattr(client, name, None)
            if method is not None:
                return method(*args)
        raise AttributeError(names[0])

    def _get_client(self) -> Any:
        if self._client is not None:
            return self._client
        if time.monotonic() < self._retry_at:
            raise RuntimeError("aurora_retry_backoff")
        try:
            self._client = self._client_factory()
        except Exception:
            self._retry_at = time.monotonic() + self.retry_seconds
            raise
        return self._client

    def read(self) -> AuroraSnapshot:
        timestamp_ns = time.time_ns()
        try:
            client = self._get_client()
            joints: dict[str, float] = {}
            for group, names in CONTROL_GROUPS.items():
                values = [
                    float(value)
                    for value in self._call(
                        client,
                        ("get_group_state",),
                        group,
                        "position",
                    )
                ]
                if len(values) != len(names):
                    raise ValueError(f"invalid_{group}_length")
                if group.endswith("_hand"):
                    values = [
                        0.17 - values[0],
                        0.17 - values[1],
                        0.17 - values[2],
                        0.17 - values[3],
                        values[4] - 0.12,
                        -values[5],
                    ]
                joints.update(zip(names, values, strict=True))

            rpy = [
                float(value)
                for value in self._call(client, ("get_base_data",), "rpy")
            ]
            position = [
                float(value)
                for value in self._call(client, ("get_base_data",), "pos_W")
            ]
            if len(rpy) < 3 or len(position) < 3:
                raise ValueError("invalid_base_state")
            if not all(
                math.isfinite(value)
                for value in (*joints.values(), *rpy[:3], *position[:3])
            ):
                raise ValueError("non_finite_robot_state")

            lifecycle = {
                "whole_body_fsm": str(
                    self._call(client, ("get_fsm_state", "get_state"))
                ),
                "upper_body_fsm": str(
                    self._call(client, ("get_upper_fsm_state", "get_upper_state"))
                ),
                "velocity_source": str(
                    self._call(client, ("get_velocity_source",))
                ),
            }
            timestamp_ns = int(
                self._call(client, ("get_state_timestamp_ns",))
            )
        except Exception:
            return AuroraSnapshot(
                timestamp_ns,
                "HOLD",
                ("aurora_state_unavailable",),
                None,
                None,
                {},
            )

        age_ns = time.time_ns() - timestamp_ns
        reasons = tuple(
            f"{name}_mismatch"
            for name, expected in self.expected_lifecycle.items()
            if lifecycle.get(name) != expected
        )
        if timestamp_ns <= 0 or not 0 <= age_ns <= self.max_state_age_ns:
            reasons += ("stale_aurora_state",)
        return AuroraSnapshot(
            timestamp_ns,
            "HOLD" if reasons else "READY",
            reasons,
            joints,
            {
                "base": {
                    "position": position[:3],
                    "rpy": rpy[:3],
                }
            },
            lifecycle,
        )

    def close(self) -> None:
        client = self._client
        self._client = None
        if client is not None and hasattr(client, "close"):
            client.close()


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("GR3 Aurora state node requires dora-rs and pyarrow") from exc

    profile = yaml.safe_load(
        Path(os.getenv("GR3_PROFILE", Path(__file__).with_name("profile.yml"))).read_text(
            encoding="utf-8"
        )
    )
    source = Gr3AuroraStateSource(
        lambda: Gr3AuroraDdsReadClient(
            int(os.getenv("GR3_ROBOT_ID", "115"))
        ),
        expected_lifecycle=dict(profile["readiness"]["expected_lifecycle"]),
        max_state_age_ms=profile["readiness"]["max_age_ms"],
    )
    node = Node()
    try:
        for event in node:
            if event.get("type") != "INPUT" or event["id"] != "tick":
                continue
            snapshot = source.read()
            readiness = {
                "status": snapshot.status,
                "reasons": list(snapshot.reasons),
                "timestamp_ns": snapshot.timestamp_ns,
                "lifecycle": snapshot.lifecycle,
            }
            metadata = {
                "timestamp_ns": snapshot.timestamp_ns,
                "source_timestamp_ns": snapshot.timestamp_ns,
                "readiness_status": snapshot.status,
                "readiness_reasons": ",".join(snapshot.reasons),
            }
            node.send_output(
                "readiness",
                pa.array([json.dumps(readiness, sort_keys=True)]),
                metadata,
            )
            if snapshot.joints is not None and snapshot.base_state is not None:
                node.send_output("state", _serialize(pa, snapshot.joints), metadata)
                node.send_output(
                    "base_state",
                    pa.array([snapshot.base_state]),
                    metadata,
                )
    finally:
        source.close()


if __name__ == "__main__":
    main()
