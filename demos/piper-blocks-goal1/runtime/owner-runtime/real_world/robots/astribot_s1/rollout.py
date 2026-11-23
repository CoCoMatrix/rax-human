from __future__ import annotations

import hashlib
import json
import os
import re
import time
from collections import Counter
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

import numpy as np
from XPolicyLab.real_world.contracts import VectorSchema
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    full_body_action_vector,
)
from XPolicyLab.real_world.robots.astribot_s1.owner import OwnerTickDecision

ROLLOUT_REPORT_SCHEMA = "astribot_s1.owner_rollout.v1"
_ROLLOUT_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def _json_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(name): _json_value(item) for name, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    partial = path.with_name(f".{path.name}.partial")
    with partial.open("w", encoding="utf-8") as stream:
        json.dump(_json_value(payload), stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(partial, path)


class AstribotS1RolloutRecorder:
    """Crash-visible JSONL evidence for one owner-runtime rollout."""

    def __init__(
        self,
        output_root: str | Path,
        *,
        mode: str,
        robot_id: str,
        instruction: str,
        generation: int,
        profile_id: str,
        state_schema: VectorSchema,
        action_schema: VectorSchema,
        rollout_id: str | None = None,
    ) -> None:
        if mode not in {"shadow", "hardware"}:
            raise ValueError("rollout mode must be shadow or hardware")
        if rollout_id is None:
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            rollout_id = f"{stamp}-{uuid4().hex[:8]}"
        if not _ROLLOUT_ID.fullmatch(str(rollout_id)):
            raise ValueError("rollout_id contains unsafe characters")
        self.rollout_id = str(rollout_id)
        self.output_root = Path(output_root).expanduser().resolve()
        self.rollout_dir = self.output_root / self.rollout_id
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.rollout_dir.mkdir(exist_ok=False)
        self.events_partial_path = self.rollout_dir / "events.jsonl.partial"
        self.events_path = self.rollout_dir / "events.jsonl"
        self.summary_path = self.rollout_dir / "summary.json"
        self.manifest_path = self.rollout_dir / "manifest.json"
        self._stream = self.events_partial_path.open("x", encoding="utf-8")
        self._closed = False
        self._event_index = 0
        self._started_timestamp_ns = time.time_ns()
        self._last_status: tuple[str, tuple[str, ...]] | None = None
        self._last_action_key: tuple[int, int, int] | None = None
        self._tick_count = 0
        self._observation_count = 0
        self._action_step_count = 0
        self._hardware_writes = 0
        self._max_camera_skew_ns = 0
        self._max_state_skew_ns = 0
        self._status_counts: Counter[str] = Counter()
        self._arm_safety_counts: Counter[str] = Counter()
        self._gripper_safety_counts: Counter[str] = Counter()
        self._driver_status_counts: Counter[str] = Counter()
        self._reason_counts: Counter[str] = Counter()
        self._manifest = {
            "schema_version": ROLLOUT_REPORT_SCHEMA,
            "rollout_id": self.rollout_id,
            "mode": mode,
            "robot_type": "astribot_s1",
            "robot_id": str(robot_id),
            "instruction": str(instruction),
            "generation": int(generation),
            "profile_id": str(profile_id),
            "started_timestamp_ns": self._started_timestamp_ns,
            "state_schema": state_schema.to_wire(),
            "action_schema": action_schema.to_wire(),
            "events_file": self.events_partial_path.name,
            "image_storage": "metadata_and_sha256_only",
        }
        _atomic_json(self.manifest_path, self._manifest)
        self.record_lifecycle("recorder_started")

    def _write_event(self, event_type: str, payload: Mapping[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("rollout recorder is finalized")
        event = {
            "schema_version": ROLLOUT_REPORT_SCHEMA,
            "event_index": self._event_index,
            "event_type": str(event_type),
            "recorded_timestamp_ns": time.time_ns(),
            **dict(payload),
        }
        self._stream.write(
            json.dumps(_json_value(event), ensure_ascii=False, sort_keys=True)
        )
        self._stream.write("\n")
        self._stream.flush()
        self._event_index += 1

    def record_lifecycle(self, name: str, **detail: Any) -> None:
        self._write_event(
            "lifecycle",
            {"name": str(name), "detail": dict(detail)},
        )

    @staticmethod
    def _image_metadata(image: np.ndarray) -> dict[str, Any]:
        array = np.ascontiguousarray(image)
        return {
            "shape": list(array.shape),
            "dtype": str(array.dtype),
            "sha256": hashlib.sha256(array.tobytes()).hexdigest(),
        }

    def record_tick(self, decision: OwnerTickDecision) -> None:
        if self._closed:
            raise RuntimeError("rollout recorder is finalized")
        self._tick_count += 1
        self._status_counts[decision.status] += 1
        self._arm_safety_counts[decision.action.arm_safety.status] += 1
        self._gripper_safety_counts[decision.action.gripper_safety.status] += 1
        self._driver_status_counts[decision.action.driver.status] += 1
        self._hardware_writes += int(decision.wrote)
        self._reason_counts.update(decision.reasons)

        status_key = (decision.status, decision.reasons)
        if status_key != self._last_status:
            self._write_event(
                "status",
                {
                    "status": decision.status,
                    "reasons": decision.reasons,
                    "state_timestamp_ns": decision.snapshot.timestamp_ns,
                    "generation": decision.action.generation,
                },
            )
            self._last_status = status_key

        if decision.observation is not None:
            observation = decision.observation
            self._observation_count += 1
            self._max_camera_skew_ns = max(
                self._max_camera_skew_ns,
                decision.camera_sync.max_camera_skew_ns,
            )
            self._max_state_skew_ns = max(
                self._max_state_skew_ns,
                decision.camera_sync.max_state_skew_ns,
            )
            self._write_event(
                "observation",
                {
                    "sequence": observation.sequence,
                    "timestamp_ns": observation.timestamp_ns,
                    "state": observation.state.tolist(),
                    "images": {
                        name: self._image_metadata(image)
                        for name, image in observation.images.items()
                    },
                    "camera_sync": observation.metadata.get("camera_sync", {}),
                },
            )

        step = decision.action.chunk_step_index
        action_key = (
            (
                decision.action.generation,
                decision.action.command_timestamp_ns,
                step,
            )
            if step is not None
            else None
        )
        if action_key is not None and action_key != self._last_action_key:
            self._action_step_count += 1
            target = decision.action.target
            self._write_event(
                "action_step",
                {
                    "generation": decision.action.generation,
                    "chunk_step_index": step,
                    "target": (
                        full_body_action_vector(target).tolist()
                        if target is not None
                        else None
                    ),
                    "chassis_safety": {
                        "status": decision.action.chassis_safety.status,
                        "reasons": decision.action.chassis_safety.reasons,
                    },
                    "auxiliary_safety": {
                        "status": decision.action.auxiliary_safety.status,
                        "reasons": decision.action.auxiliary_safety.reasons,
                    },
                    "arm_safety": {
                        "status": decision.action.arm_safety.status,
                        "reasons": decision.action.arm_safety.reasons,
                    },
                    "gripper_safety": {
                        "status": decision.action.gripper_safety.status,
                        "reasons": decision.action.gripper_safety.reasons,
                    },
                    "driver": {
                        "status": decision.action.driver.status,
                        "reasons": decision.action.driver.reasons,
                        "wrote": decision.action.driver.wrote,
                    },
                },
            )
            self._last_action_key = action_key

    def finalize(self, *, outcome: str, termination_reason: str) -> dict[str, Any]:
        if self._closed:
            raise RuntimeError("rollout recorder is already finalized")
        if outcome not in {"pass", "timeout", "fault", "aborted"}:
            raise ValueError("invalid rollout outcome")
        ended_timestamp_ns = time.time_ns()
        self.record_lifecycle(
            "recorder_finalizing",
            outcome=outcome,
            termination_reason=termination_reason,
        )
        self._stream.flush()
        os.fsync(self._stream.fileno())
        self._stream.close()
        os.replace(self.events_partial_path, self.events_path)
        self._closed = True
        duration_seconds = (
            ended_timestamp_ns - self._started_timestamp_ns
        ) / 1_000_000_000
        summary = {
            "schema_version": ROLLOUT_REPORT_SCHEMA,
            "rollout_id": self.rollout_id,
            "outcome": outcome,
            "termination_reason": str(termination_reason),
            "started_timestamp_ns": self._started_timestamp_ns,
            "ended_timestamp_ns": ended_timestamp_ns,
            "duration_seconds": duration_seconds,
            "tick_count": self._tick_count,
            "observation_count": self._observation_count,
            "action_step_count": self._action_step_count,
            "hardware_write_count": self._hardware_writes,
            "max_camera_skew_ms": self._max_camera_skew_ns / 1_000_000,
            "max_state_skew_ms": self._max_state_skew_ns / 1_000_000,
            "status_counts": dict(self._status_counts),
            "arm_safety_counts": dict(self._arm_safety_counts),
            "gripper_safety_counts": dict(self._gripper_safety_counts),
            "driver_status_counts": dict(self._driver_status_counts),
            "reason_counts": dict(self._reason_counts),
            "artifacts": {
                "manifest": self.manifest_path.name,
                "events": self.events_path.name,
                "summary": self.summary_path.name,
            },
        }
        _atomic_json(self.summary_path, summary)
        manifest = dict(self._manifest)
        manifest["events_file"] = self.events_path.name
        manifest["summary_file"] = self.summary_path.name
        _atomic_json(self.manifest_path, manifest)
        return summary
