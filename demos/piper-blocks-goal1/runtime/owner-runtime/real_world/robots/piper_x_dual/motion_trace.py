"""Optional, loss-accounted diagnostics; never a robot command or recorder label.

Set PIPER_MOTION_TRACE_PATH to a new JSONL file under the personal workspace.
One MODEL event becomes one sample, including rejected events. File creation,
JSON encoding and writes happen on a bounded background queue. A trace failure
must not change the existing hardware control result or exception propagation.
Successful SDK calls mean commands were sent, not that the joints arrived.
"""
from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
from collections.abc import Mapping, Sequence
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from .mapping import ACTION_NAMES
from .motion_stats import MotionStats

WORKSPACE_ROOT = Path("/home/ubuntu/code_junqi")


def _vector(value: Any) -> list[float | None] | None:
    if isinstance(value, str):
        value = json.loads(value)
    if isinstance(value, Mapping):
        value = [value[name] for name in ACTION_NAMES]
    values = list(value)
    if len(values) != len(ACTION_NAMES):
        raise ValueError("motion trace action must have 14 values")
    return [float(x) if math.isfinite(float(x)) else None for x in values]


class MotionTrace:
    def __init__(self, path: str = "", *, queue_size: int = 2048) -> None:
        self.path = ""
        self.error: str | None = None
        self.submitted = 0
        self.written = 0
        self.dropped = 0
        self.sequence = 0
        self.ready = False
        self._stats = MotionStats()
        self._stats_snapshot = self._stats.snapshot()
        self._closed = threading.Event()
        self._queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=queue_size)
        self._thread: threading.Thread | None = None
        if not path:
            return
        try:
            target = Path(path).expanduser()
            if not target.is_absolute():
                raise ValueError("PIPER_MOTION_TRACE_PATH must be absolute")
            target = target.resolve()
            if not target.is_relative_to(WORKSPACE_ROOT.resolve()):
                raise ValueError("PIPER_MOTION_TRACE_PATH must be inside the personal workspace")
            if target.suffix != ".jsonl":
                raise ValueError("PIPER_MOTION_TRACE_PATH must name a .jsonl file")
            self.path = str(target)
            self._thread = threading.Thread(target=self._write_loop, name="piper-motion-trace", daemon=True)
            self._thread.start()
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"

    @classmethod
    def from_env(cls) -> MotionTrace:
        return cls(os.getenv("PIPER_MOTION_TRACE_PATH", "").strip())

    def status(self) -> dict[str, Any]:
        return {"enabled": bool(self.path) and self.error is None and not self._closed.is_set(),
                "ready": self.ready, "path": self.path, "error": self.error,
                "submitted": self.submitted, "written": self.written,
                "dropped": self.dropped, "queue_size": self._queue.qsize(),
                "command_statistics": self._stats_snapshot}

    @contextmanager
    def candidate(self, action: Mapping[str, Any] | Sequence[float], *,
                  metadata: Mapping[str, Any], source_ns: int):
        sample = None
        if self.path and self.error is None and not self._closed.is_set():
            try:
                self.sequence += 1
                planned = _vector(action)
                raw_value = metadata.get("trajectory_raw_action")
                raw = planned if raw_value is None else _vector(raw_value)
                sample = {
                    "event": "model_candidate", "sequence": self.sequence,
                    "time_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns(),
                    "source_timestamp_ns": source_ns,
                    "command_timestamp_ns": metadata.get("timestamp_ns"),
                    "generation": metadata.get("generation"),
                    "model_chunk_id": metadata.get("model_chunk_id"),
                    "model_chunk_step_index": metadata.get("model_chunk_step_index"),
                    "policy_request_id": metadata.get("policy_request_id"),
                    "trajectory_metadata": {str(k): v for k, v in metadata.items()
                                            if str(k).startswith("trajectory_") and k != "trajectory_raw_action"},
                    "raw_action_14d": raw,
                    "raw_action_source": "trajectory_raw_action" if raw_value is not None else "incoming_action",
                    "planned_action_14d": planned,
                    "boundary_action_14d": None, "sdk_target_action_14d": None,
                    "dispatched_action_14d": None, "measured_action_14d": None,
                    "owner_result": "HOLD_ROUTER", "reasons": ["router_did_not_select_model"],
                    "hardware_write_attempted": False, "arm_write_success": False,
                    "gripper_write_attempted": False, "gripper_write_success": False,
                    "hardware_write_success": False,
                }
            except Exception as exc:
                # Malformed optional diagnostic metadata cannot reject an action.
                self.dropped += 1
                sample = None
        try:
            yield sample
        except BaseException as exc:
            if sample is not None:
                sample["owner_exception"] = f"{type(exc).__name__}: {exc}"
                if sample["hardware_write_attempted"] and not sample["hardware_write_success"]:
                    sample["owner_result"] = "SDK_WRITE_FAILED_OR_PARTIAL"
            raise
        finally:
            if sample is not None:
                sample["completed_time_ns"] = time.time_ns()
                sample["owner_duration_ns"] = time.monotonic_ns() - sample["monotonic_ns"]
                self.submit(sample)

    def submit(self, sample: dict[str, Any]) -> None:
        # No JSON encoding, filesystem access, waiting, or stdout in this method.
        if not self.path or self.error is not None or self._closed.is_set():
            self.dropped += 1
            return
        self.submitted += 1
        sample["trace_dropped_before"] = self.dropped
        try:
            self._queue.put_nowait(sample)
        except queue.Full:
            self.dropped += 1

    def _write_loop(self) -> None:
        try:
            target = Path(self.path)
            target.parent.mkdir(parents=True, exist_ok=True)
            # Never append separate runs or overwrite historical evidence.
            with target.open("x", encoding="utf-8", buffering=1) as handle:
                handle.write(json.dumps({"event": "trace_start", "schema": "piper.motion_trace.v1",
                    "time_ns": time.time_ns(), "pid": os.getpid(), "action_names": ACTION_NAMES,
                    "units": ["rad"] * 6 + ["m"] + ["rad"] * 6 + ["m"],
                    "feedback": "pre-dispatch measured position; SDK success does not prove arrival"}) + "\n")
                self.ready = True
                while not self._closed.is_set() or not self._queue.empty():
                    try:
                        sample = self._queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    try:
                        handle.write(json.dumps(sample, allow_nan=False, separators=(",", ":")) + "\n")
                        self.written += 1
                        self._stats.observe(sample)
                        self._stats_snapshot = self._stats.snapshot()
                    finally:
                        self._queue.task_done()
                handle.write(json.dumps({"event": "trace_end", "time_ns": time.time_ns(),
                                         **self.status()}) + "\n")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self.ready = False

    def close(self, timeout: float = 1.0) -> None:
        self._closed.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
