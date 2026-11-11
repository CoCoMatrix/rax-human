"""Opt-in status evidence from the formal owner's already-connected SDK cache.

No hardware connections, query packets, enables, calibration or movement.
Only the audited Piper v189 cached getter interface is consumed.
"""
from __future__ import annotations

from enum import Enum
import json
import math
import os
from pathlib import Path
import time

ENV_NAME = "PIPER_READONLY_STATUS_OUTPUT"
OUTPUT_ROOT = Path("/home/ubuntu/code_junqi/migration")
ROLES = ("left_arm", "right_arm")
ARM_FIELDS = ("ctrl_mode", "arm_status", "mode_feedback", "teach_status",
              "motion_status", "trajectory_num")
ERROR_FIELDS = tuple(f"joint_{i}_angle_limit" for i in range(1, 7)) + tuple(
    f"communication_status_joint_{i}" for i in range(1, 7))
DRIVER_FIELDS = ("vol", "foc_temp", "motor_temp", "bus_current")
FOC_FIELDS = ("voltage_too_low", "motor_overheating", "driver_overcurrent",
              "driver_overheating", "collision_status", "driver_error_status",
              "driver_enable_status", "stall_status")
MOTOR_FIELDS = ("position", "velocity", "current", "torque")


def diagnostic_output_path(raw: str, *, write_enabled: bool) -> Path | None:
    if not raw.strip():
        return None
    if write_enabled:
        raise ValueError(f"{ENV_NAME} requires ENABLE_WRITE=0")
    path = Path(raw)
    if not path.is_absolute() or path.suffix != ".json":
        raise ValueError("cached diagnostic output must be an absolute new .json path")
    resolved = path.resolve()
    if not resolved.is_relative_to(OUTPUT_ROOT.resolve()):
        raise ValueError("cached diagnostic output must stay in personal migration/")
    if not path.parent.is_dir() or path.exists() or path.is_symlink():
        raise ValueError("cached diagnostic parent must exist and output must be new")
    return resolved


def _scalar(value):
    if isinstance(value, Enum):
        return {"name": value.name, "value": _scalar(value.value)}
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else {"non_finite": repr(value)}
    return {"unsupported_type": type(value).__name__, "repr": repr(value)}


def _fields(value, names):
    return {name: _scalar(getattr(value, name, None)) for name in names}


def _message(message, fields, *, nested=None):
    if message is None:
        return {"available": False}
    timestamp = getattr(message, "timestamp", None)
    captured_ns = time.time_ns()
    body = getattr(message, "msg", None)
    result = {"available": True, "read_ns": captured_ns,
              "timestamp_s": _scalar(timestamp),
              "hz": _scalar(getattr(message, "hz", None)),
              "fields": _fields(body, fields)}
    if isinstance(timestamp, (int, float)) and math.isfinite(timestamp):
        result["age_ms_at_read"] = (captured_ns / 1e9 - timestamp) * 1000.0
    if nested:
        name, names = nested
        value = getattr(body, name, None)
        result[name] = {"available": value is not None,
                        "fields": _fields(value, names)}
    return result


def _read(reader, fields, *, nested=None):
    try:
        return _message(reader(), fields, nested=nested)
    except Exception as exc:
        return {"available": False, "error_type": type(exc).__name__, "error": str(exc)}


def collect_cached_status(arms, *, write_enabled: bool) -> dict:
    """Read existing arms only; accepting a writer session is forbidden."""
    if write_enabled:
        raise ValueError("cached status evidence is no-write only")
    if set(arms) != set(ROLES):
        raise ValueError("both existing owner arm readers are required")
    report = {"schema": "junqi.piper.cached_status_diagnostic.v1",
              "started_ns": time.time_ns(), "write_enabled": False,
              "read_only_cached_getters": True, "new_sdk_connections": 0,
              "atomic_snapshot": False, "motion_eligibility_evaluated": False,
              "arms": {}}
    for role in ROLES:
        arm = arms[role]
        item = {"arm_status": _read(arm.get_arm_status, ARM_FIELDS,
                                   nested=("err_status", ERROR_FIELDS)),
                "driver_states": {}, "motor_states": {}}
        for index in range(1, 7):
            item["driver_states"][str(index)] = _read(
                lambda i=index: arm.get_driver_states(i), DRIVER_FIELDS,
                nested=("foc_status", FOC_FIELDS))
            item["motor_states"][str(index)] = _read(
                lambda i=index: arm.get_motor_states(i), MOTOR_FIELDS)
        try:
            item["enable_list"] = [_scalar(v) for v in arm.get_joints_enable_status_list()]
        except Exception as exc:
            item["enable_list_error"] = {"error_type": type(exc).__name__, "error": str(exc)}
        item["enable_list_note"] = (
            "SDK False can mean disabled OR missing driver feedback; "
            "inspect individual message availability/timestamps/flags. "
            "This separate cached read is not atomic with driver_states.")
        report["arms"][role] = item
    report["completed_ns"] = time.time_ns()
    return report


def write_cached_status(path: Path, arms, *, write_enabled: bool) -> dict:
    target = diagnostic_output_path(str(path), write_enabled=write_enabled)
    report = collect_cached_status(arms, write_enabled=write_enabled)
    data = (json.dumps(report, indent=2, allow_nan=False) + "\n").encode()
    descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                         getattr(os, "O_NOFOLLOW", 0), 0o600)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    return report
