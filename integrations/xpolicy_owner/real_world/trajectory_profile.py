"""Explicit joint-trajectory processing before the robot owner's safety gate.

The quintic splice and rtg_v1 fit follow CoCoMatrix/robot_infra at
6376e4d4efec9d873d10791b30e07fd3268fbb2b. History is policy-emitted targets,
not measured state or an assertion that hardware accepted those targets.
"""
from collections import deque
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from XPolicyLab.real_world.trajectory_smoothing import smooth_action_prefix
from XPolicyLab.real_world.rtc import quintic_splice


def load_profile(path):
    data = json.loads(Path(path).read_text())
    required = {"schema", "algorithm", "action_schema", "frequency_hz",
                "execution_steps", "continuous_indices", "history_steps",
                "smoothness_weight", "fusion_decay", "max_target_deviation_rad",
                "blend_steps", "request_remaining_steps", "provenance"}
    optional = {"trajectory_time_scale", "synchronize_gripper_transition"}
    if not required <= set(data) or set(data) - required - optional or data["schema"] != "xpolicy.trajectory-profile/v1":
        raise ValueError("Invalid trajectory profile schema")
    if type(data.get("synchronize_gripper_transition", False)) is not bool:
        raise ValueError("synchronize_gripper_transition must be a boolean")
    if "trajectory_time_scale" in data:
        from XPolicyLab.real_world.action_timebase import ActionTimebase
        ActionTimebase(data["trajectory_time_scale"])
    if data["algorithm"] != "quintic_rtg_v1":
        raise ValueError("Unsupported trajectory algorithm")
    for key in ("history_steps", "blend_steps", "execution_steps", "request_remaining_steps"):
        if type(data[key]) is not int or data[key] < 2:
            raise ValueError("Invalid trajectory integer: " + key)
    if not data["blend_steps"] < data["request_remaining_steps"] < data["execution_steps"]:
        raise ValueError("Require blend < prefetch remaining < execution steps")
    for key in ("frequency_hz", "smoothness_weight", "fusion_decay", "max_target_deviation_rad"):
        if not np.isfinite(data[key]) or data[key] <= 0:
            raise ValueError("Invalid trajectory scalar: " + key)
    indices = data["continuous_indices"]
    if not isinstance(indices, list) or not indices or any(type(i) is not int or i < 0 for i in indices) or len(set(indices)) != len(indices):
        raise ValueError("Invalid continuous dimensions")
    return data


class TrajectoryProcessor:
    def __init__(self, profile, schema, frequency_hz):
        self.profile = profile
        if profile["action_schema"] != schema.to_wire() or profile["frequency_hz"] != frequency_hz:
            raise ValueError("Trajectory profile does not match action schema/cadence")
        self.indices = np.asarray(profile["continuous_indices"], dtype=np.intp)
        if np.any(self.indices >= schema.dim) or any(schema.units[i] != "rad" or schema.semantics[i] != "absolute_joint" for i in self.indices):
            raise ValueError("RTG requires explicit absolute joint radians")
        self.dim = schema.dim
        self.splice_indices = self.indices
        if profile.get("synchronize_gripper_transition", False):
            grippers = [i for i in range(schema.dim)
                        if schema.semantics[i] == "absolute_gripper_width" and schema.units[i] == "m"]
            if not grippers or set(self.indices) | set(grippers) != set(range(schema.dim)):
                raise ValueError("Synchronized transition requires only absolute joints and gripper widths")
            self.splice_indices = np.r_[self.indices, grippers]
        self.history = deque(maxlen=profile["history_steps"])
        self.last_report = {}

    def reset(self):
        self.history.clear()
        self.last_report = {}

    def emitted(self, row):
        self.history.append(row.copy())

    def process(self, old, raw):
        candidate = raw.copy()
        count = min(self.profile["blend_steps"], len(old), len(raw))
        if count >= 2:
            candidate = quintic_splice(old, raw, blend_steps=count, continuous_indices=self.splice_indices)
        history = np.asarray(tuple(self.history), dtype=np.float32).reshape(-1, self.dim)
        result = smooth_action_prefix(
            candidate, history, action_hz=self.profile["frequency_hz"],
            continuous_indices=self.indices, preserve_first_target=True,
            **{key: self.profile[key] for key in ("history_steps", "smoothness_weight",
               "fusion_decay", "max_target_deviation_rad")},
        )
        if result.fallback_reason:
            raise ValueError("RTG solver failed: " + result.fallback_reason)
        self.last_report = {
            "trajectory_algorithm": self.profile["algorithm"],
            "trajectory_history_source": "policy_emitted_targets",
            "trajectory_blend_steps": count if count >= 2 else 0,
            "trajectory_blend_unavailable": count < 2,
            "trajectory_gripper_transition": "quintic" if self.profile.get("synchronize_gripper_transition", False) else "passthrough",
            "trajectory_rtg_applied": result.applied,
            "trajectory_max_fit_deviation_rad": result.max_target_deviation_rad,
            "trajectory_prepare_ms": result.prepare_ms,
            "trajectory_before": asdict(result.before), "trajectory_after": asdict(result.after),
        }
        return result.values
