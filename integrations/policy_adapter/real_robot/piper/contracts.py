"""Strict adapters for the authoritative XPolicy PiPER wire contract.

No robot SDK, devices, camera driver or model is imported. Profile and URDF
identity is established locally; current ObservationFrame messages do not carry
those digests or individual image capture times, so this module does not claim
to verify those facts from observation metadata.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import os
import importlib
from pathlib import Path
import sys
import time
from types import MappingProxyType
from typing import Any

import numpy as np

FIELDS = (*(f"left_joint{i}" for i in range(1, 7)), "left_gripper",
          *(f"right_joint{i}" for i in range(1, 7)), "right_gripper")
JOINT_FIELDS = FIELDS[:6] + FIELDS[7:13]
JOINT_COLUMNS = (0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12)
GRIPPER_COLUMNS = (6, 13)
CAMERA_ROLES = ("camera_head", "camera_left", "camera_right")
UNITS = ("rad",) * 6 + ("m",) + ("rad",) * 6 + ("m",)
SEMANTICS = (("absolute_joint",) * 6 + ("absolute_gripper_width",)
             + ("absolute_joint",) * 6 + ("absolute_gripper_width",))
STATE_SCHEMA_ID = "xpolicy.piper_x_dual.state.v1"
ACTION_SCHEMA_ID = "xpolicy.piper_x_dual.joint_position.v1"


class ContractError(ValueError):
    """A PiPER observation or action cannot be consumed without guessing."""

    def __init__(self, message: str, *, details: dict | None = None):
        super().__init__(message)
        self.details = details or {}


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ContractError(reason)


def vector_schema(*, action: bool = False) -> dict[str, Any]:
    return {"schema_id": ACTION_SCHEMA_ID if action else STATE_SCHEMA_ID,
            "fields": list(FIELDS), "units": list(UNITS), "semantics": list(SEMANTICS)}


def _readonly(value: np.ndarray) -> np.ndarray:
    array = np.array(value, copy=True)
    array.setflags(write=False)
    return array


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, np.ndarray):
        return _readonly(value)
    return value


def _thaw(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw(item) for item in value]
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def runtime_module(repo_root: Path, suffix: str):
    """Import only from the explicitly selected local runtime checkout."""
    root = repo_root.resolve(strict=True)
    for name, module in tuple(sys.modules.items()):
        if name == "XPolicyLab" or name.startswith("XPolicyLab."):
            filename = getattr(module, "__file__", None)
            if filename is not None:
                require(Path(filename).resolve().is_relative_to(root),
                        f"already loaded runtime module from another checkout: {name}")
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    module = importlib.import_module("XPolicyLab." + suffix)
    require(Path(module.__file__).resolve().is_relative_to(root),
            "runtime module resolved outside the selected checkout")
    return module


def state14_array(value: Any, *, name: str = "state14") -> np.ndarray:
    require(isinstance(value, np.ndarray), f"{name} must be a ndarray; no implicit conversion")
    require(value.shape == (14,), f"{name} must have exact shape (14,), no flattening")
    require(value.dtype in (np.dtype("float32"), np.dtype("float64")),
            f"{name} must use float32 or float64")
    require(bool(np.isfinite(value).all()), f"{name} contains NaN or Inf")
    return _readonly(value)


def _candidate_metrics(actions: np.ndarray, measured: np.ndarray, hz: float) -> dict[str, float]:
    joint_actions = actions[:, JOINT_COLUMNS].astype(np.float64)
    joints = np.vstack([measured[list(JOINT_COLUMNS)], joint_actions])
    steps = np.diff(joints, axis=0)
    return {"max_joint_delta_from_measured_rad": float(np.max(np.abs(joint_actions - joints[0]))),
            "max_joint_step_rad": float(np.max(np.abs(steps))),
            "max_joint_velocity_rad_s": float(np.max(np.abs(steps))) * hz,
            "max_joint_acceleration_rad_s2": (float(np.max(np.abs(np.diff(steps, axis=0))))
                                                * hz**2 if len(steps) > 1 else 0.0),
            "max_gripper_delta_m": float(np.max(np.abs(actions[:, GRIPPER_COLUMNS]
                                             - measured[list(GRIPPER_COLUMNS)])))}


@dataclass(frozen=True, slots=True)
class CandidateEnvelope:
    """Optional caller-declared restrictions, not invented robot limits."""

    max_joint_delta_from_measured_rad: float | None = None
    max_joint_step_rad: float | None = None
    max_joint_velocity_rad_s: float | None = None
    max_joint_acceleration_rad_s2: float | None = None
    max_gripper_delta_m: float | None = None

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if value is not None:
                require(isinstance(value, (int, float)) and not isinstance(value, bool)
                        and np.isfinite(value) and value > 0,
                        f"{name} must be finite and positive")


@dataclass(frozen=True, slots=True)
class PiperObservation:
    state14: np.ndarray
    images: Mapping[str, np.ndarray]
    instruction: str
    timestamp_ns: int
    sequence: int
    robot_id: str
    identity: Mapping[str, Any]
    age_ms: float
    metadata: Mapping[str, Any]

    def to_policy_observation(self) -> dict[str, Any]:
        return {"data_format_version": "xpolicylab.real_world.v1",
                "instruction": self.instruction,
                "vision": {role: {"color": self.images[role], "encoding": "rgb8"}
                           for role in CAMERA_ROLES},
                "state": {"vector": self.state14, "schema": vector_schema()},
                "additional_info": {"timestamp_ns": self.timestamp_ns,
                                    "sequence": self.sequence,
                                    "robot_type": "piper_x_dual", "robot_id": self.robot_id,
                                    "metadata": _thaw(self.metadata) | {
                                        "piper_backend_identity": dict(self.identity)}}}


@dataclass(frozen=True, slots=True)
class ValidatedCandidate:
    actions: np.ndarray
    frequency_hz: float
    prefix_steps: int
    identity: Mapping[str, Any]
    metrics: Mapping[str, float]

    def to_policy_result(self) -> dict[str, Any]:
        """Preserve the complete checked horizon; the formal runtime owns prefix execution."""
        return {"actions": self.actions, "action_schema": vector_schema(action=True),
                "action_frequency_hz": self.frequency_hz,
                "policy_metadata": {"piper_backend_identity": dict(self.identity),
                                    "validated_prefix_steps": self.prefix_steps,
                                    "candidate_metrics": dict(self.metrics)}}


@dataclass(frozen=True, slots=True)
class PiperContract:
    repo_root: Path
    profile: Mapping[str, Any]
    profile_path: Path
    urdf_path: Path
    robot_id: str
    identity: Mapping[str, Any]
    lower14: np.ndarray
    upper14: np.ndarray
    horizon: int = 32
    action_prefix: int = 16
    frequency_hz: float = 30.0

    @classmethod
    def from_runtime(cls, repo_root: str | Path, *, robot_id: str = "piper_x_dual_01",
                     horizon: int = 32, action_prefix: int = 16,
                     trajectory_mode: bool = False,
                     expected_profile_sha256: str | None = None,
                     expected_urdf_sha256: str | None = None) -> PiperContract:
        max_horizon, max_prefix = (600,600) if trajectory_mode else (32, int(os.environ.get("PIPER_POLICY_PREFIX_STUDENT_MAX", "16")))
        require(type(horizon) is int and 1 <= horizon <= max_horizon, f"horizon must be 1..{max_horizon}")
        require(type(action_prefix) is int and 1 <= action_prefix <= min(horizon, max_prefix),
                f"action prefix must be 1..min(horizon,{max_prefix})")
        root = Path(repo_root).resolve(strict=True)
        owner = runtime_module(root, "real_world.robots.piper_x_dual.profile")
        profile_path = root / "real_world/robots/piper_x_dual/profile.yml"
        profile = owner.load_profile(profile_path)
        urdf_path = (profile_path.parent / profile["geometry"]["urdf_path"]).resolve()
        require(urdf_path.is_relative_to(root), "URDF resolved outside runtime checkout")
        owner.validate_urdf(profile, urdf_path)
        profile_sha, urdf_sha = _sha256(profile_path), _sha256(urdf_path)
        require(urdf_sha == profile["geometry"]["urdf_source_sha256"], "URDF digest mismatch")
        if expected_profile_sha256 is not None:
            require(profile_sha == expected_profile_sha256, "pinned profile digest mismatch")
        if expected_urdf_sha256 is not None:
            require(urdf_sha == expected_urdf_sha256, "pinned URDF digest mismatch")
        instance_path = root / profile["device_instance"]["default_path"]
        instance = owner.load_device_instance(instance_path)
        owner.validate_device_instance(profile, instance)
        require(instance["robot_id"] == robot_id, "configured robot_id does not match device instance")
        mapping = runtime_module(root, "real_world.robots.piper_x_dual.mapping")
        require(mapping.PIPER_X_DUAL_STATE_SCHEMA.to_wire() == vector_schema(),
                "runtime state schema drift")
        require(mapping.PIPER_X_DUAL_ACTION_SCHEMA.to_wire() == vector_schema(action=True),
                "runtime action schema drift")
        require(profile["geometry"]["convention"] == "x_forward_y_left_z_up",
                "unsupported axis convention")
        limits = [profile["body"]["grippers"][name]["limits"] if "gripper" in name
                  else profile["body"]["joint_limits_rad"][name] for name in FIELDS]
        bounds = np.asarray(limits, dtype=np.float64)
        identity = {"robot_type": "piper_x_dual", "robot_id": robot_id,
                    "profile_id": profile["profile_id"], "profile_sha256": profile_sha,
                    "urdf_sha256": urdf_sha, "device_instance_sha256": _sha256(instance_path),
                    "coordinate_frame": profile["geometry"]["root_frame"],
                    "axis_convention": profile["geometry"]["convention"],
                    "kinematics_only": True}
        return cls(root, _freeze(profile), profile_path, urdf_path, robot_id,
                   _freeze(identity), _readonly(bounds[:, 0]), _readonly(bounds[:, 1]),
                   horizon, action_prefix)

    def verify_identity(self) -> None:
        require(_sha256(self.profile_path) == self.identity["profile_sha256"],
                "profile changed since contract creation")
        require(_sha256(self.urdf_path) == self.identity["urdf_sha256"],
                "URDF changed since contract creation")
        instance_path = self.repo_root / self.profile["device_instance"]["default_path"]
        require(_sha256(instance_path) == self.identity["device_instance_sha256"],
                "device instance changed since contract creation")

    def validate_measured_state(self, value: Any) -> np.ndarray:
        state = state14_array(value)
        tolerance = np.full(14, self.profile["live_takeover"]["measured_state_limit_tolerance_rad"])
        tolerance[list(GRIPPER_COLUMNS)] = self.profile["read_only_probe"]["gripper_feedback_tolerance_m"]
        require(bool(np.all(state >= self.lower14 - tolerance)
                     and np.all(state <= self.upper14 + tolerance)),
                "measured state outside profile feedback tolerance")
        return state  # Never clip measurement noise or substitute an action target.

    def validate_command_state(self, value: Any) -> np.ndarray:
        state = state14_array(value, name="action14")
        bad = np.flatnonzero((state < self.lower14) | (state > self.upper14))
        if len(bad):
            raise ContractError("action outside exact profile command limits", details={
                "code": "command_limit", "violations": [
                    {"field": FIELDS[i], "value": float(state[i]), "unit": UNITS[i],
                     "lower": float(self.lower14[i]), "upper": float(self.upper14[i])}
                    for i in bad]})
        return state

    def encode_command_float32(self, value: Any) -> np.ndarray:
        """Encode legal generated commands within unchanged exact hard bounds.

        Validate the source BEFORE rounding. Never repairs illegal input or
        measured feedback. Supports one action or a batch; owns its output.
        """
        require(isinstance(value, np.ndarray) and value.dtype in (np.dtype('float32'), np.dtype('float64')),
                "command encoding requires explicit float32/float64 ndarray")
        require(value.ndim in (1, 2) and value.shape[-1] == 14 and value.size > 0,
                "command encoding requires (14,) or nonempty (N,14)")
        rows = value.reshape(-1, 14)
        for i, row in enumerate(rows):
            try:
                self.validate_command_state(row)
            except ContractError as exc:
                raise ContractError(str(exc), details={**exc.details, "sample_index": i}) from exc
        lower = self.lower14.astype(np.float32)
        upper = self.upper14.astype(np.float32)
        lower = np.where(lower.astype(np.float64) < self.lower14,
                         np.nextafter(lower, np.float32(np.inf)), lower)
        upper = np.where(upper.astype(np.float64) > self.upper14,
                         np.nextafter(upper, np.float32(-np.inf)), upper)
        require(bool(np.all(lower <= upper)), "profile has no representable float32 interval")
        wire = np.clip(value.astype(np.float32), lower, upper)
        for row in wire.reshape(-1, 14):
            self.validate_command_state(row)
        return wire

    def validate_observation(self, observation: Any, *, max_age_ms: float,
                             now_ns: int | None = None,
                             previous_sequence: int | None = None) -> PiperObservation:
        require(np.isfinite(max_age_ms) and max_age_ms > 0,
                "max_age_ms must be explicitly finite and positive")
        if not isinstance(observation, Mapping):
            frame_type = runtime_module(self.repo_root, "real_world.contracts").ObservationFrame
            require(isinstance(observation, frame_type), "expected XPolicy ObservationFrame or wire mapping")
            observation = observation.to_policy_observation()
        require(observation.get("data_format_version") == "xpolicylab.real_world.v1",
                "observation wire version mismatch")
        instruction = observation.get("instruction")
        require(isinstance(instruction, str) and bool(instruction.strip()), "instruction must be nonempty")
        raw_state = observation.get("state")
        require(isinstance(raw_state, Mapping), "missing observation state")
        require(raw_state.get("schema") == vector_schema(), "state schema/order/units/semantics mismatch")
        require(isinstance(raw_state.get("vector"), np.ndarray)
                and raw_state["vector"].dtype == np.float32,
                "wire state must be a float32 ndarray")
        state = self.validate_measured_state(raw_state["vector"])
        vision = observation.get("vision")
        require(isinstance(vision, Mapping) and set(vision) == set(CAMERA_ROLES),
                "exactly the three PiPER camera roles are required")
        images = {}
        for role in CAMERA_ROLES:
            camera = vision[role]
            require(isinstance(camera, Mapping), f"{role} must contain an RGB color image")
            require(camera.get("encoding", "rgb8") == "rgb8", f"{role} must already be rgb8")
            array = camera.get("color")
            require(isinstance(array, np.ndarray) and array.dtype == np.uint8
                    and array.ndim == 3 and array.shape[-1] == 3 and min(array.shape[:2]) > 0,
                    f"{role} must be nonempty uint8 HxWx3 RGB; no reshape or conversion")
            images[role] = _readonly(array)
        info = observation.get("additional_info")
        require(isinstance(info, Mapping) and info.get("robot_type") == "piper_x_dual"
                and info.get("robot_id") == self.robot_id, "observation robot identity mismatch")
        timestamp, sequence = info.get("timestamp_ns"), info.get("sequence")
        require(type(timestamp) is int and timestamp > 0, "timestamp_ns must be a positive integer")
        require(type(sequence) is int and sequence >= 0, "sequence must be a nonnegative integer")
        if previous_sequence is not None:
            require(sequence > previous_sequence, "observation sequence did not advance")
        now_ns = time.time_ns() if now_ns is None else now_ns
        require(type(now_ns) is int and now_ns > 0, "now_ns must be a positive integer")
        age = (now_ns - timestamp) / 1e6
        require(0 <= age <= max_age_ms, "observation is stale or timestamp is in the future")
        metadata = info.get("metadata", {})
        require(isinstance(metadata, Mapping), "observation metadata must be a mapping")
        if "piper_backend_identity" in metadata:
            require(metadata["piper_backend_identity"] == dict(self.identity),
                    "observation backend identity mismatch")
        self.verify_identity()
        return PiperObservation(state, MappingProxyType(images), instruction, timestamp,
                                sequence, self.robot_id, self.identity, age, _freeze(metadata))

    def validate_candidate(self, actions: Any, *, schema: Mapping[str, Any], frequency_hz: float,
                           measured_state14: np.ndarray, prefix_steps: int = 16,
                           envelope: CandidateEnvelope | None = None) -> ValidatedCandidate:
        require(schema == vector_schema(action=True), "action schema/order/units/semantics mismatch")
        require(frequency_hz == self.frequency_hz, "PiPER action frequency must be 30 Hz")
        require(isinstance(actions, np.ndarray) and actions.dtype in (np.dtype("float32"), np.dtype("float64"))
                and actions.ndim == 2 and actions.shape[1] == 14,
                "actions must be floating ndarray[N,14], no padding/truncation/flattening")
        require(1 <= len(actions) <= self.horizon, "action horizon exceeds declared candidate contract")
        require(type(prefix_steps) is int and 1 <= prefix_steps <= min(len(actions), self.action_prefix),
                "prefix_steps exceeds configured prefix or candidate length")
        require(bool(np.isfinite(actions).all()), "candidate contains NaN or Inf")
        require(bool(np.all(actions >= self.lower14) and np.all(actions <= self.upper14)),
                "candidate outside exact profile command limits")
        measured = self.validate_measured_state(measured_state14)
        converted = actions.astype(np.float32)
        require(bool(np.isfinite(converted).all()) and bool(np.all(converted >= self.lower14)
                                                          and np.all(converted <= self.upper14)),
                "float32 transport crosses command limits")
        original_metrics = _candidate_metrics(actions, measured, self.frequency_hz)
        transport_metrics = _candidate_metrics(converted, measured, self.frequency_hz)
        metrics = {name: max(actual, transport_metrics[name]) for name, actual in original_metrics.items()}
        if envelope is not None:
            require(isinstance(envelope, CandidateEnvelope), "envelope must be an explicit CandidateEnvelope")
            for name, actual in metrics.items():
                limit = getattr(envelope, name)
                if limit is not None:
                    require(actual <= limit, f"candidate exceeds declared {name}")
        # No implicit dtype conversion: callers receive exactly the checked input
        # dtype and values; actual float32 representability has additionally passed.
        self.verify_identity()
        return ValidatedCandidate(_readonly(actions), self.frequency_hz, prefix_steps,
                                  self.identity, MappingProxyType(metrics))
