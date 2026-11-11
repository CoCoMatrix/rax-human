"""Pure, proposal-bound PiPER review; no simulator, policy inference or dispatch.

Public API: ``make_binding(request_id, observation, proposal, step_id=...)`` and
``PiperReviewer(contract, kinematics, gate_module_path=...).review(observation,
proposal, response, envelope=ReviewEnvelope(...), candidate_envelope=...)``.
Deployment integration uses ``PiperReviewer.with_v9(contract, kinematics,
gate_module_path=..., asset_root=...)`` to verify the supplied v9 description
and require the intersection of owner/v9 command limits. Plain construction
remains available for prior owner-only offline callers.

Every response echoes ``binding.as_dict()`` and supplies ``mode``, ``reason``,
the upstream outcome/intent ``assessment``, and (except stop) integer ``steps``.
Student selects at most 16 original 14D actions, including their width commands.
Edit/eef select at most five. Both require ``coordinate_frame=dual_base_link``
and explicit left_arm/right_arm objects in ``edit``/``target`` respectively.
Each arm names ``link`` (left_link6/right_link6). Edit uses
``delta_position_m`` and ``delta_rotation_vector_rad`` (base-frame, left-applied
rotation). Eef uses absolute ``position_m`` and ``quaternion_wxyz``. Corrections
default to ``gripper: 'keep'`` (the original measured width); an explicit change
is ``gripper: {'width_m': 0.02}``, never an open/closed boolean.

The caller must atomically reserve/consume request IDs across concurrent calls
and pass its used IDs in ReviewEnvelope. This pure validator does not persist a
replay ledger or authorize execution. Stop returns candidate=None. The owner
remains responsible for real-time freshness, arbitration and collision safety.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import (CandidateEnvelope, PiperContract, PiperObservation,
                        ValidatedCandidate, _freeze, _thaw, require, vector_schema)
from .kinematics import PiperKinematics, transform_from_pose

ARMS = ("left_arm", "right_arm")
LINKS = ("left_link6", "right_link6")
FRAME = "dual_base_link"
MAX_TRANSLATION_M = 0.05
MAX_ROTATION_RAD = 0.35


def proposal_sha256(proposal: ValidatedCandidate) -> str:
    """Bind exact dtype, full horizon, cadence, prefix, robot identity and bytes."""
    require(isinstance(proposal, ValidatedCandidate), "proposal must be a checked candidate")
    header = {"shape": list(proposal.actions.shape), "dtype": proposal.actions.dtype.str,
              "frequency_hz": proposal.frequency_hz, "prefix_steps": proposal.prefix_steps,
              "identity": dict(proposal.identity), "schema": vector_schema(action=True)}
    digest = hashlib.sha256(json.dumps(header, sort_keys=True, separators=(",", ":"),
                                       allow_nan=False).encode())
    digest.update(np.ascontiguousarray(proposal.actions).tobytes())
    return digest.hexdigest()


@dataclass(frozen=True, slots=True)
class ReviewBinding:
    request_id: str
    observation_sequence: int
    observation_timestamp_ns: int
    task: str
    proposal_sha256: str
    step_id: int

    def __post_init__(self):
        require(isinstance(self.request_id, str) and 0 < len(self.request_id) <= 256
                and self.request_id.strip() == self.request_id, "invalid request_id")
        for field in ("observation_sequence", "step_id"):
            value = getattr(self, field)
            require(type(value) is int and value >= 0, f"invalid {field}")
        require(type(self.observation_timestamp_ns) is int and self.observation_timestamp_ns > 0,
                "invalid observation_timestamp_ns")
        require(isinstance(self.task, str) and bool(self.task.strip()), "invalid task")
        require(isinstance(self.proposal_sha256, str) and len(self.proposal_sha256) == 64
                and all(c in "0123456789abcdef" for c in self.proposal_sha256), "invalid proposal digest")

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def make_binding(request_id: str, observation: PiperObservation,
                 proposal: ValidatedCandidate, *, step_id: int) -> ReviewBinding:
    return ReviewBinding(request_id, observation.sequence, observation.timestamp_ns,
                         observation.instruction, proposal_sha256(proposal), step_id)


@dataclass(frozen=True, slots=True)
class ReviewEnvelope:
    expected_binding: ReviewBinding
    now_ns: int
    max_observation_age_ms: float
    consumed_request_ids: frozenset[str]

    def __post_init__(self):
        require(isinstance(self.expected_binding, ReviewBinding), "expected binding required")
        require(type(self.now_ns) is int and self.now_ns > 0, "explicit current timestamp required")
        limit = self.max_observation_age_ms
        require(isinstance(limit, (int, float)) and not isinstance(limit, bool)
                and np.isfinite(limit) and limit > 0, "explicit finite freshness budget required")
        require(isinstance(self.consumed_request_ids, frozenset)
                and all(isinstance(item, str) for item in self.consumed_request_ids),
                "explicit frozen caller replay ledger required")


@dataclass(frozen=True, slots=True)
class ReviewedCandidate:
    mode: str
    candidate: ValidatedCandidate | None
    binding: ReviewBinding
    assessment_gate: str
    metadata: Mapping[str, Any]
    dispatch_authorized: bool = False

    def to_policy_result(self) -> dict[str, Any]:
        require(self.candidate is not None, "stop has no ActionChunk or executable policy result")
        result = self.candidate.to_policy_result()
        result["policy_metadata"]["review"] = {
            "mode": self.mode, "binding": self.binding.as_dict(),
            "assessment_gate": self.assessment_gate, "dispatch_authorized": False}
        if "v9_description_identity" in self.metadata:
            result["policy_metadata"]["review"].update({
                "v9_description_identity": _thaw(self.metadata["v9_description_identity"]),
                "camera_calibration_status": "nominal_geometry_only",
                "collision_certified": False})
        result["policy_metadata"]["execute_prefix_steps"] = self.candidate.prefix_steps
        return result


def _vector(value: Any, length: int, name: str) -> np.ndarray:
    require(isinstance(value, (list, tuple, np.ndarray)), f"{name} must be an explicit numeric vector")
    raw = np.asarray(value)
    require(raw.shape == (length,) and raw.dtype.kind in "fi"
            and bool(np.isfinite(raw).all()), f"{name} must have {length} finite numeric entries")
    # Reject booleans hidden in a mixed Python list before NumPy promotes them.
    require(not any(isinstance(v, (bool, np.bool_)) for v in value), f"{name} rejects booleans")
    return raw.astype(np.float64)


def _width(value: Any, measured_width: float) -> float:
    if isinstance(value, str) and value == "keep":
        return measured_width
    require(isinstance(value, Mapping) and set(value) == {"width_m"},
            "gripper requires keep or an explicit width_m; booleans are forbidden")
    width = value["width_m"]
    require(isinstance(width, (float, int)) and not isinstance(width, bool)
            and np.isfinite(width), "width_m must be finite numeric metres")
    return float(width)


def _rotation_distance(left: np.ndarray, right: np.ndarray) -> float:
    cosine = (np.trace(left @ right.T) - 1) / 2
    return float(np.arccos(np.clip(cosine, -1, 1)))


class PiperReviewer:
    def __init__(self, contract: PiperContract, kinematics: PiperKinematics, *,
                 gate_module_path: str | Path, description: Any | None = None):
        """Load only upstream gate_assessment.py, never its simulation package."""
        path = Path(gate_module_path).resolve(strict=True)
        require(path.name == "gate_assessment.py", "select the upstream pure gate module")
        spec = importlib.util.spec_from_file_location("_piper_upstream_gate", path)
        require(spec is not None and spec.loader is not None, "cannot load assessment gate")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        require(callable(getattr(module, "validate_assessment", None)), "upstream gate missing")
        self.contract, self.kinematics = contract, kinematics
        require(kinematics.contract is contract, "kinematics must use the same contract object")
        self.description = description
        if description is not None:
            require(description.contract is contract, "v9 description must use the same contract object")
            description.verify_identity()
            require(isinstance(description.identity, Mapping), "v9 description identity must be a mapping")
        self._gate = module.validate_assessment
        self.gate_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()

    @classmethod
    def with_v9(cls, contract: PiperContract, kinematics: PiperKinematics, *,
                gate_module_path: str | Path, asset_root: str | Path) -> PiperReviewer:
        """Use verified v9 geometry/limits without claiming camera calibration.

        Asset loading is offline only. The owner still supplies FK/IK and remains
        the sole hardware writer; v9 can narrow, never expand, accepted actions.
        """
        from .description import PiperDescription

        return cls(contract, kinematics, gate_module_path=gate_module_path,
                   description=PiperDescription(contract, asset_root))

    def _validate_description_actions(self, actions: np.ndarray) -> None:
        if self.description is not None:
            self.description.verify_identity()
            self.description.validate_candidate_limits(actions)
            self.description.validate_candidate_limits(actions.astype(np.float32))

    def review(self, observation: PiperObservation, proposal: ValidatedCandidate,
               response: Mapping[str, Any], *, envelope: ReviewEnvelope,
               candidate_envelope: CandidateEnvelope) -> ReviewedCandidate:
        """Return a fully checked candidate, or stop with None; failure raises ValueError.

        The caller supplies current time/freshness, replay ledger, expected request
        binding and motion envelope. Errors must become HOLD, never fallback to
        the student proposal. A successful return still grants no hardware writes.
        """
        require(isinstance(observation, PiperObservation), "validated observation required")
        require(isinstance(envelope, ReviewEnvelope), "explicit review envelope required")
        require(isinstance(candidate_envelope, CandidateEnvelope), "explicit candidate envelope required")
        binding = envelope.expected_binding
        require(binding.request_id not in envelope.consumed_request_ids, "request_id replay rejected")
        require(binding == make_binding(binding.request_id, observation, proposal, step_id=binding.step_id),
                "request binding no longer matches observation/task/proposal")
        age_ms = (envelope.now_ns - observation.timestamp_ns) / 1e6
        require(0 <= age_ms <= envelope.max_observation_age_ms, "review observation is stale or future")
        require(dict(observation.identity) == dict(self.contract.identity)
                and dict(proposal.identity) == dict(self.contract.identity), "review robot identity mismatch")
        require(observation.robot_id == self.contract.robot_id, "review robot_id mismatch")
        self.contract.verify_identity()
        if self.description is not None:
            self.description.verify_identity()
            # Complete proposal, including the unselected suffix: a short prefix
            # must never conceal an out-of-contract v9 joint target.
            self._validate_description_actions(proposal.actions)
        require(isinstance(response, Mapping), "review response must be a mapping")
        echoed = response.get("binding")
        require(isinstance(echoed, Mapping) and set(echoed) == set(binding.as_dict()),
                "review response binding mismatch")
        require(ReviewBinding(**dict(echoed)) == binding, "review response binding mismatch")
        mode = response.get("mode")
        require(mode in ("student", "edit", "eef", "stop"), "expected student/edit/eef/stop")
        allowed = {"binding", "mode", "reason", "assessment"}
        if mode != "stop": allowed.add("steps")
        if mode in ("edit", "eef"): allowed.update(("coordinate_frame", "edit" if mode == "edit" else "target"))
        require(set(response) <= allowed, "unexpected response fields or wrong-mode controls")
        require(isinstance(response.get("reason"), str) and bool(response["reason"].strip()),
                "visible decision reason required")
        gate = self._gate(dict(response), {"step_id": binding.step_id})
        detail = {"reason": response["reason"], "assessment": response["assessment"],
                  "upstream_gate_sha256": self.gate_sha256, "coordinate_frame": FRAME,
                  "endpoint_links": LINKS, "collision_certified": False}
        if self.description is not None:
            detail.update(v9_description_identity=dict(self.description.identity),
                          camera_calibration_status="nominal_geometry_only")
        metadata = _freeze(detail)
        if mode == "stop":
            return ReviewedCandidate(mode, None, binding, gate, metadata)
        steps = response.get("steps")
        cap = min(16 if mode == "student" else 5, proposal.prefix_steps, len(proposal.actions))
        require(type(steps) is int and 1 <= steps <= cap, f"steps must be 1..{cap}")
        # Recheck the complete proposal against current contract; a dataclass is
        # not a security boundary, and caller-owned arrays may have been edited.
        self.contract.validate_candidate(proposal.actions, schema=vector_schema(action=True),
                                         frequency_hz=proposal.frequency_hz,
                                         measured_state14=observation.state14,
                                         prefix_steps=proposal.prefix_steps)
        if mode == "student":
            actions = np.array(proposal.actions[:steps], copy=True)
        else:
            actions = self._correct(observation, proposal, response, steps)
        checked = self.contract.validate_candidate(actions, schema=vector_schema(action=True),
                                                    frequency_hz=proposal.frequency_hz,
                                                    measured_state14=observation.state14,
                                                    prefix_steps=steps, envelope=candidate_envelope)
        self._validate_description_actions(checked.actions)
        return ReviewedCandidate(mode, checked, binding, gate, metadata)

    def _correct(self, observation, proposal, response, steps):
        from scipy.spatial.transform import Rotation

        mode = response["mode"]
        require(response.get("coordinate_frame") == FRAME, "correction frame must be dual_base_link")
        edits = response.get("edit" if mode == "edit" else "target")
        require(isinstance(edits, Mapping) and set(edits) == set(ARMS), "explicit left_arm/right_arm required")
        specifications, widths = [], []
        for index, (arm, link) in enumerate(zip(ARMS, LINKS, strict=True)):
            item = edits[arm]
            require(isinstance(item, Mapping) and item.get("link") == link, "explicit PiPER flange link required")
            keys = ({"delta_position_m", "delta_rotation_vector_rad"} if mode == "edit"
                    else {"position_m", "quaternion_wxyz"})
            require(keys <= set(item) and set(item) <= keys | {"link", "gripper"}, "invalid correction fields")
            widths.append(_width(item.get("gripper", "keep"), float(observation.state14[6 + 7 * index])))
            if mode == "edit":
                translation = _vector(item["delta_position_m"], 3, "delta_position_m")
                rotation = _vector(item["delta_rotation_vector_rad"], 3, "delta_rotation_vector_rad")
                require(float(np.linalg.norm(translation)) <= MAX_TRANSLATION_M + 1e-12,
                        "edit exceeds 5 cm Euclidean correction")
                require(float(np.linalg.norm(rotation)) <= MAX_ROTATION_RAD + 1e-12,
                        "edit exceeds 0.35 rad correction")
                specifications.append((translation, Rotation.from_rotvec(rotation).as_matrix()))
            else:
                specifications.append(transform_from_pose(_vector(item["position_m"], 3, "position_m"),
                                                           _vector(item["quaternion_wxyz"], 4, "quaternion_wxyz")))
        seed = np.array(observation.state14, copy=True)
        measured_pose = self.kinematics.forward(seed)
        rows = []
        for i in range(steps):
            base = self.kinematics.forward(proposal.actions[i]) if mode == "edit" else measured_pose
            targets = []
            for j, original in enumerate((base.left_transform, base.right_transform)):
                if mode == "edit":
                    delta, rotation = specifications[j]
                    target = np.array(original, copy=True)
                    target[:3, 3] += delta
                    target[:3, :3] = rotation @ original[:3, :3]
                else:
                    target = specifications[j]
                require(float(np.linalg.norm(target[:3, 3] - original[:3, 3])) <= MAX_TRANSLATION_M + 1e-12,
                        "target exceeds 5 cm Euclidean correction")
                require(_rotation_distance(target[:3, :3], original[:3, :3]) <= MAX_ROTATION_RAD + 1e-12,
                        "target exceeds 0.35 rad correction")
                targets.append(target)
            solved = self.kinematics.inverse(seed, left_target=targets[0], right_target=targets[1],
                                             coordinate_frame=FRAME)
            action = np.array(solved.action14, copy=True)
            action[[6, 13]] = widths
            self.contract.validate_command_state(action)
            rows.append(action)
            seed = action
        return np.stack(rows)
