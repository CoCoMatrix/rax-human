"""Pure review contract tests; mocked IK never connects to a robot or simulator."""
from dataclasses import replace
import os
from pathlib import Path
from types import MappingProxyType, SimpleNamespace

import numpy as np
import pytest

from real_robot.piper.contracts import (CandidateEnvelope, ContractError, PiperContract,
                                       PiperObservation, vector_schema)
from real_robot.piper.review import PiperReviewer, ReviewEnvelope, make_binding, proposal_sha256


def upstream_gate():
    explicit = os.environ.get("PIPER_UPSTREAM_GATE")
    if explicit:
        return Path(explicit)
    for root in Path(__file__).resolve().parents:
        candidates = (root / "hybrid_rollout/robodojo/robodojo_server/gate_assessment.py",
                      root / "gap-source-20260918/source/hybrid_rollout/robodojo/robodojo_server/gate_assessment.py")
        for path in candidates:
            if path.is_file():
                return path
    pytest.fail("Set PIPER_UPSTREAM_GATE to the preserved upstream gate_assessment.py")


class FakeKinematics:
    """Six joints map directly to xyz/rotation vector solely for protocol tests."""
    def __init__(self, contract):
        self.contract = contract
        self.calls = []
        self.fail = False

    def forward(self, state):
        from scipy.spatial.transform import Rotation
        transforms = []
        for offset in (0, 7):
            value = np.eye(4)
            value[:3, 3] = state[offset:offset + 3]
            value[:3, :3] = Rotation.from_rotvec(state[offset + 3:offset + 6]).as_matrix()
            transforms.append(value)
        return SimpleNamespace(left_transform=transforms[0], right_transform=transforms[1])

    def inverse(self, seed, *, left_target, right_target, coordinate_frame):
        from scipy.spatial.transform import Rotation
        self.calls.append((seed.copy(), left_target.copy(), right_target.copy(), coordinate_frame))
        if self.fail:
            raise ContractError("IK convergence failure")
        result = seed.astype(np.float64)
        for offset, target in ((0, left_target), (7, right_target)):
            result[offset:offset + 3] = target[:3, 3]
            result[offset + 3:offset + 6] = Rotation.from_matrix(target[:3, :3]).as_rotvec()
        return SimpleNamespace(action14=result)


@pytest.fixture
def setup(monkeypatch, tmp_path):
    monkeypatch.setattr(PiperContract, "verify_identity", lambda self: None)
    identity = MappingProxyType({"robot_type": "piper_x_dual", "robot_id": "piper_x_dual_01",
                                 "coordinate_frame": "dual_base_link", "profile_sha256": "a" * 64})
    lower = np.full(14, -3.0)
    upper = np.full(14, 3.0)
    lower[[6, 13]], upper[[6, 13]] = 0, .1
    profile = {"live_takeover": {"measured_state_limit_tolerance_rad": .01},
               "read_only_probe": {"gripper_feedback_tolerance_m": .005}}
    contract = PiperContract(tmp_path, profile, tmp_path / "profile.yml", tmp_path / "robot.urdf",
                             "piper_x_dual_01", identity, lower, upper)
    state = np.zeros(14, np.float32)
    state[[6, 13]] = [.012, .023]
    observation = PiperObservation(state, {}, "Place the cup.", 1_000_000_000, 9,
                                   "piper_x_dual_01", identity, 0, {})
    actions = np.tile(state, (32, 1))
    actions[:, 0] = np.arange(32) * .0001
    actions[:, 6] = .04
    proposal = contract.validate_candidate(actions, schema=vector_schema(action=True), frequency_hz=30,
                                           measured_state14=state, prefix_steps=16)
    kin = FakeKinematics(contract)
    reviewer = PiperReviewer(contract, kin, gate_module_path=upstream_gate())
    binding = make_binding("req-1", observation, proposal, step_id=1)
    envelope = ReviewEnvelope(binding, 1_050_000_000, 100, frozenset())
    candidate_envelope = CandidateEnvelope(max_joint_delta_from_measured_rad=.2, max_joint_step_rad=.1,
                                           max_joint_velocity_rad_s=3, max_gripper_delta_m=.05)
    return SimpleNamespace(contract=contract, observation=observation, proposal=proposal, kin=kin,
                           reviewer=reviewer, binding=binding, envelope=envelope, limits=candidate_envelope)


def response(s, mode="student", steps=5):
    value = {"mode": mode, "binding": s.binding.as_dict(), "reason": "Observed target needs correction.",
             "assessment": {"task_progress": {"verified_completed": [], "remaining": ["Place cup"],
                                                   "currently_attempting": "Place cup"},
                            "current_subgoal": "Place cup", "execution_status": "failed",
                            "execution_evidence": "Cup did not move.", "expected_next_intent": "Retry grasp",
                            "predicted_next_intent": "Place cup", "intent_status": "misaligned",
                            "intent_evidence": "Student flange path approaches coaster before cup was lifted."}}
    if mode != "stop":
        value["steps"] = steps
    if mode in ("edit", "eef"):
        value["coordinate_frame"] = "dual_base_link"
        field = "edit" if mode == "edit" else "target"
        value[field] = {}
        for arm in ("left", "right"):
            item = {"link": arm + "_link6"}
            if mode == "edit":
                item.update(delta_position_m=[.01, 0., 0.], delta_rotation_vector_rad=[0., 0., .05])
            else:
                item.update(position_m=[.02, 0., 0.], quaternion_wxyz=[1., 0., 0., 0.])
            value[field][arm + "_arm"] = item
    return value


def review(s, value=None, **kwargs):
    return s.reviewer.review(s.observation, s.proposal, value or response(s),
                             envelope=kwargs.get("envelope", s.envelope),
                             candidate_envelope=kwargs.get("candidate_envelope", s.limits))


def test_student_preserves_model_absolute_gripper_and_at_most_sixteen(setup):
    s = setup
    result = review(s, response(s, steps=16))
    assert result.mode == "student" and result.candidate.actions.shape == (16, 14)
    np.testing.assert_array_equal(result.candidate.actions, s.proposal.actions[:16])
    assert result.candidate.actions[0, 6] != s.observation.state14[6]
    assert not result.candidate.actions.flags.writeable and not result.dispatch_authorized
    assert result.to_policy_result()["policy_metadata"]["execute_prefix_steps"] == 16
    assert not s.kin.calls


@pytest.mark.parametrize("mode,steps", [("student", 17), ("edit", 6), ("eef", 6), ("student", True), ("edit", 0)])
def test_prefix_limits_reject_instead_of_silently_truncating(setup, mode, steps):
    with pytest.raises(ContractError, match="steps"):
        review(setup, response(setup, mode, steps))


@pytest.mark.parametrize("mode", ["edit", "eef"])
def test_correction_uses_dual_flange_ik_and_measured_width_default(setup, mode):
    s = setup
    result = review(s, response(s, mode))
    assert result.candidate.actions.shape == (5, 14) and len(s.kin.calls) == 5
    np.testing.assert_array_equal(result.candidate.actions[:, [6, 13]],
                                  np.tile(s.observation.state14[[6, 13]], (5, 1)))
    assert all(call[3] == "dual_base_link" for call in s.kin.calls)
    assert result.assessment_gate == "both"


def test_explicit_width_metres_and_keep_are_independent(setup):
    s = setup
    value = response(s, "edit")
    value["edit"]["left_arm"]["gripper"] = {"width_m": .03}
    value["edit"]["right_arm"]["gripper"] = "keep"
    result = review(s, value)
    np.testing.assert_allclose(result.candidate.actions[:, 6], .03)
    np.testing.assert_array_equal(result.candidate.actions[:, 13], np.repeat(s.observation.state14[13], 5))


@pytest.mark.parametrize("bad", [True, False, "open", "closed", {"gripper_closed": True}, {"width_m": True},
                                 {"width_m": -0.001}, {"width_m": .101}, {"width_m": float("nan")}])
def test_arx_boolean_and_invalid_gripper_values_rejected(setup, bad):
    value = response(setup, "edit")
    value["edit"]["left_arm"]["gripper"] = bad
    with pytest.raises(ContractError):
        review(setup, value)


@pytest.mark.parametrize("mode", ["edit", "eef"])
def test_uncertainty_does_not_authorize_takeover(setup, mode):
    value = response(setup, mode)
    value["assessment"].update(execution_status="uncertain", intent_status="uncertain")
    with pytest.raises(ValueError, match="observed failure or wrong task intent"):
        review(setup, value)


def test_failed_student_can_self_recover_and_stop_never_produces_chunk(setup):
    assert review(setup).assessment_gate == "none"
    stopped = review(setup, response(setup, "stop"))
    assert stopped.candidate is None and not setup.kin.calls
    with pytest.raises(ContractError, match="no ActionChunk"):
        stopped.to_policy_result()


@pytest.mark.parametrize("fault", ["request", "sequence", "timestamp", "task", "proposal", "replay", "stale", "future"])
def test_binding_freshness_and_explicit_replay_ledger(setup, fault):
    s = setup
    value, envelope = response(s), s.envelope
    fields = {"request": "request_id", "sequence": "observation_sequence", "timestamp": "observation_timestamp_ns",
              "task": "task", "proposal": "proposal_sha256"}
    if fault in fields:
        key = fields[fault]
        value["binding"][key] = value["binding"][key] + (1 if type(value["binding"][key]) is int else "x")
    elif fault == "replay":
        envelope = replace(envelope, consumed_request_ids=frozenset({"req-1"}))
    elif fault == "stale":
        envelope = replace(envelope, now_ns=1_101_000_000)
    elif fault == "future":
        envelope = replace(envelope, now_ns=999_999_999)
    with pytest.raises(ContractError):
        review(s, value, envelope=envelope)


def test_proposal_hash_covers_dtype_full_horizon_and_prefix(setup):
    s = setup
    original = proposal_sha256(s.proposal)
    assert proposal_sha256(replace(s.proposal, prefix_steps=15)) != original
    assert proposal_sha256(replace(s.proposal, actions=s.proposal.actions.astype(np.float64))) != original
    changed = s.proposal.actions.copy()
    changed[-1, 0] += .001
    forged = replace(s.proposal, actions=changed)
    assert proposal_sha256(forged) != original
    with pytest.raises(ContractError, match="binding"):
        s.reviewer.review(s.observation, forged, response(s), envelope=s.envelope, candidate_envelope=s.limits)


@pytest.mark.parametrize("fault", ["euclidean", "rotation", "frame", "link", "one_arm", "quaternion", "legacy_units"])
def test_correction_limits_frames_and_units(setup, fault):
    value = response(setup, "eef" if fault == "quaternion" else "edit")
    field = "target" if fault == "quaternion" else "edit"
    arm = value[field]["left_arm"]
    if fault == "euclidean": arm["delta_position_m"] = [.04, .04, 0.]
    elif fault == "rotation": arm["delta_rotation_vector_rad"] = [0., 0., .3501]
    elif fault == "frame": value["coordinate_frame"] = "environment_origin"
    elif fault == "link": arm["link"] = "tcp"
    elif fault == "one_arm": del value[field]["right_arm"]
    elif fault == "quaternion": arm["quaternion_wxyz"] = [2., 0., 0., 0.]
    elif fault == "legacy_units": arm["delta_position"] = arm.pop("delta_position_m")
    with pytest.raises(ContractError):
        review(setup, value)


def test_eef_distance_is_measured_from_current_not_student(setup):
    value = response(setup, "eef")
    value["target"]["left_arm"]["position_m"] = [.04, .04, 0.]
    with pytest.raises(ContractError, match="5 cm Euclidean"):
        review(setup, value)


def test_no_fallback_when_ik_or_explicit_motion_envelope_fails(setup):
    setup.kin.fail = True
    with pytest.raises(ContractError, match="IK convergence"):
        review(setup, response(setup, "edit"))
    setup.kin.fail = False
    with pytest.raises(ContractError, match="max_joint_step_rad"):
        review(setup, response(setup, "edit"), candidate_envelope=CandidateEnvelope(max_joint_step_rad=.001))


def test_first_step_assessment_matches_upstream_gate(setup):
    s = setup
    binding = replace(s.binding, step_id=0)
    envelope = replace(s.envelope, expected_binding=binding)
    value = response(s)
    value["binding"] = binding.as_dict()
    with pytest.raises(ValueError, match="not_started"):
        review(s, value, envelope=envelope)
    value["assessment"]["execution_status"] = "not_started"
    assert review(s, value, envelope=envelope).candidate is not None


def test_binding_boolean_is_not_an_integer_sequence_or_step(setup):
    value = response(setup)
    value["binding"]["step_id"] = True
    with pytest.raises(ContractError, match="step_id"):
        review(setup, value)


def test_response_cannot_smuggle_actions_or_arx_commands(setup):
    value = response(setup)
    value["gripper_closed"] = True
    with pytest.raises(ContractError, match="unexpected response"):
        review(setup, value)


@pytest.mark.parametrize("use_v9", [False, True])
def test_actual_owner_kinematics_zero_edit_is_offline_only(use_v9):
    root = os.environ.get("PIPER_RUNTIME_ROOT")
    if root is None:
        pytest.skip("PIPER_RUNTIME_ROOT required for actual owner FK/IK acceptance")
    assets = os.environ.get("PIPER_DESCRIPTION_ASSETS")
    if use_v9 and assets is None:
        pytest.skip("PIPER_DESCRIPTION_ASSETS required for actual v9 review acceptance")
    from real_robot.piper.contracts import CAMERA_ROLES
    from real_robot.piper.kinematics import PiperKinematics

    contract = PiperContract.from_runtime(root)
    measured = np.zeros(14, dtype=np.float32)
    measured[[6, 13]] = [.0003, .0004]
    wire = {"data_format_version": "xpolicylab.real_world.v1", "instruction": "Hold the present pose.",
            "state": {"vector": measured, "schema": vector_schema()},
            "vision": {role: {"color": np.zeros((4, 4, 3), dtype=np.uint8), "encoding": "rgb8"}
                       for role in CAMERA_ROLES},
            "additional_info": {"timestamp_ns": 1_000_000_000, "sequence": 0,
                                "robot_type": "piper_x_dual", "robot_id": contract.robot_id}}
    observation = contract.validate_observation(wire, max_age_ms=100, now_ns=1_000_000_000)
    proposal = contract.validate_candidate(np.tile(measured, (32, 1)), schema=vector_schema(action=True),
                                           frequency_hz=30, measured_state14=measured, prefix_steps=16)
    binding = make_binding("offline-owner-fk-ik", observation, proposal, step_id=1)
    kin = PiperKinematics(contract)
    s = SimpleNamespace(binding=binding)
    value = response(s, "edit", steps=1)
    for item in value["edit"].values():
        item["delta_position_m"] = [0., 0., 0.]
        item["delta_rotation_vector_rad"] = [0., 0., 0.]
    reviewer = (PiperReviewer.with_v9(contract, kin, gate_module_path=upstream_gate(), asset_root=assets)
                if use_v9 else PiperReviewer(contract, kin, gate_module_path=upstream_gate()))
    result = reviewer.review(
        observation, proposal, value,
        envelope=ReviewEnvelope(binding, 1_000_000_000, 100, frozenset()),
        candidate_envelope=CandidateEnvelope(max_joint_delta_from_measured_rad=.03, max_gripper_delta_m=.001))
    assert result.candidate.actions.shape == (1, 14) and not result.dispatch_authorized
    np.testing.assert_array_equal(result.candidate.actions[0, [6, 13]], measured[[6, 13]])
    assert result.metadata["collision_certified"] is False
    assert ("v9_description_identity" in result.metadata) is use_v9


class FakeDescription:
    """Contract double with the actual v9 joint6 bound; no file/device access."""
    def __init__(self, contract):
        self.contract = contract
        self.identity = {"asset_urdf_sha256": "v9-test", "calibration": {"status": "nominal"}}
        self.fail = False
        self.validations = []

    def verify_identity(self):
        if self.fail:
            raise ContractError("v9 description asset identity changed")

    def validate_candidate_limits(self, actions):
        self.validations.append(actions.copy())
        if np.any(np.abs(actions[:, [5, 12]].astype(np.float64)) > 2.0943951):
            raise ContractError("candidate exceeds owner/v9 joint6 intersection")
        return {"checked": True}


def enable_v9(s):
    description = FakeDescription(s.contract)
    s.reviewer = PiperReviewer(s.contract, s.kin, gate_module_path=upstream_gate(), description=description)
    return description


def proposal_joint6(s, value, *, suffix_only=False, dtype=np.float64):
    actions = s.proposal.actions.astype(dtype)
    actions[-1 if suffix_only else slice(None), 5] = value
    s.proposal = s.contract.validate_candidate(actions, schema=vector_schema(action=True), frequency_hz=30,
                                               measured_state14=s.observation.state14, prefix_steps=16)
    s.binding = make_binding("req-v9", s.observation, s.proposal, step_id=1)
    s.envelope = replace(s.envelope, expected_binding=s.binding)
    s.limits = CandidateEnvelope(max_joint_delta_from_measured_rad=3, max_gripper_delta_m=.05)


@pytest.mark.parametrize("suffix_only", [False, True])
def test_v9_rejects_owner_allowed_joint6_including_unselected_suffix(setup, suffix_only):
    s = setup
    proposal_joint6(s, 2.2, suffix_only=suffix_only)
    # The original owner contract accepted the complete 32-step proposal above.
    assert review(s).candidate is not None  # Backward-compatible owner-only path.
    enable_v9(s)
    with pytest.raises(ContractError, match="v9 joint6"):
        review(s)
    assert not s.kin.calls


def test_v9_valid_joint6_and_identity_are_exported_without_calibration_claim(setup):
    import json

    s = setup
    proposal_joint6(s, 1.0)
    description = enable_v9(s)
    result = review(s)
    np.testing.assert_array_equal(result.candidate.actions[:, 5], np.ones(5))
    assert [value.dtype for value in description.validations] == [np.float64, np.float32, np.float64, np.float32]
    assert [len(value) for value in description.validations] == [32, 32, 5, 5]
    metadata = result.to_policy_result()["policy_metadata"]["review"]
    assert metadata["v9_description_identity"] == description.identity
    assert metadata["camera_calibration_status"] == "nominal_geometry_only"
    assert metadata["collision_certified"] is False and metadata["dispatch_authorized"] is False
    json.dumps(metadata)
    description.identity["asset_urdf_sha256"] = "changed-after-review"
    assert result.metadata["v9_description_identity"]["asset_urdf_sha256"] == "v9-test"


def test_v9_checks_float32_transport_bound_without_clipping(setup):
    s = setup
    proposal_joint6(s, 2.09439506)
    assert s.proposal.actions[0, 5] < 2.0943951
    assert float(np.float32(s.proposal.actions[0, 5])) > 2.0943951
    enable_v9(s)
    with pytest.raises(ContractError, match="v9 joint6"):
        review(s)
    assert s.proposal.actions[0, 5] == 2.09439506


@pytest.mark.parametrize("joint6", [2.2, 2.09439506])
def test_v9_rejects_corrected_candidate_even_if_original_proposal_is_valid(setup, monkeypatch, joint6):
    s = setup
    enable_v9(s)
    s.limits = CandidateEnvelope(max_joint_delta_from_measured_rad=3)
    original_inverse = s.kin.inverse

    def owner_allowed_but_v9_rejected(*args, **kwargs):
        value = original_inverse(*args, **kwargs)
        value.action14[5] = joint6
        return value

    monkeypatch.setattr(s.kin, "inverse", owner_allowed_but_v9_rejected)
    with pytest.raises(ContractError, match="v9 joint6"):
        review(s, response(s, "eef"))
    assert len(s.kin.calls) == 5


def test_v9_asset_change_fails_closed_instead_of_owner_only_fallback(setup):
    s = setup
    description = enable_v9(s)
    description.fail = True
    with pytest.raises(ContractError, match="asset identity changed"):
        review(s)
    assert not s.kin.calls


def test_v9_requires_the_identical_contract_object(setup):
    s = setup
    other_contract = replace(s.contract)
    with pytest.raises(ContractError, match="same contract object"):
        PiperReviewer(s.contract, s.kin, gate_module_path=upstream_gate(),
                      description=FakeDescription(other_contract))
