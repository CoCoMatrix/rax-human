from . import STUDENT_MAX
"""File-backed blocking tools for one real-observation, no-send hybrid decision."""
from pathlib import Path
from dataclasses import asdict
import json
import time
import uuid

import numpy as np
from PIL import Image

from hybrid_rollout.robodojo.io import InputError, write_json
from ..backend import array_sha256, pose_json
from ..contracts import CandidateEnvelope, ContractError, require, vector_schema, runtime_module, FIELDS
from ..review import make_binding, ReviewEnvelope
from ..teacher_contract import normalize_response
from . import CONDITIONING
from .dispatch import ShadowExecutor


class PiperHybridTools:
    profile = "piper_x_dual"

    def __init__(self, output, task, student, reader, contract, kinematics, reviewer, *, mode="shadow"):
        self.output, self.task = Path(output), task
        self.mode = mode
        self.student, self.reader = student, reader
        self.contract, self.kinematics, self.reviewer = contract, kinematics, reviewer
        self.executor = ShadowExecutor()
        self.phase, self.tick, self.episode = "start", 0, None
        self.history = []
        self.counters = dict(student_steps=0, edited_steps=0, recovery_steps=0, predictions=0)
        self.request = None
        self.started = time.monotonic()
        self.status("waiting_for_agent")

    def event(self, event, **details):
        value = dict(event=event, time_ns=time.time_ns(), phase=self.phase,
                     step_id=self.tick, mode=self.mode, **details)
        with (self.output / "events.jsonl").open("a") as stream:
            stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
        print(json.dumps(value, ensure_ascii=False), flush=True)

    def status(self, status, **extra):
        write_json(self.output / "status.json", dict(status=status, phase=self.phase,
            mode=self.mode, user_goal=self.task, student_conditioning=CONDITIONING,
            base_policy="FastWAM", step_id=self.tick, executed_steps=self.tick,
            physical_execution_authorized=self.mode == "live", hardware_dispatch_confirmed=self.tick > 0,
            counters=self.counters, updated_ns=time.time_ns(), **extra))

    def next_call(self):
        if self.phase == "start":
            return dict(tool="robodojo_start", task=self.task,
                        output_dir=str(self.output / "observations/000"))
        if self.phase == "infer":
            return dict(tool="pi05_infer", observation_path=str(self.observation_path),
                        output_dir=str(self.output / "proposals" / f"{self.counters['predictions']:03d}"))
        if self.phase == "execute":
            return dict(tool="robodojo_execute", proposal_path=str(self.proposal_path),
                        output_dir=str(self.output / "observations" / f"{len(self.history)+1:03d}"),
                        response=("Supply the unchanged outcome/intent gate assessment and PiPER-native response; this only validates, never moves."
                                  if self.mode == "shadow" else "Supply the unchanged gate and a reviewed student/edit/eef/stop decision for this authorized live run."))
        return None

    def _check(self, tool, arguments):
        expected = self.next_call()
        if not expected or expected["tool"] != tool:
            raise InputError("Wrong order; next_call=" + json.dumps(expected))
        for key, value in expected.items():
            if key not in ("tool", "response") and arguments.get(key) != value:
                raise InputError(key + " must equal " + repr(value))
        path = Path(arguments["output_dir"])
        if path.exists() or not path.resolve().is_relative_to(self.output):
            raise InputError("Output directory must be fresh under the rollout output")
        return path

    def start(self, **arguments):
        directory = self._check("robodojo_start", arguments)
        self.episode = uuid.uuid4().hex
        return self.capture(directory)

    def capture(self, directory):
        self.status("reading_observation")
        self.obs, source = self.reader.read(CONDITIONING)
        directory.mkdir(parents=True)
        np.savez_compressed(directory / "observation.npz", state14=self.obs.state14, **self.obs.images)
        images = []
        for role, pixels in self.obs.images.items():
            path = directory / (role + ".png")
            Image.fromarray(pixels).save(path)
            images.append(dict(camera=role, path=str(path)))
        self.observation_path = directory / "observation.json"
        self.phase = "infer"
        packet = dict(episode_id=self.episode, step_id=self.tick, task=self.task,
            instruction=CONDITIONING, user_goal=self.task, images=images,
            current_state=self.obs.state14.tolist(),
            current_eef=pose_json(self.kinematics.forward(self.obs.state14)),
            observation_path=str(self.observation_path), arrays_path=str(directory / "observation.npz"),
            history_path=str(self.output / "history.json"), previous_result=self.history[-1] if self.history else None,
            robot_profile=self.profile, source=source, mode=self.mode,
            physical_execution_authorized=self.mode == "live",
            executed_steps=self.tick, rollout_finished=False, next_call=self.next_call())
        self.observation_packet = packet
        write_json(self.observation_path, packet)
        write_json(self.output / "history.json", self.history)
        self.status("observation_ready")
        self.event("observation", path=str(self.observation_path), source="lossless_formal_capture")
        return packet

    def infer(self, **arguments):
        directory = self._check("pi05_infer", arguments)
        self.reader.owner()
        self.status("fastwam_inference")
        started = time.monotonic()
        actions, payload = self.student.infer(self.obs)
        directory.mkdir(parents=True)
        self.proposal_path = directory / "actions.npz"
        # These are the same explicit boundary normalizations performed by the
        # formal owner; raw model output is preserved separately and never hidden.
        safety = runtime_module(self.contract.repo_root, "real_world.robots.piper_x_dual.safety")
        mapping = runtime_module(self.contract.repo_root, "real_world.robots.piper_x_dual.mapping")
        boundary = self.contract.profile["live_takeover"]["model_boundary_normalization"]
        effective, adjustments = [], []
        for index, row in enumerate(actions):
            target = mapping.action_dict(row)
            changes = ()
            if boundary["enabled"]:
                target, changes = safety.normalize_model_action_boundaries(target,
                    joint_limits_rad=self.contract.profile["body"]["joint_limits_rad"],
                    gripper_limits_m=tuple(self.contract.profile["body"]["grippers"]["left_gripper"]["limits"]),
                    joint_tolerance_rad=float(boundary["joint_tolerance_rad"]))
            effective.append([target[name] for name in FIELDS])
            adjustments.extend(dict(step=index, **change.as_dict()) for change in changes)
        effective = np.asarray(effective, np.float64)
        np.savez_compressed(self.proposal_path, raw_actions=actions, owner_boundary_preview=effective)
        # The method can select only the leading 1..{STUDENT_MAX} controls. Keep and expose
        # all H32 predictions, but validate the executable prefix rather than
        # letting an unused suffix veto a legal reviewed prefix.
        suffix_violations = [dict(step=int(i), field=FIELDS[j], value=float(effective[i, j]),
            lower=float(self.contract.lower14[j]), upper=float(self.contract.upper14[j]))
            for i, j in np.argwhere((effective < self.contract.lower14) | (effective > self.contract.upper14))]
        self.proposal = self.contract.validate_candidate(effective[:STUDENT_MAX], schema=vector_schema(action=True),
            frequency_hz=30, measured_state14=self.obs.state14, prefix_steps=STUDENT_MAX)
        self.binding = make_binding(uuid.uuid4().hex, self.obs, self.proposal, step_id=self.tick)
        binding = self.binding.as_dict()
        binding["observation_timestamp_ns"] = str(binding["observation_timestamp_ns"])
        self.counters["predictions"] += 1
        trajectory = []
        for index, row in enumerate(effective):
            try:
                trajectory.append(dict(index=index, **pose_json(self.kinematics.forward(row))))
            except ContractError as exc:
                trajectory.append(dict(index=index, fk_error=str(exc)))
        self.request = dict(request_id=self.binding.request_id, binding=binding, step_id=self.tick,
            user_goal=self.task, task=CONDITIONING, instruction=CONDITIONING,
            images=self.observation_packet["images"], current_state=self.obs.state14.tolist(),
            current_eef=self.observation_packet["current_eef"],
            student_eef_trajectory=trajectory, student_actions=actions.tolist(),
            action_diagnostics=dict(self.proposal.metrics), raw_sha256=array_sha256(actions),
            proposal_binding_scope="owner-boundary preview of selectable first 15 steps; full raw H32 hash also recorded",
            full_horizon_limit_violations=suffix_violations,
            owner_boundary_adjustments=adjustments, base_policy="FastWAM", action_horizon=32,
            action_frequency_hz=30, student_identity=self.student.metadata,
            actual_model_latency_ms=(time.monotonic() - started) * 1000,
            correction_frame="dual_base_link", gripper_semantics="absolute_width_metres",
            previous_result=self.history[-1] if self.history else None, executed_steps=self.tick, mode=self.mode, no_depth_input=True,
            physical_execution_authorized=self.mode == "live", proposal_path=str(self.proposal_path),
            request_path=str(directory / "proposal.json"), rollout_finished=False)
        self.phase = "execute"
        self.request["next_call"] = self.next_call()
        write_json(directory / "proposal.json", self.request)
        self.status("awaiting_gpt_decision")
        self.event("fastwam_proposal", path=str(directory / "proposal.json"), horizon=32)
        return self.request

    def execute(self, **arguments):
        directory = self._check("robodojo_execute", arguments)
        response, checked = self.validate_decision(arguments)
        return self.execute_shadow(directory, response, checked)

    def validate_decision(self, arguments):
        try:
            response = dict(arguments["response"])
            binding = dict(response.get("binding", {}))
            stamp = binding.get("observation_timestamp_ns")
            require(isinstance(stamp, str) and stamp == str(self.binding.observation_timestamp_ns),
                    "echo observation_timestamp_ns exactly as the supplied decimal string")
            binding["observation_timestamp_ns"] = int(stamp)
            response["binding"] = binding
            response = normalize_response(response, self.binding.as_dict())
            if response["mode"] == "student":
                require(1 <= response["steps"] <= STUDENT_MAX, f"student prefix must be 1..{STUDENT_MAX}")
            # This is an intentionally frozen snapshot review, never an execution
            # freshness assertion; a future live release must bind new feedback.
            now_ns = time.time_ns()
            frozen_review_age_ms = max(1., (now_ns - self.obs.timestamp_ns) / 1e6 + 1000.)
            checked = self.reviewer.review(self.obs, self.proposal, response,
                envelope=ReviewEnvelope(self.binding, now_ns, frozen_review_age_ms, frozenset()),
                candidate_envelope=CandidateEnvelope())
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            details = getattr(exc, "details", {})
            self.event("validation_rejected", error=str(exc), details=details, no_execution=True)
            self.status("gpt_revision_needed", last_error=str(exc), details=details)
            raise InputError(json.dumps(dict(error=str(exc), details=details, no_execution=True,
                correction="Revise this decision using the same fresh proposal binding; do not reset or fabricate execution."))) from exc
        return response, checked

    def execute_shadow(self, directory, response, checked):
        self.reader.owner()
        directory.mkdir(parents=True)
        receipt = self.executor.submit_reviewed(observation=self.obs,
            candidate=checked.candidate, binding=checked.binding)
        write_json(directory / "decision.json", response)
        write_json(directory / "shadow-receipt.json", receipt)
        if checked.candidate is not None:
            np.savez_compressed(directory / "validated_candidate.npz", actions=checked.candidate.actions)
        record = dict(decision=0, response=response, assessment_gate=checked.assessment_gate,
                      receipt=receipt, physical_actions_executed=0,
                      correction_is_offline_preview=response["mode"] in ("edit", "eef"))
        self.history.append(record)
        write_json(self.output / "history.json", self.history)
        self.event("execute_shadow", decision=response, receipt=receipt)
        result = self.finish("one_decision_shadow_complete")
        return dict(rollout_finished=True, mode="shadow", result=result, executed_steps=0,
                    task_success=False, images=self.observation_packet["images"],
                    next_call=None, note="Validation only. No object or robot motion was commanded.")

    def finish(self, reason):
        self.phase = "done"
        result = dict(reason=reason, mode="shadow", complete=reason == "one_decision_shadow_complete",
            shadow_completed=reason == "one_decision_shadow_complete", task_success=False,
            task_success_evaluated=False, executed_steps=0, physical_execution_authorized=False,
            hardware_dispatch_confirmed=False, decisions=len(self.history),
            wall_seconds=time.monotonic()-self.started, history_path=str(self.output / "history.json"))
        write_json(self.output / "result.json", result)
        self.status("shadow_complete" if result["shadow_completed"] else "stopped", result=result)
        return result

    def close(self):
        pass
