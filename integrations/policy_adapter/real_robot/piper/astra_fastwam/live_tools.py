"""Authorized continuous hybrid loop, using the same native Codex scheduler."""
import time

import numpy as np

from hybrid_rollout.robodojo.io import write_json
from .tools import PiperHybridTools


class PiperLiveTools(PiperHybridTools):
    def __init__(self, *args, executor, max_decisions=180, **kwargs):
        super().__init__(*args, mode="live", **kwargs)
        self.executor, self.max_decisions = executor, max_decisions

    def execute(self, **arguments):
        directory = self._check("robodojo_execute", arguments)
        response, checked = self.validate_decision(arguments)
        decision = len(self.history)
        record_dir = self.output / "decisions" / f"{decision:03d}"
        record_dir.mkdir(parents=True)
        write_json(record_dir / "response.json", response)
        if response["mode"] == "stop":
            self.history.append(dict(decision=decision, response=response, executed_steps=0,
                                     physical_actions_executed=0, source="model_stop"))
            write_json(self.output / "history.json", self.history)
            result = self.finish("model_stop")
            return dict(rollout_finished=True, result=result, next_call=None,
                        images=self.observation_packet["images"])
        self.status("executing_reviewed_prefix", decision_mode=response["mode"], selected_steps=response["steps"])
        start_tick = self.tick
        try:
            receipt = self.executor.execute_decision(observation=self.obs, proposal=self.proposal,
                                                     response=response, checked=checked)
        except BaseException:
            self.tick = self.executor.total_acknowledged
            self.executor.stop("controller_or_ACK_error_no_retry")
            self.status("execution_error", hardware_execution_uncertain=True)
            raise
        self.tick += receipt["executed_steps"]
        key = dict(student="student_steps", edit="edited_steps", eef="recovery_steps")[response["mode"]]
        self.counters[key] += receipt["executed_steps"]
        actual = [ack for release in receipt["releases"] for ack in release["ack"]]
        np.savez_compressed(record_dir / "execution.npz",
            applied_actions=np.asarray([ack["applied14"] for ack in actual], dtype=np.float32))
        write_json(record_dir / "owner-ACK.json", receipt)
        record = dict(decision=decision, start_tick=start_tick, end_tick=self.tick,
            response=response, assessment_gate=checked.assessment_gate,
            executed_steps=receipt["executed_steps"], source=response["mode"],
            physical_actions_executed=receipt["executed_steps"],
            owner_receipt_path=str(record_dir / "owner-ACK.json"),
            applied_targets14=[ack["applied14"] for ack in actual],
            pose_arrival_confirmed=False, object_outcome_confirmed=False)
        self.history.append(record)
        write_json(self.output / "history.json", self.history)
        self.event("formal_owner_ACK", decision=response, receipt=record)
        packet = self.capture(directory)
        if self.max_decisions and len(self.history) >= self.max_decisions:
            packet.update(rollout_finished=True, result=self.finish("decision_budget"), next_call=None)
            write_json(self.observation_path, packet)
        return packet

    def finish(self, reason):
        self.phase = "done"
        self.executor.stop(reason)
        result = dict(mode="live", reason=reason, complete=False,
            physical_execution_authorized=True, executed_steps=self.tick,
            hardware_dispatch_confirmed=self.tick > 0, task_success=None,
            task_success_evaluated=False, terminal_evaluation="operator_and_visual_review_required",
            decisions=len(self.history), wall_seconds=time.monotonic()-self.started,
            history_path=str(self.output / "history.json"))
        write_json(self.output / "result.json", result)
        self.status("stopped", result=result)
        return result

    def close(self):
        self.executor.stop("agent_closed")
