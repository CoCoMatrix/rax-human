"""Dispatch boundary. Only ShadowExecutor is implemented in this preparation.

Future hardware integration must implement submit_reviewed/release ACK/fresh
feedback using the existing formal XPolicy graph and its sole owner. No HTTP
action, SDK, graph-launch, home, or enable function exists here.
"""
from typing import Protocol


class FormalReviewedExecutor(Protocol):
    def submit_reviewed(self, *, observation, candidate, binding):
        """Publish review_release to formal PolicyServer; return after owner ACK.

        Required future bindings: exact owner generation and policy_session_id;
        fresh execution observation; ordered targets14; journal dispatch ACKs.
        Recompute each correction step after real ACK, never from planned state.
        """
        ...


class ShadowExecutor:
    def submit_reviewed(self, *, observation, candidate, binding):
        return dict(mode="shadow", result="candidate_validated_no_send",
                    selected_steps=0 if candidate is None else candidate.prefix_steps,
                    executed_steps=0, hardware_write_count=0,
                    hardware_dispatch_confirmed=False, physical_execution_authorized=False,
                    review_request_id=binding.request_id,
                    task_success=False, task_success_evaluated=False)
