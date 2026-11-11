"""Offline release→ordered ACK case; no server, model, camera or device starts."""
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest

import numpy as np

from hybrid_rollout.robodojo.io import write_json
from ..contracts import vector_schema
from . import CONDITIONING
from .live_io import LiveExecutor
from .relay import RelayModel


class FakeContract:
    def validate_observation(self, wire, **kwargs):
        return wire

    def validate_candidate(self, actions, **kwargs):
        assert actions.shape[1] == 14


class OfflineLiveTransportTest(unittest.TestCase):
    def test_release_to_ordered_ack_uses_no_device(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ("commands", "releases", "observations"):
                (root / name).mkdir()
            observed_ns = time.time_ns()
            observation = SimpleNamespace(instruction=CONDITIONING,
                timestamp_ns=observed_ns, sequence=1, state14=np.zeros(14, np.float32),
                metadata=dict(policy_session_id="offline-session", owner_generation=1))
            actions = np.zeros((15, 14), np.float32)
            actions[:, 0] = .1  # Legal profile prefix, deliberately beyond old canary.
            write_json(root / "commands/test-command.json", dict(command_id="test-command",
                run_id="offline-run", teacher_request_id="offline-teacher", kind="student",
                review_observation_timestamp_ns=observed_ns - 1, targets14=actions.tolist()))
            relay = RelayModel(FakeContract(), root, "offline-run", authorize_dispatch=True)
            result = asyncio.run(relay.infer(observation))
            self.assertEqual(result["action_schema"], vector_schema(action=True))
            release = result["policy_metadata"]["review_release"]
            self.assertEqual(release["schema"], "junqi.piper.policy_prefix_release.v1")
            self.assertEqual(release["kind"], "student")
            self.assertEqual(release["steps"], 15)
            self.assertEqual(release["generation"], 1)
            # Existing authoritative gate consumes the exact new release format.
            import sys
            sys.path.insert(0, "/home/ubuntu/code_junqi/xpolicylab-robot-runtime")
            from real_world.robots.piper_x_dual.review_release import BoundedReviewGate
            gate = BoundedReviewGate(profile="policy_prefix")
            for index, target in enumerate(actions):
                gate.reserve(release, generation=1, measured=observation.state14,
                             requested=target, now_ns=time.time_ns(), step_index=index)
            journal = root / "offline-journal.jsonl"
            submitted_ns = time.time_ns()
            rows = [dict(event="sdk_dispatch_completed", review_release_id=release["release_id"],
                teacher_request_id=release["teacher_request_id"], generation=1,
                model_chunk_step_index=index, recorded_ns=submitted_ns+index,
                hardware_dispatch_confirmed=True, applied14=actions[index].tolist()) for index in range(15)]
            journal.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
            reader = SimpleNamespace(minimum_timestamp_ns=0)
            executor = LiveExecutor(SimpleNamespace(stopped=lambda: None), journal, reader, None, None)
            ack = executor._wait_ack(release, submitted_ns)
            self.assertEqual(len(ack), 15)
            self.assertEqual(executor.total_acknowledged, 15)
            self.assertEqual(reader.minimum_timestamp_ns, submitted_ns+14)
            # A consumed command is never emitted twice.
            self.assertEqual(asyncio.run(relay.infer(observation))["result_kind"], "no_action")


if __name__ == "__main__":
    unittest.main()
