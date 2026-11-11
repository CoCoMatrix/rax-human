"""Focused no-network checks for the new shadow boundary and agent integration."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from .agent import PiperCodexPolicy
from .dispatch import ShadowExecutor
from .schema import tool_specs
from .__main__ import parser
from hybrid_rollout.robodojo.skill.run import CodexPolicy


class FakeTransport:
    def __init__(self, argv, workspace):
        self.argv, self.requests = argv, []
        self.closed = False

    def request(self, method, params, timeout):
        self.requests.append((method, params))
        if method == "thread/start":
            return dict(model="gpt-6-astra", reasoningEffort="xhigh", thread=dict(id="fake-thread"),
                        sandbox=dict(networkAccess=False))
        if method == "turn/start":
            return dict(turn=dict(id="fake-turn"))
        return {}

    def notify(self, *args):
        pass

    def close(self):
        self.closed = True


class ShadowBoundaryTests(unittest.TestCase):
    def test_cli_defaults_to_shadow_and_never_implicitly_authorizes(self):
        option = next(action for action in parser()._actions if action.dest == "mode")
        self.assertEqual(option.default, "shadow")
        args = parser().parse_args(["--output", "/tmp/test"])
        self.assertFalse(args.authorize_dispatch)

    def test_schema_is_piper_width_not_arx_boolean(self):
        tools = tool_specs()
        response = tools[2]["inputSchema"]["properties"]["response"]
        self.assertEqual(response["properties"]["steps"]["anyOf"][0]["maximum"], 15)
        target = response["properties"]["target"]["anyOf"][0]["properties"]["left_arm"]
        self.assertIn("width_m", json.dumps(target))
        self.assertNotIn("gripper_closed", json.dumps(target))
        self.assertEqual(response["properties"]["binding"]["properties"]["observation_timestamp_ns"]["type"], "string")

    def test_shadow_executor_cannot_submit_motion(self):
        result = ShadowExecutor().submit_reviewed(observation=None,
            candidate=SimpleNamespace(prefix_steps=15), binding=SimpleNamespace(request_id="test"))
        self.assertEqual(result["selected_steps"], 15)
        self.assertEqual(result["executed_steps"], 0)
        self.assertFalse(result["physical_execution_authorized"])
        self.assertFalse(result["task_success"])

    def test_native_agent_inherits_original_loop(self):
        self.assertIs(PiperCodexPolicy.run, CodexPolicy.run)
        with tempfile.TemporaryDirectory() as tmp:
            with patch("subprocess.run", return_value=SimpleNamespace(stdout="codex test")):
                agent = PiperCodexPolicy(Path(tmp) / "audit", "codex", transport_factory=FakeTransport)
            try:
                params = next(value for name, value in agent.transport.requests if name == "thread/start")
                self.assertEqual(params["model"], "gpt-6-astra")
                self.assertEqual(params["config"]["model_reasoning_effort"], "xhigh")
                self.assertEqual(len(params["dynamicTools"]), 3)
                self.assertIn("FastWAM", params["developerInstructions"])
                self.assertIn("no-send", params["developerInstructions"])
                agent._turn("Act for this single simulation rollout")
                text = agent.transport.requests[-1][1]["input"][0]["text"]
                self.assertIn("real-observation no-send shadow", text)
            finally:
                agent.close()


if __name__ == "__main__":
    unittest.main()
