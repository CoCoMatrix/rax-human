"""Use upstream CodexPolicy.run, with PiPER tools and a dedicated skill.

No Responses reviewer is used. Constructor differs to select the user's existing
personal Codex authentication and the PiPER robot/action contracts. The original
persistent transport/event loop and recoverable tool-input handling are inherited.
"""
from pathlib import Path
import os
import hashlib
import json
import subprocess
import time

from hybrid_rollout.robodojo.io import write_json
from hybrid_rollout.robodojo.skill.run import CodexPolicy, toml_value
from hybrid_rollout.robodojo.skill.transport import StdioAppServer
from . import MODEL, EFFORT
from .schema import tool_specs
from .workspace import prepare_workspace


class PublicEventTransport(StdioAppServer):
    def next_message(self, timeout):
        event = super().next_message(timeout)
        params = event.get("params", {})
        item = params.get("item", {})
        if event.get("method") == "item/completed" and item.get("type") == "agentMessage":
            # Only the explicit public assistant message; never reasoning items,
            # analysis, raw RPC records or command outputs are projected to UI.
            record = dict(event="public_assistant", time_ns=time.time_ns(),
                          text=item.get("text", ""), mode=getattr(self, "mode", "shadow"))
            with (self.workspace.parent / "events.jsonl").open("a") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        return event


class PiperCodexPolicy(CodexPolicy):
    def __init__(self, workspace, codex, *, timeout=600, model_provider=None, mode="shadow",
                 transport_factory=PublicEventTransport):
        self.method = "pi05_plus_gpt"  # Upstream event-loop route, student is FastWAM.
        self.mode = mode
        self.image_max_edge = 960
        self.timeout = timeout
        self.workspace = Path(workspace)
        self.workspace.mkdir(parents=True, exist_ok=False)
        skill_root = Path(__file__).parent / ("skill_live" if mode == "live" else "skill")
        self.agent_workspace = prepare_workspace(self.workspace, skill_root, mode=mode)
        self.prompt = "\n\n".join((skill_root / name).read_text() for name in (
            "SKILL.md", "gate_prompt.md", "context/teacher_context.md", "context/eef_control.md"))
        if os.environ.get("JUNQI_FASTWAM_FULL_HORIZON") == "1":
            self.prompt += "\n\nTEMPORARY FULL-HORIZON MODE: FastWAM provides H32 at 30 Hz. Review the complete 32-step candidate, focus your assessment on the largest flange displacement, gripper opening/closing transitions, and approach/lift/transfer phase changes. If aligned, execute the original 32-step student chunk; do not reduce it to the first 15 steps. Reobserve after this full chunk."
        if os.environ.get("JUNQI_EXECUTE_ARM") == "right_arm":
            self.prompt += "\n\nRIGHT-ARM-ONLY MODE: The user explicitly requires the right arm to perform the bottle task. Keep the left arm at its measured state. Reject a student prefix whose meaningful motion is left-arm-only; use a right-arm eef correction (1-5 targets) from fresh measured feedback, or stop if a justified right-arm target cannot be formed."
        self.prompt += "\n\nAgent workspace: " + str(self.agent_workspace)
        self.prompt_sha256 = hashlib.sha256(self.prompt.encode()).hexdigest()
        (self.workspace / "PROMPT.md").write_text(self.prompt)
        specs = tool_specs(mode)
        write_json(self.workspace / "tools.json", specs)
        config = dict(model=MODEL, model_reasoning_effort=EFFORT,
                      **{"features.shell_tool": True, "features.view_image": True,
                         "shell_environment_policy.inherit": "all",
                         "shell_environment_policy.exclude": ["JUNQI_ASTRA_API_KEY", "OPENAI_API_KEY"],
                         "default_permissions": "rollout_agent",
                         "permissions.rollout_agent.extends": ":workspace",
                         "permissions.rollout_agent.filesystem": {
                             str(self.workspace.parent): "read", str(self.agent_workspace): "write"}})
        if model_provider:
            config["model_provider"] = model_provider
        version = subprocess.run([codex, "--version"], capture_output=True,
                                 text=True, check=True).stdout.strip()
        argv = [codex, "app-server", "--stdio", "--strict-config"]
        for key, value in config.items():
            argv += ["-c", key + "=" + toml_value(value)]
        write_json(self.workspace / "launch.json", dict(argv=argv, config=config))
        self.transport = transport_factory(argv, self.workspace)
        self.transport.mode = mode
        try:
            self.transport.request("initialize", dict(
                clientInfo=dict(name="astra-fastwam-piper-shadow", version="1"),
                capabilities=dict(experimentalApi=True)), timeout)
            self.transport.notify("initialized", {})
            params = dict(cwd=str(self.agent_workspace), model=MODEL,
                          config=dict(model_reasoning_effort=EFFORT),
                          developerInstructions=self.prompt, dynamicTools=specs,
                          ephemeral=False, allowProviderModelFallback=False,
                          approvalPolicy="never", approvalsReviewer="auto_review",
                          permissions="rollout_agent",
                          runtimeWorkspaceRoots=[str(self.agent_workspace)])
            if model_provider:
                params["modelProvider"] = model_provider
            result = self.transport.request("thread/start", params, timeout)
            if result.get("model") != MODEL or result.get("reasoningEffort") != EFFORT:
                raise RuntimeError("Codex did not select gpt-6-astra / xhigh")
            if result.get("sandbox", {}).get("networkAccess") is not False:
                raise RuntimeError("Native analysis tools must have sandbox network disabled")
            self.thread_id = result["thread"]["id"]
            write_json(self.workspace / "worker.json", dict(
                model=MODEL, reasoning_effort=EFFORT, thread_id=self.thread_id,
                model_provider=result.get("modelProvider", model_provider),
                codex_version=version, prompt_sha256=self.prompt_sha256,
                base_policy="FastWAM", robot_profile="piper_x_dual", mode=mode,
                persistent_agent=True, upstream_event_loop="CodexPolicy.run",
                physical_execution_authorized=mode == "live", service_tools_are_additive=True))
            write_json(self.workspace / "sandbox.json", dict(
                sandbox=result.get("sandbox"),
                activePermissionProfile=result.get("activePermissionProfile"),
                runtimeWorkspaceRoots=result.get("runtimeWorkspaceRoots")))
        except BaseException:
            self.close()
            raise

    def _turn(self, text):
        # The inherited event loop's initial simulation wording is replaced by
        # the actual hardware adaptation and this run's no-send scope.
        text = text.replace("this single simulation rollout", "this single authorized real PiPER rollout" if self.mode == "live" else "this single real-observation no-send shadow")
        return super()._turn(text)
