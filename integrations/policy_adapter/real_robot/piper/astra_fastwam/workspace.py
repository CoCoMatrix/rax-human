"""PiPER/FastWAM workspace for the original persistent-agent method."""
from pathlib import Path
import shutil
import sys

from hybrid_rollout.robodojo.io import write_json


def prepare_workspace(audit, skill_root, *, mode="shadow"):
    audit, skill_root = Path(audit), Path(skill_root)
    agent = audit / "agent"
    agent.mkdir()
    (agent / "scratch").mkdir()
    shutil.copytree(skill_root / "context", agent / "context")
    local = agent / ".agents/skills/astra-fastwam-piper"
    shutil.copytree(skill_root, local)
    write_json(agent / "workspace.json", dict(
        robot_profile="piper_x_dual", base_policy="FastWAM drink raw step82350",
        controller_output=str(audit.parent), python_executable=sys.executable,
        history_path=str(audit.parent / "history.json"),
        observations_path=str(audit.parent / "observations"),
        proposals_path=str(audit.parent / "proposals"),
        notes_path=str(agent / "NOTES.md"), scratch=str(agent / "scratch"),
        mode=mode, physical_execution_authorized=mode == "live"))
    instructions = (
        "# Astra + FastWAM PiPER shadow workspace\n\n"
        "Use the injected skill and original outcome/intent gate. You are the persistent "
        "policy agent with normal file/image/calculation tools. Read recorded observations "
        "and write analysis only inside this workspace. The three service tool names are "
        "upstream compatibility names; the robot is real PiPER and the student is FastWAM. "
        "Only one no-send shadow decision is authorized. Never connect to a robot SDK, "
        "CAN device, operator POST endpoint, second model client, or another control path. "
        "Do not reset/home/enable the robot or edit executing source. No task execution "
        "occurs in shadow, even if a candidate validates. Public explanations must be "
        "English; private reasoning is neither requested nor exported.\n")
    if mode == "live":
        instructions = instructions.replace("# Astra + FastWAM PiPER shadow workspace", "# Astra + FastWAM PiPER authorized episode workspace")
        instructions = instructions.replace("Only one no-send shadow decision is authorized.",
            "The host has explicitly authorized this run-id for the three formal control tools only.")
        instructions = instructions.replace("No task execution occurs in shadow, even if a candidate validates.",
            "Only the host service tools can submit reviewed actions to the sole formal XPolicy owner. Inspect returned real ACKs and new observations; no shell/network/SDK dispatch is permitted.")
    (agent / "AGENTS.md").write_text(instructions)
    (agent / "NOTES.md").write_text("# Working memory\n\nNo observations yet.\n")
    return agent
