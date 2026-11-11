"""Persistent Astra + FastWAM / PiPER; shadow default, explicit authorized live mode."""
import argparse
import os
from pathlib import Path
import json
import shutil
import signal

from hybrid_rollout.robodojo.io import write_json
from ..contracts import PiperContract, require
from ..kinematics import PiperKinematics
from ..review import PiperReviewer
from .agent import PiperCodexPolicy
from .observation import ObservationReader
from .student import FastWAMStudent
from .tools import PiperHybridTools
from . import STUDENT_MAX


def parser():
    root = Path("/home/ubuntu/code_junqi")
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--owner-url", default="http://127.0.0.1:8091")
    p.add_argument("--student-url", default="ws://127.0.0.1:19023")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--task", default="把棕色瓶子放到杯垫上")
    p.add_argument("--mode", choices=["shadow", "live"], default="shadow")
    p.add_argument("--authorize-dispatch", action="store_true")
    p.add_argument("--run-id")
    p.add_argument("--exchange-dir", type=Path)
    p.add_argument("--journal", type=Path)
    p.add_argument("--max-decisions", type=int, default=180)
    p.add_argument("--observation-dir", type=Path,
                   default=root / "migration/astra-fastwam/observation")
    p.add_argument("--runtime", type=Path, default=root / "xpolicylab-robot-runtime")
    p.add_argument("--student-config", type=Path, default=root /
                   "xpolicylab-robot-runtime/policy/FastWAM/deploy_piper_pick_and_place_drink_step82350.yml")
    p.add_argument("--codex", default=shutil.which("codex") or "codex")
    p.add_argument("--codex-home", type=Path, default=root / ".codex-gpt-policy")
    p.add_argument("--model-provider", help="Optional existing configured provider; default personal Codex auth")
    p.add_argument("--timeout", type=int, default=600)
    p.add_argument("--snapshot-max-age-seconds", type=float, default=1800)
    return p


def main(argv=None):
    a = parser().parse_args(argv)
    if a.mode == "live":
        require(a.authorize_dispatch and a.run_id and a.exchange_dir and a.journal,
                "live requires explicit --authorize-dispatch, --run-id, --exchange-dir and --journal")
        from .live_io import validate_run_id
        validate_run_id(a.run_id)
    a.output = a.output.resolve()
    require(a.output.is_relative_to(Path("/home/ubuntu/code_junqi")), "output must be in personal workspace")
    require(a.codex_home.is_dir(), "selected existing Codex authentication home is unavailable")
    os.environ["CODEX_HOME"] = str(a.codex_home)
    a.output.mkdir(parents=True, exist_ok=False)
    write_json(a.output / "run.json", dict(mode=a.mode, task=a.task,
        owner_url=a.owner_url, student_url=a.student_url, student_config=str(a.student_config),
        observation_dir=str(a.observation_dir), model="gpt-6-astra", effort="xhigh",
        base_policy="FastWAM drink raw step82350", physical_execution_authorized=a.mode == "live",
        run_id=a.run_id, exchange_dir=str(a.exchange_dir) if a.exchange_dir else None,
        owner_receipt_journal=str(a.journal) if a.journal else None,
        compatibility_tool_names=["robodojo_start", "pi05_infer", "robodojo_execute"]))
    student = worker = rollout = None

    def stop(signum, frame):
        raise KeyboardInterrupt("Stopping this owned no-send shadow agent")
    signal.signal(signal.SIGTERM, stop)
    try:
        contract = PiperContract.from_runtime(a.runtime, action_prefix=STUDENT_MAX)
        kinematics = PiperKinematics(contract)
        gap = Path(__file__).resolve().parents[3]
        reviewer = PiperReviewer(contract, kinematics,
            gate_module_path=gap / "hybrid_rollout/robodojo/robodojo_server/gate_assessment.py")
        if a.mode == "live":
            from .live_io import RelayClient, LiveObservationReader, LiveExecutor
            from .live_tools import PiperLiveTools
            relay = RelayClient(a.exchange_dir, a.run_id)
            reader = LiveObservationReader(a.owner_url, relay, contract)
        else:
            reader = ObservationReader(a.owner_url, a.observation_dir, contract,
                                       max_age_s=a.snapshot_max_age_seconds)
        # Fail on absent capture/owner before starting a paid model turn.
        from . import CONDITIONING
        observation, provenance = reader.read(CONDITIONING)
        run_path = a.output / "run.json"
        run = json.loads(run_path.read_text())
        run.update(historical_replay=a.mode == "shadow", capture_timestamp_ns=observation.timestamp_ns,
                   capture_age_seconds=provenance["capture_age_s"],
                   observation_provenance=provenance["source_metadata"].get("provenance", {}),
                   snapshot_max_age_seconds=a.snapshot_max_age_seconds)
        write_json(run_path, run)
        student = FastWAMStudent(a.runtime, a.student_url, a.student_config)
        write_json(a.output / "student-hello.json", student.connect())
        if a.mode == "live":
            executor = LiveExecutor(relay, a.journal, reader, contract, reviewer)
            rollout = PiperLiveTools(a.output, a.task, student, reader, contract, kinematics, reviewer,
                                    executor=executor, max_decisions=a.max_decisions)
        else:
            rollout = PiperHybridTools(a.output, a.task, student, reader, contract, kinematics, reviewer)
        worker = PiperCodexPolicy(a.output / "codex_workspace", a.codex,
                                 timeout=a.timeout, model_provider=a.model_provider, mode=a.mode)
        worker.run(rollout)
    except BaseException as exc:
        write_json(a.output / "failure.json", dict(error_type=type(exc).__name__,
            error=str(exc), mode=a.mode, acknowledged_actions=rollout.tick if rollout else 0,
            hardware_execution_uncertain=a.mode == "live"))
        if rollout is not None and rollout.phase != "done":
            rollout.finish("controller_error")
        else:
            write_json(a.output / "status.json", dict(status="error", error=str(exc),
                mode=a.mode, executed_steps=0, physical_execution_authorized=a.mode == "live"))
        raise
    finally:
        if worker is not None:
            worker.close()
        if rollout is not None:
            rollout.close()
        if student is not None:
            student.close()


if __name__ == "__main__":
    main()
