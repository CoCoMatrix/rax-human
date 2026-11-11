"""Thin formal XPolicy PolicyServer relay. It never opens a device or SDK.

Default is observe/no_action. Live releases require this process and the native
agent to receive the same explicit run-id and authorization flag, as well as the
original owner's explicit policy_prefix review profile.
"""
import argparse
import asyncio
from pathlib import Path
import json
import signal
import sys
import time

import numpy as np
import yaml

from hybrid_rollout.robodojo.io import write_json
from ..contracts import PiperContract, require, vector_schema, _thaw
from ..hybrid_shadow import no_action_payload
from . import STUDENT_MAX, CONDITIONING
from .live_io import validate_run_id


class RelayModel:
    def __init__(self, contract, directory, run_id, *, authorize_dispatch=False):
        self.contract, self.directory = contract, Path(directory)
        self.run_id, self.authorize = validate_run_id(run_id), bool(authorize_dispatch)
        self.session = self.generation = None
        self.consumed = set()
        self.status = "observe_only" if not self.authorize else "review_wait"

    def hold(self):
        payload = no_action_payload(request_id=self.run_id, status=self.status)
        payload["policy_metadata"]["transport_receipt"]["mode"] = "continuous_review_wait"
        return payload

    async def reset(self, **kwargs):
        return dict(physical_reset=False, does_not_rearm=True)

    def get_capabilities(self):
        return dict(robot_type="piper_x_dual", mode="astra_fastwam_formal_relay",
                    review_profile="policy_prefix", student_prefix_max=STUDENT_MAX,
                    correction_steps_max=5, sole_writer="formal_xpolicy_owner",
                    authorize_dispatch=self.authorize)

    def _capture(self, obs, request):
        identifier = request["request_id"]
        folder = self.directory / "observations" / identifier
        folder.mkdir()
        np.savez(folder / "observation.npz", state14=obs.state14, **obs.images)
        metadata = dict(request_id=identifier, run_id=self.run_id,
            timestamp_ns=obs.timestamp_ns, sequence=obs.sequence,
            instruction=obs.instruction, metadata=_thaw(obs.metadata),
            directory=str(folder.relative_to(self.directory)),
            provenance=dict(mode="live_formal_observation", timestamp_unchanged=True),
            hardware_commands_sent_by_capture=0)
        write_json(folder / "metadata.json", metadata)
        write_json(self.directory / "observations" / (identifier + ".json"), metadata)

    async def infer(self, wire):
        if (self.directory / "stop.json").exists():
            self.status = "stopped"
            return self.hold()
        try:
            obs = self.contract.validate_observation(wire, max_age_ms=300)
            require(obs.instruction == CONDITIONING, "formal graph task must match FastWAM conditioning")
            metadata = _thaw(obs.metadata)
            session, generation = metadata["policy_session_id"], metadata["owner_generation"]
            if self.session is None:
                self.session, self.generation = session, generation
            require((session, generation) == (self.session, self.generation),
                    "owner generation/session changed; this run is not rearmed")
            request_path = self.directory / "observation-request.json"
            if request_path.exists():
                request = json.loads(request_path.read_text())
                require(request.get("run_id") == self.run_id, "foreign observation run-id")
                validate_run_id(request.get("request_id"))
                reply = self.directory / "observations" / (request["request_id"] + ".json")
                if not reply.exists() and obs.timestamp_ns > request.get("minimum_timestamp_ns", 0):
                    await asyncio.to_thread(self._capture, obs, request)
                    # Capturing multi-megabyte originals must never consume the
                    # freshness budget of a release; wait for the next wire frame.
                    return self.hold()
            if not self.authorize:
                return self.hold()
            for path in sorted((self.directory / "commands").glob("*.json")):
                if path.stem in self.consumed:
                    continue
                self.consumed.add(path.stem)  # Reserve before return or any possible dispatch.
                command = json.loads(path.read_text())
                response_path = self.directory / "releases" / path.name
                try:
                    require(command.get("run_id") == self.run_id and command.get("command_id") == path.stem,
                            "foreign relay command identity")
                    kind = command.get("kind")
                    require(kind in ("student", "edit", "eef"), "unknown command kind")
                    actions = np.asarray(command["targets14"], dtype=np.float32)
                    maximum = 15 if kind == "student" else 5
                    require(1 <= len(actions) <= maximum, "command exceeds original method prefix length")
                    self.contract.validate_candidate(actions, schema=vector_schema(action=True),
                        frequency_hz=30, measured_state14=obs.state14, prefix_steps=len(actions))
                    now_ns = time.time_ns()
                    require(0 <= now_ns - obs.timestamp_ns <= 250_000_000,
                            "execution observation expired before relay return")
                    release = dict(schema="junqi.piper.policy_prefix_release.v1", kind=kind,
                        release_id=path.stem, teacher_request_id=command["teacher_request_id"],
                        execution_session_id=session, generation=generation, steps=len(actions),
                        review_observation_timestamp_ns=command["review_observation_timestamp_ns"],
                        execution_observation_timestamp_ns=obs.timestamp_ns,
                        expires_ns=obs.timestamp_ns + 300_000_000 + int((len(actions)-1)*1e9/30),
                        anchor14=obs.state14.tolist(), targets14=actions.tolist(),
                        scene_check="fresh_three_cameras_and_state_match_image_comparison_disabled_by_operator")
                    write_json(response_path, dict(status="released_to_formal_transport",
                        release=release, hardware_dispatch_confirmed=False))
                    return dict(actions=actions, action_schema=vector_schema(action=True),
                        action_frequency_hz=30, policy_metadata=dict(
                            execute_prefix_steps=len(actions), review_release=release,
                            hardware_execution_confirmed=False, adapter="astra_fastwam_native_agent"))
                except Exception as exc:
                    write_json(response_path, dict(status="rejected", error=str(exc),
                        no_execution=True, command_id=path.stem))
                    self.status = "command_rejected"
                    return self.hold()
            return self.hold()
        except Exception as exc:
            self.status = "relay_error"
            write_json(self.directory / "stop.json", dict(run_id=self.run_id,
                reason=str(exc), timestamp_ns=time.time_ns(), no_sdk_command=True))
            return self.hold()


async def run(a):
    require(a.directory.resolve().is_relative_to(Path("/home/ubuntu/code_junqi/migration")),
            "relay exchange must be under personal migration runs")
    a.directory.mkdir(parents=True, exist_ok=False)
    for name in ("commands", "releases", "observations"):
        (a.directory / name).mkdir()
    write_json(a.directory / "relay.json", dict(run_id=validate_run_id(a.run_id),
        authorize_dispatch=a.authorize_dispatch, review_profile="policy_prefix",
        port=a.port, no_sdk_writer=True, started_ns=time.time_ns()))
    sys.path.insert(0, str(a.runtime))
    from client_server.ws.model_server import PolicyServer, PolicyServerConfig, deployment_identity_from_config
    config = yaml.safe_load(a.student_config.read_text())
    descriptor = yaml.safe_load((a.runtime / "policy/Pi_05/deployments/piper-pick-and-place-drink-fastwam-step82350.yml").read_text())
    config["bundle_root"] = config.get("bundle_root") or descriptor["bundle"]["default_path"]
    model = RelayModel(PiperContract.from_runtime(a.runtime, action_prefix=STUDENT_MAX),
                       a.directory, a.run_id, authorize_dispatch=a.authorize_dispatch)
    server = PolicyServer(model, PolicyServerConfig(host="127.0.0.1", port=a.port,
        compression=None, deployment_identity=deployment_identity_from_config(config)))
    await server.start()
    print(json.dumps(dict(status="relay_ready", mode="live_armed" if a.authorize_dispatch else "observe_only",
                         url=server.url, run_id=a.run_id)), flush=True)
    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, stop.set)
    try:
        await stop.wait()
    finally:
        await server.stop()


def main():
    root = Path("/home/ubuntu/code_junqi/xpolicylab-robot-runtime")
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--exchange-dir", dest="directory", type=Path, required=True)
    p.add_argument("--run-id", required=True)
    p.add_argument("--authorize-dispatch", action="store_true")
    p.add_argument("--port", type=int, default=19026)
    p.add_argument("--runtime", type=Path, default=root)
    p.add_argument("--student-config", type=Path, default=root / "policy/FastWAM/deploy_piper_pick_and_place_drink_step82350.yml")
    asyncio.run(run(p.parse_args()))


if __name__ == "__main__":
    main()
