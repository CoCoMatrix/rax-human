"""File transport to the formal WS relay; no SDK or alternative device writer."""
from pathlib import Path
from types import SimpleNamespace
import json
import re
import time
import uuid
import urllib.request

import numpy as np

from hybrid_rollout.robodojo.io import write_json
from ..contracts import PiperObservation, CAMERA_ROLES, require, vector_schema
from . import CONDITIONING


def validate_run_id(value):
    require(isinstance(value, str) and re.fullmatch(r"[A-Za-z0-9_-]{1,96}", value),
            "run-id must contain 1..96 ASCII letters, digits, underscore or dash")
    return value


class RelayClient:
    def __init__(self, directory, run_id, *, timeout=15):
        self.directory, self.run_id = Path(directory).resolve(), validate_run_id(run_id)
        self.timeout = timeout
        manifest = json.loads((self.directory / "relay.json").read_text())
        require(manifest.get("run_id") == run_id and manifest.get("authorize_dispatch") is True,
                "relay is not armed for this explicit run-id")
        require(manifest.get("review_profile") == "policy_prefix", "relay must use policy_prefix profile")

    def stopped(self):
        require(not (self.directory / "stop.json").exists(), "relay run has been stopped")

    def wait_json(self, path, *, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            self.stopped()
            if path.exists():
                result = json.loads(path.read_text())
                if result.get("status") == "rejected":
                    raise RuntimeError("formal relay rejected before dispatch: " + result.get("error", "unknown"))
                return result
            time.sleep(.025)
        raise TimeoutError("formal relay response deadline; no command is retried")

    def observation(self, minimum_timestamp_ns=0):
        identifier = uuid.uuid4().hex
        request = dict(request_id=identifier, run_id=self.run_id,
                       minimum_timestamp_ns=int(minimum_timestamp_ns), requested_ns=time.time_ns())
        write_json(self.directory / "observation-request.json", request)
        return self.wait_json(self.directory / "observations" / (identifier + ".json"))

    def command(self, *, actions, kind, binding):
        identifier = uuid.uuid4().hex
        value = dict(command_id=identifier, run_id=self.run_id, kind=kind,
            teacher_request_id=binding.request_id,
            review_observation_timestamp_ns=binding.observation_timestamp_ns,
            requested_ns=time.time_ns(), targets14=np.asarray(actions).tolist())
        write_json(self.directory / "commands" / (identifier + ".json"), value)
        return self.wait_json(self.directory / "releases" / (identifier + ".json"))

    def stop(self, reason):
        write_json(self.directory / "stop.json", dict(run_id=self.run_id,
                   reason=reason, timestamp_ns=time.time_ns(), no_sdk_command=True))


class LiveObservationReader:
    """Request a new original RGB/state frame from the existing formal graph."""
    def __init__(self, owner_url, relay, contract):
        self.url, self.relay, self.contract = owner_url.rstrip("/"), relay, contract
        self.minimum_timestamp_ns = 0

    def owner(self):
        require(self.url.startswith("http://127.0.0.1:"), "owner URL must be loopback")
        with urllib.request.urlopen(self.url + "/api/state", timeout=3) as response:
            state = json.load(response)
        require(state.get("write_enabled") is True
                and state.get("robot_status", {}).get("hardware_write_enabled") is True,
                "authorized live graph is not write-enabled")
        require(state.get("control_source") == "keyboard", "live agent expects original keyboard owner")
        require(not state.get("pigey", {}).get("active"), "unexpected competing Pigey controller")
        return state

    def read(self, conditioning=CONDITIONING):
        owner = self.owner()
        metadata = self.relay.observation(self.minimum_timestamp_ns)
        directory = (self.relay.directory / metadata["directory"]).resolve()
        require(directory.is_relative_to(self.relay.directory), "relay snapshot escaped its run")
        with np.load(directory / "observation.npz", allow_pickle=False) as record:
            state = np.array(record["state14"], copy=True)
            images = {role: np.array(record[role], copy=True) for role in CAMERA_ROLES}
        age_ms = (time.time_ns() - metadata["timestamp_ns"]) / 1e6
        require(0 <= age_ms <= 300, "fresh relay snapshot expired before agent host read")
        for role, image in images.items():
            require(image.dtype == np.uint8 and image.shape == (720, 1280, 3),
                    "live model input requires three original RGB1280x720 frames")
        observation = PiperObservation(state14=state, images=images, instruction=conditioning,
            timestamp_ns=metadata["timestamp_ns"], sequence=metadata["sequence"],
            robot_id=self.contract.robot_id, identity=self.contract.identity,
            age_ms=age_ms, metadata=metadata["metadata"])
        self.contract.validate_measured_state(state)
        return observation, dict(owner_before=owner, source_metadata=metadata,
            capture_age_s=age_ms / 1000, frozen_snapshot=False,
            execution_freshness_certified=False, source_arrays=str(directory / "observation.npz"))


class LiveExecutor:
    """Submit reviewed targets only through relay→formal graph→sole owner ACK."""
    def __init__(self, relay, journal, reader, contract, reviewer):
        self.relay, self.journal, self.reader = relay, Path(journal), reader
        self.contract, self.reviewer = contract, reviewer
        self.total_acknowledged = 0

    def _wait_ack(self, release, submitted_ns, *, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.relay.stopped()
            rows = []
            if self.journal.exists():
                lines = self.journal.read_text().splitlines()
                for index, line in enumerate(lines):
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        if index == len(lines) - 1:
                            continue
                        raise RuntimeError("owner receipt journal has a malformed completed line")
                    if record.get("event") == "release_rejected" and record.get("recorded_ns", 0) >= submitted_ns:
                        raise RuntimeError("sole owner rejected review release: " + record.get("reason", "unknown"))
                    if record.get("event") == "sdk_dispatch_completed" and record.get("review_release_id") == release["release_id"]:
                        rows.append(record)
            require(len(rows) <= release["steps"], "extra owner ACKs for this release")
            require([r["model_chunk_step_index"] for r in rows] == list(range(len(rows))),
                    "owner ACKs were skipped, duplicated or reordered")
            require(all(r.get("hardware_dispatch_confirmed") is True
                        and r.get("teacher_request_id") == release["teacher_request_id"]
                        and r.get("generation") == release["generation"] for r in rows),
                    "owner ACK identity mismatch")
            if len(rows) == release["steps"]:
                self.reader.minimum_timestamp_ns = max(r["recorded_ns"] for r in rows)
                self.total_acknowledged += len(rows)
                return rows
            time.sleep(.025)
        raise TimeoutError("owner ACK deadline; uncertain physical command is never replayed")

    def _submit(self, actions, kind, binding):
        wire = self.contract.encode_command_float32(np.asarray(actions))
        submitted_ns = time.time_ns()
        response = self.relay.command(actions=wire, kind=kind, binding=binding)
        release = response["release"]
        return dict(release=release, ack=self._wait_ack(release, submitted_ns))

    def execute_decision(self, *, observation, proposal, response, checked):
        receipts = []
        if response["mode"] == "student":
            receipts.append(self._submit(checked.candidate.actions, "student", checked.binding))
        else:
            for index in range(response["steps"]):
                fresh, _ = self.reader.read(CONDITIONING)
                row = SimpleNamespace(actions=proposal.actions[index:index+1])
                # Exactly one correction is solved from measured feedback; the
                # next solve can occur only after this release's actual owner ACK.
                actions = self.reviewer._correct(fresh, row, response, 1)
                candidate = self.contract.validate_candidate(actions, schema=vector_schema(action=True),
                    frequency_hz=30, measured_state14=fresh.state14, prefix_steps=1)
                receipts.append(self._submit(candidate.actions, response["mode"], checked.binding))
        count = sum(len(record["ack"]) for record in receipts)
        return dict(mode="live", selected_steps=response["steps"], executed_steps=count,
            hardware_dispatch_confirmed=True, pose_arrival_confirmed=False,
            object_outcome_confirmed=False, releases=receipts,
            review_request_id=checked.binding.request_id)

    def stop(self, reason):
        self.relay.stop(reason)
