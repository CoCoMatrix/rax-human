"""Read existing owner status and lossless observation files. Never POST.

The UI JPEG previews are deliberately not used as FastWAM inputs. The formal
graph's no-action observation tap publishes latest.json pointing at an immutable
NPZ with state14 and camera_head/camera_left/camera_right uint8 RGB arrays.
"""
from pathlib import Path
import json
import time
import urllib.request
import urllib.parse

import numpy as np

from ..contracts import CAMERA_ROLES, FIELDS, UNITS, STATE_SCHEMA_ID, PiperObservation, require


class ObservationReader:
    def __init__(self, owner_url, observation_dir, contract, *, max_age_s=1800):
        parsed = urllib.parse.urlparse(owner_url)
        require(parsed.scheme == "http" and parsed.hostname in ("127.0.0.1", "localhost"),
                "owner URL must be loopback HTTP")
        self.url = owner_url.rstrip("/")
        self.root = Path(observation_dir).resolve()
        self.contract, self.max_age_s = contract, max_age_s
        self.baseline_writes = None

    def owner(self):
        request = urllib.request.Request(self.url + "/api/state", method="GET")
        with urllib.request.urlopen(request, timeout=3) as response:
            value = json.load(response)
        robot = value.get("robot_status", {})
        count = robot.get("hardware_write_count")
        require(type(count) is int and count >= 0, "owner write counter unavailable")
        if self.baseline_writes is None:
            self.baseline_writes = count
        require(count == self.baseline_writes, "owner moved during no-send shadow")
        selected = robot.get("selected_action_source", value.get("collection", {}).get("selected_action_source"))
        require(selected in (None, "HOLD"), "owner must remain in HOLD during shadow")
        state = value.get("robot_state", {})
        require(state.get("available") and state.get("schema_id") == STATE_SCHEMA_ID
                and state.get("names") == list(FIELDS) and state.get("units") == list(UNITS),
                "owner measured state14 schema unavailable")
        return value

    def read(self, conditioning):
        before = self.owner()
        metadata_path = self.root / "latest.json"
        if not metadata_path.exists():
            metadata_path = self.root / "metadata.json"
        metadata = json.loads(metadata_path.read_text())
        path = (self.root / metadata.get("arrays_path", "observation.npz")).resolve()
        require(path.is_relative_to(self.root), "observation NPZ escaped capture directory")
        captured = int(metadata["timestamp_ns"])
        age = (time.time_ns() - captured) / 1e9
        require(0 <= age <= self.max_age_s, "lossless observation capture is stale")
        with np.load(path, allow_pickle=False) as record:
            state14 = np.array(record["state14"], copy=True)
            images = {key: np.array(record[key], copy=True) for key in CAMERA_ROLES}
        require(state14.shape == (14,) and state14.dtype.kind == "f" and np.isfinite(state14).all(),
                "lossless observation requires finite floating state14")
        for role, image in images.items():
            require(image.dtype == np.uint8 and image.shape == (720, 1280, 3),
                    role + " requires original uint8[720,1280,3] RGB; UI previews are not inputs")
        self.contract.validate_measured_state(state14)
        after = self.owner()
        require(before["robot_status"].get("generation") == after["robot_status"].get("generation"),
                "owner generation changed during observation")
        observation = PiperObservation(state14=state14, images=images, instruction=conditioning,
            timestamp_ns=captured, sequence=int(metadata["sequence"]),
            robot_id=self.contract.robot_id, identity=self.contract.identity,
            age_ms=age * 1000, metadata=metadata.get("metadata", {}))
        return observation, dict(owner_before=before, owner_after=after,
                                 source_metadata=metadata, source_arrays=str(path),
                                 capture_age_s=age, frozen_snapshot=True,
                                 execution_freshness_certified=False)
