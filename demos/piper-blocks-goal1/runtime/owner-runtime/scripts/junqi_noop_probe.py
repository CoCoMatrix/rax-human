#!/usr/bin/env python3
"""Probe a transport-only policy using synthetic images; never opens hardware."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import socket
import time
from typing import Any
from uuid import uuid4

import numpy as np

from client_server.ws.model_client import WsModelClient
from XPolicyLab.policy.TransportNoOp.model import CAMERAS
from XPolicyLab.real_world.contracts import ObservationFrame
from XPolicyLab.real_world.robots.piper_x_dual.mapping import (
    PIPER_X_DUAL_ACTION_SCHEMA,
    PIPER_X_DUAL_STATE_SCHEMA,
)


def synthetic_observation(
    sequence: int, *, image_width: int = 1280, image_height: int = 720
) -> dict[str, Any]:
    """Build visibly synthetic, distinct camera arrays and a nonzero state."""
    if sequence < 0 or image_width < 1 or image_height < 1:
        raise ValueError("sequence and image dimensions are invalid")
    images = {}
    for index, name in enumerate(CAMERAS):
        image = np.empty((image_height, image_width, 3), dtype=np.uint8)
        image[..., 0] = (
            np.arange(image_width, dtype=np.uint32)[None, :] + index * 71 + sequence
        ) % 256
        image[..., 1] = (
            np.arange(image_height, dtype=np.uint32)[:, None] + index * 43 + sequence
        ) % 256
        image[..., 2] = (index * 61 + sequence * 13 + 17) % 256
        images[name] = image
    return ObservationFrame(
        state_schema=PIPER_X_DUAL_STATE_SCHEMA,
        state=np.arange(1, 15, dtype=np.float32) * np.float32(0.001)
        + np.float32(sequence * 0.00001),
        images=images,
        instruction="Synthetic transport verification only; do not move the robot.",
        timestamp_ns=time.time_ns(),
        sequence=sequence,
        robot_type="piper_x_dual",
        robot_id="junqi-synthetic-no-hardware",
        metadata={
            "evidence": "synthetic_no_hardware",
            "synthetic": True,
            "transport_request_id": f"synthetic-{uuid4()}-{sequence}",
        },
    ).to_policy_observation()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_receipt(
    payload: dict[str, Any],
    observation: dict[str, Any],
    *,
    expected_server_hostname: str | None = None,
) -> dict[str, Any]:
    """Validate every transmitted array against the returned server receipt."""
    _require(payload.get("result_kind") == "no_action", "result_kind must be no_action")
    actions = payload.get("actions")
    _require(
        isinstance(actions, np.ndarray)
        and actions.dtype == np.float32
        and actions.shape == (0, 14),
        "actions must be float32[0,14] with no executable targets",
    )
    _require(
        payload.get("action_schema") == PIPER_X_DUAL_ACTION_SCHEMA.to_wire(),
        "action schema mismatch",
    )
    _require(payload.get("action_frequency_hz") == 30.0, "action frequency mismatch")
    metadata = payload.get("policy_metadata", {})
    _require(metadata.get("inference_executed") is False, "must not claim inference")
    _require(metadata.get("transport_only") is True, "must declare transport_only")
    receipt = metadata.get("transport_receipt")
    _require(isinstance(receipt, dict), "transport receipt missing")
    _require(
        receipt.get("schema_version") == "xpolicy.transport_receipt.v1",
        "transport receipt schema mismatch",
    )
    info = observation["additional_info"]
    _require(receipt.get("sequence") == info["sequence"], "sequence mismatch")
    _require(
        receipt.get("observation_timestamp_ns") == info["timestamp_ns"],
        "observation timestamp mismatch",
    )
    _require(receipt.get("robot_id") == info["robot_id"], "robot ID mismatch")
    _require(
        receipt.get("request_id") == info["metadata"]["transport_request_id"],
        "observation request ID mismatch",
    )
    hostname = receipt.get("server_hostname")
    _require(isinstance(hostname, str) and bool(hostname.strip()), "server hostname missing")
    if expected_server_hostname is not None:
        _require(hostname == expected_server_hostname, "unexpected server hostname")
    for name in ("received_time_ns", "received_monotonic_ns"):
        _require(type(receipt.get(name)) is int and receipt[name] > 0, f"invalid {name}")
    expected_arrays = {"state": observation["state"]["vector"]}
    expected_arrays.update({
        name: observation["vision"][name]["color"] for name in CAMERAS
    })
    _require(set(receipt.get("images", {})) == set(CAMERAS), "camera receipt roles mismatch")
    for name, array in expected_arrays.items():
        actual = receipt.get("state", {}) if name == "state" else receipt["images"][name]
        digest = hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()
        _require(actual.get("sha256") == digest, f"{name} SHA-256 mismatch")
        _require(actual.get("shape") == list(array.shape), f"{name} shape mismatch")
        _require(actual.get("dtype") == str(array.dtype), f"{name} dtype mismatch")
        if name != "state":
            _require(actual.get("encoding") == "rgb8", f"{name} RGB encoding mismatch")
    return receipt


def run_probe(
    url: str, *, count: int = 3, expected_server_hostname: str | None = None
) -> dict[str, Any]:
    if count < 1:
        raise ValueError("count must be positive")
    report: dict[str, Any] = {
        "schema_version": "xpolicy.junqi_noop_probe.v1",
        "evidence": "synthetic_websocket_transport_only",
        "synthetic": True,
        "hardware_access": False,
        "inference_executed": False,
        "client_hostname": socket.gethostname(),
        "url": url,
        "started_time_ns": time.time_ns(),
        "input_image_shape": [720, 1280, 3],
        "input_state_shape": [14],
        "requested_count": count,
        "responses": [],
        "status": "running",
    }
    try:
        with WsModelClient(
            url=url,
            evaluation_id="junqi-noop-synthetic-transport",
            trial_id=f"synthetic-{uuid4()}",
        ) as client:
            for sequence in range(count):
                observation = synthetic_observation(sequence)
                started = time.monotonic_ns()
                response = client.infer(observation)
                rtt_ms = (time.monotonic_ns() - started) / 1_000_000
                receipt = validate_receipt(
                    response.payload,
                    observation,
                    expected_server_hostname=expected_server_hostname,
                )
                _require(bool(response.request_id), "protocol response request ID missing")
                report["responses"].append({
                    "sequence": sequence,
                    "protocol_request_id": response.request_id,
                    "roundtrip_ms": rtt_ms,
                    "result_kind": response.payload["result_kind"],
                    "action_shape": list(response.payload["actions"].shape),
                    "action_dtype": str(response.payload["actions"].dtype),
                    "action_schema": response.payload["action_schema"],
                    "action_frequency_hz": response.payload["action_frequency_hz"],
                    "inference_executed": False,
                    "transport_receipt": receipt,
                })
        report["status"] = "passed"
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
    report["completed_time_ns"] = time.time_ns()
    report["validated_count"] = len(report["responses"])
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--expected-server-hostname")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be positive")
    report = run_probe(
        args.url, count=args.count, expected_server_hostname=args.expected_server_hostname
    )
    serialized = json.dumps(report, indent=2, sort_keys=True)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized + "\n", encoding="utf-8")
    print(serialized, flush=True)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
