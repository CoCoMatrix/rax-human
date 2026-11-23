#!/usr/bin/env python3
"""Send one synthetic dual-PiPER observation through a policy WebSocket."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any, Mapping

import numpy as np

from XPolicyLab.real_world.contracts import ActionChunk, ObservationFrame
from XPolicyLab.real_world.robots.piper_x_dual.mapping import (
    PIPER_X_DUAL_ACTION_SCHEMA,
    PIPER_X_DUAL_STATE_SCHEMA,
)


def validate_policy_payload(
    payload: Mapping[str, Any],
    *,
    expected_horizon: int,
    expected_frequency_hz: float,
    expected_execute_prefix: int,
) -> ActionChunk:
    chunk = ActionChunk.from_policy_result(payload)
    if chunk.schema != PIPER_X_DUAL_ACTION_SCHEMA:
        raise ValueError(
            f"wrong action schema: {chunk.schema.schema_id!r} != "
            f"{PIPER_X_DUAL_ACTION_SCHEMA.schema_id!r}"
        )
    if chunk.values.shape != (expected_horizon, PIPER_X_DUAL_ACTION_SCHEMA.dim):
        raise ValueError(
            f"wrong action shape: {chunk.values.shape} != "
            f"({expected_horizon}, {PIPER_X_DUAL_ACTION_SCHEMA.dim})"
        )
    if not np.isclose(chunk.frequency_hz, expected_frequency_hz):
        raise ValueError(
            f"wrong action frequency: {chunk.frequency_hz:g} != "
            f"{expected_frequency_hz:g}"
        )
    if not np.isfinite(chunk.values).all():
        raise ValueError("policy service returned NaN or Inf")
    execute_prefix = int(chunk.metadata.get("execute_prefix_steps", -1))
    if execute_prefix != expected_execute_prefix:
        raise ValueError(
            f"wrong execute prefix: {execute_prefix} != {expected_execute_prefix}"
        )
    return chunk


def synthetic_observation(
    task: str, *, image_width: int = 1920, image_height: int = 1080
) -> dict[str, Any]:
    if image_width < 1 or image_height < 1:
        raise ValueError("synthetic image dimensions must be positive")
    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
    return ObservationFrame(
        state_schema=PIPER_X_DUAL_STATE_SCHEMA,
        state=np.zeros((14,), dtype=np.float32),
        images={
            "camera_head": image,
            "camera_left": image,
            "camera_right": image,
        },
        instruction=task,
        timestamp_ns=time.time_ns(),
        sequence=0,
        robot_type="piper_x_dual",
        robot_id="offline-service-probe",
        metadata={"qualification": "synthetic_no_hardware"},
    ).to_policy_observation()


def main() -> int:
    from client_server.ws.model_client import WsModelClient

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--action-horizon", type=int, required=True)
    parser.add_argument("--frequency-hz", type=float, required=True)
    parser.add_argument("--execute-prefix", type=int, required=True)
    parser.add_argument("--image-width", type=int, required=True)
    parser.add_argument("--image-height", type=int, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    started = time.monotonic()
    with WsModelClient(
        url=args.url,
        evaluation_id="piper-qualification",
        trial_id="synthetic-service-probe",
    ) as client:
        response = client.infer(
            synthetic_observation(
                args.task,
                image_width=args.image_width,
                image_height=args.image_height,
            )
        )
    chunk = validate_policy_payload(
        response.payload,
        expected_horizon=args.action_horizon,
        expected_frequency_hz=args.frequency_hz,
        expected_execute_prefix=args.execute_prefix,
    )
    report = {
        "ok": True,
        "evidence": "websocket_synthetic_inference",
        "url": args.url,
        "roundtrip_seconds": time.monotonic() - started,
        "request_id": response.request_id,
        "action_shape": list(chunk.values.shape),
        "action_dtype": str(chunk.values.dtype),
        "actions_finite": True,
        "frequency_hz": chunk.frequency_hz,
        "execute_prefix_steps": chunk.metadata["execute_prefix_steps"],
        "input_image_shape": [args.image_height, args.image_width, 3],
        "input_state_shape": [14],
    }
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(encoded, encoding="utf-8")
        temporary.replace(output)
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
