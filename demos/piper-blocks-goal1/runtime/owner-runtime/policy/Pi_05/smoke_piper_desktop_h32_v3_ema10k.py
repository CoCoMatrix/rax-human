#!/usr/bin/env python3
"""Run one non-mutating restore/inference for the h32 v3 desktop bundle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from XPolicyLab.policy.Pi_05.model import (
    PIPER_X_DUAL_DESKTOP_H32_V3_EMA_PROFILE,
    Model,
)
from XPolicyLab.policy.Pi_05.piper_desktop_h32_v3_bundle import (
    ACTION_FREQUENCY_HZ,
    ACTION_HORIZON,
    PROMPTS,
)
from XPolicyLab.real_world.contracts import ActionChunk, ObservationFrame
from XPolicyLab.real_world.robots.piper_x_dual.mapping import (
    PIPER_X_DUAL_STATE_SCHEMA,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()

    started = time.monotonic()
    model = Model(
        {
            "policy_name": "Pi_05",
            "runtime_profile": PIPER_X_DUAL_DESKTOP_H32_V3_EMA_PROFILE,
            "bundle_root": str(args.bundle),
            "action_frequency_hz": ACTION_FREQUENCY_HZ,
            "action_horizon": ACTION_HORIZON,
            "pytorch_device": args.device,
            "rtc_enabled": False,
            "allow_dummy_policy": False,
        }
    )
    restore_seconds = time.monotonic() - started
    image = np.zeros((224, 224, 3), dtype=np.uint8)
    observation = ObservationFrame(
        state_schema=PIPER_X_DUAL_STATE_SCHEMA,
        state=np.zeros((14,), dtype=np.float32),
        images={
            "camera_head": image,
            "camera_left": image,
            "camera_right": image,
        },
        instruction=PROMPTS[0],
        timestamp_ns=time.time_ns(),
        sequence=0,
        robot_type="piper_x_dual",
        robot_id="offline-smoke",
    )
    model.update_obs(observation.to_policy_observation())
    started = time.monotonic()
    chunk = ActionChunk.from_policy_result(model.get_action())
    inference_seconds = time.monotonic() - started
    print(
        json.dumps(
            {
                "ok": True,
                "evidence": "local_non_mutating_synthetic_inference",
                "bundle": str(args.bundle.expanduser().resolve()),
                "device": args.device,
                "restore_seconds": restore_seconds,
                "inference_seconds": inference_seconds,
                "action_shape": list(chunk.values.shape),
                "action_dtype": str(chunk.values.dtype),
                "actions_finite": bool(np.isfinite(chunk.values).all()),
                "frequency_hz": chunk.frequency_hz,
                "execute_prefix_steps": chunk.metadata["execute_prefix_steps"],
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
