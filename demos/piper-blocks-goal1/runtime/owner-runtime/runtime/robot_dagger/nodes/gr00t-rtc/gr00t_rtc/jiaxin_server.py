"""Thin Jiaxin server wrapper that forwards physical previous actions into native RTC."""

from __future__ import annotations

import argparse
from typing import Any

import numpy as np
import torch

from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import MessageType, VLAStepData
from gr00t.policy.gr00t_policy import Gr00tPolicy, _rec_to_dtype
from gr00t.policy.server_client import PolicyServer


class RtcGr00tPolicy(Gr00tPolicy):
    def _get_action(
        self, observation: dict[str, Any], options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        options = dict(options or {})
        previous = options.pop("previous_action", None)
        unbatched = self._unbatch_observation(observation)
        if previous is not None and len(unbatched) != 1:
            raise ValueError("RTC prototype only supports batch size 1")

        processed = []
        states = []
        for obs in unbatched:
            actions = {
                key: np.asarray(value, dtype=np.float32)
                for key, value in (previous or {}).items()
            }
            step = VLAStepData(
                images=obs["video"],
                states=obs["state"],
                actions=actions,
                text=obs["language"][self.language_key][0],
                embodiment=self.embodiment_tag,
            )
            states.append(step.states)
            processed.append(
                self.processor([{"type": MessageType.EPISODE_STEP.value, "content": step}])
            )

        collated = _rec_to_dtype(self.collate_fn(processed), dtype=torch.bfloat16)
        model_options = None
        if previous is not None:
            required = (
                "action_horizon",
                "rtc_overlap_steps",
                "rtc_frozen_steps",
                "rtc_ramp_rate",
            )
            missing = [key for key in required if key not in options]
            if missing:
                raise ValueError(f"missing RTC options: {missing}")
            model_options = {key: options[key] for key in required}

        with torch.inference_mode():
            prediction = self.model.get_action(**collated, options=model_options)
        normalized = prediction["action_pred"].float()
        batched_states = {
            key: np.stack([state[key] for state in states], axis=0)
            for key in self.modality_configs["state"].modality_keys
        }
        action = self.processor.decode_action(
            normalized.cpu().numpy(), self.embodiment_tag, batched_states
        )
        return {key: value.astype(np.float32) for key, value in action.items()}, {
            "rtc": previous is not None
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--embodiment-tag", default="new_embodiment")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5678)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--no-strict", action="store_true")
    args = parser.parse_args()

    policy = RtcGr00tPolicy(
        embodiment_tag=EmbodimentTag.resolve(args.embodiment_tag),
        model_path=args.model_path,
        device=args.device,
        strict=not args.no_strict,
    )
    with PolicyServer(policy=policy, host=args.host, port=args.port) as server:
        server.run()


if __name__ == "__main__":
    main()
