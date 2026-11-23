"""Synchronous environment-side adapter for the policy websocket protocol."""

from __future__ import annotations

import asyncio
import math
from collections.abc import Mapping
from typing import Any, Literal, cast

from client_server.ws.protocol.client import PolicyEvalClient, PolicyEvalClientConfig


class WsModelClient:
    def __init__(
        self,
        *,
        url: str,
        evaluation_id: str,
        trial_id: str,
        action_case_id: str | None = None,
        repeat_index: int | None = None,
        policy_context: Mapping[str, Any] | None = None,
        compression: Literal["deflate"] | None = None,
        client: Any | None = None,
    ):
        self.action_case_id = action_case_id
        self.trial_id = trial_id
        self.repeat_index = repeat_index
        self.policy_context = (
            dict(policy_context) if policy_context is not None else None
        )
        self._step = 0
        self._latest_obs: Any | None = None
        self._latest_obs_batch: list[Any] | None = None
        self._inference_latencies_ms: list[float] = []
        self._loop = asyncio.new_event_loop()
        self._client = client or PolicyEvalClient(
            PolicyEvalClientConfig(
                url=url,
                evaluation_id=evaluation_id,
                compression=compression,
            )
        )
        self._loop.run_until_complete(self._client.connect(handshake=True))

    def call(self, func_name: str | None = None, obs: Any = None, **kwargs: Any) -> Any:
        if func_name == "prepare_case":
            if self.action_case_id is None:
                raise ValueError("prepare_case requires action_case_id")
            response = self._loop.run_until_complete(
                self._client.prepare_case(
                    self.action_case_id,
                    case_meta=obs if isinstance(obs, dict) else None,
                )
            )
            return response.payload.get("result")

        if func_name == "reset":
            self._step = 0
            self._latest_obs = None
            self._latest_obs_batch = None
            self._inference_latencies_ms = []
            response = self._loop.run_until_complete(
                self._client.reset(
                    trial_id=self.trial_id,
                    action_case_id=self.action_case_id,
                    repeat_index=self.repeat_index,
                    payload=obs if isinstance(obs, dict) else None,
                )
            )
            return response.payload.get("result")

        if func_name == "update_obs":
            self._latest_obs = obs
            return None

        if func_name == "get_action":
            observation = obs if obs is not None else self._latest_obs
            if observation is None:
                raise ValueError(
                    "get_action requires obs or a previous update_obs call"
                )
            response = self.infer(cast(dict[str, Any], observation))
            return response.payload.get("actions")

        if func_name == "update_obs_batch":
            self._latest_obs_batch = list(obs) if obs is not None else []
            return None

        if func_name == "get_action_batch":
            observations = self._latest_obs_batch
            if observations is None:
                raise ValueError(
                    "get_action_batch requires a previous update_obs_batch call"
                )
            actions = []
            for observation in observations:
                response = self._loop.run_until_complete(
                    self._client.infer(
                        cast(dict[str, Any], observation),
                        policy_context=self.policy_context,
                        trial_id=self.trial_id,
                        action_case_id=self.action_case_id,
                        step=self._step,
                    )
                )
                self._record_latency(response)
                actions.append(response.payload.get("actions"))
            self._step += 1
            return actions

        if func_name == "trial_end":
            response = self._loop.run_until_complete(
                self._client.trial_end(
                    trial_id=self.trial_id,
                    action_case_id=self.action_case_id,
                    repeat_index=self.repeat_index,
                    result=obs if isinstance(obs, dict) else None,
                )
            )
            return response.payload.get("result")

        raise NotImplementedError(f"unsupported websocket model call: {func_name}")

    def set_trial_context(
        self,
        *,
        trial_id: str,
        repeat_index: int | None = None,
        action_case_id: str | None = None,
    ) -> None:
        if not trial_id:
            raise ValueError("trial_id must be non-empty")
        self.trial_id = trial_id
        self.repeat_index = repeat_index
        self.action_case_id = action_case_id
        self._step = 0
        self._latest_obs = None
        self._latest_obs_batch = None
        self._inference_latencies_ms = []

    @property
    def inference_latencies_ms(self) -> tuple[float, ...]:
        return tuple(self._inference_latencies_ms)

    def _record_latency(self, response: Any) -> None:
        value = response.payload.get("latency_ms")
        if value is None:
            return
        value = float(value)
        if math.isfinite(value) and value >= 0:
            self._inference_latencies_ms.append(value)

    def infer(
        self,
        observation: dict[str, Any],
        *,
        policy_context: Mapping[str, Any] | None = None,
    ) -> Any:
        """Return the full inference frame, including action schema and metadata."""
        response = self._loop.run_until_complete(
            self._client.infer(
                observation,
                policy_context=(
                    dict(policy_context)
                    if policy_context is not None
                    else self.policy_context
                ),
                trial_id=self.trial_id,
                action_case_id=self.action_case_id,
                step=self._step,
            )
        )
        self._record_latency(response)
        self._step += 1
        return response

    def set_policy_context(self, context: Mapping[str, Any] | None) -> None:
        """Set a fixed deployment condition for subsequent inference requests."""

        self.policy_context = dict(context) if context is not None else None

    def close(self) -> None:
        if self._loop.is_closed():
            return
        try:
            self._loop.run_until_complete(self._client.close())
        finally:
            self._loop.close()

    def __enter__(self) -> WsModelClient:
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.close()
