"""Synchronous facade over the established asynchronous XPolicy WS client."""
import asyncio
from pathlib import Path
import sys
import threading

import numpy as np
import yaml

from ..contracts import require, vector_schema


class FastWAMStudent:
    def __init__(self, runtime, url, config_path):
        sys.path.insert(0, str(runtime))
        from client_server.ws.protocol.client import PolicyEvalClient, PolicyEvalClientConfig
        from client_server.ws.model_server import deployment_identity_from_config
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.client = PolicyEvalClient(PolicyEvalClientConfig(
            url=url, evaluation_id="astra-fastwam-piper-shadow", max_connect_attempts=1,
            connect_timeout_s=5, request_timeout_s=120, compression=None, proxy=None))
        config = yaml.safe_load(Path(config_path).read_text())
        if not config.get("bundle_root"):
            descriptor = yaml.safe_load((Path(runtime) / "policy/Pi_05/deployments/"
                "piper-pick-and-place-drink-fastwam-step82350.yml").read_text())
            require(descriptor["model"]["backend"] == "fastwam", "wrong student deployment backend")
            config["bundle_root"] = descriptor["bundle"]["default_path"]
        self.expected = deployment_identity_from_config(config)
        self.metadata = {}

    def connect(self):
        hello = asyncio.run_coroutine_threadsafe(self.client.connect(handshake=True), self.loop).result(15)
        require(hello is not None and hello.payload.get("deployment") == self.expected,
                "FastWAM HELLO differs from selected deployment config")
        self.metadata = dict(hello.payload)
        return self.metadata

    def infer(self, observation):
        response = asyncio.run_coroutine_threadsafe(
            self.client.infer(observation.to_policy_observation()), self.loop).result(130)
        payload = dict(response.payload)
        actions = payload.get("actions")
        require(isinstance(actions, np.ndarray) and actions.dtype == np.float32
                and actions.shape == (32, 14) and np.isfinite(actions).all(),
                "FastWAM must return unchanged finite float32[32,14]")
        require(payload.get("action_schema") == vector_schema(action=True), "FastWAM action schema differs")
        require(payload.get("action_frequency_hz") == 30, "FastWAM cadence differs from 30Hz")
        return actions.copy(), payload

    def close(self):
        if not self.loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.client.close(), self.loop).result(10)
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(timeout=5)
            self.loop.close()
