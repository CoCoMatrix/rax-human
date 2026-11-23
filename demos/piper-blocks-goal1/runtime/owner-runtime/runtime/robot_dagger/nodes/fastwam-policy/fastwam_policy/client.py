from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq

from .mapping import flatten_fastwam_action_chunk


class FastWamInferenceClient:
    def __init__(self, host: str, port: int, timeout_ms: int = 300_000):
        self.context = zmq.Context()
        self.endpoint = f"tcp://{host}:{port}"
        self.timeout_ms = timeout_ms
        self._init_socket()

    def _init_socket(self) -> None:
        if hasattr(self, "socket"):
            self.socket.close(linger=0)
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.connect(self.endpoint)

    def _request(self, endpoint: str, data=None):
        request = {"endpoint": endpoint}
        if data is not None:
            request["data"] = data
        try:
            self.socket.send(msgpack.packb(request, default=mnp.encode, use_bin_type=True))
            response = msgpack.unpackb(
                self.socket.recv(), object_hook=mnp.decode, raw=False
            )
        except zmq.error.Again:
            self._init_socket()
            raise
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(f"FastWAM server error: {response['error']}")
        return response

    def infer(
        self,
        observation: dict,
        *,
        rtc_prefix: np.ndarray | None = None,
        rtc_inference_delay: int = 0,
        rtc_prefix_horizon: int | None = None,
        rtc_prefix_attention_schedule: str = "exp",
        rtc_max_guidance_weight: float = 5.0,
        observation_action_index: int | None = None,
    ) -> tuple[np.ndarray, dict]:
        instruction = observation["language"]["task"][0][0]
        options = {
            "instruction": instruction,
            "action_space": "selected",
        }
        if observation_action_index is not None:
            options["observation_action_index"] = int(observation_action_index)
        if rtc_prefix is not None:
            prefix = np.asarray(rtc_prefix, dtype=np.float32)
            if prefix.ndim != 2 or prefix.shape[1] != 31:
                raise ValueError(
                    f"RTC prefix must have shape [K,31], got {prefix.shape}"
                )
            if not np.isfinite(prefix).all():
                raise ValueError("RTC prefix contains non-finite values")
            options.update(
                {
                    "rtc_prev_action_chunk": prefix,
                    "rtc_inference_delay": int(rtc_inference_delay),
                    "rtc_prefix_horizon": int(
                        len(prefix)
                        if rtc_prefix_horizon is None
                        else min(rtc_prefix_horizon, len(prefix))
                    ),
                    "rtc_prefix_attention_schedule": rtc_prefix_attention_schedule,
                    "rtc_max_guidance_weight": float(rtc_max_guidance_weight),
                }
            )
        action, info = self._request(
            "get_action",
            {
                "observation": observation,
                "options": options,
            },
        )
        return (
            flatten_fastwam_action_chunk(action),
            info if isinstance(info, dict) else {},
        )

    def reset(self):
        return self._request("reset", {"options": None})

    def close(self) -> None:
        self.socket.close(linger=0)
        self.context.term()


@dataclass(frozen=True)
class InferenceResult:
    generation: int
    chunk: np.ndarray | None
    info: dict
    latency_ms: float
    action_index: int
    rtc_prefix_steps: int
    error: str | None = None


class AsyncInference:
    def __init__(self, client: FastWamInferenceClient):
        self.client = client
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._result: InferenceResult | None = None

    def submit(
        self,
        observation: dict,
        generation: int,
        *,
        reset: bool = False,
        action_index: int = 0,
        rtc_prefix: np.ndarray | None = None,
        rtc_inference_delay: int = 0,
        rtc_prefix_horizon: int | None = None,
        rtc_prefix_attention_schedule: str = "exp",
        rtc_max_guidance_weight: float = 5.0,
    ) -> bool:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False

        def work():
            started = time.monotonic()
            try:
                if reset:
                    self.client.reset()
                chunk, info = self.client.infer(
                    observation,
                    rtc_prefix=rtc_prefix,
                    rtc_inference_delay=rtc_inference_delay,
                    rtc_prefix_horizon=rtc_prefix_horizon,
                    rtc_prefix_attention_schedule=rtc_prefix_attention_schedule,
                    rtc_max_guidance_weight=rtc_max_guidance_weight,
                    observation_action_index=action_index,
                )
                result = InferenceResult(
                    generation=generation,
                    chunk=chunk,
                    info=info,
                    latency_ms=(time.monotonic() - started) * 1000.0,
                    action_index=action_index,
                    rtc_prefix_steps=0 if rtc_prefix is None else len(rtc_prefix),
                )
            except Exception as exc:  # RPC boundary: report to the Dora loop.
                result = InferenceResult(
                    generation=generation,
                    chunk=None,
                    info={},
                    latency_ms=(time.monotonic() - started) * 1000.0,
                    action_index=action_index,
                    rtc_prefix_steps=0 if rtc_prefix is None else len(rtc_prefix),
                    error=f"{type(exc).__name__}: {exc}",
                )
            with self._lock:
                self._result = result

        thread = threading.Thread(target=work, daemon=True, name="fastwam-inference")
        with self._lock:
            self._thread = thread
        thread.start()
        return True

    def poll(self) -> InferenceResult | None:
        with self._lock:
            result, self._result = self._result, None
        return result

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()
