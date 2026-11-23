from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq

from .mapping import flatten_jiaxin_action_chunk
from .rtc import PendingChunk


class JiaxinInferenceClient:
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
            self.socket.send(msgpack.packb(request, default=mnp.encode))
            response = msgpack.unpackb(self.socket.recv(), object_hook=mnp.decode, raw=False)
        except zmq.error.Again:
            self._init_socket()
            raise
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(f"Jiaxin server error: {response['error']}")
        return response

    def infer(self, observation: dict) -> tuple[np.ndarray, dict]:
        options = {"instruction": observation["language"]["task"][0][0]}
        action, info = self._request(
            "get_action",
            {"observation": observation, "options": options},
        )
        return flatten_jiaxin_action_chunk(action), info if isinstance(info, dict) else {}

    def close(self) -> None:
        self.socket.close(linger=0)
        self.context.term()


@dataclass(frozen=True)
class InferenceResult:
    pending: PendingChunk
    chunk: np.ndarray | None
    info: dict
    latency_ms: float
    inference_start_timestamp_ns: int
    inference_end_timestamp_ns: int
    error: str | None = None


class AsyncInference:
    def __init__(self, client: JiaxinInferenceClient):
        self.client = client
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._result: InferenceResult | None = None

    def submit(self, observation: dict, pending: PendingChunk) -> bool:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False

        def work():
            started = time.monotonic()
            started_ns = time.time_ns()
            try:
                chunk, info = self.client.infer(observation)
                ended_ns = time.time_ns()
                result = InferenceResult(
                    pending=pending,
                    chunk=chunk,
                    info=info,
                    latency_ms=(time.monotonic() - started) * 1000.0,
                    inference_start_timestamp_ns=started_ns,
                    inference_end_timestamp_ns=ended_ns,
                )
            except Exception as exc:  # RPC boundary: report to the Dora loop.
                ended_ns = time.time_ns()
                result = InferenceResult(
                    pending=pending,
                    chunk=None,
                    info={},
                    latency_ms=(time.monotonic() - started) * 1000.0,
                    inference_start_timestamp_ns=started_ns,
                    inference_end_timestamp_ns=ended_ns,
                    error=f"{type(exc).__name__}: {exc}",
                )
            with self._lock:
                self._result = result

        thread = threading.Thread(target=work, daemon=True, name="gr00t-inference")
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
