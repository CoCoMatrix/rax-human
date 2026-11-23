from __future__ import annotations

from dataclasses import dataclass
import json
import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from farther_core.gr3_dagger import (
    ACTION_DIM,
    SCHEMA_VERSION,
    canonical_robot_action,
)
from farther_core.serde import JointStateSerde

from .client import AsyncInference, JiaxinInferenceClient
from .mapping import base_action, build_jiaxin_observation, joint_action
from .rtc import RtcQueue

@dataclass
class Observation:
    image: np.ndarray | None = None
    state: dict | None = None
    base_state: dict | None = None


def _scalar(value):
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _image(value) -> np.ndarray:
    flat = value.to_numpy(zero_copy_only=False)
    width = int(os.getenv("IMAGE_WIDTH", "640"))
    height = int(os.getenv("IMAGE_HEIGHT", "400"))
    channels = int(os.getenv("IMAGE_CHANNELS", "3"))
    return np.asarray(flat, dtype=np.uint8).reshape(height, width, channels)


def control_state(command: str) -> tuple[bool, bool]:
    return command in {"bootstrap", "run", "step"}, command == "step"


def limit_action_chunk(chunk: np.ndarray, size: int) -> np.ndarray:
    if size < 1:
        raise ValueError("ACTION_CHUNK_SIZE must be positive")
    return chunk[:size]


def main() -> None:
    host = os.getenv("MODEL_HOST", os.getenv("GR00T_HOST", "127.0.0.1"))
    port = int(os.getenv("MODEL_PORT", os.getenv("GR00T_PORT", "5678")))
    task = os.getenv("TASK", "")
    retry_seconds = float(os.getenv("MODEL_RETRY_SECONDS", "0.5"))
    bgr = os.getenv("CAMERA_IS_BGR", "1") == "1"
    rotate = os.getenv("GR00T_ROTATE_180", "0") == "1"
    action_chunk_size = int(os.getenv("ACTION_CHUNK_SIZE", "16"))
    if action_chunk_size < 1:
        raise ValueError("ACTION_CHUNK_SIZE must be positive")

    node = Node()
    rpc = JiaxinInferenceClient(host, port, int(os.getenv("MODEL_TIMEOUT_MS", "300000")))
    inference = AsyncInference(rpc)
    queue = RtcQueue(trigger_steps=0)
    observation = Observation()
    enabled = False
    single_chunk = False
    generation = 0
    chunk_id = 0
    last_status = None
    last_info: dict = {}
    retry_at = 0.0

    def send_status(status: str, **detail) -> None:
        nonlocal last_status
        payload = {"status": status, "generation": generation, **detail}
        signature = json.dumps(payload, sort_keys=True)
        if signature == last_status:
            return
        last_status = signature
        timestamp_ns = time.time_ns()
        node.send_output(
            "status",
            pa.array([signature]),
            {
                "timestamp_ns": timestamp_ns,
                "source_timestamp_ns": timestamp_ns,
                "target_execution_timestamp_ns": timestamp_ns,
                "generation": generation,
                "train_mask": False,
            },
        )

    def submit_if_needed() -> None:
        nonlocal retry_at
        if (
            not enabled
            or inference.busy
            or not queue.should_infer
            or time.monotonic() < retry_at
            or observation.image is None
            or observation.state is None
            or observation.base_state is None
        ):
            return
        _, pending = queue.begin_inference()
        model_observation = build_jiaxin_observation(
            observation.state,
            observation.base_state,
            observation.image.copy(),
            task,
            bgr=bgr,
            rotate_180=rotate,
        )
        if not inference.submit(model_observation, pending):
            queue.fail_inference(pending)

    try:
        for event in node:
            if event.get("type") != "INPUT":
                continue
            event_id = event["id"]
            try:
                if event_id == "image":
                    observation.image = _image(event["value"])
                    continue
                if event_id == "state":
                    observation.state = JointStateSerde.deserialize(event["value"])
                    continue
                if event_id == "base_state":
                    observation.base_state = event["value"].to_pylist()[0]
                    continue
                if event_id == "control":
                    control = json.loads(str(_scalar(event["value"])))
                    generation = int(control["generation"])
                    queue.reset(generation)
                    enabled, single_chunk = control_state(control["command"])
                    retry_at = 0.0
                    send_status("loading" if enabled else "paused")
                    continue
                if event_id != "tick":
                    continue

                result = inference.poll()
                if result is not None:
                    if result.error:
                        if queue.fail_inference(result.pending):
                            retry_at = time.monotonic() + retry_seconds
                            send_status("error", error=result.error)
                    else:
                        model_horizon = len(result.chunk)
                        chunk = limit_action_chunk(result.chunk, action_chunk_size)
                        consumed = queue.accept(chunk, result.pending)
                        if consumed is not None:
                            chunk_id += 1
                            last_info = {
                                **result.info,
                                "chunk_id": chunk_id,
                                "generation": generation,
                                "roundtrip_ms": result.latency_ms,
                                "inference_start_timestamp_ns": result.inference_start_timestamp_ns,
                                "inference_end_timestamp_ns": result.inference_end_timestamp_ns,
                                "actual_delay_steps": consumed,
                                "model_action_horizon": model_horizon,
                                "action_horizon": len(chunk),
                            }
                            send_status("ready", chunk_id=chunk_id)

                submit_if_needed()
                queued_step = queue.pop_with_index() if enabled else None
                if queued_step is None:
                    if enabled and inference.busy:
                        send_status("loading" if chunk_id == 0 else "starved")
                    continue
                chunk_step_index, step = queued_step

                timestamp_ns = time.time_ns()
                source_timestamp_ns = int(
                    last_info.get("inference_end_timestamp_ns", timestamp_ns)
                )
                metadata = {
                    "timestamp_ns": timestamp_ns,
                    "source_timestamp_ns": source_timestamp_ns,
                    "target_execution_timestamp_ns": timestamp_ns,
                    "predicted_execution_timestamp_ns": timestamp_ns,
                    "generation": generation,
                    "model_generation": generation,
                    "chunk_id": chunk_id,
                    "model_chunk_id": chunk_id,
                    "model_chunk_step_index": chunk_step_index,
                    "model_action_horizon": int(
                        last_info.get("model_action_horizon", action_chunk_size)
                    ),
                    "queue_depth": len(queue.actions),
                    "inference_start_timestamp_ns": int(
                        last_info.get("inference_start_timestamp_ns", source_timestamp_ns)
                    ),
                    "inference_end_timestamp_ns": source_timestamp_ns,
                    "model_action_valid": True,
                    "train_mask": False,
                    "schema_version": SCHEMA_VERSION,
                    "action_dim": ACTION_DIM,
                }
                joints = joint_action(step)
                base = base_action(step)
                action_robot = canonical_robot_action(joints, base)
                node.send_output("action", JointStateSerde.serialize(joints), metadata)
                node.send_output("base_action", JointStateSerde.serialize(base), metadata)
                node.send_output(
                    "action_robot",
                    JointStateSerde.serialize(action_robot),
                    metadata,
                )
                node.send_output(
                    "model_info",
                    pa.array([json.dumps({**last_info, "queue_depth": len(queue.actions)})]),
                    metadata,
                )
                if single_chunk and not queue.actions:
                    enabled = False
                    single_chunk = False
                    send_status("paused")
                else:
                    submit_if_needed()
            except Exception as exc:
                send_status("error", error=f"{type(exc).__name__}: {exc}")
    finally:
        rpc.close()


if __name__ == "__main__":
    main()
