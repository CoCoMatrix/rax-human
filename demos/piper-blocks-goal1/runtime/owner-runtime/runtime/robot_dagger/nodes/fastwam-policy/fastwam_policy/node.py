from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from farther_core.gr3_dagger import (
    ACTION_DIM as ROBOT_ACTION_DIM,
    SCHEMA_VERSION,
    canonical_robot_action,
)
from farther_core.serde import JointStateSerde

from .client import AsyncInference, FastWamInferenceClient
from .mapping import JOINT_NAMES, base_action, build_fastwam_observation, joint_action


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


def limit_action_chunk(
    chunk: np.ndarray,
    size: int,
    *,
    require_exact: bool = False,
) -> np.ndarray:
    if size < 1:
        raise ValueError("ACTION_CHUNK_SIZE must be positive")
    if require_exact and len(chunk) != size:
        raise ValueError(
            f"FastWAM RTC must return exactly {size} actions, got {len(chunk)}"
        )
    return chunk[:size]


def replace_rtc_chunk(
    chunk: np.ndarray,
    *,
    action_index_before_inference: int,
    action_index_now: int,
) -> tuple[np.ndarray, int]:
    delay = max(0, action_index_now - action_index_before_inference)
    return chunk[min(delay, len(chunk)) :], delay


def main() -> None:
    host = os.getenv("FASTWAM_HOST", "127.0.0.1")
    port = int(os.getenv("FASTWAM_PORT", "5555"))
    task = os.getenv(
        "TASK",
        "Pick the tomato from the table and place it inside the basket on the left.",
    )
    retry_seconds = float(os.getenv("MODEL_RETRY_SECONDS", "0.5"))
    bgr = os.getenv("CAMERA_IS_BGR", "1") == "1"
    action_chunk_size = int(os.getenv("ACTION_CHUNK_SIZE", "32"))
    rtc_setting = os.getenv("FASTWAM_RTC_ENABLED", "0")
    if rtc_setting not in {"0", "1"}:
        raise ValueError("FASTWAM_RTC_ENABLED must be 0 or 1")
    rtc_enabled = rtc_setting == "1"
    rtc_prefix_steps = int(os.getenv("FASTWAM_RTC_PREFIX_STEPS", "16"))
    rtc_initial_delay = int(os.getenv("FASTWAM_RTC_INFERENCE_DELAY_STEPS", "10"))
    rtc_schedule = os.getenv("FASTWAM_RTC_PREFIX_ATTENTION_SCHEDULE", "exp").lower()
    rtc_max_guidance_weight = float(
        os.getenv("FASTWAM_RTC_MAX_GUIDANCE_WEIGHT", "5.0")
    )
    if action_chunk_size < 1:
        raise ValueError("ACTION_CHUNK_SIZE must be positive")
    if rtc_enabled and not 0 < rtc_prefix_steps < action_chunk_size:
        raise ValueError(
            "RTC requires 0 < FASTWAM_RTC_PREFIX_STEPS < ACTION_CHUNK_SIZE"
        )
    if rtc_enabled and action_chunk_size != 32:
        raise ValueError("FastWAM RTC requires ACTION_CHUNK_SIZE=32")
    if rtc_enabled and rtc_prefix_steps != 16:
        raise ValueError("FastWAM RTC requires FASTWAM_RTC_PREFIX_STEPS=16")
    if rtc_initial_delay < 0:
        raise ValueError("FASTWAM_RTC_INFERENCE_DELAY_STEPS must be non-negative")
    if rtc_schedule not in {"exp", "linear", "ones", "zeros"}:
        raise ValueError(
            "FASTWAM_RTC_PREFIX_ATTENTION_SCHEDULE must be exp/linear/ones/zeros"
        )
    if rtc_max_guidance_weight < 0:
        raise ValueError("FASTWAM_RTC_MAX_GUIDANCE_WEIGHT must be non-negative")

    node = Node()
    rpc = FastWamInferenceClient(
        host, port, int(os.getenv("FASTWAM_TIMEOUT_MS", "600000"))
    )
    inference = AsyncInference(rpc)
    observation = Observation()
    actions: deque[np.ndarray] = deque()
    enabled = False
    single_chunk = False
    reset_before_next = True
    generation = 0
    chunk_id = 0
    last_status = None
    last_info: dict = {}
    retry_at = 0.0
    action_index = 0
    observed_delays: deque[int] = deque([rtc_initial_delay], maxlen=10)

    def send_status(status: str, **detail) -> None:
        nonlocal last_status
        payload = {"status": status, "generation": generation, **detail}
        signature = json.dumps(payload, sort_keys=True)
        if signature == last_status:
            return
        last_status = signature
        node.send_output("status", pa.array([signature]), {"generation": generation})

    def submit_if_needed(*, allow_rtc: bool) -> None:
        nonlocal retry_at
        if (
            not enabled
            or inference.busy
            or time.monotonic() < retry_at
            or observation.image is None
            or observation.state is None
            or observation.base_state is None
        ):
            return
        prefix = None
        if actions:
            if (
                not allow_rtc
                or not rtc_enabled
                or single_chunk
                or len(actions) > rtc_prefix_steps
            ):
                return
            prefix = np.stack(tuple(actions), axis=0)
        model_observation = build_fastwam_observation(
            observation.state,
            observation.base_state,
            observation.image.copy(),
            task,
            bgr=bgr,
        )
        inference.submit(
            model_observation,
            generation,
            reset=reset_before_next,
            action_index=action_index,
            rtc_prefix=prefix,
            rtc_inference_delay=max(observed_delays),
            rtc_prefix_horizon=rtc_prefix_steps,
            rtc_prefix_attention_schedule=rtc_schedule,
            rtc_max_guidance_weight=rtc_max_guidance_weight,
        )

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
                    command = str(control["command"])
                    generation = int(control["generation"])
                    actions.clear()
                    observed_delays.clear()
                    observed_delays.append(rtc_initial_delay)
                    action_index = 0
                    enabled, single_chunk = control_state(command)
                    if command in {"bootstrap", "pause"}:
                        reset_before_next = True
                    retry_at = 0.0
                    send_status("loading" if enabled else "paused")
                    continue
                if event_id != "tick":
                    continue

                result = inference.poll()
                if result is not None and result.generation == generation and enabled:
                    if result.error:
                        retry_at = time.monotonic() + retry_seconds
                        send_status("error", error=result.error)
                    else:
                        model_horizon = len(result.chunk)
                        chunk = limit_action_chunk(
                            result.chunk,
                            action_chunk_size,
                            require_exact=rtc_enabled,
                        )
                        actual_delay = 0
                        rtc_replacement = bool(result.rtc_prefix_steps)
                        if rtc_replacement:
                            chunk, actual_delay = replace_rtc_chunk(
                                chunk,
                                action_index_before_inference=result.action_index,
                                action_index_now=action_index,
                            )
                            actions.clear()
                            observed_delays.append(actual_delay)
                        actions.extend(row.copy() for row in chunk)
                        reset_before_next = False
                        chunk_id += 1
                        inference_end_ns = time.time_ns()
                        last_info = {
                            **result.info,
                            "backend": "fastwam",
                            "chunk_id": chunk_id,
                            "generation": generation,
                            "roundtrip_ms": result.latency_ms,
                            "inference_start_timestamp_ns": inference_end_ns
                            - int(result.latency_ms * 1e6),
                            "inference_end_timestamp_ns": inference_end_ns,
                            "model_action_horizon": model_horizon,
                            "action_horizon": len(chunk),
                            "native_action_horizon": len(chunk),
                            "rtc_enabled": rtc_enabled,
                            "rtc_request": rtc_replacement,
                            "rtc_prefix_steps": result.rtc_prefix_steps,
                            "rtc_actual_delay_steps": actual_delay,
                            "rtc_predicted_delay_steps": max(observed_delays),
                        }
                        chunk_metadata = {
                            "timestamp_ns": inference_end_ns,
                            "source_timestamp_ns": inference_end_ns,
                            "generation": generation,
                            "model_generation": generation,
                            "chunk_id": chunk_id,
                            "model_chunk_id": chunk_id,
                            "model_action_horizon": model_horizon,
                            "native_action_horizon": len(chunk),
                            "native_action_dim": len(JOINT_NAMES),
                            "rtc_replacement": rtc_replacement,
                            "rtc_enabled": rtc_enabled,
                        }
                        node.send_output(
                            "action_chunk",
                            pa.array(chunk.reshape(-1), type=pa.float32()),
                            chunk_metadata,
                        )
                        print(
                            "FASTWAM_INFERENCE",
                            json.dumps(last_info, sort_keys=True),
                            flush=True,
                        )
                        send_status("ready", chunk_id=chunk_id)

                submit_if_needed(allow_rtc=False)
                raw_step = actions.popleft() if enabled and actions else None
                if raw_step is None:
                    if enabled and inference.busy:
                        send_status("loading" if chunk_id == 0 else "starved")
                    continue

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
                    "model_chunk_step_index": max(
                        0,
                        int(last_info.get("action_horizon", len(actions) + 1))
                        - len(actions)
                        - 1,
                    ),
                    "model_action_horizon": int(
                        last_info.get("model_action_horizon", len(actions) + 1)
                    ),
                    "queue_depth": len(actions),
                    "inference_start_timestamp_ns": int(
                        last_info.get("inference_start_timestamp_ns", source_timestamp_ns)
                    ),
                    "inference_end_timestamp_ns": source_timestamp_ns,
                    "model_action_valid": True,
                    "train_mask": False,
                    "schema_version": SCHEMA_VERSION,
                    "action_dim": ROBOT_ACTION_DIM,
                }
                joints = joint_action(raw_step)
                base = base_action()
                node.send_output("action", JointStateSerde.serialize(joints), metadata)
                node.send_output(
                    "base_action", JointStateSerde.serialize(base), metadata
                )
                node.send_output(
                    "action_robot",
                    JointStateSerde.serialize(canonical_robot_action(joints, base)),
                    metadata,
                )
                node.send_output(
                    "model_info",
                    pa.array(
                        [
                            json.dumps(
                                {
                                    **last_info,
                                    "queue_depth": len(actions),
                                }
                            )
                        ]
                    ),
                    metadata,
                )
                action_index += 1
                if single_chunk and not actions:
                    enabled = False
                    single_chunk = False
                    send_status("paused")
                else:
                    submit_if_needed(allow_rtc=True)
            except Exception as exc:
                send_status("error", error=f"{type(exc).__name__}: {exc}")
    finally:
        rpc.close()


if __name__ == "__main__":
    main()
