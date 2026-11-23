from __future__ import annotations

from dataclasses import dataclass
import json
import os
import time
from typing import Any

import numpy as np

from client_server.ws.model_client import WsModelClient
from XPolicyLab.real_world.inference import AsyncPolicyInference
from XPolicyLab.real_world.rtc import (
    RealTimeChunkingConfig,
    RtcMode,
)
from XPolicyLab.real_world.robots.gr3.mapping import (
    BASE_ACTION_NAMES,
    GR3_ACTION_SCHEMA,
    JOINT_NAMES,
    action_dict,
    build_observation,
)
from XPolicyLab.real_world.robots.gr3.runtime import Gr3Runtime


@dataclass
class Observation:
    image: np.ndarray | None = None
    joints: dict[str, float] | None = None
    base_state: dict[str, Any] | None = None
    timestamp_ns: int = 0


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _image(value: Any) -> np.ndarray:
    flat = value.to_numpy(zero_copy_only=False)
    width = int(os.getenv("IMAGE_WIDTH", "640"))
    height = int(os.getenv("IMAGE_HEIGHT", "400"))
    channels = int(os.getenv("IMAGE_CHANNELS", "3"))
    return np.asarray(flat, dtype=np.uint8).reshape(height, width, channels)


def _deserialize_joints(value: Any) -> dict[str, float]:
    return {
        str(item["name"].as_py()): float(item["value"].as_py())
        for item in value
    }


def _serialize_joints(pa: Any, values: dict[str, float]) -> Any:
    return pa.StructArray.from_arrays(
        [pa.array(list(values)), pa.array(list(values.values()), type=pa.float32())],
        names=["name", "value"],
    )


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("GR3 Dora runtime requires dora-rs and pyarrow") from exc

    host = os.getenv("MODEL_HOST", "127.0.0.1")
    port = int(os.getenv("MODEL_PORT", "19000"))
    policy_url = os.getenv("POLICY_URL", f"ws://{host}:{port}")
    robot_id = os.getenv("ROBOT_ID", "115")
    task = os.getenv("TASK", "")
    retry_seconds = float(os.getenv("MODEL_RETRY_SECONDS", "0.5"))
    bgr = os.getenv("CAMERA_IS_BGR", "1") == "1"
    rotate_180 = os.getenv("GR00T_ROTATE_180", "0") == "1"
    model_state_dim = int(os.getenv("POLICY_STATE_DIM", "33"))
    if model_state_dim not in {31, 33}:
        raise ValueError("POLICY_STATE_DIM must be 31 or 33")

    node = Node()
    rtc_default = RealTimeChunkingConfig(
        mode=(
            RtcMode.ALIGN
            if os.getenv("INFERENCE_AUTO_PREFETCH", "1").strip().lower()
            in {"1", "true", "yes", "on"}
            else RtcMode.DISABLED
        ),
        latency_quantile=float(
            os.getenv("INFERENCE_PREFETCH_QUANTILE", "0.95")
        ),
        latency_margin_steps=int(
            os.getenv("INFERENCE_PREFETCH_MARGIN_STEPS", "1")
        ),
        latency_window_size=int(
            os.getenv("INFERENCE_PREFETCH_WINDOW_SIZE", "32")
        ),
    )
    runtime = Gr3Runtime(
        action_frequency_hz=float(os.getenv("ACTION_HZ", "30")),
        action_chunk_size=int(os.getenv("ACTION_CHUNK_SIZE", "16")),
        trigger_steps=int(os.getenv("INFERENCE_TRIGGER_STEPS", "0")),
        rtc_config=RealTimeChunkingConfig.from_environ(
            os.environ,
            default=rtc_default,
        ),
    )
    inference = AsyncPolicyInference(
        lambda: WsModelClient(
            url=policy_url,
            evaluation_id=f"gr3-{robot_id}",
            trial_id=f"gr3-{robot_id}",
        )
    )
    observation = Observation()
    generation = 0
    sequence = 0
    chunk_id = 0
    action_index = 0
    last_status: str | None = None
    last_info: dict[str, Any] = {}
    retry_at = 0.0

    def send_status(status: str, **detail: Any) -> None:
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
        nonlocal retry_at, sequence
        if (
            not runtime.should_infer
            or inference.busy
            or time.monotonic() < retry_at
            or observation.image is None
            or observation.joints is None
            or observation.base_state is None
        ):
            return
        frame = build_observation(
            observation.joints,
            observation.base_state,
            observation.image.copy(),
            task,
            timestamp_ns=observation.timestamp_ns or time.time_ns(),
            sequence=sequence,
            robot_id=robot_id,
            bgr=bgr,
            rotate_180=rotate_180,
            model_state_dim=model_state_dim,
            metadata={
                "inference_context": {
                    "action_index": action_index,
                    "queued_actions": (
                        np.stack(tuple(runtime.queue.actions))
                        if runtime.queue.actions
                        else None
                    ),
                }
            },
        )
        pending = runtime.begin_inference(
            observation_timestamp_ns=frame.timestamp_ns,
            submitted_timestamp_ns=time.time_ns(),
        )
        sequence += 1
        if not inference.submit(frame.to_policy_observation(), pending):
            runtime.fail_inference(pending)

    try:
        for event in node:
            if event.get("type") != "INPUT":
                continue
            event_id = event["id"]
            try:
                if event_id == "image":
                    observation.image = _image(event["value"])
                    metadata = event.get("metadata", {})
                    observation.timestamp_ns = int(
                        metadata.get("source_timestamp_ns")
                        or metadata.get("timestamp_ns")
                        or time.time_ns()
                    )
                    continue
                if event_id == "state":
                    observation.joints = _deserialize_joints(event["value"])
                    continue
                if event_id == "base_state":
                    observation.base_state = event["value"].to_pylist()[0]
                    continue
                if event_id == "control":
                    control = json.loads(str(_scalar(event["value"])))
                    generation = int(control["generation"])
                    runtime.control(str(control["command"]), generation)
                    action_index = 0
                    retry_at = 0.0
                    send_status("loading" if runtime.enabled else "paused")
                    continue
                if event_id != "tick":
                    continue

                result = inference.poll()
                if result is not None:
                    if result.error:
                        if runtime.fail_inference(result.pending):
                            retry_at = time.monotonic() + retry_seconds
                            send_status("error", error=result.error)
                    else:
                        assert result.chunk is not None
                        try:
                            consumed = runtime.accept(
                                result.chunk,
                                result.pending,
                                accepted_timestamp_ns=time.time_ns(),
                            )
                        except Exception:
                            runtime.fail_inference(result.pending)
                            retry_at = time.monotonic() + retry_seconds
                            raise
                        if consumed is not None:
                            alignment = runtime.queue.last_alignment
                            alignment_info = (
                                {} if alignment is None else alignment.as_dict()
                            )
                            if alignment is not None and not alignment.accepted:
                                send_status("stale_chunk", **alignment_info)
                            else:
                                chunk_id += 1
                                accepted = (
                                    np.stack(tuple(runtime.queue.actions)).astype(
                                        np.float32,
                                        copy=False,
                                    )
                                    if runtime.queue.actions
                                    else np.empty(
                                        (0, GR3_ACTION_SCHEMA.dim),
                                        dtype=np.float32,
                                    )
                                )
                                last_info = {
                                    **dict(result.chunk.metadata),
                                    "chunk_id": chunk_id,
                                    "generation": generation,
                                    "roundtrip_ms": result.roundtrip_ms,
                                    "inference_start_timestamp_ns": result.started_ns,
                                    "inference_end_timestamp_ns": result.ended_ns,
                                    "actual_delay_steps": consumed,
                                    **alignment_info,
                                    "model_action_horizon": len(result.chunk.values),
                                    "action_horizon": len(accepted),
                                }
                                chunk_log = {
                                    key: last_info[key]
                                    for key in (
                                        "chunk_id",
                                        "generation",
                                        "roundtrip_ms",
                                        "chunk_accepted",
                                        "aligned_offset_steps",
                                        "queue_consumed_steps",
                                        "observation_age_steps",
                                        "inference_delay_steps",
                                        "inference_trigger_steps",
                                        "queued_at_submit",
                                        "queue_depth_after_accept",
                                        "rtc_mode",
                                        "rtc_latency_quantile_steps",
                                        "rtc_merge_applied",
                                        "rtc_preserved_steps",
                                        "rtc_transition_steps",
                                        "rtc_transition_curve",
                                        "rtc_boundary_linf_before",
                                        "rtc_boundary_linf_after",
                                        "model_action_horizon",
                                        "action_horizon",
                                    )
                                    if key in last_info
                                }
                                print(
                                    "GR3_ACTION_CHUNK",
                                    json.dumps(chunk_log, sort_keys=True),
                                    flush=True,
                                )
                                if len(accepted):
                                    chunk_metadata = {
                                        "timestamp_ns": int(
                                            alignment_info.get(
                                                "chunk_accept_timestamp_ns",
                                                result.ended_ns,
                                            )
                                        ),
                                        "source_timestamp_ns": int(
                                            alignment_info.get(
                                                "observation_timestamp_ns",
                                                result.started_ns,
                                            )
                                        ),
                                        "generation": generation,
                                        "model_generation": generation,
                                        "chunk_id": chunk_id,
                                        "model_chunk_id": chunk_id,
                                        "model_action_horizon": len(
                                            result.chunk.values
                                        ),
                                        "native_action_horizon": len(accepted),
                                        "native_action_dim": len(JOINT_NAMES),
                                        "rtc_replacement": (
                                            result.pending.queued_at_submit > 0
                                        ),
                                        "rtc_actual_delay_steps": consumed,
                                        **alignment_info,
                                    }
                                    node.send_output(
                                        "action_chunk",
                                        pa.array(
                                            accepted[
                                                :, : len(JOINT_NAMES)
                                            ].reshape(-1),
                                            type=pa.float32(),
                                        ),
                                        chunk_metadata,
                                    )
                                send_status("ready", chunk_id=chunk_id)

                submit_if_needed()
                step = runtime.tick()
                if step is None:
                    if runtime.enabled and inference.busy:
                        send_status("loading" if chunk_id == 0 else "starved")
                    continue

                timestamp_ns = time.time_ns()
                action_index += 1
                source_timestamp_ns = int(
                    last_info.get("observation_timestamp_ns", timestamp_ns)
                )
                metadata = {
                    "timestamp_ns": timestamp_ns,
                    "source_timestamp_ns": source_timestamp_ns,
                    "target_execution_timestamp_ns": timestamp_ns,
                    "predicted_execution_timestamp_ns": timestamp_ns,
                    "generation": step.generation,
                    "model_generation": step.generation,
                    "chunk_id": chunk_id,
                    "model_chunk_id": chunk_id,
                    "model_chunk_step_index": step.chunk_step_index,
                    "model_action_horizon": int(
                        last_info.get(
                            "model_action_horizon",
                            runtime.action_chunk_size,
                        )
                    ),
                    "queue_depth": len(runtime.queue.actions),
                    "inference_start_timestamp_ns": int(
                        last_info.get(
                            "inference_start_timestamp_ns",
                            source_timestamp_ns,
                        )
                    ),
                    "inference_end_timestamp_ns": int(
                        last_info.get("inference_end_timestamp_ns", timestamp_ns)
                    ),
                    "model_action_valid": True,
                    "train_mask": False,
                    "schema_version": GR3_ACTION_SCHEMA.schema_id,
                    "action_dim": GR3_ACTION_SCHEMA.dim,
                }
                robot_action = action_dict(step.values)
                joints = {name: robot_action[name] for name in JOINT_NAMES}
                base = {name: robot_action[name] for name in BASE_ACTION_NAMES}
                node.send_output("action", _serialize_joints(pa, joints), metadata)
                node.send_output("base_action", _serialize_joints(pa, base), metadata)
                node.send_output(
                    "action_robot",
                    _serialize_joints(pa, robot_action),
                    metadata,
                )
                node.send_output(
                    "model_info",
                    pa.array(
                        [
                            json.dumps(
                                {
                                    **last_info,
                                    "queue_depth": len(runtime.queue.actions),
                                }
                            )
                        ]
                    ),
                    metadata,
                )
                if not runtime.enabled:
                    send_status("paused")
                else:
                    submit_if_needed()
            except Exception as exc:
                send_status("error", error=f"{type(exc).__name__}: {exc}")
    finally:
        inference.close()


if __name__ == "__main__":
    main()
