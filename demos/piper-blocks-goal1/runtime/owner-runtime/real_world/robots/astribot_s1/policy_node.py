from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import time
from typing import Any

import numpy as np
import yaml

from XPolicyLab.real_world.contracts import ObservationFrame
from XPolicyLab.real_world.inference import (
    ActionChunkPolicyRuntime,
    ActionChunkRuntime,
    AsyncPolicyInference,
)
from XPolicyLab.real_world.rtc import (
    RealTimeChunkingConfig,
    RtcMode,
)
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
    ASTRIBOT_S1_STATE_SCHEMA,
)


DEFAULT_PROFILE = Path(__file__).with_name("profile.yml")
CAMERA_NAMES = ("cam_head", "cam_left_wrist", "cam_right_wrist")


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _named_vector(value: Any, fields: tuple[str, ...]) -> np.ndarray:
    named = {
        str(item["name"].as_py()): float(item["value"].as_py())
        for item in value
    }
    if set(named) != set(fields):
        raise ValueError("Astribot policy state does not match its declared schema")
    return np.asarray([named[name] for name in fields], dtype=np.float32)


def _named_vector_array(pa: Any, fields: tuple[str, ...], values: Any) -> Any:
    return pa.StructArray.from_arrays(
        [pa.array(fields), pa.array(values, type=pa.float32())],
        names=["name", "value"],
    )


def _image(value: Any, metadata: dict[str, Any]) -> np.ndarray:
    width = int(metadata["width"])
    height = int(metadata["height"])
    return np.asarray(value.to_numpy(zero_copy_only=False), dtype=np.uint8).reshape(
        height,
        width,
        3,
    )


@dataclass
class AstribotPolicyObservationAdapter:
    instruction: str
    robot_id: str
    state: np.ndarray | None = None
    images: dict[str, np.ndarray] = field(default_factory=dict)
    timestamp_ns: int = 0
    sequence: int = 0

    def update_state(self, value: Any, metadata: dict[str, Any]) -> None:
        self.state = _named_vector(value, ASTRIBOT_S1_STATE_SCHEMA.fields)
        self.timestamp_ns = int(
            metadata.get("source_timestamp_ns")
            or metadata.get("timestamp_ns")
            or time.time_ns()
        )
        self.sequence = int(metadata.get("sequence", self.sequence))

    def update_image(
        self,
        camera_name: str,
        value: Any,
        metadata: dict[str, Any],
    ) -> None:
        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"unsupported Astribot camera: {camera_name}")
        self.images[camera_name] = _image(value, metadata)

    def build(self, queue: Any) -> ObservationFrame | None:
        if self.state is None or any(name not in self.images for name in CAMERA_NAMES):
            return None
        queued = (
            np.stack(tuple(queue.actions))
            if queue.actions
            else None
        )
        return ObservationFrame(
            state_schema=ASTRIBOT_S1_STATE_SCHEMA,
            state=self.state,
            images={name: self.images[name] for name in CAMERA_NAMES},
            instruction=self.instruction,
            timestamp_ns=self.timestamp_ns,
            sequence=self.sequence,
            robot_type="astribot_s1",
            robot_id=self.robot_id,
            metadata={
                "inference_context": {
                    "queued_actions": queued,
                    "execution_step": queue.execution_step,
                }
            },
        )


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
        from client_server.ws.model_client import WsModelClient
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Astribot policy node requires dora-rs, pyarrow and the policy client"
        ) from exc

    profile_path = Path(os.getenv("ASTRIBOT_PROFILE", str(DEFAULT_PROFILE))).resolve()
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    runtime_profile = profile["runtime"]
    policy_url = os.getenv("POLICY_URL", "ws://127.0.0.1:19001")
    robot_id = os.getenv("ROBOT_ID", "astribot-s1")
    instruction = os.getenv("TASK", "pick the tomato")
    node = Node(os.getenv("DORA_NODE_ID", "astribot_policy"))

    rtc_default = RealTimeChunkingConfig(
        mode=(
            RtcMode.ALIGN
            if bool(runtime_profile.get("auto_prefetch", True))
            else RtcMode.DISABLED
        ),
        latency_quantile=float(runtime_profile.get("prefetch_quantile", 0.95)),
        latency_margin_steps=int(
            runtime_profile.get("prefetch_margin_steps", 1)
        ),
        latency_window_size=int(
            runtime_profile.get("prefetch_window_size", 32)
        ),
    )
    scheduler = ActionChunkRuntime(
        ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
        action_frequency_hz=float(profile["action_frequency_hz"]),
        action_chunk_size=int(runtime_profile.get("action_chunk_size", 50)),
        trigger_steps=int(runtime_profile.get("trigger_steps", 0)),
        rtc_config=RealTimeChunkingConfig.from_environ(
            os.environ,
            default=rtc_default,
        ),
    )
    policy = ActionChunkPolicyRuntime(
        scheduler,
        AsyncPolicyInference(
            lambda: WsModelClient(
                url=policy_url,
                evaluation_id=f"astribot-policy-{robot_id}",
                trial_id=f"astribot-policy-{robot_id}",
            )
        ),
        retry_seconds=float(os.getenv("MODEL_RETRY_SECONDS", "0.5")),
    )
    observation = AstribotPolicyObservationAdapter(instruction, robot_id)
    generation = 0
    chunk_id = 0
    last_status = ""
    pause_pending = False

    def send_status(status: str, **detail: Any) -> None:
        nonlocal last_status
        payload = {"status": status, "generation": generation, **detail}
        signature = json.dumps(payload, sort_keys=True, default=str)
        if signature == last_status:
            return
        last_status = signature
        now_ns = time.time_ns()
        node.send_output(
            "status",
            pa.array([signature]),
            {
                "timestamp_ns": now_ns,
                "source_timestamp_ns": now_ns,
                "generation": generation,
            },
        )

    try:
        for event in node:
            if event is None or event.get("type") == "STOP":
                break
            if event.get("type") != "INPUT":
                continue
            event_id = str(event["id"])
            metadata = dict(event.get("metadata", {}))
            if event_id == "state_robot":
                observation.update_state(event["value"], metadata)
                continue
            if event_id.startswith("image_"):
                observation.update_image(
                    event_id.removeprefix("image_"),
                    event["value"],
                    metadata,
                )
                continue
            if event_id == "control":
                control = json.loads(str(_scalar(event["value"])))
                generation = int(control["generation"])
                policy.control(str(control["command"]), generation)
                pause_pending = False
                send_status("loading" if scheduler.enabled else "paused")
                continue
            if event_id != "tick":
                continue
            if pause_pending:
                pause_pending = False
                send_status("paused", model_chunk_id=chunk_id)
                continue

            frame = observation.build(scheduler.queue)
            tick = policy.tick(
                None if frame is None else frame.to_policy_observation(),
                observation_timestamp_ns=0 if frame is None else frame.timestamp_ns,
                accepted_timestamp_ns=time.time_ns(),
            )
            result = tick.inference
            if result is not None:
                if result.error is not None:
                    send_status("error", error=result.error)
                elif result.chunk is not None:
                    alignment = scheduler.queue.last_alignment
                    alignment_info = {} if alignment is None else alignment.as_dict()
                    if alignment is not None and alignment.accepted:
                        chunk_id += 1
                        info = {
                            "model_chunk_id": chunk_id,
                            "model_action_horizon": len(result.chunk.values),
                            "native_action_horizon": min(
                                len(result.chunk.values),
                                scheduler.action_chunk_size or len(result.chunk.values),
                            ),
                            "roundtrip_ms": result.roundtrip_ms,
                            "inference_start_timestamp_ns": result.started_ns,
                            "inference_end_timestamp_ns": result.ended_ns,
                            **alignment_info,
                        }
                        node.send_output(
                            "model_info",
                            pa.array([json.dumps(info, sort_keys=True)]),
                            {
                                "timestamp_ns": result.ended_ns,
                                "source_timestamp_ns": observation.timestamp_ns,
                                "generation": generation,
                                **info,
                            },
                        )
                        print(
                            "ASTRIBOT_ACTION_CHUNK",
                            json.dumps(info, sort_keys=True),
                            flush=True,
                        )

            if tick.step is not None:
                now_ns = time.time_ns()
                node.send_output(
                    "action_robot",
                    _named_vector_array(
                        pa,
                        ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields,
                        tick.step.values,
                    ),
                    {
                        "timestamp_ns": now_ns,
                        "source_timestamp_ns": observation.timestamp_ns,
                        "generation": tick.step.generation,
                        "model_generation": tick.step.generation,
                        "model_chunk_id": chunk_id,
                        "model_chunk_step_index": tick.step.chunk_step_index,
                        "queue_depth": len(scheduler.queue.actions),
                    },
                )
                send_status("ready", model_chunk_id=chunk_id)
                pause_pending = not scheduler.enabled
            elif scheduler.enabled and policy.inference.busy:
                send_status("loading" if chunk_id == 0 else "starved")
    finally:
        policy.close()


if __name__ == "__main__":
    main()
