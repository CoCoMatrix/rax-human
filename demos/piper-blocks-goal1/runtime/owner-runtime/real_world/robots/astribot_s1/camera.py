from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
import threading
import time
from types import MappingProxyType
from typing import Any

import numpy as np
import yaml

from XPolicyLab.real_world.contracts import ObservationFrame
from XPolicyLab.real_world.robots.astribot_s1.mapping import build_observation
from XPolicyLab.real_world.robots.astribot_s1.sdk import (
    AstribotS1SdkSession,
    RobotSnapshot,
)


CAMERA_STREAMS = MappingProxyType(
    {
        "cam_head": "head_rgbd",
        "cam_left_wrist": "left_wrist_rgbd",
        "cam_right_wrist": "right_wrist_rgbd",
    }
)


def _positive_ns(milliseconds: float, name: str) -> int:
    value = float(milliseconds)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive")
    return int(value * 1_000_000)


def _message_timestamp_ns(message: Any, fallback_ns: int) -> tuple[int, bool]:
    try:
        stamp = message.header.stamp
        timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        timestamp_ns = 0
    return (timestamp_ns, False) if timestamp_ns > 0 else (fallback_ns, True)


def _sdk_camera_from_topic(topic_name: str) -> str:
    parts = str(topic_name).strip("/").split("/")
    if len(parts) < 3 or parts[0] != "astribot_camera":
        raise ValueError(f"invalid Astribot camera topic: {topic_name!r}")
    return parts[1]


@dataclass(frozen=True, slots=True)
class CameraFrame:
    policy_name: str
    sdk_name: str
    topic_name: str
    image: np.ndarray
    source_timestamp_ns: int
    received_timestamp_ns: int
    sequence: int
    source_timestamp_fallback: bool = False
    encoding: str = "bgr8"

    def __post_init__(self) -> None:
        source = np.asarray(self.image)
        if source.dtype != np.uint8:
            raise ValueError("Astribot color image must use uint8 pixels")
        image = np.array(source, copy=True)
        if image.ndim != 3 or image.shape[2] != 3 or not image.size:
            raise ValueError("Astribot color image must be non-empty HxWx3")
        if self.source_timestamp_ns <= 0 or self.received_timestamp_ns <= 0:
            raise ValueError("camera timestamps must be positive")
        image.setflags(write=False)
        object.__setattr__(self, "image", image)


@dataclass(frozen=True, slots=True)
class CameraSyncDecision:
    status: str
    reasons: tuple[str, ...]
    frames: Mapping[str, CameraFrame]
    state_timestamp_ns: int
    reference_timestamp_ns: int = 0
    max_camera_skew_ns: int = 0
    max_state_skew_ns: int = 0
    camera_clock_offset_ns: int = 0
    aligned_timestamps_ns: Mapping[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "frames", MappingProxyType(dict(self.frames)))
        object.__setattr__(
            self,
            "aligned_timestamps_ns",
            MappingProxyType(dict(self.aligned_timestamps_ns)),
        )

    @property
    def ready(self) -> bool:
        return self.status == "READY"


class AstribotS1CameraSource:
    """Three-camera SDK callback cache with state-time nearest selection."""

    def __init__(
        self,
        session: AstribotS1SdkSession,
        *,
        camera_streams: Mapping[str, str] = CAMERA_STREAMS,
        history_size: int = 16,
        max_frame_age_ms: float = 250.0,
        max_camera_skew_ms: float = 50.0,
        max_state_skew_ms: float = 100.0,
    ) -> None:
        if history_size < 2:
            raise ValueError("camera history_size must be at least 2")
        streams = {str(policy): str(sdk) for policy, sdk in camera_streams.items()}
        if not streams or len(streams) != len(set(streams.values())):
            raise ValueError("camera streams must be non-empty and unique")
        self.session = session
        self.camera_streams = MappingProxyType(streams)
        self.sdk_to_policy = {sdk: policy for policy, sdk in streams.items()}
        self.history_size = int(history_size)
        self.max_frame_age_ns = _positive_ns(max_frame_age_ms, "max_frame_age_ms")
        self.max_camera_skew_ns = _positive_ns(
            max_camera_skew_ms, "max_camera_skew_ms"
        )
        self.max_state_skew_ns = _positive_ns(
            max_state_skew_ms, "max_state_skew_ms"
        )
        self._frames = {
            policy: deque(maxlen=self.history_size) for policy in self.camera_streams
        }
        self._sequences = dict.fromkeys(self.camera_streams, 0)
        self._subscriptions: list[Any] = []
        self._lock = threading.Lock()
        self._started = False

    @classmethod
    def from_profile(
        cls,
        path: str | Path,
        session: AstribotS1SdkSession,
    ) -> AstribotS1CameraSource:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        cameras = profile["cameras"]
        return cls(
            session,
            camera_streams=cameras["streams"],
            history_size=cameras["history_size"],
            max_frame_age_ms=cameras["max_frame_age_ms"],
            max_camera_skew_ms=cameras["max_camera_skew_ms"],
            max_state_skew_ms=cameras["max_state_skew_ms"],
        )

    def start(self, *, activate: bool = False) -> dict[str, Any]:
        if self._started:
            raise RuntimeError("Astribot camera source is already started")
        if activate and self.session.activate_cameras() is False:
            raise RuntimeError("Astribot SDK failed to activate cameras")
        camera_info = self.session.get_cameras_info()
        try:
            # The vendor interface stores one user callback slot globally. Use
            # the same dispatcher for every subscription and route by topic.
            for sdk_name in self.camera_streams.values():
                subscription = self.session.register_image_callback(
                    sdk_name,
                    "color",
                    self._image_callback,
                    need_decode=True,
                )
                if subscription is None:
                    raise RuntimeError(f"failed to subscribe to {sdk_name}")
                self._subscriptions.append(subscription)
        except Exception:
            self.close()
            raise
        self._started = True
        return dict(camera_info)

    def close(self) -> None:
        for subscription in reversed(self._subscriptions):
            try:
                self.session.unregister_image_callback(subscription)
            except Exception:
                pass
        self._subscriptions.clear()
        self._started = False

    def _image_callback(
        self,
        topic_name: str,
        message: Any,
        width: int,
        height: int,
        image: np.ndarray,
    ) -> None:
        received_ns = time.time_ns()
        try:
            sdk_name = _sdk_camera_from_topic(topic_name)
            policy_name = self.sdk_to_policy[sdk_name]
            array = np.asarray(image)
            if (
                array.dtype != np.uint8
                or array.ndim != 3
                or array.shape[2] != 3
                or int(width) != array.shape[1]
                or int(height) != array.shape[0]
            ):
                raise ValueError("decoded camera shape does not match SDK metadata")
            timestamp_ns, fallback = _message_timestamp_ns(message, received_ns)
            with self._lock:
                sequence = self._sequences[policy_name]
                self._sequences[policy_name] = sequence + 1
                self._frames[policy_name].append(
                    CameraFrame(
                        policy_name=policy_name,
                        sdk_name=sdk_name,
                        topic_name=str(topic_name),
                        image=array,
                        source_timestamp_ns=timestamp_ns,
                        received_timestamp_ns=received_ns,
                        sequence=sequence,
                        source_timestamp_fallback=fallback,
                    )
                )
        except (KeyError, TypeError, ValueError):
            return

    def synchronize(
        self,
        *,
        state_timestamp_ns: int,
        now_ns: int | None = None,
    ) -> CameraSyncDecision:
        now_ns = time.time_ns() if now_ns is None else int(now_ns)
        state_timestamp_ns = int(state_timestamp_ns)
        if state_timestamp_ns <= 0 or now_ns < state_timestamp_ns:
            return CameraSyncDecision(
                "HOLD", ("invalid_state_timestamp",), {}, state_timestamp_ns
            )
        with self._lock:
            history = {name: tuple(frames) for name, frames in self._frames.items()}
        missing = tuple(name for name, frames in history.items() if not frames)
        if missing:
            return CameraSyncDecision(
                "HOLD",
                tuple(f"camera_missing:{name}" for name in missing),
                {},
                state_timestamp_ns,
            )
        # Camera headers use the camera computer's clock, while RobotSnapshot
        # is timestamped on this computer after the SDK read. Estimate their
        # common offset from the latest frame of each stream. Keeping one
        # shared offset preserves the inter-camera source timing.
        offset_samples = [
            frames[-1].source_timestamp_ns - frames[-1].received_timestamp_ns
            for frames in history.values()
            if not frames[-1].source_timestamp_fallback
        ]
        camera_clock_offset_ns = (
            int(np.median(offset_samples)) if offset_samples else 0
        )

        def aligned_timestamp(frame: CameraFrame) -> int:
            if frame.source_timestamp_fallback:
                return frame.received_timestamp_ns
            return frame.source_timestamp_ns - camera_clock_offset_ns

        fresh_history = {
            name: tuple(
                frame
                for frame in frames
                if 0 <= now_ns - frame.received_timestamp_ns <= self.max_frame_age_ns
            )
            for name, frames in history.items()
        }
        stale = tuple(
            name for name, frames in fresh_history.items() if not frames
        )
        if stale:
            selected = {name: frames[-1] for name, frames in history.items()}
            return CameraSyncDecision(
                "HOLD",
                tuple(f"camera_stale:{name}" for name in stale),
                selected,
                state_timestamp_ns,
                camera_clock_offset_ns=camera_clock_offset_ns,
                aligned_timestamps_ns={
                    name: aligned_timestamp(frame)
                    for name, frame in selected.items()
                },
            )

        def selection_metrics(
            frames: Mapping[str, CameraFrame],
        ) -> tuple[dict[str, int], int, int]:
            timestamps = {
                name: aligned_timestamp(frame) for name, frame in frames.items()
            }
            values = list(timestamps.values())
            camera_skew = max(values) - min(values)
            state_skew = max(
                abs(value - state_timestamp_ns) for value in values
            )
            return timestamps, camera_skew, state_skew

        # Choose a coherent capture set instead of independently choosing each
        # camera's state-nearest frame, which can mix adjacent 30 Hz cycles.
        candidates = []
        for reference in (
            frame for frames in fresh_history.values() for frame in frames
        ):
            reference_ns = aligned_timestamp(reference)
            candidate = {
                name: min(
                    frames,
                    key=lambda frame: abs(
                        aligned_timestamp(frame) - reference_ns
                    ),
                )
                for name, frames in fresh_history.items()
            }
            timestamps, camera_skew, state_skew = selection_metrics(candidate)
            within_limits = (
                camera_skew <= self.max_camera_skew_ns
                and state_skew <= self.max_state_skew_ns
            )
            score = (
                0 if within_limits else 1,
                (
                    float(state_skew)
                    if within_limits
                    else max(
                        camera_skew / self.max_camera_skew_ns,
                        state_skew / self.max_state_skew_ns,
                    )
                ),
                float(camera_skew),
            )
            candidates.append(
                (score, candidate, timestamps, camera_skew, state_skew)
            )

        _, selected, aligned_timestamps, camera_skew_ns, state_skew_ns = min(
            candidates,
            key=lambda item: item[0],
        )
        timestamps = list(aligned_timestamps.values())
        reasons = []
        if camera_skew_ns > self.max_camera_skew_ns:
            reasons.append("camera_skew_exceeded")
        if state_skew_ns > self.max_state_skew_ns:
            reasons.append("camera_state_skew_exceeded")
        return CameraSyncDecision(
            "HOLD" if reasons else "READY",
            tuple(reasons),
            selected,
            state_timestamp_ns,
            reference_timestamp_ns=max(timestamps),
            max_camera_skew_ns=camera_skew_ns,
            max_state_skew_ns=state_skew_ns,
            camera_clock_offset_ns=camera_clock_offset_ns,
            aligned_timestamps_ns=aligned_timestamps,
        )


class AstribotS1ObservationAdapter:
    """Build one RGB/25D policy observation from a synchronized camera set."""

    def __init__(self, camera_source: AstribotS1CameraSource) -> None:
        self.camera_source = camera_source

    def build(
        self,
        snapshot: RobotSnapshot,
        instruction: str,
        *,
        sequence: int,
        robot_id: str,
        now_ns: int | None = None,
    ) -> tuple[ObservationFrame | None, CameraSyncDecision]:
        decision = self.camera_source.synchronize(
            state_timestamp_ns=snapshot.timestamp_ns,
            now_ns=now_ns,
        )
        if not decision.ready:
            return None, decision
        images = {
            name: np.ascontiguousarray(frame.image[..., ::-1])
            for name, frame in decision.frames.items()
        }
        camera_metadata = {
            name: {
                "sdk_name": frame.sdk_name,
                "source_timestamp_ns": frame.source_timestamp_ns,
                "aligned_timestamp_ns": decision.aligned_timestamps_ns[name],
                "received_timestamp_ns": frame.received_timestamp_ns,
                "sequence": frame.sequence,
                "source_timestamp_fallback": frame.source_timestamp_fallback,
                "source_encoding": frame.encoding,
                "observation_encoding": "rgb8",
                "shape": list(frame.image.shape),
            }
            for name, frame in decision.frames.items()
        }
        observation = build_observation(
            snapshot.joints,
            images,
            instruction,
            timestamp_ns=snapshot.timestamp_ns,
            sequence=sequence,
            robot_id=robot_id,
            metadata={
                "camera_sync": {
                    "reference_timestamp_ns": decision.reference_timestamp_ns,
                    "camera_clock_offset_ns": decision.camera_clock_offset_ns,
                    "max_camera_skew_ns": decision.max_camera_skew_ns,
                    "max_state_skew_ns": decision.max_state_skew_ns,
                    "cameras": camera_metadata,
                }
            },
        )
        return observation, decision
