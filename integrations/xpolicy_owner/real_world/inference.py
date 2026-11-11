from __future__ import annotations

from collections import deque
from collections.abc import Callable, Mapping
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, replace
import json
import time
from types import MappingProxyType
from typing import Any

import numpy as np

from XPolicyLab.real_world.contracts import ActionChunk, VectorSchema
from XPolicyLab.real_world.action_timebase import ActionTimebase
from XPolicyLab.real_world.rtc import (
    ChunkMerge,
    RealTimeChunker,
    RealTimeChunkingConfig,
    RtcMode,
)


@dataclass(frozen=True, slots=True)
class PendingChunk:
    generation: int
    queued_at_submit: int
    observation_timestamp_ns: int = 0
    submitted_timestamp_ns: int = 0
    execution_step_at_submit: int = 0


@dataclass(frozen=True, slots=True)
class ChunkAlignment:
    """Evidence used to align a returned chunk to the execution timeline."""

    accepted: bool
    horizon_steps: int
    queued_at_submit: int
    queue_consumed_steps: int
    observation_age_steps: int
    aligned_offset_steps: int
    inference_delay_steps: int
    trigger_steps: int
    queue_depth_after: int
    observation_timestamp_ns: int
    submitted_timestamp_ns: int
    accepted_timestamp_ns: int
    rtc_mode: str = RtcMode.DISABLED.value
    rtc_latency_quantile_steps: int = 0
    rtc_merge_applied: bool = False
    rtc_previous_steps_at_accept: int = 0
    rtc_preserved_steps: int = 0
    rtc_transition_steps: int = 0
    rtc_transition_curve: str = "smoothstep"
    rtc_boundary_linf_before: float | None = None
    rtc_boundary_linf_after: float | None = None

    def as_dict(self) -> dict[str, int | float | str | bool | None]:
        return {
            "chunk_accepted": self.accepted,
            "scheduler_horizon_steps": self.horizon_steps,
            "queued_at_submit": self.queued_at_submit,
            "queue_consumed_steps": self.queue_consumed_steps,
            "observation_age_steps": self.observation_age_steps,
            "aligned_offset_steps": self.aligned_offset_steps,
            "inference_delay_steps": self.inference_delay_steps,
            "inference_trigger_steps": self.trigger_steps,
            "queue_depth_after_accept": self.queue_depth_after,
            "observation_timestamp_ns": self.observation_timestamp_ns,
            "inference_submit_timestamp_ns": self.submitted_timestamp_ns,
            "chunk_accept_timestamp_ns": self.accepted_timestamp_ns,
            "rtc_mode": self.rtc_mode,
            "rtc_latency_quantile_steps": self.rtc_latency_quantile_steps,
            "rtc_merge_applied": self.rtc_merge_applied,
            "rtc_previous_steps_at_accept": self.rtc_previous_steps_at_accept,
            "rtc_preserved_steps": self.rtc_preserved_steps,
            "rtc_transition_steps": self.rtc_transition_steps,
            "rtc_transition_curve": self.rtc_transition_curve,
            "rtc_boundary_linf_before": self.rtc_boundary_linf_before,
            "rtc_boundary_linf_after": self.rtc_boundary_linf_after,
        }


@dataclass(frozen=True, slots=True)
class NoActionResult:
    """An explicit transport receipt, never an executable ActionChunk."""

    schema: VectorSchema
    frequency_hz: float
    generation: int
    request_id: str
    transport_receipt: Mapping[str, Any]
    inference_executed: bool

    @classmethod
    def from_policy_result(
        cls,
        payload: Mapping[str, Any],
        *,
        generation: int,
        request_id: str,
    ) -> NoActionResult:
        if payload.get("result_kind") != "no_action":
            raise ValueError("expected explicit no_action result")
        schema = VectorSchema.from_wire(payload["action_schema"])
        actions = payload["actions"]
        if (
            not isinstance(actions, np.ndarray)
            or actions.dtype != np.dtype(np.float32)
            or actions.shape != (0, schema.dim)
        ):
            raise ValueError(
                f"no_action requires float32 ndarray with shape (0, {schema.dim})"
            )
        frequency = payload["action_frequency_hz"]
        if isinstance(frequency, (bool, str)):
            raise ValueError("no_action frequency must be a finite positive number")
        frequency = float(frequency)
        if not np.isfinite(frequency) or frequency <= 0:
            raise ValueError("no_action frequency must be a finite positive number")
        metadata = payload.get("policy_metadata")
        if not isinstance(metadata, Mapping):
            raise ValueError("no_action requires policy_metadata")
        receipt = metadata.get("transport_receipt")
        executed = metadata.get("inference_executed")
        if not isinstance(receipt, dict) or type(executed) is not bool:
            raise ValueError(
                "no_action requires a transport_receipt dict and inference_executed bool"
            )
        if not request_id or generation < 0:
            raise ValueError("no_action requires request identity and valid generation")
        # Receipts become JSON status diagnostics. Reject NaN/objects and detach
        # the data from the mutable transport payload before publishing it.
        receipt = json.loads(json.dumps(receipt, allow_nan=False))
        return cls(
            schema, frequency, generation, request_id,
            MappingProxyType(receipt), executed,
        )


@dataclass(frozen=True, slots=True)
class InferenceResult:
    pending: PendingChunk
    chunk: ActionChunk | None
    roundtrip_ms: float
    started_ns: int
    ended_ns: int
    error: str | None = None
    no_action: NoActionResult | None = None


class AsyncPolicyInference:
    """Run one persistent synchronous XPolicy client off the robot event loop."""

    def __init__(
        self,
        client_factory: Callable[[], Any],
        *,
        allow_no_action: bool = False,
        allow_review_wait: bool = False,
    ):
        self._client_factory = client_factory
        self.allow_no_action = bool(allow_no_action)
        self.allow_review_wait = bool(allow_review_wait)
        self._client: Any | None = None
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="xpolicy-inference",
        )
        self._future: Future[InferenceResult] | None = None
        self._closed = False

    def submit(
        self,
        observation: Mapping[str, Any],
        pending: PendingChunk,
    ) -> bool:
        if self._closed:
            raise RuntimeError("inference worker is closed")
        if self._future is not None:
            return False
        self._future = self._executor.submit(
            self._infer,
            dict(observation),
            pending,
        )
        return True

    def _infer(
        self,
        observation: dict[str, Any],
        pending: PendingChunk,
    ) -> InferenceResult:
        started = time.monotonic()
        started_ns = time.time_ns()
        try:
            if self._client is None:
                self._client = self._client_factory()
            response = self._client.infer(observation)
            ended_ns = time.time_ns()
            if (
                self.allow_no_action
                and response.payload.get("result_kind") != "no_action"
            ):
                raise ValueError("no_action transport mode rejects executable policy results")
            if response.payload.get("result_kind") == "no_action":
                if not (self.allow_no_action or self.allow_review_wait):
                    raise ValueError("no_action is disabled for this policy client")
                no_action = NoActionResult.from_policy_result(
                    response.payload,
                    generation=pending.generation,
                    request_id=response.request_id,
                )
                return InferenceResult(
                    pending=pending,
                    chunk=None,
                    roundtrip_ms=(time.monotonic() - started) * 1000.0,
                    started_ns=started_ns,
                    ended_ns=ended_ns,
                    no_action=no_action,
                )
            return InferenceResult(
                pending=pending,
                chunk=ActionChunk.from_policy_result(
                    response.payload,
                    generation=pending.generation,
                    request_id=response.request_id,
                ),
                roundtrip_ms=(time.monotonic() - started) * 1000.0,
                started_ns=started_ns,
                ended_ns=ended_ns,
            )
        except Exception as exc:
            ended_ns = time.time_ns()
            return InferenceResult(
                pending=pending,
                chunk=None,
                roundtrip_ms=(time.monotonic() - started) * 1000.0,
                started_ns=started_ns,
                ended_ns=ended_ns,
                error=f"{type(exc).__name__}: {exc}",
            )

    def poll(self) -> InferenceResult | None:
        if self._future is None or not self._future.done():
            return None
        future, self._future = self._future, None
        return future.result()

    @property
    def busy(self) -> bool:
        return self._future is not None and not self._future.done()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._future is not None:
            self._future.result()
            self._future = None
        if self._client is not None:
            self._executor.submit(self._client.close).result()
        self._executor.shutdown()


class ActionQueue:
    """Generation-aware, time-aligned action queue for real-world runtimes."""

    def __init__(
        self,
        action_dim: int,
        trigger_steps: int = 0,
        *,
        action_frequency_hz: float | None = None,
        auto_prefetch: bool = False,
        prefetch_quantile: float = 0.95,
        prefetch_margin_steps: int = 1,
        prefetch_window_size: int = 32,
        rtc_config: RealTimeChunkingConfig | None = None,
    ):
        if action_dim < 1 or trigger_steps < 0:
            raise ValueError("action_dim must be positive and trigger_steps non-negative")
        if action_frequency_hz is not None and (
            not np.isfinite(action_frequency_hz) or action_frequency_hz <= 0
        ):
            raise ValueError("action_frequency_hz must be positive when configured")
        if rtc_config is None:
            rtc_config = RealTimeChunkingConfig(
                mode=RtcMode.ALIGN if auto_prefetch else RtcMode.DISABLED,
                latency_quantile=prefetch_quantile,
                latency_margin_steps=prefetch_margin_steps,
                latency_window_size=prefetch_window_size,
            )
        if rtc_config.mode is not RtcMode.DISABLED and action_frequency_hz is None:
            raise ValueError("real-time chunking requires action_frequency_hz")
        self.action_dim = action_dim
        self.configured_trigger_steps = trigger_steps
        self.trigger_steps = trigger_steps
        self.action_frequency_hz = (
            None if action_frequency_hz is None else float(action_frequency_hz)
        )
        self.rtc_config = rtc_config
        self.rtc = RealTimeChunker(rtc_config)
        self.auto_prefetch = rtc_config.mode is not RtcMode.DISABLED
        self.prefetch_quantile = rtc_config.latency_quantile
        self.prefetch_margin_steps = rtc_config.latency_margin_steps
        self.generation = 0
        self.actions: deque[np.ndarray] = deque()
        self.step_indices: deque[int] = deque()
        self.pending: PendingChunk | None = None
        self.execution_step = 0
        self.last_alignment: ChunkAlignment | None = None
        self._retry_required = False

        self.trajectory_processor = None

    def configure_rtc(self, config: RealTimeChunkingConfig) -> None:
        """Replace RTC scheduling parameters while no chunk is in flight.

        Latency samples belong to the configuration that collected them, so a
        reconfiguration deliberately starts a fresh estimator instead of
        carrying old timing evidence into the new mode.
        """
        if self.pending is not None or self.actions:
            raise RuntimeError("pause and empty the action queue before changing RTC")
        if config.mode is not RtcMode.DISABLED and self.action_frequency_hz is None:
            raise ValueError("real-time chunking requires action_frequency_hz")
        self.rtc_config = config
        self.rtc = RealTimeChunker(config)
        self.auto_prefetch = config.mode is not RtcMode.DISABLED
        self.prefetch_quantile = config.latency_quantile
        self.prefetch_margin_steps = config.latency_margin_steps
        self.trigger_steps = self.configured_trigger_steps
        self.last_alignment = None
        self._retry_required = False

    def reset(self, generation: int) -> None:
        generation = int(generation)
        if generation < 0:
            raise ValueError("generation must be non-negative")
        self.generation = generation
        self.actions.clear()
        self.step_indices.clear()
        self.pending = None
        self.execution_step = 0
        self.last_alignment = None
        self._retry_required = False
        if self.trajectory_processor is not None:
            self.trajectory_processor.reset()

    def begin_inference(
        self,
        *,
        observation_timestamp_ns: int = 0,
        submitted_timestamp_ns: int | None = None,
    ) -> PendingChunk:
        if self.pending is not None:
            raise RuntimeError("inference already pending")
        observation_timestamp_ns = int(observation_timestamp_ns)
        submitted_timestamp_ns = (
            time.time_ns()
            if submitted_timestamp_ns is None
            else int(submitted_timestamp_ns)
        )
        if observation_timestamp_ns < 0 or submitted_timestamp_ns < 0:
            raise ValueError("inference timestamps must be non-negative")
        self.pending = PendingChunk(
            self.generation,
            len(self.actions),
            observation_timestamp_ns,
            submitted_timestamp_ns,
            self.execution_step,
        )
        self._retry_required = False
        return self.pending

    def fail_inference(self, pending: PendingChunk) -> bool:
        if self.pending != pending:
            return False
        self.pending = None
        return True

    def hold(self, pending: PendingChunk) -> bool:
        """Discard a current reply and every queued action as one HOLD boundary."""
        if self.pending != pending or pending.generation != self.generation:
            return False
        self.pending = None
        self.actions.clear()
        self.step_indices.clear()
        self.last_alignment = None
        self._retry_required = False
        return True

    def _steps_for_duration(self, duration_ns: int) -> int:
        if self.action_frequency_hz is None or duration_ns <= 0:
            return 0
        return int(np.ceil(duration_ns * self.action_frequency_hz / 1_000_000_000))

    def _update_prefetch(self, delay_steps: int, horizon_steps: int) -> None:
        if not self.auto_prefetch:
            return
        self.trigger_steps = self.rtc.observe_latency(
            max(0, int(delay_steps)),
            horizon_steps=horizon_steps,
            trigger_floor_steps=self.configured_trigger_steps,
        )

    def accept(
        self,
        chunk: np.ndarray,
        pending: PendingChunk,
        *,
        accepted_timestamp_ns: int | None = None,
    ) -> int | None:
        if self.pending != pending or pending.generation != self.generation:
            return None
        values = self._validate(chunk)
        accepted_timestamp_ns = (
            time.time_ns()
            if accepted_timestamp_ns is None
            else int(accepted_timestamp_ns)
        )
        if accepted_timestamp_ns < 0:
            raise ValueError("accepted_timestamp_ns must be non-negative")
        queue_consumed = max(
            0,
            self.execution_step - pending.execution_step_at_submit,
        )
        observation_age_steps = 0
        if pending.queued_at_submit and pending.observation_timestamp_ns:
            observation_age_steps = self._steps_for_duration(
                pending.submitted_timestamp_ns - pending.observation_timestamp_ns
            )
        consumed = queue_consumed + observation_age_steps
        inference_delay_steps = self._steps_for_duration(
            accepted_timestamp_ns - pending.submitted_timestamp_ns
        )
        self._update_prefetch(inference_delay_steps, len(values))
        self.pending = None
        accepted = consumed < len(values)
        self._retry_required = not accepted
        merge: ChunkMerge | None = None
        if accepted:
            previous = (
                np.stack(tuple(self.actions)).astype(np.float32, copy=False)
                if self.actions
                else np.empty((0, self.action_dim), dtype=np.float32)
            )
            merge = self.rtc.merge(previous, values[consumed:])
            if self.trajectory_processor is not None:
                try:
                    processed = self.trajectory_processor.process(previous, merge.values)
                except Exception:
                    self.actions.clear()
                    self.step_indices.clear()
                    self.trajectory_processor.reset()
                    raise
                merge = replace(merge, values=processed)
            self.actions.clear()
            self.step_indices.clear()
            self.actions.extend(row.copy() for row in merge.values)
            self.step_indices.extend(range(consumed, consumed + len(merge.values)))
        self.last_alignment = ChunkAlignment(
            accepted=accepted,
            horizon_steps=len(values),
            queued_at_submit=pending.queued_at_submit,
            queue_consumed_steps=queue_consumed,
            observation_age_steps=observation_age_steps,
            aligned_offset_steps=consumed,
            inference_delay_steps=inference_delay_steps,
            trigger_steps=self.trigger_steps,
            queue_depth_after=len(self.actions),
            observation_timestamp_ns=pending.observation_timestamp_ns,
            submitted_timestamp_ns=pending.submitted_timestamp_ns,
            accepted_timestamp_ns=accepted_timestamp_ns,
            rtc_mode=self.rtc_config.mode.value,
            rtc_latency_quantile_steps=self.rtc.last_latency_quantile_steps,
            rtc_merge_applied=False if merge is None else merge.applied,
            rtc_previous_steps_at_accept=(
                0 if merge is None else merge.previous_steps
            ),
            rtc_preserved_steps=0 if merge is None else merge.preserved_steps,
            rtc_transition_steps=(
                0 if merge is None else merge.transitioned_steps
            ),
            rtc_transition_curve=(
                self.rtc_config.transition_curve.value
                if merge is None
                else merge.transition_curve
            ),
            rtc_boundary_linf_before=(
                None if merge is None else merge.boundary_linf_before
            ),
            rtc_boundary_linf_after=(
                None if merge is None else merge.boundary_linf_after
            ),
        )
        return consumed

    def pop(self) -> tuple[int, np.ndarray] | None:
        if not self.actions:
            return None
        step = self.step_indices.popleft(), self.actions.popleft()
        self.execution_step += 1
        if self.trajectory_processor is not None:
            self.trajectory_processor.emitted(step[1])
        return step

    @property
    def should_infer(self) -> bool:
        return self.pending is None and (
            self._retry_required or len(self.actions) <= self.trigger_steps
        )

    def _validate(self, chunk: np.ndarray) -> np.ndarray:
        values = np.asarray(chunk, dtype=np.float32)
        if (
            values.ndim != 2
            or values.shape[1] != self.action_dim
            or not len(values)
        ):
            raise ValueError(
                f"expected non-empty (T,{self.action_dim}) chunk, got {values.shape}"
            )
        if not np.isfinite(values).all():
            raise ValueError("action chunk contains non-finite values")
        return values


@dataclass(frozen=True, slots=True)
class ActionChunkStep:
    generation: int
    chunk_step_index: int
    values: np.ndarray
    execution_tick_index: int | None = None
    source_phase: float | None = None


class ActionChunkRuntime:
    """Robot-independent ActionChunk scheduling and prefetch runtime.

    Dora nodes only adapt observations and publish the returned steps. Robot
    runtimes provide the action schema, frequency and optional horizon limit.
    """

    def __init__(
        self,
        schema: VectorSchema,
        *,
        action_frequency_hz: float,
        action_chunk_size: int | None = None,
        auto_action_chunk_size: bool = False,
        trigger_steps: int = 0,
        auto_prefetch: bool = False,
        prefetch_quantile: float = 0.95,
        prefetch_margin_steps: int = 1,
        prefetch_window_size: int = 32,
        rtc_config: RealTimeChunkingConfig | None = None,
        allow_no_action: bool = False,
        allow_review_wait: bool = False,
        trajectory_profile: str | None = None,
        trajectory_time_scale: float = 1.0,
    ) -> None:
        frequency = float(action_frequency_hz)
        if not np.isfinite(frequency) or frequency <= 0:
            raise ValueError("action_frequency_hz must be positive")
        if action_chunk_size is not None and int(action_chunk_size) < 1:
            raise ValueError("action_chunk_size must be positive")
        self.schema = schema
        self.action_frequency_hz = frequency
        self.timebase = ActionTimebase(trajectory_time_scale)
        self.timebase.validate_schema(schema)
        if self.timebase.scale != 1 and (allow_no_action or allow_review_wait):
            raise ValueError("retiming is not supported for transport-only or reviewed actions")
        self.last_dispatch_chunk_size: int | None = None
        self.action_chunk_size = (
            None if action_chunk_size is None else int(action_chunk_size)
        )
        self.default_action_chunk_size = self.action_chunk_size
        self.auto_action_chunk_size = bool(auto_action_chunk_size)
        self.last_effective_action_chunk_size: int | None = None
        self.queue = ActionQueue(
            schema.dim,
            trigger_steps,
            action_frequency_hz=frequency,
            auto_prefetch=auto_prefetch,
            prefetch_quantile=prefetch_quantile,
            prefetch_margin_steps=prefetch_margin_steps,
            prefetch_window_size=prefetch_window_size,
            rtc_config=rtc_config,
        )
        self.enabled = False
        self.single_chunk = False
        self.allow_no_action = bool(allow_no_action)
        self.allow_review_wait = bool(allow_review_wait)

        self._validate_rtc_request(rtc_config or self.queue.rtc_config)
        if trajectory_profile:
            from XPolicyLab.real_world.trajectory_profile import load_profile, TrajectoryProcessor
            profile = load_profile(trajectory_profile)
            if ("trajectory_time_scale" in profile
                    and self.timebase.scale != profile["trajectory_time_scale"]):
                raise ValueError("Trajectory time scale is pinned by the trajectory profile")
            if self.queue.rtc_config.mode is not RtcMode.ALIGN or self.queue.rtc_config.request_remaining_steps != profile["request_remaining_steps"]:
                raise ValueError("Trajectory profile requires its pinned align/prefetch configuration")
            if self.auto_action_chunk_size or self.action_chunk_size != profile["execution_steps"]:
                raise ValueError("Trajectory profile requires its pinned fixed execution prefix")
            self.queue.trajectory_processor = TrajectoryProcessor(profile, schema, frequency)

    def _validate_rtc_request(
        self,
        config: RealTimeChunkingConfig,
        *,
        action_chunk_size: int | None = None,
    ) -> None:
        request = config.request_remaining_steps
        queue_size = (
            self.action_chunk_size
            if action_chunk_size is None
            else int(action_chunk_size)
        )
        if (
            config.mode is not RtcMode.DISABLED
            and request
            and queue_size is not None
            and request >= self.timebase.dispatch_steps(queue_size)
        ):
            raise ValueError(
                "RTC request remaining steps must be smaller than the "
                f"execution chunk ({queue_size})"
            )

    @property
    def action_chunk_mode(self) -> str:
        return "auto" if self.auto_action_chunk_size else "fixed"

    def configure_action_chunk(self, mode: str, steps: int | None = None) -> None:
        """Configure the execution prefix while the runtime is safely paused.

        Auto mode follows the model's ``execute_prefix_steps`` metadata and
        falls back to the deployment default. Fixed mode executes exactly the
        requested prefix. Changing this while an episode is active would mix
        two scheduling contracts, so it is rejected.
        """
        normalized = str(mode).strip().lower()
        if self.queue.trajectory_processor is not None:
            if normalized != "fixed" or steps != self.action_chunk_size:
                raise ValueError("Execution prefix is pinned by the trajectory profile")
        if normalized not in {"auto", "fixed"}:
            raise ValueError("action chunk mode must be auto or fixed")
        if self.enabled or self.queue.pending is not None or self.queue.actions:
            raise RuntimeError("pause the ActionChunk runtime before reconfiguring it")
        if normalized == "fixed":
            if steps is None or int(steps) < 1:
                raise ValueError("fixed action chunk steps must be positive")
            next_size = int(steps)
            next_auto = False
        else:
            next_size = self.default_action_chunk_size
            next_auto = True
        self._validate_rtc_request(
            self.queue.rtc_config,
            action_chunk_size=next_size,
        )
        self.action_chunk_size = next_size
        self.auto_action_chunk_size = next_auto
        self.last_effective_action_chunk_size = None
        self.last_dispatch_chunk_size = None

    def configure_rtc(self, config: RealTimeChunkingConfig) -> None:
        """Configure client-side RTC only while execution is safely paused."""
        if not isinstance(config, RealTimeChunkingConfig):
            raise TypeError("RTC configuration must be RealTimeChunkingConfig")
        if self.queue.trajectory_processor is not None and config != self.queue.rtc_config:
            raise ValueError("RTC configuration is pinned by the trajectory profile")
        if self.enabled or self.queue.pending is not None or self.queue.actions:
            raise RuntimeError("pause the ActionChunk runtime before changing RTC")
        self._validate_rtc_request(config)
        self.queue.configure_rtc(config)

    def _effective_action_chunk_size(self, chunk: ActionChunk) -> int | None:
        limit = self.action_chunk_size
        if self.auto_action_chunk_size:
            recommended = chunk.metadata.get("execute_prefix_steps")
            if recommended is not None:
                try:
                    limit = int(recommended)
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        "model execute_prefix_steps metadata must be an integer"
                    ) from exc
        if limit is None:
            return None
        if limit < 1 or limit > len(chunk.values):
            raise ValueError(
                f"execution prefix {limit} is outside model horizon "
                f"[1, {len(chunk.values)}]"
            )
        return limit

    def control(self, command: str, generation: int) -> None:
        if command not in {"bootstrap", "run", "step", "pause"}:
            raise ValueError(f"unsupported ActionChunk command: {command}")
        self.queue.reset(generation)
        self.last_dispatch_chunk_size = None
        self.enabled = command in {"bootstrap", "run", "step"}
        self.single_chunk = command == "step"

    def begin_inference(
        self,
        *,
        observation_timestamp_ns: int = 0,
        submitted_timestamp_ns: int | None = None,
    ) -> PendingChunk:
        if not self.enabled:
            raise RuntimeError("ActionChunk runtime is paused")
        return self.queue.begin_inference(
            observation_timestamp_ns=observation_timestamp_ns,
            submitted_timestamp_ns=submitted_timestamp_ns,
        )

    def accept(
        self,
        chunk: ActionChunk,
        pending: PendingChunk,
        *,
        accepted_timestamp_ns: int | None = None,
    ) -> int | None:
        if self.allow_no_action:
            raise ValueError("no_action transport mode rejects executable ActionChunks")
        requested_trajectory = chunk.metadata.get("trajectory_profile")
        processor = self.queue.trajectory_processor
        if requested_trajectory is not None or processor is not None:
            if processor is None or requested_trajectory != processor.profile["algorithm"]:
                raise ValueError("Model and execution trajectory profile must match")
        if chunk.schema != self.schema:
            raise ValueError(
                f"expected action schema {self.schema.schema_id}, "
                f"got {chunk.schema.schema_id}"
            )
        if not np.isclose(chunk.frequency_hz, self.action_frequency_hz):
            raise ValueError(
                f"action frequency {chunk.frequency_hz:g} Hz does not match "
                f"runtime {self.action_frequency_hz:g} Hz"
            )
        if (
            chunk.generation != pending.generation
            or pending.generation != self.queue.generation
        ):
            return None
        values = chunk.values
        effective_size = self._effective_action_chunk_size(chunk)
        self.last_effective_action_chunk_size = (
            len(values) if effective_size is None else effective_size
        )
        if effective_size is not None:
            values = values[:effective_size]
        # Expand before alignment, overlap processing and prefetch estimation.
        # The queue continues to consume actual dispatch ticks at frequency Hz,
        # so a 200 ms inference skips six dispatch rows (three source rows at
        # scale=.5), not six slowed source rows.
        values = self.timebase.expand(values)
        self.last_dispatch_chunk_size = len(values)
        return self.queue.accept(
            values,
            pending,
            accepted_timestamp_ns=accepted_timestamp_ns,
        )

    def fail_inference(self, pending: PendingChunk) -> bool:
        return self.queue.fail_inference(pending)

    def accept_no_action(self, result: NoActionResult, pending: PendingChunk) -> bool:
        if not (self.allow_no_action or self.allow_review_wait):
            raise ValueError("no_action is disabled for this ActionChunk runtime")
        if (
            not self.enabled
            or result.generation != pending.generation
            or pending.generation != self.queue.generation
            or self.queue.pending != pending
        ):
            return False
        if result.schema != self.schema:
            raise ValueError("no_action schema does not match the runtime schema")
        if result.frequency_hz != self.action_frequency_hz:
            raise ValueError("no_action frequency does not match the runtime frequency")
        if not self.queue.hold(pending):
            return False
        self.last_effective_action_chunk_size = 0
        if self.single_chunk:
            self.enabled = False
            self.single_chunk = False
        return True

    def tick(self) -> ActionChunkStep | None:
        if self.allow_no_action:
            # The transport-only mode has no executable output path, including
            # while a cloud request is still in flight.
            return None
        queued = self.queue.pop() if self.enabled else None
        if queued is None:
            return None
        index, values = queued
        if self.single_chunk and not self.queue.actions:
            self.enabled = False
            self.single_chunk = False
        phase = index * self.timebase.scale
        if self.timebase.scale != 1 and self.last_effective_action_chunk_size is not None:
            phase = min(phase, self.last_effective_action_chunk_size - 1)
        return ActionChunkStep(self.queue.generation, int(np.floor(phase)), values,
                               execution_tick_index=index, source_phase=phase)

    @property
    def should_infer(self) -> bool:
        if self.single_chunk and self.queue.actions:
            return False
        return self.enabled and self.queue.should_infer


@dataclass(frozen=True, slots=True)
class ActionChunkPolicyTick:
    step: ActionChunkStep | None
    inference: InferenceResult | None
    inference_submitted: bool = False
    no_action_accepted: bool = False


class ActionChunkPolicyRuntime:
    """Model transport plus ActionChunk scheduling, with no Dora dependency."""

    def __init__(
        self,
        runtime: ActionChunkRuntime,
        inference: AsyncPolicyInference,
        *,
        retry_seconds: float = 0.5,
    ) -> None:
        if retry_seconds < 0:
            raise ValueError("retry_seconds must be non-negative")
        self.runtime = runtime
        self.inference = inference
        self.retry_seconds = float(retry_seconds)
        self.retry_at = 0.0
        self._closed = False

    def control(self, command: str, generation: int) -> None:
        self.runtime.control(command, generation)
        self.retry_at = 0.0

    def tick(
        self,
        observation: Mapping[str, Any] | None,
        *,
        observation_timestamp_ns: int = 0,
        accepted_timestamp_ns: int | None = None,
    ) -> ActionChunkPolicyTick:
        if self._closed:
            raise RuntimeError("ActionChunk policy runtime is closed")
        accepted_timestamp_ns = (
            time.time_ns()
            if accepted_timestamp_ns is None
            else int(accepted_timestamp_ns)
        )
        result = self.inference.poll()
        no_action_accepted = False
        if result is not None:
            if result.error is not None:
                if self.runtime.allow_no_action:
                    if self.runtime.queue.hold(result.pending):
                        self.retry_at = time.monotonic() + max(self.retry_seconds, 0.5)
                elif self.runtime.fail_inference(result.pending):
                    self.retry_at = time.monotonic() + self.retry_seconds
            elif getattr(result, "no_action", None) is not None:
                try:
                    no_action_accepted = self.runtime.accept_no_action(
                        result.no_action, result.pending
                    )
                except Exception as exc:
                    # Contract rejection is a recoverable HOLD, not a reason to
                    # crash the formal node or strand an in-flight request.
                    if self.runtime.queue.hold(result.pending):
                        self.retry_at = time.monotonic() + max(self.retry_seconds, 0.5)
                    result = replace(
                        result,
                        no_action=None,
                        error=f"{type(exc).__name__}: {exc}",
                    )
                if no_action_accepted:
                    # Empty replies must not create a busy request loop or let
                    # buffered actions run while the cloud explicitly holds.
                    self.retry_at = time.monotonic() + max(self.retry_seconds, 0.5)
            elif result.chunk is not None:
                try:
                    self.runtime.accept(
                        result.chunk,
                        result.pending,
                        accepted_timestamp_ns=accepted_timestamp_ns,
                    )
                except Exception:
                    self.runtime.fail_inference(result.pending)
                    self.retry_at = time.monotonic() + self.retry_seconds
                    raise

        submitted = False
        if (
            observation is not None
            and self.runtime.should_infer
            and not self.inference.busy
            and time.monotonic() >= self.retry_at
        ):
            pending = self.runtime.begin_inference(
                observation_timestamp_ns=observation_timestamp_ns,
                submitted_timestamp_ns=time.time_ns(),
            )
            submitted = self.inference.submit(observation, pending)
            if not submitted:
                self.runtime.fail_inference(pending)

        return ActionChunkPolicyTick(
            step=self.runtime.tick(),
            inference=result,
            inference_submitted=submitted,
            no_action_accepted=no_action_accepted,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.inference.close()
