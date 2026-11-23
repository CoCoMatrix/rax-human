"""Opt-in hard-prefix scheduling; no SDK access, filters, or hardware writes.

Keep the full prediction horizon as reserve, commit ten controller targets at
a time, and condition the next prediction on a fixed five-step future prefix.
Observation-age rows are counted from emitted-target history, not rounded
wall-clock time. Prefix targets are in the policy space, before owner safety
and filtering. An owner Hold/generation change invalidates the whole request.
"""
from collections import deque
from dataclasses import dataclass, replace
import math
import json
from pathlib import Path
import time
import uuid

import numpy as np

from XPolicyLab.real_world.inference import (
    ActionChunkRuntime, ActionChunkPolicyTick, ChunkAlignment, InferenceResult, AsyncPolicyInference,
)
from XPolicyLab.real_world.rtc import RtcMode


@dataclass(frozen=True)
class PrefixCommitment:
    pending: object
    token: str
    prefix: np.ndarray
    delay: int
    past_steps: int
    future_steps: int
    apply_at_step: int
    submitted_ns: int
    one_euro_state: np.ndarray | None = None


class TrainingPrefixInference(AsyncPolicyInference):
    def __init__(self, *args, suffix_processor=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.suffix_processor = suffix_processor

    def _infer(self, observation, pending):
        # The synchronous WS client's event loop is idle between episodes and
        # an old connection may have hit the server's ping timeout. Reconnect
        # only at cold start, on the existing inference worker thread.
        rtc = observation.get("training_time_rtc", {})
        if rtc.get("delay") == 0 and pending.queued_at_submit == 0 and self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass  # Cleanup failure must not reuse the stale connection.
            self._client = None
        result = super()._infer(observation, pending)
        if self.suffix_processor is not None and result.error is None:
            try:
                result = self.suffix_processor(result)
            except Exception as exc:
                result = replace(result, chunk=None, error=str(exc))
        return result


class TrainingPrefixRuntime(ActionChunkRuntime):
    def __init__(self, *args, prediction_horizon=50, commit_steps=10,
                 future_budget_steps=5, max_prefix_steps=6, native_trajectory_profile=None,
                 velocity_seam_profile=None, one_euro_profile=None, joint_clip_profile=None,
                 enforce_wallclock_deadline=False, precommit_profile=None, **kwargs):
        super().__init__(*args, **kwargs)
        if self.timebase.scale != 1:
            raise ValueError("Training-prefix RTC requires its original trajectory timebase")
        if self.queue.trajectory_processor is not None:
            raise ValueError("Native prefix candidate requires optional trajectory postprocessing disabled")
        if self.queue.rtc_config.mode is not RtcMode.DISABLED:
            raise ValueError("Training-prefix RTC must not be combined with posthoc RTC blending")
        if not 0 < future_budget_steps < commit_steps < prediction_horizon:
            raise ValueError("Require 0 < future budget < commit interval < prediction horizon")
        if not future_budget_steps <= max_prefix_steps < prediction_horizon:
            raise ValueError("Prefix budget outside trained range")
        if self.auto_action_chunk_size or self.action_chunk_size != commit_steps:
            raise ValueError("Native prefix scheduling requires the declared fixed commit interval")
        self.prediction_horizon = prediction_horizon
        self.commit_steps = commit_steps
        self.future_budget_steps = future_budget_steps
        self.max_prefix_steps = max_prefix_steps
        self.enforce_wallclock_deadline = bool(enforce_wallclock_deadline)
        self.commitment = None
        self.next_boundary = None
        self.history = deque(maxlen=prediction_horizon * 2)
        self.last_native_info = {}
        self.native_trajectory = None
        self.velocity_seam = None
        self.one_euro = None
        self.one_euro_states = None
        self.joint_clip = None
        self.precommit = None
        if precommit_profile:
            if not velocity_seam_profile or any((native_trajectory_profile, one_euro_profile, joint_clip_profile)):
                raise ValueError('Precommit trial requires velocity seam only')
            from XPolicyLab.real_world.precommit_limits import PrecommitLimits
            self.precommit = PrecommitLimits(precommit_profile, self.schema)
        if sum(bool(p) for p in (native_trajectory_profile, velocity_seam_profile, one_euro_profile, joint_clip_profile)) > 1:
            raise ValueError('Choose one native trajectory experiment at a time')
        if joint_clip_profile:
            from XPolicyLab.real_world.bounded_joint_clip import BoundedJointClip
            self.joint_clip = BoundedJointClip(joint_clip_profile, self.schema, self.action_frequency_hz)
        if one_euro_profile:
            from XPolicyLab.real_world.bounded_one_euro import BoundedOneEuro
            self.one_euro = BoundedOneEuro(one_euro_profile, self.schema, self.action_frequency_hz)
        if velocity_seam_profile:
            if native_trajectory_profile:
                raise ValueError('Choose one native trajectory experiment at a time')
            from XPolicyLab.real_world.velocity_seam import VelocitySeam
            seam_class = VelocitySeam
            seam_config = json.loads(Path(velocity_seam_profile).read_text())
            if seam_config.get('schema') == 'xpolicy.acceleration-seam/v1':
                from XPolicyLab.real_world.acceleration_seam import AccelerationSeam
                seam_class = AccelerationSeam
            elif seam_config.get('tiered_velocity') is True:
                from XPolicyLab.real_world.tiered_velocity_seam import TieredVelocitySeam
                seam_class = TieredVelocitySeam
            elif seam_config.get('acceleration_blend') is True:
                from XPolicyLab.real_world.acceleration_blend_seam import AccelerationBlendSeam
                seam_class = AccelerationBlendSeam
            self.velocity_seam = seam_class(velocity_seam_profile, self.schema, self.action_frequency_hz)
            if future_budget_steps < getattr(self.velocity_seam, 'anchor_count', 2):
                raise ValueError('Velocity seam has insufficient committed future anchors')
        if native_trajectory_profile:
            from XPolicyLab.real_world.native_trajectory import NativeTrajectory
            self.native_trajectory = NativeTrajectory(native_trajectory_profile, self.schema, self.action_frequency_hz)
            if future_budget_steps < 3:
                raise ValueError('Native trajectory requires at least three committed future anchors')
        self.seed_state = None
        self.original_chunk = None
        self.last_raw_action = None

    def control(self, command, generation):
        super().control(command, generation)
        self.commitment = None
        self.next_boundary = None
        self.history.clear()
        self.last_native_info = {}
        self.seed_state = None
        self.original_chunk = None
        self.last_raw_action = None
        self.one_euro_states = None

    def configure_rtc(self, config):
        if config.mode is not RtcMode.DISABLED:
            raise ValueError("Training-prefix RTC is model conditioning; leave posthoc RTC Off")
        super().configure_rtc(config)

    def configure_action_chunk(self, mode, steps=None):
        if mode != "fixed" or steps != self.commit_steps:
            raise ValueError(f"This native RTC deployment fixes the commit interval at {self.commit_steps} steps")
        super().configure_action_chunk(mode, steps)

    @property
    def should_infer(self):
        if not self.enabled or self.queue.pending is not None:
            return False
        if self.next_boundary is None:
            return True
        if self.single_chunk:
            return False
        if self.native_trajectory is not None:
            # If a worker is still busy at the exact prefetch point, consume
            # the already validated reserve and try at its next boundary.
            return self.queue.execution_step == self.next_boundary - self.future_budget_steps
        return self.queue.execution_step >= self.next_boundary - self.future_budget_steps

    def make_request(self, observation, observation_timestamp_ns, now_ns):
        if not self.should_infer:
            raise RuntimeError("A new prediction is not due")
        if observation_timestamp_ns <= 0 or observation_timestamp_ns > now_ns:
            raise ValueError("Require a valid same-clock observation timestamp")
        cold = self.next_boundary is None
        if cold and self.native_trajectory is not None:
            self.seed_state = self.schema.coerce(observation['state']['vector'], name='trajectory seed', ndim=1)
        past = [] if cold else [v for stamp, v in self.history if stamp >= observation_timestamp_ns]
        if (now_ns - observation_timestamp_ns) * self.action_frequency_hz > self.max_prefix_steps * 1e9:
            raise ValueError("Observation is too old for the trained prefix budget")
        future = 0 if cold else self.next_boundary - self.queue.execution_step
        if not cold and future != self.future_budget_steps:
            raise ValueError("Missed the declared prefetch point")
        delay = len(past) + future
        if delay > self.max_prefix_steps or len(self.queue.actions) < future:
            raise ValueError("No valid committed prefix within trained delay range")
        prefix = np.zeros((self.prediction_horizon, self.schema.dim), np.float32)
        values = past + list(self.queue.actions)[:future]
        if values:
            prefix[:delay] = np.stack(values)
        one_euro_state = None
        if self.one_euro is not None and not cold:
            if self.one_euro_states is None:
                raise ValueError('Missing One Euro committed plan state')
            one_euro_state = self.one_euro_states[self.queue.step_indices[future - 1]].copy()
            one_euro_state.setflags(write=False)
        pending = self.begin_inference(observation_timestamp_ns=observation_timestamp_ns,
                                       submitted_timestamp_ns=now_ns)
        commitment = PrefixCommitment(pending, uuid.uuid4().hex, prefix, delay, len(past), future,
                                      self.queue.execution_step + future, now_ns, one_euro_state)
        self.commitment = commitment
        packet = dict(observation)
        packet["training_time_rtc"] = {
            "protocol": "xpolicy.training_prefix.v1", "request_token": commitment.token,
            "generation": pending.generation, "action_prefix": prefix.copy(), "delay": delay,
            "observation_timestamp_ns": observation_timestamp_ns,
            "commit_until_control_step": commitment.apply_at_step,
            "prefix_space": "physical_policy_targets_before_owner_processing",
        }
        return packet, pending

    def fail_closed(self):
        self.enabled = False
        self.single_chunk = False
        self.queue.actions.clear()
        self.queue.step_indices.clear()
        self.queue.pending = None
        self.queue.last_alignment = None
        self.commitment = None
        self.next_boundary = None
        self.one_euro_states = None

    def accept_prefix_result(self, result, now_ns):
        c = self.commitment
        if c is None or result.pending != c.pending or result.pending.generation != self.queue.generation:
            return False
        if self.queue.execution_step != c.apply_at_step:
            raise ValueError("Prefix execution boundary missed")
        if self.enforce_wallclock_deadline and c.future_steps and result.ended_ns > c.submitted_ns + math.ceil(c.future_steps * 1e9 / self.action_frequency_hz):
            raise ValueError("Training-prefix wall-clock deadline missed")
        chunk = result.chunk
        if chunk is None or chunk.schema != self.schema or chunk.generation != c.pending.generation:
            raise ValueError("Returned chunk identity differs from commitment")
        if not np.isclose(chunk.frequency_hz, self.action_frequency_hz):
            raise ValueError("Returned action cadence differs")
        values = self.queue._validate(chunk.values)
        if values.shape != (self.prediction_horizon, self.schema.dim):
            raise ValueError("Native RTC requires the complete prediction horizon before alignment")
        meta = chunk.metadata
        if (meta.get("training_time_rtc") is not True
                or meta.get("native_prefix_consumed") is not True
                or meta.get("rtc_prefix_protocol") != "xpolicy.training_prefix.v1"
                or meta.get("rtc_request_token") != c.token
                or meta.get("rtc_generation") != c.pending.generation
                or meta.get("rtc_prefix_length") != c.delay):
            raise ValueError("Model did not acknowledge the exact hard-prefix request")
        if not np.array_equal(values[:c.delay], c.prefix[:c.delay]):
            raise ValueError("Model changed committed physical prefix targets")
        remaining = values[c.delay:]
        if len(remaining) < self.commit_steps:
            raise ValueError("Aligned prediction cannot cover the next commit interval")
        if self.native_trajectory is not None:
            if meta.get('native_trajectory_prepared_token') != c.token:
                raise ValueError('Native trajectory must be prepared on inference worker before handoff')
            self.native_trajectory.last_report = meta['native_trajectory_report']
        if self.velocity_seam is not None and meta.get('velocity_seam_prepared_token') != c.token:
            raise ValueError('Velocity seam must be prepared on the inference worker')
        if self.precommit is not None:
            if meta.get('precommit_prepared_token') != c.token:
                raise ValueError('Precommit trial was not prepared on worker')
            if self.precommit.strict_commit:
                self.precommit.validate(remaining, self.commit_steps)
        if self.joint_clip is not None and meta.get('joint_clip_prepared_token') != c.token:
            raise ValueError('Joint clip must be prepared on the inference worker')
        if self.one_euro is not None:
            if meta.get('one_euro_prepared_token') != c.token:
                raise ValueError('One Euro must be prepared on the inference worker')
            states = np.asarray(meta['one_euro_states'])
            if (states.shape != (self.prediction_horizon, 2, len(self.one_euro.indices))
                    or not np.isfinite(states).all()
                    or not np.array_equal(states[c.delay:, 0], remaining[:, self.one_euro.indices])):
                raise ValueError('One Euro state does not match the accepted plan')
            self.one_euro_states = states.copy()
        self.original_chunk = np.asarray(meta.get('native_trajectory_raw_values', values)).copy()
        self.queue.actions = deque(row.copy() for row in remaining)
        self.queue.step_indices = deque(range(c.delay, self.prediction_horizon))
        self.queue.pending = None
        self.queue._retry_required = False
        self.last_effective_action_chunk_size = self.commit_steps
        self.next_boundary = self.queue.execution_step + self.commit_steps
        delay_steps = int(math.ceil((result.ended_ns - c.submitted_ns) * self.action_frequency_hz / 1e9))
        self.queue.last_alignment = ChunkAlignment(
            accepted=True, horizon_steps=self.prediction_horizon,
            queued_at_submit=c.pending.queued_at_submit, queue_consumed_steps=c.future_steps,
            observation_age_steps=c.past_steps, aligned_offset_steps=c.delay,
            inference_delay_steps=max(0, delay_steps), trigger_steps=self.future_budget_steps,
            queue_depth_after=len(remaining), observation_timestamp_ns=c.pending.observation_timestamp_ns,
            submitted_timestamp_ns=c.submitted_ns, accepted_timestamp_ns=now_ns,
            rtc_mode="training_prefix", rtc_merge_applied=False,
        )
        self.last_native_info = {
            "training_time_rtc": True, "native_prefix_length": c.delay,
            "native_prefix_past_steps": c.past_steps, "native_prefix_future_steps": c.future_steps,
            "native_commit_interval": self.commit_steps, "native_apply_at_step": c.apply_at_step,
            "native_next_boundary": self.next_boundary, "native_prefix_exact": True,
            "native_prefix_consumed": True,
            "native_model_inference_ms": meta.get("native_model_inference_ms"),
            **(self.native_trajectory.last_report if self.native_trajectory else {}),
            **(meta.get('velocity_seam_report', {}) if self.velocity_seam else {}),
            **(meta.get('one_euro_report', {}) if self.one_euro else {}),
            **(meta.get('joint_clip_report', {}) if self.joint_clip else {}),
            **(meta.get('precommit_report', {}) if self.precommit else {}),
        }
        self.commitment = None
        return True

    def prepare_prefix_result(self, result):
        """Worker-only suffix fit using the immutable future commitment.

        No queue edits or emitted-history reads while the control thread runs.
        A stopped/stale request is discarded by the normal generation checks.
        """
        c = self.commitment
        if ((self.native_trajectory is None and self.velocity_seam is None and self.one_euro is None and self.joint_clip is None)
                or c is None or result.pending != c.pending
                or result.chunk is None):
            return result
        values = self.schema.coerce(result.chunk.values, name='native raw chunk', ndim=2)
        if values.shape != (self.prediction_horizon, self.schema.dim):
            raise ValueError('Native trajectory needs the full prediction horizon')
        if not np.array_equal(values[:c.delay], c.prefix[:c.delay]):
            raise ValueError('Model changed committed physical prefix targets')
        if self.joint_clip is not None:
            processed = values.copy()
            processed[c.delay:], report = self.joint_clip.process(values[c.delay:])
            metadata = dict(result.chunk.metadata)
            metadata.update(joint_clip_prepared_token=c.token,
                            native_trajectory_raw_values=values, joint_clip_report=report)
            return replace(result, chunk=replace(result.chunk, values=processed, metadata=metadata),
                           ended_ns=time.time_ns())
        if self.one_euro is not None:
            processed = values.copy()
            processed[c.delay:], suffix_states, report = self.one_euro.process(values[c.delay:], c.one_euro_state)
            states = np.zeros((self.prediction_horizon, 2, len(self.one_euro.indices)), dtype=float)
            states[c.delay:] = suffix_states
            metadata = dict(result.chunk.metadata)
            metadata.update(one_euro_prepared_token=c.token, one_euro_states=states,
                            native_trajectory_raw_values=values, one_euro_report=report)
            return replace(result, chunk=replace(result.chunk, values=processed, metadata=metadata),
                           ended_ns=time.time_ns())
        if self.velocity_seam is not None:
            processed = values.copy()
            precommit_report = {}
            if self.precommit is not None:
                processed[c.delay:], precommit_report = self.precommit.process(values[c.delay:], self.commit_steps)
            count = getattr(self.velocity_seam, 'anchor_count', 2)
            report = {'trajectory_algorithm': getattr(self.velocity_seam, 'algorithm', 'discrete_velocity_seam_v1'),
                      'trajectory_cold_start_bypass': True}
            if c.future_steps >= count:
                processed[c.delay:], report = self.velocity_seam.process(
                    processed[c.delay:], c.prefix[c.delay-count:c.delay])
            if self.precommit is not None and self.precommit.strict_commit:
                self.precommit.validate(processed[c.delay:], self.commit_steps)
            metadata = dict(result.chunk.metadata)
            metadata.update(velocity_seam_prepared_token=c.token,
                            native_trajectory_raw_values=values, velocity_seam_report=report)
            if self.precommit is not None:
                metadata.update(precommit_prepared_token=c.token, precommit_report=precommit_report)
            return replace(result, chunk=replace(result.chunk, values=processed, metadata=metadata),
                           ended_ns=time.time_ns())
        # Warm commitments contain five future samples: their last three are
        # exactly the samples that will precede the new suffix at handoff.
        anchors = (c.prefix[c.delay-3:c.delay] if c.future_steps >= 3
                   else np.repeat(self.seed_state[None], 3, axis=0))
        processed = values.copy()
        processed[c.delay:] = self.native_trajectory.process(values[c.delay:], anchors)
        metadata = dict(result.chunk.metadata)
        metadata.update(native_trajectory_prepared_token=c.token,
                        native_trajectory_raw_values=values,
                        native_trajectory_report=dict(self.native_trajectory.last_report))
        return replace(result, chunk=replace(result.chunk, values=processed, metadata=metadata),
                       ended_ns=time.time_ns())

    def extend_validated_reserve(self, reason):
        """Only the opt-in smoother has a bounded, terminal-rest reserve.

        Never use a late result at a shifted boundary. Never invent new samples
        or replay a gripper event. Exhaustion finishes at the reserve's rest end.
        """
        if self.native_trajectory is None or not self.queue.actions or self.single_chunk:
            return False
        self.queue.pending = None
        self.commitment = None
        self.next_boundary = self.queue.execution_step + min(self.commit_steps, len(self.queue.actions))
        self.last_native_info.update(
            trajectory_reserve_fallbacks=self.last_native_info.get('trajectory_reserve_fallbacks', 0) + 1,
            trajectory_reserve_reason=reason,
            trajectory_reserve_remaining_steps=len(self.queue.actions),
        )
        return True

    def tick_at(self, now_ns):
        step = super().tick()
        if step is not None:
            self.history.append((now_ns, step.values.copy()))
            if self.original_chunk is not None:
                self.last_raw_action = self.original_chunk[step.chunk_step_index].copy()
        if self.single_chunk and self.next_boundary is not None and self.queue.execution_step >= self.next_boundary:
            self.fail_closed()
        return step


class TrainingPrefixPolicyRuntime:
    """Reuse the owner async client; defer early replies until prefix completion."""
    def __init__(self, runtime, inference):
        self.runtime = runtime
        self.inference = inference
        self.ready = None
        self.closed = False

    def control(self, command, generation):
        self.runtime.control(command, generation)
        self.ready = None

    def tick(self, observation, *, observation_timestamp_ns=0, accepted_timestamp_ns=None):
        if self.closed:
            raise RuntimeError("Native prefix policy runtime is closed")
        now = time.time_ns() if accepted_timestamp_ns is None else int(accepted_timestamp_ns)
        result = self.inference.poll()
        accepted = None
        if result is not None:
            c = self.runtime.commitment
            if c is not None and result.pending == c.pending and result.pending.generation == self.runtime.queue.generation:
                if result.error is not None:
                    if self.runtime.native_trajectory is None or self.runtime.next_boundary is None:
                        self.runtime.fail_closed()
                        return ActionChunkPolicyTick(None, result)
                self.ready = result
            # Stale generation replies are discarded without touching the current queue.
        c = self.runtime.commitment
        if c is not None and self.runtime.queue.execution_step >= c.apply_at_step:
            # Cold start is allowed to wait while holding; there is no old plan.
            cold = c.future_steps == 0 and self.runtime.next_boundary is None
            if (not cold and self.runtime.native_trajectory is not None
                    and (self.ready is None or self.ready.error is not None)):
                reason = 'late_result' if self.ready is None else self.ready.error
                self.ready = None
                if self.runtime.extend_validated_reserve(reason):
                    c = None
                else:
                    self.runtime.fail_closed()
                    return ActionChunkPolicyTick(None, None)
            if c is not None and self.ready is None and not cold:
                self.runtime.fail_closed()
                failure = InferenceResult(c.pending, None, 0., c.submitted_ns, now,
                                          error="training_prefix_deadline_missed: held; no stale-plan continuation")
                return ActionChunkPolicyTick(None, failure)
            if self.ready is not None:
                candidate = self.ready
                self.ready = None
                try:
                    if self.runtime.accept_prefix_result(candidate, now):
                        accepted = candidate
                except Exception as exc:
                    self.runtime.fail_closed()
                    return ActionChunkPolicyTick(None, replace(candidate, chunk=None, error=str(exc)))
        submitted = False
        if observation is not None and self.runtime.should_infer and not self.inference.busy:
            try:
                packet, pending = self.runtime.make_request(observation, observation_timestamp_ns, now)
                submitted = self.inference.submit(packet, pending)
                if not submitted:
                    raise RuntimeError("Native RTC async client refused the committed request")
            except Exception as exc:
                pending = self.runtime.queue.pending
                self.runtime.fail_closed()
                if pending is None:
                    # Preserve the standard error envelope even before transport submission.
                    from XPolicyLab.real_world.inference import PendingChunk
                    pending = PendingChunk(self.runtime.queue.generation, 0,
                                           observation_timestamp_ns=observation_timestamp_ns,
                                           submitted_timestamp_ns=now)
                return ActionChunkPolicyTick(None, InferenceResult(pending, None, 0., now, now, error=str(exc)))
        # Never run beyond a commit boundary merely because a fresh observation
        # or an asynchronous response has not arrived.
        if (self.runtime.enabled and self.runtime.next_boundary is not None
                and self.runtime.queue.execution_step >= self.runtime.next_boundary):
            if not self.runtime.extend_validated_reserve('prefetch_unavailable'):
                self.runtime.fail_closed()
                return ActionChunkPolicyTick(None, accepted, inference_submitted=submitted)
        return ActionChunkPolicyTick(self.runtime.tick_at(now), accepted, inference_submitted=submitted)

    def close(self):
        if not self.closed:
            self.closed = True
            self.runtime.fail_closed()
            self.inference.close()
