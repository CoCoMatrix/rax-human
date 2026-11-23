from __future__ import annotations

from collections import deque
import json
import os
import time
from typing import Any

import numpy as np

from XPolicyLab.real_world.output import (
    LowPassFilter,
    resample_action_chunk,
    smooth_action_chunk,
)
from XPolicyLab.real_world.robots.gr3.mapping import (
    ACTION_NAMES,
    BASE_ACTION_NAMES,
    JOINT_NAMES,
)
from XPolicyLab.real_world.robots.gr3.predictive_mink import (
    PredictiveMinkProjector,
    ProjectionError,
    ProjectionTimeout,
    ProjectorConfig,
)


def _bool_env(name: str, default: bool = False) -> bool:
    value = os.getenv(name, "1" if default else "0")
    if value not in {"0", "1"}:
        raise ValueError(f"{name} must be 0 or 1")
    return value == "1"


def _metadata_int(metadata: dict[str, Any], name: str, default: int) -> int:
    value = metadata.get(name, default)
    return int(value.as_py() if hasattr(value, "as_py") else value)


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _deserialize(value: Any) -> dict[str, float]:
    return {
        str(item["name"].as_py()): float(item["value"].as_py())
        for item in value
    }


def _serialize(pa: Any, values: dict[str, float]) -> Any:
    return pa.StructArray.from_arrays(
        [pa.array(list(values)), pa.array(list(values.values()), type=pa.float32())],
        names=["name", "value"],
    )


def advance_execution_grid(
    native_actions: deque[np.ndarray],
    phase: int,
    factor: int,
) -> tuple[int, bool]:
    if factor < 1 or not 0 <= phase < factor:
        raise ValueError("invalid spline execution phase")
    phase = (phase + 1) % factor
    completed_native_step = phase == 0
    if completed_native_step:
        if not native_actions:
            raise RuntimeError("native output queue lost spline alignment")
        native_actions.popleft()
    return phase, completed_native_step


class Gr3OutputPipeline:
    """Robot-side optional processing between a raw model chunk and the router."""

    def __init__(
        self,
        *,
        qp_enabled: bool = False,
        qp_acceleration_weight: float = 0.25,
        qp_overlap_weight: float = 4.0,
        mink_mode: str = "off",
        filter_alpha: float | None = None,
        spline_factor: int = 1,
        projector: Any = None,
    ) -> None:
        if mink_mode not in {"off", "shadow", "enforce"}:
            raise ValueError("OUTPUT_MINK_MODE must be off, shadow, or enforce")
        if spline_factor < 1:
            raise ValueError("OUTPUT_SPLINE_FACTOR must be positive")
        if spline_factor > 1 and mink_mode != "off":
            raise ValueError("spline execution requires OUTPUT_MINK_MODE=off")
        self.qp_enabled = qp_enabled
        self.qp_acceleration_weight = float(qp_acceleration_weight)
        self.qp_overlap_weight = float(qp_overlap_weight)
        self.mink_mode = mink_mode
        self.spline_factor = spline_factor
        self.projector = projector
        self.lowpass = (
            None if filter_alpha is None else LowPassFilter(float(filter_alpha))
        )
        self.raw_actions: deque[np.ndarray] = deque()
        self.qp_actions: deque[np.ndarray] = deque()
        self.native_actions: deque[np.ndarray] = deque()
        self.native_history: deque[np.ndarray] = deque(maxlen=2)
        self.raw_history: deque[np.ndarray] = deque(maxlen=1)
        self.sent_actions: deque[np.ndarray] = deque(maxlen=3)
        self.projected_actions: deque[np.ndarray] = deque(maxlen=3)
        self.safe_tail: deque[np.ndarray] = deque()
        self.execution_phase = 0
        self.output_index = 0
        self.metadata: dict[str, Any] = {}
        self.qp_info: dict[str, Any] = {"enabled": False, "overlap_steps": 0}

    def reset(self) -> None:
        self.raw_actions.clear()
        self.qp_actions.clear()
        self.native_actions.clear()
        self.native_history.clear()
        self.raw_history.clear()
        self.sent_actions.clear()
        self.projected_actions.clear()
        self.safe_tail.clear()
        self.execution_phase = 0
        self.output_index = 0
        self.metadata = {}
        self.qp_info = {"enabled": False, "overlap_steps": 0}
        if self.lowpass is not None:
            self.lowpass.reset()
        if self.projector is not None:
            self.projector.reset()

    @staticmethod
    def _state_vector(state: dict[str, float]) -> np.ndarray:
        try:
            vector = np.asarray([state[name] for name in JOINT_NAMES], dtype=np.float32)
        except KeyError as exc:
            raise ValueError(f"robot state is missing {exc.args[0]}") from exc
        if not np.isfinite(vector).all():
            raise ValueError("robot state contains non-finite values")
        return vector

    def accept_chunk(
        self,
        raw_chunk: np.ndarray,
        state: dict[str, float],
        metadata: dict[str, Any],
    ) -> None:
        raw_chunk = np.asarray(raw_chunk, dtype=np.float32)
        if raw_chunk.ndim != 2 or raw_chunk.shape[1] != len(JOINT_NAMES):
            raise ValueError(f"raw action chunk must be [T,{len(JOINT_NAMES)}]")
        if not len(raw_chunk) or not np.isfinite(raw_chunk).all():
            raise ValueError("raw action chunk must be non-empty and finite")

        state_vector = self._state_vector(state)
        old_overlap = (
            np.stack(tuple(self.native_actions))
            if metadata.get("rtc_replacement") and self.native_actions
            else None
        )
        previous = self.native_history[-1] if self.native_history else state_vector
        previous_previous = (
            self.native_history[-2] if len(self.native_history) > 1 else previous
        )
        if self.qp_enabled:
            qp_chunk, self.qp_info = smooth_action_chunk(
                raw_chunk,
                previous_action=previous,
                previous_previous_action=previous_previous,
                old_overlap=old_overlap,
                acceleration_weight=self.qp_acceleration_weight,
                overlap_weight=self.qp_overlap_weight,
            )
        else:
            qp_chunk = raw_chunk.copy()
            self.qp_info = {"enabled": False, "overlap_steps": 0}

        raw_previous = self.raw_history[-1] if self.raw_history else state_vector
        command_previous = self.sent_actions[-1] if self.sent_actions else state_vector
        raw_execution = resample_action_chunk(
            raw_chunk,
            previous_action=raw_previous,
            factor=self.spline_factor,
        )
        qp_execution = resample_action_chunk(
            qp_chunk,
            previous_action=command_previous,
            factor=self.spline_factor,
        )
        self.raw_actions.clear()
        self.qp_actions.clear()
        self.native_actions.clear()
        self.raw_actions.extend(row.copy() for row in raw_execution)
        self.qp_actions.extend(row.copy() for row in qp_execution)
        self.native_actions.extend(row.copy() for row in qp_chunk)
        self.execution_phase = 0
        self.output_index = 0
        self.metadata = dict(metadata)

    def pop(
        self,
        state: dict[str, float],
        state_received_at: float | None,
    ) -> dict[str, Any] | None:
        if not self.qp_actions:
            return None
        if not self.raw_actions:
            raise RuntimeError("raw and processed output queues lost alignment")
        raw_step = self.raw_actions.popleft()
        qp_step = self.qp_actions.popleft()
        q_enc = self._state_vector(state)
        projected_step = None
        projector_info: dict[str, Any] = {
            "mode": self.mink_mode,
            "status": "off",
        }

        if self.projector is not None:
            try:
                if state_received_at is None:
                    raise ProjectionError("encoder state is unavailable")
                if not self.projected_actions:
                    self.projected_actions.extend(q_enc.copy() for _ in range(3))
                reference_horizon = np.vstack(
                    (
                        qp_step,
                        *tuple(self.qp_actions)[
                            : max(0, self.projector.config.horizon_steps - 1)
                        ],
                    )
                )
                projected_horizon, projector_info = self.projector.project_horizon(
                    reference_horizon,
                    q_enc,
                    tuple(self.projected_actions),
                    state_age_seconds=time.monotonic() - state_received_at,
                )
                projected_step = projected_horizon[0]
                if self.mink_mode == "enforce":
                    self.safe_tail.clear()
                    self.safe_tail.extend(
                        action.copy() for action in projected_horizon[1:]
                    )
                projector_info = {"mode": self.mink_mode, **projector_info}
            except Exception as exc:
                if (
                    self.mink_mode == "enforce"
                    and isinstance(exc, ProjectionTimeout)
                    and self.safe_tail
                ):
                    projected_step = self.safe_tail.popleft()
                projector_info = {
                    "mode": self.mink_mode,
                    "status": "safe_tail" if projected_step is not None else "error",
                    "fallback_reason": f"{type(exc).__name__}: {exc}",
                }
                if self.mink_mode == "enforce" and projected_step is None:
                    self.raw_actions.clear()
                    self.qp_actions.clear()
                    self.native_actions.clear()
                    raise ProjectionError(projector_info["fallback_reason"]) from exc

        mink_step = (
            projected_step
            if self.mink_mode == "enforce" and projected_step is not None
            else qp_step
        )
        filtered_step = (
            self.lowpass.process(mink_step, initial=q_enc)
            if self.lowpass is not None
            else mink_step
        )
        if projected_step is not None:
            self.projected_actions.append(projected_step.copy())
        self.sent_actions.append(mink_step.copy())
        self.raw_history.append(raw_step.copy())

        self.execution_phase, completed_native_step = advance_execution_grid(
            self.native_actions,
            self.execution_phase,
            self.spline_factor,
        )
        if completed_native_step:
            self.native_history.append(mink_step.copy())

        metadata = {
            **self.metadata,
            "timestamp_ns": time.time_ns(),
            "model_chunk_step_index": self.output_index,
            "queue_depth": len(self.qp_actions),
            "action_dim": len(ACTION_NAMES),
        }
        self.output_index += 1
        return {
            "raw": raw_step,
            "qp": qp_step,
            "projected": projected_step,
            "mink": mink_step,
            "filtered": filtered_step,
            "projector_info": projector_info,
            "metadata": metadata,
        }


def main() -> None:
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError("GR3 output node requires dora-rs and pyarrow") from exc

    qp_enabled = _bool_env("OUTPUT_QP_ENABLED")
    filter_enabled = _bool_env("OUTPUT_FILTER_ENABLED")
    mink_mode = os.getenv("OUTPUT_MINK_MODE", "off").lower()
    spline_factor = (
        int(os.getenv("OUTPUT_SPLINE_FACTOR", "3"))
        if _bool_env("OUTPUT_SPLINE_ENABLED")
        else 1
    )
    projector = (
        None
        if mink_mode == "off"
        else PredictiveMinkProjector(ProjectorConfig.from_env())
    )
    pipeline = Gr3OutputPipeline(
        qp_enabled=qp_enabled,
        qp_acceleration_weight=float(os.getenv("OUTPUT_QP_ACCEL_WEIGHT", "0.25")),
        qp_overlap_weight=float(os.getenv("OUTPUT_QP_OVERLAP_WEIGHT", "4.0")),
        mink_mode=mink_mode,
        filter_alpha=(
            float(os.getenv("OUTPUT_FILTER_ALPHA", "0.6"))
            if filter_enabled
            else None
        ),
        spline_factor=spline_factor,
        projector=projector,
    )
    node = Node()
    robot_state: dict[str, float] = {}
    state_received_at: float | None = None
    generation = 0
    enabled = False

    def send_error(error: Exception) -> None:
        payload = json.dumps(
            {
                "status": "error",
                "generation": generation,
                "error": f"{type(error).__name__}: {error}",
                "stage": "output_pipeline",
            },
            sort_keys=True,
        )
        node.send_output("status", pa.array([payload]), {"generation": generation})

    for event in node:
        if event.get("type") != "INPUT":
            continue
        event_id = event["id"]
        try:
            if event_id == "robot_state":
                robot_state = _deserialize(event["value"])
                state_received_at = time.monotonic()
                continue
            if event_id == "control":
                control = json.loads(str(_scalar(event["value"])))
                generation = int(control["generation"])
                enabled = str(control["command"]) in {"bootstrap", "run", "step"}
                pipeline.reset()
                continue
            if event_id == "model_status":
                node.send_output("status", event["value"], event.get("metadata", {}))
                continue
            if event_id == "action_chunk":
                metadata = dict(event.get("metadata", {}))
                if not enabled or _metadata_int(metadata, "generation", -1) != generation:
                    continue
                action_dim = _metadata_int(metadata, "native_action_dim", len(JOINT_NAMES))
                values = np.asarray(
                    event["value"].to_numpy(zero_copy_only=False),
                    dtype=np.float32,
                )
                if action_dim != len(JOINT_NAMES) or len(values) % action_dim:
                    raise ValueError("invalid raw action chunk payload")
                pipeline.accept_chunk(
                    values.reshape(-1, action_dim),
                    robot_state,
                    metadata,
                )
                continue
            if event_id not in {"tick", "control_tick"}:
                continue
            execution_tick = (
                event_id == "control_tick"
                if pipeline.spline_factor > 1
                else event_id == "tick"
            )
            if not enabled or not execution_tick:
                continue
            result = pipeline.pop(robot_state, state_received_at)
            if result is None:
                continue

            metadata = result["metadata"]
            stages = {
                "raw_action": result["raw"],
                "qp_action": result["qp"],
                "mink_action": result["mink"],
                "filtered_action": result["filtered"],
                "action": result["filtered"],
            }
            for output, values in stages.items():
                node.send_output(
                    output,
                    _serialize(
                        pa,
                        dict(zip(JOINT_NAMES, map(float, values), strict=True)),
                    ),
                    metadata,
                )
            if result["projected"] is not None:
                node.send_output(
                    "projected_action",
                    _serialize(
                        pa,
                        dict(
                            zip(
                                JOINT_NAMES,
                                map(float, result["projected"]),
                                strict=True,
                            )
                        ),
                    ),
                    metadata,
                )
            base = dict.fromkeys(BASE_ACTION_NAMES, 0.0)
            joints = dict(
                zip(JOINT_NAMES, map(float, result["filtered"]), strict=True)
            )
            node.send_output("base_action", _serialize(pa, base), metadata)
            node.send_output(
                "action_robot",
                _serialize(
                    pa,
                    {name: {**joints, **base}[name] for name in ACTION_NAMES},
                ),
                metadata,
            )
            node.send_output(
                "projector_info",
                pa.array([json.dumps(result["projector_info"], sort_keys=True)]),
                metadata,
            )
            node.send_output(
                "output_info",
                pa.array(
                    [
                        json.dumps(
                            {
                                "qp": pipeline.qp_info,
                                "mink": result["projector_info"],
                                "filter": {
                                    "enabled": pipeline.lowpass is not None,
                                    "alpha": (
                                        None
                                        if pipeline.lowpass is None
                                        else pipeline.lowpass.alpha
                                    ),
                                },
                                "spline_factor": pipeline.spline_factor,
                            },
                            sort_keys=True,
                        )
                    ]
                ),
                metadata,
            )
        except Exception as exc:
            pipeline.reset()
            enabled = False
            send_error(exc)


if __name__ == "__main__":
    main()
