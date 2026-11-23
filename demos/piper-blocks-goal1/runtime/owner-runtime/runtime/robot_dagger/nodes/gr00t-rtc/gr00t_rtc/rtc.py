from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class PendingChunk:
    generation: int
    queued_at_submit: int


class RtcQueue:
    """Small action queue; model RPC/threading stays outside this pure class."""

    def __init__(self, action_dim: int = 37, trigger_steps: int = 12):
        if action_dim < 1 or trigger_steps < 0:
            raise ValueError("action_dim must be positive and trigger_steps non-negative")
        self.action_dim = action_dim
        self.trigger_steps = trigger_steps
        self.generation = 0
        self.actions: deque[np.ndarray] = deque()
        self.step_indices: deque[int] = deque()
        self.pending: PendingChunk | None = None

    def reset(self, generation: int) -> None:
        self.generation = int(generation)
        self.actions.clear()
        self.step_indices.clear()
        self.pending = None

    def replace(self, chunk: np.ndarray, start_index: int = 0) -> None:
        chunk = self._validate(chunk)
        self.actions.clear()
        self.step_indices.clear()
        self.actions.extend(row.copy() for row in chunk)
        self.step_indices.extend(range(start_index, start_index + len(chunk)))

    def begin_inference(self) -> tuple[np.ndarray, PendingChunk]:
        if self.pending is not None:
            raise RuntimeError("inference already pending")
        snapshot = self.remaining()
        self.pending = PendingChunk(self.generation, len(snapshot))
        return snapshot, self.pending

    def fail_inference(self, pending: PendingChunk) -> bool:
        if self.pending != pending:
            return False
        self.pending = None
        return True

    def accept(self, chunk: np.ndarray, pending: PendingChunk) -> int | None:
        if self.pending != pending or pending.generation != self.generation:
            return None
        chunk = self._validate(chunk)
        consumed = max(0, pending.queued_at_submit - len(self.actions))
        self.pending = None
        if consumed >= len(chunk):
            self.actions.clear()
            self.step_indices.clear()
            return 0
        self.replace(chunk[consumed:], start_index=consumed)
        return consumed

    def pop(self) -> np.ndarray | None:
        item = self.pop_with_index()
        return item[1] if item is not None else None

    def pop_with_index(self) -> tuple[int, np.ndarray] | None:
        if not self.actions:
            return None
        return self.step_indices.popleft(), self.actions.popleft()

    def remaining(self) -> np.ndarray:
        if not self.actions:
            return np.empty((0, self.action_dim), dtype=np.float32)
        return np.stack(self.actions).astype(np.float32, copy=False)

    @property
    def should_infer(self) -> bool:
        return self.pending is None and len(self.actions) <= self.trigger_steps

    def _validate(self, chunk: np.ndarray) -> np.ndarray:
        chunk = np.asarray(chunk, dtype=np.float32)
        if chunk.ndim != 2 or chunk.shape[1] != self.action_dim or not len(chunk):
            raise ValueError(f"expected non-empty (T,{self.action_dim}) chunk, got {chunk.shape}")
        if not np.isfinite(chunk).all():
            raise ValueError("action chunk contains non-finite values")
        return chunk


def frozen_steps(latencies_ms, action_hz: float, margin_steps: int, overlap_steps: int) -> int:
    if overlap_steps <= 0:
        return 0
    worst_ms = max(latencies_ms, default=0.0)
    predicted = math.ceil(worst_ms * action_hz / 1000.0) + margin_steps
    return max(0, min(int(predicted), int(overlap_steps)))
