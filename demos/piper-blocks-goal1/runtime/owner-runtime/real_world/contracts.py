from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any

import numpy as np


def _readonly_array(
    value: Any,
    *,
    name: str,
    dtype: np.dtype | type | None = None,
    ndim: int | None = None,
    finite: bool = False,
) -> np.ndarray:
    array = np.array(value, dtype=dtype, copy=True)
    if ndim is not None and array.ndim != ndim:
        raise ValueError(f"{name} must have {ndim} dimensions, got {array.shape}")
    if finite and not np.isfinite(array).all():
        raise ValueError(f"{name} contains NaN or Inf")
    array.setflags(write=False)
    return array


@dataclass(frozen=True, slots=True)
class VectorSchema:
    """Names and semantics for one ordered robot vector."""

    schema_id: str
    fields: tuple[str, ...]
    units: tuple[str, ...]
    semantics: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.schema_id:
            raise ValueError("schema_id must be non-empty")
        if not self.fields or len(set(self.fields)) != len(self.fields):
            raise ValueError("schema fields must be non-empty and unique")
        if len(self.units) != self.dim or len(self.semantics) != self.dim:
            raise ValueError("schema fields, units, and semantics must have equal lengths")

    @property
    def dim(self) -> int:
        return len(self.fields)

    def coerce(self, values: Any, *, name: str, ndim: int) -> np.ndarray:
        array = _readonly_array(
            values,
            name=name,
            dtype=np.float32,
            ndim=ndim,
            finite=True,
        )
        if array.shape[-1] != self.dim:
            raise ValueError(
                f"{name} must end with {self.dim} values for {self.schema_id}, "
                f"got {array.shape}"
            )
        return array

    def to_wire(self) -> dict[str, Any]:
        return {
            "schema_id": self.schema_id,
            "fields": list(self.fields),
            "units": list(self.units),
            "semantics": list(self.semantics),
        }

    @classmethod
    def from_wire(cls, value: Mapping[str, Any]) -> VectorSchema:
        return cls(
            schema_id=str(value["schema_id"]),
            fields=tuple(map(str, value["fields"])),
            units=tuple(map(str, value["units"])),
            semantics=tuple(map(str, value["semantics"])),
        )


@dataclass(frozen=True, slots=True)
class ObservationFrame:
    """One immutable robot observation ready for a policy request."""

    state_schema: VectorSchema
    state: np.ndarray
    images: Mapping[str, np.ndarray]
    instruction: str
    timestamp_ns: int
    sequence: int
    robot_type: str
    robot_id: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.robot_type:
            raise ValueError("robot_type must be non-empty")
        if self.timestamp_ns < 0 or self.sequence < 0:
            raise ValueError("timestamp_ns and sequence must be non-negative")
        object.__setattr__(
            self,
            "state",
            self.state_schema.coerce(self.state, name="observation state", ndim=1),
        )
        images = {
            str(name): _readonly_array(image, name=f"image {name}")
            for name, image in self.images.items()
        }
        object.__setattr__(self, "images", MappingProxyType(images))
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))

    def to_policy_observation(self) -> dict[str, Any]:
        return {
            "data_format_version": "xpolicylab.real_world.v1",
            "instruction": self.instruction,
            "vision": {
                name: {"color": image}
                for name, image in self.images.items()
            },
            "state": {
                "vector": self.state,
                "schema": self.state_schema.to_wire(),
            },
            "additional_info": {
                "timestamp_ns": self.timestamp_ns,
                "sequence": self.sequence,
                "robot_type": self.robot_type,
                "robot_id": self.robot_id,
                "metadata": dict(self.metadata),
            },
        }

    @classmethod
    def from_policy_observation(
        cls, observation: Mapping[str, Any]
    ) -> ObservationFrame:
        state = observation["state"]
        vision = observation.get("vision", {})
        info = observation.get("additional_info", {})
        return cls(
            state_schema=VectorSchema.from_wire(state["schema"]),
            state=state["vector"],
            images={
                str(name): camera["color"]
                for name, camera in vision.items()
                if "color" in camera
            },
            instruction=str(observation.get("instruction", "")),
            timestamp_ns=int(info.get("timestamp_ns", 0)),
            sequence=int(info.get("sequence", 0)),
            robot_type=str(info["robot_type"]),
            robot_id=str(info.get("robot_id", "")),
            metadata=info.get("metadata", {}),
        )


@dataclass(frozen=True, slots=True)
class ActionChunk:
    """An immutable raw or processed action chunk."""

    schema: VectorSchema
    values: np.ndarray
    frequency_hz: float
    generation: int = 0
    request_id: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        frequency_hz = float(self.frequency_hz)
        if not np.isfinite(frequency_hz) or frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        if self.generation < 0:
            raise ValueError("generation must be non-negative")
        values = self.schema.coerce(self.values, name="action chunk", ndim=2)
        if values.shape[0] == 0:
            raise ValueError("action chunk must contain at least one step")
        object.__setattr__(self, "frequency_hz", frequency_hz)
        object.__setattr__(
            self,
            "values",
            values,
        )
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))

    def to_policy_result(self) -> dict[str, Any]:
        """Return a PolicyServer-compatible result payload."""
        return {
            "actions": self.values,
            "action_schema": self.schema.to_wire(),
            "action_frequency_hz": self.frequency_hz,
            "policy_metadata": dict(self.metadata),
        }

    @classmethod
    def from_policy_result(
        cls,
        payload: Mapping[str, Any],
        *,
        generation: int = 0,
        request_id: str = "",
    ) -> ActionChunk:
        return cls(
            schema=VectorSchema.from_wire(payload["action_schema"]),
            values=payload["actions"],
            frequency_hz=float(payload["action_frequency_hz"]),
            generation=generation,
            request_id=request_id,
            metadata=payload.get("policy_metadata", {}),
        )
