"""Versioned trial-result records shared with experiment controllers."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field, field_validator, model_validator

SCHEMA_VERSION = "xpolicy_interactive.v1"


class TrialResult(BaseModel):
    schema_version: Literal["xpolicy_interactive.v1"] = SCHEMA_VERSION
    evaluation_id: str = Field(min_length=1)
    checkpoint_id: str = Field(min_length=1)
    trial_id: str = Field(min_length=1)
    task: str = Field(min_length=1)
    seed: int = Field(ge=0)
    repeat_index: int = Field(ge=0)
    evidence_type: Literal[
        "task_outcome", "protocol_debug", "replay_validation"
    ] = "task_outcome"
    outcome: Literal["success", "failure", "aborted", "timeout", "invalid"]
    success: float | None = Field(default=None, ge=0.0, le=1.0)
    episode_steps: int | None = Field(default=None, ge=0)
    termination_reason: str = ""
    metrics: dict[str, float] = Field(default_factory=dict)
    artifacts: dict[str, str | None] = Field(default_factory=dict)

    @field_validator("metrics")
    @classmethod
    def finite_metrics(cls, value: dict[str, float]) -> dict[str, float]:
        for name, metric in value.items():
            if not name or isinstance(metric, bool) or not math.isfinite(float(metric)):
                raise ValueError(
                    "metrics require non-empty names and finite numeric values"
                )
        return {name: float(metric) for name, metric in value.items()}

    @model_validator(mode="after")
    def outcome_matches_success(self):
        if self.outcome == "success" and self.success != 1.0:
            raise ValueError("success outcome requires success=1.0")
        if self.outcome == "failure" and self.success != 0.0:
            raise ValueError("failure outcome requires success=0.0")
        if self.evidence_type == "protocol_debug" and (
            self.outcome != "invalid" or self.success is not None
        ):
            raise ValueError("protocol_debug evidence requires invalid outcome")
        if self.evidence_type == "replay_validation" and self.outcome not in {
            "success",
            "failure",
        }:
            raise ValueError(
                "replay_validation evidence requires success or failure outcome"
            )
        return self


class TrialResultJsonlWriter:
    """Append validated records and flush each episode for crash recovery."""

    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser().resolve()

    def append(self, result: TrialResult | dict) -> TrialResult:
        result = (
            result
            if isinstance(result, TrialResult)
            else TrialResult.model_validate(result)
        )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(result.model_dump(mode="json"), sort_keys=True))
            stream.write("\n")
            stream.flush()
        return result
