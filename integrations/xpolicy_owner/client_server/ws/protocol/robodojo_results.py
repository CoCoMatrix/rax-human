"""Convert RoboDojo's authoritative result file into bridge trial records."""

from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path
from typing import Any

from client_server.ws.protocol.client import PolicyEvalClient, PolicyEvalClientConfig
from client_server.ws.protocol.trial_results import TrialResult, TrialResultJsonlWriter

_MAX_RESULT_BYTES = 16 * 1024 * 1024


def _ordered_details(details: Any) -> list[tuple[int, dict[str, Any]]]:
    if not isinstance(details, dict):
        raise TypeError("RoboDojo result details must be an object")
    ordered: list[tuple[int, dict[str, Any]]] = []
    for key, value in details.items():
        if isinstance(key, bool):
            raise TypeError("RoboDojo detail indices must be integers")
        try:
            index = int(key)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid RoboDojo detail index: {key!r}") from exc
        if str(index) != str(key) and key != index:
            raise ValueError(f"invalid RoboDojo detail index: {key!r}")
        if index < 0 or not isinstance(value, dict):
            raise ValueError("RoboDojo details require non-negative indices and objects")
        ordered.append((index, value))
    ordered.sort(key=lambda item: item[0])
    if [index for index, _ in ordered] != list(range(len(ordered))):
        raise ValueError("RoboDojo detail indices must be contiguous from zero")
    return ordered


def load_robodojo_trial_results(
    result_path: str | Path,
    *,
    evaluation_id: str,
    checkpoint_id: str,
    task: str,
    seed: int,
) -> list[TrialResult]:
    """Load and cross-check a completed RoboDojo ``_result.json`` file."""
    path = Path(result_path).expanduser().resolve()
    try:
        if path.stat().st_size > _MAX_RESULT_BYTES:
            raise ValueError(
                f"RoboDojo result exceeds {_MAX_RESULT_BYTES} bytes: {path}"
            )
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"RoboDojo result not found: {path}") from None
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid RoboDojo result JSON: {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise TypeError("RoboDojo result root must be an object")

    ordered = _ordered_details(payload.get("details"))
    eval_time = payload.get("eval_time")
    if isinstance(eval_time, bool) or not isinstance(eval_time, int) or eval_time < 0:
        raise ValueError("RoboDojo eval_time must be a non-negative integer")
    if eval_time != len(ordered):
        raise ValueError(
            f"RoboDojo eval_time={eval_time} does not match {len(ordered)} details"
        )

    records: list[TrialResult] = []
    layout_ids: set[int] = set()
    success_count = 0
    for repeat_index, detail in ordered:
        layout_id = detail.get("layout_id")
        if isinstance(layout_id, bool) or not isinstance(layout_id, int) or layout_id < 0:
            raise ValueError("RoboDojo layout_id must be a non-negative integer")
        if layout_id in layout_ids:
            raise ValueError(f"duplicate RoboDojo layout_id: {layout_id}")
        layout_ids.add(layout_id)

        succeeded = detail.get("success")
        if not isinstance(succeeded, bool):
            raise TypeError("RoboDojo success must be a boolean")
        score = detail.get("score")
        if isinstance(score, bool) or not isinstance(score, (int, float)):
            raise TypeError("RoboDojo score must be numeric")
        score = float(score)
        if not math.isfinite(score):
            raise ValueError("RoboDojo score must be finite")
        success_count += int(succeeded)
        records.append(
            TrialResult(
                evaluation_id=evaluation_id,
                checkpoint_id=checkpoint_id,
                trial_id=(
                    f"{evaluation_id}-{task}-repeat-{repeat_index}-layout-{layout_id}"
                ),
                task=task,
                seed=seed,
                repeat_index=repeat_index,
                outcome="success" if succeeded else "failure",
                success=1.0 if succeeded else 0.0,
                termination_reason=(
                    "robodojo_task_success" if succeeded else "robodojo_task_failure"
                ),
                metrics={"robodojo_score": score},
                artifacts={"robodojo_result_json": str(path)},
            )
        )

    success_rate = payload.get("success_rate")
    if isinstance(success_rate, bool) or not isinstance(success_rate, (int, float)):
        raise TypeError("RoboDojo success_rate must be numeric")
    expected_rate = success_count / eval_time if eval_time else 0.0
    if not math.isfinite(float(success_rate)) or not math.isclose(
        float(success_rate), expected_rate, rel_tol=1e-9, abs_tol=1e-9
    ):
        raise ValueError(
            "RoboDojo success_rate is inconsistent with episode details"
        )
    return records


async def send_trial_end_events(
    policy_server_url: str, records: list[TrialResult]
) -> None:
    """Send validated outcomes while the policy server is still available."""
    if not records:
        return
    client = PolicyEvalClient(
        PolicyEvalClientConfig(
            url=policy_server_url,
            evaluation_id=records[0].evaluation_id,
        )
    )
    await client.connect(handshake=True)
    try:
        for record in records:
            await client.trial_end(
                trial_id=record.trial_id,
                repeat_index=record.repeat_index,
                result=record.model_dump(mode="json"),
            )
    finally:
        await client.close()


def publish_robodojo_trial_results(
    records: list[TrialResult],
    output_path: str | Path,
    *,
    policy_server_url: str | None = None,
) -> None:
    """Publish events first, then persist the validated batch to JSONL."""
    if policy_server_url:
        asyncio.run(send_trial_end_events(policy_server_url, records))
    writer = TrialResultJsonlWriter(output_path)
    for record in records:
        writer.append(record)
