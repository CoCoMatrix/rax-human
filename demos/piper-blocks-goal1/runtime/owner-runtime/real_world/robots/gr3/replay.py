from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path
from statistics import mean
from typing import Any

import numpy as np

_XPOLICYLAB_ROOT = Path(__file__).resolve().parents[3]
if str(_XPOLICYLAB_ROOT) not in sys.path:
    sys.path.insert(0, str(_XPOLICYLAB_ROOT))

from XPolicyLab.real_world.contracts import ActionChunk, ObservationFrame
from XPolicyLab.real_world.robots.gr3.mapping import (
    ACTION_NAMES,
    GR3_STATE_SCHEMA,
    STATE_NAMES,
    build_observation,
)

from client_server.ws.protocol.trial_results import (
    TrialResult,
    TrialResultJsonlWriter,
)

_MAX_MANIFEST_BYTES = 1024 * 1024
_MAX_MANIFEST_EPISODES = 1000


class Gr3EpisodeReplay:
    """Read recorded GR3 observations without starting robot hardware."""

    def __init__(self, episode_dir: str | Path):
        import pyarrow.parquet as pq

        self.episode_dir = Path(episode_dir).expanduser().resolve()
        with (self.episode_dir / "schema.json").open(encoding="utf-8") as file:
            schema = json.load(file)
        with (self.episode_dir / "metadata.json").open(encoding="utf-8") as file:
            self.metadata = json.load(file)
        self._anygrasp_dataset = None
        self._anygrasp_indices: list[int] = []
        if schema.get("schema_version") == "gr3_anygrasp_reference_v1":
            if (
                tuple(schema.get("state_order", ())) != STATE_NAMES
                or tuple(schema.get("action_order", ())) != ACTION_NAMES
                or self.metadata.get("actuation_enabled") is not False
            ):
                raise ValueError("AnyGrasp replay reference has an invalid GR3/safety contract")
            reference = schema.get("reference", {})
            turbovla_root = Path(str(reference.get("turbovla_root", ""))).expanduser()
            manifest_path = Path(str(reference.get("dataset_manifest", ""))).expanduser()
            if turbovla_root.is_symlink() or manifest_path.is_symlink():
                raise ValueError("AnyGrasp replay references cannot be symlinks")
            turbovla_root = turbovla_root.resolve()
            manifest_path = manifest_path.resolve()
            if not (turbovla_root / "turbovla/data/gr3_anygrasp.py").is_file():
                raise FileNotFoundError("AnyGrasp replay TurboVLA runtime is unavailable")
            if not manifest_path.is_file():
                raise FileNotFoundError("AnyGrasp replay dataset manifest is unavailable")
            if str(turbovla_root) not in sys.path:
                sys.path.insert(0, str(turbovla_root))
            from turbovla.data.gr3_anygrasp import Gr3AnygraspDataset
            from turbovla.data.gr3_common import Gr3NormalizationStats

            dummy_stats = Gr3NormalizationStats(
                state_mean=np.zeros(len(STATE_NAMES), dtype=np.float32),
                state_std=np.ones(len(STATE_NAMES), dtype=np.float32),
                action_low=np.full(len(ACTION_NAMES), -1.0, dtype=np.float32),
                action_high=np.full(len(ACTION_NAMES), 1.0, dtype=np.float32),
            )
            self._anygrasp_dataset = Gr3AnygraspDataset(
                manifest_path,
                horizon=1,
                image_size=224,
                stats=dummy_stats,
            )
            if reference.get("dataset_id") != self._anygrasp_dataset.manifest.get("dataset_id"):
                raise ValueError("AnyGrasp replay dataset identity mismatch")
            raw_indices = reference.get("sample_indices")
            if not isinstance(raw_indices, list) or not raw_indices:
                raise ValueError("AnyGrasp replay requires non-empty sample_indices")
            if any(isinstance(value, bool) or not isinstance(value, int) for value in raw_indices):
                raise ValueError("AnyGrasp replay sample indices must be integers")
            self._anygrasp_indices = list(raw_indices)
            if len(set(self._anygrasp_indices)) != len(self._anygrasp_indices):
                raise ValueError("AnyGrasp replay sample indices must be unique")
            if min(self._anygrasp_indices) < 0 or max(self._anygrasp_indices) >= len(
                self._anygrasp_dataset
            ):
                raise IndexError("AnyGrasp replay sample index is outside the dataset")
            return
        if (
            schema.get("schema_version") != "gr3_dagger_v2"
            or tuple(schema.get("state_order", ())) != STATE_NAMES
            or tuple(schema.get("action_order", ())) != ACTION_NAMES
        ):
            raise ValueError("episode does not use the GR3 dagger v2 vector layout")

        self.width = int(schema["camera_top"]["width"])
        self.height = int(schema["camera_top"]["height"])
        if schema["camera_top"].get("encoding") != "bgr8":
            raise ValueError("GR3 replay currently requires raw bgr8 camera frames")
        self.camera = pq.read_table(
            self.episode_dir / "observation.images.camera_top.parquet",
            columns=["source_timestamp_ns", "observation.images.camera_top"],
        )
        self.state = pq.read_table(
            self.episode_dir / "observation.state.parquet",
            columns=["source_timestamp_ns", "observation.state"],
        )
        self.base_state = pq.read_table(
            self.episode_dir / "observation.base_state.parquet",
            columns=["source_timestamp_ns", "observation.base_state"],
        )
        self._state_timestamps = np.asarray(self.state["source_timestamp_ns"])
        self._base_timestamps = np.asarray(self.base_state["source_timestamp_ns"])

    def __len__(self) -> int:
        if self._anygrasp_dataset is not None:
            return len(self._anygrasp_indices)
        return len(self.camera)

    @staticmethod
    def _nearest(timestamps: np.ndarray, target: int) -> int:
        index = int(np.searchsorted(timestamps, target))
        candidates = [value for value in (index - 1, index) if 0 <= value < len(timestamps)]
        return min(candidates, key=lambda value: abs(int(timestamps[value]) - target))

    def frame(self, index: int) -> ObservationFrame:
        if not 0 <= index < len(self):
            raise IndexError(f"camera frame {index} is outside [0, {len(self)})")
        if self._anygrasp_dataset is not None:
            source_index = self._anygrasp_indices[index]
            observation = self._anygrasp_dataset.raw_observation(source_index)
            metadata = observation["metadata"]
            timestamp_ns = int(round(int(metadata["frame_index"]) * 1_000_000_000 / 30))
            return ObservationFrame(
                state_schema=GR3_STATE_SCHEMA,
                state=GR3_STATE_SCHEMA.coerce(
                    observation["state"], name="AnyGrasp GR3 state", ndim=1
                ),
                images={"cam_head": np.ascontiguousarray(observation["image"])},
                instruction=str(observation["lang"]),
                timestamp_ns=timestamp_ns,
                sequence=index,
                robot_type="gr3",
                robot_id="gr3-anygrasp-recorded",
                metadata={
                    "replay": True,
                    "source_schema": "gr3_anygrasp_reference_v1",
                    "source_sample_index": source_index,
                    **metadata,
                },
            )
        timestamp_ns = int(self.camera["source_timestamp_ns"][index].as_py())
        state_index = self._nearest(self._state_timestamps, timestamp_ns)
        base_index = self._nearest(self._base_timestamps, timestamp_ns)
        joints = {
            item["name"]: float(item["value"])
            for item in self.state["observation.state"][state_index].as_py()
        }
        base_rows: list[dict[str, Any]] = self.base_state[
            "observation.base_state"
        ][base_index].as_py()
        image = np.asarray(
            self.camera["observation.images.camera_top"][index].as_py(),
            dtype=np.uint8,
        ).reshape(self.height, self.width, 3)
        return build_observation(
            joints,
            base_rows[0],
            image,
            str(self.metadata.get("task_instruction", "")),
            timestamp_ns=timestamp_ns,
            sequence=index,
            robot_id=str(self.metadata.get("robot_id", "")),
            bgr=True,
            metadata={
                "replay": True,
                "episode_id": self.metadata.get("episode_id", self.episode_dir.name),
                "state_time_offset_ns": int(self._state_timestamps[state_index])
                - timestamp_ns,
                "base_time_offset_ns": int(self._base_timestamps[base_index])
                - timestamp_ns,
            },
        )


def _load_episode_manifest(path: str | Path) -> list[dict[str, Any]]:
    source = Path(path).expanduser()
    if source.is_symlink():
        raise ValueError("replay episode manifest cannot be a symlink")
    source = source.resolve()
    if source.stat().st_size > _MAX_MANIFEST_BYTES:
        raise ValueError("replay episode manifest is too large")
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, list) or not payload:
        raise ValueError("replay episode manifest must be a non-empty list")
    if len(payload) > _MAX_MANIFEST_EPISODES:
        raise ValueError("replay episode manifest has too many entries")
    entries: list[dict[str, Any]] = []
    for index, entry in enumerate(payload):
        if not isinstance(entry, dict):
            raise TypeError(f"replay manifest entry {index} must be an object")
        episode_dir = entry.get("episode_dir")
        task = entry.get("task")
        repeat_index = entry.get("repeat_index")
        trial_id = entry.get("trial_id")
        if not isinstance(episode_dir, str) or not episode_dir:
            raise ValueError(f"replay manifest entry {index} needs episode_dir")
        if not isinstance(task, str) or not task:
            raise ValueError(f"replay manifest entry {index} needs task")
        if (
            isinstance(repeat_index, bool)
            or not isinstance(repeat_index, int)
            or repeat_index < 0
        ):
            raise ValueError(f"replay manifest entry {index} has invalid repeat_index")
        if not isinstance(trial_id, str) or not trial_id:
            raise ValueError(f"replay manifest entry {index} needs trial_id")
        entries.append(
            {
                "episode_dir": episode_dir,
                "task": task,
                "repeat_index": repeat_index,
                "trial_id": trial_id,
            }
        )
    return entries


def validate_replay_episode(
    replay: Gr3EpisodeReplay,
    client: Any,
    *,
    indices: list[int],
    evaluation_id: str,
    checkpoint_id: str,
    trial_id: str,
    task: str,
    repeat_index: int,
    writer: TrialResultJsonlWriter | None = None,
) -> tuple[TrialResult, list[dict[str, Any]]]:
    """Validate policy outputs for recorded frames without claiming task success."""
    client.set_trial_context(trial_id=trial_id, repeat_index=repeat_index)
    client.call("reset")
    summaries: list[dict[str, Any]] = []
    action_schema: tuple[str, tuple[str, ...]] | None = None
    chunk_steps: list[float] = []
    action_l2: list[float] = []
    error: Exception | None = None

    try:
        for index in indices:
            frame = replay.frame(index)
            response = client.infer(frame.to_policy_observation())
            chunk = ActionChunk.from_policy_result(response.payload)
            current_schema = (chunk.schema.schema_id, chunk.schema.fields)
            if action_schema is None:
                action_schema = current_schema
            elif current_schema != action_schema:
                raise ValueError("policy action schema changed during replay")
            values = np.asarray(chunk.values, dtype=np.float32)
            norm = float(np.linalg.norm(values))
            chunk_steps.append(float(values.shape[0]))
            action_l2.append(norm)
            summary = {
                "frame": index,
                "timestamp_ns": frame.timestamp_ns,
                "action_shape": list(chunk.values.shape),
                "action_min": float(values.min()),
                "action_max": float(values.max()),
                "action_l2": norm,
                "action_sha256": hashlib.sha256(
                    np.ascontiguousarray(values).tobytes()
                ).hexdigest()[:16],
                "latency_ms": response.payload.get("latency_ms"),
                "policy": chunk.metadata.get("policy"),
            }
            summaries.append(summary)
    # Any per-episode decoder, transport, or action-contract error is gate evidence.
    except Exception as exc:  # noqa: BLE001
        error = exc

    latencies = client.inference_latencies_ms
    passed = error is None and len(summaries) == len(indices)
    metrics: dict[str, float] = {
        "replay_frame_count": float(len(summaries)),
    }
    if latencies:
        metrics["inference_latency_ms"] = mean(latencies)
        metrics["cold_start_inference_latency_ms"] = latencies[0]
        warm_latencies = latencies[1:] or latencies
        metrics["warm_inference_latency_ms"] = mean(warm_latencies)
        metrics["warm_inference_latency_p95_ms"] = float(
            np.quantile(warm_latencies, 0.95)
        )
    if chunk_steps:
        metrics["action_chunk_steps_mean"] = mean(chunk_steps)
        metrics["action_l2_max"] = max(action_l2)
    result = TrialResult(
        evaluation_id=evaluation_id,
        checkpoint_id=checkpoint_id,
        trial_id=trial_id,
        task=task,
        seed=0,
        repeat_index=repeat_index,
        evidence_type="replay_validation",
        outcome="success" if passed else "failure",
        success=1.0 if passed else 0.0,
        episode_steps=len(summaries),
        termination_reason=(
            "replay_contract_valid"
            if passed
            else f"replay_validation_failed:{type(error).__name__}"
        ),
        metrics=metrics,
        artifacts={"recorded_episode": str(replay.episode_dir)},
    )
    client.call("trial_end", obs=result.model_dump(mode="json"))
    if writer is not None:
        writer.append(result)
    if error is not None:
        print(f"[REPLAY] {trial_id} failed: {type(error).__name__}: {error}")
        if writer is None:
            raise error
    return result, summaries


def main() -> None:
    from client_server.ws.model_client import WsModelClient

    parser = argparse.ArgumentParser(description="Replay GR3 observations through XPolicyLab")
    parser.add_argument("episode_dir", nargs="?")
    parser.add_argument("--episode-manifest")
    parser.add_argument("--url", default="ws://127.0.0.1:19001")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument(
        "--evaluation-id",
        default=os.environ.get("XPOLICYLAB_EVALUATION_ID", "gr3-replay"),
    )
    parser.add_argument(
        "--checkpoint-id",
        default=os.environ.get("XPOLICYLAB_CHECKPOINT_ID", "untracked-checkpoint"),
    )
    parser.add_argument("--task")
    parser.add_argument("--trial-id")
    parser.add_argument(
        "--output-jsonl", default=os.environ.get("XPOLICYLAB_TRIAL_RESULT_JSONL")
    )
    args = parser.parse_args()
    if args.count < 1 or args.stride < 1:
        parser.error("--count and --stride must be positive")
    if bool(args.episode_dir) == bool(args.episode_manifest):
        parser.error("provide exactly one episode_dir or --episode-manifest")
    if args.episode_manifest:
        entries = _load_episode_manifest(args.episode_manifest)
    else:
        assert args.episode_dir is not None
        entries = [
            {
                "episode_dir": args.episode_dir,
                "task": args.task or "recorded_gr3",
                "repeat_index": 0,
                "trial_id": args.trial_id or Path(args.episode_dir).name,
            }
        ]
    writer = TrialResultJsonlWriter(args.output_jsonl) if args.output_jsonl else None
    first = entries[0]
    with WsModelClient(
        url=args.url,
        evaluation_id=args.evaluation_id,
        trial_id=first["trial_id"],
    ) as client:
        for entry in entries:
            replay = Gr3EpisodeReplay(entry["episode_dir"])
            indices = list(
                range(
                    args.start,
                    args.start + args.count * args.stride,
                    args.stride,
                )
            )
            _, summaries = validate_replay_episode(
                replay,
                client,
                indices=indices,
                evaluation_id=args.evaluation_id,
                checkpoint_id=args.checkpoint_id,
                trial_id=entry["trial_id"],
                task=entry["task"],
                repeat_index=entry["repeat_index"],
                writer=writer,
            )
            for summary in summaries:
                print(json.dumps(summary))


if __name__ == "__main__":
    main()
