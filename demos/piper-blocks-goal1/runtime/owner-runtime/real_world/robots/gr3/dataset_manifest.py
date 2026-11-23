"""Build auditable dataset manifests from recorded GR3 DAgger episodes."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import subprocess
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "xpolicy_dataset.v1"
PROFILE_ID = "xpolicylab.gr3_dagger_v2"
MAX_EPISODES = 10_000
ALIGNMENT_LIMIT_MS = 20.0
TYPED_PARAMETER_KEYS = {"Bool", "Float", "Integer", "String", "Timestamp"}


def _unwrap_parameter(value: Any) -> Any:
    if isinstance(value, list):
        return [_unwrap_parameter(item) for item in value]
    if isinstance(value, dict):
        if len(value) == 1:
            key, item = next(iter(value.items()))
            if key in TYPED_PARAMETER_KEYS:
                return _unwrap_parameter(item)
        return {key: _unwrap_parameter(item) for key, item in value.items()}
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _stable_digest(value: Any) -> str:
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def _git_commit(path: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def _read_table(episode: Path, stream: str, columns: list[str]):
    import pyarrow.parquet as pq

    path = episode / f"{stream}.parquet"
    if path.is_symlink() or not path.is_file():
        raise FileNotFoundError(f"missing regular stream: {path}")
    return pq.read_table(path, columns=columns)


def _timestamps(episode: Path, stream: str) -> list[int]:
    table = _read_table(episode, stream, ["source_timestamp_ns"])
    return [int(value.as_py()) for value in table["source_timestamp_ns"]]


def _nearest_errors_ms(targets: list[int], candidates: list[int]) -> list[float]:
    if not targets or not candidates:
        raise ValueError("alignment requires non-empty timestamp streams")
    ordered = sorted(candidates)
    errors = []
    for target in targets:
        index = bisect.bisect_left(ordered, target)
        neighbors = ordered[max(0, index - 1) : index + 1]
        errors.append(min(abs(target - value) for value in neighbors) / 1e6)
    return errors


def _scalar_stream_counts(episode: Path, stream: str) -> dict[str, int]:
    table = _read_table(episode, stream, [stream])
    counts: Counter[str] = Counter()
    for scalar in table[stream]:
        value = _unwrap_parameter(scalar.as_py())
        if isinstance(value, list) and len(value) == 1:
            value = value[0]
        if isinstance(value, dict) and set(value) == {"value"}:
            value = value["value"]
        if isinstance(value, bool):
            key = "true" if value else "false"
        else:
            key = str(value)
        counts[key] += 1
    return dict(sorted(counts.items()))


def _file_manifest(episode: Path) -> list[dict[str, Any]]:
    entries = []
    for path in sorted(item for item in episode.iterdir() if item.is_file()):
        if path.is_symlink():
            raise ValueError(f"episode cannot contain symlink files: {path}")
        entries.append(
            {
                "path": path.name,
                "size": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        )
    return entries


def scan_episode(episode_path: str | Path) -> dict[str, Any]:
    episode = Path(episode_path).expanduser()
    if episode.is_symlink():
        raise ValueError(f"episode cannot be a symlink: {episode}")
    episode = episode.resolve()
    if not episode.is_dir():
        raise FileNotFoundError(f"episode not found: {episode}")
    metadata_path = episode / "metadata.json"
    schema_path = episode / "schema.json"
    if (
        metadata_path.is_symlink()
        or schema_path.is_symlink()
        or not metadata_path.is_file()
        or not schema_path.is_file()
    ):
        raise FileNotFoundError(
            "episode requires regular metadata.json and schema.json"
        )
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    if metadata.get("schema_version") != "gr3_dagger_v2":
        raise ValueError("metadata does not declare gr3_dagger_v2")
    if (
        schema.get("schema_version") != "gr3_dagger_v2"
        or schema.get("state_dim") != 33
        or schema.get("action_dim") != 37
    ):
        raise ValueError("unsupported GR3 DAgger schema")

    camera_ts = _timestamps(episode, "observation.images.camera_top")
    state_ts = _timestamps(episode, "observation.state_robot")
    expert_ts = _timestamps(episode, "action.expert_safe_robot")
    if not camera_ts or not state_ts or not expert_ts:
        raise ValueError(
            "camera, canonical state, and expert streams must be non-empty"
        )
    training_camera_ts = [value for value in camera_ts if value <= state_ts[-1]]
    trailing_frames = len(camera_ts) - len(training_camera_ts)
    state_errors = _nearest_errors_ms(training_camera_ts, state_ts)
    expert_errors = _nearest_errors_ms(training_camera_ts, expert_ts)
    max_state_error = max(state_errors)
    max_expert_error = max(expert_errors)

    source_counts = _scalar_stream_counts(episode, "control.selected_action_source")
    mode_counts = _scalar_stream_counts(episode, "control.mode")
    label_counts = _scalar_stream_counts(episode, "label.valid")
    valid_labels = label_counts.get("true", 0)
    warnings: list[str] = []
    exclusion_reasons: list[str] = []
    filters: list[dict[str, Any]] = []
    if trailing_frames:
        warnings.append("camera_contains_post_termination_audit_tail")
        filters.append(
            {
                "name": "drop_camera_without_state_alignment",
                "threshold_ms": ALIGNMENT_LIMIT_MS,
                "dropped_frames": trailing_frames,
            }
        )
    if max_state_error > ALIGNMENT_LIMIT_MS:
        exclusion_reasons.append("canonical_state_alignment_exceeds_20ms")
    if max_expert_error > ALIGNMENT_LIMIT_MS:
        exclusion_reasons.append("expert_alignment_exceeds_20ms")
    if valid_labels < 1:
        exclusion_reasons.append("no_valid_expert_labels")
    if metadata.get("recording_saved_successfully") is not True:
        exclusion_reasons.append("recording_not_confirmed_saved")
    if not training_camera_ts:
        exclusion_reasons.append("no_trainable_camera_frames")

    files = _file_manifest(episode)
    file_digest = _stable_digest(files)
    outcome = str(metadata.get("task_outcome", "UNKNOWN")).lower()
    if outcome not in {"success", "failure", "aborted", "unknown"}:
        outcome = "unknown"
    result = {
        "episode_id": str(metadata.get("episode_id") or episode.name),
        "path": str(episode),
        "source_schema": "gr3_dagger_v2",
        "profile_id": PROFILE_ID,
        "file_manifest_sha256": file_digest,
        "files": files,
        "task_id": str(metadata.get("task_id", "")),
        "task_instruction": str(metadata.get("task_instruction", "")),
        "task_outcome": outcome,
        "outcome_confirmed_by_operator": bool(
            metadata.get("outcome_confirmed_by_operator", False)
        ),
        "termination_reason": str(metadata.get("termination_reason", "")),
        "recording_saved_successfully": bool(
            metadata.get("recording_saved_successfully", False)
        ),
        "provenance": {
            "model_backend": str(metadata.get("model_backend", "")),
            "model_checkpoint": str(metadata.get("model_checkpoint", "")),
            "dagger_iteration": str(metadata.get("dagger_iteration", "")),
        },
        "statistics": {
            "intervention_count": int(metadata.get("intervention_count", 0)),
            "camera_frames": len(camera_ts),
            "trainable_camera_frames": len(training_camera_ts),
            "valid_label_rows": valid_labels,
        },
        "profile_data": {
            "state_dim": 33,
            "action_dim": 37,
            "camera_frames": len(camera_ts),
            "trainable_camera_frames": len(training_camera_ts),
            "trailing_audit_frames": trailing_frames,
            "valid_label_rows": valid_labels,
            "label_rows": sum(label_counts.values()),
            "selected_action_source_counts": source_counts,
            "control_mode_counts": mode_counts,
            "alignment": {
                "limit_ms": ALIGNMENT_LIMIT_MS,
                "canonical_state_max_ms_after_filter": max_state_error,
                "expert_max_ms_after_filter": max_expert_error,
                "raw_camera_state_max_ms": max(_nearest_errors_ms(camera_ts, state_ts)),
            },
        },
        "filters": filters,
        "warnings": warnings,
        "exclusion_reasons": exclusion_reasons,
        "requires_filtering": bool(filters),
        "train_eligible_after_filters": not exclusion_reasons,
    }
    result["episode_manifest_sha256"] = _stable_digest(result)
    return result


def build_dataset_manifest(
    source_root: str | Path,
    *,
    dataset_name: str | None = None,
    robot_dagger_root: str | Path | None = None,
) -> dict[str, Any]:
    source = Path(source_root).expanduser()
    if source.is_symlink():
        raise ValueError("dataset source root cannot be a symlink")
    source = source.resolve()
    if not source.is_dir():
        raise FileNotFoundError(f"dataset source not found: {source}")
    episodes = sorted(path.parent for path in source.rglob("schema.json"))
    if not episodes:
        raise ValueError("dataset contains no episodes with schema.json")
    if len(episodes) > MAX_EPISODES:
        raise ValueError(f"dataset exceeds {MAX_EPISODES} episodes")
    scanned = [scan_episode(path) for path in episodes]
    identity_payload = [
        {
            "episode_id": episode["episode_id"],
            "episode_manifest_sha256": episode["episode_manifest_sha256"],
        }
        for episode in scanned
    ]
    dataset_digest = _stable_digest(identity_payload)
    eligible = [ep for ep in scanned if ep["train_eligible_after_filters"]]
    outcome_counts = Counter(ep["task_outcome"] for ep in scanned)
    dagger_root = (
        Path(robot_dagger_root).expanduser().resolve()
        if robot_dagger_root is not None
        else None
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "dataset_id": f"{dataset_name or source.name}-{dataset_digest[:12]}",
        "dataset_name": dataset_name or source.name,
        "dataset_sha256": dataset_digest,
        "source_root": str(source),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "profile_id": PROFILE_ID,
        "source_revisions": {
            "xpolicylab_commit": _git_commit(Path(__file__).resolve().parents[3]),
            "robot_dagger_commit": _git_commit(dagger_root) if dagger_root else None,
        },
        "profile_config": {
            "source_schema": "gr3_dagger_v2",
            "state_dim": 33,
            "action_dim": 37,
            "camera_alignment_limit_ms": ALIGNMENT_LIMIT_MS,
            "allow_trailing_post_termination_camera_trim": True,
            "require_recording_saved_successfully": True,
            "require_at_least_one_valid_label": True,
        },
        "summary": {
            "episode_count": len(scanned),
            "eligible_episode_count": len(eligible),
            "excluded_episode_count": len(scanned) - len(eligible),
            "requires_filtering_episode_count": sum(
                bool(ep["requires_filtering"]) for ep in scanned
            ),
            "task_outcome_counts": dict(sorted(outcome_counts.items())),
            "statistics": {
                "camera_frames": sum(
                    ep["statistics"]["camera_frames"] for ep in scanned
                ),
                "valid_label_rows": sum(
                    ep["statistics"]["valid_label_rows"] for ep in scanned
                ),
                "trainable_camera_frames": sum(
                    ep["statistics"]["trainable_camera_frames"] for ep in scanned
                ),
                "intervention_count": sum(
                    ep["statistics"]["intervention_count"] for ep in scanned
                ),
            },
        },
        "episodes": scanned,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dataset-name")
    parser.add_argument("--robot-dagger-root", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"refusing to overwrite existing manifest: {args.output}")
    manifest = build_dataset_manifest(
        args.source_root,
        dataset_name=args.dataset_name,
        robot_dagger_root=args.robot_dagger_root,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(manifest["summary"], sort_keys=True))
    print(f"Dataset manifest: {args.output.resolve()}")


if __name__ == "__main__":
    main()
