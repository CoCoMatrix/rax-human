"""Merge curated AnyGrasp clips and add full-prompt deployment trajectories."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq


CLIP_COLUMNS = (
    "clip_id",
    "task_id",
    "batch_path",
    "episode_index",
    "subtask",
    "prompt",
    "clip_frame_start",
    "video_frame_start",
    "num_clip_frames",
    "source_video",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _deployment_windows(
    episodes_path: Path,
    *,
    prompt_pattern: str,
    frames_per_clip: int,
    stride: int,
) -> list[dict[str, object]]:
    pattern = re.compile(prompt_pattern, re.IGNORECASE)
    episodes = pq.read_table(
        episodes_path,
        columns=[
            "task_id",
            "batch_path",
            "episode_index",
            "length",
            "dataset_from_index",
            "prompt",
        ],
    ).to_pylist()
    rows: list[dict[str, object]] = []
    for episode in episodes:
        prompt = str(episode["prompt"]).strip()
        if not pattern.search(prompt):
            continue
        length = int(episode["length"])
        if length < 1:
            continue
        starts = list(range(0, max(1, length - frames_per_clip + 1), stride))
        last = max(0, length - frames_per_clip)
        if not starts or starts[-1] != last:
            starts.append(last)
        task_id = str(episode["task_id"])
        batch_path = str(episode["batch_path"])
        episode_index = int(episode["episode_index"])
        dataset_start = int(episode["dataset_from_index"])
        for start in starts:
            clip_frames = min(frames_per_clip, length - start)
            rows.append(
                {
                    "clip_id": (
                        f"deploy_{task_id}_{batch_path.rsplit('/', 1)[-1]}_"
                        f"episode_{episode_index:06d}_f{start:06d}_n{clip_frames}"
                    ),
                    "task_id": task_id,
                    "batch_path": batch_path,
                    "episode_index": episode_index,
                    "subtask": prompt,
                    "prompt": prompt,
                    "clip_frame_start": start,
                    "video_frame_start": dataset_start + start,
                    "num_clip_frames": clip_frames,
                    "source_video": (
                        f"{batch_path}/videos/observation.images.top/"
                        "chunk-000/file-000.mp4"
                    ),
                }
            )
    return rows


def build_selection(
    clip_paths: list[Path],
    episodes_path: Path,
    output_root: Path,
    *,
    prompt_pattern: str,
    frames_per_clip: int,
    target_stride: int,
) -> tuple[Path, Path]:
    if frames_per_clip < 1 or target_stride < 1:
        raise ValueError("clip length and stride must be positive")
    output_root = output_root.expanduser().resolve()
    manifest_dir = output_root / "manifest"
    clips_output = manifest_dir / "clips.parquet"
    report_output = manifest_dir / "selection_report.json"
    if clips_output.exists() or report_output.exists():
        raise FileExistsError(f"refusing to overwrite deployment selection: {output_root}")

    merged: dict[str, dict[str, object]] = {}
    source_counts: dict[str, dict[str, int]] = {}
    canonical_task_ids: set[str] | None = None
    for source_index, source in enumerate(clip_paths):
        source = source.expanduser().resolve()
        rows = pq.read_table(source, columns=list(CLIP_COLUMNS)).to_pylist()
        source_task_ids = {str(row["task_id"]) for row in rows}
        if source_index == 0:
            canonical_task_ids = source_task_ids
        else:
            assert canonical_task_ids is not None
            rows = [
                row for row in rows if str(row["task_id"]) in canonical_task_ids
            ]
        source_counts[str(source)] = {
            "selected_clips": len(rows),
            "selected_frames": sum(int(row["num_clip_frames"]) for row in rows),
            "source_tasks": len(source_task_ids),
            "selected_tasks": len({str(row["task_id"]) for row in rows}),
        }
        for row in rows:
            merged.setdefault(str(row["clip_id"]), row)

    assert canonical_task_ids is not None
    candidate_deployment_rows = _deployment_windows(
        episodes_path.expanduser().resolve(),
        prompt_pattern=prompt_pattern,
        frames_per_clip=frames_per_clip,
        stride=target_stride,
    )
    # Keep the established task universe stable so adding target windows cannot
    # silently reshuffle the sealed task-disjoint held-out partition.
    deployment_rows = [
        row
        for row in candidate_deployment_rows
        if str(row["task_id"]) in canonical_task_ids
    ]
    for row in deployment_rows:
        merged.setdefault(str(row["clip_id"]), row)
    rows = sorted(
        merged.values(),
        key=lambda row: (
            str(row["task_id"]),
            str(row["batch_path"]),
            int(row["episode_index"]),
            int(row["video_frame_start"]),
            str(row["clip_id"]),
        ),
    )
    manifest_dir.mkdir(parents=True, exist_ok=False)
    pq.write_table(pa.Table.from_pylist(rows), clips_output, compression="zstd")
    report = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "selection_strategy": "curated_union_plus_full_prompt_deployment_windows",
        "canonical_task_source": str(clip_paths[0].expanduser().resolve()),
        "canonical_task_count": len(canonical_task_ids),
        "source_clips": source_counts,
        "source_clip_sha256": {
            str(path.expanduser().resolve()): _sha256(path.expanduser().resolve())
            for path in clip_paths
        },
        "episodes_path": str(episodes_path.expanduser().resolve()),
        "episodes_sha256": _sha256(episodes_path.expanduser().resolve()),
        "prompt_pattern": prompt_pattern,
        "target_frames_per_clip": frames_per_clip,
        "target_stride": target_stride,
        "deployment_window_clips": len(deployment_rows),
        "deployment_window_clips_excluded_new_tasks": (
            len(candidate_deployment_rows) - len(deployment_rows)
        ),
        "deployment_window_frames": sum(
            int(row["num_clip_frames"]) for row in deployment_rows
        ),
        "selected_clips": len(rows),
        "selected_frames": sum(int(row["num_clip_frames"]) for row in rows),
        "selected_tasks": len({str(row["task_id"]) for row in rows}),
    }
    report_output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return clips_output, report_output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--clips", type=Path, action="append", required=True)
    parser.add_argument("--episodes", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--prompt-pattern",
        default=r"pick the (red )?tomato .*place it (in|into|inside) the left basket",
    )
    parser.add_argument("--frames-per-clip", type=int, default=81)
    parser.add_argument("--target-stride", type=int, default=32)
    args = parser.parse_args()
    clips, report = build_selection(
        args.clips,
        args.episodes,
        args.output_root,
        prompt_pattern=args.prompt_pattern,
        frames_per_clip=args.frames_per_clip,
        target_stride=args.target_stride,
    )
    print(clips)
    print(report)


if __name__ == "__main__":
    main()
