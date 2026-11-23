"""Build a small reference-only AnyGrasp episode for XPolicyLab replay."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from real_world.robots.gr3.mapping import ACTION_NAMES, STATE_NAMES


def _key(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-manifest", type=Path, required=True)
    parser.add_argument("--turbovla-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--task-count", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"refusing to overwrite output directory: {args.output_dir}")
    if args.task_count < 1:
        parser.error("task-count must be positive")

    turbovla_root = args.turbovla_root.expanduser().resolve()
    manifest_path = args.dataset_manifest.expanduser().resolve()
    import sys

    sys.path.insert(0, str(turbovla_root))
    from turbovla.data.gr3_anygrasp import Gr3AnygraspDataset
    from turbovla.data.gr3_common import Gr3NormalizationStats

    dummy_stats = Gr3NormalizationStats(
        state_mean=np.zeros(33, dtype=np.float32),
        state_std=np.ones(33, dtype=np.float32),
        action_low=np.full(37, -1.0, dtype=np.float32),
        action_high=np.full(37, 1.0, dtype=np.float32),
    )
    dataset = Gr3AnygraspDataset(
        manifest_path,
        horizon=1,
        image_size=224,
        stats=dummy_stats,
    )
    grouped: dict[str, list[int]] = defaultdict(list)
    for index, sample in enumerate(dataset.samples):
        grouped[sample.task_id].append(index)
    task_ids = sorted(grouped, key=lambda task_id: _key(f"{args.seed}:task:{task_id}"))
    selected_task_ids = task_ids[: min(args.task_count, len(task_ids))]
    sample_indices = [
        min(grouped[task_id], key=lambda index: _key(f"{args.seed}:{task_id}:{index}"))
        for task_id in selected_task_ids
    ]
    manifest = dataset.manifest
    schema = {
        "schema_version": "gr3_anygrasp_reference_v1",
        "state_order": list(STATE_NAMES),
        "action_order": list(ACTION_NAMES),
        "camera_top": {"encoding": "rgb8"},
        "reference": {
            "dataset_manifest": str(manifest_path),
            "dataset_id": manifest["dataset_id"],
            "dataset_sha256": manifest["dataset_sha256"],
            "turbovla_root": str(turbovla_root),
            "sample_indices": sample_indices,
            "task_ids": selected_task_ids,
            "selection_strategy": "task_round_robin_sha256_one_per_task",
            "seed": args.seed,
            "raw_media_access": "read_only_in_place",
        },
    }
    metadata = {
        "episode_id": f"anygrasp-reference-{manifest['dataset_id']}",
        "robot_id": "gr3-anygrasp-recorded",
        "task_instruction": "Per-sample AnyGrasp instruction",
        "task_count": len(selected_task_ids),
        "sample_count": len(sample_indices),
        "actuation_enabled": False,
    }
    args.output_dir.mkdir(parents=True)
    (args.output_dir / "schema.json").write_text(
        json.dumps(schema, indent=2) + "\n", encoding="utf-8"
    )
    (args.output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print(args.output_dir.resolve())


if __name__ == "__main__":
    main()
