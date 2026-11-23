#!/usr/bin/env python3
"""Upload a GR3 ThinkPlus capture with a stable, device-independent OSS layout.

The source is treated as read-only.  Task directory names are normalized from
the recorder labels (for example ``1-A(black)`` becomes
``task_01_A_black``); operator/device implementation directories are not part
of the delivery key.  The uploader is resumable and only writes state below
the selected state root.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import json
from pathlib import Path
import os
import re
import time

import oss2


def normalize_task(name: str) -> str:
    match = re.fullmatch(r"(\d+)-([A-C])(?:\(([^)]+)\))?", name)
    if match:
        number, letter, label = match.groups()
        return f"task_{int(number):02d}_{letter}" + (f"_{label.lower()}" if label else "")
    return "task_" + re.sub(r"[^A-Za-z0-9_-]+", "_", name).strip("_").lower()


def retry(operation, label: str, attempts: int = 8):
    for attempt in range(1, attempts + 1):
        try:
            return operation()
        except Exception as exc:  # noqa: BLE001 - network failures are retriable
            if attempt == attempts:
                raise
            delay = min(30, 2 ** min(attempt - 1, 5))
            print(f"retry {attempt}/{attempts - 1} {label}: {type(exc).__name__}; sleep {delay}s", flush=True)
            time.sleep(delay)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path("/media/ubuntu/thinkplus/2026-08-30/-1/qnexo"))
    # The delivery target is the PPU Wulanchabu bucket.  Do not inherit the
    # workstation's ALIBABA_OSS_* defaults: this capture was accidentally
    # published to the separate Beijing bucket during an earlier run.
    parser.add_argument("--bucket", default="ali-pai-oss-ppu")
    parser.add_argument("--endpoint", default="https://oss-cn-wulanchabu.aliyuncs.com")
    parser.add_argument("--prefix", default="Data/real_gr3/raw/pick_doll_demo/gr3_pick_doll_20260830/")
    parser.add_argument("--dataset-id", default="gr3_pick_doll_20260830_qnexo_raw")
    parser.add_argument("--state-root", type=Path, default=Path("/tmp/gr3_pick_doll_20260830_normalized_oss_state"))
    parser.add_argument("--file-workers", type=int, default=2)
    parser.add_argument("--part-workers", type=int, default=4)
    parser.add_argument("--timeout", type=int, default=90)
    args = parser.parse_args()

    source = args.source.resolve(strict=True)
    prefix = args.prefix.strip("/") + "/"
    args.state_root.mkdir(parents=True, exist_ok=True)
    key_id = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_ID")
    key_secret = os.environ.get("ALIBABA_CLOUD_ACCESS_KEY_SECRET")
    if not key_id or not key_secret:
        raise SystemExit("Alibaba OSS credentials are not present")
    if args.file_workers < 1 or args.part_workers < 1:
        raise SystemExit("worker counts must be positive")

    tasks = sorted(path.name for path in source.iterdir() if path.is_dir())
    mapping = {name: normalize_task(name) for name in tasks}
    if len(set(mapping.values())) != len(mapping):
        raise SystemExit(f"task key collision: {mapping}")
    files: list[tuple[str, Path]] = []
    counts: dict[str, int] = {}
    for task in tasks:
        episodes = sorted(path for path in (source / task).iterdir() if path.is_dir() and path.name.startswith("episode_"))
        counts[mapping[task]] = len(episodes)
        for episode in episodes:
            for path in sorted(path for path in episode.iterdir() if path.is_file()):
                files.append((f"{mapping[task]}/{episode.name}/{path.name}", path))
    if not files:
        raise SystemExit("source contains no files")

    manifest = {
        "schema_version": "cocomatrix.gr3_raw_dataset_manifest.v1",
        "dataset_id": args.dataset_id,
        "robot": "GR3",
        "capture_date": "2026-08-30",
        "equipment": "qnexo",
        "machine_id": "GR301AB0153",
        "camera_type": "oak_w97",
        "image_contract": {"rgb_encoding": "jpeg", "depth_encoding": "png", "width": 960, "height": 600, "capture_hz": 30},
        "source_task_to_delivery_task": mapping,
        "delivery_task_directories": sorted(mapping.values()),
        "episode_counts": counts,
        "expected_episode_count": 360,
        "actual_completed_episode_count": sum(counts.values()),
        "missing_completed_episodes": max(0, 360 - sum(counts.values())),
        "layout": "<task>/episode_<source_id>/{DONE,metadata.json,*.parquet}",
        "file_count": len(files),
        "total_bytes": sum(path.stat().st_size for _, path in files),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "notes": "Byte-preserving key remap from ThinkPlus; operator/device directories omitted; episode IDs remain source IDs; 4-C has 29 completed episodes.",
    }
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()
    manifest_key = prefix + "DATASET_MANIFEST.json"
    session = oss2.Session(pool_size=max(32, args.file_workers * args.part_workers + 8))
    bucket = oss2.Bucket(
        oss2.Auth(key_id, key_secret), args.endpoint, args.bucket, session=session,
        connect_timeout=args.timeout, enable_crc=True,
    )
    bucket.timeout = args.timeout

    def inventory() -> dict[str, int]:
        return retry(
            lambda: {obj.key: obj.size for obj in oss2.ObjectIteratorV2(bucket, prefix=prefix) if not obj.key.endswith("/")},
            f"list {prefix}",
        )

    existing = inventory()
    expected_sizes = {prefix + relative: path.stat().st_size for relative, path in files}
    expected_sizes[manifest_key] = len(manifest_bytes)
    print(json.dumps({"destination": f"oss://{args.bucket}/{prefix}", "local_files": len(files), "local_bytes": manifest["total_bytes"], "task_counts": counts, "existing_objects": len(existing)}, sort_keys=True), flush=True)
    for key, size in existing.items():
        if key not in expected_sizes:
            raise SystemExit(f"unexpected remote object: {key}")
        if expected_sizes[key] != size:
            raise SystemExit(f"wrong-size remote object: {key}: {size} != {expected_sizes[key]}")
    pending = [(relative, path) for relative, path in files if prefix + relative not in existing]
    small = [item for item in pending if item[1].stat().st_size < 32 * 1024 * 1024]
    large = [item for item in pending if item[1].stat().st_size >= 32 * 1024 * 1024]
    print(json.dumps({"pending_files": len(pending), "pending_small": len(small), "pending_large": len(large), "state_root": str(args.state_root)}, sort_keys=True), flush=True)
    headers = {"x-oss-meta-dataset-id": args.dataset_id}

    def upload_small(item: tuple[str, Path]) -> tuple[str, int]:
        relative, path = item
        retry(lambda: bucket.put_object_from_file(prefix + relative, str(path), headers=headers), relative)
        return relative, path.stat().st_size

    def upload_large(item: tuple[str, Path]) -> tuple[str, int]:
        relative, path = item
        retry(lambda: oss2.resumable_upload(
            bucket, prefix + relative, str(path), store=oss2.ResumableStore(root=str(args.state_root)),
            headers=headers, multipart_threshold=32 * 1024 * 1024,
            part_size=16 * 1024 * 1024, num_threads=args.part_workers,
        ), relative)
        return relative, path.stat().st_size

    started = time.monotonic(); done_bytes = 0
    if small:
        with ThreadPoolExecutor(max_workers=min(args.file_workers * 4, 8)) as pool:
            futures = [pool.submit(upload_small, item) for item in small]
            for index, future in enumerate(as_completed(futures), 1):
                _, size = future.result(); done_bytes += size
                if index % 250 == 0 or index == len(small):
                    print(f"small files {index}/{len(small)}; bytes {done_bytes}", flush=True)
    if large:
        with ThreadPoolExecutor(max_workers=args.file_workers) as pool:
            futures = [pool.submit(upload_large, item) for item in large]
            for index, future in enumerate(as_completed(futures), 1):
                relative, size = future.result(); done_bytes += size
                elapsed = max(time.monotonic() - started, 1e-9)
                print(f"large files {index}/{len(large)}; session {done_bytes / 1024**3:.2f}GiB; rate {done_bytes / elapsed / 1024**2:.1f}MiB/s; {relative}", flush=True)

    retry(lambda: bucket.put_object(manifest_key, manifest_bytes, headers=headers), "DATASET_MANIFEST.json")
    remote = inventory()
    missing = sorted(set(expected_sizes) - set(remote))
    extra = sorted(set(remote) - set(expected_sizes))
    wrong = sorted(key for key in expected_sizes if key in remote and remote[key] != expected_sizes[key])
    if missing or extra or wrong:
        raise SystemExit(json.dumps({"remote_inventory_mismatch": True, "missing": missing[:20], "extra": extra[:20], "wrong_size": wrong[:20]}))
    print(json.dumps({"ok": True, "uri": f"oss://{args.bucket}/{prefix}", "objects": len(remote), "bytes": sum(remote.values()), "manifest": manifest_key, "task_mapping": mapping, "task_counts": counts}, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
