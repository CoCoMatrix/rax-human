#!/usr/bin/env python3
"""Extract a tiny, read-only visual sample from the Piper Songling LeRobot v3 dataset."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
import shutil
from typing import Any, Iterable

import av
from PIL import Image, ImageDraw
import pyarrow.parquet as pq


EXPECTED_CAMERA_SUFFIXES = ("head", "left_wrist", "right_wrist")


def _jsonable(value: Any) -> Any:
    if hasattr(value, "as_py"):
        value = value.as_py()
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _read_parquet_records(paths: Iterable[Path]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for path in sorted(paths):
        records.extend(pq.read_table(path).to_pylist())
    return records


def _find_split(value: Any, split_name: str) -> list[int] | None:
    def episode_list(candidate: Any) -> list[int] | None:
        if isinstance(candidate, list) and all(
            isinstance(item, (int, float, str)) for item in candidate
        ):
            try:
                return [int(item) for item in candidate]
            except (TypeError, ValueError):
                return None
        if isinstance(candidate, dict):
            for key in ("episodes", "episode_indices", "indices"):
                if key in candidate:
                    result = episode_list(candidate[key])
                    if result is not None:
                        return result
        return None

    if isinstance(value, dict):
        if split_name in value:
            candidate = value[split_name]
            result = episode_list(candidate)
            if result is not None:
                return result
            nested = _find_split(candidate, split_name)
            if nested is not None:
                return nested
        for nested_value in value.values():
            nested = _find_split(nested_value, split_name)
            if nested is not None:
                return nested
    return None


def _evenly_spaced(values: list[int], count: int) -> list[int]:
    if count < 1:
        raise ValueError("sample count must be positive")
    if not values:
        raise ValueError("cannot sample an empty split")
    if count >= len(values):
        return list(values)
    if count == 1:
        return [values[len(values) // 2]]
    positions = [round(index * (len(values) - 1) / (count - 1)) for index in range(count)]
    return [values[position] for position in positions]


def _camera_keys(info: dict[str, Any]) -> dict[str, str]:
    video_keys = [
        key
        for key, feature in info["features"].items()
        if feature.get("dtype") == "video"
    ]
    result: dict[str, str] = {}
    for suffix in EXPECTED_CAMERA_SUFFIXES:
        matches = [key for key in video_keys if key.split(".")[-1] == suffix]
        if len(matches) != 1:
            raise ValueError(
                f"expected one video key ending in {suffix!r}, found {matches} in {video_keys}"
            )
        result[suffix] = matches[0]
    return result


def _decode_frame(video_path: Path, timestamp_seconds: float) -> Image.Image:
    with av.open(str(video_path)) as container:
        stream = container.streams.video[0]
        time_base = float(stream.time_base)
        target_pts = max(0, int(timestamp_seconds / time_base))
        container.seek(target_pts, stream=stream, backward=True, any_frame=False)
        latest: Image.Image | None = None
        for frame in container.decode(stream):
            latest = frame.to_image().convert("RGB")
            if frame.pts is None or float(frame.pts * stream.time_base) >= timestamp_seconds:
                return latest
        if latest is None:
            raise RuntimeError(f"no frame decoded from {video_path} at {timestamp_seconds:.6f}s")
        return latest


def _labeled_contact_sheet(
    rows: list[tuple[str, dict[str, Image.Image]]], output_path: Path
) -> None:
    thumb_size = (480, 270)
    label_height = 32
    canvas = Image.new(
        "RGB",
        (thumb_size[0] * 3, (thumb_size[1] + label_height) * len(rows)),
        "white",
    )
    draw = ImageDraw.Draw(canvas)
    for row_index, (stage, images) in enumerate(rows):
        top = row_index * (thumb_size[1] + label_height)
        for column, camera in enumerate(EXPECTED_CAMERA_SUFFIXES):
            image = images[camera].resize(thumb_size, Image.Resampling.LANCZOS)
            left = column * thumb_size[0]
            canvas.paste(image, (left, top + label_height))
            draw.text((left + 10, top + 9), f"{stage} · {camera}", fill="black")
    canvas.save(output_path, quality=92)


def _training_mosaic(images: dict[str, Image.Image]) -> Image.Image:
    head = images["head"].resize((320, 256), Image.Resampling.BILINEAR)
    left = images["left_wrist"].resize((160, 128), Image.Resampling.BILINEAR)
    right = images["right_wrist"].resize((160, 128), Image.Resampling.BILINEAR)
    mosaic = Image.new("RGB", (320, 384), "black")
    mosaic.paste(head, (0, 0))
    mosaic.paste(left, (0, 256))
    mosaic.paste(right, (160, 256))
    return mosaic


def _mosaic_sheet(rows: list[tuple[str, dict[str, Image.Image]]], output_path: Path) -> None:
    label_height = 32
    canvas = Image.new("RGB", (320 * len(rows), 384 + label_height), "white")
    draw = ImageDraw.Draw(canvas)
    for index, (stage, images) in enumerate(rows):
        canvas.paste(_training_mosaic(images), (index * 320, label_height))
        draw.text((index * 320 + 10, 9), f"{stage} · model mosaic", fill="black")
    canvas.save(output_path, quality=92)


def _write_gallery(output_root: Path, manifest: dict[str, Any]) -> None:
    cards = []
    for episode in manifest["samples"]:
        index = episode["episode_index"]
        cards.append(
            f"""
            <article>
              <h2>Episode {index}</h2>
              <p>{html.escape(episode['task'])} · {episode['length']} frames</p>
              <img src="episode_{index:06d}/contact_sheet.jpg" alt="Raw three-camera samples">
              <h3>Actual FastWAM model mosaic</h3>
              <img class="mosaic" src="episode_{index:06d}/training_mosaics.jpg" alt="Processed model input mosaics">
            </article>
            """
        )
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Piper Songling FastWAM dataset samples</title>
<style>
body{{margin:0;background:#f5f5f3;color:#161616;font:15px/1.5 system-ui,sans-serif}}main{{max-width:1280px;margin:auto;padding:40px 24px}}
h1{{font-size:34px;margin:0 0 8px}}.summary{{color:#666;margin-bottom:32px}}article{{background:white;border:1px solid #ddd;border-radius:16px;padding:22px;margin:20px 0;box-shadow:0 8px 24px #0000000a}}
img{{display:block;width:100%;height:auto;border-radius:10px;border:1px solid #ddd}}img.mosaic{{max-width:960px}}h3{{margin-top:24px}}
</style></head><body><main><h1>Piper Songling · FastWAM training samples</h1>
<p class="summary">LeRobot {html.escape(str(manifest['codebase_version']))} · {manifest['total_episodes']} episodes · {manifest['total_frames']} frames · {manifest['fps']} FPS · split: {html.escape(manifest['split_name'])}</p>
{''.join(cards)}</main></body></html>"""
    (output_root / "index.html").write_text(document, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--split-manifest", type=Path, required=True)
    parser.add_argument("--split", default="train")
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    dataset_root = args.dataset_root.resolve()
    info = json.loads((dataset_root / "meta" / "info.json").read_text(encoding="utf-8"))
    split_payload = json.loads(args.split_manifest.read_text(encoding="utf-8"))
    split_episodes = _find_split(split_payload, args.split)
    if split_episodes is None:
        raise ValueError(f"split {args.split!r} not found in {args.split_manifest}")

    episode_records = _read_parquet_records(
        (dataset_root / "meta" / "episodes").glob("chunk-*/file-*.parquet")
    )
    episodes = {int(record["episode_index"]): record for record in episode_records}
    unknown = sorted(set(split_episodes) - set(episodes))
    if unknown:
        raise ValueError(f"split references unknown episodes: {unknown[:10]}")

    task_records = pq.read_table(dataset_root / "meta" / "tasks.parquet").to_pylist()
    tasks = {int(record["task_index"]): str(record["task"]) for record in task_records}
    camera_keys = _camera_keys(info)
    output_root = args.output_dir.resolve()
    output_root.mkdir(parents=True, exist_ok=False)
    shutil.copy2(dataset_root / "meta" / "info.json", output_root / "dataset_info.json")
    shutil.copy2(args.split_manifest, output_root / "episode_splits.json")

    shard_starts: dict[tuple[int, int], int] = {}
    for record in episode_records:
        key = (int(record["data/chunk_index"]), int(record["data/file_index"]))
        start = int(record["dataset_from_index"])
        shard_starts[key] = min(start, shard_starts.get(key, start))

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "dataset_root": str(dataset_root),
        "codebase_version": info.get("codebase_version"),
        "fps": int(info["fps"]),
        "total_episodes": int(info["total_episodes"]),
        "total_frames": int(info["total_frames"]),
        "split_name": args.split,
        "split_episode_count": len(split_episodes),
        "camera_keys": camera_keys,
        "state_order": [
            "left_j1", "left_j2", "left_j3", "left_j4", "left_j5", "left_j6", "left_gripper_width",
            "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6", "right_gripper_width",
        ],
        "samples": [],
    }

    for episode_index in _evenly_spaced(sorted(split_episodes), args.count):
        record = episodes[episode_index]
        episode_start = int(record["dataset_from_index"])
        episode_end = int(record["dataset_to_index"])
        length = episode_end - episode_start
        offsets = [min(length - 1, max(0, round(fraction * (length - 1)))) for fraction in (0.1, 0.5, 0.9)]
        stages = ("early", "middle", "late")
        data_key = (int(record["data/chunk_index"]), int(record["data/file_index"]))
        data_path = dataset_root / info["data_path"].format(
            chunk_index=data_key[0], file_index=data_key[1]
        )
        table = pq.read_table(data_path, memory_map=True)
        episode_dir = output_root / f"episode_{episode_index:06d}"
        episode_dir.mkdir()
        visual_rows: list[tuple[str, dict[str, Image.Image]]] = []
        frame_reports: list[dict[str, Any]] = []

        for stage, offset in zip(stages, offsets, strict=True):
            local_row = episode_start - shard_starts[data_key] + offset
            data_row = table.slice(local_row, 1).to_pylist()[0]
            images: dict[str, Image.Image] = {}
            for camera_suffix, video_key in camera_keys.items():
                prefix = f"videos/{video_key}"
                video_path = dataset_root / info["video_path"].format(
                    video_key=video_key,
                    chunk_index=int(record[f"{prefix}/chunk_index"]),
                    file_index=int(record[f"{prefix}/file_index"]),
                )
                timestamp = float(record[f"{prefix}/from_timestamp"]) + offset / int(info["fps"])
                image = _decode_frame(video_path, timestamp)
                images[camera_suffix] = image
                image.save(episode_dir / f"{stage}_{camera_suffix}.jpg", quality=92)
            visual_rows.append((stage, images))
            if "task_index" in data_row:
                task_index = int(data_row["task_index"])
            else:
                episode_tasks = record.get("tasks") or []
                task_text = str(episode_tasks[0]) if episode_tasks else ""
                task_index = next(
                    (index for index, text in tasks.items() if text == task_text), -1
                )
            frame_reports.append(
                {
                    "stage": stage,
                    "episode_offset": offset,
                    "dataset_index": episode_start + offset,
                    "timestamp": _jsonable(data_row.get("timestamp")),
                    "task_index": task_index,
                    "state": _jsonable(data_row.get("observation.state")),
                    "action": _jsonable(data_row.get("action")),
                }
            )

        _labeled_contact_sheet(visual_rows, episode_dir / "contact_sheet.jpg")
        _mosaic_sheet(visual_rows, episode_dir / "training_mosaics.jpg")
        task_index = frame_reports[0]["task_index"]
        manifest["samples"].append(
            {
                "episode_index": episode_index,
                "length": length,
                "task_index": task_index,
                "task": tasks.get(task_index, f"unknown task {task_index}"),
                "frames": frame_reports,
            }
        )

    (output_root / "sample_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    _write_gallery(output_root, manifest)
    print(json.dumps({"ok": True, "output": str(output_root), "episodes": [item["episode_index"] for item in manifest["samples"]]}))


if __name__ == "__main__":
    main()
