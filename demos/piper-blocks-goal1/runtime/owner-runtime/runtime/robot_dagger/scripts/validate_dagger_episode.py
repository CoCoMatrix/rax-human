#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import json
import math
from itertools import pairwise
from pathlib import Path

import pyarrow.parquet as pq

REQUIRED_STREAMS = (
    "observation.state_robot",
    "action.model_robot",
    "action.expert_proposed_robot",
    "action.expert_safe_robot",
    "action.executed_robot",
    "control.mode",
    "control.selected_action_source",
    "control.event",
    "model.status",
)
TYPED_PARAMETER_KEYS = {
    "Bool",
    "Float",
    "Integer",
    "String",
    "Timestamp",
}


def unwrap_parameter(value):
    """Decode Dora's typed JSON parameter representation."""
    if isinstance(value, list):
        return [unwrap_parameter(item) for item in value]
    if isinstance(value, dict):
        if len(value) == 1:
            key, item = next(iter(value.items()))
            if key in TYPED_PARAMETER_KEYS:
                return unwrap_parameter(item)
        return {key: unwrap_parameter(item) for key, item in value.items()}
    return value


def decode_parameters(value: str) -> dict:
    parameters = json.loads(value)
    if not isinstance(parameters, dict):
        raise TypeError("Parquet parameters must decode to an object")
    return {str(key): unwrap_parameter(item) for key, item in parameters.items()}


def load_rows(episode: Path, stream: str) -> list[dict]:
    path = episode / f"{stream}.parquet"
    if not path.is_file():
        raise ValueError(f"missing stream: {path.name}")
    table = pq.read_table(path)
    data = table[stream].combine_chunks()
    return [
        {
            "data": data[index].as_py(),
            "parameters": decode_parameters(table["parameters"][index].as_py()),
            "source_timestamp_ns": table["source_timestamp_ns"][index].as_py(),
            "target_execution_timestamp_ns": table["target_execution_timestamp_ns"][
                index
            ].as_py(),
            "logged_timestamp_ns": table["logged_timestamp_ns"][index].as_py(),
        }
        for index in range(table.num_rows)
    ]


def action_dict(row: dict, expected_order: list[str]) -> dict[str, float]:
    items = row["data"]
    names = [item["name"] for item in items]
    if names != expected_order:
        raise ValueError(f"action order mismatch: {names}")
    values = {item["name"]: float(item["value"]) for item in items}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("action contains NaN or Inf")
    return values


def require_strictly_increasing(rows: list[dict], stream: str) -> None:
    timestamps = [row["target_execution_timestamp_ns"] for row in rows]
    if any(current <= previous for previous, current in pairwise(timestamps)):
        raise ValueError(
            f"{stream} target execution timestamps are not strictly increasing"
        )


def nearest_error_ms(timestamp_ns: int, candidates: list[int]) -> float:
    index = bisect.bisect_left(candidates, timestamp_ns)
    neighbors = candidates[max(0, index - 1) : index + 1]
    return min(abs(timestamp_ns - value) for value in neighbors) / 1e6


def longest_expert_segment_seconds(
    rows: list[dict], max_gap_ms: float = 100.0
) -> float:
    timestamps = [
        row["source_timestamp_ns"]
        for row in rows
        if row["parameters"].get("expert_action_valid") is True
    ]
    if not timestamps:
        return 0.0
    longest = 0
    start = previous = timestamps[0]
    for timestamp in timestamps[1:]:
        if timestamp - previous > max_gap_ms * 1e6:
            longest = max(longest, previous - start)
            start = timestamp
        previous = timestamp
    return max(longest, previous - start) / 1e9


def validate(episode: Path) -> None:
    schema = json.loads((episode / "schema.json").read_text())
    if schema.get("storage_format", "xpolicylab_dagger_parquet_v1") != (
        "xpolicylab_dagger_parquet_v1"
    ):
        raise ValueError("unsupported DAgger storage format")
    state_order = schema["state_order"]
    action_order = schema["action_order"]
    if schema["state_dim"] != len(state_order) or schema["action_dim"] != len(
        action_order
    ):
        raise ValueError("schema dimensions do not match ordered fields")
    cameras = schema.get("cameras")
    if cameras is None and "camera_top" in schema:
        cameras = {"camera_top": schema["camera_top"]}
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError("schema must declare at least one camera")

    rows = {stream: load_rows(episode, stream) for stream in REQUIRED_STREAMS}
    image_streams = {
        camera: load_rows(episode, f"observation.images.{camera}")
        for camera in cameras
    }
    for stream in REQUIRED_STREAMS:
        if not rows[stream]:
            raise ValueError(f"empty stream: {stream}")
    for camera, image_rows in image_streams.items():
        if not image_rows:
            raise ValueError(f"empty camera stream: observation.images.{camera}")

    for row in rows["observation.state_robot"]:
        action_dict(row, state_order)
    for stream in (
        "action.model_robot",
        "action.expert_proposed_robot",
        "action.expert_safe_robot",
        "action.executed_robot",
    ):
        for row in rows[stream]:
            action_dict(row, action_order)
        require_strictly_increasing(rows[stream], stream)

    model_fields = {
        "model_chunk_id",
        "model_chunk_step_index",
        "model_generation",
        "model_action_horizon",
        "queue_depth",
        "inference_start_timestamp_ns",
        "inference_end_timestamp_ns",
        "predicted_execution_timestamp_ns",
    }
    for row in rows["action.model_robot"]:
        missing = model_fields - row["parameters"].keys()
        if missing:
            raise ValueError(f"model action metadata is missing: {sorted(missing)}")

    for stream in ("action.expert_proposed_robot", "action.expert_safe_robot"):
        for row in rows[stream]:
            if row["parameters"].get("expert_action_valid") is not True:
                raise ValueError(f"{stream} contains an invalid expert action row")

    safe_by_source = {
        row["source_timestamp_ns"]: action_dict(row, action_order)
        for row in rows["action.expert_safe_robot"]
    }
    max_expert_execution_error = 0.0
    expert_executed = 0
    adjusted_expert_executed = 0
    for row in rows["action.executed_robot"]:
        if row["parameters"].get("selected_action_source") != "EXPERT":
            continue
        expert_executed += 1
        safe = safe_by_source.get(row["source_timestamp_ns"])
        if safe is None:
            raise ValueError(
                "EXPERT executed action has no same-timestamp expert_safe action"
            )
        executed = action_dict(row, action_order)
        error = max(abs(executed[name] - safe[name]) for name in action_order)
        max_expert_execution_error = max(max_expert_execution_error, error)
        if error >= 1e-6:
            reason = row["parameters"].get("controller_adjustment_reason") or row[
                "parameters"
            ].get("action_adjustment_reason")
            if not reason:
                raise ValueError(
                    f"expert_safe/executed error {error:.3g} has no adjustment reason"
                )
            adjusted_expert_executed += 1

    state_timestamps = sorted(
        row["source_timestamp_ns"] for row in rows["observation.state_robot"]
    )
    expert_timestamps = sorted(
        row["source_timestamp_ns"] for row in rows["action.expert_safe_robot"]
    )
    dropped_camera_frames = 0
    trailing_camera_frames = 0
    state_alignment_errors = []
    expert_alignment_errors = []
    for camera, image_rows in image_streams.items():
        expected_encoding = str(cameras[camera].get("encoding", ""))
        for index, row in enumerate(image_rows):
            parameters = row["parameters"]
            if parameters.get("camera_name") != camera:
                raise ValueError(f"camera_name must be {camera}")
            if parameters.get("encoding") != expected_encoding:
                raise ValueError(
                    f"{camera} encoding must be {expected_encoding}"
                )
            if parameters.get("frame_index") != index:
                raise ValueError(f"{camera} frame_index is not contiguous")
            expected_bytes = (
                int(parameters["width"]) * int(parameters["height"]) * 3
            )
            if len(row["data"]) != expected_bytes:
                raise ValueError(f"{camera} row is not raw width*height*3 bytes")
        device_indices = [
            int(row["parameters"]["device_frame_index"]) for row in image_rows
        ]
        dropped_camera_frames += sum(
            max(0, current - previous - 1)
            for previous, current in pairwise(device_indices)
        )
        # Camera streams stay alive through the save-confirm window. Frames
        # after canonical state ends are audit evidence, not training frames.
        training_rows = [
            row
            for row in image_rows
            if row["source_timestamp_ns"] <= state_timestamps[-1]
        ]
        trailing_camera_frames += len(image_rows) - len(training_rows)
        if not training_rows:
            raise ValueError(f"no {camera} frames overlap canonical robot state")
        state_alignment_errors.extend(
            nearest_error_ms(row["source_timestamp_ns"], state_timestamps)
            for row in training_rows
        )
        expert_alignment_errors.extend(
            nearest_error_ms(row["source_timestamp_ns"], expert_timestamps)
            for row in image_rows
        )
    max_state_alignment_ms = max(state_alignment_errors)
    max_expert_alignment_ms = max(expert_alignment_errors)
    if max_state_alignment_ms > 20.0 or max_expert_alignment_ms > 20.0:
        raise ValueError(
            "camera alignment exceeds 20 ms: "
            f"state={max_state_alignment_ms:.2f}, expert={max_expert_alignment_ms:.2f}"
        )

    segment_seconds = longest_expert_segment_seconds(rows["action.expert_safe_robot"])
    model_starved_events = sum(
        '"status": "starved"' in str(row["data"][0]) for row in rows["model.status"]
    )
    print(f"episode: {episode}")
    print(f"expert executed rows: {expert_executed}")
    print(f"expert adjusted executed rows: {adjusted_expert_executed}")
    print(f"expert_safe/executed max error: {max_expert_execution_error:.3g}")
    print(
        "camera max alignment ms: "
        f"state={max_state_alignment_ms:.2f}, expert={max_expert_alignment_ms:.2f}"
    )
    print(f"camera dropped device frames: {dropped_camera_frames}")
    print(f"camera trailing audit frames to trim: {trailing_camera_frames}")
    print(f"model_starved events: {model_starved_events}")
    print(
        "longest valid expert segment: "
        f"{segment_seconds:.2f}s (~{math.floor(segment_seconds * 30) + 1} frames at 30 Hz)"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    validate(args.episode.resolve())


if __name__ == "__main__":
    main()
