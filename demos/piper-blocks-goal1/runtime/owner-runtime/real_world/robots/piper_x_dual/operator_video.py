from __future__ import annotations

import json
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Mapping


CAMERA_NAMES = ("left", "head", "right")
DEFAULT_OPERATOR_VIDEO_ROOT = (
    Path(__file__).resolve().parents[3] / "Log" / "operator_recordings"
)


def _safe_folder_name(value: str) -> str:
    normalized = "".join(
        character if character.isalnum() or character in {"-", "_", "."} else "-"
        for character in str(value).strip()
    ).strip("-.")
    return normalized or "unregistered-deployment"


class OperatorVideoRecorder:
    """Record the three JPEG previews exposed by the PiPER operator UI."""

    def __init__(
        self,
        *,
        root: str | Path = DEFAULT_OPERATOR_VIDEO_ROOT,
        fps: float = 10.0,
        max_seconds: float = 600.0,
    ) -> None:
        self.root = Path(root).expanduser().resolve()
        self.fps = float(fps)
        self.max_seconds = float(max_seconds)
        if not 0 < self.fps <= 30:
            raise ValueError("Piper operator-video FPS must be in (0, 30]")
        if not 1 <= self.max_seconds <= 3600:
            raise ValueError(
                "Piper operator-video maximum duration must be in [1, 3600]"
            )
        self._lock = threading.Lock()
        self._entries: list[dict[str, Any]] = []
        self._last_frame_monotonic_ns = 0
        self._state: dict[str, Any] = self._idle_state()

    def _idle_state(self) -> dict[str, Any]:
        return {
            "status": "idle",
            "active": False,
            "capture_mode": "operator_three_camera",
            "cameras": list(CAMERA_NAMES),
            "record_fps": self.fps,
            "max_seconds": self.max_seconds,
            "root": str(self.root),
            "frame_count": 0,
            "message": "Ready to record all three operator UI views.",
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state)

    @staticmethod
    def _camera_detail(metadata: Mapping[str, Any]) -> dict[str, Any]:
        detail: dict[str, Any] = {}
        for key in (
            "width",
            "height",
            "timestamp_ns",
            "source_timestamp_ns",
            "device_time_ns",
            "device_frame_index",
            "jpeg_bytes",
        ):
            if key in metadata:
                detail[key] = int(metadata[key])
        for key in ("encoding", "timestamp_domain", "camera_role", "camera_serial"):
            if key in metadata:
                detail[key] = str(metadata[key])
        return detail

    def start(
        self,
        *,
        frames: Mapping[str, bytes],
        metadata: Mapping[str, Mapping[str, Any]],
        deployment: str,
        task: str,
        policy_label: str,
        model_info: Mapping[str, Any],
    ) -> dict[str, Any]:
        with self._lock:
            if self._state.get("active") or self._state.get("status") in {
                "finalizing",
                "limit_reached",
            }:
                raise ValueError(
                    "an operator-video recording is active or awaiting finalization"
                )
            missing = [name for name in CAMERA_NAMES if not frames.get(name)]
            if missing:
                raise ValueError(
                    "operator-video recording requires live camera previews: "
                    + ", ".join(missing)
                )
            started_timestamp_ns = time.time_ns()
            session_id = (
                time.strftime("%Y%m%d-%H%M%S")
                + f"-{started_timestamp_ns % 1_000_000_000:09d}"
            )
            deployment_name = _safe_folder_name(deployment)
            directory = self.root / deployment_name / session_id
            try:
                for camera_name in CAMERA_NAMES:
                    (directory / ".frames" / f"camera_{camera_name}").mkdir(
                        parents=True, exist_ok=False
                    )
            except OSError as exc:
                raise ValueError(
                    f"cannot create operator-video directory: {exc}"
                ) from exc
            self._entries = []
            self._last_frame_monotonic_ns = 0
            self._state = {
                "status": "recording",
                "active": True,
                "capture_mode": "operator_three_camera",
                "cameras": list(CAMERA_NAMES),
                "record_fps": self.fps,
                "max_seconds": self.max_seconds,
                "root": str(self.root),
                "deployment": str(deployment).strip() or deployment_name,
                "task": str(task),
                "policy_label": str(policy_label),
                "model_info": dict(model_info),
                "session_id": session_id,
                "directory": str(directory),
                "started_timestamp_ns": started_timestamp_ns,
                "frame_count": 0,
                "duration_seconds": 0.0,
                "message": "Recording left, head, and right operator UI views.",
            }
            self._record_locked(frames, metadata, force=True)
            return dict(self._state)

    def update(
        self,
        trigger_camera: str,
        *,
        frames: Mapping[str, bytes],
        metadata: Mapping[str, Mapping[str, Any]],
    ) -> None:
        if str(trigger_camera) != "head":
            return
        with self._lock:
            if not self._state.get("active"):
                return
            if any(not frames.get(name) for name in CAMERA_NAMES):
                self._state["message"] = (
                    "Recording paused while a camera preview is unavailable."
                )
                return
            self._record_locked(frames, metadata)

    def _record_locked(
        self,
        frames: Mapping[str, bytes],
        metadata: Mapping[str, Mapping[str, Any]],
        *,
        force: bool = False,
    ) -> None:
        now_monotonic_ns = time.monotonic_ns()
        period_ns = int(1_000_000_000 / self.fps)
        if (
            not force
            and self._last_frame_monotonic_ns
            and now_monotonic_ns - self._last_frame_monotonic_ns < period_ns
        ):
            return
        maximum_frames = max(2, int(self.fps * self.max_seconds))
        if len(self._entries) >= maximum_frames:
            self._state["active"] = False
            self._state["status"] = "limit_reached"
            self._state["message"] = (
                "Maximum recording duration reached; press Stop & Save."
            )
            return
        frame_number = len(self._entries) + 1
        directory = Path(self._state["directory"])
        try:
            for camera_name in CAMERA_NAMES:
                frame_path = (
                    directory
                    / ".frames"
                    / f"camera_{camera_name}"
                    / f"frame_{frame_number:06d}.jpg"
                )
                frame_path.write_bytes(frames[camera_name])
        except OSError as exc:
            self._state["active"] = False
            self._state["status"] = "error"
            self._state["message"] = f"Operator-video frame write failed: {exc}"
            raise ValueError(self._state["message"]) from exc
        recorded_timestamp_ns = time.time_ns()
        self._entries.append(
            {
                "video_frame_index": frame_number - 1,
                "recorded_timestamp_ns": recorded_timestamp_ns,
                "cameras": {
                    name: self._camera_detail(metadata.get(name, {}))
                    for name in CAMERA_NAMES
                },
            }
        )
        self._last_frame_monotonic_ns = now_monotonic_ns
        self._state["frame_count"] = frame_number
        self._state["duration_seconds"] = round(
            max(
                0.0,
                (recorded_timestamp_ns - self._state["started_timestamp_ns"])
                / 1e9,
            ),
            3,
        )
        self._state["message"] = (
            "Recording left, head, and right operator UI views."
        )

    @staticmethod
    def _encode_mp4(frames_dir: Path, output_path: Path, fps: float) -> str | None:
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            return "ffmpeg is unavailable; JPEG frames were retained"
        try:
            subprocess.run(
                [
                    ffmpeg,
                    "-nostdin",
                    "-v",
                    "error",
                    "-y",
                    "-framerate",
                    f"{fps:.8f}",
                    "-i",
                    str(frames_dir / "frame_%06d.jpg"),
                    "-c:v",
                    "libx264",
                    "-preset",
                    "veryfast",
                    "-vf",
                    "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                    "-pix_fmt",
                    "yuv420p",
                    "-movflags",
                    "+faststart",
                    str(output_path),
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=300,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return f"MP4 encoding failed: {exc}"
        return None

    @staticmethod
    def _encode_concat_mp4(
        frames_root: Path,
        output_path: Path,
        fps: float,
        *,
        tile_width: int = 640,
        tile_height: int = 360,
    ) -> str | None:
        """Encode a synchronized horizontal left/head/right composite video."""
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            return "ffmpeg is unavailable; JPEG frames were retained"
        filter_parts = []
        for index, camera_name in enumerate(CAMERA_NAMES):
            filter_parts.append(
                (
                    f"[{index}:v]scale={tile_width}:{tile_height}:"
                    "force_original_aspect_ratio=decrease,"
                    f"pad={tile_width}:{tile_height}:(ow-iw)/2:(oh-ih)/2,"
                    f"setsar=1[v{index}]"
                )
            )
        filter_parts.append(
            "[v0][v1][v2]hstack=inputs=3:shortest=1[vout]"
        )
        command = [
            ffmpeg,
            "-nostdin",
            "-v",
            "error",
            "-y",
        ]
        for camera_name in CAMERA_NAMES:
            command.extend(
                [
                    "-framerate",
                    f"{fps:.8f}",
                    "-i",
                    str(
                        frames_root
                        / f"camera_{camera_name}"
                        / "frame_%06d.jpg"
                    ),
                ]
            )
        command.extend(
            [
                "-filter_complex",
                ";".join(filter_parts),
                "-map",
                "[vout]",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(output_path),
            ]
        )
        try:
            subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
                timeout=300,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return f"Composite MP4 encoding failed: {exc}"
        return None

    def stop(self) -> dict[str, Any]:
        with self._lock:
            if not self._state.get("active") and self._state.get("status") != (
                "limit_reached"
            ):
                raise ValueError("no operator-video recording is active")
            self._state["active"] = False
            self._state["status"] = "finalizing"
            stopped_timestamp_ns = time.time_ns()
            entries = [dict(entry) for entry in self._entries]
            recording = dict(self._state)
        if len(entries) < 2:
            with self._lock:
                self._state["status"] = "error"
                self._state["message"] = (
                    "Record long enough to capture at least two three-camera samples."
                )
            raise ValueError(self._state["message"])

        directory = Path(recording["directory"])
        videos: dict[str, str] = {}
        video_errors: dict[str, str] = {}
        for camera_name in CAMERA_NAMES:
            relative_video = f"camera_{camera_name}.mp4"
            output_path = directory / relative_video
            error = self._encode_mp4(
                directory / ".frames" / f"camera_{camera_name}",
                output_path,
                self.fps,
            )
            if error is None:
                videos[camera_name] = relative_video
            else:
                video_errors[camera_name] = error

        concat_relative_video = "camera_concat.mp4"
        concat_error = self._encode_concat_mp4(
            directory / ".frames",
            directory / concat_relative_video,
            self.fps,
        )
        if concat_error is None:
            videos["concat"] = concat_relative_video
        else:
            video_errors["concat"] = concat_error

        duration_seconds = max(
            0.0,
            (stopped_timestamp_ns - int(recording["started_timestamp_ns"])) / 1e9,
        )
        manifest_path = directory / "recording_manifest.json"
        manifest = {
            "schema_version": "xpolicy.operator_video/v1",
            "session_id": recording["session_id"],
            "source": "xpolicy.operator_ui_preview",
            "deployment": recording["deployment"],
            "task": recording["task"],
            "policy_label": recording["policy_label"],
            "model_info": recording["model_info"],
            "cameras": list(CAMERA_NAMES),
            "videos": videos,
            "video_index": "video_index.json",
            "video_order": [*CAMERA_NAMES, "concat"],
            "concat_layout": {
                "video_id": "concat",
                "order": list(CAMERA_NAMES),
                "orientation": "horizontal",
                "tile_width": 640,
                "tile_height": 360,
            },
            "started_timestamp_ns": recording["started_timestamp_ns"],
            "stopped_timestamp_ns": stopped_timestamp_ns,
            "duration_seconds": round(duration_seconds, 3),
            "record_fps": self.fps,
            "frame_count": len(entries),
            "entries": entries,
        }
        temporary_manifest = manifest_path.with_suffix(".json.tmp")
        try:
            temporary_manifest.write_text(json.dumps(manifest, indent=2) + "\n")
            temporary_manifest.replace(manifest_path)
        except OSError as exc:
            with self._lock:
                self._state["status"] = "error"
                self._state["message"] = f"Operator-video manifest write failed: {exc}"
            raise ValueError(self._state["message"]) from exc

        video_index_path = directory / "video_index.json"
        indexed_videos = []
        for camera_name in CAMERA_NAMES:
            indexed_videos.append(
                {
                    "video_id": camera_name,
                    "path": videos.get(camera_name),
                    "kind": "camera",
                    "camera": camera_name,
                    "frame_count": len(entries),
                    "status": "ready"
                    if camera_name in videos
                    else "error",
                    **(
                        {"error": video_errors[camera_name]}
                        if camera_name in video_errors
                        else {}
                    ),
                }
            )
        indexed_videos.append(
            {
                "video_id": "concat",
                "path": videos.get("concat"),
                "kind": "composite",
                "camera_order": list(CAMERA_NAMES),
                "layout": "horizontal",
                "tile_width": 640,
                "tile_height": 360,
                "frame_count": len(entries),
                "status": "ready" if "concat" in videos else "error",
                **(
                    {"error": video_errors["concat"]}
                    if "concat" in video_errors
                    else {}
                ),
            }
        )
        video_index = {
            "schema_version": "xpolicy.operator_video_index/v1",
            "session_id": recording["session_id"],
            "recording_manifest": manifest_path.name,
            "record_fps": self.fps,
            "frame_count": len(entries),
            "videos": indexed_videos,
            "frames": [
                {
                    "video_frame_index": entry["video_frame_index"],
                    "recorded_timestamp_ns": entry["recorded_timestamp_ns"],
                    "video_frame_indices": {
                        video_id: entry["video_frame_index"]
                        for video_id in (*CAMERA_NAMES, "concat")
                    },
                    "camera_source_frames": {
                        camera_name: entry["cameras"].get(camera_name, {})
                        for camera_name in CAMERA_NAMES
                    },
                }
                for entry in entries
            ],
        }
        temporary_index = video_index_path.with_suffix(".json.tmp")
        try:
            temporary_index.write_text(json.dumps(video_index, indent=2) + "\n")
            temporary_index.replace(video_index_path)
        except OSError as exc:
            with self._lock:
                self._state["status"] = "error"
                self._state["message"] = f"Video index write failed: {exc}"
            raise ValueError(self._state["message"]) from exc

        if not video_errors:
            shutil.rmtree(directory / ".frames")
        absolute_videos = {
            name: str(directory / relative_path)
            for name, relative_path in videos.items()
        }
        with self._lock:
            self._state.update(
                {
                    "status": "ready" if not video_errors else "partial",
                    "active": False,
                    "stopped_timestamp_ns": stopped_timestamp_ns,
                    "duration_seconds": round(duration_seconds, 3),
                    "frame_count": len(entries),
                    "manifest_path": str(manifest_path),
                    "video_index_path": str(video_index_path),
                    "video_paths": absolute_videos,
                    "video_errors": video_errors,
                    "temporary_frames_retained": bool(video_errors),
                    "message": (
                        "Saved three camera videos, camera_concat.mp4, and indexes."
                        if not video_errors
                        else "Some MP4 encodes failed; JPEG frames were retained."
                    ),
                }
            )
            return dict(self._state)
