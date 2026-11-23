"""Opt-in bounded head stereo export inside the existing RealSense owner.

Only this owner's head pipeline enables IR1/IR2. The bounded queue replaces
pending packets instead of delaying RGB for disk I/O. No SDK import, pipeline,
robot action, model inference, or camera option write occurs in this module.
"""
from __future__ import annotations

import importlib
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Any, Mapping

import numpy as np


def sink_from_env(environ: Mapping[str, str] | None = None) -> HeadStereoExport | None:
    env = os.environ if environ is None else environ
    output = env.get("REALSENSE_STEREO_EXPORT_DIR", "").strip()
    if not output:
        return None
    adapter = env.get("REALSENSE_STEREO_ADAPTER_ROOT", "").strip()
    calibration = env.get("REALSENSE_STEREO_CALIBRATION", "").strip()
    if not adapter or not calibration:
        raise ValueError("stereo export requires explicit adapter root and calibration")
    root = Path(adapter).resolve(strict=True)
    expected = root / "real_robot/piper/stereo_packet.py"
    if not expected.is_file():
        raise ValueError("stereo packet adapter is missing from explicit root")
    sys.path.insert(0, str(root))
    module = importlib.import_module("real_robot.piper.stereo_packet")
    if Path(module.__file__).resolve() != expected:
        raise ValueError("stereo packet module came from another root")
    return HeadStereoExport(
        json.loads(Path(calibration).read_text()), Path(output), module,
        max_packets=int(env.get("REALSENSE_STEREO_MAX_PACKETS", "120")),
        interval_ms=float(env.get("REALSENSE_STEREO_INTERVAL_MS", "500")),
        continuous=env.get("REALSENSE_STEREO_CONTINUOUS") == "1",
    )


class HeadStereoExport:
    def __init__(self, calibration, output, packet_module, *, max_packets=120, interval_ms=500, continuous=False):
        valid_count = (max_packets == 0) if continuous else (1 <= max_packets <= 10000)
        if not valid_count or not np.isfinite(interval_ms) or interval_ms < (500 if continuous else 0):
            raise ValueError("invalid bounded stereo export count or interval")
        self.calibration = calibration
        self.identity = packet_module.calibration_identity(calibration)
        self.api = packet_module
        self.output = Path(output).resolve()
        # A new directory prevents mixing observations across runs or stale latest.
        self.output.mkdir(parents=True, exist_ok=False)
        self.max_packets = int(max_packets)
        self.interval_ns = int(interval_ms * 1_000_000)
        self.pending: queue.Queue = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.written = self.queued = self.dropped = self.rejected = 0
        self.error = ""
        self.writer_failed = False
        self.generation = ""
        self.guard = None
        self.last_offer_ns = -1
        self.thread = threading.Thread(target=self._write_loop, name="head-stereo-export", daemon=True)
        self.thread.start()

    def validate_camera(self, serial, width, height, fps):
        if serial != self.calibration["serial"]:
            raise ValueError("head stereo calibration does not match owner serial")
        for role in ("color", "ir_left", "ir_right"):
            profile = self.calibration["profiles"][role]
            if (width, height, fps) != (profile["width"], profile["height"], profile["fps"]):
                raise ValueError("head stereo calibration does not match owner stream dimensions/rate")

    def configure_streams(self, config, rs):
        for role, index in (("ir_left", 1), ("ir_right", 2)):
            profile = self.calibration["profiles"][role]
            config.enable_stream(rs.stream.infrared, index, profile["width"],
                                 profile["height"], rs.format.y8, profile["fps"])

    def capture(self, frameset, *, serial, owner_generation, host_received_unix_ns):
        with self.lock:
            if (self.max_packets > 0 and self.written >= self.max_packets) or self.writer_failed or self.stop.is_set():
                return
        if self.generation != owner_generation:
            self.generation = owner_generation
            self.guard = self.api.PacketSequenceGuard(owner_generation)
            self.last_offer_ns = -1
        if host_received_unix_ns - self.last_offer_ns < self.interval_ns:
            return
        self.last_offer_ns = host_received_unix_ns
        try:
            arrays, packet = self.api.capture_head_packet(
                frameset, self.calibration, serial=serial,
                owner_generation=owner_generation,
                host_received_unix_ns=host_received_unix_ns,
            )
            timing = self.api.validate_packet(arrays, packet, self.calibration, mode="live")
            self.guard.accept(packet)
            packet["capture_timing_validation"] = timing
        except Exception as exc:
            # Stereo errors fail closed for depth only; RGB has its existing gate.
            with self.lock:
                self.rejected += 1
                self.error = f"{type(exc).__name__}: {exc}"
            return
        try:
            self.pending.put_nowait((arrays, packet))
        except queue.Full:
            try:
                self.pending.get_nowait()
                with self.lock:
                    self.dropped += 1
            except queue.Empty:
                pass
            self.pending.put_nowait((arrays, packet))
        with self.lock:
            self.queued += 1
            self.error = ""

    def _write_loop(self):
        while not self.stop.is_set() or not self.pending.empty():
            try:
                arrays, packet = self.pending.get(timeout=0.1)
            except queue.Empty:
                continue
            with self.lock:
                if (self.max_packets > 0 and self.written >= self.max_packets):
                    continue
                count = self.written + 1
            name = f"packet-{count:06d}-{packet['sample_id']}"
            folder = self.output / name
            try:
                folder.mkdir()
                # The pointer is published only after both files are closed.
                np.savez(folder / "images.npz", **arrays)
                (folder / "packet.json").write_text(json.dumps(packet, indent=2) + "\n")
                pointer = {
                    "schema": "junqi.piper.head_stereo_latest.v1", "packet_dir": name,
                    "sample_id": packet["sample_id"], "owner_generation": packet["owner_generation"],
                    "host_received_unix_ns": packet["host_received_unix_ns"],
                    "exported_unix_ns": time.time_ns(), "packets_written": count,
                }
                partial = self.output / ".latest.partial.json"
                partial.write_text(json.dumps(pointer, indent=2) + "\n")
                partial.replace(self.output / "latest.json")
                with self.lock:
                    self.written = count
            except Exception as exc:
                with self.lock:
                    self.writer_failed = True
                    self.error = f"writer {type(exc).__name__}: {exc}"
                return

    def status(self):
        with self.lock:
            status = ("ERROR" if self.writer_failed else "COMPLETE" if (self.max_packets > 0 and self.written >= self.max_packets)
                      else "DEGRADED" if self.error else "EXPORTING")
            return {"status": status, "packets_written": self.written,
                    "packets_queued": self.queued, "pending_replaced": self.dropped,
                    "packets_rejected": self.rejected, "error": self.error,
                    "max_packets": self.max_packets, "output": str(self.output),
                    "calibration_payload_sha256": self.identity,
                    "robot_action_included": False}

    def close(self):
        self.stop.set()
        self.thread.join(timeout=5.0)
        status = self.status()
        status["writer_joined"] = not self.thread.is_alive()
        (self.output / "export-summary.json").write_text(json.dumps(status, indent=2) + "\n")
