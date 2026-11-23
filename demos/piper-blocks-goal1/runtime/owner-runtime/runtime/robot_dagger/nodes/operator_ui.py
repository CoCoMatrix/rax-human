#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import threading
import time
import urllib.request
from urllib.parse import parse_qs, urlencode, urlparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
INDEX = ROOT / "ui" / "index.html"
LOG_DIR = ROOT / "logs"
ROBOT_PID = LOG_DIR / "operator-robot.pid"
ROBOT_LOG = LOG_DIR / "operator-robot.log"
MODELS = {"gr00t", "fastwam", "pi05"}
CHAINS = MODELS | {"qnexo", "ee"}
NAME = re.compile(r"^[a-zA-Z0-9_.:/+-]+$")
INTEGER = re.compile(r"^[1-9][0-9]*$")
FLOAT = re.compile(r"^(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)$")

STATE: dict[str, Any] = {
    "camera_at": 0.0,
    "robot_state": {},
    "model_action": {},
    "base_state": {},
    "robot_status": {},
    "model_status": {},
    "control_mode": "—",
    "action_chunk": {"id": None, "horizon": 0, "cursor": -1, "steps": []},
    "review_event_sequence": 0,
    "review_events": [],
}
CAMERA_NAMES = ("cam_head", "cam_left_wrist", "cam_right_wrist")
CAMERAS = dict.fromkeys(CAMERA_NAMES, b"")
LOCK = threading.Lock()
CAMERA_UPDATED = threading.Condition(LOCK)
CAMERA_SEQUENCES = dict.fromkeys(CAMERA_NAMES, 0)
ROBOT_LOCK = threading.Lock()


def camera_stream_name(value: str | None) -> str:
    name = str(value or "cam_head")
    if name not in CAMERA_NAMES:
        raise ValueError(f"未知相机流: {name}")
    return name


def text(value: Any, *, limit: int = 4096) -> str:
    value = str(value or "").strip()
    if len(value) > limit or "\x00" in value:
        raise ValueError("输入过长或包含空字符")
    return value


def option(command: list[str], flag: str, value: Any, pattern: re.Pattern[str] | None = None) -> None:
    value = text(value)
    if not value:
        return
    if pattern and not pattern.fullmatch(value):
        raise ValueError(f"{flag} 参数格式无效")
    command.extend((flag, value))


def model_command(payload: dict[str, Any], *, dry_run: bool = False) -> list[str]:
    model = text(payload.get("model"))
    if model not in MODELS:
        raise ValueError("未知模型")
    command = [str(ROOT / "scripts" / "launch_model.sh"), model]
    option(command, "--ckpt", payload.get("ckpt"))
    option(command, "--host", payload.get("host"), NAME)
    option(command, "--port", payload.get("port"), INTEGER)
    option(command, "--device", payload.get("device"), NAME)
    if model in {"fastwam", "pi05"}:
        option(command, "--config", payload.get("config"), NAME)
    if model == "fastwam":
        option(command, "--dataset-stats", payload.get("dataset_stats"))
        if payload.get("rtc") is not None:
            command.append("--rtc" if payload["rtc"] else "--no-rtc")
    if model == "pi05":
        option(command, "--task", payload.get("task"))
        option(command, "--chunk", payload.get("chunk"), INTEGER)
    command.append("--dry-run" if dry_run else "--background")
    return command


def robot_command(payload: dict[str, Any], *, dry_run: bool = False) -> list[str]:
    chain = text(payload.get("chain"))
    if chain not in CHAINS:
        raise ValueError("未知机器人链路")
    command = [str(ROOT / "scripts" / "launch_robot.sh"), chain]
    if chain not in {"qnexo", "ee"}:
        option(command, "--ckpt", payload.get("ckpt"))
        option(command, "--task", payload.get("task"))
        option(command, "--chunk", payload.get("chunk"), INTEGER)
        option(command, "--host", payload.get("host"), NAME)
        option(command, "--port", payload.get("port"), INTEGER)
    option(command, "--robot-id", payload.get("robot_id"), INTEGER)
    option(command, "--record-dir", payload.get("record_dir"))
    option(command, "--max-joint-step", payload.get("max_joint_step"), FLOAT)
    if payload.get("record") is not None:
        command.append("--record" if payload["record"] else "--no-record")
    if chain == "fastwam":
        if payload.get("rtc") is not None:
            command.append("--rtc" if payload["rtc"] else "--no-rtc")
        option(command, "--mink", payload.get("mink"), re.compile(r"^(off|shadow|enforce)$"))
        option(command, "--horizon", payload.get("horizon"), INTEGER)
        if payload.get("action_qp") is not None:
            command.append("--action-qp" if payload["action_qp"] else "--no-action-qp")
        if payload.get("spline") is not None:
            command.append("--spline" if payload["spline"] else "--no-spline")
        option(command, "--spline-factor", payload.get("spline_factor"), INTEGER)
    elif chain == "ee":
        option(command, "--translation-scale", payload.get("translation_scale"), FLOAT)
        option(command, "--rotation-scale", payload.get("rotation_scale"), FLOAT)
    if dry_run:
        command.append("--dry-run")
    return command


def run(command: list[str]) -> tuple[int, str]:
    result = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=30,
        check=False,
    )
    return result.returncode, result.stdout


def read_pid() -> int | None:
    try:
        pid = int(ROBOT_PID.read_text().strip())
    except (FileNotFoundError, ValueError):
        return None
    try:
        stat = Path(f"/proc/{pid}/stat").read_text().split()
        cwd = Path(f"/proc/{pid}/cwd").resolve()
        if stat[2] == "Z" or os.getpgid(pid) != pid or cwd != ROOT:
            raise ProcessLookupError
        return pid
    except (FileNotFoundError, IndexError, ValueError, ProcessLookupError, PermissionError):
        if session_pids(pid):
            return pid
        ROBOT_PID.unlink(missing_ok=True)
        return None


def session_pids(session_id: int) -> list[int]:
    pids = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
            if fields[0] != "Z" and int(fields[3]) == session_id:
                pids.append(int(proc.name))
        except (FileNotFoundError, IndexError, ValueError, PermissionError):
            pass
    return pids


def robot_sessions() -> list[int]:
    sessions = set()
    graph_root = f"{ROOT / 'graphs'}/"
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
            pid, session_id = int(proc.name), int(fields[3])
            args = (proc / "cmdline").read_bytes().decode(errors="replace").split("\0")
            if (
                fields[0] != "Z"
                and pid == session_id
                and "run" in args
                and any(Path(arg).name == "dora" for arg in args)
                and any(arg.startswith(graph_root) and arg.endswith(".yml") for arg in args)
            ):
                sessions.add(session_id)
        except (FileNotFoundError, IndexError, ValueError, PermissionError):
            pass
    return sorted(sessions)


def start_robot(command: list[str]) -> str:
    with ROBOT_LOCK:
        sessions = set(robot_sessions())
        if pid := read_pid():
            sessions.add(pid)
        if sessions:
            raise ValueError(f"机器人链路已在运行，session={sorted(sessions)}")
        LOG_DIR.mkdir(exist_ok=True)
        stream = ROBOT_LOG.open("ab", buffering=0)
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdin=subprocess.DEVNULL,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        ROBOT_PID.write_text(f"{process.pid}\n")
        time.sleep(0.2)
        if process.poll() is not None:
            ROBOT_PID.unlink(missing_ok=True)
            raise RuntimeError(ROBOT_LOG.read_text(errors="replace")[-4000:])
        return f"机器人链路已启动，pid={process.pid}"


def stop_robot() -> str:
    with ROBOT_LOCK:
        sessions = set(robot_sessions())
        if pid := read_pid():
            sessions.add(pid)
        if not sessions:
            return "机器人链路已经停止"
        for session_id in sessions:
            try:
                os.killpg(session_id, signal.SIGINT)
            except ProcessLookupError:
                pass
        for _ in range(50):
            if not any(session_pids(session_id) for session_id in sessions):
                ROBOT_PID.unlink(missing_ok=True)
                return "机器人链路已停止"
            time.sleep(0.1)
        survivors = [
            survivor
            for session_id in sessions
            for survivor in session_pids(session_id)
        ]
        for survivor in survivors:
            try:
                os.kill(survivor, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for _ in range(10):
            if not any(session_pids(session_id) for session_id in sessions):
                ROBOT_PID.unlink(missing_ok=True)
                return "机器人链路未正常退出，已强制清理全部节点"
            time.sleep(0.1)
        remaining = [
            survivor
            for session_id in sessions
            for survivor in session_pids(session_id)
        ]
        raise RuntimeError(f"机器人链路仍有残留进程：{remaining}")


def tail(path: Path, size: int = 12000) -> str:
    try:
        with path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell() - size))
            return stream.read().decode(errors="replace")
    except FileNotFoundError:
        return ""


def public_state(model: str) -> dict[str, Any]:
    with LOCK:
        state = json.loads(json.dumps(STATE))
    state["camera_online"] = time.monotonic() - state.pop("camera_at", 0.0) < 2
    sessions = set(robot_sessions())
    if pid := read_pid():
        sessions.add(pid)
    state["robot_process"] = {
        "running": bool(sessions),
        "pid": min(sessions) if sessions else None,
        "sessions": sorted(sessions),
    }
    if model in MODELS:
        code, output = run([str(ROOT / "scripts" / "launch_model.sh"), "status", model])
        state["model_process"] = {"ok": code == 0, "text": output}
    return state


class Handler(BaseHTTPRequestHandler):
    server_version = "GR3OperatorUI/1"

    def log_message(self, format: str, *args: Any) -> None:
        if self.path.startswith(("/api/", "/telemetry/")):
            return
        print(f"UI {self.address_string()} {format % args}")

    def send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def json(self, status: int, value: Any) -> None:
        self.send(status, json.dumps(value, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    def payload(self, limit: int = 1_000_000) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length < 0 or length > limit:
            raise ValueError("请求体过大")
        value = json.loads(self.rfile.read(length) or b"{}")
        if not isinstance(value, dict):
            raise ValueError("请求体必须是 JSON 对象")
        return value

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        route = parsed.path
        query = parse_qs(parsed.query)
        if route == "/":
            self.send(200, INDEX.read_bytes(), "text/html; charset=utf-8")
        elif route == "/api/state":
            model = self.path.partition("model=")[2].partition("&")[0]
            self.json(200, public_state(model))
        elif route == "/api/telemetry":
            with LOCK:
                state = json.loads(json.dumps(STATE))
            self.json(200, state)
        elif route == "/api/camera":
            camera_name = camera_stream_name(query.get("name", [None])[0])
            with LOCK:
                frame = CAMERAS[camera_name]
            self.send(200 if frame else 204, frame, "image/jpeg")
        elif route == "/api/camera.mjpg":
            camera_name = camera_stream_name(query.get("name", [None])[0])
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = -1
            try:
                while True:
                    with CAMERA_UPDATED:
                        CAMERA_UPDATED.wait_for(
                            lambda: CAMERA_SEQUENCES[camera_name] != sequence,
                            timeout=1.0,
                        )
                        frame = CAMERAS[camera_name]
                        sequence = CAMERA_SEQUENCES[camera_name]
                    if not frame:
                        continue
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(frame)}\r\n\r\n".encode()
                        + frame
                        + b"\r\n"
                    )
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return
        elif route == "/api/logs":
            target = self.path.partition("target=")[2].partition("&")[0]
            if target == "robot":
                value = tail(ROBOT_LOG)
            elif target in MODELS:
                names = {
                    "gr00t": "gr00t-jiaxin-server.log",
                    "fastwam": "fastwam-server.log",
                    "pi05": "pi05-server.log",
                }
                value = tail(LOG_DIR / names[target])
            else:
                value = ""
            self.json(200, {"text": value})
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            parsed = urlparse(self.path)
            if parsed.path == "/telemetry/camera":
                query = parse_qs(parsed.query)
                camera_name = camera_stream_name(query.get("name", [None])[0])
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 2_000_000:
                    raise ValueError("图像大小无效")
                frame = self.rfile.read(length)
                with CAMERA_UPDATED:
                    CAMERAS[camera_name] = frame
                    CAMERA_SEQUENCES[camera_name] += 1
                    STATE["camera_at"] = time.monotonic()
                    CAMERA_UPDATED.notify_all()
                self.send(204, b"", "text/plain")
                return
            payload = self.payload()
            if self.path == "/telemetry/event":
                update_telemetry(payload)
                self.send(204, b"", "text/plain")
            elif self.path == "/api/model/start":
                model = text(payload.get("model"))
                if model not in MODELS:
                    raise ValueError("未知模型")
                _, status = run(
                    [str(ROOT / "scripts" / "launch_model.sh"), "status", model]
                )
                if "status: STOPPED" not in status:
                    raise ValueError(f"模型服务未停止：\n{status}")
                code, output = run(model_command(payload))
                self.json(200 if code == 0 else 400, {"ok": code == 0, "output": output})
            elif self.path == "/api/model/stop":
                model = text(payload.get("model"))
                if model not in MODELS:
                    raise ValueError("未知模型")
                code, output = run([str(ROOT / "scripts" / "launch_model.sh"), "stop", model])
                self.json(200 if code == 0 else 400, {"ok": code == 0, "output": output})
            elif self.path == "/api/robot/start":
                if not payload.get("confirm_hardware"):
                    raise ValueError("启动真机前必须确认安全区域")
                self.json(200, {"ok": True, "output": start_robot(robot_command(payload))})
            elif self.path == "/api/robot/stop":
                self.json(200, {"ok": True, "output": stop_robot()})
            elif self.path == "/api/preview":
                model = model_command(payload["model"], dry_run=True) if payload.get("model") else None
                robot = robot_command(payload["robot"], dry_run=True) if payload.get("robot") else None
                outputs = []
                for command in (model, robot):
                    if command:
                        code, output = run(command)
                        outputs.append(output)
                        if code:
                            self.json(400, {"ok": False, "output": "\n".join(outputs)})
                            return
                self.json(200, {"ok": True, "output": "\n".join(outputs)})
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except (ValueError, KeyError, json.JSONDecodeError) as error:
            self.json(400, {"ok": False, "output": str(error)})
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            self.json(500, {"ok": False, "output": str(error)})


def update_telemetry(payload: dict[str, Any]) -> None:
    kind = text(payload.get("kind"), limit=64)
    data = payload.get("data")
    metadata = payload.get("metadata") or {}
    with LOCK:
        if kind in {"robot_state", "base_state", "robot_status", "model_status"}:
            STATE[kind] = data
        elif kind == "control_mode":
            STATE[kind] = data
        elif kind == "model_action":
            STATE["model_action"] = data
            chunk = STATE["action_chunk"]
            chunk_id = metadata.get("model_chunk_id", metadata.get("chunk_id"))
            cursor = int(metadata.get("model_chunk_step_index", len(chunk["steps"])))
            if chunk["id"] != chunk_id or cursor == 0:
                chunk = {
                    "id": chunk_id,
                    "horizon": int(metadata.get("model_action_horizon", 0)),
                    "cursor": cursor,
                    "steps": [],
                }
                STATE["action_chunk"] = chunk
            while len(chunk["steps"]) <= cursor:
                chunk["steps"].append({})
            chunk["steps"][cursor] = data
            chunk["cursor"] = cursor
        elif kind == "review_input":
            sequence = int(STATE["review_event_sequence"]) + 1
            STATE["review_event_sequence"] = sequence
            events = STATE["review_events"]
            events.append(
                {
                    "sequence": sequence,
                    "event": text(data, limit=64),
                    "source_timestamp_ns": int(
                        metadata.get("source_timestamp_ns")
                        or metadata.get("timestamp_ns")
                        or time.time_ns()
                    ),
                }
            )
            del events[:-64]


def post(url: str, body: bytes, content_type: str) -> None:
    try:
        urllib.request.urlopen(
            urllib.request.Request(url, data=body, headers={"Content-Type": content_type}),
            timeout=0.15,
        ).close()
    except OSError:
        pass


def encode_camera_preview(frame, max_width: int, quality: int) -> bytes:
    import cv2

    if max_width < 1 or not 1 <= quality <= 95:
        raise ValueError("UI camera width/quality must be positive and quality <= 95")
    height, width = frame.shape[:2]
    if width > max_width:
        frame = cv2.resize(
            frame,
            (max_width, round(height * max_width / width)),
            interpolation=cv2.INTER_AREA,
        )
    ok, encoded = cv2.imencode(".jpg", frame, (cv2.IMWRITE_JPEG_QUALITY, quality))
    if not ok:
        raise ValueError("UI camera JPEG encoding failed")
    return encoded.tobytes()


def scalar(value: Any) -> Any:
    result = value.to_pylist()
    return result[0] if len(result) == 1 else result


def bridge() -> None:
    import numpy as np
    from dora import Node
    from farther_core.serde import JointStateSerde

    base_url = os.getenv("OPERATOR_UI_URL", "http://127.0.0.1:8090").rstrip("/")
    width = int(os.getenv("IMAGE_WIDTH", "640"))
    height = int(os.getenv("IMAGE_HEIGHT", "400"))
    preview_width = int(os.getenv("OPERATOR_UI_CAMERA_MAX_WIDTH", "480"))
    jpeg_quality = int(os.getenv("OPERATOR_UI_CAMERA_JPEG_QUALITY", "65"))
    preview_fps = float(os.getenv("OPERATOR_UI_CAMERA_FPS", "15"))
    if (
        preview_width < 1
        or not 1 <= jpeg_quality <= 95
        or not 0 < preview_fps <= 30
    ):
        raise ValueError(
            "UI camera width/quality/fps must be positive, quality <= 95, fps <= 30"
        )
    preview_period = 1.0 / preview_fps
    camera_inputs = {
        "camera": "cam_head",
        "camera_cam_head": "cam_head",
        "camera_cam_left_wrist": "cam_left_wrist",
        "camera_cam_right_wrist": "cam_right_wrist",
    }
    last_camera = dict.fromkeys(CAMERA_NAMES, 0.0)
    last_state = {"robot_state": 0.0, "base_state": 0.0}
    node = Node()
    for event in node:
        if event.get("type") != "INPUT":
            continue
        kind = event["id"]
        now = time.monotonic()
        if kind in camera_inputs:
            camera_name = camera_inputs[kind]
            if now - last_camera[camera_name] < preview_period:
                continue
            last_camera[camera_name] = now
            event_metadata = event.get("metadata", {})
            frame_width = int(event_metadata.get("width", width))
            frame_height = int(event_metadata.get("height", height))
            frame = event["value"].to_numpy(zero_copy_only=False).view(np.uint8)
            try:
                frame = frame.reshape(frame_height, frame_width, 3)
            except ValueError:
                continue
            if str(event_metadata.get("encoding", "")).lower() == "rgb8":
                frame = frame[..., ::-1]
            encoded = encode_camera_preview(frame, preview_width, jpeg_quality)
            query = urlencode({"name": camera_name})
            post(f"{base_url}/telemetry/camera?{query}", encoded, "image/jpeg")
            continue
        if kind == "review_input":
            body = json.dumps(
                {
                    "kind": "review_input",
                    "data": scalar(event["value"]),
                    "metadata": event.get("metadata", {}),
                },
                separators=(",", ":"),
                default=str,
            ).encode()
            post(f"{base_url}/telemetry/event", body, "application/json")
            continue
        if kind in last_state:
            if now - last_state[kind] < 0.05:
                continue
            last_state[kind] = now
        if kind == "robot_state":
            data = JointStateSerde.deserialize(event["value"])
        elif kind == "model_action":
            data = JointStateSerde.deserialize(event["value"])
        elif kind in {"robot_status", "model_status"}:
            raw = scalar(event["value"])
            data = json.loads(raw) if isinstance(raw, str) else raw
        else:
            data = scalar(event["value"])
        body = json.dumps(
            {"kind": kind, "data": data, "metadata": event.get("metadata", {})},
            separators=(",", ":"),
            default=str,
        ).encode()
        post(f"{base_url}/telemetry/event", body, "application/json")


def serve(host: str, port: int) -> None:
    if host not in {"127.0.0.1", "localhost", "::1"}:
        raise SystemExit("operator UI only binds to loopback")
    LOG_DIR.mkdir(exist_ok=True)
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"XPolicyLab operator preview: http://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="XPolicyLab operator preview / Dora telemetry bridge"
    )
    parser.add_argument("mode", nargs="?", choices=("serve", "bridge"), default="bridge")
    parser.add_argument("--host", default=os.getenv("OPERATOR_UI_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("OPERATOR_UI_PORT", "8090")))
    args = parser.parse_args()
    serve(args.host, args.port) if args.mode == "serve" else bridge()


if __name__ == "__main__":
    main()
