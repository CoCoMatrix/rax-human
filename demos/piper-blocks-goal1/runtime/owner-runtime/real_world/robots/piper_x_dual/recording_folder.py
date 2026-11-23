"""Open a recorder-owned directory on the operator desktop, without robot IO."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess

from .operator_video import _safe_folder_name


def recording_folder(recording, *, root, deployment=None):
    root = Path(root).resolve()

    def checked(path):
        path = Path(path).resolve()
        if not path.is_relative_to(root):
            raise ValueError("录像路径超出指定录像目录")
        if not path.is_dir():
            raise ValueError("录像文件夹不存在")
        return path

    current = recording.get("directory")
    if current:
        return {"directory": str(checked(current)), "selection": "current"}
    # A formal runtime restart clears in-memory recorder state. Preserve the
    # useful destination by finding the latest saved session for this model.
    base = root / _safe_folder_name(deployment) if deployment is not None else root
    if base.is_dir():
        base = checked(base)
        for entry in sorted(base.iterdir(), key=lambda p: p.name, reverse=True):
            if entry.is_dir() and any((entry / name).is_file() for name in
                                     ("recording_manifest.json", "demo_manifest.json")):
                return {"directory": str(checked(entry)), "selection": "latest"}
    return {"directory": str(checked(root)), "selection": "root"}


def open_recording_folder(folder):
    """Only called with a server-selected folder after operator-token checks."""
    executable = shutil.which("gio")
    if not executable:
        raise ValueError("系统文件管理器不可用；请复制下方录像路径打开")
    env = dict(os.environ)
    env.setdefault("DISPLAY", ":1")
    runtime = env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    env.setdefault("DBUS_SESSION_BUS_ADDRESS", f"unix:path={runtime}/bus")
    authority = Path(runtime) / "gdm/Xauthority"
    if authority.is_file():
        env.setdefault("XAUTHORITY", str(authority))
    try:
        result = subprocess.run([executable, "open", folder["directory"]], env=env,
                                capture_output=True, text=True, timeout=5, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError("文件管理器未响应；请复制下方录像路径打开") from exc
    if result.returncode:
        raise ValueError("无法打开录像文件夹：" + (result.stderr.strip() or "桌面会话不可用"))
    return {**folder, "message": "已请求在机器人电脑的文件管理器中打开录像文件夹"}
