"""JSON helpers shared by the MCP servers, dashboards and trial runtime."""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any
import uuid


def jsonable(value: Any) -> Any:
    """Convert numpy scalars/arrays, paths and other values into JSON-safe values.

    Duck-typed so MCP tool discovery does not import numpy; anything else becomes str.
    """
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return jsonable(value.tolist())
    if hasattr(value, "item"):
        try:
            return jsonable(value.item())
        except ValueError:
            pass
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def write_private_json(path: Path, value: Any, *, directory_mode: int = 0o777) -> None:
    """Atomically write operator-owned JSON readable only by its owner (0600)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=directory_mode)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        path.chmod(0o600)
    finally:
        if temporary.exists():
            temporary.unlink()
