"""The storage tier that holds session artifacts (sessions, logs, recordings).

`RAX_ARTIFACT_ROOT` selects it explicitly; otherwise it is the repository's `runtime`
directory, usually a link to a large data disk.
"""
import os
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
ENVIRONMENT = "RAX_ARTIFACT_ROOT"


def artifact_root() -> Path:
    configured = os.environ.get(ENVIRONMENT)
    if configured:
        return Path(configured).resolve()
    return (PROJECT / "runtime").resolve()


def require_artifact_path(path, what="Session artifacts") -> Path:
    path = Path(path).resolve()
    root = artifact_root()
    if not path.is_relative_to(root):
        raise ValueError(f"{what} must be under {root} (set {ENVIRONMENT} to change it)")
    return path
