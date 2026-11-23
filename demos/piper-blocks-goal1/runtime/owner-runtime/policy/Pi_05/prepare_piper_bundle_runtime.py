#!/usr/bin/env python3
"""Prepare a mutable environment from an immutable Piper bundle source archive."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import uuid

from XPolicyLab.policy.Pi_05.piper_multitask_bundle import (
    SOURCE_COMMIT,
    verify_piper_multitask_bundle,
)


READY_SCHEMA = "xpolicylab.pi05_bundle_runtime.v1"
TRANSPORT_PACKAGES = ("msgpack-numpy==0.4.8",)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_tree_sha256(root: Path) -> str:
    """Hash extracted source while excluding generated environment/cache files."""
    files: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(root):
        current = Path(dirpath)
        kept_dirs: list[str] = []
        for name in sorted(dirnames):
            child = current / name
            if name in {".venv", "__pycache__", ".pytest_cache"}:
                if child.is_symlink():
                    raise ValueError(
                        f"prepared runtime generated directory cannot be a symlink: {child}"
                    )
                continue
            if child.is_symlink():
                raise ValueError(f"prepared runtime source cannot contain symlink: {child}")
            kept_dirs.append(name)
        dirnames[:] = kept_dirs
        for name in sorted(filenames):
            path = current / name
            if name == "_XPL_BUNDLE_RUNTIME_READY.json" or name.endswith(".pyc"):
                continue
            if path.is_symlink():
                raise ValueError(f"prepared runtime source cannot contain symlink: {path}")
            files.append(path)
    digest = hashlib.sha256()
    for path in sorted(files, key=lambda item: item.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(_sha256(path)))
    return digest.hexdigest()


def _safe_extract(archive_path: Path, destination: Path) -> None:
    with tarfile.open(archive_path, "r:*") as archive:
        members = archive.getmembers()
        if not members:
            raise ValueError("bundle runtime source archive is empty")
        for member in members:
            relative = Path(member.name)
            if (
                not member.name
                or relative.is_absolute()
                or ".." in relative.parts
                or member.issym()
                or member.islnk()
                or member.isdev()
                or member.isfifo()
                or not (member.isfile() or member.isdir())
            ):
                raise ValueError(f"unsafe bundle runtime member: {member.name!r}")
        archive.extractall(destination, members=members)


def prepare(bundle: Path, runtime_root: Path, uv: str) -> dict[str, str | bool]:
    identity = verify_piper_multitask_bundle(bundle)
    bundle = bundle.expanduser().resolve()
    runtime_root = runtime_root.expanduser().resolve()
    source_archive = bundle / "runtime/source.tar"
    bundled_lock = bundle / "runtime/uv.lock"
    archive_sha = _sha256(source_archive)
    lock_sha = _sha256(bundled_lock)
    ready_path = runtime_root / "_XPL_BUNDLE_RUNTIME_READY.json"
    python = runtime_root / ".venv/bin/python"
    if ready_path.is_file() and python.is_file():
        ready = json.loads(ready_path.read_text(encoding="utf-8"))
        if (
            ready.get("schema") == READY_SCHEMA
            and ready.get("package_id") == identity["package_id"]
            and ready.get("source_commit") == SOURCE_COMMIT
            and ready.get("source_archive_sha256") == archive_sha
            and ready.get("uv_lock_sha256") == lock_sha
            and ready.get("transport_packages") == list(TRANSPORT_PACKAGES)
            and ready.get("source_tree_sha256") == _source_tree_sha256(runtime_root)
        ):
            subprocess.run(
                [str(python), "-c", "import msgpack_numpy"],
                check=True,
                stdout=subprocess.DEVNULL,
            )
            return {
                "ready": True,
                "reused": True,
                "runtime_source_root": str(runtime_root),
                "python": str(python),
            }
        raise ValueError(f"prepared runtime identity drift: {runtime_root}")
    if runtime_root.exists():
        raise FileExistsError(
            f"incomplete runtime already exists; inspect or remove it deliberately: {runtime_root}"
        )

    runtime_root.parent.mkdir(parents=True, exist_ok=True)
    temporary = runtime_root.with_name(
        f".{runtime_root.name}.source-partial-{uuid.uuid4().hex}"
    )
    temporary.mkdir()
    try:
        _safe_extract(source_archive, temporary)
        extracted_lock = temporary / "uv.lock"
        if not (temporary / "pyproject.toml").is_file():
            raise ValueError("bundle source archive has no root pyproject.toml")
        if not extracted_lock.is_file() or _sha256(extracted_lock) != lock_sha:
            raise ValueError("bundle source uv.lock differs from runtime/uv.lock")
        os.replace(temporary, runtime_root)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    uv_executable = shutil.which(uv)
    if uv_executable is None:
        raise FileNotFoundError(f"uv executable not found: {uv!r}")
    subprocess.run(
        [uv_executable, "sync", "--frozen", "--project", str(runtime_root)],
        cwd=runtime_root,
        check=True,
    )
    if not python.is_file():
        raise RuntimeError(f"uv sync did not create {python}")
    subprocess.run(
        [
            uv_executable,
            "pip",
            "install",
            "--python",
            str(python),
            *TRANSPORT_PACKAGES,
        ],
        cwd=runtime_root,
        check=True,
    )
    payload = {
        "schema": READY_SCHEMA,
        "package_id": identity["package_id"],
        "source_commit": SOURCE_COMMIT,
        "source_archive_sha256": archive_sha,
        "uv_lock_sha256": lock_sha,
        "source_tree_sha256": _source_tree_sha256(runtime_root),
        "transport_packages": list(TRANSPORT_PACKAGES),
        "python": ".venv/bin/python",
    }
    temporary_ready = ready_path.with_name(f".{ready_path.name}.{uuid.uuid4().hex}")
    temporary_ready.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary_ready, ready_path)
    return {
        "ready": True,
        "reused": False,
        "runtime_source_root": str(runtime_root),
        "python": str(python),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--uv", default="uv")
    args = parser.parse_args()
    print(json.dumps(prepare(args.bundle, args.runtime_root, args.uv), sort_keys=True))


if __name__ == "__main__":
    main()
