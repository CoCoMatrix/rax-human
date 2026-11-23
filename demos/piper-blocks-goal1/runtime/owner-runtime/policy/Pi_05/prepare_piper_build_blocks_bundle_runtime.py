#!/usr/bin/env python3
"""Prepare mutable source and an environment for the build-blocks EMA bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
from urllib.request import urlopen
import uuid

from XPolicyLab.policy.Pi_05.piper_build_blocks_ema_bundle import (
    SOURCE_COMMIT,
    verify_piper_build_blocks_ema_bundle,
)


READY_SCHEMA = "xpolicylab.pi05_source_tree_bundle_runtime.v1"
TRANSPORT_PACKAGES = ("msgpack-numpy==0.4.8",)
GENERATED_NAMES = {".venv", "__pycache__", ".pytest_cache"}
AV_WHEEL_FILENAME = (
    "av-14.4.0-cp311-cp311-manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
)
AV_WHEEL_URL = (
    "https://files.pythonhosted.org/packages/f8/9a/"
    "8ffabfcafb42154b4b3a67d63f9b69e68fa8c34cb39ddd5cb813dd049ed4/"
    + AV_WHEEL_FILENAME
)
AV_WHEEL_SHA256 = "6bed513cbcb3437d0ae47743edc1f5b4a113c0b66cdd4e1aafc533abf5b2fbf2"
RUNTIME_OVERLAYS = (
    f"av==14.4.0@sha256:{AV_WHEEL_SHA256}",
    *TRANSPORT_PACKAGES,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_tree_sha256(root: Path) -> str:
    files: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(root):
        current = Path(dirpath)
        kept_dirs: list[str] = []
        for name in sorted(dirnames):
            child = current / name
            if name in GENERATED_NAMES:
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


def _verified_av_wheel(runtime_root: Path) -> Path:
    """Cache the exact CPython 3.11 Linux wheel absent from the Aliyun lock."""
    if platform.machine() != "x86_64":
        raise RuntimeError("the frozen PyAV overlay requires Linux x86_64")
    cache = runtime_root.parent.parent / "cache/wheels"
    cache.mkdir(parents=True, exist_ok=True)
    wheel = cache / AV_WHEEL_FILENAME
    if wheel.is_file():
        if _sha256(wheel) != AV_WHEEL_SHA256:
            raise ValueError(f"cached PyAV wheel hash mismatch: {wheel}")
        return wheel
    partial = wheel.with_name(f".{wheel.name}.partial-{uuid.uuid4().hex}")
    try:
        with urlopen(AV_WHEEL_URL, timeout=120) as response, partial.open(
            "wb"
        ) as output:
            shutil.copyfileobj(response, output, length=8 * 1024 * 1024)
        if _sha256(partial) != AV_WHEEL_SHA256:
            raise ValueError("downloaded PyAV wheel hash mismatch")
        os.replace(partial, wheel)
    except BaseException:
        partial.unlink(missing_ok=True)
        raise
    return wheel


def prepare(bundle: Path, runtime_root: Path, uv: str) -> dict[str, str | bool]:
    identity = verify_piper_build_blocks_ema_bundle(bundle)
    bundle = bundle.expanduser().resolve()
    runtime_root = runtime_root.expanduser().resolve()
    source_root = bundle / "runtime/source"
    source_lock = source_root / "uv.lock"
    lock_sha = _sha256(source_lock)
    bundle_source_tree_sha = _source_tree_sha256(source_root)
    ready_path = runtime_root / "_XPL_BUNDLE_RUNTIME_READY.json"
    python = runtime_root / ".venv/bin/python"
    if ready_path.is_file() and python.is_file():
        ready = json.loads(ready_path.read_text(encoding="utf-8"))
        if (
            ready.get("schema") == READY_SCHEMA
            and ready.get("package_id") == identity["package_id"]
            and ready.get("source_commit") == SOURCE_COMMIT
            and ready.get("uv_lock_sha256") == lock_sha
            and ready.get("runtime_overlays") == list(RUNTIME_OVERLAYS)
            and ready.get("source_tree_sha256") == bundle_source_tree_sha
            and _source_tree_sha256(runtime_root) == bundle_source_tree_sha
        ):
            subprocess.run(
                [
                    str(python),
                    "-c",
                    "import av,msgpack_numpy; assert av.__version__ == '14.4.0'",
                ],
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
    try:
        shutil.copytree(source_root, temporary, symlinks=False)
        if not (temporary / "pyproject.toml").is_file():
            raise ValueError("bundle runtime/source has no root pyproject.toml")
        if _sha256(temporary / "uv.lock") != lock_sha:
            raise ValueError("copied source uv.lock differs from the bundle")
        if _source_tree_sha256(temporary) != bundle_source_tree_sha:
            raise ValueError("copied runtime source tree differs from the bundle")
        os.replace(temporary, runtime_root)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    uv_executable = shutil.which(uv)
    if uv_executable is None:
        raise FileNotFoundError(f"uv executable not found: {uv!r}")
    subprocess.run(
        [
            uv_executable,
            "sync",
            "--frozen",
            "--project",
            str(runtime_root),
            "--no-install-package",
            "av",
        ],
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
            "--no-deps",
            str(_verified_av_wheel(runtime_root)),
        ],
        cwd=runtime_root,
        check=True,
    )
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
    subprocess.run(
        [
            str(python),
            "-c",
            "import av,sys; assert sys.version_info[:2] == (3,11); "
            "assert av.__version__ == '14.4.0'",
        ],
        check=True,
    )
    if _source_tree_sha256(runtime_root) != bundle_source_tree_sha:
        raise ValueError("uv setup modified the frozen runtime source tree")
    payload = {
        "schema": READY_SCHEMA,
        "package_id": identity["package_id"],
        "source_commit": SOURCE_COMMIT,
        "uv_lock_sha256": lock_sha,
        "source_tree_sha256": bundle_source_tree_sha,
        "runtime_overlays": list(RUNTIME_OVERLAYS),
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
