#!/usr/bin/env python3
"""Verify a packaged dual-PiPER bundle plus local OSS intake evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from pathlib import Path, PurePosixPath
from typing import Any


PACKAGE_MANIFEST = "package_manifest.json"
INTAKE_MANIFEST = "manifest.json"
INTAKE_SUMS = "SHA256SUMS"
LOCAL_EVIDENCE = {INTAKE_MANIFEST, INTAKE_SUMS}
DENY_FILE_NAMES = {".env", ".netrc", "rclone.conf"}
DENY_FILE_PREFIXES = ("id_ecdsa", "id_ed25519", "id_rsa")
DENY_FILE_SUFFIXES = (".pem",)
DENY_DIR_NAMES = {
    ".cache",
    ".git",
    ".oss-resume",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "node_modules",
    "wandb",
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
CHUNK_BYTES = 8 * 1024 * 1024
DATASET_ID = "dual_piper_desktop_cleanup_20260825_v1"
TRAIN_CONFIG = "pi05_dual_piper_official"
PROJECT_COMMIT = "9888bca00155c11877ff697f634f61fb945eb72a"
OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
PROMPTS = (
    "Close the laptop.",
    "Pick up one drink container and place it on the coaster.",
    "Pick up one mouse and place it on the mouse pad.",
    "Pick up one paper ball and place it in the trash bin.",
    "Pick up one pen and place it in the pen holder.",
)


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _safe_relative(value: Any) -> str:
    path = PurePosixPath(str(value))
    if not path.parts or path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe manifest path: {value!r}")
    return path.as_posix()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(CHUNK_BYTES), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _collect_files(root: Path) -> set[str]:
    files: set[str] = set()
    problems: list[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        current = Path(dirpath)
        kept_dirs: list[str] = []
        for name in sorted(dirnames):
            child = current / name
            relative = child.relative_to(root).as_posix()
            if child.is_symlink():
                problems.append(f"symlink directory: {relative}")
            elif name in DENY_DIR_NAMES:
                problems.append(f"forbidden directory: {relative}")
            else:
                kept_dirs.append(name)
        dirnames[:] = kept_dirs
        for name in sorted(filenames):
            child = current / name
            relative = child.relative_to(root).as_posix()
            lowered = name.lower()
            if child.is_symlink():
                problems.append(f"symlink file: {relative}")
            elif (
                lowered in DENY_FILE_NAMES
                or lowered.startswith(DENY_FILE_PREFIXES)
                or lowered.endswith(DENY_FILE_SUFFIXES)
            ):
                problems.append(f"secret-like file: {relative}")
            else:
                files.add(relative)
    if problems:
        raise ValueError("bundle policy violations:\n  " + "\n  ".join(problems))
    return files


def _indexed_entries(
    values: Any, *, path_key: str, size_key: str
) -> dict[str, tuple[int, str]]:
    if not isinstance(values, list) or not values:
        raise ValueError("manifest entries must be a non-empty list")
    result: dict[str, tuple[int, str]] = {}
    for entry in values:
        if not isinstance(entry, dict):
            raise ValueError("manifest entry must be an object")
        relative = _safe_relative(entry.get(path_key))
        size = entry.get(size_key)
        digest = str(entry.get("sha256", ""))
        if relative in result:
            raise ValueError(f"duplicate manifest path: {relative}")
        if not isinstance(size, int) or size < 0:
            raise ValueError(f"invalid size for {relative}")
        if not SHA256_RE.fullmatch(digest):
            raise ValueError(f"invalid SHA-256 for {relative}")
        result[relative] = (size, digest)
    return result


def _read_sums(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            digest, relative_value = line.split("  ", 1)
        except ValueError as exc:
            raise ValueError(f"invalid SHA256SUMS line {line_number}") from exc
        relative = _safe_relative(relative_value)
        if not SHA256_RE.fullmatch(digest) or relative in result:
            raise ValueError(f"invalid SHA256SUMS line {line_number}")
        result[relative] = digest
    if not result:
        raise ValueError("SHA256SUMS is empty")
    return result


def verify(root: Path) -> dict[str, Any]:
    root = root.expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f"bundle directory not found: {root}")
    actual_files = _collect_files(root)
    required_manifests = {PACKAGE_MANIFEST, INTAKE_MANIFEST, INTAKE_SUMS}
    missing_manifests = sorted(required_manifests - actual_files)
    if missing_manifests:
        raise ValueError("missing manifests: " + ", ".join(missing_manifests))

    package = _load_json(root / PACKAGE_MANIFEST)
    if package.get("schema") != "coco-bundle-manifest/1":
        raise ValueError(f"unsupported package manifest: {package.get('schema')!r}")
    package_entries = _indexed_entries(
        package.get("files"), path_key="path", size_key="size"
    )
    if package.get("file_count") != len(package_entries):
        raise ValueError("package manifest file_count mismatch")
    if package.get("total_bytes") != sum(size for size, _ in package_entries.values()):
        raise ValueError("package manifest total_bytes mismatch")

    intake = _load_json(root / INTAKE_MANIFEST)
    source = intake.get("source")
    if intake.get("schema_version") != 1:
        raise ValueError(f"unsupported intake manifest: {intake.get('schema_version')!r}")
    if not isinstance(source, dict) or source.get("provider") != "aliyun-oss":
        raise ValueError("intake manifest is not an Aliyun OSS source")
    intake_entries = _indexed_entries(
        intake.get("objects"), path_key="path", size_key="bytes"
    )
    if source.get("object_count") != len(intake_entries):
        raise ValueError("intake manifest object_count mismatch")
    if source.get("total_bytes") != sum(size for size, _ in intake_entries.values()):
        raise ValueError("intake manifest total_bytes mismatch")

    expected_remote = set(package_entries) | {PACKAGE_MANIFEST}
    if set(intake_entries) != expected_remote:
        raise ValueError("publisher package and OSS object inventories do not match")
    allowed_actual = expected_remote | LOCAL_EVIDENCE
    extras = sorted(actual_files - allowed_actual)
    missing = sorted(allowed_actual - actual_files)
    if extras or missing:
        details = [*(f"extra file: {item}" for item in extras)]
        details.extend(f"missing file: {item}" for item in missing)
        raise ValueError("bundle inventory mismatch:\n  " + "\n  ".join(details))
    sums = _read_sums(root / INTAKE_SUMS)
    if set(sums) != expected_remote:
        raise ValueError("SHA256SUMS inventory does not match OSS intake")

    for relative in sorted(expected_remote):
        path = root / relative
        size = path.stat().st_size
        intake_size, intake_digest = intake_entries[relative]
        if size != intake_size:
            raise ValueError(f"OSS size mismatch: {relative}")
        digest = _sha256(path)
        if digest != intake_digest or digest != sums[relative]:
            raise ValueError(f"OSS SHA-256 mismatch: {relative}")
        if relative in package_entries:
            package_size, package_digest = package_entries[relative]
            if size != package_size or digest != package_digest:
                raise ValueError(f"publisher manifest mismatch: {relative}")

    model_path = "checkpoint/model.safetensors"
    if model_path not in intake_entries:
        raise ValueError(f"model weights missing from bundle: {model_path}")
    contract = _load_json(root / "contract.json")
    ticket = _load_json(root / "deploy_input.json")
    checkpoint = _load_json(root / "checkpoint" / "metadata.json")
    validation = _load_json(root / "validation" / "synthetic_inference.json")
    template = _load_json(
        root
        / "runtime"
        / "project"
        / "configs"
        / "real"
        / "dual_piper_gripper_template.json"
    )
    robot = template.get("robot", {})
    policy = template.get("policy", {})
    safety = template.get("safety", {})
    step = int(contract.get("step", -1))
    package_id = f"desktop_cleanup_20260825_v1_pi05_raw_step{step}_ckpt"
    model_digest = intake_entries[model_path][1]
    checks = {
        "supported_step": step in {5000, 10000},
        "package_id": contract.get("package_id") == package_id,
        "ticket_package_id": ticket.get("package_id") == package_id,
        "manifest_package_id": package.get("package_id") == package_id,
        "model": contract.get("model") == "pi0.5",
        "train_config": contract.get("train_config") == TRAIN_CONFIG,
        "ticket_train_config": ticket.get("runtime", {}).get("train_config")
        == TRAIN_CONFIG,
        "dataset_id": ticket.get("runtime", {}).get("dataset_id") == DATASET_ID,
        "raw_parameters": contract.get("params_semantics") == "raw"
        and ticket.get("params_semantics") == "raw"
        and checkpoint.get("params_semantics") == "raw",
        "step": ticket.get("checkpoint_step") == step
        and checkpoint.get("checkpoint_step") == step,
        "project_commit": ticket.get("source", {}).get("project_commit")
        == PROJECT_COMMIT,
        "openpi_commit": ticket.get("source", {}).get("openpi_commit")
        == OPENPI_COMMIT,
        "model_sha256": ticket.get("source", {}).get("checkpoint_sha256")
        == model_digest
        and checkpoint.get("model_sha256") == model_digest,
        "state": contract.get("observation", {}).get("state")
        == "float32[14] absolute joints/grippers",
        "action_shape": contract.get("action", {}).get("shape") == [10, 14],
        "action_representation": contract.get("action", {}).get("representation")
        == "absolute joint/gripper targets after output transform",
        "prompts": tuple(contract.get("canonical_prompts", ())) == PROMPTS,
        "robot_dimensions": robot.get("state_dim") == 14
        and robot.get("action_dim") == 14,
        "robot_units": robot.get("joint_unit") == "radian"
        and robot.get("gripper_unit") == "meter",
        "robot_action_contract": policy.get("action_representation")
        == "absolute_joint_position"
        and policy.get("control_hz") == 15
        and policy.get("action_chunk") == 10
        and policy.get("execute_chunk") == 8,
        "hardware_fail_closed": robot.get("calibration_complete") is False
        and safety.get("motion_enabled") is False,
        "restore_evidence": ticket.get("validation", {}).get("restore") == "passed",
        "inference_evidence": ticket.get("validation", {}).get("synthetic_inference")
        == "passed",
        "validation_report": validation.get("restore") is True
        and validation.get("synthetic_inference") is True
        and validation.get("action_shape") == [10, 14]
        and validation.get("all_finite") is True,
        "intake_family": intake.get("model", {}).get("family") == "pi05",
        "intake_robot": intake.get("model", {}).get("robot") == "dual_piper",
        "intake_task": intake.get("model", {}).get("task")
        == "desktop_cleanup_20260825_v1",
        "intake_step": intake.get("model", {}).get("step") == str(step),
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise ValueError("dual-PiPER identity mismatch: " + ", ".join(failed))
    return {
        "ok": True,
        "bundle": str(root),
        "package_id": package_id,
        "step": step,
        "object_count": len(intake_entries),
        "total_bytes": source["total_bytes"],
        "model_sha256": model_digest,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    args = parser.parse_args()
    print(json.dumps(verify(args.bundle), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
