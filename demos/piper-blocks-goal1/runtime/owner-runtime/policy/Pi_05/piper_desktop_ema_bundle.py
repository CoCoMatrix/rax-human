"""Identity and integrity checks for the dual-PiPER desktop EMA bundle."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


PACKAGE_ID = "dual_piper_desktop_cleanup_20260825_v1_pi05_ema_step15000_ckpt"
CONFIG_NAME = "pi05_dual_piper_real"
DATASET_ID = "dual_piper_desktop_cleanup_20260825_v1"
PROJECT_COMMIT = "f2989e88c4df0322e089753cb1272116230e05a9"
OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
STEP = 15_000
EMA_DECAY = 0.999
ACTION_HORIZON = 16
EXECUTE_HORIZON = 8
ACTION_FREQUENCY_HZ = 15.0
MODEL_SHA256 = "222a4a118df7da2f1ac2482aca408bf76f5e35326f287997bf973b022a14cf40"
NORM_SHA256 = "10e6d5d1185474f06a8f04abdcf0ab37a5aba64eff9e207292ff005dac22444c"
TOKENIZER_SHA256 = "8986bb4f423f07f8c7f70d0dbe3526fb2316056c17bae71b1ea975e77a168fc6"
OSS_URI = (
    "oss://ali-pai-oss-ppu/spy/deploy/"
    "dual_piper_desktop_cleanup_20260825_v1_pi05_ema_step15000_ckpt/"
)
OSS_ENDPOINT = "https://oss-cn-wulanchabu.aliyuncs.com"
OSS_OBJECT_COUNT = 760
OSS_TOTAL_BYTES = 7_245_857_741
LOCAL_INTAKE_FILES = frozenset({"manifest.json", "SHA256SUMS"})
PROMPTS = (
    "Close the laptop.",
    "Pick up one drink container and place it on the coaster.",
    "Pick up one mouse and place it on the mouse pad.",
    "Pick up one paper ball and place it in the trash bin.",
    "Pick up one pen and place it in the pen holder.",
)


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _valid_digest(value: object) -> bool:
    text = str(value)
    return len(text) == 64 and all(character in "0123456789abcdef" for character in text)


def _relative(value: object) -> str:
    text = str(value)
    path = Path(text)
    if not text or path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe manifest path: {value!r}")
    return text


def _package_entries(manifest: dict[str, Any]) -> dict[str, tuple[int, str]]:
    rows = manifest.get("files")
    if not isinstance(rows, list) or not rows:
        raise ValueError("package_manifest.json has no files")
    entries: dict[str, tuple[int, str]] = {}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("package manifest row must be an object")
        relative = _relative(row.get("path"))
        size = row.get("size")
        digest = str(row.get("sha256", ""))
        if (
            relative in entries
            or not isinstance(size, int)
            or size < 0
            or not _valid_digest(digest)
        ):
            raise ValueError(f"invalid package manifest row: {row!r}")
        entries[relative] = (size, digest)
    return entries


def _intake_entries(manifest: dict[str, Any]) -> dict[str, tuple[int, str]]:
    rows = manifest.get("objects")
    if not isinstance(rows, list) or not rows:
        raise ValueError("model-store intake manifest has no objects")
    entries: dict[str, tuple[int, str]] = {}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("model-store intake row must be an object")
        relative = _relative(row.get("path"))
        size = row.get("bytes")
        digest = str(row.get("sha256", ""))
        if (
            relative in entries
            or not isinstance(size, int)
            or size < 0
            or not _valid_digest(digest)
        ):
            raise ValueError(f"invalid model-store intake row: {row!r}")
        entries[relative] = (size, digest)
    return entries


def _checksums(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            digest, relative_value = line.split("  ", 1)
        except ValueError as exc:
            raise ValueError(f"invalid SHA256SUMS line {line_number}") from exc
        relative = _relative(relative_value)
        if relative in result or not _valid_digest(digest):
            raise ValueError(f"invalid SHA256SUMS line {line_number}")
        result[relative] = digest
    if not result:
        raise ValueError("SHA256SUMS is empty")
    return result


def verify_piper_desktop_ema_bundle(
    bundle_root: Path, *, full_manifest: bool = False
) -> dict[str, Any]:
    """Verify the exact EMA deployment identity and optionally every payload hash."""
    root = bundle_root.expanduser().resolve()
    required = {
        "deploy": root / "deploy_input.json",
        "contract": root / "contract.json",
        "checkpoint": root / "checkpoint/metadata.json",
        "weights": root / "checkpoint/model.safetensors",
        "norm": root / f"checkpoint/assets/{DATASET_ID}/norm_stats.json",
        "tokenizer": root / "assets/big_vision/paligemma_tokenizer.model",
        "validation": root / "validation/synthetic_inference.json",
        "template": root / "runtime/project/configs/real/dual_piper_gripper_template.json",
        "adapter": root / "runtime/project/src/desk_icl/training/openpi_dual_piper_policy.py",
        "openpi_config": root / "runtime/openpi/src/openpi/training/config.py",
        "package": root / "package_manifest.json",
        "intake": root / "manifest.json",
        "sums": root / "SHA256SUMS",
    }
    missing = sorted(name for name, path in required.items() if not path.is_file())
    if missing:
        raise FileNotFoundError("desktop EMA bundle is incomplete: " + ", ".join(missing))
    symlinks = sorted(name for name, path in required.items() if path.is_symlink())
    if symlinks:
        raise ValueError("desktop EMA identity files cannot be symlinks: " + ", ".join(symlinks))

    deploy = _json(required["deploy"])
    contract = _json(required["contract"])
    checkpoint = _json(required["checkpoint"])
    validation = _json(required["validation"])
    template = _json(required["template"])
    package = _json(required["package"])
    intake = _json(required["intake"])
    package_entries = _package_entries(package)
    intake_entries = _intake_entries(intake)
    sums = _checksums(required["sums"])
    robot = template.get("robot", {})
    cameras = template.get("cameras", {})
    policy = template.get("policy", {})
    safety = template.get("safety", {})

    checks = {
        "package_id": contract.get("package_id") == PACKAGE_ID
        and deploy.get("package_id") == PACKAGE_ID
        and package.get("package_id") == PACKAGE_ID,
        "model": contract.get("model") == "pi0.5",
        "config": contract.get("train_config") == CONFIG_NAME
        and deploy.get("runtime", {}).get("train_config") == CONFIG_NAME,
        "dataset": deploy.get("runtime", {}).get("dataset_id") == DATASET_ID,
        "step": contract.get("step") == STEP
        and deploy.get("checkpoint_step") == STEP
        and checkpoint.get("checkpoint_step") == STEP,
        "ema": contract.get("params_semantics") == "ema"
        and deploy.get("params_semantics") == "ema"
        and checkpoint.get("params_semantics") == "ema"
        and checkpoint.get("ema_configured") == EMA_DECAY
        and checkpoint.get("ema_exported") is True,
        "checkpoint_role": deploy.get("checkpoint_role") == "intermediate_evaluation_candidate"
        and checkpoint.get("checkpoint_role") == "intermediate_evaluation_candidate",
        "source_format": deploy.get("source", {}).get("training_checkpoint_format") == "orbax_jax"
        and checkpoint.get("source_checkpoint_format") == "orbax_jax",
        "commits": deploy.get("source", {}).get("project_commit") == PROJECT_COMMIT
        and deploy.get("source", {}).get("openpi_commit") == OPENPI_COMMIT,
        "source_hashes": deploy.get("source", {}).get("checkpoint_sha256") == MODEL_SHA256
        and checkpoint.get("model_sha256") == MODEL_SHA256
        and deploy.get("source", {}).get("norm_stats_sha256") == NORM_SHA256
        and deploy.get("source", {}).get("tokenizer_sha256") == TOKENIZER_SHA256,
        "observation": contract.get("observation", {}).get("head_rgb") == "uint8[H,W,3] RGB"
        and contract.get("observation", {}).get("left_wrist_rgb") == "uint8[H,W,3] RGB"
        and contract.get("observation", {}).get("right_wrist_rgb") == "uint8[H,W,3] RGB"
        and contract.get("observation", {}).get("state")
        == "float32[14] absolute joints/grippers",
        "action": contract.get("action", {}).get("shape") == [ACTION_HORIZON, 14]
        and contract.get("action", {}).get("representation")
        == "absolute joint/gripper targets after output transform",
        "prompts": tuple(contract.get("canonical_prompts", ())) == PROMPTS,
        "robot": robot.get("state_dim") == 14
        and robot.get("action_dim") == 14
        and robot.get("joint_unit") == "radian"
        and robot.get("gripper_unit") == "meter",
        "cameras": set(cameras) == {"head", "left_wrist", "right_wrist"}
        and all(
            cameras[name].get("width") == 1920
            and cameras[name].get("height") == 1080
            and cameras[name].get("fps") == 15
            for name in cameras
        ),
        "execution": policy.get("action_representation") == "absolute_joint_position"
        and policy.get("control_hz") == ACTION_FREQUENCY_HZ
        and policy.get("action_chunk") == ACTION_HORIZON
        and policy.get("execute_chunk") == EXECUTE_HORIZON,
        "fail_closed_template": robot.get("calibration_complete") is False
        and safety.get("motion_enabled") is False
        and safety.get("operator_estop_required") is True,
        "validation": deploy.get("validation", {}).get("restore") == "passed"
        and deploy.get("validation", {}).get("synthetic_inference") == "passed"
        and deploy.get("validation", {}).get("action_shape") == [ACTION_HORIZON, 14]
        and deploy.get("validation", {}).get("all_finite") is True
        and validation.get("restore") is True
        and validation.get("synthetic_inference") is True
        and validation.get("action_shape") == [ACTION_HORIZON, 14]
        and validation.get("all_finite") is True,
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise ValueError("desktop EMA bundle identity mismatch: " + ", ".join(failed))

    if package.get("schema") != "coco-bundle-manifest/1":
        raise ValueError("unsupported desktop EMA package manifest schema")
    if package.get("file_count") != len(package_entries):
        raise ValueError("desktop EMA package manifest file_count mismatch")
    if package.get("total_bytes") != sum(size for size, _ in package_entries.values()):
        raise ValueError("desktop EMA package manifest total_bytes mismatch")
    publisher_files = set(package_entries) | {"package_manifest.json"}
    source = intake.get("source", {})
    model = intake.get("model", {})
    if (
        intake.get("schema_version") != 1
        or source.get("uri") != OSS_URI
        or source.get("endpoint") != OSS_ENDPOINT
        or source.get("object_count") != OSS_OBJECT_COUNT
        or source.get("total_bytes") != OSS_TOTAL_BYTES
        or model.get("family") != "pi05"
        or model.get("robot") != "dual_piper"
        or model.get("task") != "desktop_cleanup_20260825_v1"
        or model.get("step") != "15000"
        or model.get("params_semantics") != "ema"
        or model.get("train_config") != CONFIG_NAME
        or set(intake_entries) != publisher_files
        or set(sums) != publisher_files
    ):
        raise ValueError("desktop EMA model-store intake identity mismatch")
    for relative, expected in package_entries.items():
        if intake_entries.get(relative) != expected or sums.get(relative) != expected[1]:
            raise ValueError(f"desktop EMA intake/package mismatch: {relative}")
    package_digest = _sha256(required["package"])
    if intake_entries.get("package_manifest.json") != (
        required["package"].stat().st_size,
        package_digest,
    ) or sums.get("package_manifest.json") != package_digest:
        raise ValueError("desktop EMA package manifest intake checksum mismatch")

    critical_hashes = {
        "checkpoint/model.safetensors": MODEL_SHA256,
        f"checkpoint/assets/{DATASET_ID}/norm_stats.json": NORM_SHA256,
        "assets/big_vision/paligemma_tokenizer.model": TOKENIZER_SHA256,
    }
    for relative, digest in critical_hashes.items():
        path = root / relative
        if package_entries.get(relative, (None, None))[1] != digest:
            raise ValueError(f"desktop EMA critical asset hash mismatch: {relative}")
        if not full_manifest and _sha256(path) != digest:
            raise ValueError(f"desktop EMA critical asset hash mismatch: {relative}")

    if full_manifest:
        bundle_symlinks = sorted(
            path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_symlink()
        )
        if bundle_symlinks:
            raise ValueError(
                "desktop EMA bundle cannot contain symlinks: "
                + ", ".join(bundle_symlinks)
            )
        actual = {
            path.relative_to(root).as_posix()
            for path in root.rglob("*")
            if path.is_file() and path.relative_to(root).as_posix() not in LOCAL_INTAKE_FILES
        }
        if actual != publisher_files:
            raise ValueError(
                "desktop EMA bundle inventory mismatch: "
                f"missing={sorted(publisher_files - actual)}, "
                f"extra={sorted(actual - publisher_files)}"
            )
        for relative, (size, digest) in intake_entries.items():
            path = root / relative
            if path.stat().st_size != size or _sha256(path) != digest:
                raise ValueError(f"desktop EMA package hash mismatch: {relative}")

    return {
        "ok": True,
        "bundle": str(root),
        "package_id": PACKAGE_ID,
        "step": STEP,
        "params_semantics": "ema",
        "ema_decay": EMA_DECAY,
        "train_config": CONFIG_NAME,
        "action_shape": [ACTION_HORIZON, 14],
        "execute_horizon": EXECUTE_HORIZON,
        "prompts": list(PROMPTS),
        "package_file_count": len(package_entries),
        "package_total_bytes": package["total_bytes"],
        "full_manifest_verified": full_manifest,
        "deploy": deploy,
        "contract": contract,
    }
