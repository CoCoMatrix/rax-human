"""Identity and integrity checks for the Piper build-blocks EMA bundle."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


PACKAGE_ID = "piper_build_blocks_jiaxin_v1_pi05_ema_step12000_ckpt"
CONTRACT_ID = "piper_dual_arm_tricam_joint14_delta12_absgripper2_v1"
CONFIG_NAME = "pi05_piper_build_blocks_delta_joint_v1"
DATASET_ID = "piper_build_blocks_jiaxin_v1"
SOURCE_COMMIT = "abcb0a93d010aaf4c4c4b02fc772b741b23ec2e5"
STEP = 12_000
ACTION_HORIZON = 15
EXECUTE_HORIZON = 15
ACTION_FREQUENCY_HZ = 15.0
IMAGE_SIZE = (224, 224)
PROMPT = (
    "Stack the second cube on top of the first cube, then place the cone on top "
    "of the second cube."
)
OSS_URI = (
    "oss://oss-pai-r9x4/PeterX/deploy/"
    "piper_build_blocks_jiaxin_v1_pi05_ema_step12000_ckpt/"
)
OSS_ENDPOINT = "https://oss-cn-beijing.aliyuncs.com"
OSS_OBJECT_COUNT = 272
OSS_TOTAL_BYTES = 12_447_734_009
CONTRACT_SHA256 = "1ed6ca0b395a03a20ee98a317e3a0488fcc868cb4b2293f23af74e3ada0faa8a"
NORM_SHA256 = "2fb9d815b61d3e911a977178cf03a30d70264b72f7a4d1bbc2764088a4a4fa88"
TOKENIZER_SHA256 = "8986bb4f423f07f8c7f70d0dbe3526fb2316056c17bae71b1ea975e77a168fc6"
TRAINING_MANIFEST_SHA256 = (
    "750a9844967ea65e24a508d671ebbe590dc2ac2a58aa6805d9591346b942c4bf"
)
CHECKPOINT_SOURCE = (
    "oss://oss-pai-r9x4/PeterX/outputs/"
    "piper-build-blocks-pi05-delta-joint-20k-v1/checkpoints/"
    "pi05_piper_build_blocks_delta_joint_v1/"
    "piper-pi05-dj-tricam-b512-20k-0825-r3/12000"
)
TRAINING_MANIFEST_URI = (
    "oss://oss-pai-r9x4/PeterX/outputs/piper-build-blocks-data-20260824/"
    "prepared/train_manifest.json"
)
LOCAL_INTAKE_FILES = frozenset({"manifest.json", "SHA256SUMS"})


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


def _manifest_entries(manifest: dict[str, Any]) -> dict[str, tuple[int, str]]:
    rows = manifest.get("files")
    if not isinstance(rows, list) or not rows:
        raise ValueError("package_manifest.json has no files")
    result: dict[str, tuple[int, str]] = {}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("package manifest row must be an object")
        relative = str(row.get("path", ""))
        path = Path(relative)
        size = row.get("size")
        digest = str(row.get("sha256", ""))
        if (
            not relative
            or path.is_absolute()
            or ".." in path.parts
            or relative in result
            or not isinstance(size, int)
            or size < 0
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise ValueError(f"invalid package manifest row: {row!r}")
        result[relative] = (size, digest)
    return result


def _intake_entries(manifest: dict[str, Any]) -> dict[str, tuple[int, str]]:
    rows = manifest.get("objects")
    if not isinstance(rows, list) or not rows:
        raise ValueError("model-store intake manifest has no objects")
    result: dict[str, tuple[int, str]] = {}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("model-store intake row must be an object")
        relative = str(row.get("path", ""))
        path = Path(relative)
        size = row.get("bytes")
        digest = str(row.get("sha256", ""))
        if (
            not relative
            or path.is_absolute()
            or ".." in path.parts
            or relative in result
            or not isinstance(size, int)
            or size < 0
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise ValueError(f"invalid model-store intake row: {row!r}")
        result[relative] = (size, digest)
    return result


def _intake_checksums(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            digest, relative = line.split("  ", 1)
        except ValueError as exc:
            raise ValueError(f"invalid SHA256SUMS line {line_number}") from exc
        candidate = Path(relative)
        if (
            not relative
            or candidate.is_absolute()
            or ".." in candidate.parts
            or relative in result
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise ValueError(f"invalid SHA256SUMS line {line_number}")
        result[relative] = digest
    if not result:
        raise ValueError("SHA256SUMS is empty")
    return result


def verify_piper_build_blocks_ema_bundle(
    bundle_root: Path, *, full_manifest: bool = False
) -> dict[str, Any]:
    root = bundle_root.expanduser().resolve()
    required = {
        "deploy": root / "deploy_input.json",
        "contract": root / "contract.json",
        "checkpoint": root / "checkpoint/metadata.json",
        "params": root / "checkpoint/params/manifest.ocdbt",
        "norm": root
        / "checkpoint/assets/piper_build_blocks_jiaxin_v1/norm_stats.json",
        "tokenizer": root / "runtime/assets/big_vision/paligemma_tokenizer.model",
        "identity_manifest": root / "runtime/piper_manifest_identity.json",
        "source_commit": root / "runtime/SOURCE_COMMIT",
        "source_config": root / "runtime/source/src/openpi/training/config.py",
        "source_tasks": root
        / "runtime/source/src/openpi/policies/piper_build_blocks_tasks.py",
        "source_lock": root / "runtime/source/uv.lock",
        "runtime_lock": root / "runtime/uv.lock",
        "package_manifest": root / "package_manifest.json",
        "intake_manifest": root / "manifest.json",
        "intake_sums": root / "SHA256SUMS",
    }
    missing = sorted(name for name, path in required.items() if not path.is_file())
    if missing:
        raise FileNotFoundError(
            "Piper build-blocks EMA bundle is incomplete: " + ", ".join(missing)
        )
    symlinks = sorted(name for name, path in required.items() if path.is_symlink())
    if symlinks:
        raise ValueError(
            "Piper build-blocks identity files cannot be symlinks: "
            + ", ".join(symlinks)
        )

    deploy = _json(required["deploy"])
    contract = _json(required["contract"])
    checkpoint = _json(required["checkpoint"])
    identity_manifest = _json(required["identity_manifest"])
    package = _json(required["package_manifest"])
    intake_manifest = _json(required["intake_manifest"])
    intake_entries = _intake_entries(intake_manifest)
    intake_sums = _intake_checksums(required["intake_sums"])
    inputs = contract.get("inputs", {})
    images = inputs.get("images", {})
    state = inputs.get("state", {})
    model_action = contract.get("model_action", {})
    output = contract.get("outputs", {}).get("actions", {})
    prompt = contract.get("prompt", {})
    validation = deploy.get("validation", {})
    checks = {
        "deploy_schema": deploy.get("schema_version") == "coco-deploy-input/1",
        "package_id": deploy.get("package_id") == PACKAGE_ID
        and package.get("package_id") == PACKAGE_ID,
        "contract_id": deploy.get("contract_id") == CONTRACT_ID
        and contract.get("contract_id") == CONTRACT_ID,
        "config": deploy.get("config_name") == CONFIG_NAME
        and checkpoint.get("config_name") == CONFIG_NAME,
        "dataset": deploy.get("data_version") == DATASET_ID,
        "task": deploy.get("task") == "piper_dual_arm_build_blocks",
        "delivery_role": deploy.get("delivery_role") == "comparison",
        "training_run": deploy.get("training_job_id") == "dlc19ppl3e3ec9ta"
        and deploy.get("training_snapshot")
        == "piper-pi05-dj-tricam-b512-20k-0825-r4-resume-2k",
        "checkpoint_source": deploy.get("checkpoint_source") == CHECKPOINT_SOURCE
        and checkpoint.get("source_checkpoint_uri") == CHECKPOINT_SOURCE,
        "source": deploy.get("source_commit") == SOURCE_COMMIT
        and checkpoint.get("source_commit") == SOURCE_COMMIT
        and required["source_commit"].read_text(encoding="utf-8").strip()
        == SOURCE_COMMIT,
        "source_verified": deploy.get("source_commit_verified") is True
        and checkpoint.get("source_commit_verified") is True,
        "step": deploy.get("step") == STEP
        and checkpoint.get("step") == STEP
        and contract.get("step") == STEP,
        "step_verified": deploy.get("step_verified") is True
        and checkpoint.get("step_verified_by_targeted_train_state_restore") is True,
        "ema": deploy.get("params_semantics") == "ema"
        and checkpoint.get("params_semantics") == "ema"
        and deploy.get("ema_decay") == 0.99
        and checkpoint.get("ema_decay") == 0.99,
        "contract_schema": contract.get("schema_version")
        == "coco-policy-contract/1",
        "images": set(images) == {"head", "left_wrist", "right_wrist"}
        and all(
            images[name]
            == {"dtype": "uint8", "required": True, "shape": [224, 224, 3]}
            for name in images
        ),
        "state": state.get("shape") == [14]
        and state.get("dtype") == "float32"
        and state.get("semantics")
        == "absolute joint12 plus absolute gripper-width2",
        "model_action": model_action.get("shape") == [15, 32]
        and model_action.get("semantics")
        == "delta for 12 joints, absolute for 2 grippers, padded to 32D"
        and model_action.get("delta_mask_segments") == [6, -1, 6, -1],
        "output": output.get("shape") == [ACTION_HORIZON, 14]
        and output.get("dtype") == "float32"
        and output.get("frequency_hz") == ACTION_FREQUENCY_HZ
        and output.get("joint_unit") == "radian"
        and output.get("gripper_width_unit") == "meter"
        and output.get("semantics")
        == (
            "absolute dual-arm target: left joint6, left gripper, right joint6, "
            "right gripper"
        ),
        "prompt": prompt.get("mode") == "fixed_exact"
        and prompt.get("value") == PROMPT,
        "normalization": contract.get("normalization")
        == {
            "asset_id": DATASET_ID,
            "sha256": NORM_SHA256,
            "use_quantiles": True,
        },
        "validation": validation
        == {
            "actions_dtype": "float32",
            "actions_finite": True,
            "actions_shape": [ACTION_HORIZON, 14],
            "fresh_process": True,
            "l0_manifest": "pass",
            "l1_restore": "pass",
            "l2_synthetic_inference": "pass",
            "l3": "not_run",
            "l4": "not_run",
        },
        "contract_sha": deploy.get("contract_sha256") == CONTRACT_SHA256
        and _sha256(required["contract"]) == CONTRACT_SHA256,
        "norm_sha": deploy.get("norm_sha256") == NORM_SHA256
        and _sha256(required["norm"]) == NORM_SHA256,
        "tokenizer_sha": deploy.get("tokenizer_sha256") == TOKENIZER_SHA256
        and _sha256(required["tokenizer"]) == TOKENIZER_SHA256,
        "training_manifest": deploy.get("training_manifest_sha256")
        == TRAINING_MANIFEST_SHA256
        and identity_manifest.get("training_manifest_sha256")
        == TRAINING_MANIFEST_SHA256
        and identity_manifest.get("schema_version")
        == "piper-deploy-manifest-identity/1"
        and identity_manifest.get("training_manifest_uri")
        == TRAINING_MANIFEST_URI,
        "runtime_lock": _sha256(required["source_lock"])
        == _sha256(required["runtime_lock"]),
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise ValueError(
            "Piper build-blocks EMA bundle identity mismatch: " + ", ".join(failed)
        )

    entries = _manifest_entries(package)
    if (
        package.get("schema") != "coco-bundle-manifest/1"
        or package.get("file_count") != len(entries)
        or package.get("total_bytes") != sum(size for size, _ in entries.values())
    ):
        raise ValueError("Piper build-blocks package manifest summary mismatch")
    publisher_files = set(entries) | {"package_manifest.json"}
    intake_source = intake_manifest.get("source", {})
    intake_model = intake_manifest.get("model", {})
    if (
        intake_manifest.get("schema_version") != 1
        or intake_source.get("uri") != OSS_URI
        or intake_source.get("endpoint") != OSS_ENDPOINT
        or intake_source.get("object_count") != OSS_OBJECT_COUNT
        or intake_source.get("total_bytes") != OSS_TOTAL_BYTES
        or intake_model
        != {
            "family": "pi05",
            "package_id": PACKAGE_ID,
            "params_semantics": "ema",
            "robot": "dual_piper",
            "step": str(STEP),
            "task": DATASET_ID,
        }
        or set(intake_entries) != publisher_files
        or set(intake_sums) != publisher_files
    ):
        raise ValueError("Piper build-blocks model-store intake identity mismatch")
    for relative, (size, digest) in entries.items():
        if intake_entries.get(relative) != (size, digest):
            raise ValueError(f"Piper intake/package manifest mismatch: {relative}")
        if intake_sums.get(relative) != digest:
            raise ValueError(f"Piper intake checksum mismatch: {relative}")
    package_size = required["package_manifest"].stat().st_size
    package_digest = _sha256(required["package_manifest"])
    if intake_entries.get("package_manifest.json") != (
        package_size,
        package_digest,
    ) or intake_sums.get("package_manifest.json") != package_digest:
        raise ValueError("Piper package manifest intake checksum mismatch")

    if full_manifest:
        bundle_symlinks = sorted(
            path.relative_to(root).as_posix()
            for path in root.rglob("*")
            if path.is_symlink()
        )
        if bundle_symlinks:
            raise ValueError(
                "Piper build-blocks bundle cannot contain symlinks: "
                + ", ".join(bundle_symlinks)
            )
        actual = {
            path.relative_to(root).as_posix()
            for path in root.rglob("*")
            if path.is_file()
            and path.relative_to(root).as_posix() not in LOCAL_INTAKE_FILES
        }
        if actual != publisher_files:
            raise ValueError(
                "Piper build-blocks bundle inventory mismatch: "
                f"missing={sorted(publisher_files - actual)}, "
                f"extra={sorted(actual - publisher_files)}"
            )
        for relative, (size, digest) in intake_entries.items():
            path = root / relative
            if path.stat().st_size != size or _sha256(path) != digest:
                raise ValueError(f"Piper build-blocks package hash mismatch: {relative}")

    return {
        "ok": True,
        "bundle": str(root),
        "package_id": PACKAGE_ID,
        "step": STEP,
        "params_semantics": "ema",
        "source_commit": SOURCE_COMMIT,
        "contract_id": CONTRACT_ID,
        "action_shape": [ACTION_HORIZON, 14],
        "action_frequency_hz": ACTION_FREQUENCY_HZ,
        "execute_horizon": EXECUTE_HORIZON,
        "prompt": PROMPT,
        "package_file_count": len(entries),
        "package_total_bytes": package["total_bytes"],
        "full_manifest_verified": full_manifest,
    }
