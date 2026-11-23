"""Identity and integrity checks for the 2026-08-25 dual-PiPER EMA bundle."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


PACKAGE_ID = "piper_multitask5_20260825_pi05_ema_step20000_ckpt"
CONTRACT_ID = "dual_piper_joint_position_multitask_h16_v1"
CONFIG_NAME = "pi05_piper_dual_arm_multitask_20260825"
SOURCE_COMMIT = "bd48ebf4a2f848976fa216d1245c5c796cc22827"
STEP = 20_000
ACTION_HORIZON = 16
EXECUTE_HORIZON = 4
ACTION_FREQUENCY_HZ = 15.0
OSS_URI = (
    "oss://ali-pai-oss-ppu/weikang/deploy/piper_dual_arm/pi05_multitask/"
    "20260825_v1/piper_multitask5_20260825_pi05_ema_step20000_ckpt/"
)
OSS_ENDPOINT = "https://oss-cn-wulanchabu.aliyuncs.com"
OSS_OBJECT_COUNT = 40
OSS_TOTAL_BYTES = 12_448_636_696
PROMPTS = (
    "Pick up the paper ball and place it in the bin.",
    "Place the cup on the coaster.",
    "Place the pen in the pen holder.",
    "Close the laptop.",
    "Place the mouse on the mouse pad.",
)
ACTION_NAMES = (
    *(f"left_j{index}" for index in range(1, 7)),
    "left_gripper_width",
    *(f"right_j{index}" for index in range(1, 7)),
    "right_gripper_width",
)
DELTA_ACTION_MASK = (
    True,
    True,
    True,
    True,
    True,
    True,
    False,
    True,
    True,
    True,
    True,
    True,
    True,
    False,
)
IMAGE_INPUTS = {
    "observation/head_image": "head_camera_rgb",
    "observation/left_wrist_image": "left_wrist_camera_rgb",
    "observation/right_wrist_image": "right_wrist_camera_rgb",
}
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
            candidate.is_absolute()
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


def verify_piper_multitask_bundle(
    bundle_root: Path, *, full_manifest: bool = False
) -> dict[str, Any]:
    root = bundle_root.expanduser().resolve()
    required = {
        "deploy": root / "deploy_input.json",
        "contract": root / "contract.json",
        "catalog": root / "task_catalog.json",
        "checkpoint": root / "checkpoint/metadata.json",
        "params": root / "checkpoint/params/manifest.ocdbt",
        "norm": root
        / "checkpoint/assets/piper/dual_arm_multitask_20260825_train/norm_stats.json",
        "norm_provenance": root
        / "checkpoint/assets/piper/dual_arm_multitask_20260825_train/norm_provenance.json",
        "tokenizer": root / "tokenizer/paligemma_tokenizer.model",
        "source": root / "runtime/source.tar",
        "source_commit": root / "runtime/SOURCE_COMMIT",
        "runtime_manifest": root / "runtime/ppu-runtime-manifest.json",
        "validation": root / "validation/synthetic_inference.json",
        "package_manifest": root / "package_manifest.json",
        "intake_manifest": root / "manifest.json",
        "intake_sums": root / "SHA256SUMS",
    }
    missing = sorted(name for name, path in required.items() if not path.is_file())
    if missing:
        raise FileNotFoundError(
            "Piper multitask bundle is incomplete: " + ", ".join(missing)
        )
    symlinks = sorted(name for name, path in required.items() if path.is_symlink())
    if symlinks:
        raise ValueError("Piper bundle identity files cannot be symlinks: " + ", ".join(symlinks))

    deploy = _json(required["deploy"])
    contract = _json(required["contract"])
    catalog = _json(required["catalog"])
    checkpoint = _json(required["checkpoint"])
    validation = _json(required["validation"])
    package = _json(required["package_manifest"])
    intake_manifest = _json(required["intake_manifest"])
    intake_entries = _intake_entries(intake_manifest)
    intake_sums = _intake_checksums(required["intake_sums"])
    inputs = contract.get("inputs", {})
    state_input = inputs.get("observation/state", {})
    action_contract = contract.get("action", {})
    action_output = contract.get("outputs", {}).get("actions", {})
    image_contract_ok = set(IMAGE_INPUTS).issubset(inputs) and all(
        inputs[name].get("semantic") == semantic
        and inputs[name].get("dtype") == "uint8"
        and inputs[name].get("channel_order") == "RGB"
        and inputs[name].get("training_native_shape") == [1080, 1920, 3]
        and inputs[name].get("model_preprocessed_shape") == [224, 224, 3]
        for name, semantic in IMAGE_INPUTS.items()
    )
    prompts = tuple(
        row.get("task")
        for row in catalog.get("tasks", ())
        if isinstance(row, dict)
    )
    checks = {
        "deploy_schema": deploy.get("schema_version")
        == "openpi_piper_deploy_input_v1",
        "model_variant": deploy.get("model") == "pi05"
        and deploy.get("model_variant") == "dual_piper_multitask_h16",
        "package_id": deploy.get("package_id") == PACKAGE_ID
        and package.get("package_id") == PACKAGE_ID,
        "contract_id": deploy.get("contract_id") == CONTRACT_ID
        and contract.get("contract_id") == CONTRACT_ID,
        "config": deploy.get("config_name") == CONFIG_NAME
        and checkpoint.get("config_name") == CONFIG_NAME,
        "source": deploy.get("source_commit") == SOURCE_COMMIT
        and checkpoint.get("source_commit") == SOURCE_COMMIT
        and required["source_commit"].read_text(encoding="utf-8").strip()
        == SOURCE_COMMIT,
        "source_verified": deploy.get("source_commit_verified") is True
        and checkpoint.get("source_commit_verified") is True,
        "step": deploy.get("step") == STEP and checkpoint.get("step") == STEP,
        "step_semantics": deploy.get("checkpoint_source", {}).get("step_semantics")
        == "completed_optimizer_updates_v1"
        and checkpoint.get("step_semantics") == "completed_optimizer_updates_v1",
        "ema": deploy.get("params_semantics") == "ema"
        and checkpoint.get("params_semantics") == "ema"
        and deploy.get("ema_decay") == 0.99
        and checkpoint.get("ema_decay") == 0.99,
        "release_gate": deploy.get("release_role") == "intermediate_candidate"
        and deploy.get("target_num_train_steps") == 80_000
        and deploy.get("validation")
        == {
            "L0_manifest": True,
            "L1_restore": True,
            "L2_synthetic_inference": True,
            "L3_service_or_validation_data": False,
            "L4_live_robot": False,
        },
        "robot": contract.get("robot") == "dual_piper_gripper",
        "contract_schema": contract.get("schema")
        == "openpi_piper_policy_contract_v1",
        "images": image_contract_ok,
        "state": state_input.get("shape") == [14]
        and state_input.get("dtype") == "float32"
        and tuple(state_input.get("names", ())) == ACTION_NAMES
        and state_input.get("joint_unit") == "radian"
        and state_input.get("gripper_unit") == "meter"
        and state_input.get("representation")
        == "absolute_joint_position_and_gripper_width",
        "action": action_output.get("shape") == [ACTION_HORIZON, 14]
        and action_output.get("dtype") == "float64"
        and tuple(action_output.get("names", ())) == ACTION_NAMES,
        "action_semantics": action_output.get("representation")
        == "absolute_robot_targets_after_frozen_inverse_transform",
        "action_training_contract": action_contract.get("horizon")
        == ACTION_HORIZON
        and action_contract.get("physical_dim") == 14
        and tuple(action_contract.get("names", ())) == ACTION_NAMES
        and tuple(action_contract.get("delta_action_mask", ()))
        == DELTA_ACTION_MASK,
        "cadence": action_contract.get("control_frequency_hz")
        == ACTION_FREQUENCY_HZ
        and contract.get("execution", {}).get("control_frequency_hz")
        == ACTION_FREQUENCY_HZ,
        "execute_horizon": contract.get("execution", {}).get(
            "recommended_execute_horizon"
        )
        == EXECUTE_HORIZON,
        "catalog_schema": catalog.get("schema")
        == "piper_multitask_prompt_catalog_v1"
        and catalog.get("exact_match_required") is True,
        "prompts": prompts == PROMPTS,
        "prompt_contract": tuple(inputs.get("prompt", {}).get("allowed_values", ()))
        == PROMPTS
        and inputs.get("prompt", {}).get("exact_match_required") is True,
        "validation": validation.get("restored_from_bundle_only") is True
        and validation.get("params_semantics") == "ema"
        and validation.get("evidence_level") == "L2"
        and validation.get("production_default_denoise_steps") == 10
        and validation.get("synthetic_denoise_steps") == 1
        and validation.get("actions", {}).get("shape") == [ACTION_HORIZON, 14]
        and validation.get("actions", {}).get("finite") is True,
        "contract_sha": deploy.get("contract_sha256")
        == _sha256(required["contract"]),
        "catalog_sha": deploy.get("task_catalog_sha256")
        == _sha256(required["catalog"]),
        "norm_sha": deploy.get("norm_sha256") == _sha256(required["norm"]),
        "norm_provenance_sha": deploy.get("norm_provenance_sha256")
        == _sha256(required["norm_provenance"]),
        "tokenizer_sha": deploy.get("tokenizer_sha256")
        == _sha256(required["tokenizer"]),
        "source_archive_sha": deploy.get("runtime_source_archive_sha256")
        == _sha256(required["source"]),
        "runtime_manifest_sha": deploy.get("runtime_manifest_sha256")
        == _sha256(required["runtime_manifest"]),
        "validation_sha": deploy.get("synthetic_inference_sha256")
        == _sha256(required["validation"]),
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise ValueError("Piper multitask bundle identity mismatch: " + ", ".join(failed))

    entries = _manifest_entries(package)
    if package.get("schema") != "coco-bundle-manifest/1":
        raise ValueError("unsupported Piper package manifest schema")
    if package.get("file_count") != len(entries):
        raise ValueError("Piper package manifest file_count mismatch")
    if package.get("total_bytes") != sum(size for size, _ in entries.values()):
        raise ValueError("Piper package manifest total_bytes mismatch")
    publisher_files = set(entries) | {"package_manifest.json"}
    intake_source = intake_manifest.get("source", {})
    intake_model = intake_manifest.get("model", {})
    if (
        intake_manifest.get("schema_version") != 1
        or intake_source.get("uri") != OSS_URI
        or intake_source.get("endpoint") != OSS_ENDPOINT
        or intake_source.get("object_count") != OSS_OBJECT_COUNT
        or intake_source.get("total_bytes") != OSS_TOTAL_BYTES
        or intake_model.get("family") != "pi05"
        or intake_model.get("robot") != "dual_piper"
        or intake_model.get("task") != "piper_multitask5_20260825_v1"
        or intake_model.get("step") != "20000"
        or intake_model.get("params_semantics") != "ema"
        or set(intake_entries) != publisher_files
        or set(intake_sums) != publisher_files
    ):
        raise ValueError("Piper model-store intake identity mismatch")
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
                "Piper bundle cannot contain symlinks: " + ", ".join(bundle_symlinks)
            )
        actual = {
            path.relative_to(root).as_posix()
            for path in root.rglob("*")
            if path.is_file() and path.relative_to(root).as_posix() not in LOCAL_INTAKE_FILES
        }
        expected = publisher_files
        if actual != expected:
            raise ValueError(
                "Piper bundle inventory mismatch: "
                f"missing={sorted(expected - actual)}, extra={sorted(actual - expected)}"
            )
        for relative, (size, digest) in intake_entries.items():
            path = root / relative
            if path.stat().st_size != size or _sha256(path) != digest:
                raise ValueError(f"Piper package hash mismatch: {relative}")

    return {
        "ok": True,
        "bundle": str(root),
        "package_id": PACKAGE_ID,
        "step": STEP,
        "params_semantics": "ema",
        "source_commit": SOURCE_COMMIT,
        "contract_id": CONTRACT_ID,
        "action_shape": [ACTION_HORIZON, 14],
        "execute_horizon": EXECUTE_HORIZON,
        "prompts": list(PROMPTS),
        "package_file_count": len(entries),
        "package_total_bytes": package["total_bytes"],
        "full_manifest_verified": full_manifest,
        "deploy": deploy,
        "contract": contract,
    }
