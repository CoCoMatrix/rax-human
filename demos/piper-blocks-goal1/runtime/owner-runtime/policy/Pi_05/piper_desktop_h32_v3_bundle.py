"""Identity and integrity checks for registered dual-PiPER h32 EMA bundles."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class H32BundleIdentity:
    package_id: str
    dataset_id: str
    data_version: str
    norm_sha256: str
    prompts: tuple[str, ...]
    intake_task: str
    step: int
    checkpoint_role: str
    model_sha256: str
    package_manifest_sha256: str
    oss_uri: str
    oss_total_bytes: int


# Compatibility name retained for existing h32-v3 consumers.
H32V3BundleIdentity = H32BundleIdentity


DESKTOP_PROMPTS = (
    "Close the laptop.",
    "Pick up one drink container and place it on the coaster.",
    "Pick up one mouse and place it on the mouse pad.",
    "Pick up one paper ball and place it in the trash bin.",
    "Pick up one pen and place it in the pen holder.",
)
PAPERALL50_PROMPTS = (
    "Close the laptop.",
    "Pick up one drink container and place it on the coaster.",
    "Pick up one mouse and place it on the mouse pad.",
    "Pick up all paper balls and place them in the trash bin.",
    "Pick up one pen and place it in the pen holder.",
)
OBJECT50_PROMPTS = PAPERALL50_PROMPTS
PEN_TRASH_PROMPTS = (
    "Close the laptop.",
    "Pick up one drink container and place it on the coaster.",
    "Pick up one mouse and place it on the mouse pad.",
    "Pick up one pen and place it in the trash bin.",
    "Pick up one pen and place it in the pen holder.",
)


EMA_STEP10000 = H32BundleIdentity(
    package_id=(
        "dual_piper_desktop_cleanup_20260827_h32_v3_"
        "pi05_ema_step10000_ckpt"
    ),
    dataset_id="dual_piper_desktop_cleanup_20260827_h32_v3",
    data_version="20260827_h32_v3",
    norm_sha256=(
        "291e81cba57b48de54245e009548a0c0a43e5a793f591a6115af683dbb28dcb9"
    ),
    prompts=DESKTOP_PROMPTS,
    intake_task="desktop_cleanup_20260827_h32_v3",
    step=10_000,
    checkpoint_role="intermediate_evaluation_candidate",
    model_sha256=(
        "ebdfc6ce02e624fedf4c1545c5a34a0a03d5d9b4c02555d1c9703a360545d4ca"
    ),
    package_manifest_sha256=(
        "ef42264febd3e20860f11ebfa2640af33aaa88f33d2a9fb58c93df6b4912b915"
    ),
    oss_uri=(
        "oss://ali-pai-oss-ppu/spy/deploy/"
        "dual_piper_desktop_cleanup_20260827_h32_v3_"
        "pi05_ema_step10000_ckpt/"
    ),
    oss_total_bytes=7_245_883_104,
)
EMA_STEP19999 = H32BundleIdentity(
    package_id=(
        "dual_piper_desktop_cleanup_20260827_h32_v3_"
        "pi05_ema_step19999_ckpt"
    ),
    dataset_id="dual_piper_desktop_cleanup_20260827_h32_v3",
    data_version="20260827_h32_v3",
    norm_sha256=(
        "291e81cba57b48de54245e009548a0c0a43e5a793f591a6115af683dbb28dcb9"
    ),
    prompts=DESKTOP_PROMPTS,
    intake_task="desktop_cleanup_20260827_h32_v3",
    step=19_999,
    checkpoint_role="final",
    model_sha256=(
        "8e71011699cd13a3e6d968beb8cb659246d0b640e9f62231004f200d0139cfc0"
    ),
    package_manifest_sha256=(
        "c9d93ab46cb32ddd1d32ec13faceef16633463a161fb7a9e9437326bbed10b99"
    ),
    oss_uri=(
        "oss://ali-pai-oss-ppu/spy/deploy/"
        "dual_piper_desktop_cleanup_20260827_h32_v3_"
        "pi05_ema_step19999_ckpt/"
    ),
    oss_total_bytes=7_245_883_019,
)
PAPERALL50_EMA_STEP4999 = H32BundleIdentity(
    package_id=(
        "dual_piper_paperall50_20260828_h32_v1_pi05_ema_step4999_ckpt"
    ),
    dataset_id="dual_piper_paperall50_from10k_h32_v1",
    data_version="20260828_paperall50_h32_v1",
    norm_sha256=(
        "8c35024e8645802b2e34b0eabf9c7efba508b66bef93f94bf2526712f114ce8a"
    ),
    prompts=PAPERALL50_PROMPTS,
    intake_task="paperall50_20260828_h32_v1",
    step=4_999,
    checkpoint_role="final",
    model_sha256=(
        "afd6027073ff457099a9d6c0ffbf65bfbd717a006cbb18ee6844cb4639279cc1"
    ),
    package_manifest_sha256=(
        "7185c012aae2375d683c465906ba712b855b4ca32cf55505f143047a22216450"
    ),
    oss_uri=(
        "oss://ali-pai-oss-ppu/spy/deploy/"
        "dual_piper_paperall50_20260828_h32_v1_pi05_ema_step4999_ckpt/"
    ),
    oss_total_bytes=7_245_887_919,
)
DAGGER50_EMA_STEP4999 = H32BundleIdentity(
    package_id=(
        "dual_piper_dagger50_20260829_h32_v1_pi05_ema_step4999_ckpt"
    ),
    dataset_id="dual_piper_dagger50_from_paperall4999_h32_v1",
    data_version="20260829_dagger50_h32_v1",
    norm_sha256=(
        "c074f1a8ad1a8783b4f3e5179db3dbd5a6e4fcd2da1a4f63810cd2880f3cfd3e"
    ),
    prompts=PAPERALL50_PROMPTS,
    intake_task="dagger50_20260829_h32_v1",
    step=4_999,
    checkpoint_role="final",
    model_sha256=(
        "e35ace7b0a301d4f6f72e48c26a8aaee51dc9371ce6556d4d62236c84842eea4"
    ),
    package_manifest_sha256=(
        "31480621add28ed32e79ad722367354d47b27a3adcdd8fb418eadf51df3f6f9b"
    ),
    oss_uri=(
        "oss://ali-pai-oss-ppu/spy/deploy/"
        "dual_piper_dagger50_20260829_h32_v1_pi05_ema_step4999_ckpt/"
    ),
    oss_total_bytes=7_245_891_272,
)
OBJECT50_EMA_STEP6000 = H32BundleIdentity(
    package_id=(
        "dual_piper_object50_20260830_h32_v1_pi05_ema_step6000_ckpt"
    ),
    dataset_id="dual_piper_object50_from_dagger4999_h32_v1",
    data_version="20260830_object50_h32_v1",
    norm_sha256=(
        "39283c6f688b6c7fbe343a2cfa1f56e3fff1e2cb73219cf65b920224f55e14ed"
    ),
    prompts=OBJECT50_PROMPTS,
    intake_task="object50_20260830_h32_v1",
    step=6_000,
    checkpoint_role="evaluation_candidate_loss0p0025",
    model_sha256=(
        "aace5e0f50c3c9bf4c3c603b57a2384e9a0b62e90697614299591ab0f92c4a7e"
    ),
    package_manifest_sha256=(
        "e8fc02a93a0db2df6bf20dd2de59bfae2a87677f52c20600ef889cfffaf1a181"
    ),
    oss_uri=(
        "oss://ali-pai-oss-ppu/spy/deploy/"
        "dual_piper_object50_20260830_h32_v1_pi05_ema_step6000_ckpt/"
    ),
    oss_total_bytes=7_245_895_894,
)
OBJECT50_EMA_STEP9999 = H32BundleIdentity(
    package_id=(
        "dual_piper_object50_20260830_h32_v1_pi05_ema_step9999_ckpt"
    ),
    dataset_id="dual_piper_object50_from_dagger4999_h32_v1",
    data_version="20260830_object50_h32_v1",
    norm_sha256=(
        "39283c6f688b6c7fbe343a2cfa1f56e3fff1e2cb73219cf65b920224f55e14ed"
    ),
    prompts=OBJECT50_PROMPTS,
    intake_task="object50_20260830_h32_v1",
    step=9_999,
    checkpoint_role="final",
    model_sha256=(
        "ea6c2cb7db9ce9608fed9b63c7d64c7128781a8449375ed3e718c4fc0da8d3a5"
    ),
    package_manifest_sha256=(
        "97512584b1fac324c848cec9900c39ccb40150a9898a3d019edab38ffad73e23"
    ),
    oss_uri=(
        "oss://ali-pai-oss-ppu/spy/deploy/"
        "dual_piper_object50_20260830_h32_v1_pi05_ema_step9999_ckpt/"
    ),
    oss_total_bytes=7_245_895_816,
)
PEN_TRASH_EMA_STEP9999 = H32BundleIdentity(
    package_id=(
        "dual_piper_pen_trash_20260904_h32_v1_pi05_ema_step9999_ckpt"
    ),
    dataset_id="dual_piper_pen_trash_curriculum50to20_h32_v1",
    data_version="20260904_pen_trash_curriculum50to20_h32_v1",
    norm_sha256=(
        "39283c6f688b6c7fbe343a2cfa1f56e3fff1e2cb73219cf65b920224f55e14ed"
    ),
    prompts=PEN_TRASH_PROMPTS,
    intake_task="pen_trash_20260904_h32_v1",
    step=9_999,
    checkpoint_role="final",
    model_sha256=(
        "9eb688a9c99a1d760ee61645d43c73c7c416158f9779ac1a7215e4a8e755a476"
    ),
    package_manifest_sha256=(
        "ea68c3c62febd3fdeb70a6288905e0d530d834eb026cedeeba80db34827257ec"
    ),
    oss_uri=(
        "oss://ali-pai-oss-ppu/spy/deploy/"
        "dual_piper_pen_trash_20260904_h32_v1_pi05_ema_step9999_ckpt/"
    ),
    oss_total_bytes=7_245_894_467,
)
KNOWN_IDENTITIES = {
    identity.package_id: identity
    for identity in (
        EMA_STEP10000,
        EMA_STEP19999,
        PAPERALL50_EMA_STEP4999,
        DAGGER50_EMA_STEP4999,
        OBJECT50_EMA_STEP6000,
        OBJECT50_EMA_STEP9999,
        PEN_TRASH_EMA_STEP9999,
    )
}

# Backwards-compatible step-10000 names for callers that consume the original
# constants. Runtime verification below selects an exact registered identity.
PACKAGE_ID = EMA_STEP10000.package_id
CONFIG_NAME = "pi05_dual_piper_real"
DATASET_ID = "dual_piper_desktop_cleanup_20260827_h32_v3"
DATA_VERSION = "20260827_h32_v3"
PROJECT_COMMIT = "e458f6d137e894f3110152e3b3f2031def4a8888"
OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
STEP = EMA_STEP10000.step
EMA_DECAY = 0.999
ACTION_HORIZON = 32
EXECUTE_HORIZON = 16
ACTION_FREQUENCY_HZ = 30.0
MODEL_SHA256 = EMA_STEP10000.model_sha256
NORM_SHA256 = "291e81cba57b48de54245e009548a0c0a43e5a793f591a6115af683dbb28dcb9"
TOKENIZER_SHA256 = "8986bb4f423f07f8c7f70d0dbe3526fb2316056c17bae71b1ea975e77a168fc6"
PACKAGE_MANIFEST_SHA256 = EMA_STEP10000.package_manifest_sha256
OSS_URI = EMA_STEP10000.oss_uri
OSS_ENDPOINT = "https://oss-cn-wulanchabu.aliyuncs.com"
OSS_OBJECT_COUNT = 763
OSS_TOTAL_BYTES = EMA_STEP10000.oss_total_bytes
LOCAL_INTAKE_FILES = frozenset({"manifest.json", "SHA256SUMS"})
PROMPTS = DESKTOP_PROMPTS
PAPERALL50_DATASET_ID = PAPERALL50_EMA_STEP4999.dataset_id
PAPERALL50_DATA_VERSION = PAPERALL50_EMA_STEP4999.data_version
PAPERALL50_NORM_SHA256 = PAPERALL50_EMA_STEP4999.norm_sha256
DAGGER50_DATASET_ID = DAGGER50_EMA_STEP4999.dataset_id
DAGGER50_DATA_VERSION = DAGGER50_EMA_STEP4999.data_version
DAGGER50_NORM_SHA256 = DAGGER50_EMA_STEP4999.norm_sha256
OBJECT50_DATASET_ID = OBJECT50_EMA_STEP6000.dataset_id
OBJECT50_DATA_VERSION = OBJECT50_EMA_STEP6000.data_version
OBJECT50_NORM_SHA256 = OBJECT50_EMA_STEP6000.norm_sha256


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


def _bundle_identity(
    contract: dict[str, Any],
    *,
    expected_step: int | None,
    expected_package_id: str | None,
) -> H32BundleIdentity:
    package_id = str(contract.get("package_id", ""))
    try:
        identity = KNOWN_IDENTITIES[package_id]
    except KeyError as exc:
        raise ValueError(
            f"unsupported dual-PiPER h32 EMA package_id: {package_id!r}"
        ) from exc
    if expected_package_id is not None and identity.package_id != expected_package_id:
        raise ValueError(
            "dual-PiPER h32 EMA bundle package mismatch: "
            f"expected {expected_package_id!r}, got {identity.package_id!r}"
        )
    if expected_step is not None and identity.step != int(expected_step):
        raise ValueError(
            "desktop h32 v3 EMA bundle step mismatch: "
            f"expected {int(expected_step)}, got {identity.step}"
        )
    return identity


def verify_piper_h32_ema_bundle(
    bundle_root: Path,
    *,
    full_manifest: bool = False,
    expected_step: int | None = None,
    expected_package_id: str | None = None,
) -> dict[str, Any]:
    """Verify one registered h32 deployment identity and its inventory."""
    root = bundle_root.expanduser().resolve()
    contract_path = root / "contract.json"
    if not contract_path.is_file():
        raise FileNotFoundError(
            "dual-PiPER h32 EMA bundle is incomplete: contract"
        )
    if contract_path.is_symlink():
        raise ValueError("dual-PiPER h32 EMA identity files cannot be symlinks: contract")
    contract = _json(contract_path)
    identity = _bundle_identity(
        contract,
        expected_step=expected_step,
        expected_package_id=expected_package_id,
    )
    required = {
        "deploy": root / "deploy_input.json",
        "contract": contract_path,
        "checkpoint": root / "checkpoint/metadata.json",
        "weights": root / "checkpoint/model.safetensors",
        "norm": root / f"checkpoint/assets/{identity.dataset_id}/norm_stats.json",
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
        raise FileNotFoundError(
            "dual-PiPER h32 EMA bundle is incomplete: " + ", ".join(missing)
        )
    symlinks = sorted(name for name, path in required.items() if path.is_symlink())
    if symlinks:
        raise ValueError(
            "dual-PiPER h32 EMA identity files cannot be symlinks: "
            + ", ".join(symlinks)
        )

    deploy = _json(required["deploy"])
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
    source = deploy.get("source", {})
    runtime = deploy.get("runtime", {})

    checks = {
        "package_id": contract.get("package_id") == identity.package_id
        and deploy.get("package_id") == identity.package_id
        and package.get("package_id") == identity.package_id,
        "model": contract.get("model") == "pi0.5",
        "config": contract.get("train_config") == CONFIG_NAME
        and runtime.get("train_config") == CONFIG_NAME,
        "dataset": contract.get("dataset_id") == identity.dataset_id
        and runtime.get("dataset_id") == identity.dataset_id,
        "data_version": contract.get("data_version") == identity.data_version
        and runtime.get("data_version") == identity.data_version,
        "step": contract.get("step") == identity.step
        and deploy.get("checkpoint_step") == identity.step
        and checkpoint.get("checkpoint_step") == identity.step,
        "ema": contract.get("params_semantics") == "ema"
        and deploy.get("params_semantics") == "ema"
        and checkpoint.get("params_semantics") == "ema"
        and checkpoint.get("ema_configured") == EMA_DECAY
        and checkpoint.get("ema_exported") is True,
        "checkpoint_role": deploy.get("checkpoint_role")
        == identity.checkpoint_role
        and checkpoint.get("checkpoint_role") == identity.checkpoint_role,
        "source_format": source.get("training_checkpoint_format") == "orbax_jax"
        and checkpoint.get("source_checkpoint_format") == "orbax_jax",
        "commits": source.get("project_commit") == PROJECT_COMMIT
        and source.get("project_commit_verified") is True
        and source.get("openpi_commit") == OPENPI_COMMIT,
        "source_hashes": source.get("checkpoint_sha256") == identity.model_sha256
        and checkpoint.get("model_sha256") == identity.model_sha256
        and source.get("norm_stats_sha256") == identity.norm_sha256
        and source.get("tokenizer_sha256") == TOKENIZER_SHA256,
        "observation": contract.get("observation", {}).get("head_rgb")
        == "uint8[H,W,3] RGB"
        and contract.get("observation", {}).get("left_wrist_rgb")
        == "uint8[H,W,3] RGB"
        and contract.get("observation", {}).get("right_wrist_rgb")
        == "uint8[H,W,3] RGB"
        and contract.get("observation", {}).get("state")
        == "float32[14] absolute joints/grippers",
        "action": contract.get("action", {}).get("shape") == [ACTION_HORIZON, 14]
        and contract.get("action", {}).get("representation")
        == "absolute joint/gripper targets after output transform",
        "prompts": tuple(contract.get("canonical_prompts", ())) == identity.prompts,
        "robot": robot.get("state_dim") == 14
        and robot.get("action_dim") == 14
        and robot.get("joint_unit") == "radian"
        and robot.get("gripper_unit") == "meter",
        "cameras": set(cameras) == {"head", "left_wrist", "right_wrist"}
        and all(
            cameras[name].get("width") == 1280
            and cameras[name].get("height") == 720
            and cameras[name].get("fps") == 30
            for name in cameras
        ),
        "execution": policy.get("action_representation")
        == "absolute_joint_position"
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
        raise ValueError(
            "dual-PiPER h32 EMA bundle identity mismatch: " + ", ".join(failed)
        )

    if package.get("schema") != "coco-bundle-manifest/1":
        raise ValueError("unsupported desktop h32 v3 package manifest schema")
    if package.get("file_count") != len(package_entries):
        raise ValueError("desktop h32 v3 package manifest file_count mismatch")
    if package.get("total_bytes") != sum(size for size, _ in package_entries.values()):
        raise ValueError("desktop h32 v3 package manifest total_bytes mismatch")
    publisher_files = set(package_entries) | {"package_manifest.json"}
    intake_source = intake.get("source", {})
    intake_model = intake.get("model", {})
    if (
        intake.get("schema_version") != 1
        or intake_source.get("uri") != identity.oss_uri
        or intake_source.get("endpoint") != OSS_ENDPOINT
        or intake_source.get("object_count") != OSS_OBJECT_COUNT
        or intake_source.get("total_bytes") != identity.oss_total_bytes
        or intake_model.get("family") != "pi05"
        or intake_model.get("robot") != "dual_piper"
        or intake_model.get("task") != identity.intake_task
        or intake_model.get("step") != str(identity.step)
        or intake_model.get("params_semantics") != "ema"
        or intake_model.get("train_config") != CONFIG_NAME
        or set(intake_entries) != publisher_files
        or set(sums) != publisher_files
    ):
        raise ValueError("desktop h32 v3 model-store intake identity mismatch")
    for relative, expected in package_entries.items():
        if intake_entries.get(relative) != expected or sums.get(relative) != expected[1]:
            raise ValueError(f"desktop h32 v3 intake/package mismatch: {relative}")
    package_digest = _sha256(required["package"])
    if package_digest != identity.package_manifest_sha256:
        raise ValueError("desktop h32 v3 package manifest identity mismatch")
    if intake_entries.get("package_manifest.json") != (
        required["package"].stat().st_size,
        package_digest,
    ) or sums.get("package_manifest.json") != package_digest:
        raise ValueError("desktop h32 v3 package manifest intake checksum mismatch")

    critical_hashes = {
        "checkpoint/model.safetensors": identity.model_sha256,
        f"checkpoint/assets/{identity.dataset_id}/norm_stats.json": identity.norm_sha256,
        "assets/big_vision/paligemma_tokenizer.model": TOKENIZER_SHA256,
    }
    for relative, digest in critical_hashes.items():
        path = root / relative
        if package_entries.get(relative, (None, None))[1] != digest:
            raise ValueError(f"desktop h32 v3 critical asset hash mismatch: {relative}")
        if not full_manifest and _sha256(path) != digest:
            raise ValueError(f"desktop h32 v3 critical asset hash mismatch: {relative}")

    if full_manifest:
        bundle_symlinks = sorted(
            path.relative_to(root).as_posix()
            for path in root.rglob("*")
            if path.is_symlink()
        )
        if bundle_symlinks:
            raise ValueError(
                "desktop h32 v3 bundle cannot contain symlinks: "
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
                "desktop h32 v3 bundle inventory mismatch: "
                f"missing={sorted(publisher_files - actual)}, "
                f"extra={sorted(actual - publisher_files)}"
            )
        for relative, (size, digest) in intake_entries.items():
            path = root / relative
            if path.stat().st_size != size or _sha256(path) != digest:
                raise ValueError(f"desktop h32 v3 package hash mismatch: {relative}")

    return {
        "ok": True,
        "bundle": str(root),
        "package_id": identity.package_id,
        "step": identity.step,
        "checkpoint_role": identity.checkpoint_role,
        "params_semantics": "ema",
        "ema_decay": EMA_DECAY,
        "train_config": CONFIG_NAME,
        "action_shape": [ACTION_HORIZON, 14],
        "execute_horizon": EXECUTE_HORIZON,
        "dataset_id": identity.dataset_id,
        "data_version": identity.data_version,
        "prompts": list(identity.prompts),
        "package_file_count": len(package_entries),
        "package_total_bytes": package["total_bytes"],
        "full_manifest_verified": full_manifest,
        "deploy": deploy,
        "contract": contract,
    }


def verify_piper_desktop_h32_v3_bundle(
    bundle_root: Path,
    *,
    full_manifest: bool = False,
    expected_step: int | None = None,
    expected_package_id: str | None = None,
) -> dict[str, Any]:
    """Compatibility entrypoint for the original desktop h32-v3 profiles."""
    return verify_piper_h32_ema_bundle(
        bundle_root,
        full_manifest=full_manifest,
        expected_step=expected_step,
        expected_package_id=expected_package_id,
    )
