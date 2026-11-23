#!/usr/bin/env python
# -- coding: UTF-8
"""
#!/usr/bin/python3
"""
import dataclasses
import importlib.util
import json
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np

from XPolicyLab.model_template import ModelTemplate
from XPolicyLab.real_world.contracts import ActionChunk, ObservationFrame
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
    ASTRIBOT_S1_STATE_SCHEMA,
)
from XPolicyLab.real_world.robots.gr3.mapping import (
    GR3_ACTION_SCHEMA,
    GR3_STATE_SCHEMA,
)
from XPolicyLab.real_world.robots.piper_x_dual.mapping import (
    PIPER_X_DUAL_ACTION_SCHEMA,
    PIPER_X_DUAL_STATE_SCHEMA,
)
from XPolicyLab.policy.Pi_05.piper_desktop_ema_bundle import (
    ACTION_FREQUENCY_HZ as PIPER_DESKTOP_EMA_ACTION_FREQUENCY_HZ,
    ACTION_HORIZON as PIPER_DESKTOP_EMA_ACTION_HORIZON,
    CONFIG_NAME as PIPER_DESKTOP_EMA_CONFIG_NAME,
    DATASET_ID as PIPER_DESKTOP_EMA_DATASET_ID,
    EXECUTE_HORIZON as PIPER_DESKTOP_EMA_EXECUTE_HORIZON,
    verify_piper_desktop_ema_bundle,
)
from XPolicyLab.policy.Pi_05.piper_build_blocks_ema_bundle import (
    ACTION_FREQUENCY_HZ as PIPER_BUILD_BLOCKS_EMA_ACTION_FREQUENCY_HZ,
    ACTION_HORIZON as PIPER_BUILD_BLOCKS_EMA_ACTION_HORIZON,
    CONFIG_NAME as PIPER_BUILD_BLOCKS_EMA_CONFIG_NAME,
    DATASET_ID as PIPER_BUILD_BLOCKS_EMA_DATASET_ID,
    EXECUTE_HORIZON as PIPER_BUILD_BLOCKS_EMA_EXECUTE_HORIZON,
    IMAGE_SIZE as PIPER_BUILD_BLOCKS_EMA_IMAGE_SIZE,
    PROMPT as PIPER_BUILD_BLOCKS_EMA_PROMPT,
    verify_piper_build_blocks_ema_bundle,
)
from XPolicyLab.policy.Pi_05.piper_desktop_h32_v3_bundle import (
    ACTION_FREQUENCY_HZ as PIPER_DESKTOP_H32_V3_ACTION_FREQUENCY_HZ,
    ACTION_HORIZON as PIPER_DESKTOP_H32_V3_ACTION_HORIZON,
    CONFIG_NAME as PIPER_DESKTOP_H32_V3_CONFIG_NAME,
    DATASET_ID as PIPER_DESKTOP_H32_V3_DATASET_ID,
    EXECUTE_HORIZON as PIPER_DESKTOP_H32_V3_EXECUTE_HORIZON,
    OBJECT50_PROMPTS as PIPER_OBJECT50_PROMPTS,
    verify_piper_desktop_h32_v3_bundle,
    verify_piper_h32_ema_bundle,
)
from XPolicyLab.policy.Pi_05.piper_paperall50_h32_v1_bundle import (
    ACTION_FREQUENCY_HZ as PIPER_PAPERALL50_H32_V1_ACTION_FREQUENCY_HZ,
    ACTION_HORIZON as PIPER_PAPERALL50_H32_V1_ACTION_HORIZON,
    CONFIG_NAME as PIPER_PAPERALL50_H32_V1_CONFIG_NAME,
    DATASET_ID as PIPER_PAPERALL50_H32_V1_DATASET_ID,
    EXECUTE_HORIZON as PIPER_PAPERALL50_H32_V1_EXECUTE_HORIZON,
    PROMPTS as PIPER_PAPERALL50_H32_V1_PROMPTS,
    verify_piper_paperall50_h32_v1_bundle,
)
from XPolicyLab.policy.Pi_05.piper_dagger50_h32_v1_bundle import (
    ACTION_FREQUENCY_HZ as PIPER_DAGGER50_H32_V1_ACTION_FREQUENCY_HZ,
    ACTION_HORIZON as PIPER_DAGGER50_H32_V1_ACTION_HORIZON,
    CONFIG_NAME as PIPER_DAGGER50_H32_V1_CONFIG_NAME,
    DATASET_ID as PIPER_DAGGER50_H32_V1_DATASET_ID,
    EXECUTE_HORIZON as PIPER_DAGGER50_H32_V1_EXECUTE_HORIZON,
    PROMPTS as PIPER_DAGGER50_H32_V1_PROMPTS,
    verify_piper_dagger50_h32_v1_bundle,
)
from XPolicyLab.policy.Pi_05.piper_pen_trash_h32_v1_bundle import (
    ACTION_FREQUENCY_HZ as PIPER_PEN_TRASH_H32_V1_ACTION_FREQUENCY_HZ,
    ACTION_HORIZON as PIPER_PEN_TRASH_H32_V1_ACTION_HORIZON,
    CONFIG_NAME as PIPER_PEN_TRASH_H32_V1_CONFIG_NAME,
    DATASET_ID as PIPER_PEN_TRASH_H32_V1_DATASET_ID,
    EXECUTE_HORIZON as PIPER_PEN_TRASH_H32_V1_EXECUTE_HORIZON,
    PROMPTS as PIPER_PEN_TRASH_H32_V1_PROMPTS,
    verify_piper_pen_trash_h32_v1_bundle,
)
from XPolicyLab.policy.Pi_05.piper_multitask_bundle import (
    ACTION_FREQUENCY_HZ as PIPER_MULTITASK_ACTION_FREQUENCY_HZ,
    ACTION_HORIZON as PIPER_MULTITASK_ACTION_HORIZON,
    CONFIG_NAME as PIPER_MULTITASK_CONFIG_NAME,
    EXECUTE_HORIZON as PIPER_MULTITASK_EXECUTE_HORIZON,
    PROMPTS as PIPER_MULTITASK_PROMPTS,
    verify_piper_multitask_bundle,
)
from XPolicyLab.utils.checkpoint_resolver import candidate_checkpoint_roots
from XPolicyLab.utils.process_data import (
    decode_image_bit,
    get_robot_action_dim_info,
    pack_robot_state,
    unpack_robot_state,
)


_POLICY_DIR = Path(__file__).resolve().parent
_CHECKPOINTS_DIR = _POLICY_DIR / "checkpoints"
GR3_PROFILE = "gr3_anygrasp"
ASTRIBOT_S1_PROFILE = "astribot_s1_tomato_v3"
PIPER_X_DUAL_PROFILE = "piper_x_dual_build_blocks_v1"
PIPER_X_DUAL_BUILD_BLOCKS_EMA_PROFILE = (
    "piper_x_dual_build_blocks_ema_h15_v1"
)
PIPER_X_DUAL_DESKTOP_CLEANUP_PROFILE = "piper_x_dual_desktop_cleanup_v1"
PIPER_X_DUAL_DESKTOP_EMA_PROFILE = "piper_x_dual_desktop_cleanup_ema_h16_v1"
PIPER_X_DUAL_DESKTOP_H32_V3_EMA_PROFILE = (
    "piper_x_dual_desktop_cleanup_ema_h32_v3"
)
PIPER_X_DUAL_PAPERALL50_H32_V1_EMA_PROFILE = (
    "piper_x_dual_paperall50_ema_h32_v1"
)
PIPER_X_DUAL_DAGGER50_H32_V1_EMA_PROFILE = (
    "piper_x_dual_dagger50_ema_h32_v1"
)
PIPER_X_DUAL_PEN_TRASH_H32_V1_EMA_PROFILE = (
    "piper_x_dual_pen_trash_ema_h32_v1"
)
PIPER_X_DUAL_H32_EMA_PROFILE = "piper_x_dual_h32_ema"
PIPER_X_DUAL_MULTITASK5_PROFILE = "piper_x_dual_multitask5_ema_h16_v1"
ASTRIBOT_TOMATO_PROMPT = "pick the tomato"
ASTRIBOT_ACTION_HORIZON = 30
PIPER_ACTION_HORIZON = 15
PIPER_DESKTOP_ACTION_HORIZON = 10
PIPER_DESKTOP_ACTION_FREQUENCY_HZ = 15.0
PIPER_DESKTOP_EXECUTE_PREFIX_STEPS = 10
PIPER_DESKTOP_DATASET_ID = "dual_piper_desktop_cleanup_20260825_v1"
PIPER_BUILD_BLOCKS_PROMPT = (
    "Stack the second cube on top of the first cube, then place the cone on top of the second cube."
)
PIPER_DESKTOP_PROMPTS = (
    "Close the laptop.",
    "Pick up one drink container and place it on the coaster.",
    "Pick up one mouse and place it on the mouse pad.",
    "Pick up one paper ball and place it in the trash bin.",
    "Pick up one pen and place it in the pen holder.",
)
PI05_ACTION_KEEP = (1, 2, *range(5, 31), 32)

REAL_WORLD_PROFILES = {
    GR3_PROFILE,
    ASTRIBOT_S1_PROFILE,
    PIPER_X_DUAL_PROFILE,
    PIPER_X_DUAL_BUILD_BLOCKS_EMA_PROFILE,
    PIPER_X_DUAL_DESKTOP_CLEANUP_PROFILE,
    PIPER_X_DUAL_DESKTOP_EMA_PROFILE,
    PIPER_X_DUAL_DESKTOP_H32_V3_EMA_PROFILE,
    PIPER_X_DUAL_PAPERALL50_H32_V1_EMA_PROFILE,
    PIPER_X_DUAL_DAGGER50_H32_V1_EMA_PROFILE,
    PIPER_X_DUAL_PEN_TRASH_H32_V1_EMA_PROFILE,
    PIPER_X_DUAL_H32_EMA_PROFILE,
    PIPER_X_DUAL_MULTITASK5_PROFILE,
}


def _extract_step_number(value: Any) -> int | None:
    matches = [part for part in str(value).split("/") if part]
    if not matches:
        return None
    digits = "".join(ch for ch in matches[-1] if ch.isdigit())
    return int(digits) if digits else None


def _resolve_pi05_model_root(model_cfg: dict[str, Any]) -> Path:
    # Shared precedence: model_path/checkpoint_path keys > ckpt_name-as-path >
    # {bench}-{ckpt}-{env}-{action}-{seed} concat > checkpoints/<ckpt_name>.
    candidates = candidate_checkpoint_roots(
        model_cfg,
        _CHECKPOINTS_DIR,
        policy_dir=_POLICY_DIR,
        explicit_keys=("model_path", "checkpoint_path"),
    )
    if not candidates:
        raise ValueError("ckpt_name or model_path is required for Pi_05.")
    checkpoint_root = next((candidate for candidate in candidates if candidate.exists()), candidates[0])
    if not checkpoint_root.is_dir():
        return checkpoint_root

    candidate_dirs = []
    if (checkpoint_root / "params").exists() or (checkpoint_root / "assets").exists():
        candidate_dirs.append(checkpoint_root)
    candidate_dirs.extend(
        child
        for child in sorted(checkpoint_root.iterdir())
        if child.is_dir() and ((child / "params").exists() or (child / "assets").exists())
    )
    if not candidate_dirs:
        return checkpoint_root

    checkpoint_num = model_cfg.get("checkpoint_num")
    desired_step = _extract_step_number(checkpoint_num)
    if desired_step is not None:
        normalized = str(desired_step)
        for candidate in candidate_dirs:
            name = candidate.name.lstrip("0") or "0"
            if name == normalized:
                return candidate

        for candidate in candidate_dirs:
            candidate_step = _extract_step_number(candidate.name)
            if candidate_step is None:
                continue
            scaled_step = desired_step
            while len(str(scaled_step)) < len(str(candidate_step)):
                scaled_step *= 10
            if candidate_step in {desired_step, scaled_step}:
                return candidate

    numeric_dirs = [candidate for candidate in candidate_dirs if _extract_step_number(candidate.name) is not None]
    if numeric_dirs:
        return max(numeric_dirs, key=lambda candidate: _extract_step_number(candidate.name) or -1)
    return candidate_dirs[0]


def _is_true(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def encode_gr3_observation(
    observation: ObservationFrame,
    camera_name: str = "cam_head",
) -> dict[str, Any]:
    if observation.robot_type != "gr3":
        raise ValueError(f"PI0.5 GR3 adapter received {observation.robot_type!r}")
    if observation.state_schema != GR3_STATE_SCHEMA:
        raise ValueError(
            f"expected state schema {GR3_STATE_SCHEMA.schema_id}, "
            f"got {observation.state_schema.schema_id}"
        )
    if camera_name not in observation.images:
        raise ValueError(f"observation is missing camera {camera_name!r}")
    image = np.asarray(observation.images[camera_name])
    if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
        raise ValueError(f"PI0.5 camera must be uint8 HxWx3, got {image.dtype} {image.shape}")
    return {
        "observation/state": np.asarray(observation.state, dtype=np.float32),
        "observation/image": image,
        "prompt": observation.instruction,
    }


def scatter_gr3_actions(actions: Any) -> np.ndarray:
    values = np.asarray(actions, dtype=np.float32)
    if values.ndim == 3 and values.shape[0] == 1:
        values = values[0]
    if values.ndim == 1:
        values = values[None]
    if values.ndim != 2 or values.shape[1] != len(PI05_ACTION_KEEP):
        raise ValueError(
            f"PI0.5 action must be [T,{len(PI05_ACTION_KEEP)}], got {values.shape}"
        )
    if not np.isfinite(values).all():
        raise ValueError("PI0.5 action contains NaN or Inf")
    result = np.zeros((len(values), GR3_ACTION_SCHEMA.dim), dtype=np.float32)
    result[:, PI05_ACTION_KEEP] = values
    return result


def encode_astribot_observation(
    observation: ObservationFrame,
    camera_name: str = "cam_head",
) -> dict[str, Any]:
    """Validate the frozen tomato-v3 input contract.

    Astribot may publish three synchronized cameras, but this checkpoint was
    trained with head RGB only. Wrist images remain available in the shared
    observation for other policies and are intentionally ignored here.
    """
    if observation.robot_type != "astribot_s1":
        raise ValueError(
            f"PI0.5 Astribot adapter received {observation.robot_type!r}"
        )
    if observation.state_schema != ASTRIBOT_S1_STATE_SCHEMA:
        raise ValueError(
            f"expected state schema {ASTRIBOT_S1_STATE_SCHEMA.schema_id}, "
            f"got {observation.state_schema.schema_id}"
        )
    if observation.instruction != ASTRIBOT_TOMATO_PROMPT:
        raise ValueError(
            "tomato-v3 prompt must exactly equal "
            f"{ASTRIBOT_TOMATO_PROMPT!r}"
        )
    if camera_name not in observation.images:
        raise ValueError(f"observation is missing camera {camera_name!r}")
    image = np.asarray(observation.images[camera_name])
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3:
        raise ValueError(
            f"PI0.5 camera must be uint8 HxWx3, got {image.dtype} {image.shape}"
        )
    state = np.asarray(observation.state, dtype=np.float32)
    if state.shape != (ASTRIBOT_S1_STATE_SCHEMA.dim,) or not np.isfinite(state).all():
        raise ValueError("PI0.5 Astribot state must be finite float32 full25")
    return {"image": image, "state": state, "prompt": observation.instruction}


def encode_piper_observation(observation: ObservationFrame) -> dict[str, Any]:
    if observation.robot_type != "piper_x_dual":
        raise ValueError(f"PI0.5 Piper adapter received {observation.robot_type!r}")
    if observation.state_schema != PIPER_X_DUAL_STATE_SCHEMA:
        raise ValueError(
            f"expected state schema {PIPER_X_DUAL_STATE_SCHEMA.schema_id}, "
            f"got {observation.state_schema.schema_id}"
        )
    if observation.instruction != PIPER_BUILD_BLOCKS_PROMPT:
        raise ValueError(
            "Piper build-blocks prompt must exactly equal "
            f"{PIPER_BUILD_BLOCKS_PROMPT!r}"
        )
    camera_mapping = {
        "head": "camera_head",
        "left_wrist": "camera_left",
        "right_wrist": "camera_right",
    }
    images: dict[str, np.ndarray] = {}
    for model_name, runtime_name in camera_mapping.items():
        if runtime_name not in observation.images:
            raise ValueError(f"Piper observation is missing camera {runtime_name!r}")
        image = np.asarray(observation.images[runtime_name])
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3:
            raise ValueError(
                f"Piper camera {runtime_name} must be uint8 HxWx3, got {image.dtype} {image.shape}"
            )
        images[model_name] = np.transpose(image, (2, 0, 1))
    return {
        "images": images,
        "state": np.asarray(observation.state, dtype=np.float32),
        "prompt": observation.instruction,
    }


def _resize_rgb_uint8(
    image: np.ndarray, *, height: int, width: int
) -> np.ndarray:
    """Resize one RGB frame with dependency-free bilinear interpolation."""
    source_height, source_width = image.shape[:2]
    if (source_height, source_width) == (height, width):
        return image
    y = np.linspace(0, source_height - 1, height, dtype=np.float32)
    x = np.linspace(0, source_width - 1, width, dtype=np.float32)
    y0 = np.floor(y).astype(np.intp)
    x0 = np.floor(x).astype(np.intp)
    y1 = np.minimum(y0 + 1, source_height - 1)
    x1 = np.minimum(x0 + 1, source_width - 1)
    wy = (y - y0).reshape(height, 1, 1)
    wx = (x - x0).reshape(1, width, 1)
    top = (
        image[y0[:, None], x0[None, :]].astype(np.float32) * (1.0 - wx)
        + image[y0[:, None], x1[None, :]].astype(np.float32) * wx
    )
    bottom = (
        image[y1[:, None], x0[None, :]].astype(np.float32) * (1.0 - wx)
        + image[y1[:, None], x1[None, :]].astype(np.float32) * wx
    )
    return np.clip(np.rint(top * (1.0 - wy) + bottom * wy), 0, 255).astype(
        np.uint8
    )


def encode_piper_build_blocks_ema_observation(
    observation: ObservationFrame,
) -> dict[str, Any]:
    """Validate and map the frozen Jiaxin build-blocks bundle contract."""
    if observation.robot_type != "piper_x_dual":
        raise ValueError(
            f"PI0.5 Piper build-blocks adapter received {observation.robot_type!r}"
        )
    if observation.state_schema != PIPER_X_DUAL_STATE_SCHEMA:
        raise ValueError(
            f"expected state schema {PIPER_X_DUAL_STATE_SCHEMA.schema_id}, "
            f"got {observation.state_schema.schema_id}"
        )
    if observation.instruction != PIPER_BUILD_BLOCKS_EMA_PROMPT:
        raise ValueError(
            "Piper build-blocks prompt must exactly match the EMA bundle contract"
        )
    camera_mapping = {
        "head": "camera_head",
        "left_wrist": "camera_left",
        "right_wrist": "camera_right",
    }
    images: dict[str, np.ndarray] = {}
    target_height, target_width = PIPER_BUILD_BLOCKS_EMA_IMAGE_SIZE
    for model_name, runtime_name in camera_mapping.items():
        if runtime_name not in observation.images:
            raise ValueError(
                f"Piper build-blocks observation is missing camera {runtime_name!r}"
            )
        image = np.asarray(observation.images[runtime_name])
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3:
            raise ValueError(
                f"Piper camera {runtime_name} must be uint8 HxWx3, "
                f"got {image.dtype} {image.shape}"
            )
        if image.shape[:2] != (target_height, target_width):
            image = _resize_rgb_uint8(
                image,
                height=target_height,
                width=target_width,
            )
        images[model_name] = image
    state = np.asarray(observation.state, dtype=np.float32)
    if state.shape != (PIPER_X_DUAL_STATE_SCHEMA.dim,) or not np.isfinite(
        state
    ).all():
        raise ValueError("Piper build-blocks state must be finite float32 state14")
    return {
        "images": images,
        "state": state,
        "prompt": observation.instruction,
    }


def encode_piper_desktop_observation(
    observation: ObservationFrame,
    allowed_prompts: tuple[str, ...] = PIPER_DESKTOP_PROMPTS,
) -> dict[str, Any]:
    """Validate and map the packaged desktop-cleanup dual-PiPER contract."""
    if observation.robot_type != "piper_x_dual":
        raise ValueError(
            f"PI0.5 desktop-cleanup adapter received {observation.robot_type!r}"
        )
    if observation.state_schema != PIPER_X_DUAL_STATE_SCHEMA:
        raise ValueError(
            f"expected state schema {PIPER_X_DUAL_STATE_SCHEMA.schema_id}, "
            f"got {observation.state_schema.schema_id}"
        )
    if observation.instruction not in allowed_prompts:
        raise ValueError(
            "desktop-cleanup prompt is not in the checkpoint's canonical catalog"
        )
    camera_mapping = {
        "head_rgb": "camera_head",
        "left_wrist_rgb": "camera_left",
        "right_wrist_rgb": "camera_right",
    }
    encoded: dict[str, Any] = {}
    for model_name, runtime_name in camera_mapping.items():
        if runtime_name not in observation.images:
            raise ValueError(
                f"desktop-cleanup observation is missing camera {runtime_name!r}"
            )
        image = np.asarray(observation.images[runtime_name])
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3:
            raise ValueError(
                f"Piper camera {runtime_name} must be uint8 HxWx3, "
                f"got {image.dtype} {image.shape}"
            )
        encoded[model_name] = image
    state = np.asarray(observation.state, dtype=np.float32)
    if (
        state.shape != (PIPER_X_DUAL_STATE_SCHEMA.dim,)
        or not np.isfinite(state).all()
    ):
        raise ValueError("desktop-cleanup Piper state must be finite float32 state14")
    encoded["state"] = state
    encoded["prompt"] = observation.instruction
    return encoded


def encode_piper_multitask_observation(
    observation: ObservationFrame,
) -> dict[str, Any]:
    """Validate and map the frozen JAX/Orbax multitask bundle contract."""
    if observation.robot_type != "piper_x_dual":
        raise ValueError(
            f"PI0.5 Piper multitask adapter received {observation.robot_type!r}"
        )
    if observation.state_schema != PIPER_X_DUAL_STATE_SCHEMA:
        raise ValueError(
            f"expected state schema {PIPER_X_DUAL_STATE_SCHEMA.schema_id}, "
            f"got {observation.state_schema.schema_id}"
        )
    if observation.instruction not in PIPER_MULTITASK_PROMPTS:
        raise ValueError(
            "Piper multitask prompt is not in the bundle's exact task catalog"
        )
    camera_mapping = {
        "observation/head_image": "camera_head",
        "observation/left_wrist_image": "camera_left",
        "observation/right_wrist_image": "camera_right",
    }
    encoded: dict[str, Any] = {}
    for model_name, runtime_name in camera_mapping.items():
        if runtime_name not in observation.images:
            raise ValueError(
                f"Piper multitask observation is missing camera {runtime_name!r}"
            )
        image = np.asarray(observation.images[runtime_name])
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3:
            raise ValueError(
                f"Piper camera {runtime_name} must be uint8 HxWx3, "
                f"got {image.dtype} {image.shape}"
            )
        encoded[model_name] = image
    state = np.asarray(observation.state, dtype=np.float32)
    if state.shape != (PIPER_X_DUAL_STATE_SCHEMA.dim,) or not np.isfinite(
        state
    ).all():
        raise ValueError("Piper multitask state must be finite float32 state14")
    encoded["observation/state"] = state
    encoded["prompt"] = observation.instruction
    return encoded


def _load_astribot_bundle_module(bundle_root: Path):
    infer_path = bundle_root / "infer.py"
    if not infer_path.is_file():
        raise FileNotFoundError(f"Astribot bundle infer.py is missing: {infer_path}")
    module_name = "_xpolicylab_pi05_astribot_bundle_infer"
    spec = importlib.util.spec_from_file_location(module_name, infer_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load Astribot bundle inference module: {infer_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _verify_astribot_bundle_identity(bundle_root: Path) -> dict[str, Any]:
    ticket_path = bundle_root / "deploy_input.json"
    contract_path = bundle_root / "contract.json"
    checkpoint_metadata_path = bundle_root / "checkpoint" / "metadata.json"
    for path in (ticket_path, contract_path, checkpoint_metadata_path):
        if not path.is_file():
            raise FileNotFoundError(f"Astribot bundle identity file is missing: {path}")
    ticket = json.loads(ticket_path.read_text(encoding="utf-8"))
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    checkpoint_metadata = json.loads(
        checkpoint_metadata_path.read_text(encoding="utf-8")
    )
    expected = {
        "config_name": "pi05_astribot_pick_tomato_v3",
        "step": 20000,
        "params_semantics": "ema",
        "source_commit": "c0311f3bc6204fb82fbad544c433a25ac19b23ef",
        "prompt": ASTRIBOT_TOMATO_PROMPT,
        "contract_id": "astribot_pick_tomato_v3.pi05.core22.full25_adapter.v1",
    }
    for key, wanted in expected.items():
        if ticket.get(key) != wanted:
            raise ValueError(
                f"Astribot bundle identity mismatch for {key}: "
                f"{ticket.get(key)!r} != {wanted!r}"
            )
    if ticket.get("source_commit_verified") is not True:
        raise ValueError("Astribot bundle source commit is not verified")
    if contract.get("contract_id") != expected["contract_id"]:
        raise ValueError("Astribot bundle contract identity mismatch")
    for key in ("config_name", "step", "params_semantics", "source_commit"):
        if checkpoint_metadata.get(key) != expected[key]:
            raise ValueError(f"Astribot checkpoint metadata mismatch for {key}")
    return ticket


def _verify_piper_desktop_bundle_identity(bundle_root: Path) -> dict[str, Any]:
    paths = {
        "ticket": bundle_root / "deploy_input.json",
        "contract": bundle_root / "contract.json",
        "checkpoint": bundle_root / "checkpoint" / "metadata.json",
        "weights": bundle_root / "checkpoint" / "model.safetensors",
        "norm_stats": (
            bundle_root
            / "checkpoint"
            / "assets"
            / PIPER_DESKTOP_DATASET_ID
            / "norm_stats.json"
        ),
        "tokenizer": (
            bundle_root / "assets" / "big_vision" / "paligemma_tokenizer.model"
        ),
        "manifest": bundle_root / "package_manifest.json",
        "validation": bundle_root / "validation" / "synthetic_inference.json",
        "openpi": bundle_root / "runtime" / "openpi" / "src" / "openpi",
        "adapter": (
            bundle_root
            / "runtime"
            / "project"
            / "src"
            / "desk_icl"
            / "training"
            / "openpi_dual_piper_policy.py"
        ),
        "robot_template": (
            bundle_root
            / "runtime"
            / "project"
            / "configs"
            / "real"
            / "dual_piper_gripper_template.json"
        ),
    }
    missing = [name for name, path in paths.items() if not path.exists()]
    if missing:
        raise FileNotFoundError(
            "desktop-cleanup bundle is incomplete: " + ", ".join(sorted(missing))
        )

    ticket = json.loads(paths["ticket"].read_text(encoding="utf-8"))
    contract = json.loads(paths["contract"].read_text(encoding="utf-8"))
    checkpoint = json.loads(paths["checkpoint"].read_text(encoding="utf-8"))
    validation = json.loads(paths["validation"].read_text(encoding="utf-8"))
    robot_template = json.loads(
        paths["robot_template"].read_text(encoding="utf-8")
    )
    robot = robot_template.get("robot", {})
    policy = robot_template.get("policy", {})
    safety = robot_template.get("safety", {})
    step = int(contract.get("step", -1))
    expected_package_id = f"desktop_cleanup_20260825_v1_pi05_raw_step{step}_ckpt"
    checks = {
        "supported_step": step in {5000, 10000},
        "package_id": contract.get("package_id") == expected_package_id,
        "ticket_package_id": ticket.get("package_id") == expected_package_id,
        "model": contract.get("model") == "pi0.5",
        "train_config": contract.get("train_config") == "pi05_dual_piper_official",
        "ticket_train_config": ticket.get("runtime", {}).get("train_config")
        == "pi05_dual_piper_official",
        "dataset_id": ticket.get("runtime", {}).get("dataset_id")
        == PIPER_DESKTOP_DATASET_ID,
        "raw_parameters": contract.get("params_semantics") == "raw"
        and ticket.get("params_semantics") == "raw"
        and checkpoint.get("params_semantics") == "raw",
        "step": ticket.get("checkpoint_step") == step
        and checkpoint.get("checkpoint_step") == step,
        "project_commit": ticket.get("source", {}).get("project_commit")
        == "9888bca00155c11877ff697f634f61fb945eb72a",
        "openpi_commit": ticket.get("source", {}).get("openpi_commit")
        == "15a9616a00943ada6c20a0f158e3adb39df2ccac",
        "model_sha": ticket.get("source", {}).get("checkpoint_sha256")
        == checkpoint.get("model_sha256"),
        "observation_state": contract.get("observation", {}).get("state")
        == "float32[14] absolute joints/grippers",
        "action_shape": contract.get("action", {}).get("shape") == [10, 14],
        "action_representation": contract.get("action", {}).get("representation")
        == "absolute joint/gripper targets after output transform",
        "prompts": tuple(contract.get("canonical_prompts", ()))
        == PIPER_DESKTOP_PROMPTS,
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
        "inference_evidence": ticket.get("validation", {}).get(
            "synthetic_inference"
        )
        == "passed",
        "validation_report": validation.get("restore") is True
        and validation.get("synthetic_inference") is True
        and validation.get("action_shape") == [10, 14]
        and validation.get("all_finite") is True,
    }
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise ValueError(
            "desktop-cleanup bundle identity mismatch: " + ", ".join(failed)
        )
    return {"ticket": ticket, "contract": contract, "checkpoint": checkpoint}


class Model(ModelTemplate):
    def __init__(self, model_cfg: dict[str, Any]):
        super().__init__()
        self.model_cfg = dict(model_cfg)
        self.runtime_profile = str(model_cfg.get("runtime_profile") or "robotwin").lower()
        if self.runtime_profile == GR3_PROFILE:
            self._init_gr3()
            return
        if self.runtime_profile == ASTRIBOT_S1_PROFILE:
            self._init_astribot_s1()
            return
        if self.runtime_profile == PIPER_X_DUAL_PROFILE:
            self._init_piper_x_dual()
            return
        if self.runtime_profile == PIPER_X_DUAL_BUILD_BLOCKS_EMA_PROFILE:
            self._init_piper_x_dual_build_blocks_ema()
            return
        if self.runtime_profile == PIPER_X_DUAL_DESKTOP_CLEANUP_PROFILE:
            self._init_piper_x_dual_desktop_cleanup()
            return
        if self.runtime_profile == PIPER_X_DUAL_DESKTOP_EMA_PROFILE:
            self._init_piper_x_dual_desktop_ema()
            return
        if self.runtime_profile == PIPER_X_DUAL_DESKTOP_H32_V3_EMA_PROFILE:
            self._init_piper_x_dual_desktop_h32_v3_ema()
            return
        if self.runtime_profile == PIPER_X_DUAL_H32_EMA_PROFILE:
            self._init_piper_x_dual_h32_ema()
            return
        if self.runtime_profile == PIPER_X_DUAL_PAPERALL50_H32_V1_EMA_PROFILE:
            self._init_piper_x_dual_paperall50_h32_v1_ema()
            return
        if self.runtime_profile == PIPER_X_DUAL_DAGGER50_H32_V1_EMA_PROFILE:
            self._init_piper_x_dual_dagger50_h32_v1_ema()
            return
        if self.runtime_profile == PIPER_X_DUAL_PEN_TRASH_H32_V1_EMA_PROFILE:
            self._init_piper_x_dual_pen_trash_h32_v1_ema()
            return
        if self.runtime_profile == PIPER_X_DUAL_MULTITASK5_PROFILE:
            self._init_piper_x_dual_multitask5()
            return
        if self.runtime_profile != "robotwin":
            raise ValueError(f"unsupported PI0.5 runtime_profile: {self.runtime_profile!r}")
        self.task_name = model_cfg["task_name"]
        self.action_type = model_cfg.get("action_type", "joint")
        self.robot_action_dim_info = (
            get_robot_action_dim_info(model_cfg["env_cfg_type"]) if model_cfg.get("env_cfg_type") is not None else None
        )
        self.observation_window: dict[str, Any] | None = None
        self._latest_env_idx_list: list[int] = [0]

        self.policy = self.get_model(model_cfg=model_cfg)
        self.model = self.policy

    def _init_piper_x_dual(self) -> None:
        self.action_frequency_hz = float(self.model_cfg.get("action_frequency_hz") or 30.0)
        self.action_horizon = int(self.model_cfg.get("action_horizon") or PIPER_ACTION_HORIZON)
        if self.action_frequency_hz != 30.0 or self.action_horizon != PIPER_ACTION_HORIZON:
            raise ValueError("Piper checkpoint requires 30 Hz and action horizon 15")
        if _is_true(self.model_cfg.get("rtc_enabled", False)):
            raise ValueError("PI0.5 Piper RTC is not implemented")
        self.allow_dummy_policy = _is_true(self.model_cfg.get("allow_dummy_policy", False))
        self.model_root: Path | None = None
        self.observation: ObservationFrame | None = None
        if self.allow_dummy_policy:
            self.policy = None
            self.model = None
            return
        policy_root = Path(
            self.model_cfg.get("pi05_policy_root") or _POLICY_DIR / "openpi"
        ).expanduser().resolve()
        for path in (policy_root, policy_root / "src"):
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))
        from openpi.policies import policy_config
        from openpi.training import config

        self.model_root = _resolve_pi05_model_root(self.model_cfg)
        if not (self.model_root / "params").is_dir() or not (self.model_root / "assets").is_dir():
            raise FileNotFoundError(f"Piper Pi0.5 checkpoint is incomplete: {self.model_root}")
        train_config = config.get_config(
            self.model_cfg.get("train_config_name", "pi05_piper_build_blocks_delta_joint_v1")
        )
        self.policy = policy_config.create_trained_policy(train_config, str(self.model_root))
        self.model = self.policy

    def _init_piper_x_dual_desktop_cleanup(self) -> None:
        self.action_frequency_hz = float(
            self.model_cfg.get("action_frequency_hz")
            or PIPER_DESKTOP_ACTION_FREQUENCY_HZ
        )
        self.action_horizon = int(
            self.model_cfg.get("action_horizon") or PIPER_DESKTOP_ACTION_HORIZON
        )
        self.allow_dummy_policy = _is_true(
            self.model_cfg.get("allow_dummy_policy", False)
        )
        self.observation: ObservationFrame | None = None
        self.model_root: Path | None = None
        self.bundle_identity: dict[str, Any] | None = None
        if self.action_frequency_hz != PIPER_DESKTOP_ACTION_FREQUENCY_HZ:
            raise ValueError("desktop-cleanup Piper action frequency must be exactly 15 Hz")
        if self.action_horizon != PIPER_DESKTOP_ACTION_HORIZON:
            raise ValueError("desktop-cleanup Piper action horizon must be exactly 10")
        if _is_true(self.model_cfg.get("rtc_enabled", False)):
            raise ValueError("PI0.5 desktop-cleanup Piper RTC is not implemented")
        if self.allow_dummy_policy:
            self.policy = None
            self.model = None
            return

        raw_bundle_root = self.model_cfg.get("bundle_root") or self.model_cfg.get(
            "checkpoint_path"
        )
        if not raw_bundle_root:
            raise ValueError(
                "bundle_root is required for the desktop-cleanup Piper profile"
            )
        self.model_root = Path(raw_bundle_root).expanduser().resolve()
        self.bundle_identity = _verify_piper_desktop_bundle_identity(self.model_root)
        runtime_paths = (
            self.model_root / "runtime" / "project" / "src",
            self.model_root / "runtime" / "openpi" / "src",
            self.model_root
            / "runtime"
            / "openpi"
            / "packages"
            / "openpi-client"
            / "src",
            self.model_root / "runtime" / "lerobot",
        )
        for path in runtime_paths:
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))
        os.environ["OPENPI_DATA_HOME"] = str(self.model_root / "assets")
        os.environ["OPENPI_DATASET_ID"] = PIPER_DESKTOP_DATASET_ID

        from openpi.policies import policy_config
        from openpi.training import config

        train_config = config.get_config("pi05_dual_piper_official")
        self.policy = policy_config.create_trained_policy(
            train_config,
            self.model_root / "checkpoint",
            pytorch_device=str(self.model_cfg.get("pytorch_device") or "cuda"),
        )
        self.model = self.policy

    def _init_piper_x_dual_build_blocks_ema(self) -> None:
        self.action_frequency_hz = float(
            self.model_cfg.get("action_frequency_hz")
            or PIPER_BUILD_BLOCKS_EMA_ACTION_FREQUENCY_HZ
        )
        self.action_horizon = int(
            self.model_cfg.get("action_horizon")
            or PIPER_BUILD_BLOCKS_EMA_ACTION_HORIZON
        )
        self.allow_dummy_policy = _is_true(
            self.model_cfg.get("allow_dummy_policy", False)
        )
        self.observation: ObservationFrame | None = None
        self.model_root: Path | None = None
        self.runtime_source_root: Path | None = None
        self.bundle_identity: dict[str, Any] | None = None
        if self.action_frequency_hz != PIPER_BUILD_BLOCKS_EMA_ACTION_FREQUENCY_HZ:
            raise ValueError(
                "Piper build-blocks EMA action frequency must be exactly 15 Hz"
            )
        if self.action_horizon != PIPER_BUILD_BLOCKS_EMA_ACTION_HORIZON:
            raise ValueError(
                "Piper build-blocks EMA action horizon must be exactly 15"
            )
        if _is_true(self.model_cfg.get("rtc_enabled", False)):
            raise ValueError(
                "PI0.5 build-blocks model-native RTC is not implemented; "
                "use the shared client RTC"
            )
        if self.allow_dummy_policy:
            self.policy = None
            self.model = None
            return

        raw_bundle_root = self.model_cfg.get("bundle_root") or self.model_cfg.get(
            "checkpoint_path"
        )
        raw_runtime_root = self.model_cfg.get("runtime_source_root") or os.getenv(
            "PI05_RUNTIME_SOURCE_ROOT"
        )
        if not raw_bundle_root:
            raise ValueError(
                "bundle_root is required for the Piper build-blocks EMA profile"
            )
        if not raw_runtime_root:
            raise ValueError(
                "runtime_source_root is required; run "
                "setup_piper_build_blocks_ema12k.sh"
            )
        self.model_root = Path(raw_bundle_root).expanduser().resolve()
        self.runtime_source_root = Path(raw_runtime_root).expanduser().resolve()
        self.bundle_identity = verify_piper_build_blocks_ema_bundle(self.model_root)
        if not (self.runtime_source_root / "src/openpi").is_dir():
            raise FileNotFoundError(
                f"prepared Piper runtime source is missing: {self.runtime_source_root}"
            )
        for path in (
            self.runtime_source_root / "src",
            self.runtime_source_root / "packages/openpi-client/src",
        ):
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))
        os.environ["OPENPI_DATA_HOME"] = str(self.model_root / "runtime/assets")
        os.environ["OPENPI_PALIGEMMA_TOKENIZER_PATH"] = str(
            self.model_root / "runtime/assets/big_vision/paligemma_tokenizer.model"
        )

        from openpi.policies import policy_config
        from openpi.training import config

        train_config = config.get_config(PIPER_BUILD_BLOCKS_EMA_CONFIG_NAME)
        checkpoint_assets = self.model_root / "checkpoint/assets"
        identity_manifest = self.model_root / "runtime/piper_manifest_identity.json"
        data_factory = dataclasses.replace(
            train_config.data,
            assets=dataclasses.replace(
                train_config.data.assets,
                assets_dir=str(checkpoint_assets),
                asset_id=PIPER_BUILD_BLOCKS_EMA_DATASET_ID,
            ),
            base_config=dataclasses.replace(
                train_config.data.base_config,
                lerobot_root=None,
                video_backend=None,
                lerobot_v3_manifest=str(identity_manifest),
            ),
        )
        train_config = dataclasses.replace(
            train_config,
            data=data_factory,
            assets_base_dir=str(checkpoint_assets),
        )
        denoise_steps = int(self.model_cfg.get("num_denoise_steps") or 10)
        if denoise_steps < 1:
            raise ValueError(
                "Piper build-blocks num_denoise_steps must be positive"
            )
        self.policy = policy_config.create_trained_policy(
            train_config,
            self.model_root / "checkpoint",
            default_prompt=PIPER_BUILD_BLOCKS_EMA_PROMPT,
            sample_kwargs={"num_steps": denoise_steps},
        )
        self.model = self.policy

    def _init_piper_x_dual_desktop_ema(self) -> None:
        self._init_piper_x_dual_desktop_ema_bundle(
            action_frequency_hz=PIPER_DESKTOP_EMA_ACTION_FREQUENCY_HZ,
            action_horizon=PIPER_DESKTOP_EMA_ACTION_HORIZON,
            execute_horizon=PIPER_DESKTOP_EMA_EXECUTE_HORIZON,
            dataset_id=PIPER_DESKTOP_EMA_DATASET_ID,
            config_name=PIPER_DESKTOP_EMA_CONFIG_NAME,
            prompts=PIPER_DESKTOP_PROMPTS,
            verifier=verify_piper_desktop_ema_bundle,
        )

    def _init_piper_x_dual_desktop_h32_v3_ema(self) -> None:
        self._init_piper_x_dual_desktop_ema_bundle(
            action_frequency_hz=PIPER_DESKTOP_H32_V3_ACTION_FREQUENCY_HZ,
            action_horizon=PIPER_DESKTOP_H32_V3_ACTION_HORIZON,
            execute_horizon=PIPER_DESKTOP_H32_V3_EXECUTE_HORIZON,
            dataset_id=PIPER_DESKTOP_H32_V3_DATASET_ID,
            config_name=PIPER_DESKTOP_H32_V3_CONFIG_NAME,
            prompts=PIPER_DESKTOP_PROMPTS,
            verifier=verify_piper_desktop_h32_v3_bundle,
        )

    def _init_piper_x_dual_h32_ema(self) -> None:
        """Load any identity registered by the shared h32 EMA verifier."""
        self._init_piper_x_dual_desktop_ema_bundle(
            action_frequency_hz=30.0,
            action_horizon=32,
            execute_horizon=16,
            dataset_id=None,
            config_name=PIPER_DESKTOP_H32_V3_CONFIG_NAME,
            prompts=None,
            verifier=verify_piper_h32_ema_bundle,
        )

    def _init_piper_x_dual_paperall50_h32_v1_ema(self) -> None:
        self._init_piper_x_dual_desktop_ema_bundle(
            action_frequency_hz=PIPER_PAPERALL50_H32_V1_ACTION_FREQUENCY_HZ,
            action_horizon=PIPER_PAPERALL50_H32_V1_ACTION_HORIZON,
            execute_horizon=PIPER_PAPERALL50_H32_V1_EXECUTE_HORIZON,
            dataset_id=PIPER_PAPERALL50_H32_V1_DATASET_ID,
            config_name=PIPER_PAPERALL50_H32_V1_CONFIG_NAME,
            prompts=PIPER_PAPERALL50_H32_V1_PROMPTS,
            verifier=verify_piper_paperall50_h32_v1_bundle,
        )

    def _init_piper_x_dual_dagger50_h32_v1_ema(self) -> None:
        self._init_piper_x_dual_desktop_ema_bundle(
            action_frequency_hz=PIPER_DAGGER50_H32_V1_ACTION_FREQUENCY_HZ,
            action_horizon=PIPER_DAGGER50_H32_V1_ACTION_HORIZON,
            execute_horizon=PIPER_DAGGER50_H32_V1_EXECUTE_HORIZON,
            dataset_id=PIPER_DAGGER50_H32_V1_DATASET_ID,
            config_name=PIPER_DAGGER50_H32_V1_CONFIG_NAME,
            prompts=PIPER_DAGGER50_H32_V1_PROMPTS,
            verifier=verify_piper_dagger50_h32_v1_bundle,
        )

    def _init_piper_x_dual_pen_trash_h32_v1_ema(self) -> None:
        self._init_piper_x_dual_desktop_ema_bundle(
            action_frequency_hz=PIPER_PEN_TRASH_H32_V1_ACTION_FREQUENCY_HZ,
            action_horizon=PIPER_PEN_TRASH_H32_V1_ACTION_HORIZON,
            execute_horizon=PIPER_PEN_TRASH_H32_V1_EXECUTE_HORIZON,
            dataset_id=PIPER_PEN_TRASH_H32_V1_DATASET_ID,
            config_name=PIPER_PEN_TRASH_H32_V1_CONFIG_NAME,
            prompts=PIPER_PEN_TRASH_H32_V1_PROMPTS,
            verifier=verify_piper_pen_trash_h32_v1_bundle,
        )

    def _init_piper_x_dual_desktop_ema_bundle(
        self,
        *,
        action_frequency_hz: float,
        action_horizon: int,
        execute_horizon: int,
        dataset_id: str | None,
        config_name: str,
        prompts: tuple[str, ...] | None,
        verifier: Any,
    ) -> None:
        self.action_frequency_hz = float(
            self.model_cfg.get("action_frequency_hz") or action_frequency_hz
        )
        self.action_horizon = int(
            self.model_cfg.get("action_horizon") or action_horizon
        )
        self.desktop_ema_execute_horizon = execute_horizon
        # Generic h32 deployments bind these values to the verified bundle
        # contract below. The fallback is used only by explicit dummy-policy
        # tests, where there is no bundle identity to inspect.
        self.desktop_ema_prompts = tuple(prompts or PIPER_OBJECT50_PROMPTS)
        self.allow_dummy_policy = _is_true(
            self.model_cfg.get("allow_dummy_policy", False)
        )
        self.observation: ObservationFrame | None = None
        self.model_root: Path | None = None
        self.bundle_identity: dict[str, Any] | None = None
        if self.action_frequency_hz != action_frequency_hz:
            raise ValueError(
                "desktop EMA Piper action frequency must be exactly "
                f"{action_frequency_hz:g} Hz"
            )
        if self.action_horizon != action_horizon:
            raise ValueError(
                f"desktop EMA Piper action horizon must be exactly {action_horizon}"
            )
        if _is_true(self.model_cfg.get("rtc_enabled", False)):
            raise ValueError("PI0.5 desktop EMA Piper RTC is not implemented")
        if self.allow_dummy_policy:
            self.policy = None
            self.model = None
            return

        raw_bundle_root = self.model_cfg.get("bundle_root") or self.model_cfg.get(
            "checkpoint_path"
        )
        if not raw_bundle_root:
            raise ValueError("bundle_root is required for the desktop EMA Piper profile")
        self.model_root = Path(raw_bundle_root).expanduser().resolve()
        self.bundle_identity = verifier(self.model_root)
        verified_dataset = self.bundle_identity.get("dataset_id") or self.bundle_identity.get(
            "contract", {}
        ).get("dataset_id")
        if not isinstance(verified_dataset, str) or not verified_dataset:
            raise ValueError("dual-PiPER verifier returned no dataset identity")
        if dataset_id is None:
            dataset_id = verified_dataset
        elif verified_dataset != dataset_id:
            raise ValueError("dual-PiPER verifier returned the wrong dataset")
        verified_prompts = tuple(self.bundle_identity.get("prompts", ()))
        if not verified_prompts:
            raise ValueError("dual-PiPER verifier returned no prompt catalog")
        if prompts is None:
            self.desktop_ema_prompts = verified_prompts
        elif verified_prompts != self.desktop_ema_prompts:
            raise ValueError("dual-PiPER verifier returned the wrong prompt catalog")
        runtime_paths = (
            self.model_root / "runtime" / "project" / "src",
            self.model_root / "runtime" / "openpi" / "src",
            self.model_root
            / "runtime"
            / "openpi"
            / "packages"
            / "openpi-client"
            / "src",
            self.model_root / "runtime" / "lerobot",
        )
        for path in runtime_paths:
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))
        os.environ["OPENPI_DATA_HOME"] = str(self.model_root / "assets")
        os.environ["OPENPI_DATASET_ID"] = dataset_id

        from openpi.policies import policy_config
        from openpi.training import config

        train_config = config.get_config(config_name)
        self.policy = policy_config.create_trained_policy(
            train_config,
            self.model_root / "checkpoint",
            pytorch_device=str(self.model_cfg.get("pytorch_device") or "cuda"),
        )
        self.model = self.policy

    def _init_piper_x_dual_multitask5(self) -> None:
        self.action_frequency_hz = float(
            self.model_cfg.get("action_frequency_hz")
            or PIPER_MULTITASK_ACTION_FREQUENCY_HZ
        )
        self.action_horizon = int(
            self.model_cfg.get("action_horizon") or PIPER_MULTITASK_ACTION_HORIZON
        )
        self.allow_dummy_policy = _is_true(
            self.model_cfg.get("allow_dummy_policy", False)
        )
        self.observation: ObservationFrame | None = None
        self.model_root: Path | None = None
        self.runtime_source_root: Path | None = None
        self.bundle_identity: dict[str, Any] | None = None
        if self.action_frequency_hz != PIPER_MULTITASK_ACTION_FREQUENCY_HZ:
            raise ValueError("Piper multitask action frequency must be exactly 15 Hz")
        if self.action_horizon != PIPER_MULTITASK_ACTION_HORIZON:
            raise ValueError("Piper multitask action horizon must be exactly 16")
        if _is_true(self.model_cfg.get("rtc_enabled", False)):
            raise ValueError("PI0.5 Piper multitask RTC is not implemented")
        if self.allow_dummy_policy:
            self.policy = None
            self.model = None
            return

        raw_bundle_root = self.model_cfg.get("bundle_root") or self.model_cfg.get(
            "checkpoint_path"
        )
        raw_runtime_root = self.model_cfg.get("runtime_source_root") or os.getenv(
            "PI05_RUNTIME_SOURCE_ROOT"
        )
        if not raw_bundle_root:
            raise ValueError("bundle_root is required for the Piper multitask profile")
        if not raw_runtime_root:
            raise ValueError(
                "runtime_source_root is required; run setup_piper_multitask5.sh"
            )
        self.model_root = Path(raw_bundle_root).expanduser().resolve()
        self.runtime_source_root = Path(raw_runtime_root).expanduser().resolve()
        self.bundle_identity = verify_piper_multitask_bundle(self.model_root)
        if not (self.runtime_source_root / "src/openpi").is_dir():
            raise FileNotFoundError(
                f"prepared Piper runtime source is missing: {self.runtime_source_root}"
            )
        for path in (
            self.runtime_source_root / "src",
            self.runtime_source_root / "packages/openpi-client/src",
        ):
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))
        os.environ["OPENPI_PALIGEMMA_TOKENIZER_PATH"] = str(
            self.model_root / "tokenizer/paligemma_tokenizer.model"
        )

        from openpi.policies import policy_config
        from openpi.training import config

        train_config = config.get_config(PIPER_MULTITASK_CONFIG_NAME)
        denoise_steps = int(self.model_cfg.get("num_denoise_steps") or 10)
        if denoise_steps < 1:
            raise ValueError("Piper multitask num_denoise_steps must be positive")
        self.policy = policy_config.create_trained_policy(
            train_config,
            self.model_root / "checkpoint",
            sample_kwargs={"num_steps": denoise_steps},
        )
        self.model = self.policy

    def _init_astribot_s1(self) -> None:
        self.camera_name = str(self.model_cfg.get("camera_name") or "cam_head")
        self.action_frequency_hz = float(
            self.model_cfg.get("action_frequency_hz") or 30.0
        )
        self.dummy_horizon = int(
            self.model_cfg.get("dummy_horizon") or ASTRIBOT_ACTION_HORIZON
        )
        self.allow_dummy_policy = _is_true(
            self.model_cfg.get("allow_dummy_policy", False)
        )
        self.observation: ObservationFrame | None = None
        self.model_root: Path | None = None
        self.bundle_ticket: dict[str, Any] | None = None
        if self.action_frequency_hz != 30.0:
            raise ValueError("tomato-v3 action frequency must be exactly 30 Hz")
        if self.dummy_horizon != ASTRIBOT_ACTION_HORIZON:
            raise ValueError("tomato-v3 action horizon must be exactly 30")
        if _is_true(self.model_cfg.get("rtc_enabled", False)):
            raise ValueError("PI0.5 Astribot RTC is not implemented in XPolicyLab")
        if self.allow_dummy_policy:
            self.policy = None
            self.model = None
            return

        raw_bundle_root = self.model_cfg.get("bundle_root") or self.model_cfg.get(
            "checkpoint_path"
        )
        if not raw_bundle_root:
            raise ValueError("bundle_root is required for the Astribot tomato-v3 profile")
        self.model_root = Path(raw_bundle_root).expanduser().resolve()
        self.bundle_ticket = _verify_astribot_bundle_identity(self.model_root)
        module = _load_astribot_bundle_module(self.model_root)
        denoise_steps = int(self.model_cfg.get("num_denoise_steps") or 10)
        self.policy = module.AstribotPolicy(
            self.model_root,
            num_denoise_steps=denoise_steps,
        )
        self.model = self.policy

    def _init_gr3(self) -> None:
        self.camera_name = str(self.model_cfg.get("camera_name") or "cam_head")
        self.action_frequency_hz = float(
            self.model_cfg.get("action_frequency_hz") or 30.0
        )
        self.dummy_horizon = int(self.model_cfg.get("dummy_horizon") or 50)
        self.allow_dummy_policy = _is_true(
            self.model_cfg.get("allow_dummy_policy", False)
        )
        self.observation: ObservationFrame | None = None
        self.model_root: Path | None = None
        if self.action_frequency_hz <= 0 or self.dummy_horizon < 1:
            raise ValueError("action_frequency_hz and dummy_horizon must be positive")
        if _is_true(self.model_cfg.get("rtc_enabled", False)):
            raise ValueError("PI0.5 GR3 RTC is not implemented")
        if self.allow_dummy_policy:
            self.policy = None
            return

        policy_root = Path(
            self.model_cfg.get("pi05_policy_root") or _POLICY_DIR / "openpi"
        ).expanduser().resolve()
        if not (policy_root / "src" / "openpi").is_dir():
            raise FileNotFoundError(f"PI0.5 policy root not found: {policy_root}")
        for path in (policy_root, policy_root / "src"):
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))

        from openpi.policies import policy_config
        from openpi.training import config

        self.model_root = _resolve_pi05_model_root(self.model_cfg)
        if not (self.model_root / "params").exists():
            raise FileNotFoundError(f"PI0.5 checkpoint not found: {self.model_root}")
        train_config = config.get_config(
            self.model_cfg.get("train_config_name", "pi05_anygrasp")
        )
        self.policy = policy_config.create_trained_policy(
            train_config, str(self.model_root)
        )
        self.model = self.policy

    def get_model(self, model_cfg: dict[str, Any]):
        from openpi.policies import policy_config
        from openpi.shared import normalize
        from openpi.training import config as training_config

        train_config_name = model_cfg.get("train_config_name", "pi05_aloha")
        repo_id = model_cfg.get("repo_id", "1118")
        model_root = _resolve_pi05_model_root(model_cfg)

        config = training_config.get_config(train_config_name)
        norm_stats = None
        if repo_id is not None:
            norm_stats = normalize.load(model_root / "assets" / str(repo_id))

        return policy_config.create_trained_policy(config, str(model_root), norm_stats=norm_stats)

    def update_obs(self, obs):
        if self.runtime_profile in REAL_WORLD_PROFILES:
            self.observation = ObservationFrame.from_policy_observation(obs)
            return
        self.update_obs_batch([obs])

    def update_obs_batch(self, obs_list):
        if self.runtime_profile in REAL_WORLD_PROFILES:
            raise NotImplementedError(
                "PI0.5 real-world profiles support one robot per server"
            )
        self._latest_env_idx_list = [obs.get("env_idx", index) for index, obs in enumerate(obs_list)]
        encoded_obs_list = [
            encode_obs(obs, self.action_type, self.robot_action_dim_info) for obs in obs_list
        ]
        self.observation_window = stack_obs(encoded_obs_list)

    def get_action(self, **kwargs):
        if self.runtime_profile == PIPER_X_DUAL_BUILD_BLOCKS_EMA_PROFILE:
            if self.observation is None:
                raise RuntimeError("get_action called before update_obs")
            encoded = encode_piper_build_blocks_ema_observation(self.observation)
            if self.policy is None:
                values = np.zeros(
                    (
                        PIPER_BUILD_BLOCKS_EMA_ACTION_HORIZON,
                        PIPER_X_DUAL_ACTION_SCHEMA.dim,
                    ),
                    dtype=np.float32,
                )
            else:
                result = self.policy.infer(encoded)
                values = np.asarray(result["actions"], dtype=np.float32)
            expected_shape = (
                PIPER_BUILD_BLOCKS_EMA_ACTION_HORIZON,
                PIPER_X_DUAL_ACTION_SCHEMA.dim,
            )
            if values.shape != expected_shape or not np.isfinite(values).all():
                raise ValueError(
                    "PI0.5 Piper build-blocks EMA action must be finite "
                    f"{expected_shape}, got {values.shape}"
                )
            step = (
                self.bundle_identity["step"]
                if self.bundle_identity is not None
                else None
            )
            return ActionChunk(
                schema=PIPER_X_DUAL_ACTION_SCHEMA,
                values=values,
                frequency_hz=self.action_frequency_hz,
                metadata={
                    "policy": "Pi_05",
                    "runtime_profile": PIPER_X_DUAL_BUILD_BLOCKS_EMA_PROFILE,
                    "checkpoint": str(self.model_root or "dummy"),
                    "checkpoint_step": step,
                    "params_semantics": "ema",
                    "execute_prefix_steps": PIPER_BUILD_BLOCKS_EMA_EXECUTE_HORIZON,
                    "prompt": self.observation.instruction,
                    "cameras": ["camera_head", "camera_left", "camera_right"],
                    "model_action_semantics": (
                        "delta_joint_12_plus_absolute_gripper_width_2"
                    ),
                    "output_action_semantics": (
                        "absolute_joint_12_plus_absolute_gripper_width_2"
                    ),
                    "rtc": {"enabled": False},
                },
            ).to_policy_result()
        if self.runtime_profile in {
            PIPER_X_DUAL_DESKTOP_EMA_PROFILE,
            PIPER_X_DUAL_DESKTOP_H32_V3_EMA_PROFILE,
            PIPER_X_DUAL_H32_EMA_PROFILE,
            PIPER_X_DUAL_PAPERALL50_H32_V1_EMA_PROFILE,
            PIPER_X_DUAL_DAGGER50_H32_V1_EMA_PROFILE,
            PIPER_X_DUAL_PEN_TRASH_H32_V1_EMA_PROFILE,
        }:
            if self.observation is None:
                raise RuntimeError("get_action called before update_obs")
            encoded = encode_piper_desktop_observation(
                self.observation,
                self.desktop_ema_prompts,
            )
            if self.policy is None:
                values = np.zeros(
                    (
                        self.action_horizon,
                        PIPER_X_DUAL_ACTION_SCHEMA.dim,
                    ),
                    dtype=np.float32,
                )
            else:
                result = self.policy.infer(encoded)
                values = np.asarray(result["actions"], dtype=np.float32)
            expected_shape = (
                self.action_horizon,
                PIPER_X_DUAL_ACTION_SCHEMA.dim,
            )
            if values.shape != expected_shape or not np.isfinite(values).all():
                raise ValueError(
                    f"PI0.5 desktop EMA Piper action must be finite "
                    f"{expected_shape}, got {values.shape}"
                )
            step = (
                self.bundle_identity["step"]
                if self.bundle_identity is not None
                else None
            )
            return ActionChunk(
                schema=PIPER_X_DUAL_ACTION_SCHEMA,
                values=values,
                frequency_hz=self.action_frequency_hz,
                metadata={
                    "policy": "Pi_05",
                    "runtime_profile": self.runtime_profile,
                    "checkpoint": str(self.model_root or "dummy"),
                    "checkpoint_step": step,
                    "params_semantics": "ema",
                    "execute_prefix_steps": self.desktop_ema_execute_horizon,
                    "prompt": self.observation.instruction,
                    "cameras": ["camera_head", "camera_left", "camera_right"],
                    "output_action_semantics": (
                        "absolute_joint_12_plus_absolute_gripper_width_2"
                    ),
                    "rtc": {"enabled": False},
                },
            ).to_policy_result()
        if self.runtime_profile == PIPER_X_DUAL_MULTITASK5_PROFILE:
            if self.observation is None:
                raise RuntimeError("get_action called before update_obs")
            encoded = encode_piper_multitask_observation(self.observation)
            if self.policy is None:
                values = np.zeros(
                    (
                        PIPER_MULTITASK_ACTION_HORIZON,
                        PIPER_X_DUAL_ACTION_SCHEMA.dim,
                    ),
                    dtype=np.float32,
                )
            else:
                result = self.policy.infer(encoded)
                values = np.asarray(result["actions"], dtype=np.float32)
            expected_shape = (
                PIPER_MULTITASK_ACTION_HORIZON,
                PIPER_X_DUAL_ACTION_SCHEMA.dim,
            )
            if values.shape != expected_shape or not np.isfinite(values).all():
                raise ValueError(
                    f"PI0.5 Piper multitask action must be finite "
                    f"{expected_shape}, got {values.shape}"
                )
            return ActionChunk(
                schema=PIPER_X_DUAL_ACTION_SCHEMA,
                values=values,
                frequency_hz=self.action_frequency_hz,
                metadata={
                    "policy": "Pi_05",
                    "runtime_profile": PIPER_X_DUAL_MULTITASK5_PROFILE,
                    "checkpoint": str(self.model_root or "dummy"),
                    "checkpoint_step": 20_000,
                    "params_semantics": "ema",
                    "execute_prefix_steps": PIPER_MULTITASK_EXECUTE_HORIZON,
                    "prompt": self.observation.instruction,
                    "cameras": ["camera_head", "camera_left", "camera_right"],
                    "output_action_semantics": (
                        "absolute_joint_12_plus_absolute_gripper_width_2"
                    ),
                    "rtc": {"enabled": False},
                },
            ).to_policy_result()
        if self.runtime_profile == PIPER_X_DUAL_DESKTOP_CLEANUP_PROFILE:
            if self.observation is None:
                raise RuntimeError("get_action called before update_obs")
            encoded = encode_piper_desktop_observation(self.observation)
            if self.policy is None:
                values = np.zeros(
                    (
                        PIPER_DESKTOP_ACTION_HORIZON,
                        PIPER_X_DUAL_ACTION_SCHEMA.dim,
                    ),
                    dtype=np.float32,
                )
            else:
                result = self.policy.infer(encoded)
                values = np.asarray(result["actions"], dtype=np.float32)
            expected_shape = (
                PIPER_DESKTOP_ACTION_HORIZON,
                PIPER_X_DUAL_ACTION_SCHEMA.dim,
            )
            if values.shape != expected_shape or not np.isfinite(values).all():
                raise ValueError(
                    f"PI0.5 desktop-cleanup Piper action must be finite "
                    f"{expected_shape}, got {values.shape}"
                )
            step = (
                self.bundle_identity["contract"]["step"]
                if self.bundle_identity is not None
                else None
            )
            return ActionChunk(
                schema=PIPER_X_DUAL_ACTION_SCHEMA,
                values=values,
                frequency_hz=self.action_frequency_hz,
                metadata={
                    "policy": "Pi_05",
                    "runtime_profile": PIPER_X_DUAL_DESKTOP_CLEANUP_PROFILE,
                    "checkpoint": str(self.model_root or "dummy"),
                    "checkpoint_step": step,
                    "params_semantics": "raw",
                    "execute_prefix_steps": PIPER_DESKTOP_EXECUTE_PREFIX_STEPS,
                    "prompt": self.observation.instruction,
                    "cameras": ["camera_head", "camera_left", "camera_right"],
                    "output_action_semantics": (
                        "absolute_joint_12_plus_absolute_gripper_width_2"
                    ),
                    "rtc": {"enabled": False},
                },
            ).to_policy_result()
        if self.runtime_profile == PIPER_X_DUAL_PROFILE:
            if self.observation is None:
                raise RuntimeError("get_action called before update_obs")
            if self.policy is None:
                raise RuntimeError("Piper dummy profile requires an injected no-send test policy")
            result = self.policy.infer(encode_piper_observation(self.observation))
            values = np.asarray(result["actions"], dtype=np.float32)
            expected_shape = (PIPER_ACTION_HORIZON, PIPER_X_DUAL_ACTION_SCHEMA.dim)
            if values.shape != expected_shape or not np.isfinite(values).all():
                raise ValueError(f"PI0.5 Piper action must be finite {expected_shape}, got {values.shape}")
            return ActionChunk(
                schema=PIPER_X_DUAL_ACTION_SCHEMA,
                values=values,
                frequency_hz=self.action_frequency_hz,
                metadata={
                    "policy": "Pi_05",
                    "runtime_profile": PIPER_X_DUAL_PROFILE,
                    "checkpoint": str(self.model_root),
                    "prompt": PIPER_BUILD_BLOCKS_PROMPT,
                    "cameras": ["camera_head", "camera_left", "camera_right"],
                    "model_action_semantics": "delta_joint_12_plus_absolute_gripper_width_2",
                    "output_action_semantics": "absolute_joint_12_plus_absolute_gripper_width_2",
                    "rtc": {"enabled": False},
                },
            ).to_policy_result()
        if self.runtime_profile == ASTRIBOT_S1_PROFILE:
            if self.observation is None:
                raise RuntimeError("get_action called before update_obs")
            encoded = encode_astribot_observation(
                self.observation,
                self.camera_name,
            )
            if self.policy is None:
                values = np.zeros(
                    (ASTRIBOT_ACTION_HORIZON, ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.dim),
                    dtype=np.float32,
                )
                timing = {}
            else:
                # XPolicyLab's chassis contract is velocity-based. Zero is the
                # only valid hold target; never reuse observed chassis pose as
                # a velocity command.
                values, timing = self.policy.infer(
                    **encoded,
                    base_hold_target=np.zeros(3, dtype=np.float32),
                )
                values = np.asarray(values, dtype=np.float32)
            expected_shape = (
                ASTRIBOT_ACTION_HORIZON,
                ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.dim,
            )
            if values.shape != expected_shape or not np.isfinite(values).all():
                raise ValueError(
                    f"PI0.5 Astribot action must be finite {expected_shape}, "
                    f"got {values.shape}"
                )
            if not np.array_equal(values[:, :3], np.zeros_like(values[:, :3])):
                raise ValueError("PI0.5 Astribot chassis velocity hold must be exactly zero")
            return ActionChunk(
                schema=ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
                values=values,
                frequency_hz=self.action_frequency_hz,
                metadata={
                    "policy": "Pi_05",
                    "runtime_profile": ASTRIBOT_S1_PROFILE,
                    "checkpoint": str(self.model_root or "dummy"),
                    "checkpoint_step": 20000,
                    "params_semantics": "ema",
                    "prompt": ASTRIBOT_TOMATO_PROMPT,
                    "camera": self.camera_name,
                    "ignored_cameras": ["cam_left_wrist", "cam_right_wrist"],
                    "policy_timing": timing,
                    "chassis_velocity_hold": [0.0, 0.0, 0.0],
                    "rtc": {"enabled": False},
                },
            ).to_policy_result()
        if self.runtime_profile == GR3_PROFILE:
            if self.observation is None:
                raise RuntimeError("get_action called before update_obs")
            if self.policy is None:
                values = np.zeros(
                    (self.dummy_horizon, GR3_ACTION_SCHEMA.dim), dtype=np.float32
                )
            else:
                result = self.policy.infer(
                    encode_gr3_observation(self.observation, self.camera_name)
                )
                values = scatter_gr3_actions(result["actions"])
            return ActionChunk(
                schema=GR3_ACTION_SCHEMA,
                values=values,
                frequency_hz=self.action_frequency_hz,
                metadata={
                    "policy": "Pi_05",
                    "runtime_profile": GR3_PROFILE,
                    "checkpoint": str(self.model_root or "dummy"),
                    "rtc": {"enabled": False},
                },
            ).to_policy_result()
        action_list = self.get_action_batch(env_idx_list=[self._latest_env_idx_list[0]], **kwargs)
        return action_list[0]

    def get_action_batch(self, env_idx_list=None, **kwargs):
        if self.runtime_profile in REAL_WORLD_PROFILES:
            raise NotImplementedError(
                "PI0.5 real-world profiles do not support batches"
            )
        if self.observation_window is None:
            raise AssertionError("update_obs or update_obs_batch first!")

        env_idx_list = env_idx_list or self._latest_env_idx_list
        # actions = self.policy.infer(self.observation_window, **kwargs)["actions"]
        action_list = []

        for batch_index, _ in enumerate(env_idx_list):
            single_observation = slice_stacked_obs(self.observation_window, batch_index)
            actions = self.policy.infer(single_observation, **kwargs)["actions"]
            if self.robot_action_dim_info is None:
                action_list.append(actions)
            else:
                action_list.append(
                    unpack_robot_state(
                        actions,
                        self.action_type,
                        self.robot_action_dim_info,
                        source_type="obs",
                    )
                )

        return action_list

    def reset(self):
        if self.runtime_profile in REAL_WORLD_PROFILES:
            self.observation = None
            return
        self.observation_window = None
        self._latest_env_idx_list = [0]

    def reset_obsrvationwindows(self):
        self.reset()


def encode_obs(observation, action_type, robot_action_dim_info):
    if "images" in observation and "state" in observation:
        state = np.asarray(observation["state"], dtype=np.float32)
        images = {
            "cam_high": ensure_chw_uint8(observation["images"]["cam_high"]),
            "cam_left_wrist": ensure_chw_uint8(observation["images"]["cam_left_wrist"]),
            "cam_right_wrist": ensure_chw_uint8(observation["images"]["cam_right_wrist"]),
        }
        prompt = observation.get("instruction")
        return {"state": state, "images": images, "prompt": prompt}

    if robot_action_dim_info is None:
        raise ValueError("env_cfg_type is required when encoding raw environment observations.")

    images = {
        "cam_high": ensure_chw_uint8(extract_image(observation, ["cam_high", "cam_head", "head_camera", "top_camera"])),
        "cam_left_wrist": ensure_chw_uint8(
            extract_image(observation, ["cam_left_wrist", "left_camera", "left_wrist", "wrist_left"])
        ),
        "cam_right_wrist": ensure_chw_uint8(
            extract_image(observation, ["cam_right_wrist", "right_camera", "right_wrist", "wrist_right"])
        ),
    }
    state = pack_robot_state(observation, action_type, robot_action_dim_info, source_type="obs").astype(np.float32)
    prompt = observation.get("instruction")
    return {"state": state, "images": images, "prompt": prompt}


def stack_obs(obs_list: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "state": np.stack([obs["state"] for obs in obs_list], axis=0),
        "images": {
            "cam_high": np.stack([obs["images"]["cam_high"] for obs in obs_list], axis=0),
            "cam_left_wrist": np.stack([obs["images"]["cam_left_wrist"] for obs in obs_list], axis=0),
            "cam_right_wrist": np.stack([obs["images"]["cam_right_wrist"] for obs in obs_list], axis=0),
        },
        "prompt": [obs["prompt"] for obs in obs_list],
    }


def slice_stacked_obs(obs: dict[str, Any], batch_index: int) -> dict[str, Any]:
    return {
        "state": obs["state"][batch_index],
        "images": {
            "cam_high": obs["images"]["cam_high"][batch_index],
            "cam_left_wrist": obs["images"]["cam_left_wrist"][batch_index],
            "cam_right_wrist": obs["images"]["cam_right_wrist"][batch_index],
        },
        "prompt": obs["prompt"][batch_index],
    }


def extract_image(observation, candidate_names):
    vision = observation.get("vision", {})
    for candidate_name in candidate_names:
        if candidate_name not in vision:
            continue
        image = vision[candidate_name]
        if isinstance(image, dict):
            for image_key in ("color", "rgb"):
                if image_key in image:
                    return image[image_key]
        else:
            return image
    raise KeyError(f"Could not find any image for candidates: {candidate_names}")


def ensure_chw_uint8(image):
    if isinstance(image, (bytes, bytearray, memoryview)):
        image = decode_compressed_image(np.frombuffer(bytes(image), dtype=np.uint8))

    image = np.asarray(image)

    if image.ndim == 1 and image.dtype == np.uint8:
        image = decode_compressed_image(image)

    if image.ndim != 3:
        raise ValueError(f"Expected image ndim=3, got shape {image.shape}")

    if np.issubdtype(image.dtype, np.floating):
        image = np.clip(image, 0.0, 1.0)
        image = (image * 255.0).astype(np.uint8)
    elif image.dtype != np.uint8:
        image = image.astype(np.uint8)

    if image.shape[-1] in (1, 3):
        image_hwc = image
    elif image.shape[0] in (1, 3):
        image_hwc = np.transpose(image, (1, 2, 0))
    else:
        raise ValueError(f"Unsupported image shape: {image.shape}")

    return np.transpose(image_hwc, (2, 0, 1))


def decode_compressed_image(image_buffer):
    return decode_image_bit(image_buffer)
