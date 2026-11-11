"""Pinned Fast-FoundationStereo PyTorch adapter for *rectified* saved stereo pairs.

Importing this module needs NumPy, never Torch, CUDA, RealSense or robot SDKs.
There is no device capture or action interface. Use a dedicated worker process:
the upstream serialized model imports generic modules named ``core`` / ``Utils``.
The manifest is a provenance receipt; hashes must be obtained from the same
official downloaded model/config pair, not invented or borrowed from a model.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time
from typing import Any

import numpy as np

UPSTREAM_URL = "https://github.com/NVlabs/Fast-FoundationStereo"
UPSTREAM_COMMIT = "476f4249561f7c79ca707326954f9255643412a6"
DEFAULT_MODEL_ID = "20-30-48"
MODEL_FOLDER_URL = "https://drive.google.com/drive/folders/1WWT0I8VB1k7cA2LqnjhUHOux8Sz-BZMw"
CHECKPOINT_FILE_ID = "1ZsXw3WwBmtNdv1lqdEJ95kyvf7J4YusE"
CONFIG_FILE_ID = "1DvQrzvR46fsh0dVVbvwdQlSmvO8sVf69"
CONFIG_SHA256 = "d45afe99b176454d5aff416edf16c8da6a99579f8f374b927f37907442a7d6bc"
CHECKPOINT_SHA256 = "98b5a9acf39fbfa795025de8cea95ce123daa40f6b6234d719167751024cf692"
MANIFEST_SCHEMA = "junqi.piper.ffs_model_manifest.v1"


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_serialized_config(model_args: Any, cfg: dict[str, Any],
                              identity: dict[str, Any]) -> list[dict[str, Any]]:
    """Check paired config, preserving one proven upstream serialization omission.

    The official 20-30-48 pickle omits ``slow_fast_gru`` although its cfg has
    false. The pinned upstream implementation never reads that field, and its
    own demo does not inject it. Do not generalize this exception or mutate args.
    """
    exceptions = []
    for key, value in cfg.items():
        if key in ("valid_iters", "max_disp"):
            continue  # Explicit runtime overrides also used by official demo.
        actual = model_args.get(key)
        if (key == "slow_fast_gru" and key not in model_args and value is False
                and identity.get("checkpoint_sha256") == CHECKPOINT_SHA256
                and identity.get("config_sha256") == CONFIG_SHA256
                and identity.get("upstream_commit") == UPSTREAM_COMMIT
                and identity.get("model_id") == DEFAULT_MODEL_ID):
            exceptions.append({"key": key, "config_value": False,
                               "serialized_status": "absent",
                               "reason": "known official pickle omission; unused in pinned upstream",
                               "model_args_modified": False})
            continue
        if isinstance(value, list) and actual is not None:
            actual = list(actual)
        _require(actual == value, f"serialized model/config disagree for {key}")
    return exceptions


@dataclass(frozen=True)
class ModelManifest:
    """Verified assets. Paths may be absolute or relative to the manifest file."""
    upstream_dir: Path
    checkpoint: Path
    config: Path
    checkpoint_sha256: str
    config_sha256: str
    model_id: str = DEFAULT_MODEL_ID

    @classmethod
    def read(cls, path: str | Path) -> "ModelManifest":
        path = Path(path).resolve()
        data = json.loads(path.read_text())
        _require(isinstance(data, dict) and data.get("schema") == MANIFEST_SCHEMA,
                 "unsupported FFS manifest schema")
        _require(data.get("upstream_commit") == UPSTREAM_COMMIT,
                 "FFS upstream commit differs from the audited commit")
        _require(data.get("model_id") == DEFAULT_MODEL_ID,
                 "this adapter currently pins the official 20-30-48 model")
        _require(data.get("source_model_folder_url") == MODEL_FOLDER_URL,
                 "manifest must identify the audited official model folder")
        paths = {}
        for name in ("upstream_dir", "checkpoint", "config"):
            value = data.get(name)
            _require(isinstance(value, str) and bool(value), f"missing {name}")
            paths[name] = (path.parent / value).resolve()
        for key in ("checkpoint_sha256", "config_sha256"):
            _require(isinstance(data.get(key), str)
                     and re.fullmatch("[0-9a-f]{64}", data[key]) is not None,
                     f"{key} must be an actual lowercase SHA-256, not a placeholder")
        _require(data["config_sha256"] == CONFIG_SHA256,
                 "config SHA differs from the audited official 20-30-48 cfg.yaml")
        _require(paths["checkpoint"].name == "model_best_bp2_serialize.pth"
                 and paths["config"].name == "cfg.yaml"
                 and paths["checkpoint"].parent == paths["config"].parent,
                 "official checkpoint and cfg.yaml must be in the same directory")
        return cls(**paths, checkpoint_sha256=data["checkpoint_sha256"],
                   config_sha256=data["config_sha256"])

    def verify(self) -> dict[str, Any]:
        """Hash assets and check exact clean tracked upstream, before importing it."""
        for path, expected in ((self.checkpoint, self.checkpoint_sha256),
                               (self.config, self.config_sha256)):
            _require(path.is_file(), f"missing model asset: {path}")
            _require(sha256_file(path) == expected, f"SHA-256 mismatch: {path.name}")
        def git(*args: str) -> str:
            return subprocess.check_output(["git", "-C", str(self.upstream_dir), *args],
                                           text=True).strip()
        _require(git("rev-parse", "HEAD") == UPSTREAM_COMMIT,
                 "upstream checkout HEAD differs from audited commit")
        _require(not git("status", "--porcelain", "--untracked-files=no"),
                 "upstream tracked files are modified")
        return {"schema": MANIFEST_SCHEMA, "model_id": self.model_id,
                "upstream_url": UPSTREAM_URL, "upstream_commit": UPSTREAM_COMMIT,
                "source_model_folder_url": MODEL_FOLDER_URL,
                "checkpoint_sha256": self.checkpoint_sha256,
                "config_sha256": self.config_sha256,
                "hashes_verified": True}


def prepare_stereo_pair(left: Any, right: Any, *, input_scale: float = 1.0
                        ) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Accept RGB uint8 or monochrome uint8, never BGR/normalized float arrays.

    Rectification is the caller's responsibility. Both arrays must be the same
    sensor exposure pair and the same geometry; this adapter has no timestamps.
    """
    _require(isinstance(input_scale, (int, float)) and not isinstance(input_scale, bool)
             and math.isfinite(input_scale) and 0 < input_scale <= 1,
             "input_scale must lie in (0,1]")
    images = []
    for image in (left, right):
        array = np.asarray(image)
        _require(array.dtype == np.uint8, "stereo input must be uint8 0-255")
        _require(array.ndim == 2 or (array.ndim == 3 and array.shape[2] in (1, 3)),
                 "stereo input must be monochrome HxW/HxWx1 or RGB HxWx3")
        _require(min(array.shape[:2]) >= 32, "stereo image dimensions must be >=32")
        if array.ndim == 2:
            array = array[:, :, None]
        if array.shape[2] == 1:
            array = np.repeat(array, 3, axis=2)
        images.append(np.array(array, copy=True, order="C"))
    _require(images[0].shape == images[1].shape,
             "rectified stereo images must have identical sizes")
    h, w = images[0].shape[:2]
    target = (int(round(w * input_scale)), int(round(h * input_scale)))
    _require(min(target) >= 32, "scaled dimensions must be >=32")
    if target != (w, h):
        import cv2
        images = [cv2.resize(item, target, interpolation=cv2.INTER_LINEAR) for item in images]
    return images[0], images[1], {
        "source_shape_hw": [h, w], "inference_shape_hw": [target[1], target[0]],
        "scale_xy": [target[0] / w, target[1] / h], "input_scale_requested": input_scale,
        "input_order": "RGB", "input_range": [0, 255],
        "normalization": "inside upstream PyTorch forward; not pre-normalized",
        "rectification": "required_from_caller", "gray_conversion": "repeat_channel_three_times",
    }


def restore_disparity_resolution(disparity: Any, image_metadata: dict[str, Any]
                                 ) -> np.ndarray:
    """Restore disparity to source size AND source pixel units after input resize."""
    array = np.asarray(disparity)
    target_hw = tuple(image_metadata["inference_shape_hw"])
    _require(array.shape == target_hw and array.dtype.kind == "f",
             "un-padded disparity must be a floating HxW map at inference resolution")
    source_h, source_w = image_metadata["source_shape_hw"]
    array = np.array(array, dtype=np.float32, copy=True)
    if target_hw != (source_h, source_w):
        import cv2
        array = cv2.resize(array, (source_w, source_h), interpolation=cv2.INTER_LINEAR)
        array /= image_metadata["scale_xy"][0]
    # Preserve NaN/Inf/nonpositive estimates for downstream validity filtering.
    return np.ascontiguousarray(array)


class FastFoundationStereoRunner:
    """Lazy CUDA PyTorch inference, explicitly requiring trusted pickle loading.

    ``infer_rectified`` returns disparity in *original input* left-camera pixels,
    d = x_left - x_right. It does not produce metric depth, confidence, alignment
    to RGB, camera extrinsics, TCP, or hardware targets. The finite-positive mask
    is numerical validity only, not a learned confidence/occlusion mask.
    """
    def __init__(self, manifest_path: str | Path, *, valid_iters: int = 4,
                 max_disp: int = 192, input_scale: float = 1.0,
                 allow_trusted_pickle: bool = False):
        _require(type(valid_iters) is int and 1 <= valid_iters <= 32,
                 "valid_iters must be an integer in [1,32]")
        _require(type(max_disp) is int and max_disp >= 32 and max_disp % 32 == 0,
                 "max_disp must be a positive multiple of 32, >=32")
        _require(isinstance(input_scale, (int, float)) and not isinstance(input_scale, bool)
                 and math.isfinite(input_scale) and 0 < input_scale <= 1,
                 "input_scale must lie in (0,1]")
        _require(type(allow_trusted_pickle) is bool, "allow_trusted_pickle must be bool")
        self.manifest = ModelManifest.read(manifest_path)
        self.valid_iters, self.max_disp, self.input_scale = valid_iters, max_disp, input_scale
        self.allow_trusted_pickle = allow_trusted_pickle
        self._model = self._torch = self._padder_type = None
        self._identity = None

    def _load(self) -> None:
        if self._model is not None:
            return
        _require(self.allow_trusted_pickle,
                 "official serialized checkpoint uses pickle; explicit allow_trusted_pickle=True required")
        identity = self.manifest.verify()
        import yaml
        cfg = yaml.safe_load(self.manifest.config.read_text())
        _require(isinstance(cfg, dict), "cfg.yaml must be a mapping")
        root = self.manifest.upstream_dir.resolve()
        for name, module in list(sys.modules.items()):
            if name == "Utils" or name == "core" or name.startswith("core."):
                origin = getattr(module, "__file__", None)
                _require(origin is not None and Path(origin).resolve().is_relative_to(root),
                         f"conflicting upstream module {name}; run in a dedicated FFS process")
        # Verified official source must precede any unrelated `core` package.
        sys.path.insert(0, str(root))
        torch = importlib.import_module("torch")
        _require(torch.cuda.is_available(), "FFS requires a CUDA target; CPU inference is not prepared")
        # Registers upstream backward-compatibility aliases before unpickling.
        importlib.import_module("core.foundation_stereo")
        padder_type = importlib.import_module("core.utils.utils").InputPadder
        model = torch.load(str(self.manifest.checkpoint), map_location="cpu", weights_only=False)
        _require(callable(getattr(model, "forward", None)) and hasattr(model, "args"),
                 "checkpoint is not the official serialized FFS model")
        config_compatibility = _verify_serialized_config(model.args, cfg, identity)
        # Official demo loads cfg.yaml but only changes these runtime model args.
        model.args.valid_iters = self.valid_iters
        model.args.max_disp = self.max_disp
        model.cuda().eval()
        self._torch, self._padder_type, self._model = torch, padder_type, model
        self._identity = {**identity, "torch_version": str(torch.__version__),
                          "torch_cuda_version": str(torch.version.cuda),
                          "device_name": torch.cuda.get_device_name(),
                          "device_capability": list(torch.cuda.get_device_capability()),
                          "backend": "pytorch", "amp_dtype": "float16",
                          "valid_iters": self.valid_iters, "max_disp": self.max_disp,
                          "max_disp_unit": "inference_resolution_pixels",
                          "optimize_build_volume": "pytorch1",
                          "serialized_config_compatibility": config_compatibility}

    def infer_rectified(self, left: Any, right: Any) -> dict[str, Any]:
        image_left, image_right, metadata = prepare_stereo_pair(left, right, input_scale=self.input_scale)
        self._load()
        torch = self._torch
        tensor_left = torch.as_tensor(image_left).cuda().float()[None].permute(0, 3, 1, 2)
        tensor_right = torch.as_tensor(image_right).cuda().float()[None].permute(0, 3, 1, 2)
        padder = self._padder_type(tensor_left.shape, divis_by=32, force_square=False)
        tensor_left, tensor_right = padder.pad(tensor_left, tensor_right)
        torch.cuda.synchronize()
        start = time.perf_counter()
        with torch.inference_mode(), torch.amp.autocast("cuda", enabled=True, dtype=torch.float16):
            disparity = self._model.forward(tensor_left, tensor_right, iters=self.valid_iters,
                                            test_mode=True, optimize_build_volume="pytorch1")
        disparity = padder.unpad(disparity.float())
        expected_shape = (1, 1, *image_left.shape[:2])
        _require(tuple(disparity.shape) == expected_shape,
                 f"unexpected upstream disparity shape {tuple(disparity.shape)} != {expected_shape}")
        output = disparity.detach().cpu().numpy()[0, 0]
        elapsed_ms = (time.perf_counter() - start) * 1000
        output = restore_disparity_resolution(output, metadata)
        valid = np.isfinite(output) & (output > 0)
        return {"disparity_px": output, "finite_positive_mask": valid,
                "metadata": {**metadata, **self._identity,
                             "forward_and_copy_ms": elapsed_ms,
                             "padded_shape_hw": list(tensor_left.shape[-2:]),
                             "disparity_reference": "left_rectified_optical",
                             "disparity_definition": "x_left - x_right",
                             "disparity_unit": "original_input_pixels",
                             "finite_positive_fraction": float(valid.mean()),
                             "confidence_available": False,
                             "hardware_access": False, "hardware_dispatch_authorized": False}}
