"""Offline adapter contracts only; no Torch, model weights, CUDA or devices."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from real_robot.piper.ffs_runtime import (
    CONFIG_SHA256, DEFAULT_MODEL_ID, MANIFEST_SCHEMA, MODEL_FOLDER_URL,
    UPSTREAM_COMMIT, FastFoundationStereoRunner, ModelManifest,
    prepare_stereo_pair, restore_disparity_resolution,
)


CONFIG = ("corr_levels: 2\ncorr_radius: 4\nhidden_dims:\n- 128\nlow_memory: 0\n"
          "max_disp: 416\nmixed_precision: true\nn_downsample: 2\nn_gru_layers: 1\n"
          "slow_fast_gru: false\nvalid_iters: 8\nvit_size: vitl\n")


@pytest.fixture
def bundle(tmp_path):
    folder = tmp_path / DEFAULT_MODEL_ID
    folder.mkdir()
    checkpoint = folder / "model_best_bp2_serialize.pth"
    checkpoint.write_bytes(b"OFFLINE TEST DUMMY, NOT A PICKLE")
    config = folder / "cfg.yaml"
    config.write_text(CONFIG)
    assert hashlib.sha256(config.read_bytes()).hexdigest() == CONFIG_SHA256
    data = {"schema": MANIFEST_SCHEMA, "upstream_commit": UPSTREAM_COMMIT,
            "upstream_dir": "upstream", "model_id": DEFAULT_MODEL_ID,
            "checkpoint": str(checkpoint.relative_to(tmp_path)),
            "config": str(config.relative_to(tmp_path)),
            "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
            "config_sha256": CONFIG_SHA256, "source_model_folder_url": MODEL_FOLDER_URL}
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(data))
    return manifest, data


def test_import_does_not_import_torch_or_robot_sdk():
    package_root = str(Path(__file__).resolve().parents[3])
    code = (f"import sys; sys.path.insert(0, {package_root!r}); "
            "import real_robot.piper.ffs_runtime; "
            "assert 'torch' not in sys.modules; "
            "assert 'pyrealsense2' not in sys.modules; "
            "assert not any('piper_sdk' in m for m in sys.modules)")
    subprocess.run([sys.executable, "-c", code], check=True)


def test_grayscale_replication_preserves_raw_intensity_and_inputs():
    left = np.arange(64 * 96, dtype=np.uint8).reshape(64, 96)
    before = left.copy()
    a, b, meta = prepare_stereo_pair(left, left[:, :, None])
    assert a.shape == b.shape == (64, 96, 3)
    assert a.flags.c_contiguous and b.flags.c_contiguous
    for channel in range(3):
        np.testing.assert_array_equal(a[:, :, channel], before)
    a[:] = 0
    np.testing.assert_array_equal(left, before)
    assert meta["input_range"] == [0, 255]
    assert meta["scale_xy"] == [1.0, 1.0]


def test_rgb_channels_are_not_reversed():
    pair = np.broadcast_to(np.array([7, 100, 240], dtype=np.uint8), (64, 64, 3))
    left, right, _ = prepare_stereo_pair(pair, pair)
    np.testing.assert_array_equal(left[0, 0], [7, 100, 240])
    np.testing.assert_array_equal(right[0, 0], [7, 100, 240])


@pytest.mark.parametrize("bad", [np.zeros((64, 64), np.float32),
                                 np.zeros((64, 64), np.uint16),
                                 np.zeros((64, 64, 4), np.uint8),
                                 np.zeros((1, 3, 64, 64), np.uint8),
                                 np.zeros((1, 64), np.uint8)])
def test_input_rejects_ambiguous_or_wrong_units(bad):
    with pytest.raises(ValueError):
        prepare_stereo_pair(bad, bad)


def test_stereo_dimensions_must_match():
    with pytest.raises(ValueError, match="identical"):
        prepare_stereo_pair(np.zeros((64, 96), np.uint8), np.zeros((64, 95), np.uint8))


@pytest.mark.parametrize("scale", [0, -1, 2, float("nan"), float("inf"), True])
def test_invalid_input_scale(scale):
    with pytest.raises(ValueError, match="input_scale"):
        prepare_stereo_pair(np.zeros((64, 64), np.uint8), np.zeros((64, 64), np.uint8),
                            input_scale=scale)


def test_resize_restores_horizontal_disparity_scale_including_rounding():
    pair = np.zeros((101, 203), np.uint8)
    a, _, meta = prepare_stereo_pair(pair, pair, input_scale=0.5)
    assert a.shape[:2] == (50, 102)
    sx = 102 / 203
    raw = np.full(a.shape[:2], 12 * sx, np.float32)
    restored = restore_disparity_resolution(raw, meta)
    assert restored.shape == pair.shape
    np.testing.assert_allclose(restored, 12, rtol=1e-6)
    # Depth f*b/d remains invariant under the horizontal resize.
    assert np.isclose((600 * sx) * 0.05 / raw[0, 0], 600 * 0.05 / restored[0, 0])


def test_invalid_disparities_remain_invalid_not_fabricated():
    _, _, meta = prepare_stereo_pair(np.zeros((64, 64), np.uint8), np.zeros((64, 64), np.uint8))
    raw = np.ones((64, 64), np.float32)
    raw[0, :4] = [np.nan, np.inf, -1, 0]
    result = restore_disparity_resolution(raw, meta)
    assert np.isnan(result[0, 0]) and np.isinf(result[0, 1])
    assert result[0, 2] == -1 and result[0, 3] == 0


def test_disparity_requires_unpadded_shape():
    _, _, meta = prepare_stereo_pair(np.zeros((70, 96), np.uint8), np.zeros((70, 96), np.uint8))
    with pytest.raises(ValueError, match="un-padded"):
        restore_disparity_resolution(np.ones((96, 96), np.float32), meta)


def test_manifest_verifies_both_files_and_exact_upstream(bundle, monkeypatch):
    manifest, _ = bundle
    calls = []
    def git(args, **kwargs):
        calls.append(args)
        return UPSTREAM_COMMIT if args[-2:] == ["rev-parse", "HEAD"] else ""
    monkeypatch.setattr(subprocess, "check_output", git)
    parsed = ModelManifest.read(manifest)
    result = parsed.verify()
    assert result["hashes_verified"] and result["config_sha256"] == CONFIG_SHA256
    assert len(calls) == 2
    parsed.checkpoint.write_bytes(b"CHANGED")
    with pytest.raises(ValueError, match="SHA-256"):
        parsed.verify()


@pytest.mark.parametrize("key,value", [("upstream_commit", "0" * 40),
                                      ("model_id", "23-36-37"),
                                      ("checkpoint_sha256", "TODO"),
                                      ("config_sha256", "0" * 64),
                                      ("source_model_folder_url", "https://unrelated.example"),
                                      ("config", "other/cfg.yaml")])
def test_manifest_rejects_unreviewed_assets(bundle, key, value):
    manifest, data = bundle
    data[key] = value
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        ModelManifest.read(manifest)


def test_dirty_upstream_is_rejected(bundle, monkeypatch):
    manifest, _ = bundle
    monkeypatch.setattr(subprocess, "check_output", lambda args, **kwargs:
                        UPSTREAM_COMMIT if args[-2:] == ["rev-parse", "HEAD"] else " M core/submodule.py")
    with pytest.raises(ValueError, match="modified"):
        ModelManifest.read(manifest).verify()


def test_untrusted_pickle_rejected_before_torch_import(bundle):
    manifest, _ = bundle
    runner = FastFoundationStereoRunner(manifest)
    with pytest.raises(ValueError, match="explicit allow_trusted_pickle"):
        runner.infer_rectified(np.zeros((64, 64), np.uint8), np.zeros((64, 64), np.uint8))
    assert runner._model is None


@pytest.mark.parametrize("args", [{"valid_iters": 0}, {"valid_iters": True},
                                  {"max_disp": 191}, {"max_disp": -32},
                                  {"allow_trusted_pickle": "yes"}])
def test_runtime_settings_rejected_before_loading(bundle, args):
    manifest, _ = bundle
    with pytest.raises(ValueError):
        FastFoundationStereoRunner(manifest, **args)


def test_official_checkpoint_omits_unused_slow_fast_gru():
    from real_robot.piper.ffs_runtime import _verify_serialized_config
    identity = {"checkpoint_sha256": "98b5a9acf39fbfa795025de8cea95ce123daa40f6b6234d719167751024cf692",
                "config_sha256": CONFIG_SHA256, "upstream_commit": UPSTREAM_COMMIT,
                "model_id": DEFAULT_MODEL_ID}
    model_args = {"corr_levels": 2}
    result = _verify_serialized_config(model_args, {"corr_levels": 2, "slow_fast_gru": False}, identity)
    assert result[0]["key"] == "slow_fast_gru"
    assert result[0]["serialized_status"] == "absent"
    assert "slow_fast_gru" not in model_args


@pytest.mark.parametrize("change", ["checkpoint", "source", "config", "true", "none", "other_key"])
def test_serialized_config_exception_is_narrowly_pinned(change):
    from real_robot.piper.ffs_runtime import _verify_serialized_config
    identity = {"checkpoint_sha256": "98b5a9acf39fbfa795025de8cea95ce123daa40f6b6234d719167751024cf692",
                "config_sha256": CONFIG_SHA256, "upstream_commit": UPSTREAM_COMMIT,
                "model_id": DEFAULT_MODEL_ID}
    model_args = {}
    config = {"slow_fast_gru": False}
    if change == "checkpoint": identity["checkpoint_sha256"] = "a" * 64
    if change == "source": identity["upstream_commit"] = "a" * 40
    if change == "config": identity["config_sha256"] = "a" * 64
    if change == "true": model_args["slow_fast_gru"] = True
    if change == "none": model_args["slow_fast_gru"] = None
    if change == "other_key": config = {"mixed_precision": True}
    with pytest.raises(ValueError, match="serialized model/config disagree"):
        _verify_serialized_config(model_args, config, identity)
