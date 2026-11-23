"""Exact identity selector for the DAgger50 dual-PiPER h32 EMA bundle."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from XPolicyLab.policy.Pi_05.piper_desktop_h32_v3_bundle import (
    ACTION_FREQUENCY_HZ,
    ACTION_HORIZON,
    CONFIG_NAME,
    DAGGER50_DATASET_ID as DATASET_ID,
    DAGGER50_DATA_VERSION as DATA_VERSION,
    DAGGER50_EMA_STEP4999 as IDENTITY,
    DAGGER50_NORM_SHA256 as NORM_SHA256,
    EXECUTE_HORIZON,
    PAPERALL50_PROMPTS as PROMPTS,
    verify_piper_h32_ema_bundle,
)


def verify_piper_dagger50_h32_v1_bundle(
    bundle_root: Path,
    *,
    full_manifest: bool = False,
) -> dict[str, Any]:
    """Verify the exact DAgger50 h32 v1 final EMA step-4999 identity."""
    return verify_piper_h32_ema_bundle(
        bundle_root,
        full_manifest=full_manifest,
        expected_step=IDENTITY.step,
        expected_package_id=IDENTITY.package_id,
    )
