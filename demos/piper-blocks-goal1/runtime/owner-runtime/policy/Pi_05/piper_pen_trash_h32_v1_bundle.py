"""Verified prompt-catalog correction for pen-trash h32 v1 EMA step 9999."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from XPolicyLab.policy.Pi_05.piper_desktop_h32_v3_bundle import (
    ACTION_FREQUENCY_HZ,
    ACTION_HORIZON,
    CONFIG_NAME,
    EXECUTE_HORIZON,
    PEN_TRASH_EMA_STEP9999 as IDENTITY,
    PEN_TRASH_PROMPTS as PUBLISHER_PROMPTS,
    verify_piper_h32_ema_bundle,
)


DATASET_ID = IDENTITY.dataset_id
DATA_VERSION = IDENTITY.data_version
NORM_SHA256 = IDENTITY.norm_sha256
PROMPTS = (
    "Close the laptop.",
    "Pick up one drink container and place it on the coaster.",
    "Pick up one mouse and place it on the mouse pad.",
    "Pick up one pen and place it in the pen holder.",
    "Pick up one pen and place it in the trash bin.",
    "Pick up all paper balls and place them in the trash bin.",
)
ADDED_PROMPTS = (
    "Pick up all paper balls and place them in the trash bin.",
)


def verify_piper_pen_trash_h32_v1_bundle(
    bundle_root: Path,
    *,
    full_manifest: bool = False,
) -> dict[str, Any]:
    """Verify the immutable package, then apply its acknowledged catalog fix."""

    result = verify_piper_h32_ema_bundle(
        bundle_root,
        full_manifest=full_manifest,
        expected_step=IDENTITY.step,
        expected_package_id=IDENTITY.package_id,
    )
    if tuple(result.get("prompts", ())) != PUBLISHER_PROMPTS:
        raise ValueError("pen-trash publisher prompt catalog changed unexpectedly")

    corrected = dict(result)
    corrected["publisher_prompts"] = list(PUBLISHER_PROMPTS)
    corrected["prompts"] = list(PROMPTS)
    corrected["prompt_catalog_correction"] = {
        "acknowledged": True,
        "source": "operator-confirmed-2026-09-07",
        "added_prompts": list(ADDED_PROMPTS),
    }
    return corrected


__all__ = [
    "ACTION_FREQUENCY_HZ",
    "ACTION_HORIZON",
    "CONFIG_NAME",
    "DATASET_ID",
    "DATA_VERSION",
    "EXECUTE_HORIZON",
    "IDENTITY",
    "NORM_SHA256",
    "PROMPTS",
    "PUBLISHER_PROMPTS",
    "verify_piper_pen_trash_h32_v1_bundle",
]
