"""Model-agnostic verifier for registered dual-PiPER h32 EMA bundles.

The exact package identity is selected from the contract's ``package_id`` by
the shared h32 registry.  This adapter intentionally contains no dataset- or
checkpoint-specific branch; adding a bundle only extends that registry and a
deployment descriptor.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from XPolicyLab.policy.Pi_05.piper_desktop_h32_v3_bundle import (
    ACTION_FREQUENCY_HZ,
    ACTION_HORIZON,
    CONFIG_NAME,
    EXECUTE_HORIZON,
    OBJECT50_DATASET_ID,
    OBJECT50_DATA_VERSION,
    OBJECT50_NORM_SHA256,
    OBJECT50_PROMPTS,
    verify_piper_h32_ema_bundle,
)


def verify_piper_h32_ema_bundle_package(
    bundle_root: Path,
    *,
    full_manifest: bool = False,
) -> dict[str, Any]:
    """Verify any registered dual-PiPER h32 EMA package."""

    return verify_piper_h32_ema_bundle(bundle_root, full_manifest=full_manifest)


__all__ = [
    "ACTION_FREQUENCY_HZ",
    "ACTION_HORIZON",
    "CONFIG_NAME",
    "EXECUTE_HORIZON",
    "OBJECT50_DATASET_ID",
    "OBJECT50_DATA_VERSION",
    "OBJECT50_NORM_SHA256",
    "OBJECT50_PROMPTS",
    "verify_piper_h32_ema_bundle_package",
]
