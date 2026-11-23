#!/usr/bin/env python3
"""Verify one registered dual-PiPER h32 EMA bundle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from XPolicyLab.policy.Pi_05.piper_h32_ema_bundle import (
    verify_piper_h32_ema_bundle_package,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--full-manifest", action="store_true")
    args = parser.parse_args()
    result = verify_piper_h32_ema_bundle_package(
        args.bundle,
        full_manifest=args.full_manifest,
    )
    result.pop("deploy", None)
    result.pop("contract", None)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
