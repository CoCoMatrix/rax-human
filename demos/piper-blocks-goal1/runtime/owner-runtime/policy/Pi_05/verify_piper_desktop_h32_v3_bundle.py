#!/usr/bin/env python3
"""Verify the promoted dual-PiPER desktop-cleanup h32 v3 EMA bundle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from XPolicyLab.policy.Pi_05.piper_desktop_h32_v3_bundle import (
    verify_piper_desktop_h32_v3_bundle,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument(
        "--full-manifest",
        action="store_true",
        help="rehash every package file; intended for setup/intake",
    )
    args = parser.parse_args()
    result = verify_piper_desktop_h32_v3_bundle(
        args.bundle,
        full_manifest=args.full_manifest,
        expected_step=10_000,
    )
    result.pop("deploy", None)
    result.pop("contract", None)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
