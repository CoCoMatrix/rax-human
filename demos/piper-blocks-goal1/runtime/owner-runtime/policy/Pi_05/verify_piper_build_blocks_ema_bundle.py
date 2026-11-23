#!/usr/bin/env python3
"""Verify the promoted dual-PiPER build-blocks EMA bundle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from XPolicyLab.policy.Pi_05.piper_build_blocks_ema_bundle import (
    verify_piper_build_blocks_ema_bundle,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument(
        "--full-manifest",
        action="store_true",
        help="rehash every package file; intended for intake, not every launch",
    )
    args = parser.parse_args()
    print(
        json.dumps(
            verify_piper_build_blocks_ema_bundle(
                args.bundle, full_manifest=args.full_manifest
            ),
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
