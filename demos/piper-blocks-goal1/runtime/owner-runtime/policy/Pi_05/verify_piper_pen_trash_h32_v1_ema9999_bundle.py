#!/usr/bin/env python3
"""Verify pen-trash h32 v1 final EMA step-9999 plus its catalog correction."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from XPolicyLab.policy.Pi_05.piper_pen_trash_h32_v1_bundle import (
    verify_piper_pen_trash_h32_v1_bundle,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--full-manifest", action="store_true")
    args = parser.parse_args()
    result = verify_piper_pen_trash_h32_v1_bundle(
        args.bundle,
        full_manifest=args.full_manifest,
    )
    result.pop("deploy", None)
    result.pop("contract", None)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
