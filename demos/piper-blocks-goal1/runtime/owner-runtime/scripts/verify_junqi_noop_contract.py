"""Verify a transport-only contract, never a checkpoint or live qualification."""
import hashlib
import json
from pathlib import Path
import sys

EXPECTED_SHA256 = "961deeff6bc9db372b0e2ad77eceb481227a2f6d2888486ccff276248670a162"


def main():
    bundle = Path(sys.argv[1]).resolve()
    actual_path = bundle / "contract.json"
    digest = hashlib.sha256(actual_path.read_bytes()).hexdigest()
    if digest != EXPECTED_SHA256:
        raise SystemExit("transport contract differs from registered identity")
    print(json.dumps({"transport_contract_valid": True, "checkpoint_loaded": False,
                      "sha256": digest}))


if __name__ == "__main__":
    main()
