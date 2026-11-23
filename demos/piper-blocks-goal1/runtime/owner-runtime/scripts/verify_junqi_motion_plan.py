"""Verify the exact offline-qualified trajectory and its kinematic sources."""
import hashlib
import json
from pathlib import Path
import sys

EXPECTED = "45bf7f883c60efc4793da7c0fa1941a60bde9e8df2f6e956e7c38a9d5b36a85b"

def main():
    bundle = Path(sys.argv[1]).resolve()
    repo = Path(__file__).resolve().parents[1]
    raw = (bundle / "plan.json").read_bytes()
    if hashlib.sha256(raw).hexdigest() != EXPECTED:
        raise SystemExit("Frozen motion plan SHA256 mismatch")
    plan = json.loads(raw)
    for path, field in [
        ("real_world/robots/piper_x_dual/profile.yml", "profile_sha256"),
        ("real_world/robots/piper_x_dual/description/dual_piper_x_mink.urdf", "urdf_sha256"),
    ]:
        if hashlib.sha256((repo / path).read_bytes()).hexdigest() != plan[field]:
            raise SystemExit(f"Motion plan source changed: {field}")
    catalog = json.loads((bundle / "task_catalog.json").read_text())
    if catalog != {"exact_match_required": True, "tasks": [{"task_index": 0, "task": plan["instruction"]}]}:
        raise SystemExit("Motion task catalog mismatch")
    if len(plan["actions"]) != 541 or plan["float32_transport_validation"]["passed"] is not True:
        raise SystemExit("Motion plan offline qualification missing")
    print(json.dumps({"plan_sha256": EXPECTED, "frozen_plan_valid": True, "hardware_qualified": False}))

if __name__ == "__main__":
    main()
