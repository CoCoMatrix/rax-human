#!/usr/bin/env bash
# Two independently pinned checkpoints; native RTC and instance02 are explicit.
set -euo pipefail
grip_variant="${1:?Usage: bash scripts/cup_gripper_timing.sh event|phase [--enable-write] [--dry-run]}"
shift
case "$grip_variant" in
  event|phase) ;;
  *) echo 'Expected event or phase' >&2; exit 2 ;;
esac
grip_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
source "$grip_repo/policy/Pi05GripperTiming/native_env.sh"
cd "$grip_repo"
exec /usr/bin/python3 policy/Pi_05/piper_deployments.py run \
  "junqi-cup-grip-${grip_variant}-ema1500-instance02" \
  --model-server managed --model-port 19043 --no-pigey --control-source keyboard \
  --operator-ui-port 8097 --no-model-filter --no-write --open-ui "$@"
