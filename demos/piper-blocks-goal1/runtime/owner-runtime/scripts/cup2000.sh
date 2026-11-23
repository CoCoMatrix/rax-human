#!/usr/bin/env bash
# Short entry point; the formal launcher remains the only robot control path.
set -euo pipefail
cup_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
cd "$cup_repo"
export ACTION_RTC_MODE=disabled
exec /usr/bin/python3 policy/Pi_05/piper_deployments.py run \
  junqi-cup-switch-bat2-baseline-ema-step2000-instance02 \
  --no-pigey --control-source keyboard --no-write --open-ui "$@"
