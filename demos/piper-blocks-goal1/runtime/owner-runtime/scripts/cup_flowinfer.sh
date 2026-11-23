#!/usr/bin/env bash
set -euo pipefail
flow_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
cd "$flow_repo"
exec /usr/bin/python3 policy/Pi_05/piper_deployments.py run \
  junqi-flowinfer-cup-switch-rtc1000-instance02 \
  --no-pigey --control-source keyboard --no-write --open-ui "$@"
