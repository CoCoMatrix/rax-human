#!/usr/bin/env bash
# Formal launcher only. Default is hardware-free dry-run.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
mode="${1:---dry-run}"
[[ $# -le 1 && ( "$mode" == --dry-run || "$mode" == --run ) ]] || {
  echo 'Usage: scripts/junqi_cloud_noop.sh [--dry-run|--run]' >&2; exit 2;
}
if [[ "$mode" == --run && "${JUNQI_DEVICE_RELEASED:-0}" != 1 ]]; then
  echo 'Await device handover; set JUNQI_DEVICE_RELEASED=1 only after the operator releases the equipment.' >&2
  exit 2
fi
export XPOLICY_ALLOW_NO_ACTION=1
export ENABLE_WRITE=0
cd "$repo_root"
args=(piper_x_dual --deployment junqi-cloud-noop --model-server external
      --policy-url ws://127.0.0.1:19992 --operator-ui-port 18091
      --control-source keyboard --dagger-mode off --no-write --no-open-ui
      --duration 60)
[[ "$mode" != --dry-run ]] || args+=(--dry-run)
exec bash scripts/launch_robot.sh "${args[@]}"
