#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
export PIPER_TRAINING_RTC_ENABLED=1 PIPER_NATIVE_RTC_FUTURE_STEPS=5
export ACTION_RTC_MODE=disabled ACTION_RTC_TRANSITION_STEPS=0 PIPER_MODEL_FILTER_ENABLED=0
unset ACTION_TRAJECTORY_PROFILE PIPER_RELEASE_SOFTWARE_STOP_ARM PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M
cd "$repo"
exec /usr/bin/python3 policy/Pi_05/piper_deployments.py run \
  junqi-cup-native-rtc5-live-instance01 --model-server managed --model-port 19027 \
  --no-pigey --control-source keyboard --operator-ui-port 8097 --no-model-filter \
  --enable-write "$@"
