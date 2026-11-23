#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
export PIPER_TRAINING_RTC_ENABLED=1 PIPER_NATIVE_RTC_FUTURE_STEPS=5
export ACTION_RTC_MODE=disabled ACTION_RTC_TRANSITION_STEPS=0 PIPER_MODEL_FILTER_ENABLED=0
unset ACTION_TRAJECTORY_PROFILE PIPER_NATIVE_TRAJECTORY_PROFILE PIPER_NATIVE_VELOCITY_SEAM_PROFILE PIPER_NATIVE_ONE_EURO_PROFILE PIPER_NATIVE_JOINT_CLIP_PROFILE
cd "$repo"
exec python policy/Pi_05/piper_deployments.py run junqi-bat23-top3-ema1500-rtc5-instance01-1013 "$@"
