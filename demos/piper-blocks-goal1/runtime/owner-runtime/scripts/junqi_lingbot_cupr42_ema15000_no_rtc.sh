#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
cd "$repo"
# User-requested deployment ablation: same weights, 15 executed actions, no RTC.
# Keep the original RTC launcher and immutable checkpoint unchanged.
export PIPER_TRAINING_RTC_ENABLED=0
export ACTION_RTC_MODE=disabled ACTION_RTC_TRANSITION_STEPS=0
export PIPER_MODEL_FILTER_ENABLED=0
unset PIPER_NATIVE_RTC_FUTURE_STEPS PIPER_NATIVE_RTC_MAX_PREFIX_STEPS PIPER_NATIVE_RTC_WALLCLOCK_DEADLINE
unset ACTION_TRAJECTORY_PROFILE PIPER_NATIVE_TRAJECTORY_PROFILE PIPER_NATIVE_ONE_EURO_PROFILE PIPER_NATIVE_JOINT_CLIP_PROFILE PIPER_NATIVE_VELOCITY_SEAM_PROFILE
exec .xpolicy/envs/piper_qualification/bin/python -B policy/Pi_05/piper_deployments.py run junqi-lingbot-cupr42-ema15000-rtc15-instance01 "$@"
