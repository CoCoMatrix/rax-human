#!/usr/bin/env bash
# Prints formal launch wiring only. Never starts a graph or hardware owner.
set -euo pipefail
repo=/home/ubuntu/code_junqi/xpolicylab-robot-runtime
export PIPER_TRAINING_RTC_ENABLED=1
export PIPER_NATIVE_RTC_FUTURE_STEPS="${PIPER_NATIVE_RTC_FUTURE_STEPS:-5}"
export ACTION_RTC_MODE=disabled ACTION_RTC_TRANSITION_STEPS=0
unset ACTION_TRAJECTORY_PROFILE
cd "$repo"
exec /usr/bin/python3 policy/Pi_05/piper_deployments.py run junqi-cup-native-rtc-instance02 \
  --model-server managed --model-port 19027 --no-pigey --control-source keyboard \
  --operator-ui-port 8097 --no-model-filter --no-write --dry-run
