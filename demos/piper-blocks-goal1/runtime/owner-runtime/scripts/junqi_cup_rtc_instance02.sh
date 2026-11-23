#!/usr/bin/env bash
set -euo pipefail
source /home/ubuntu/code_junqi/env.sh
instance_repo=/home/ubuntu/code_junqi/xpolicylab-robot-runtime
export PIPER_DEVICE_INSTANCE="$instance_repo/runtime/robot_dagger/config/robot_instances/piper_x_dual_02.yml"
export ACTION_RTC_MODE=disabled ACTION_RTC_TRANSITION_STEPS=0
export PIPER_TRAINING_RTC_ENABLED=0
unset PIPER_READONLY_STATUS_OUTPUT PIPER_RELEASE_SOFTWARE_STOP_ARM
instance_write=--no-write
instance_args=()
for argument in "$@"; do
  case "$argument" in
    --enable-write|--no-write) instance_write="$argument" ;;
    *) instance_args+=("$argument") ;;
  esac
done
cd "$instance_repo"
exec /usr/bin/python3 policy/Pi_05/piper_deployments.py run junqi-cup-switch-bat2-trainingrtc-ema-step1000-instance02 \
  --model-server managed --model-port 19003 --no-pigey --control-source keyboard \
  --operator-ui-port 8091 "$instance_write" "${instance_args[@]}"
