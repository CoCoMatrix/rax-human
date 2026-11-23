#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset PIPER_NATIVE_TRAJECTORY_PROFILE PIPER_NATIVE_ONE_EURO_PROFILE
unset PIPER_NATIVE_JOINT_CLIP_PROFILE
export PIPER_NATIVE_VELOCITY_SEAM_PROFILE="$repo/policy/Pi05Bat23MixedRTC5/velocity-seam-trial.json"
export PIPER_MOTION_TRACE_PATH="/home/ubuntu/code_junqi/migration/velocity-seam-trial-20261010/motion-$(date +%Y%m%d-%H%M%S)-$$.jsonl"
exec bash "$repo/scripts/junqi_bat23_mixed2500_rtc5.sh" "$@"
