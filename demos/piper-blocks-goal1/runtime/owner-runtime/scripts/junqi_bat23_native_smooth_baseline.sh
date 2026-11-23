#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset PIPER_NATIVE_TRAJECTORY_PROFILE PIPER_NATIVE_VELOCITY_SEAM_PROFILE PIPER_NATIVE_ONE_EURO_PROFILE PIPER_POLICY_CHUNK_LOG_INTERVAL
unset PIPER_NATIVE_JOINT_CLIP_PROFILE
export PIPER_MOTION_TRACE_PATH="/home/ubuntu/code_junqi/migration/native-smooth-trial-20261010/baseline-$(date +%Y%m%d-%H%M%S)-$$.jsonl"
exec bash "$repo/scripts/junqi_bat23_mixed2500_rtc5.sh" "$@"
