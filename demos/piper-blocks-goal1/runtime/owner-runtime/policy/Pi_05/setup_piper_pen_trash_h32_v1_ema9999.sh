#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export PI05_BUNDLE_ROOT="${PI05_BUNDLE_ROOT:-/home/ubuntu/michael_dev/model-store/models/pi05/pen-trash-dual-piper-real-h32-v1-ema-v1/step-9999}"
export PI05_MODEL_ENV="${PI05_MODEL_ENV:-${XPL_ROOT}/.xpolicy/envs/pi05_desktop_cleanup_py312}"
export PI05_BUNDLE_VERIFIER="${SCRIPT_DIR}/verify_piper_pen_trash_h32_v1_ema9999_bundle.py"
export PI05_BUNDLE_FULL_VERIFY_ARG="--full-manifest"
export PI05_SETUP_LABEL="pen-trash h32 v1 final EMA step 9999"

exec bash "${SCRIPT_DIR}/setup_piper_desktop_cleanup.sh"
