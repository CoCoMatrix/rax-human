#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export PI05_BUNDLE_ROOT="${PI05_BUNDLE_ROOT:?PI05_BUNDLE_ROOT is required}"
export PI05_MODEL_ENV="${PI05_MODEL_ENV:-${XPL_ROOT}/.xpolicy/envs/pi05_desktop_cleanup_py312}"
export PI05_BUNDLE_VERIFIER="${SCRIPT_DIR}/verify_piper_h32_ema_bundle.py"
export PI05_BUNDLE_FULL_VERIFY_ARG="--full-manifest"
export PI05_SETUP_LABEL="dual-PiPER h32 EMA"

exec bash "${SCRIPT_DIR}/setup_piper_desktop_cleanup.sh"
