#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

export PI05_BUNDLE_ROOT="${PI05_BUNDLE_ROOT:-/home/ubuntu/michael_dev/model-store/models/pi05/paperall50-dual-piper-real-h32-v1-ema-v1/step-4999}"
export PI05_MODEL_ENV="${PI05_MODEL_ENV:-${XPL_ROOT}/.xpolicy/envs/pi05_desktop_cleanup_py312}"
export PI05_BUNDLE_VERIFIER="${SCRIPT_DIR}/verify_piper_paperall50_h32_v1_ema4999_bundle.py"
export PI05_DEPLOY_CONFIG="${SCRIPT_DIR}/deploy_piper_paperall50_h32_v1_ema4999.yml"
export PI05_SETUP_SCRIPT="${SCRIPT_DIR}/setup_piper_paperall50_h32_v1_ema4999.sh"
export PI05_SETUP_LABEL="PaperAll50 h32 v1 final EMA step 4999"
export PI05_DATASET_ID="dual_piper_paperall50_from10k_h32_v1"

exec bash "${SCRIPT_DIR}/serve_piper_desktop_cleanup.sh"
