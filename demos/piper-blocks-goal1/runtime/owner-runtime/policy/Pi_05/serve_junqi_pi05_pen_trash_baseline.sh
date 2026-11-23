#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
export PI05_BUNDLE_ROOT="${PI05_BUNDLE_ROOT:-/home/ubuntu/code_junqi/model-store/models/pi05/pen-trash-dual-piper-real-h32-v1-ema-v1/step-9999}"
export PI05_MODEL_ENV="${XPL_ROOT}/.xpolicy/envs/junqi_pi05_pen_trash_py312"
export PI05_BLACKWELL_OVERLAY="${XPL_ROOT}/.xpolicy/overlays/junqi_pi05_torch213_cu130"
export PI05_BUNDLE_VERIFIER="${SCRIPT_DIR}/verify_piper_pen_trash_h32_v1_ema9999_bundle.py"
export PI05_DEPLOY_CONFIG="${SCRIPT_DIR}/deploy_junqi_pi05_pen_trash_baseline.yml"
export PI05_SETUP_SCRIPT="${SCRIPT_DIR}/setup_junqi_pi05_pen_trash_baseline.sh"
export PI05_SETUP_LABEL="Junqi pen-trash h32 EMA 9999"
export PI05_DATASET_ID="dual_piper_pen_trash_curriculum50to20_h32_v1"
export PI05_HOST=127.0.0.1
export PI05_PORT="${PI05_PORT:-19013}"
export PYTHONPATH="${PI05_BLACKWELL_OVERLAY}:${PYTHONPATH}"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
[[ "${PI05_BUNDLE_ROOT}" == /home/ubuntu/code_junqi/model-store/* ]] || {
  echo "Junqi baseline only accepts a personal immutable model-store path" >&2; exit 1;
}
[[ -x "${PI05_MODEL_ENV}/bin/python" && -d "${PI05_BLACKWELL_OVERLAY}/torch" ]] || {
  echo "Junqi independent model runtime is not prepared" >&2; exit 1;
}
# Only this model service is bounded; this script never starts a robot graph.
exec timeout --signal=TERM --kill-after=15s 1800s \
  bash "${SCRIPT_DIR}/serve_piper_desktop_cleanup.sh"
