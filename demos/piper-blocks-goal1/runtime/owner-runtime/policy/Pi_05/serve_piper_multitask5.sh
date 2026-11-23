#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DEFAULT_BUNDLE="/home/ubuntu/michael_dev/model-store/models/pi05/piper-dual-arm-multitask5-20260825-v1/step-20000"
BUNDLE="${PI05_BUNDLE_ROOT:-${DEFAULT_BUNDLE}}"
RUNTIME_SOURCE_ROOT="${PI05_RUNTIME_SOURCE_ROOT:-${XPL_ROOT}/.xpolicy/runtimes/piper_multitask5_20260825_ema_step20000}"
PYTHON_BIN="${RUNTIME_SOURCE_ROOT}/.venv/bin/python"
HOST="${PI05_HOST:-127.0.0.1}"
PORT="${PI05_PORT:-19003}"

[[ -x "${PYTHON_BIN}" ]] || {
    echo "Piper multitask runtime is missing: ${PYTHON_BIN}" >&2
    echo "Run: PI05_BUNDLE_ROOT=${BUNDLE} bash ${SCRIPT_DIR}/setup_piper_multitask5.sh" >&2
    exit 1
}
PYTHONPATH="${XPL_ROOT}:${PYTHONPATH:-}" /usr/bin/python3 \
    "${SCRIPT_DIR}/verify_piper_multitask_bundle.py" "${BUNDLE}"
PYTHONPATH="${XPL_ROOT}:${PYTHONPATH:-}" /usr/bin/python3 \
    "${SCRIPT_DIR}/prepare_piper_bundle_runtime.py" \
    --bundle "${BUNDLE}" \
    --runtime-root "${RUNTIME_SOURCE_ROOT}" >/dev/null

export OPENPI_PALIGEMMA_TOKENIZER_PATH="${BUNDLE}/tokenizer/paligemma_tokenizer.model"
export XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export PYTHONDONTWRITEBYTECODE=1
export PI05_RUNTIME_SOURCE_ROOT="${RUNTIME_SOURCE_ROOT}"
export PYTHONPATH="${XPL_ROOT}:${RUNTIME_SOURCE_ROOT}/src:${RUNTIME_SOURCE_ROOT}/packages/openpi-client/src:${PYTHONPATH:-}"

cd "${RUNTIME_SOURCE_ROOT}"
exec "${PYTHON_BIN}" -u "${XPL_ROOT}/setup_policy_server.py" \
    --config_path "${SCRIPT_DIR}/deploy_piper_multitask5.yml" \
    --overrides \
        host="${HOST}" \
        port="${PORT}" \
        bundle_root="${BUNDLE}" \
        runtime_source_root="${RUNTIME_SOURCE_ROOT}"
