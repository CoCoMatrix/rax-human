#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUNDLE="${PI05_BUNDLE_ROOT:-/home/ubuntu/astribot_pick_tomato_v3_pi05_ema_step20000_ckpt}"
POLICY_ROOT="${BUNDLE}/runtime/openpi_source"
MODEL_ENV="${PI05_MODEL_ENV:-${XPL_ROOT}/.runtime/pi05-astribot-tomato-v3}"
PYTHON_BIN="${PI05_PYTHON:-${MODEL_ENV}/bin/python}"
HOST="${PI05_HOST:-0.0.0.0}"
PORT="${PI05_PORT:-19001}"

if [[ ! -f "${BUNDLE}/deploy_input.json" || ! -f "${BUNDLE}/infer.py" ]]; then
    echo "Invalid Astribot Pi0.5 bundle: ${BUNDLE}" >&2
    exit 1
fi
if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "Bundle environment is missing: ${PYTHON_BIN}" >&2
    echo "Run: UV_PROJECT_ENVIRONMENT=${MODEL_ENV} GIT_LFS_SKIP_SMUDGE=1 uv sync --project ${POLICY_ROOT} --frozen --no-dev" >&2
    exit 1
fi
if ! "${PYTHON_BIN}" -c "import websockets" >/dev/null 2>&1; then
    echo "XPolicyLab websocket dependencies are missing from ${PYTHON_BIN}." >&2
    echo "Run: uv pip install --python ${PYTHON_BIN} -e ${XPL_ROOT}" >&2
    exit 1
fi

CHECKPOINT_SHA256="${PI05_CHECKPOINT_SHA256:-}"
if [[ -z "${CHECKPOINT_SHA256}" ]]; then
    CHECKPOINT_SHA256="$(
        "${PYTHON_BIN}" -c \
            'from learning.recap.console.model_registry import checkpoint_digest; import sys; print(checkpoint_digest(sys.argv[1]))' \
            "${BUNDLE}"
    )"
fi
if [[ ! "${CHECKPOINT_SHA256}" =~ ^[0-9a-f]{64}$ ]]; then
    echo "Astribot Pi0.5 bundle SHA256 is missing or invalid: ${CHECKPOINT_SHA256:-unset}" >&2
    exit 1
fi
SOURCE_REVISION="${PI05_SOURCE_REVISION:-$(tr -d '[:space:]' < "${BUNDLE}/runtime/SOURCE_COMMIT")}"
if [[ ! "${SOURCE_REVISION}" =~ ^[0-9a-f]{40}$ ]]; then
    echo "Astribot Pi0.5 source revision is missing or invalid: ${SOURCE_REVISION:-unset}" >&2
    exit 1
fi

echo "Astribot Pi0.5 bundle: ${BUNDLE}"
echo "Astribot Pi0.5 bundle SHA256: ${CHECKPOINT_SHA256}"
echo "Astribot Pi0.5 source revision: ${SOURCE_REVISION}"
echo "Astribot Pi0.5 endpoint: ws://${HOST}:${PORT}"

export OPENPI_DATA_HOME="${BUNDLE}/openpi_data"
export XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="${XPL_ROOT}:${POLICY_ROOT}:${POLICY_ROOT}/src:${PYTHONPATH:-}"

cd "${POLICY_ROOT}"
exec "${PYTHON_BIN}" -u "${XPL_ROOT}/setup_policy_server.py" \
    --config_path "${SCRIPT_DIR}/deploy_astribot.yml" \
    --overrides \
        host="${HOST}" \
        port="${PORT}" \
        bundle_root="${BUNDLE}" \
        checkpoint_sha256="${CHECKPOINT_SHA256}" \
        source_revision="${SOURCE_REVISION}"
