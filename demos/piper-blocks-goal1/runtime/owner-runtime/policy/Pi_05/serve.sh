#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
POLICY_ROOT="${PI05_POLICY_ROOT:-${SCRIPT_DIR}/openpi}"
PYTHON_BIN="${PI05_PYTHON:-${POLICY_ROOT}/.venv/bin/python}"
CHECKPOINT="${PI05_CHECKPOINT:-/home/fourier/siyuan/pi05-experiments-main/checkpoints/pi05_anygrasp/anygrasp_20hr/28000}"
HOST="${PI05_HOST:-0.0.0.0}"
PORT="${PI05_PORT:-19001}"
ALLOW_DUMMY="${PI05_ALLOW_DUMMY_POLICY:-0}"

if [[ "${ALLOW_DUMMY}" != "0" && "${ALLOW_DUMMY}" != "1" ]]; then
    echo "PI05_ALLOW_DUMMY_POLICY must be 0 or 1." >&2
    exit 2
fi
if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "PI0.5 environment is missing: ${PYTHON_BIN}" >&2
    echo "Run: cd ${POLICY_ROOT} && uv sync --frozen" >&2
    exit 1
fi
if ! "${PYTHON_BIN}" -c "import websockets" >/dev/null 2>&1; then
    echo "XPolicyLab websocket dependencies are missing from ${PYTHON_BIN}." >&2
    echo "Run: uv pip install --python ${PYTHON_BIN} -e ${XPL_ROOT}" >&2
    exit 1
fi

export OPENPI_DATA_HOME="${OPENPI_DATA_HOME:-${POLICY_ROOT}/.cache/openpi}"
export XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export PYTHONPATH="${XPL_ROOT}:${POLICY_ROOT}:${POLICY_ROOT}/src:${PYTHONPATH:-}"

cd "${POLICY_ROOT}"
exec "${PYTHON_BIN}" -u "${XPL_ROOT}/setup_policy_server.py" \
    --config_path "${SCRIPT_DIR}/deploy_gr3.yml" \
    --overrides \
        host="${HOST}" \
        port="${PORT}" \
        pi05_policy_root="${POLICY_ROOT}" \
        checkpoint_path="${CHECKPOINT}" \
        allow_dummy_policy="${ALLOW_DUMMY}"
