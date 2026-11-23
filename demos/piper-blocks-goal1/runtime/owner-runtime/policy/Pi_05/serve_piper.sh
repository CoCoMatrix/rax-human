#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPOLICYLAB_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
POLICY_ROOT="${PI05_POLICY_ROOT:-${SCRIPT_DIR}/openpi}"
PYTHON_BIN="${PI05_PYTHON:-${POLICY_ROOT}/.venv/bin/python}"
CHECKPOINT="${PI05_CHECKPOINT:-${SCRIPT_DIR}/checkpoints/piper-pi05-dj-tricam-b512-20k-0825-r3/2000}"
HOST="${PI05_HOST:-0.0.0.0}"
PORT="${PI05_PORT:-19003}"

[[ -x "${PYTHON_BIN}" ]] || { echo "Pi0.5 environment is missing: ${PYTHON_BIN}" >&2; exit 1; }
[[ -d "${CHECKPOINT}/params" && -d "${CHECKPOINT}/assets" ]] || {
  echo "Piper Pi0.5 checkpoint is incomplete: ${CHECKPOINT}" >&2
  exit 1
}

export OPENPI_DATA_HOME="${OPENPI_DATA_HOME:-${POLICY_ROOT}/.cache/openpi}"
export XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export PYTHONPATH="${XPOLICYLAB_ROOT}:${POLICY_ROOT}:${POLICY_ROOT}/src:${PYTHONPATH:-}"

cd "${POLICY_ROOT}"
exec "${PYTHON_BIN}" -u "${XPOLICYLAB_ROOT}/setup_policy_server.py" \
  --config_path "${SCRIPT_DIR}/deploy_piper.yml" \
  --overrides host="${HOST}" port="${PORT}" checkpoint_path="${CHECKPOINT}"
