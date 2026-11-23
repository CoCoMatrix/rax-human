#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DEFAULT_BUNDLE="/home/ubuntu/michael_dev/model-store/models/pi05/piper-dual-arm-multitask5-20260825-v1/step-20000"
BUNDLE="${PI05_BUNDLE_ROOT:-${DEFAULT_BUNDLE}}"
RUNTIME_SOURCE_ROOT="${PI05_RUNTIME_SOURCE_ROOT:-${XPL_ROOT}/.xpolicy/runtimes/piper_multitask5_20260825_ema_step20000}"
UV_BIN="${PI05_UV_BIN:-}"
if [[ -z "${UV_BIN}" ]]; then
    if command -v uv >/dev/null 2>&1; then
        UV_BIN="$(command -v uv)"
    elif [[ -x /home/ubuntu/miniconda3/envs/infra/bin/uv ]]; then
        UV_BIN=/home/ubuntu/miniconda3/envs/infra/bin/uv
    else
        echo "uv is required; set PI05_UV_BIN to its executable path" >&2
        exit 1
    fi
fi

for required in \
    "${BUNDLE}/deploy_input.json" \
    "${BUNDLE}/contract.json" \
    "${BUNDLE}/task_catalog.json" \
    "${BUNDLE}/checkpoint/params/manifest.ocdbt" \
    "${BUNDLE}/runtime/source.tar"; do
    [[ -f "${required}" ]] || {
        echo "Invalid Piper multitask bundle; missing ${required}" >&2
        exit 1
    }
done

echo "Verifying immutable Piper multitask bundle identity and package hashes..."
PYTHONPATH="${XPL_ROOT}:${PYTHONPATH:-}" /usr/bin/python3 \
    "${SCRIPT_DIR}/verify_piper_multitask_bundle.py" \
    "${BUNDLE}" --full-manifest

echo "Preparing mutable runtime outside the immutable model store..."
PYTHONPATH="${XPL_ROOT}:${PYTHONPATH:-}" /usr/bin/python3 \
    "${SCRIPT_DIR}/prepare_piper_bundle_runtime.py" \
    --bundle "${BUNDLE}" \
    --runtime-root "${RUNTIME_SOURCE_ROOT}" \
    --uv "${UV_BIN}"

PYTHON_BIN="${RUNTIME_SOURCE_ROOT}/.venv/bin/python"
[[ -x "${PYTHON_BIN}" ]] || {
    echo "Piper multitask runtime Python is missing: ${PYTHON_BIN}" >&2
    exit 1
}
PYTHONPATH="${XPL_ROOT}:${RUNTIME_SOURCE_ROOT}/src:${RUNTIME_SOURCE_ROOT}/packages/openpi-client/src:${PYTHONPATH:-}" \
    "${PYTHON_BIN}" - <<'PY'
from importlib.metadata import version

import jax
import msgpack_numpy
import yaml
import websockets
from client_server.ws.model_server import PolicyServer
from XPolicyLab.policy.Pi_05.piper_multitask_bundle import PROMPTS

assert len(PROMPTS) == 5
assert PolicyServer is not None
print(
    "Piper multitask runtime imports passed: "
    f"jax={jax.__version__}, msgpack_numpy={version('msgpack-numpy')}, "
    f"yaml={yaml.__version__}, websockets={websockets.__version__}"
)
PY

echo "Piper multitask bundle setup complete."
echo "Bundle: ${BUNDLE}"
echo "Runtime source: ${RUNTIME_SOURCE_ROOT}"
echo "Python: ${PYTHON_BIN}"
echo "No robot device was opened and no model server was started."
