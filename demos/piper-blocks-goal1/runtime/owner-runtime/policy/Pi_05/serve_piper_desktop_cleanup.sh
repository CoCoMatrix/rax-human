#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DEFAULT_BUNDLE="/home/ubuntu/michael_dev/model-store/models/pi05/desktop-cleanup-dual-piper-v1/step-10000"
BUNDLE="${PI05_BUNDLE_ROOT:-${DEFAULT_BUNDLE}}"
BUNDLE_VERIFIER="${PI05_BUNDLE_VERIFIER:-${SCRIPT_DIR}/verify_dual_piper_bundle.py}"
DEPLOY_CONFIG="${PI05_DEPLOY_CONFIG:-${SCRIPT_DIR}/deploy_piper_desktop_cleanup.yml}"
SETUP_SCRIPT="${PI05_SETUP_SCRIPT:-${SCRIPT_DIR}/setup_piper_desktop_cleanup.sh}"
SETUP_LABEL="${PI05_SETUP_LABEL:-desktop-cleanup}"
DATASET_ID="${PI05_DATASET_ID:-dual_piper_desktop_cleanup_20260825_v1}"
POLICY_ROOT="${BUNDLE}/runtime/openpi"
PROJECT_ROOT="${BUNDLE}/runtime/project"
LEROBOT_ROOT="${BUNDLE}/runtime/lerobot"
MODEL_ENV="${PI05_MODEL_ENV:-${XPL_ROOT}/.xpolicy/envs/pi05_desktop_cleanup_py312}"
PYTHON_BIN="${PI05_PYTHON:-${MODEL_ENV}/bin/python}"
BLACKWELL_OVERLAY="${PI05_BLACKWELL_OVERLAY:-${XPL_ROOT}/.xpolicy/overlays/pi05_torch213_cu130}"
HOST="${PI05_HOST:-127.0.0.1}"
PORT="${PI05_PORT:-19003}"
DEVICE="${PI05_DEVICE:-cuda}"

for required in \
    "${BUNDLE}/deploy_input.json" \
    "${BUNDLE}/contract.json" \
    "${BUNDLE}/checkpoint/model.safetensors" \
    "${POLICY_ROOT}/pyproject.toml" \
    "${PROJECT_ROOT}/src/desk_icl/training/openpi_dual_piper_policy.py"; do
    [[ -f "${required}" ]] || {
        echo "Invalid desktop-cleanup bundle; missing ${required}" >&2
        exit 1
    }
done
[[ -x "${PYTHON_BIN}" ]] || {
    echo "${SETUP_LABEL} model environment is missing: ${PYTHON_BIN}" >&2
    echo "Run: bash ${SETUP_SCRIPT}" >&2
    exit 1
}
[[ "$("${PYTHON_BIN}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" == "3.12" ]] || {
    echo "Desktop-cleanup bundle requires Python 3.12: ${PYTHON_BIN}" >&2
    exit 1
}
if [[ "${DEVICE}" == cuda* ]]; then
    BASE_TORCH_STATUS="$("${PYTHON_BIN}" - <<'PY'
import warnings

warnings.filterwarnings("ignore", category=UserWarning, module=r"torch\.cuda")
import torch

if not torch.cuda.is_available():
    print("no-cuda")
else:
    major, minor = torch.cuda.get_device_capability()
    wanted = f"sm_{major}{minor}"
    support = "supported" if wanted in torch.cuda.get_arch_list() else "unsupported"
    print(f"{wanted}:{support}")
PY
)"
    if [[ "${BASE_TORCH_STATUS}" =~ ^sm_12[0-9]*:unsupported$ ]]; then
        [[ -d "${BLACKWELL_OVERLAY}/torch" && -d "${BLACKWELL_OVERLAY}/torchvision" ]] || {
            echo "Blackwell Torch overlay is missing; run ${SETUP_SCRIPT}" >&2
            exit 1
        }
        export PYTHONPATH="${BLACKWELL_OVERLAY}:${PYTHONPATH:-}"
        echo "Blackwell CUDA overlay active: ${BLACKWELL_OVERLAY}"
    fi
    "${PYTHON_BIN}" - <<'PY'
import sys
import torch

if not torch.cuda.is_available():
    raise SystemExit("PI05_DEVICE requests CUDA but torch.cuda.is_available() is false")
major, minor = torch.cuda.get_device_capability()
wanted = f"sm_{major}{minor}"
supported = set(torch.cuda.get_arch_list())
if wanted not in supported:
    print(
        f"Pinned Torch does not support {torch.cuda.get_device_name(0)} "
        f"({wanted}); supported architectures: {', '.join(sorted(supported))}. "
        "Run the deployment setup script to prepare the local Blackwell overlay.",
        file=sys.stderr,
    )
    raise SystemExit(1)
PY
fi

PYTHONPATH="${XPL_ROOT}:${PYTHONPATH:-}" /usr/bin/python3 \
    "${BUNDLE_VERIFIER}" "${BUNDLE}"

export OPENPI_DATA_HOME="${BUNDLE}/assets"
export OPENPI_DATASET_ID="${DATASET_ID}"
export XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="${XPL_ROOT}:${PROJECT_ROOT}/src:${POLICY_ROOT}/src:${POLICY_ROOT}/packages/openpi-client/src:${LEROBOT_ROOT}:${PYTHONPATH:-}"

# Keep relative logs/caches outside the shared bundle; imports/assets stay pinned.
cd "${XPL_ROOT}"
exec "${PYTHON_BIN}" -W ignore::SyntaxWarning -u "${XPL_ROOT}/setup_policy_server.py" \
    --config_path "${DEPLOY_CONFIG}" \
    --overrides \
        host="${HOST}" \
        port="${PORT}" \
        bundle_root="${BUNDLE}" \
        pytorch_device="${DEVICE}"
