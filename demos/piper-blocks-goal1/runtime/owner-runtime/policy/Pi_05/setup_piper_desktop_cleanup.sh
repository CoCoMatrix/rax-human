#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DEFAULT_BUNDLE="/home/ubuntu/michael_dev/model-store/models/pi05/desktop-cleanup-dual-piper-v1/step-10000"
BUNDLE="${PI05_BUNDLE_ROOT:-${DEFAULT_BUNDLE}}"
BUNDLE_VERIFIER="${PI05_BUNDLE_VERIFIER:-${SCRIPT_DIR}/verify_dual_piper_bundle.py}"
BUNDLE_FULL_VERIFY_ARG="${PI05_BUNDLE_FULL_VERIFY_ARG:-}"
SETUP_LABEL="${PI05_SETUP_LABEL:-desktop-cleanup}"
POLICY_ROOT="${BUNDLE}/runtime/openpi"
MODEL_ENV="${PI05_MODEL_ENV:-${XPL_ROOT}/.xpolicy/envs/pi05_desktop_cleanup_py312}"
MODEL_PYTHON="${PI05_MODEL_PYTHON:-3.12}"
FORCE_MODEL_SYNC="${PI05_FORCE_MODEL_SYNC:-0}"
ROBOT_ENV="${PIPER_ROBOT_ENV:-${XPL_ROOT}/.xpolicy/envs/piper_x_dual}"
DAGGER_ENV="${PIPER_DAGGER_ENV:-${XPL_ROOT}/runtime/robot_dagger/.venv}"
UV_BIN="${PI05_UV:-$(command -v uv || true)}"
PACKAGE_INDEX="${PI05_PACKAGE_INDEX:-https://mirrors.aliyun.com/pypi/simple}"
BLACKWELL_SOURCE_ENV="${PI05_BLACKWELL_SOURCE_ENV:-/home/ubuntu/michael_dev/model-serving/.venv/qwen-vl}"
BLACKWELL_SOURCE_SITE="${BLACKWELL_SOURCE_ENV}/lib/python3.12/site-packages"
BLACKWELL_OVERLAY="${PI05_BLACKWELL_OVERLAY:-${XPL_ROOT}/.xpolicy/overlays/pi05_torch213_cu130}"
if [[ -z "${UV_BIN}" && -x /home/ubuntu/miniconda3/envs/infra/bin/uv ]]; then
    UV_BIN=/home/ubuntu/miniconda3/envs/infra/bin/uv
fi

prepare_model_setup_source() {
    # uv may build workspace packages even with --no-install-project. Give it
    # a verified private source copy; all bundle assets remain read-only inputs.
    /usr/bin/python3 - "${BUNDLE}" "${XPL_ROOT}" <<'PY'
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile

bundle = Path(sys.argv[1]).expanduser().resolve()
repo = Path(sys.argv[2]).resolve()
source = bundle / "runtime/openpi"
key = hashlib.sha256(str(bundle).encode()).hexdigest()
parent = repo / ".xpolicy/setup_sources" / key
target = parent / "openpi"
if parent.resolve() != parent or target.is_symlink():
    raise SystemExit("private setup source path must not traverse symlinks")

def digest(path):
    with path.open("rb") as stream:
        value = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
        return value.hexdigest()

ignored = {".git", ".venv", "__pycache__"}
files = {}
for path in sorted(source.rglob("*")):
    relative = path.relative_to(source)
    if ignored.intersection(relative.parts) or path.suffix == ".pyc":
        continue
    if path.is_symlink():
        raise SystemExit(f"bundle setup source contains a symlink: {relative}")
    if path.is_file():
        files[str(relative)] = digest(path)
if not {"pyproject.toml", "uv.lock"}.issubset(files):
    raise SystemExit("bundle setup source is missing pyproject.toml or uv.lock")
origin = {
    "schema": "xpolicy.private_setup_source.v1",
    "bundle": str(bundle),
    "source": str(source),
    "files_sha256": files,
}
marker = target / "_XPOLICY_SETUP_SOURCE.json"
if target.exists():
    if not marker.is_file() or json.loads(marker.read_text()) != origin:
        raise SystemExit(f"private setup source identity mismatch: {target}")
    for name, expected in files.items():
        path = target / name
        if path.resolve() != path or not path.is_file() or digest(path) != expected:
            raise SystemExit(f"private setup source has changed: {path}")
else:
    parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".openpi-", dir=parent) as temporary:
        copied = Path(temporary) / "openpi"
        shutil.copytree(
            source, copied, symlinks=False,
            ignore=shutil.ignore_patterns(".git", ".venv", "__pycache__", "*.pyc"),
        )
        for path in [copied, *copied.rglob("*")]:
            path.chmod(path.stat().st_mode | (0o700 if path.is_dir() else 0o600))
        for name, expected in files.items():
            if digest(copied / name) != expected:
                raise SystemExit(f"bundle source changed during copy: {name}")
        (copied / marker.name).write_text(json.dumps(origin, indent=2) + "\n")
        copied.rename(target)
print(target)
PY
}

prepare_blackwell_overlay() {
    local source_item
    local target_item
    local current_target
    local -a items=(
        torch
        torch-2.13.0.dist-info
        functorch
        torchgen
        torchvision
        torchvision-0.28.0.dist-info
        torchvision.libs
        triton
        triton-3.7.1.dist-info
        cuda
        cuda_bindings-13.3.1.dist-info
        cuda_core-1.0.1.dist-info
        cuda_pathfinder-1.6.0.dist-info
        cuda_python-13.3.1.dist-info
        cuda_tile-1.5.0.dist-info
        cuda_toolkit-13.0.3.0.dist-info
        nvidia
    )

    [[ -x "${BLACKWELL_SOURCE_ENV}/bin/python" ]] || {
        echo "Blackwell Torch source environment is missing: ${BLACKWELL_SOURCE_ENV}" >&2
        return 1
    }
    "${BLACKWELL_SOURCE_ENV}/bin/python" - <<'PY'
import sys
import torch
import torchvision

assert sys.version_info[:2] == (3, 12)
assert torch.__version__ == "2.13.0+cu130"
assert torchvision.__version__ == "0.28.0+cu130"
assert torch.version.cuda == "13.0"
assert "sm_120" in torch.cuda.get_arch_list()
PY
    mkdir -p "${BLACKWELL_OVERLAY}"
    for source_item in "${items[@]}"; do
        [[ -e "${BLACKWELL_SOURCE_SITE}/${source_item}" ]] || {
            echo "Blackwell Torch source item is missing: ${source_item}" >&2
            return 1
        }
        target_item="${BLACKWELL_OVERLAY}/${source_item}"
        if [[ -L "${target_item}" ]]; then
            current_target="$(readlink "${target_item}")"
            if [[ "${current_target}" != "${BLACKWELL_SOURCE_SITE}/${source_item}" ]]; then
                ln -sfn "${BLACKWELL_SOURCE_SITE}/${source_item}" "${target_item}"
            fi
        elif [[ -e "${target_item}" ]]; then
            echo "Blackwell overlay target exists and is not a symlink: ${target_item}" >&2
            return 1
        else
            ln -s "${BLACKWELL_SOURCE_SITE}/${source_item}" "${target_item}"
        fi
    done
    echo "Blackwell Torch overlay ready: ${BLACKWELL_OVERLAY}"
}

for required in \
    "${BUNDLE}/deploy_input.json" \
    "${BUNDLE}/contract.json" \
    "${BUNDLE}/package_manifest.json" \
    "${BUNDLE}/checkpoint/model.safetensors" \
    "${POLICY_ROOT}/pyproject.toml" \
    "${POLICY_ROOT}/uv.lock"; do
    [[ -f "${required}" ]] || {
        echo "Invalid desktop-cleanup bundle; missing ${required}" >&2
        exit 1
    }
done
[[ -n "${UV_BIN}" && -x "${UV_BIN}" ]] || {
    echo "uv is required to build the model and robot environments" >&2
    exit 1
}
[[ "${FORCE_MODEL_SYNC}" == "0" || "${FORCE_MODEL_SYNC}" == "1" ]] || {
    echo "PI05_FORCE_MODEL_SYNC must be 0 or 1" >&2
    exit 1
}

# Verify the complete publisher and OSS inventories before consuming weights.
VERIFY_ARGS=()
[[ -z "${BUNDLE_FULL_VERIFY_ARG}" ]] || VERIFY_ARGS+=("${BUNDLE_FULL_VERIFY_ARG}")
PYTHONPATH="${XPL_ROOT}:${PYTHONPATH:-}" /usr/bin/python3 \
    "${BUNDLE_VERIFIER}" "${BUNDLE}" "${VERIFY_ARGS[@]}"

# Mutable model dependencies stay outside the immutable deployment bundle.
# gym-aloha/dm-control/mujoco are training/simulator dependencies. The pinned
# mujoco 2.3.7 has no Python 3.12 wheel and is not imported by this inference
# path, so a deployment host must not need a separate MUJOCO_PATH installation.
MODEL_BASE_READY=0
if [[ "${FORCE_MODEL_SYNC}" == "0" && -x "${MODEL_ENV}/bin/python" ]]; then
    if "${MODEL_ENV}/bin/python" - <<'PY' >/dev/null 2>&1
import sys
from importlib import metadata

assert sys.version_info[:2] == (3, 12)
expected = {
    "jax": "0.5.3",
    "safetensors": "0.5.3",
    "transformers": "4.53.2",
    "websockets": "15.0.1",
}
for package, version in expected.items():
    assert metadata.version(package) == version
import torch
import torchvision
PY
    then
        MODEL_BASE_READY=1
        echo "Reusing verified ${SETUP_LABEL} model base environment: ${MODEL_ENV}"
    fi
fi
if [[ "${MODEL_BASE_READY}" == "0" ]]; then
    SETUP_POLICY_ROOT="$(prepare_model_setup_source)"
    UV_PROJECT_ENVIRONMENT="${MODEL_ENV}" \
    GIT_LFS_SKIP_SMUDGE=1 \
    UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-600}" \
    UV_CONCURRENT_DOWNLOADS="${UV_CONCURRENT_DOWNLOADS:-8}" \
        "${UV_BIN}" sync \
            --project "${SETUP_POLICY_ROOT}" \
            --python "${MODEL_PYTHON}" \
            --frozen \
            --no-dev \
            --no-install-project \
            --no-install-package lerobot \
            --no-install-package gym-aloha \
            --no-install-package dm-control \
            --no-install-package mujoco
fi

# The publisher lock resolves a CUDA 12.6 Torch wheel with no sm_120 kernels.
# Keep that environment unchanged. On this workstation only, prepend a
# selective overlay from the existing Python 3.12 Qwen runtime. The overlay
# contains matching Torch/Torchvision/Triton/CUDA 13 packages and deliberately
# excludes Qwen's Transformers and NumPy so the Pi0.5 bundle retains its pinned
# model dependencies. The checkpoint bundle and both source environments stay
# immutable.
TORCH_ARCH_STATUS="$("${MODEL_ENV}/bin/python" - <<'PY'
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
TORCH_OVERLAY=""
if [[ "${TORCH_ARCH_STATUS}" =~ ^sm_12[0-9]*:unsupported$ ]]; then
    prepare_blackwell_overlay
    TORCH_OVERLAY="${BLACKWELL_OVERLAY}"
fi

PYTHONPATH="${TORCH_OVERLAY}${TORCH_OVERLAY:+:${PYTHONPATH:-}}" \
"${MODEL_ENV}/bin/python" - <<'PY'
import torch

if not torch.cuda.is_available():
    print("CUDA runtime check skipped: no CUDA device is visible during setup")
else:
    major, minor = torch.cuda.get_device_capability()
    wanted = f"sm_{major}{minor}"
    supported = torch.cuda.get_arch_list()
    if wanted not in supported:
        raise SystemExit(
            f"Installed Torch {torch.__version__} does not support {wanted}: {supported}"
        )
    value = (torch.ones(8, device="cuda") * 2).sum()
    torch.cuda.synchronize()
    if value.item() != 16:
        raise SystemExit("CUDA Torch smoke calculation returned the wrong value")
    print(
        f"CUDA Torch runtime passed: torch={torch.__version__} "
        f"gpu={torch.cuda.get_device_name(0)} arch={wanted}"
    )
PY

# OpenPI's PyTorch Pi0.5 backend requires its pinned Transformers replacements
# inside the mutable site-packages tree. The source comes from the verified
# bundle; no deployment-bundle file is changed.
TRANSFORMERS_REPLACE="${POLICY_ROOT}/src/openpi/models_pytorch/transformers_replace"
[[ -d "${TRANSFORMERS_REPLACE}" ]] || {
    echo "OpenPI transformers replacement is missing: ${TRANSFORMERS_REPLACE}" >&2
    exit 1
}
TRANSFORMERS_PACKAGE="$("${MODEL_ENV}/bin/python" - <<'PY'
from pathlib import Path
import transformers
print(Path(transformers.__file__).resolve().parent)
PY
)"
cp -R "${TRANSFORMERS_REPLACE}/." "${TRANSFORMERS_PACKAGE}/"

"${UV_BIN}" pip install \
    --python "${MODEL_ENV}/bin/python" \
    --no-deps \
    --editable "${XPL_ROOT}"
# The bundle already supplies the rest of the websocket stack. Install this
# small codec explicitly because --no-deps above intentionally avoids letting
# the harness resolver replace the checkpoint's pinned NumPy/Transformers set.
"${UV_BIN}" pip install \
    --python "${MODEL_ENV}/bin/python" \
    --no-deps \
    "msgpack-numpy==0.4.8"

# The robot environment is separate and installs the repository-pinned
# pyAgxArm snapshot. This does not open CAN, serial, or camera devices.
bash "${XPL_ROOT}/scripts/setup_robot_sdks.sh" verify pyagxarm
UV_PROJECT_ENVIRONMENT="${ROBOT_ENV}" \
    "${UV_BIN}" sync \
        --project "${XPL_ROOT}" \
        --frozen \
        --extra piper-x-dual \
        --no-dev

# QNexo and the Dora CLI live in the graph environment. The current Piper
# qualification graph has no recorder node, so a Rust toolchain is not needed.
UV_PROJECT_ENVIRONMENT="${DAGGER_ENV}" \
    "${UV_BIN}" sync \
        --project "${XPL_ROOT}/runtime/robot_dagger" \
        --frozen \
        --no-dev
"${UV_BIN}" pip install \
    --python "${DAGGER_ENV}/bin/python" \
    --default-index "${PACKAGE_INDEX}" \
    --no-deps \
    "dora-rs-cli==0.5.0"
"${DAGGER_ENV}/bin/dora" --version

PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="${TORCH_OVERLAY}${TORCH_OVERLAY:+:}${XPL_ROOT}:${BUNDLE}/runtime/project/src:${POLICY_ROOT}/src:${POLICY_ROOT}/packages/openpi-client/src:${BUNDLE}/runtime/lerobot" \
    "${MODEL_ENV}/bin/python" - <<'PY'
import openpi
import websockets
from openpi.models_pytorch.pi0_pytorch import PI0Pytorch

print("Pi0.5 model imports passed")
PY

echo "Model environment ready: ${MODEL_ENV}"
echo "Robot environment ready: ${ROBOT_ENV}"
echo "Dora/QNexo environment ready: ${DAGGER_ENV}"
[[ -z "${TORCH_OVERLAY}" ]] || echo "Blackwell Torch overlay active: ${TORCH_OVERLAY}"
echo "Pinned pyAgxArm identity verified"
echo "No robot, CAN, serial, or camera device was opened"
