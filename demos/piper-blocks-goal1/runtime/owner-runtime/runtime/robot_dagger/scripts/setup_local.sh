#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPOSITORY_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"

UV_BIN="${UV_BIN:-$(command -v uv || true)}"
if [[ -z "${UV_BIN}" && -x /home/ubuntu/miniconda3/envs/infra/bin/uv ]]; then
  UV_BIN=/home/ubuntu/miniconda3/envs/infra/bin/uv
fi
[[ -n "${UV_BIN}" && -x "${UV_BIN}" ]] || {
  echo "uv is required to install the robot DAgger Python runtime." >&2
  exit 1
}
LOCAL_CARGO_BIN="${REPOSITORY_ROOT}/.xpolicy/cargo/bin/cargo"
CARGO_BIN="${CARGO_BIN:-$(command -v cargo || true)}"
if [[ -z "${CARGO_BIN}" && -x "${LOCAL_CARGO_BIN}" ]]; then
  CARGO_BIN="${LOCAL_CARGO_BIN}"
fi
if [[ -n "${CARGO_BIN}" && "$(realpath -m "${CARGO_BIN}")" == "${LOCAL_CARGO_BIN}" ]]; then
  export CARGO_HOME="${REPOSITORY_ROOT}/.xpolicy/cargo"
  export RUSTUP_HOME="${REPOSITORY_ROOT}/.xpolicy/rustup"
fi
[[ -n "${CARGO_BIN}" && -x "${CARGO_BIN}" ]] || {
  echo "cargo is required to build the robot DAgger recorder." >&2
  exit 1
}

cd "${PROJECT_ROOT}"
"${UV_BIN}" sync --frozen
"${CARGO_BIN}" build -p fourier-record --release

echo "Robot DAgger runtime ready: ${PROJECT_ROOT}/.venv"
echo "Robot DAgger recorder ready: ${PROJECT_ROOT}/target/release/fourier-record"
