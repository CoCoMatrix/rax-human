#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BOOTSTRAP="${REPO_ROOT}/third_party/robot_sdks/bootstrap.py"

if (($# == 0)); then
  set -- prepare bundled
fi

python3 "${BOOTSTRAP}" "$@"
