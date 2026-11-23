#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
exec "${RAX_PYTHON:-python3}" -m harness.codex_cli.launcher "$@"
