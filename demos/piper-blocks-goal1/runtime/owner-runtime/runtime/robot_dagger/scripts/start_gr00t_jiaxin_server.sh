#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
if (($#)); then
  echo "This entry now starts the XPolicy server; configure it with GR00T_* variables." >&2
  exit 2
fi
exec "${ROOT}/scripts/gr3.sh" serve gr00t
