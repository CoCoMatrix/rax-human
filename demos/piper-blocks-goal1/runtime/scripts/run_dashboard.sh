#!/usr/bin/env bash
# Trusted operator dashboard (loopback only):
#   bash scripts/run_dashboard.sh [--host 127.0.0.1] [--port 8770]
# Open the printed tokenized URL. Remote: ssh -L 8770:127.0.0.1:8770 <workstation>.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON="${RAX_PYTHON:-$(command -v python3 || true)}"
[[ -x "$PYTHON" ]] || { echo "No robot Python: set RAX_PYTHON to the interpreter of the robot environment (got '$PYTHON')" >&2; exit 2; }
export PYTHONPATH="$PROJECT_ROOT"
cd "$PROJECT_ROOT"
exec "$PYTHON" -m services.dashboard.server "$@"
