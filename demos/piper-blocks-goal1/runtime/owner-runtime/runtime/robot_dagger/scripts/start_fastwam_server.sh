#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SERVE="${XPL_ROOT}/policy/FastWAM/serve.sh"
LOG_PATH="${FASTWAM_LOG_PATH:-${XPL_ROOT}/runtime/robot_dagger/logs/fastwam-server.log}"
PID_PATH="${FASTWAM_PID_PATH:-${XPL_ROOT}/runtime/robot_dagger/logs/fastwam-server.pid}"

case "${1:-}" in
  --dry-run)
    echo "CMD: ${SERVE}"
    exit 0
    ;;
  --background)
    mkdir -p "$(dirname "${LOG_PATH}")" "$(dirname "${PID_PATH}")"
    setsid -f bash -c 'echo "$$" > "$1"; exec "$2"' fastwam-server "${PID_PATH}" "${SERVE}" >"${LOG_PATH}" 2>&1 < /dev/null
    sleep 0.2
    echo "FastWAM XPolicyLab server starting: $(cat "${PID_PATH}") (${LOG_PATH})"
    exit 0
    ;;
  "")
    exec "${SERVE}"
    ;;
  *)
    echo "Usage: $0 [--dry-run|--background]" >&2
    exit 2
    ;;
esac
