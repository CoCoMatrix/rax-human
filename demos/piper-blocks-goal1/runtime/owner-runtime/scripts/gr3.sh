#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMAND="${1:-}"
MODEL="${2:-}"

usage() {
  echo "Usage:" >&2
  echo "  $0 serve {gr00t|fastwam|pi05|turbovla}" >&2
  echo "  $0 dagger {gr00t|fastwam|pi05|turbovla} [--dry-run]" >&2
  echo "  $0 replay {gr00t|fastwam|pi05|turbovla} EPISODE [replay options]" >&2
  echo "  $0 replay-all EPISODE [replay options]" >&2
  exit 2
}

normalize_model() {
  case "$1" in
    gr00t|jiaxin) echo gr00t ;;
    fastwam|pi05|turbovla|turbo) [[ "$1" == "turbo" ]] && echo turbovla || echo "$1" ;;
    *) usage ;;
  esac
}

serve() {
  local model
  model="$(normalize_model "$1")"
  export GR3_POLICY_HOST="${GR3_POLICY_HOST:-127.0.0.1}"
  export GR3_POLICY_PORT="${GR3_POLICY_PORT:-19000}"
  case "${model}" in
    gr00t)
      export GR00T_POLICY_ROOT="${GR00T_POLICY_ROOT:-${ROOT}/runtime/gr00t/source}"
      export GR00T_PYTHON="${GR00T_PYTHON:-${ROOT}/runtime/gr00t/.venv/bin/python}"
      export GR00T_MODEL_PATH="${GR00T_MODEL_PATH:-/home/fourier/jiaxin/728-v5}"
      export GR00T_HOST="${GR3_POLICY_HOST}"
      export GR00T_PORT="${GR3_POLICY_PORT}"
      exec "${ROOT}/policy/GR00T_Jiaxin/serve.sh"
      ;;
    fastwam)
      export FASTWAM_HOST="${GR3_POLICY_HOST}"
      export FASTWAM_PORT="${GR3_POLICY_PORT}"
      exec "${ROOT}/policy/FastWAM/serve.sh"
      ;;
    pi05)
      export PI05_HOST="${GR3_POLICY_HOST}"
      export PI05_PORT="${GR3_POLICY_PORT}"
      exec "${ROOT}/policy/Pi_05/serve.sh"
      ;;
    turbovla)
      export TURBOVLA_HOST="${GR3_POLICY_HOST}"
      export TURBOVLA_PORT="${GR3_POLICY_PORT}"
      exec "${ROOT}/policy/TurboVLA/serve.sh"
      ;;
  esac
}

wait_server() {
  local pid="$1" log="$2" started="${SECONDS}"
  local timeout="${GR3_REPLAY_SERVER_TIMEOUT_SECONDS:-900}"
  while ((SECONDS - started < timeout)); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" || true
      cat "${log}" >&2
      return 1
    fi
    if "${ROOT}/.venv/bin/python" - "${GR3_POLICY_HOST}" "${GR3_POLICY_PORT}" >/dev/null 2>&1 <<'PY'
import sys
from websockets.sync.client import connect
with connect(f"ws://{sys.argv[1]}:{sys.argv[2]}", open_timeout=0.5, close_timeout=0.5):
    pass
PY
    then
      return 0
    fi
    sleep 2
  done
  echo "Timed out waiting for policy server; log: ${log}" >&2
  return 1
}

case "${COMMAND}" in
  serve)
    [[ -n "${MODEL}" ]] || usage
    serve "${MODEL}"
    ;;
  dagger)
    [[ -n "${MODEL}" ]] || usage
    shift 2
    exec "${ROOT}/runtime/robot_dagger/scripts/run_policy_dagger.sh" \
      "$(normalize_model "${MODEL}")" "$@"
    ;;
  replay)
    [[ -n "${MODEL}" && -n "${3:-}" ]] || usage
    normalize_model "${MODEL}" >/dev/null
    EPISODE="$3"
    shift 3
    export GR3_POLICY_HOST="${GR3_POLICY_HOST:-127.0.0.1}"
    export GR3_POLICY_PORT="${GR3_POLICY_PORT:-19000}"
    exec "${ROOT}/.venv/bin/python" -m XPolicyLab.real_world.robots.gr3.replay \
      "${EPISODE}" --url "ws://${GR3_POLICY_HOST}:${GR3_POLICY_PORT}" "$@"
    ;;
  replay-all)
    [[ -n "${MODEL}" ]] || usage
    EPISODE="${MODEL}"
    shift 2
    export GR3_POLICY_HOST="${GR3_POLICY_HOST:-127.0.0.1}"
    export GR3_POLICY_PORT="${GR3_POLICY_PORT:-19000}"
    LOG_DIR="${GR3_REPLAY_LOG_DIR:-/tmp/xpolicylab-gr3-replay-$$}"
    mkdir -p "${LOG_DIR}"
    SERVER_PID=""
    cleanup() {
      if [[ -n "${SERVER_PID}" ]] && kill -0 "${SERVER_PID}" 2>/dev/null; then
        kill -TERM "${SERVER_PID}" 2>/dev/null || true
        wait "${SERVER_PID}" 2>/dev/null || true
      fi
    }
    trap cleanup EXIT INT TERM
    for policy in gr00t fastwam pi05 turbovla; do
      log="${LOG_DIR}/${policy}.log"
      echo "=== ${policy} ==="
      "${BASH_SOURCE[0]}" serve "${policy}" >"${log}" 2>&1 &
      SERVER_PID=$!
      wait_server "${SERVER_PID}" "${log}"
      "${BASH_SOURCE[0]}" replay "${policy}" "${EPISODE}" "$@"
      cleanup
      SERVER_PID=""
    done
    echo "Server logs: ${LOG_DIR}"
    ;;
  *)
    usage
    ;;
esac
