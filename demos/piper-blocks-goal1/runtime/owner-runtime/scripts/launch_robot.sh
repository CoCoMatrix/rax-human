#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

usage() {
  cat <<'EOF'
Usage:
  scripts/launch_robot.sh list
  scripts/launch_robot.sh <gr00t|fastwam|pi05|turbovla> [options]
  scripts/launch_robot.sh astribot serve
  scripts/launch_robot.sh astribot policy [--record|--no-record] [--dry-run] [--duration SECONDS]
  scripts/launch_robot.sh astribot dagger [--record|--no-record] [--dry-run] [--duration SECONDS]
  scripts/launch_robot.sh astribot <command> [Astribot CLI options]
  scripts/launch_robot.sh piper_x_dual [--deployment NAME] [--dagger-mode off|mixed|data] [--dagger-expert-source qnexo|leader_follower] [--record-dir PATH] [--collection-batch-id ID] [--dagger-iteration N] [--control-source both|qnexo|keyboard] [--qnexo-port PATH] [--operator-ui-port PORT] [--open-ui|--no-open-ui] [--model-filter|--no-model-filter] [--model-filter-alpha ALPHA] [--enable-write|--no-write] [--model-server managed|external] [--model-host HOST] [--model-port PORT] [--model-device DEVICE] [--model-ready-timeout SECONDS] [--policy-url URL] [--task TEXT] [--ckpt PATH] [--initialize-left-gripper-width METERS] [--duration SECONDS]
  python policy/Pi_05/piper_deployments.py list

Piper maintenance: --release-software-stop left_arm|right_arm --enable-write --duration SECONDS
  Sends one software-stop resume request, then exits owner without enabling or moving.

GR3 options:
  --ckpt PATH
  --task TEXT
  --robot-id ID
  --chunk N
  --host HOST
  --port PORT
  --record | --no-record
  --record-dir PATH
  --max-joint-step RAD
  --freeze-waist | --no-freeze-waist
  --freeze-waist-joints CSV
  --rtc | --no-rtc
  --mink off|shadow|enforce
  --action-qp | --no-action-qp
  --spline | --no-spline
  --filter | --no-filter
  --enable-write | --no-write
  --dry-run

Astribot commands and options are passed to scripts/astribot_s1.sh. Hardware
writes still require its explicit acknowledgement and --enable-write flags.
EOF
}

list_chains() {
  cat <<'EOF'
ROBOT     CHAIN/COMMAND       RUNTIME
gr3       gr00t              unified gr3-policy-dagger.yml
gr3       fastwam            unified gr3-policy-dagger.yml
gr3       pi05               unified gr3-policy-dagger.yml
gr3       turbovla           unified gr3-policy-dagger.yml
astribot  doctor             SDK/environment diagnosis
astribot  serve              Astribot Pi0.5 PolicyServer only; no robot connection
astribot  owner-shadow       25D owner runtime, no hardware writes
astribot  policy             formal policy graph, recording disabled by default
astribot  dagger             same formal graph, DAgger recording enabled by default
astribot  teleop             joint-increment teleoperation
astribot  full-body-smoke    supervised 25D SDK smoke
EOF
  /usr/bin/python3 "${SCRIPT_DIR}/../policy/Pi_05/piper_deployments.py" list
}

die() {
  echo "launch_robot.sh: $*" >&2
  exit 2
}

need_value() {
  (($# >= 2)) || die "$1 requires a value"
}

append_env() {
  [[ -z "$2" ]] || ROBOT_ENV+=("$1=$2")
}

piper_process_belongs_to_repo() {
  local process_cwd
  [[ -n "${REPO_ROOT:-}" && "$1" =~ ^[1-9][0-9]*$ ]] || return 1
  # Dora nodes run inside this repository. Unknown, deleted, or inaccessible
  # process directories are never sufficient evidence to signal a process.
  process_cwd="$(readlink -e -- "/proc/$1/cwd" 2>/dev/null)" || return 1
  [[ "${process_cwd}" == "${REPO_ROOT}" || "${process_cwd}" == "${REPO_ROOT}/"* ]]
}

cleanup_stale_piper_operator_ui() {
  local port="${PIPER_OPERATOR_UI_PORT:-8091}"
  local pid
  local command
  local active_nodes
  local -a listeners=()
  while IFS= read -r pid; do
    [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || continue
    listeners+=("${pid}")
  done < <(
    ss -H -ltnp "sport = :${port}" 2>/dev/null \
      | sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' \
      | sort -u
  )
  ((${#listeners[@]})) || return 0

  active_nodes="$(
    pgrep -u "$(id -u)" -f \
      'XPolicyLab\.real_world\.(cameras\.realsense_dora_node|robots\.piper_x_dual\.(dora_owner_node|policy_node|control_mux_node|operator_ui_node|retarget_node))$' \
      || true
  )"
  if [[ -n "${active_nodes}" ]]; then
    die "operator UI port 127.0.0.1:${port} is used by an active Piper graph (PIDs: ${active_nodes//$'\n'/, }); stop it before launching a second graph"
  fi

  for pid in "${listeners[@]}"; do
    [[ -r "/proc/${pid}/cmdline" ]] || continue
    command="$(tr '\0' ' ' < "/proc/${pid}/cmdline")"
    if ! piper_process_belongs_to_repo "${pid}"; then
      die "operator UI port 127.0.0.1:${port} is used by a process outside this repository (PID ${pid}); stop it or choose --operator-ui-port"
    fi
    if [[ "${command}" == *"XPolicyLab.real_world.robots.piper_x_dual.operator_ui_node"* \
       || "${command}" == *"policy/Pi05Cup/ui.py"* \
       || "${command}" == *"Pi05Cup.ui"* ]]; then
      echo "Stopping stale Piper operator UI on 127.0.0.1:${port}: ${pid}" >&2
      stop_piper_child "${pid}"
    else
      die "operator UI port 127.0.0.1:${port} is used by an unknown repository process (PID ${pid}); stop it or choose --operator-ui-port"
    fi
  done
}

cleanup_stale_piper_camera_node() {
  local user_id
  local sibling_pids
  local pid
  local found_camera=0
  local -a camera_pids=()
  user_id="$(id -u)"
  while IFS= read -r pid; do
    found_camera=1
    if piper_process_belongs_to_repo "${pid}"; then
      camera_pids+=("${pid}")
    fi
  done < <(
    pgrep -u "${user_id}" -f \
      'XPolicyLab\.real_world\.cameras\.realsense_dora_node$' || true
  )
  ((found_camera)) || return 0

  sibling_pids="$(
    pgrep -u "${user_id}" -f \
      'XPolicyLab\.real_world\.robots\.piper_x_dual\.(dora_owner_node|policy_node|control_mux_node|operator_ui_node)$' \
      || true
  )"
  if [[ -n "${sibling_pids}" ]]; then
    die "another Piper dataflow is already running (PIDs: ${sibling_pids//$'\n'/, }); stop it before launching a second graph"
  fi
  ((${#camera_pids[@]})) || return 0

  echo "Stopping stale Piper RealSense node from a completed dataflow: ${camera_pids[*]}"
  for pid in "${camera_pids[@]}"; do
    piper_process_belongs_to_repo "${pid}" || continue
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    local any_alive=0
    for pid in "${camera_pids[@]}"; do
      piper_process_belongs_to_repo "${pid}" || continue
      if kill -0 "${pid}" 2>/dev/null; then
        any_alive=1
        break
      fi
    done
    ((any_alive)) || return 0
    sleep 0.1
  done
  die "stale Piper RealSense node did not exit; stop PID(s) ${camera_pids[*]} before retrying"
}

piper_tcp_port_in_use() {
  /usr/bin/python3 - "$1" "$2" <<'PY' >/dev/null 2>&1
import errno
import socket
import sys

host = socket.gethostbyname(sys.argv[1])
port = int(sys.argv[2])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.bind((host, port))
except OSError as exc:
    raise SystemExit(0 if exc.errno == errno.EADDRINUSE else 2)
finally:
    sock.close()
raise SystemExit(1)
PY
}

piper_model_server_ready() {
  /usr/bin/python3 - "$1" "$2" <<'PY' >/dev/null 2>&1
import base64
import os
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
key = base64.b64encode(os.urandom(16)).decode("ascii")
request = (
    "GET / HTTP/1.1\r\n"
    f"Host: {host}:{port}\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    f"Sec-WebSocket-Key: {key}\r\n"
    "Sec-WebSocket-Version: 13\r\n\r\n"
).encode("ascii")

try:
    with socket.create_connection((host, port), timeout=0.5) as sock:
        sock.sendall(request)
        response = sock.recv(4096)
        if not response.startswith(b"HTTP/1.1 101"):
            raise SystemExit(1)
        # A client-to-server WebSocket frame must be masked. This is a valid
        # empty CLOSE frame, so readiness checks don't create handshake errors.
        sock.sendall(b"\x88\x80" + os.urandom(4))
        try:
            sock.recv(4096)
        except TimeoutError:
            pass
except OSError:
    raise SystemExit(1)
PY
}

wait_for_piper_model_server() {
  local server_pid="$1"
  local host="$2"
  local port="$3"
  local timeout_seconds="$4"
  local deadline=$((SECONDS + timeout_seconds))
  while ! piper_model_server_ready "${host}" "${port}"; do
    if ! kill -0 "${server_pid}" 2>/dev/null; then
      local server_status=0
      wait "${server_pid}" || server_status=$?
      echo "Managed Piper model server exited before becoming ready (status ${server_status})" >&2
      return 1
    fi
    if ((SECONDS >= deadline)); then
      echo "Managed Piper model server was not ready at ${host}:${port} after ${timeout_seconds}s" >&2
      return 1
    fi
    sleep 0.5
  done
}

cleanup_stale_piper_model_server() {
  local host="$1"
  local port="$2"
  local user_id
  local pid
  local command
  local setup_server_path
  local -a stale_pids=()
  user_id="$(id -u)"
  setup_server_path="$(realpath -m "${REPO_ROOT}/setup_policy_server.py")"
  while IFS= read -r pid; do
    [[ -r "/proc/${pid}/cmdline" ]] || continue
    command="$(tr '\0' ' ' < "/proc/${pid}/cmdline")"
    [[ "${command}" == *"${setup_server_path}"* ]] || continue
    [[ "${command}" == *"host=${host}"* && "${command}" == *"port=${port}"* ]] || continue
    stale_pids+=("${pid}")
  done < <(pgrep -u "${user_id}" -f 'setup_policy_server\.py' || true)
  ((${#stale_pids[@]})) || return 0

  echo "Stopping stale managed Piper model server on ${host}:${port}: ${stale_pids[*]}"
  for pid in "${stale_pids[@]}"; do
    stop_piper_child "${pid}"
  done
}

stop_piper_child() {
  local child_pid="${1:-}"
  [[ -n "${child_pid}" ]] || return 0
  kill -0 "${child_pid}" 2>/dev/null || {
    wait "${child_pid}" 2>/dev/null || true
    return 0
  }
  kill -TERM "${child_pid}" 2>/dev/null || true
  for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
    kill -0 "${child_pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${child_pid}" 2>/dev/null; then
    kill -KILL "${child_pid}" 2>/dev/null || true
  fi
  wait "${child_pid}" 2>/dev/null || true
}

cleanup_orphaned_piper_graph_nodes() {
  local user_id
  local pid
  local any_alive
  local -a node_pids=()
  user_id="$(id -u)"
  while IFS= read -r pid; do
    if piper_process_belongs_to_repo "${pid}"; then
      node_pids+=("${pid}")
    fi
  done < <(
    pgrep -u "${user_id}" -f \
      'XPolicyLab\.real_world\.(cameras\.realsense_dora_node|robots\.piper_x_dual\.(dora_owner_node|policy_node|control_mux_node|operator_ui_node|retarget_node))$' \
      || true
  )
  ((${#node_pids[@]})) || return 0
  echo "Stopping orphaned Piper graph node(s): ${node_pids[*]}" >&2
  for pid in "${node_pids[@]}"; do
    piper_process_belongs_to_repo "${pid}" || continue
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    any_alive=0
    for pid in "${node_pids[@]}"; do
      piper_process_belongs_to_repo "${pid}" || continue
      if kill -0 "${pid}" 2>/dev/null; then
        any_alive=1
        break
      fi
    done
    ((any_alive)) || break
    sleep 0.1
  done
  for pid in "${node_pids[@]}"; do
    piper_process_belongs_to_repo "${pid}" || continue
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
  done
}

cleanup_managed_piper_processes() {
  local cleanup_status=$?
  trap - EXIT
  stop_piper_child "${PIPER_DORA_PID:-}"
  cleanup_orphaned_piper_graph_nodes
  stop_piper_child "${PIPER_MODEL_SERVER_PID:-}"
  return "${cleanup_status}"
}

if [[ $# -eq 0 || "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
if [[ "$1" == "list" ]]; then
  (($# == 1)) || die "list takes no options"
  list_chains
  exit 0
fi
if [[ "$1" == "astribot" ]]; then
  shift
  (($#)) || die "astribot requires a command"
  if [[ "$1" == "doctor" ]]; then
    (($# == 1)) || die "astribot doctor takes no options"
    exec "${SCRIPT_DIR}/setup_astribot_host.sh" doctor
  fi
  if [[ "$1" == "serve" ]]; then
    (($# == 1)) || die "astribot serve takes no options; use PI05_* environment variables"
    exec "${SCRIPT_DIR}/../policy/Pi_05/serve_astribot.sh"
  fi
  if [[ "$1" == "policy" || "$1" == "dagger" ]]; then
    ASTRIBOT_RUN_MODE="$1"
    shift
    exec "${SCRIPT_DIR}/../runtime/robot_dagger/scripts/run_astribot_dagger.sh" \
      --mode "${ASTRIBOT_RUN_MODE}" "$@"
  fi
  ASTRIBOT_DRY_RUN=0
  ASTRIBOT_ARGS=()
  for argument in "$@"; do
    if [[ "${argument}" == "--dry-run" ]]; then
      ASTRIBOT_DRY_RUN=1
    else
      ASTRIBOT_ARGS+=("${argument}")
    fi
  done
  if [[ "${ASTRIBOT_DRY_RUN}" == "1" ]]; then
    printf 'CMD:'
    printf ' %q' "${SCRIPT_DIR}/astribot_s1.sh" "${ASTRIBOT_ARGS[@]}"
    printf '\n'
    exit 0
  fi
  exec "${SCRIPT_DIR}/astribot_s1.sh" "${ASTRIBOT_ARGS[@]}"
fi
if [[ "$1" == "piper_x_dual" || "$1" == "piper-x-dual" ]]; then
  shift
  PIPER_WRITE=0
  PIPER_DEPLOYMENT="${PIPER_DEPLOYMENT:-build-blocks}"
  PIPER_DURATION=""
  PIPER_EXECUTE_STEPS_OVERRIDE=""
  PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M=""
  PIPER_RELEASE_SOFTWARE_STOP_ARM=""
  PIPER_POLICY_URL_WAS_SET=0
  [[ -z "${PIPER_POLICY_URL:-}" ]] || PIPER_POLICY_URL_WAS_SET=1
  PIPER_POLICY_URL="${PIPER_POLICY_URL:-ws://127.0.0.1:19003}"
  PIPER_TASK="${PIPER_TASK:-}"
  PIPER_ROBOT_ID="${PIPER_ROBOT_ID:-piper_x_dual_01}"
  PIPER_MODEL_CHECKPOINT="${PIPER_MODEL_CHECKPOINT:-}"
  PIPER_MODEL_SERVER_MODE="${PIPER_MODEL_SERVER_MODE:-}"
  PIPER_MODEL_HOST="${PIPER_MODEL_HOST:-127.0.0.1}"
  PIPER_MODEL_PORT="${PIPER_MODEL_PORT:-19003}"
  PIPER_MODEL_DEVICE="${PIPER_MODEL_DEVICE:-cuda}"
  PIPER_MODEL_READY_TIMEOUT="${PIPER_MODEL_READY_TIMEOUT:-300}"
  PIPER_CONTROL_SOURCE="${PIPER_CONTROL_SOURCE:-}"
  PIPER_QNEXO_PORT="${PIPER_QNEXO_PORT:-${QNEXO_PORT:-}}"
  PIPER_CONTROL_SOURCE_WAS_SET=0
  [[ -z "${PIPER_CONTROL_SOURCE}" ]] || PIPER_CONTROL_SOURCE_WAS_SET=1
  PIPER_DAGGER_MODE="${PIPER_DAGGER_MODE:-}"
  PIPER_DAGGER_EXPERT_SOURCE="${PIPER_DAGGER_EXPERT_SOURCE:-}"
  PIPER_DAGGER_RECORD_DIR="${PIPER_DAGGER_RECORD_DIR:-}"
  PIPER_COLLECTION_BATCH_ID="${PIPER_COLLECTION_BATCH_ID:-}"
  PIPER_DAGGER_ITERATION="${PIPER_DAGGER_ITERATION:-0}"
  PIPER_OPERATOR_UI_PORT="${PIPER_OPERATOR_UI_PORT:-8091}"
  PIPER_OPERATOR_UI_AUTO_OPEN="${PIPER_OPERATOR_UI_AUTO_OPEN:-1}"
  PIPER_MODEL_FILTER_ENABLED="${PIPER_MODEL_FILTER_ENABLED:-1}"
  PIPER_MODEL_FILTER_ALPHA="${PIPER_MODEL_FILTER_ALPHA:-0.6}"
  PIPER_DRY_RUN=0
  while (($#)); do
    case "$1" in
      --deployment) need_value "$@"; PIPER_DEPLOYMENT="$2"; shift 2 ;;
      --execute-steps) need_value "$@"; PIPER_EXECUTE_STEPS_OVERRIDE="$2"; shift 2 ;;
      --enable-write) PIPER_WRITE=1; shift ;;
      --no-write) PIPER_WRITE=0; shift ;;
      --policy-url)
        need_value "$@"
        PIPER_POLICY_URL="$2"
        PIPER_POLICY_URL_WAS_SET=1
        shift 2
        ;;
      --model-server)
        need_value "$@"
        PIPER_MODEL_SERVER_MODE="$2"
        shift 2
        ;;
      --model-host) need_value "$@"; PIPER_MODEL_HOST="$2"; shift 2 ;;
      --model-port) need_value "$@"; PIPER_MODEL_PORT="$2"; shift 2 ;;
      --model-device) need_value "$@"; PIPER_MODEL_DEVICE="$2"; shift 2 ;;
      --model-ready-timeout)
        need_value "$@"
        PIPER_MODEL_READY_TIMEOUT="$2"
        shift 2
        ;;
      --task) need_value "$@"; PIPER_TASK="$2"; shift 2 ;;
      --robot-id) need_value "$@"; PIPER_ROBOT_ID="$2"; shift 2 ;;
      --ckpt) need_value "$@"; PIPER_MODEL_CHECKPOINT="$2"; shift 2 ;;
      --control-source) need_value "$@"; PIPER_CONTROL_SOURCE="$2"; PIPER_CONTROL_SOURCE_WAS_SET=1; shift 2 ;;
      --qnexo-port) need_value "$@"; PIPER_QNEXO_PORT="$2"; shift 2 ;;
      --dagger-mode) need_value "$@"; PIPER_DAGGER_MODE="$2"; shift 2 ;;
      --dagger-expert-source) need_value "$@"; PIPER_DAGGER_EXPERT_SOURCE="$2"; shift 2 ;;
      --record-dir) need_value "$@"; PIPER_DAGGER_RECORD_DIR="$2"; shift 2 ;;
      --collection-batch-id) need_value "$@"; PIPER_COLLECTION_BATCH_ID="$2"; shift 2 ;;
      --dagger-iteration) need_value "$@"; PIPER_DAGGER_ITERATION="$2"; shift 2 ;;
      --operator-ui-port)
        need_value "$@"
        PIPER_OPERATOR_UI_PORT="$2"
        shift 2
        ;;
      --open-ui) PIPER_OPERATOR_UI_AUTO_OPEN=1; shift ;;
      --no-open-ui) PIPER_OPERATOR_UI_AUTO_OPEN=0; shift ;;
      --model-filter) PIPER_MODEL_FILTER_ENABLED=1; shift ;;
      --no-model-filter) PIPER_MODEL_FILTER_ENABLED=0; shift ;;
      --model-filter-alpha)
        need_value "$@"
        PIPER_MODEL_FILTER_ALPHA="$2"
        shift 2
        ;;
      --initialize-left-gripper-width)
        need_value "$@"
        PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M="$2"
        shift 2
        ;;
      --release-software-stop)
        need_value "$@"
        PIPER_RELEASE_SOFTWARE_STOP_ARM="$2"
        shift 2
        ;;
      --duration)
        need_value "$@"
        PIPER_DURATION="$2"
        shift 2
        ;;
      --dry-run) PIPER_DRY_RUN=1; shift ;;
      --help|-h) usage; exit 0 ;;
      *) die "unknown Piper option: $1" ;;
    esac
  done
  if [[ "${PIPER_WRITE}" != "0" && "${PIPER_WRITE}" != "1" ]]; then
    die "ENABLE_WRITE must be 0 or 1"
  fi
  if [[ -n "${PIPER_DURATION}" && ! "${PIPER_DURATION}" =~ ^[1-9][0-9]*$ ]]; then
    die "--duration must be a positive integer number of seconds"
  fi
  if [[ -n "${PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M}" && "${PIPER_WRITE}" != "1" ]]; then
    die "--initialize-left-gripper-width requires --enable-write"
  fi
  if [[ -n "${PIPER_RELEASE_SOFTWARE_STOP_ARM}" ]]; then
    [[ "${PIPER_RELEASE_SOFTWARE_STOP_ARM}" =~ ^(left_arm|right_arm)$ ]] || die "--release-software-stop requires left_arm or right_arm"
    [[ "${PIPER_WRITE}" == "1" ]] || die "--release-software-stop requires --enable-write"
    [[ -z "${PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M}" ]] || die "software-stop release cannot initialize a gripper"
    [[ -n "${PIPER_DURATION}" ]] || die "--release-software-stop requires --duration"
  fi
  [[ "${PIPER_OPERATOR_UI_PORT}" =~ ^[1-9][0-9]{0,4}$ ]] || \
    die "--operator-ui-port must be an integer in [1, 65535]"
  ((PIPER_OPERATOR_UI_PORT <= 65535)) || \
    die "--operator-ui-port must be an integer in [1, 65535]"
  [[ "${PIPER_MODEL_PORT}" =~ ^[1-9][0-9]{0,4}$ ]] || \
    die "--model-port must be an integer in [1, 65535]"
  ((PIPER_MODEL_PORT <= 65535)) || \
    die "--model-port must be an integer in [1, 65535]"
  [[ "${PIPER_MODEL_READY_TIMEOUT}" =~ ^[1-9][0-9]*$ ]] || \
    die "--model-ready-timeout must be a positive integer number of seconds"
  [[ "${PIPER_MODEL_FILTER_ENABLED}" =~ ^[01]$ ]] || \
    die "PIPER_MODEL_FILTER_ENABLED must be 0 or 1"
  /usr/bin/python3 - "${PIPER_MODEL_FILTER_ALPHA}" <<'PY' || \
    die "--model-filter-alpha must be finite and in (0, 1]"
import math
import sys

try:
    alpha = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(alpha) and 0.0 < alpha <= 1.0 else 1)
PY

  REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
  PIPER_RUNTIME="${REPO_ROOT}/runtime/robot_dagger"
  PIPER_DORA_BIN="${DORA_BIN:-$(command -v dora || true)}"
  if [[ -z "${PIPER_DORA_BIN}" && -x "${PIPER_RUNTIME}/.venv/bin/dora" ]]; then
    PIPER_DORA_BIN="${PIPER_RUNTIME}/.venv/bin/dora"
  fi
  PIPER_PROFILE="${PIPER_PROFILE:-${REPO_ROOT}/real_world/robots/piper_x_dual/profile.yml}"
  PIPER_MODEL_SERVER_SCRIPT=""
  PIPER_TASK_CATALOG_PATH=""
  PIPER_OPERATOR_UI_ENABLED=0
  PIPER_DEPLOYMENT_REGISTRY="${REPO_ROOT}/policy/Pi_05/piper_deployments.py"
  [[ -f "${PIPER_DEPLOYMENT_REGISTRY}" ]] || \
    die "Piper deployment registry not found: ${PIPER_DEPLOYMENT_REGISTRY}"
  mapfile -d '' -t PIPER_RECIPE < <(
    /usr/bin/python3 "${PIPER_DEPLOYMENT_REGISTRY}" \
      shell-fields "${PIPER_DEPLOYMENT}"
  )
  ((${#PIPER_RECIPE[@]} == 23)) || \
    die "Piper deployment recipe is invalid: ${PIPER_DEPLOYMENT}"
  PIPER_DEFAULT_MODEL_SERVER_MODE="${PIPER_RECIPE[0]}"
  PIPER_DEFAULT_CONTROL_SOURCE="${PIPER_RECIPE[1]}"
  PIPER_KEYBOARD_GRAPH="${PIPER_RECIPE[2]}"
  PIPER_DAGGER_GRAPH="${PIPER_RECIPE[3]}"
  PIPER_POLICY_LABEL="${PIPER_RECIPE[4]}"
  PIPER_DEFAULT_TASK="${PIPER_RECIPE[5]}"
  PIPER_DEFAULT_MODEL_CHECKPOINT="${PIPER_RECIPE[6]}"
  PIPER_ACTION_FREQUENCY_HZ="${PIPER_RECIPE[7]}"
  PIPER_ACTION_HORIZON="${PIPER_RECIPE[8]}"
  PIPER_EXECUTE_CHUNK_SIZE="${PIPER_RECIPE[9]}"
  PIPER_EXECUTE_CHUNK_MODE="${PIPER_RECIPE[10]}"
  if [[ -n "${PIPER_EXECUTE_STEPS_OVERRIDE}" ]]; then
    PIPER_EXECUTE_CHUNK_SIZE="$(PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}" /usr/bin/python3 \
      -m XPolicyLab.policy.Pi_05.execution_selection "${PIPER_DEPLOYMENT}" "${PIPER_EXECUTE_STEPS_OVERRIDE}")" || \
      die "Deployment does not permit the requested --execute-steps"
    PIPER_EXECUTE_CHUNK_MODE=fixed
  fi
  PIPER_MODEL_SERVER_SCRIPT="${PIPER_RECIPE[11]}"
  PIPER_OPERATOR_UI_ENABLED="${PIPER_RECIPE[12]}"
  PIPER_TASK_CATALOG_RELATIVE="${PIPER_RECIPE[13]}"
  PIPER_BUNDLE_KIND="${PIPER_RECIPE[14]}"
  PIPER_BUNDLE_VERIFIER="${PIPER_RECIPE[15]}"
  PIPER_MODEL_BACKEND="${PIPER_RECIPE[18]:-pi05}"
  PIPER_MODEL_BACKEND="${PIPER_MODEL_BACKEND:-pi05}"
  PIPER_CAMERA_PROFILE="${PIPER_CAMERA_PROFILE:-${PIPER_RECIPE[19]}}"
  PIPER_RELEASE_STATUS="${PIPER_RECIPE[20]}"
  PIPER_DEVICE_INSTANCE="${PIPER_DEVICE_INSTANCE:-${PIPER_RECIPE[21]:-${PIPER_RUNTIME}/config/robot_instances/piper_x_dual_01.yml}}"
  if [[ -n "${PIPER_RECIPE[22]}" ]]; then
    export ACTION_TRAJECTORY_PROFILE="${PIPER_RECIPE[22]}"
    mapfile -t PIPER_TRAJECTORY_FIELDS < <(/usr/bin/python3 - "${ACTION_TRAJECTORY_PROFILE}" <<'PY'
import json, os, sys
p = json.load(open(sys.argv[1]))
assert p["schema"] == "xpolicy.trajectory-profile/v1" and p["algorithm"] == "quintic_rtg_v1"
assert type(p["request_remaining_steps"]) is int and 0 < p["request_remaining_steps"] < p["execution_steps"]
print(p["request_remaining_steps"])
print(p["algorithm"])
print("quintic with arms" if p.get("synchronize_gripper_transition", False) else "bypassed")
scale = p.get("trajectory_time_scale")
if scale is not None:
    assert float(os.environ.get("ACTION_TRAJECTORY_TIME_SCALE", scale)) == scale, "trajectory time scale is pinned"
print("" if scale is None else scale)
PY
    )
    ((${#PIPER_TRAJECTORY_FIELDS[@]} == 4)) || die "Invalid trajectory profile"
    if [[ -n "${PIPER_TRAJECTORY_FIELDS[3]}" ]]; then
      export ACTION_TRAJECTORY_TIME_SCALE="${PIPER_TRAJECTORY_FIELDS[3]}"
    fi
    export ACTION_RTC_MODE=align
    export ACTION_RTC_REQUEST_REMAINING_STEPS="${PIPER_TRAJECTORY_FIELDS[0]}"
    export ACTION_RTC_PRESERVE_STEPS=0 ACTION_RTC_TRANSITION_STEPS=0
    PIPER_MODEL_FILTER_ENABLED=0
    echo "Trajectory: ${PIPER_TRAJECTORY_FIELDS[1]} (pinned profile: ${ACTION_TRAJECTORY_PROFILE}; grippers ${PIPER_TRAJECTORY_FIELDS[2]}; time scale ${ACTION_TRAJECTORY_TIME_SCALE:-1})"
  elif [[ -n "${ACTION_TRAJECTORY_PROFILE:-}" ]]; then
    die "ACTION_TRAJECTORY_PROFILE requires a deployment-bound policy.trajectory_profile"
  fi
  if [[ "${PIPER_WRITE}" == "1" && "${PIPER_RELEASE_STATUS}" == "hardware_free_candidate" ]]; then
    die "deployment ${PIPER_DEPLOYMENT} is hardware-free only; publisher hardware contract is incomplete"
  fi

  if [[ -z "${PIPER_MODEL_SERVER_MODE}" ]]; then
    if [[ "${PIPER_POLICY_URL_WAS_SET}" == "1" ]]; then
      PIPER_MODEL_SERVER_MODE=external
    else
      PIPER_MODEL_SERVER_MODE="${PIPER_DEFAULT_MODEL_SERVER_MODE}"
    fi
  fi
  PIPER_CONTROL_SOURCE="${PIPER_CONTROL_SOURCE:-${PIPER_DEFAULT_CONTROL_SOURCE}}"
  PIPER_DAGGER_EXPERT_SOURCE="${PIPER_DAGGER_EXPERT_SOURCE:-qnexo}"
  PIPER_DAGGER_EXPERT_SOURCE="${PIPER_DAGGER_EXPERT_SOURCE,,}"
  [[ "${PIPER_DAGGER_EXPERT_SOURCE}" =~ ^(qnexo|leader_follower)$ ]] || \
    die "--dagger-expert-source must be qnexo or leader_follower"
  PIPER_DAGGER_MODE="${PIPER_DAGGER_MODE,,}"
  if [[ -z "${PIPER_DAGGER_MODE}" ]]; then
    if [[ "${PIPER_CONTROL_SOURCE}" == "keyboard" ]]; then
      PIPER_DAGGER_MODE=off
    else
      PIPER_DAGGER_MODE=mixed
    fi
  fi
  [[ "${PIPER_DAGGER_MODE}" =~ ^(off|mixed|data)$ ]] || \
    die "--dagger-mode must be off, mixed, or data"
  if [[ "${PIPER_DAGGER_MODE}" != "off" ]]; then
    if [[ "${PIPER_CONTROL_SOURCE_WAS_SET}" == "0" ]]; then
      PIPER_CONTROL_SOURCE=both
    fi
    [[ "${PIPER_CONTROL_SOURCE}" != "keyboard" ]] || \
      die "--dagger-mode ${PIPER_DAGGER_MODE} requires QNexo control (both or qnexo)"
  elif [[ "${PIPER_CONTROL_SOURCE}" != "keyboard" ]]; then
    die "QNexo control requires --dagger-mode mixed or data"
  fi
  [[ "${PIPER_DAGGER_ITERATION}" =~ ^[0-9]+$ ]] || \
    die "--dagger-iteration must be a non-negative integer"
  if [[ -z "${PIPER_DAGGER_RECORD_DIR}" ]]; then
    PIPER_DAGGER_RECORD_DIR="${REPO_ROOT}/Log/dagger/piper_x_dual"
  else
    PIPER_DAGGER_RECORD_DIR="$(realpath -m "${PIPER_DAGGER_RECORD_DIR}")"
  fi
  PIPER_TASK="${PIPER_TASK:-${PIPER_DEFAULT_TASK}}"
  PIPER_MODEL_CHECKPOINT="${PIPER_MODEL_CHECKPOINT:-${PIPER_DEFAULT_MODEL_CHECKPOINT}}"
  if [[ -n "${PIPER_TASK_CATALOG_RELATIVE}" ]]; then
    PIPER_TASK_CATALOG_RELATIVE="${PIPER_TASK_CATALOG_RELATIVE//\{repo_root\}/${REPO_ROOT}}"
    if [[ "${PIPER_TASK_CATALOG_RELATIVE}" == /* ]]; then
      PIPER_TASK_CATALOG_PATH="${PIPER_TASK_CATALOG_RELATIVE}"
    else
      PIPER_TASK_CATALOG_PATH="${PIPER_MODEL_CHECKPOINT}/${PIPER_TASK_CATALOG_RELATIVE}"
    fi
  fi
  if [[ "${PIPER_DAGGER_MODE}" == "off" ]]; then
    [[ -n "${PIPER_KEYBOARD_GRAPH}" ]] || \
      die "deployment ${PIPER_DEPLOYMENT} does not support keyboard-only control"
    PIPER_GRAPH="${PIPER_KEYBOARD_GRAPH}"
  else
    [[ -n "${PIPER_DAGGER_GRAPH}" ]] || \
      die "deployment ${PIPER_DEPLOYMENT} does not support QNexo control"
    PIPER_GRAPH="${PIPER_DAGGER_GRAPH}"
  fi
  [[ "${PIPER_CONTROL_SOURCE}" =~ ^(both|qnexo|keyboard)$ ]] || \
    die "--control-source must be both, qnexo, or keyboard"
  if [[ "${PIPER_DAGGER_MODE}" == "data" ]]; then
    PIPER_MODEL_SERVER_MODE=disabled
  fi
  [[ "${PIPER_MODEL_SERVER_MODE}" =~ ^(managed|external|disabled)$ ]] || \
    die "--model-server must be managed or external"
  if [[ "${PIPER_MODEL_SERVER_MODE}" == "managed" ]]; then
    [[ -n "${PIPER_MODEL_SERVER_SCRIPT}" ]] || \
      die "--model-server managed is not configured for --deployment ${PIPER_DEPLOYMENT}"
    [[ "${PIPER_MODEL_HOST}" == "127.0.0.1" || "${PIPER_MODEL_HOST}" == "localhost" ]] || \
      die "managed Piper model server must bind to 127.0.0.1 or localhost"
    PIPER_MANAGED_POLICY_URL="ws://${PIPER_MODEL_HOST}:${PIPER_MODEL_PORT}"
    if [[ "${PIPER_POLICY_URL_WAS_SET}" == "1" && "${PIPER_POLICY_URL}" != "${PIPER_MANAGED_POLICY_URL}" ]]; then
      die "--policy-url must equal ${PIPER_MANAGED_POLICY_URL} when --model-server managed"
    fi
    PIPER_POLICY_URL="${PIPER_MANAGED_POLICY_URL}"
  fi
  [[ -f "${PIPER_GRAPH}" ]] || die "Piper graph not found: ${PIPER_GRAPH}"
  [[ -f "${PIPER_PROFILE}" ]] || die "Piper profile not found: ${PIPER_PROFILE}"
  [[ -f "${PIPER_DEVICE_INSTANCE}" ]] || die "Piper device instance not found: ${PIPER_DEVICE_INSTANCE}"
  if [[ -n "${PIPER_CAMERA_PROFILE}" ]]; then
    [[ -f "${PIPER_CAMERA_PROFILE}" ]] || \
      die "Piper camera profile not found: ${PIPER_CAMERA_PROFILE}"
  fi
  if [[ "${PIPER_MODEL_SERVER_MODE}" == "managed" ]]; then
    [[ -f "${PIPER_MODEL_SERVER_SCRIPT}" ]] || \
      die "Piper model-server launcher not found: ${PIPER_MODEL_SERVER_SCRIPT}"
  fi
  # Verify the exact configuration that the graph will receive, including defaults.
  export PIPER_PROFILE PIPER_DEVICE_INSTANCE PIPER_CAMERA_PROFILE PIPER_GRAPH
  PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}" /usr/bin/python3 \
    "${PIPER_DEPLOYMENT_REGISTRY}" validate "${PIPER_DEPLOYMENT}" \
      --bundle "${PIPER_MODEL_CHECKPOINT}" \
      --task "${PIPER_TASK}" >/dev/null || \
    die "Piper deployment validation failed: ${PIPER_DEPLOYMENT}"
  if [[ "${PIPER_MODEL_SERVER_MODE}" != "disabled" ]]; then
    [[ "${PIPER_POLICY_URL}" == ws://* || "${PIPER_POLICY_URL}" == wss://* ]] || die "Piper --policy-url must use ws:// or wss://"
  fi

  echo "Graph: ${PIPER_GRAPH}"
  echo "Robot profile: ${PIPER_PROFILE}"
  echo "Device instance: ${PIPER_DEVICE_INSTANCE}"
  if [[ -n "${PIPER_CAMERA_PROFILE}" ]]; then
    echo "Camera profile: ${PIPER_CAMERA_PROFILE}"
  else
    echo "Camera profile: robot profile default"
  fi
  echo "Policy: ${PIPER_POLICY_LABEL}"
  if [[ "${PIPER_MODEL_SERVER_MODE}" == "disabled" ]]; then
    echo "Policy endpoint: disabled (expert-only collection)"
  else
    echo "Policy endpoint: ${PIPER_POLICY_URL}"
  fi
  echo "Checkpoint: ${PIPER_MODEL_CHECKPOINT}"
  if [[ "${PIPER_MODEL_SERVER_MODE}" == "managed" ]]; then
    echo "Model server: managed by this launcher (${PIPER_MODEL_HOST}:${PIPER_MODEL_PORT}, device=${PIPER_MODEL_DEVICE})"
  else
    if [[ "${PIPER_MODEL_SERVER_MODE}" == "disabled" ]]; then
      echo "Model server: disabled"
    else
      echo "Model server: external (${PIPER_POLICY_URL})"
    fi
  fi
  echo "Task: ${PIPER_TASK}"
  echo "ActionChunk: ${PIPER_ACTION_HORIZON} predicted, ${PIPER_EXECUTE_CHUNK_MODE} execution (${PIPER_EXECUTE_CHUNK_SIZE} steps; bundle default ${PIPER_RECIPE[9]})"
  if [[ "${PIPER_MODEL_FILTER_ENABLED}" == "1" ]]; then
    echo "Model low-pass filter: enabled (joint alpha=${PIPER_MODEL_FILTER_ALPHA}, grippers bypassed)"
  else
    echo "Model low-pass filter: disabled"
  fi
  echo "Operator controls: ${PIPER_CONTROL_SOURCE}"
  if [[ "${PIPER_CONTROL_SOURCE}" != "keyboard" ]]; then
    echo "QNexo device: ${PIPER_QNEXO_PORT:-auto (/dev/qnbot or stable /dev/serial/by-id link)}"
  fi
  if [[ "${PIPER_OPERATOR_UI_ENABLED}" == "1" ]]; then
    echo "Operator UI: http://127.0.0.1:${PIPER_OPERATOR_UI_PORT}"
    echo "Operator UI auto-open: ${PIPER_OPERATOR_UI_AUTO_OPEN}"
  fi
  if [[ -n "${PIPER_TASK_CATALOG_PATH}" ]]; then
    echo "Task catalog: ${PIPER_TASK_CATALOG_PATH}"
  fi
  echo "DAgger mode: ${PIPER_DAGGER_MODE}"
  echo "DAgger expert source: ${PIPER_DAGGER_EXPERT_SOURCE}"
  if [[ "${PIPER_DAGGER_MODE}" == "off" ]]; then
    echo "Recording: disabled"
  else
    echo "Recording: enabled (${PIPER_DAGGER_RECORD_DIR})"
    echo "Collection batch: ${PIPER_COLLECTION_BATCH_ID:-auto dataflow id}"
    echo "DAgger iteration: ${PIPER_DAGGER_ITERATION}"
  fi
  if [[ -n "${PIPER_RELEASE_SOFTWARE_STOP_ARM}" ]]; then
    echo "Execution: one software-stop release on ${PIPER_RELEASE_SOFTWARE_STOP_ARM}; no motor enable or motion targets"
  elif [[ "${PIPER_WRITE}" == "1" ]]; then
    echo "Execution: supervised dual-Piper hardware dispatch (ENABLE_WRITE=1)"
  else
    echo "Execution: shadow/no-send (ENABLE_WRITE=0)"
  fi
  if [[ -n "${PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M}" ]]; then
    echo "Left gripper initialization: ${PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M} m at profile force"
  fi
  if [[ -n "${PIPER_DURATION}" ]]; then
    echo "Duration: ${PIPER_DURATION}s"
  else
    echo "Duration: operator-controlled"
  fi

  if [[ "${PIPER_DRY_RUN}" == "1" ]]; then
    if [[ "${PIPER_MODEL_SERVER_MODE}" == "managed" ]]; then
      printf 'CMD:'
      printf ' %q' env \
        "MODEL_BUNDLE_ROOT=${PIPER_MODEL_CHECKPOINT}" \
        "MODEL_HOST=${PIPER_MODEL_HOST}" \
        "MODEL_PORT=${PIPER_MODEL_PORT}" \
        "MODEL_DEVICE=${PIPER_MODEL_DEVICE}" \
        "PI05_BUNDLE_ROOT=${PIPER_MODEL_CHECKPOINT}" \
        "PI05_HOST=${PIPER_MODEL_HOST}" \
        "PI05_PORT=${PIPER_MODEL_PORT}" \
        "PI05_DEVICE=${PIPER_MODEL_DEVICE}" \
        bash "${PIPER_MODEL_SERVER_SCRIPT}"
      printf '\n'
      echo "WAIT: ws://${PIPER_MODEL_HOST}:${PIPER_MODEL_PORT} (timeout ${PIPER_MODEL_READY_TIMEOUT}s)"
    fi
    if [[ -n "${PIPER_DURATION}" ]]; then
      echo "CMD: ${PIPER_DORA_BIN:-dora} run --stop-after ${PIPER_DURATION}s ${PIPER_GRAPH}"
    else
      echo "CMD: ${PIPER_DORA_BIN:-dora} run ${PIPER_GRAPH}"
    fi
    exit 0
  fi
  if [[ "${PIPER_DAGGER_MODE}" != "off" && ! -x "${PIPER_RUNTIME}/target/release/fourier-record" ]]; then
    die "DAgger recorder is not built; install Rust cargo and run ${PIPER_RUNTIME}/scripts/setup_local.sh before a collection run"
  fi
  if [[ "${PIPER_CONTROL_SOURCE}" != "keyboard" ]]; then
    if [[ -z "${PIPER_QNEXO_PORT}" ]]; then
      if [[ -e /dev/qnbot ]]; then
        PIPER_QNEXO_PORT=/dev/qnbot
      else
        mapfile -t PIPER_QNEXO_CANDIDATES < <(
          find /dev/serial/by-id -maxdepth 1 -type l \
            -name 'usb-Qnbot_Inc._Qnbot_Exoskeleton_Controller_*-if00' \
            -print 2>/dev/null | sort
        )
        if ((${#PIPER_QNEXO_CANDIDATES[@]} == 1)); then
          PIPER_QNEXO_PORT="${PIPER_QNEXO_CANDIDATES[0]}"
        elif ((${#PIPER_QNEXO_CANDIDATES[@]} > 1)); then
          die "multiple QNexo serial devices found; select one with --qnexo-port PATH"
        fi
      fi
    fi
    [[ -n "${PIPER_QNEXO_PORT}" && -e "${PIPER_QNEXO_PORT}" ]] || \
      die "QNexo serial device not found (checked --qnexo-port, /dev/qnbot, and stable /dev/serial/by-id links; use --control-source keyboard to run without QNexo)"
    echo "QNexo device resolved: ${PIPER_QNEXO_PORT}"
  fi
  [[ -n "${PIPER_DORA_BIN}" && -x "${PIPER_DORA_BIN}" ]] || die "dora executable not found"
  cleanup_stale_piper_operator_ui
  cleanup_stale_piper_camera_node

  PIPER_MODEL_SERVER_PID=""
  PIPER_DORA_PID=""
  if [[ "${PIPER_MODEL_SERVER_MODE}" == "managed" ]]; then
    cleanup_stale_piper_model_server "${PIPER_MODEL_HOST}" "${PIPER_MODEL_PORT}"
    if piper_tcp_port_in_use "${PIPER_MODEL_HOST}" "${PIPER_MODEL_PORT}"; then
      die "model-server port ${PIPER_MODEL_HOST}:${PIPER_MODEL_PORT} is already in use; stop the old server or use --model-server external --policy-url ${PIPER_POLICY_URL}"
    fi
    trap cleanup_managed_piper_processes EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    echo "Starting managed Piper model server..."
    env \
      "MODEL_BUNDLE_ROOT=${PIPER_MODEL_CHECKPOINT}" \
      "MODEL_HOST=${PIPER_MODEL_HOST}" \
      "MODEL_PORT=${PIPER_MODEL_PORT}" \
      "MODEL_DEVICE=${PIPER_MODEL_DEVICE}" \
      "PI05_BUNDLE_ROOT=${PIPER_MODEL_CHECKPOINT}" \
      "PI05_HOST=${PIPER_MODEL_HOST}" \
      "PI05_PORT=${PIPER_MODEL_PORT}" \
      "PI05_DEVICE=${PIPER_MODEL_DEVICE}" \
      bash "${PIPER_MODEL_SERVER_SCRIPT}" &
    PIPER_MODEL_SERVER_PID=$!
    wait_for_piper_model_server \
      "${PIPER_MODEL_SERVER_PID}" \
      "${PIPER_MODEL_HOST}" \
      "${PIPER_MODEL_PORT}" \
      "${PIPER_MODEL_READY_TIMEOUT}" || exit 1
    echo "Managed Piper model server is ready: ${PIPER_POLICY_URL}"
  fi

  cd "${PIPER_RUNTIME}"
  export XPOLICYLAB_ROOT="${REPO_ROOT}"
  export PIPER_PROFILE
  export PIPER_DEVICE_INSTANCE
  export PIPER_CAMERA_PROFILE
  export PIPER_CONTROL_SOURCE
  export PIPER_DAGGER_EXPERT_SOURCE
  export QNEXO_PORT="${PIPER_QNEXO_PORT}"
  if [[ "${PIPER_DAGGER_MODE}" == "data" ]]; then
    export RECAP_COLLECTION_MODE=DATA_COLLECTION
  elif [[ "${PIPER_DAGGER_MODE}" == "mixed" ]]; then
    export RECAP_COLLECTION_MODE=MIXED
  else
    export RECAP_COLLECTION_MODE=OFF
  fi
  export ENABLE_DAGGER_RECORDING="$([[ "${PIPER_DAGGER_MODE}" == "off" ]] && echo 0 || echo 1)"
  export DAGGER_RECORD_DIR="${PIPER_DAGGER_RECORD_DIR}"
  export COLLECTION_BATCH_ID="${PIPER_COLLECTION_BATCH_ID}"
  export DAGGER_ITERATION="${PIPER_DAGGER_ITERATION}"
  export XPOLICYLAB_SOURCE_COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
  export PIPER_OPERATOR_UI_PORT
  export PIPER_OPERATOR_UI_AUTO_OPEN
  export PIPER_DEPLOYMENT
  export PIPER_TASK_CATALOG_PATH
  export PIPER_POLICY_ACTION_HORIZON="${PIPER_ACTION_HORIZON}"
  export PIPER_POLICY_LABEL
  export PIPER_MODEL_FILTER_ENABLED
  export PIPER_MODEL_FILTER_ALPHA
  export ENABLE_WRITE="${PIPER_WRITE}"
  export PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M
  export PIPER_RELEASE_SOFTWARE_STOP_ARM
  export POLICY_URL="${PIPER_POLICY_URL}"
  export TASK="${PIPER_TASK}"
  export ROBOT_ID="${PIPER_ROBOT_ID}"
  export MODEL_BACKEND="${PIPER_MODEL_BACKEND}"
  export MODEL_CHECKPOINT="${PIPER_MODEL_CHECKPOINT}"
  export POLICY_ACTION_FREQUENCY_HZ="${PIPER_ACTION_FREQUENCY_HZ}"
  export POLICY_EXECUTE_CHUNK_SIZE="${PIPER_EXECUTE_CHUNK_SIZE}"
  export POLICY_EXECUTE_CHUNK_MODE="${PIPER_EXECUTE_CHUNK_MODE}"
  if [[ "${PIPER_MODEL_SERVER_MODE}" != "managed" ]]; then
    if [[ -n "${PIPER_DURATION}" ]]; then
      exec "${PIPER_DORA_BIN}" run --stop-after "${PIPER_DURATION}s" "${PIPER_GRAPH}"
    fi
    exec "${PIPER_DORA_BIN}" run "${PIPER_GRAPH}"
  fi

  PIPER_DORA_ARGS=(run)
  if [[ -n "${PIPER_DURATION}" ]]; then
    PIPER_DORA_ARGS+=(--stop-after "${PIPER_DURATION}s")
  fi
  PIPER_DORA_ARGS+=("${PIPER_GRAPH}")
  "${PIPER_DORA_BIN}" "${PIPER_DORA_ARGS[@]}" &
  PIPER_DORA_PID=$!
  PIPER_COMPLETED_PID=""
  set +e
  wait -n -p PIPER_COMPLETED_PID \
    "${PIPER_MODEL_SERVER_PID}" "${PIPER_DORA_PID}"
  PIPER_CHILD_STATUS=$?
  set -e
  if [[ "${PIPER_COMPLETED_PID}" == "${PIPER_MODEL_SERVER_PID}" ]]; then
    echo "Managed Piper model server exited; stopping the robot graph" >&2
    stop_piper_child "${PIPER_DORA_PID}"
    PIPER_DORA_PID=""
    ((PIPER_CHILD_STATUS != 0)) || PIPER_CHILD_STATUS=1
  else
    PIPER_DORA_PID=""
  fi
  exit "${PIPER_CHILD_STATUS}"
fi

MODEL="$1"
shift
case "${MODEL}" in
  jiaxin) MODEL="gr00t" ;;
  pi0.5) MODEL="pi05" ;;
  turbo) MODEL="turbovla" ;;
esac
case "${MODEL}" in gr00t|fastwam|pi05|turbovla) ;; *) die "unknown chain: ${MODEL}" ;; esac

CKPT=""
# RealEval injects the immutable run instruction through TASK. Preserve that
# value unless an explicit --task option overrides it below.
TASK="${TASK:-}"
ROBOT_ID=""
CHUNK=""
HOST=""
PORT=""
RECORD=""
RECORD_DIR=""
MAX_JOINT_STEP=""
FREEZE_WAIST=""
FREEZE_WAIST_JOINTS=""
RTC=""
MINK=""
QP=""
SPLINE=""
FILTER=""
WRITE=0
DRY_RUN=0

while (($#)); do
  case "$1" in
    --ckpt) need_value "$@"; CKPT="$2"; shift 2 ;;
    --task) need_value "$@"; TASK="$2"; shift 2 ;;
    --robot-id) need_value "$@"; ROBOT_ID="$2"; shift 2 ;;
    --chunk) need_value "$@"; CHUNK="$2"; shift 2 ;;
    --host) need_value "$@"; HOST="$2"; shift 2 ;;
    --port) need_value "$@"; PORT="$2"; shift 2 ;;
    --record) RECORD=1; shift ;;
    --no-record) RECORD=0; shift ;;
    --record-dir) need_value "$@"; RECORD_DIR="$2"; shift 2 ;;
    --max-joint-step) need_value "$@"; MAX_JOINT_STEP="$2"; shift 2 ;;
    --freeze-waist) FREEZE_WAIST=1; shift ;;
    --no-freeze-waist) FREEZE_WAIST=0; shift ;;
    --freeze-waist-joints) need_value "$@"; FREEZE_WAIST_JOINTS="$2"; shift 2 ;;
    --rtc) RTC=1; shift ;;
    --no-rtc) RTC=0; shift ;;
    --mink) need_value "$@"; MINK="$2"; shift 2 ;;
    --action-qp) QP=1; shift ;;
    --no-action-qp) QP=0; shift ;;
    --spline) SPLINE=1; shift ;;
    --no-spline) SPLINE=0; shift ;;
    --filter) FILTER=1; shift ;;
    --no-filter) FILTER=0; shift ;;
    --enable-write) WRITE=1; shift ;;
    --no-write) WRITE=0; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

if [[ -n "${CHUNK}" && ! "${CHUNK}" =~ ^[1-9][0-9]*$ ]]; then
  die "--chunk must be positive"
fi
if [[ -n "${MINK}" ]]; then
  case "${MINK}" in off|shadow|enforce) ;; *) die "invalid --mink mode" ;; esac
fi
if [[ "${MODEL}" != "fastwam" && -n "${RTC}" ]]; then
  die "--rtc requires the fastwam chain"
fi
if [[ "${SPLINE}" == "1" && -n "${MINK}" && "${MINK}" != "off" ]]; then
  die "spline execution and Mink enforcement are mutually exclusive"
fi

ROBOT_ENV=()
append_env MODEL_CHECKPOINT "${CKPT}"
append_env TASK "${TASK}"
append_env ROBOT_ID "${ROBOT_ID}"
append_env ACTION_CHUNK_SIZE "${CHUNK}"
append_env GR3_POLICY_HOST "${HOST}"
append_env GR3_POLICY_PORT "${PORT}"
append_env ENABLE_DAGGER_RECORDING "${RECORD}"
append_env DAGGER_RECORD_DIR "${RECORD_DIR}"
append_env MAX_JOINT_STEP_RAD "${MAX_JOINT_STEP}"
append_env FREEZE_WAIST "${FREEZE_WAIST}"
append_env FREEZE_WAIST_JOINTS "${FREEZE_WAIST_JOINTS}"
append_env FASTWAM_RTC_ENABLED "${RTC}"
append_env OUTPUT_MINK_MODE "${MINK}"
append_env OUTPUT_QP_ENABLED "${QP}"
append_env OUTPUT_SPLINE_ENABLED "${SPLINE}"
append_env OUTPUT_FILTER_ENABLED "${FILTER}"
append_env GR3_ENABLE_WRITE "${WRITE}"

ARGS=()
[[ "${DRY_RUN}" != "1" ]] || ARGS+=(--dry-run)
exec env "${ROBOT_ENV[@]}" "${SCRIPT_DIR}/gr3.sh" dagger "${MODEL}" "${ARGS[@]}"
