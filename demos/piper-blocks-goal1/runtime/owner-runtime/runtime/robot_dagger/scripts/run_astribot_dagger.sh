#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"
GRAPH="${PROJECT_ROOT}/graphs/astribot-policy-dagger.yml"
ASTRIBOT_BUNDLED_SDK_ROOT="${REPO_ROOT}/third_party/robot_sdks/astribot_s1/runtime/astribot_sdk_ros2-master"
DAGGER_ENV="${PROJECT_ROOT}/.venv"
DORA_BIN="${DAGGER_ENV}/bin/dora"
POLICY_PYTHON="${DAGGER_ENV}/bin/python"
OWNER_PYTHON="${REPO_ROOT}/.runtime/astribot-owner/bin/python"
ROS_ROOT="${ASTRIBOT_ROS_ROOT:-/opt/ros/humble}"
CHECKPOINT="${MODEL_CHECKPOINT:-/home/ubuntu/astribot_pick_tomato_v3_pi05_ema_step20000_ckpt}"
POLICY_HOST="${ASTRIBOT_POLICY_HOST:-127.0.0.1}"
POLICY_PORT="${ASTRIBOT_POLICY_PORT:-19001}"
DURATION="${DAGGER_DURATION_SECONDS:-}"
DRY_RUN=0
WRITE=0
RUN_MODE=dagger
RECORD_OVERRIDE=""

usage() {
  cat <<'EOF'
Usage: run_astribot_dagger.sh [--mode policy|dagger] [--record|--no-record]
                               [--enable-write|--no-write] [--dry-run]
                               [--duration SECONDS]

Runs the one formal Astribot policy graph. Policy mode disables trainable
recording by default; DAgger mode enables it by default. Both modes default to
shadow/no-send, and the owner driver is enabled only through --enable-write.
EOF
}

while (($#)); do
  case "$1" in
    --dry-run) DRY_RUN=1; shift ;;
    --enable-write) WRITE=1; shift ;;
    --no-write) WRITE=0; shift ;;
    --mode)
      (($# >= 2)) || { echo "--mode requires a value" >&2; exit 2; }
      RUN_MODE="$2"; shift 2 ;;
    --record) RECORD_OVERRIDE=1; shift ;;
    --no-record) RECORD_OVERRIDE=0; shift ;;
    --duration)
      (($# >= 2)) || { echo "--duration requires a value" >&2; exit 2; }
      DURATION="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "${RUN_MODE}" != "policy" && "${RUN_MODE}" != "dagger" ]]; then
  echo "--mode must be policy or dagger" >&2
  exit 2
fi
if [[ -n "${DURATION}" && ! "${DURATION}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--duration must be a positive integer number of seconds" >&2
  exit 2
fi
export MODEL_BACKEND="pi05"
export MODEL_CHECKPOINT="${CHECKPOINT}"
export XPOLICYLAB_ROOT="${REPO_ROOT}"
export ASTRIBOT_SDK_ROOT="${ASTRIBOT_SDK_ROOT:-${ASTRIBOT_BUNDLED_SDK_ROOT}}"
export ASTRIBOT_COMMON_ROOT="${ASTRIBOT_COMMON_ROOT:-${ASTRIBOT_SDK_ROOT}}"
export ASTRIBOT_ROS_ROOT="${ROS_ROOT}"
export POLICY_URL="${POLICY_URL:-ws://${POLICY_HOST}:${POLICY_PORT}}"
export TASK="${TASK:-pick the tomato}"
export ROBOT_ID="${ROBOT_ID:-astribot-s1}"
export ACTIVATE_CAMERAS="${ACTIVATE_CAMERAS:-1}"
export ENABLE_WRITE="${WRITE}"
export ASTRIBOT_HIGH_CONTROL_RIGHTS="${WRITE}"
export ASTRIBOT_PROFILE="${ASTRIBOT_PROFILE:-${REPO_ROOT}/real_world/robots/astribot_s1/profile.yml}"
if [[ -n "${RECORD_OVERRIDE}" ]]; then
  export ENABLE_DAGGER_RECORDING="${RECORD_OVERRIDE}"
elif [[ -n "${ENABLE_DAGGER_RECORDING:-}" ]]; then
  export ENABLE_DAGGER_RECORDING
elif [[ "${RUN_MODE}" == "dagger" ]]; then
  export ENABLE_DAGGER_RECORDING=1
else
  export ENABLE_DAGGER_RECORDING=0
fi
export DAGGER_RECORD_DIR="${DAGGER_RECORD_DIR:-${REPO_ROOT}/Log/dagger/astribot_s1}"
export ROLLOUT_REPORT_DIR="${ROLLOUT_REPORT_DIR:-${REPO_ROOT}/Log/astribot_s1_rollout_reports}"
export FOOTSWITCH_DEVICE="${FOOTSWITCH_DEVICE:-/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd}"
export FOOTSWITCH_TAKEOVER_KEY_CODE="${FOOTSWITCH_TAKEOVER_KEY_CODE:-45}"
export EVIDENCE_RETENTION_POLICY="${EVIDENCE_RETENTION_POLICY:-AUTO_RETAIN_ALL_EVAL}"
export XPOLICY_EVAL_RUN_ID="${XPOLICY_EVAL_RUN_ID:-}"
export XPOLICY_EVAL_SUITE_ID="${XPOLICY_EVAL_SUITE_ID:-adhoc}"
export XPOLICY_EVAL_SUITE_REVISION="${XPOLICY_EVAL_SUITE_REVISION:-v1}"
export XPOLICY_EVAL_TASK_REVISION="${XPOLICY_EVAL_TASK_REVISION:-v1}"
export XPOLICY_EVAL_CASE_IDS="${XPOLICY_EVAL_CASE_IDS:-}"
if [[ -n "${RECAP_COLLECTION_MODE:-}" ]]; then
  export RECAP_COLLECTION_MODE
elif [[ "${RUN_MODE}" == "dagger" ]]; then
  export RECAP_COLLECTION_MODE=MIXED
else
  export RECAP_COLLECTION_MODE=AUTONOMOUS
fi
export RECAP_POLICY_CONDITION="${RECAP_POLICY_CONDITION:-UNCONDITIONED}"
export XPOLICYLAB_SOURCE_REPO="${XPOLICYLAB_SOURCE_REPO:-CoCoMatrix/xpolicylabdagger}"
if [[ -z "${XPOLICYLAB_SOURCE_COMMIT:-}" ]]; then
  XPOLICYLAB_SOURCE_COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || true)"
  export XPOLICYLAB_SOURCE_COMMIT="${XPOLICYLAB_SOURCE_COMMIT:-unknown}"
fi

if [[ "${ENABLE_DAGGER_RECORDING}" != "0" && "${ENABLE_DAGGER_RECORDING}" != "1" ]]; then
  echo "ENABLE_DAGGER_RECORDING must be 0 or 1" >&2
  exit 2
fi
if [[ ! -f "${ASTRIBOT_PROFILE}" ]]; then
  echo "Astribot robot profile not found: ${ASTRIBOT_PROFILE}" >&2
  exit 1
fi
if [[ "${EVIDENCE_RETENTION_POLICY}" != "AUTO_RETAIN_ALL_EVAL" ]]; then
  echo "EVIDENCE_RETENTION_POLICY must be AUTO_RETAIN_ALL_EVAL; pedal-gated saving was removed." >&2
  exit 2
fi
if [[ -n "${XPOLICY_EVAL_RUN_ID}" && "${ENABLE_DAGGER_RECORDING}" != "1" ]]; then
  echo "An evaluation run requires ENABLE_DAGGER_RECORDING=1." >&2
  exit 2
fi

echo "Graph: ${GRAPH}"
echo "Runtime mode: ${RUN_MODE}"
echo "Robot profile: ${ASTRIBOT_PROFILE}"
echo "Robot SDK: ${ASTRIBOT_SDK_ROOT}"
echo "ROS 2: ${ASTRIBOT_ROS_ROOT}"
echo "Owner Python: ${OWNER_PYTHON}"
echo "Dora runtime: ${DAGGER_ENV}"
echo "Policy: pi05 (${POLICY_URL})"
echo "Checkpoint: ${MODEL_CHECKPOINT}"
echo "Task: ${TASK}"
echo "Cameras: Astribot SDK head + left wrist + right wrist"
echo "QNexo: shared exo_driver_node + robot-independent controls"
if [[ "${ENABLE_DAGGER_RECORDING}" == "1" ]]; then
  echo "Recording: enabled, GR3-compatible Parquet -> ${DAGGER_RECORD_DIR}"
else
  echo "Recording: disabled"
fi
echo "Evaluation: ${XPOLICY_EVAL_RUN_ID:-none}; retention=${EVIDENCE_RETENTION_POLICY}"
if [[ "${ENABLE_WRITE}" == "1" ]]; then
  echo "Execution: Astribot full25 hardware dispatch (ENABLE_WRITE=1)"
else
  echo "Execution: shadow/no-send (ENABLE_WRITE=0)"
fi
if [[ -n "${DURATION}" ]]; then
  echo "Duration: ${DURATION}s"
else
  echo "Duration: operator-controlled"
fi

if [[ "${DRY_RUN}" == "1" ]]; then
  if [[ -n "${DURATION}" ]]; then
    echo "CMD: ${DORA_BIN} run --stop-after ${DURATION}s ${GRAPH}"
  else
    echo "CMD: ${DORA_BIN} run ${GRAPH}"
  fi
  exit 0
fi
if [[ ! -d "${CHECKPOINT}" ]]; then
  echo "Astribot checkpoint not found: ${CHECKPOINT}" >&2
  exit 1
fi
if [[ ! -f "${ASTRIBOT_SDK_ROOT}/env.sh" ]]; then
  echo "Astribot SDK is not prepared: ${ASTRIBOT_SDK_ROOT}" >&2
  echo "Run: scripts/setup_robot_sdks.sh prepare astribot_s1" >&2
  exit 1
fi
if [[ ! -f "${ASTRIBOT_ROS_ROOT}/setup.bash" ]]; then
  echo "ROS 2 Humble is not prepared: ${ASTRIBOT_ROS_ROOT}" >&2
  echo "Run: scripts/setup_astribot_host.sh doctor" >&2
  exit 1
fi
if [[ ! -x "${OWNER_PYTHON}" ]]; then
  echo "Astribot owner environment is missing: ${OWNER_PYTHON}" >&2
  echo "Run: scripts/setup_astribot_host.sh prepare" >&2
  exit 1
fi
if [[ ! -x "${DORA_BIN}" || ! -x "${POLICY_PYTHON}" ]]; then
  echo "Astribot Dora environment is missing: ${DAGGER_ENV}" >&2
  echo "Run: runtime/robot_dagger/scripts/setup_local.sh" >&2
  exit 1
fi
if [[ "${ENABLE_DAGGER_RECORDING}" == "1" ]]; then
  mkdir -p "${DAGGER_RECORD_DIR}"
  if [[ ! -d "${DAGGER_RECORD_DIR}" || ! -w "${DAGGER_RECORD_DIR}" ]]; then
    echo "Astribot DAgger record directory is not writable: ${DAGGER_RECORD_DIR}" >&2
    exit 1
  fi
fi
# Invoking the live-only launcher is the explicit SDK-connect operation. Keep
# the internal owner guard out of the public CLI contract.
export ASTRIBOT_SDK_CONNECT_ACKNOWLEDGED=1

if [[ ! -e /dev/qnbot ]]; then
  echo "QNexo serial device not found: /dev/qnbot" >&2
  exit 1
fi
if [[ ! -e "${FOOTSWITCH_DEVICE}" ]]; then
  echo "Footswitch not found: ${FOOTSWITCH_DEVICE}" >&2
  exit 1
fi
if ! "${OWNER_PYTHON}" - <<'PY'
import dora
import numpy
import pyarrow
import serial
import websockets
import yaml
import XPolicyLab
PY
then
  echo "Astribot owner environment is missing required Python dependencies." >&2
  exit 1
fi
if ! "${POLICY_PYTHON}" - <<'PY'
import dora
import numpy
import pyarrow
import websockets
import yaml
from client_server.ws.model_client import WsModelClient
PY
then
  echo "Astribot Dora environment is missing the ActionChunk policy-node dependencies." >&2
  exit 1
fi
if ! "${OWNER_PYTHON}" - "${POLICY_HOST}" "${POLICY_PORT}" <<'PY'
import socket
import sys
with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=1):
    pass
PY
then
  echo "Astribot policy server is not reachable at ${POLICY_HOST}:${POLICY_PORT}." >&2
  exit 1
fi

cd "${PROJECT_ROOT}"
if [[ -n "${DURATION}" ]]; then
  exec "${DORA_BIN}" run --stop-after "${DURATION}s" "${GRAPH}"
fi
exec "${DORA_BIN}" run "${GRAPH}"
