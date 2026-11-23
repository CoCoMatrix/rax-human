#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"
GRAPH="${PROJECT_ROOT}/graphs/gr3-policy-dagger.yml"
MODEL="${1:-}"
DRY_RUN=0
shift || true

if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=1
  shift
fi
if (($#)); then
  echo "Usage: $0 {gr00t|fastwam|pi05|turbovla} [--dry-run]" >&2
  exit 2
fi

export GR3_POLICY_HOST="${GR3_POLICY_HOST:-127.0.0.1}"
export GR3_POLICY_PORT="${GR3_POLICY_PORT:-19000}"
export POLICY_URL="${POLICY_URL:-ws://${GR3_POLICY_HOST}:${GR3_POLICY_PORT}}"
export GR3_PROFILE="${GR3_PROFILE:-${PROJECT_ROOT}/../../real_world/robots/gr3/profile.yml}"
export GR3_ENABLE_WRITE="${GR3_ENABLE_WRITE:-0}"
if [[ "${GR3_ENABLE_WRITE}" != "0" && "${GR3_ENABLE_WRITE}" != "1" ]]; then
  echo "GR3_ENABLE_WRITE must be 0 or 1." >&2
  exit 2
fi
case "${MODEL}" in
  gr00t|jiaxin)
    MODEL="gr00t"
    export MODEL_BACKEND="jiaxin"
    export MODEL_CHECKPOINT="${MODEL_CHECKPOINT:-/home/fourier/jiaxin/728-v5}"
    export ACTION_CHUNK_SIZE="${ACTION_CHUNK_SIZE:-16}"
    export INFERENCE_TRIGGER_STEPS="${INFERENCE_TRIGGER_STEPS:-0}"
    export TASK="${TASK:-Use the left hand to pick the tomato and place it into the left basket}"
    ;;
  fastwam)
    export MODEL_BACKEND="fastwam"
    export MODEL_CHECKPOINT="${MODEL_CHECKPOINT:-/home/fourier/xingyu/checkpoints/checkpoints_wan_710/step_020650.pt}"
    export ACTION_CHUNK_SIZE="${ACTION_CHUNK_SIZE:-32}"
    export FASTWAM_RTC_ENABLED="${FASTWAM_RTC_ENABLED:-0}"
    export FASTWAM_RTC_PREFIX_STEPS="${FASTWAM_RTC_PREFIX_STEPS:-16}"
    if [[ "${FASTWAM_RTC_ENABLED}" == "1" ]]; then
      export INFERENCE_TRIGGER_STEPS="${INFERENCE_TRIGGER_STEPS:-${FASTWAM_RTC_PREFIX_STEPS}}"
    else
      export INFERENCE_TRIGGER_STEPS="${INFERENCE_TRIGGER_STEPS:-0}"
    fi
    ;;
  pi05)
    export MODEL_BACKEND="pi05"
    export MODEL_CHECKPOINT="${MODEL_CHECKPOINT:-/home/fourier/siyuan/pi05-experiments-main/checkpoints/pi05_anygrasp/anygrasp_20hr/28000}"
    export ACTION_CHUNK_SIZE="${ACTION_CHUNK_SIZE:-25}"
    export INFERENCE_TRIGGER_STEPS="${INFERENCE_TRIGGER_STEPS:-0}"
    ;;
  turbovla|turbo)
    MODEL="turbovla"
    export MODEL_BACKEND="turbovla"
    export POLICY_STATE_DIM="${POLICY_STATE_DIM:-31}"
    export MODEL_CHECKPOINT="${MODEL_CHECKPOINT:-${PROJECT_ROOT}/../../policy/TurboVLA/checkpoints/GR3-deploy-v4-round-a-joint31-e1-lr1e4-s13200-gr3-joint31_canonical37-0/model_final.pt}"
    export ACTION_CHUNK_SIZE="${ACTION_CHUNK_SIZE:-16}"
    export INFERENCE_TRIGGER_STEPS="${INFERENCE_TRIGGER_STEPS:-4}"
    export TASK="${TASK:-Pick the red tomato from the table and place it into the left basket.}"
    ;;
  *)
    echo "Usage: $0 {gr00t|fastwam|pi05|turbovla} [--dry-run]" >&2
    exit 2
    ;;
esac

export POLICY_STATE_DIM="${POLICY_STATE_DIM:-33}"

export ROBOT_ID="${ROBOT_ID:-115}"
export FREEZE_WAIST="${FREEZE_WAIST:-1}"
export FREEZE_WAIST_JOINTS="${FREEZE_WAIST_JOINTS:-waist_roll_joint}"
export OUTPUT_QP_ENABLED="${OUTPUT_QP_ENABLED:-0}"
export OUTPUT_QP_ACCEL_WEIGHT="${OUTPUT_QP_ACCEL_WEIGHT:-0.25}"
export OUTPUT_QP_OVERLAP_WEIGHT="${OUTPUT_QP_OVERLAP_WEIGHT:-4.0}"
export OUTPUT_SPLINE_ENABLED="${OUTPUT_SPLINE_ENABLED:-0}"
export OUTPUT_SPLINE_FACTOR="${OUTPUT_SPLINE_FACTOR:-3}"
export OUTPUT_MINK_MODE="${OUTPUT_MINK_MODE:-off}"
export OUTPUT_ACTION_HZ="${OUTPUT_ACTION_HZ:-30}"
export PREDICTIVE_MINK_ENCODER_SYNC_COST="${PREDICTIVE_MINK_ENCODER_SYNC_COST:-1.0}"
export PREDICTIVE_MINK_HORIZON_STEPS="${PREDICTIVE_MINK_HORIZON_STEPS:-7}"
export OUTPUT_FILTER_ENABLED="${OUTPUT_FILTER_ENABLED:-0}"
export OUTPUT_FILTER_ALPHA="${OUTPUT_FILTER_ALPHA:-0.6}"
export INFERENCE_AUTO_PREFETCH="${INFERENCE_AUTO_PREFETCH:-1}"
export INFERENCE_PREFETCH_QUANTILE="${INFERENCE_PREFETCH_QUANTILE:-0.95}"
export INFERENCE_PREFETCH_MARGIN_STEPS="${INFERENCE_PREFETCH_MARGIN_STEPS:-1}"
export INFERENCE_PREFETCH_WINDOW_SIZE="${INFERENCE_PREFETCH_WINDOW_SIZE:-32}"
if [[ -z "${ACTION_RTC_MODE:-}" ]]; then
  if [[ "${INFERENCE_AUTO_PREFETCH}" == "1" ]]; then
    ACTION_RTC_MODE=align
  else
    ACTION_RTC_MODE=disabled
  fi
fi
export ACTION_RTC_MODE
export ACTION_RTC_LATENCY_QUANTILE="${ACTION_RTC_LATENCY_QUANTILE:-${INFERENCE_PREFETCH_QUANTILE}}"
export ACTION_RTC_LATENCY_MARGIN_STEPS="${ACTION_RTC_LATENCY_MARGIN_STEPS:-${INFERENCE_PREFETCH_MARGIN_STEPS}}"
export ACTION_RTC_LATENCY_WINDOW_SIZE="${ACTION_RTC_LATENCY_WINDOW_SIZE:-${INFERENCE_PREFETCH_WINDOW_SIZE}}"
export ACTION_RTC_PRESERVE_STEPS="${ACTION_RTC_PRESERVE_STEPS:-0}"
export ACTION_RTC_TRANSITION_STEPS="${ACTION_RTC_TRANSITION_STEPS:-0}"
export ACTION_RTC_TRANSITION_CURVE="${ACTION_RTC_TRANSITION_CURVE:-smoothstep}"
export ACTION_RTC_NEW_WEIGHT_START="${ACTION_RTC_NEW_WEIGHT_START:-0.0}"
export ACTION_RTC_NEW_WEIGHT_END="${ACTION_RTC_NEW_WEIGHT_END:-1.0}"
export ENABLE_DAGGER_RECORDING="${ENABLE_DAGGER_RECORDING:-0}"
export DAGGER_RECORD_DIR="${DAGGER_RECORD_DIR:-${REPO_ROOT}/Log/dagger/gr3}"
export FOOTSWITCH_DEVICE="${FOOTSWITCH_DEVICE:-/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd}"
export FOOTSWITCH_TAKEOVER_KEY_CODE="${FOOTSWITCH_TAKEOVER_KEY_CODE:-45}"
export EVIDENCE_RETENTION_POLICY="${EVIDENCE_RETENTION_POLICY:-AUTO_RETAIN_ALL_EVAL}"
export XPOLICY_EVAL_RUN_ID="${XPOLICY_EVAL_RUN_ID:-}"
export XPOLICY_EVAL_SUITE_ID="${XPOLICY_EVAL_SUITE_ID:-adhoc}"
export XPOLICY_EVAL_SUITE_REVISION="${XPOLICY_EVAL_SUITE_REVISION:-v1}"
export XPOLICY_EVAL_TASK_REVISION="${XPOLICY_EVAL_TASK_REVISION:-v1}"
export XPOLICY_EVAL_CASE_IDS="${XPOLICY_EVAL_CASE_IDS:-}"
export TASK="${TASK:-Pick the tomato from the table and place it inside the basket on the left.}"
export RECAP_COLLECTION_MODE="${RECAP_COLLECTION_MODE:-MIXED}"
export RECAP_POLICY_CONDITION="${RECAP_POLICY_CONDITION:-UNCONDITIONED}"
export XPOLICYLAB_SOURCE_REPO="${XPOLICYLAB_SOURCE_REPO:-CoCoMatrix/xpolicylabdagger}"
if [[ -z "${XPOLICYLAB_SOURCE_COMMIT:-}" ]]; then
  XPOLICYLAB_SOURCE_COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || true)"
  export XPOLICYLAB_SOURCE_COMMIT="${XPOLICYLAB_SOURCE_COMMIT:-unknown}"
fi
if [[ -z "${MODEL_CHECKPOINT_SHA256:-}" && -f "${MODEL_CHECKPOINT}" ]]; then
  MODEL_CHECKPOINT_SHA256="$(sha256sum "${MODEL_CHECKPOINT}" | awk '{print $1}')"
  export MODEL_CHECKPOINT_SHA256
fi

if [[ ! "${ACTION_CHUNK_SIZE}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ACTION_CHUNK_SIZE must be a positive integer." >&2
  exit 2
fi
for name in ENABLE_DAGGER_RECORDING FREEZE_WAIST OUTPUT_FILTER_ENABLED OUTPUT_QP_ENABLED OUTPUT_SPLINE_ENABLED; do
  if [[ "${!name}" != "0" && "${!name}" != "1" ]]; then
    echo "${name} must be 0 or 1." >&2
    exit 2
  fi
done
if [[ "${EVIDENCE_RETENTION_POLICY}" != "AUTO_RETAIN_ALL_EVAL" ]]; then
  echo "EVIDENCE_RETENTION_POLICY must be AUTO_RETAIN_ALL_EVAL; pedal-gated saving was removed." >&2
  exit 2
fi
if [[ -n "${XPOLICY_EVAL_RUN_ID}" && "${ENABLE_DAGGER_RECORDING}" != "1" ]]; then
  echo "An evaluation run requires ENABLE_DAGGER_RECORDING=1." >&2
  exit 2
fi
if [[ "${OUTPUT_MINK_MODE}" != "off" && "${OUTPUT_MINK_MODE}" != "shadow" && "${OUTPUT_MINK_MODE}" != "enforce" ]]; then
  echo "OUTPUT_MINK_MODE must be off, shadow, or enforce." >&2
  exit 2
fi
if [[ "${OUTPUT_SPLINE_ENABLED}" == "1" && "${OUTPUT_MINK_MODE}" != "off" ]]; then
  echo "Spline execution requires OUTPUT_MINK_MODE=off." >&2
  exit 2
fi
if [[ ! "${OUTPUT_SPLINE_FACTOR}" =~ ^[1-9][0-9]*$ ]]; then
  echo "OUTPUT_SPLINE_FACTOR must be a positive integer." >&2
  exit 2
fi
if [[ "${MODEL}" == "fastwam" && "${FASTWAM_RTC_ENABLED}" != "0" && "${FASTWAM_RTC_ENABLED}" != "1" ]]; then
  echo "FASTWAM_RTC_ENABLED must be 0 or 1." >&2
  exit 2
fi
if [[ "${MODEL}" == "fastwam" && "${FASTWAM_RTC_ENABLED}" == "1" ]]; then
  if [[ ! "${FASTWAM_RTC_PREFIX_STEPS}" =~ ^[1-9][0-9]*$ || "${FASTWAM_RTC_PREFIX_STEPS}" -ge "${ACTION_CHUNK_SIZE}" ]]; then
    echo "FastWAM RTC requires a positive prefix smaller than ACTION_CHUNK_SIZE." >&2
    exit 2
  fi
  if [[ "${ACTION_CHUNK_SIZE}" != "32" || "${FASTWAM_RTC_PREFIX_STEPS}" != "16" ]]; then
    echo "FastWAM RTC requires ACTION_CHUNK_SIZE=32 and FASTWAM_RTC_PREFIX_STEPS=16." >&2
    exit 2
  fi
fi

echo "Policy: ${MODEL} (${POLICY_URL})"
echo "Checkpoint: ${MODEL_CHECKPOINT}"
echo "Policy state: ${POLICY_STATE_DIM}D"
echo "Action chunk: ${ACTION_CHUNK_SIZE}; inference trigger: ${INFERENCE_TRIGGER_STEPS}"
echo "Auto prefetch: ${INFERENCE_AUTO_PREFETCH}; quantile=${INFERENCE_PREFETCH_QUANTILE}; margin=${INFERENCE_PREFETCH_MARGIN_STEPS}"
echo "Client RTC: mode=${ACTION_RTC_MODE}; preserve=${ACTION_RTC_PRESERVE_STEPS}; transition=${ACTION_RTC_TRANSITION_STEPS}/${ACTION_RTC_TRANSITION_CURVE}; new_weight=${ACTION_RTC_NEW_WEIGHT_START}->${ACTION_RTC_NEW_WEIGHT_END}"
echo "Robot policy: freeze_waist=${FREEZE_WAIST} joints=${FREEZE_WAIST_JOINTS}"
echo "Output: qp=${OUTPUT_QP_ENABLED} mink=${OUTPUT_MINK_MODE} filter=${OUTPUT_FILTER_ENABLED} spline=${OUTPUT_SPLINE_ENABLED}"
echo "Recording: ${ENABLE_DAGGER_RECORDING} -> ${DAGGER_RECORD_DIR}"
echo "Evaluation: ${XPOLICY_EVAL_RUN_ID:-none}; retention=${EVIDENCE_RETENTION_POLICY}"
if [[ "${GR3_ENABLE_WRITE}" == "1" ]]; then
  echo "Execution: supervised GR3 hardware dispatch (GR3_ENABLE_WRITE=1)"
else
  echo "Execution: dry-run/preflight only (GR3_ENABLE_WRITE=0; Aurora SDK blocked)"
fi

if [[ "${DRY_RUN}" == "1" ]]; then
  echo "CMD: dora run ${GRAPH}"
  exit 0
fi
if [[ "${GR3_ENABLE_WRITE}" != "1" ]]; then
  echo "GR3 no-write is dry-run/preflight only; live graph startup requires --enable-write." >&2
  exit 2
fi
if [[ "${ENABLE_DAGGER_RECORDING}" == "1" ]]; then
  mkdir -p "${DAGGER_RECORD_DIR}"
  if [[ ! -d "${DAGGER_RECORD_DIR}" || ! -w "${DAGGER_RECORD_DIR}" ]]; then
    echo "GR3 DAgger record directory is not writable: ${DAGGER_RECORD_DIR}" >&2
    exit 1
  fi
fi
if [[ ! -x "${PROJECT_ROOT}/../../.venv/bin/xpolicylab-gr3-dora" ]]; then
  echo "XPolicyLab GR3 bridge is missing from the root .venv." >&2
  exit 1
fi
if [[ ! -x "${PROJECT_ROOT}/target/release/fourier-record" ]]; then
  echo "Recorder is missing. Run scripts/setup_local.sh first." >&2
  exit 1
fi
if [[ ! -e "${FOOTSWITCH_DEVICE}" ]]; then
  echo "Footswitch not found: ${FOOTSWITCH_DEVICE}" >&2
  exit 1
fi
if ! "${PROJECT_ROOT}/../../.venv/bin/python" - "${GR3_POLICY_HOST}" "${GR3_POLICY_PORT}" <<'PY'
import sys
from websockets.sync.client import connect
with connect(f"ws://{sys.argv[1]}:{sys.argv[2]}", open_timeout=1, close_timeout=1):
    pass
PY
then
  echo "XPolicyLab policy server is not reachable at ${GR3_POLICY_HOST}:${GR3_POLICY_PORT}." >&2
  exit 1
fi

cd "${PROJECT_ROOT}"
exec dora run "${GRAPH}"
