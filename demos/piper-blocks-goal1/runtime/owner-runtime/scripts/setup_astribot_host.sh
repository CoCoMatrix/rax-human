#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
SDK_ROOT="${ASTRIBOT_SDK_ROOT:-${REPO_ROOT}/third_party/robot_sdks/astribot_s1/runtime/astribot_sdk_ros2-master}"
PINNED_SDK_ROOT="${REPO_ROOT}/third_party/robot_sdks/astribot_s1/runtime/astribot_sdk_ros2-master"
ROS_ROOT="${ASTRIBOT_ROS_ROOT:-/opt/ros/humble}"
OWNER_ENV="${REPO_ROOT}/.runtime/astribot-owner"
OWNER_REQUIREMENTS="${REPO_ROOT}/real_world/robots/astribot_s1/requirements-owner.txt"
DAGGER_ENV="${REPO_ROOT}/runtime/robot_dagger/.venv"
RECORDER="${REPO_ROOT}/runtime/robot_dagger/target/release/fourier-record"

usage() {
  cat <<'EOF'
Usage: scripts/setup_astribot_host.sh <prepare|doctor>

prepare  Verify/materialize the pinned SDK and create the isolated Python 3.10
         Astribot owner environment. This never connects to the robot.
doctor   Run read-only host checks. Robot serial/input devices are deliberately
         excluded and remain supervised physical checks.
EOF
}

resolve_uv() {
  local candidate
  candidate="${UV_BIN:-$(command -v uv || true)}"
  if [[ -z "${candidate}" && -x /home/ubuntu/miniconda3/envs/infra/bin/uv ]]; then
    candidate=/home/ubuntu/miniconda3/envs/infra/bin/uv
  fi
  [[ -n "${candidate}" && -x "${candidate}" ]] || {
    echo "Astribot host setup requires uv." >&2
    return 1
  }
  printf '%s\n' "${candidate}"
}

sdk_import_probe() (
  export PYTHONPATH="${REPO_ROOT}:${SDK_ROOT}/third_party/third_pkg/pinocchio/lib/python3.10/site-packages:${SDK_ROOT}/third_party/third_pkg/hpp-fcl/lib/python3.10/site-packages:${SDK_ROOT}/astribot_msgs/local/lib/python3.10/dist-packages:${SDK_ROOT}/third_party/software/astribot_ros_middleware/lib/python3.10/site-packages:${SDK_ROOT}:${ROS_ROOT}/lib/python3.10/site-packages:${ROS_ROOT}/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="${SDK_ROOT}/third_party/third_pkg/pinocchio/lib:${SDK_ROOT}/third_party/third_pkg/hpp-fcl/lib:${SDK_ROOT}/astribot_msgs/lib:${SDK_ROOT}/third_party/software/ruckig/lib:${ROS_ROOT}/opt/rviz_ogre_vendor/lib:${ROS_ROOT}/lib/x86_64-linux-gnu:${ROS_ROOT}/lib:${SDK_ROOT}/astribot_sdk/core/common/robotics_library_py:${SDK_ROOT}/astribot_sdk/core/common/whole_body_control/third_party:${SDK_ROOT}/third_party/drake/lib:${LD_LIBRARY_PATH:-}"
  export AMENT_PREFIX_PATH="${SDK_ROOT}/third_party/third_pkg/pinocchio:${SDK_ROOT}/third_party/third_pkg/hpp-fcl:${SDK_ROOT}/astribot_msgs:${SDK_ROOT}/third_party/software/astribot_ros_middleware:${ROS_ROOT}:${AMENT_PREFIX_PATH:-}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  "${OWNER_ENV}/bin/python" - <<'PY' >/dev/null 2>&1
import astribot_ros_middleware
import rclpy
from astribot_msgs.msg import RobotJointController
from astribot_sdk.core.astribot_api.astribot_client import Astribot
PY
)

doctor() {
  local failures=0
  local warnings=0

  pass() { echo "PASS $*"; }
  fail() { echo "FAIL $*" >&2; failures=$((failures + 1)); }
  warn() { echo "WARN $*" >&2; warnings=$((warnings + 1)); }

  if [[ "$(uname -m)" == "x86_64" ]]; then
    pass "architecture=x86_64"
  else
    fail "architecture=$(uname -m), expected x86_64"
  fi

  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
    pass "operating_system=ubuntu-22.04"
  else
    fail "operating_system=${ID:-unknown}-${VERSION_ID:-unknown}, expected ubuntu-22.04"
  fi

  if [[ "$(/usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" == "3.10" ]]; then
    pass "system_python=3.10"
  else
    fail "system Python must be 3.10"
  fi

  if "${REPO_ROOT}/scripts/setup_robot_sdks.sh" doctor astribot >/dev/null 2>&1; then
    pass "astribot_sdk=verified"
  else
    fail "Astribot SDK is not verified; run scripts/setup_astribot_host.sh prepare"
  fi
  if [[ "$(realpath -m "${SDK_ROOT}")" == "$(realpath -m "${PINNED_SDK_ROOT}")" ]]; then
    pass "astribot_sdk_root=${PINNED_SDK_ROOT}"
  else
    fail "ASTRIBOT_SDK_ROOT overrides the pinned runtime and requires separate qualification: ${SDK_ROOT}"
  fi

  if [[ -f "${ROS_ROOT}/setup.bash" ]]; then
    pass "ros2_humble=${ROS_ROOT}"
  else
    fail "ROS 2 Humble setup is missing: ${ROS_ROOT}/setup.bash"
    warn "install it with: sudo scripts/install_astribot_system_deps.sh"
  fi

  for package in \
    libgsl27 \
    libmumps-seq-5.4 \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-ros-base \
    ros-humble-tf-transformations
  do
    if dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q '^install ok installed$'; then
      pass "system_package=${package}"
    else
      fail "required Astribot runtime package is missing: ${package}"
    fi
  done

  if [[ -x "${OWNER_ENV}/bin/python" ]] && "${OWNER_ENV}/bin/python" - <<'PY' >/dev/null 2>&1
import dora
import numpy
import pyarrow
import serial
import websockets
import yaml
import XPolicyLab
PY
  then
    pass "astribot_owner_environment=${OWNER_ENV}"
  else
    fail "Astribot owner environment is incomplete: ${OWNER_ENV}"
  fi
  if [[ -x "${OWNER_ENV}/bin/python" ]] && [[ "$("${OWNER_ENV}/bin/python" -c 'import numpy; print(numpy.__version__)' 2>/dev/null || true)" == "1.26.4" ]]; then
    pass "astribot_owner_numpy=1.26.4"
  else
    fail "Astribot owner NumPy must be 1.26.4 for the vendor SDK ABI"
  fi

  if [[ -f "${ROS_ROOT}/setup.bash" && -x "${OWNER_ENV}/bin/python" ]]; then
    if sdk_import_probe; then
      pass "astribot_sdk_python_imports=verified_no_connect"
    else
      fail "Astribot SDK/ROS Python imports failed without constructing the SDK client"
    fi
  else
    warn "Astribot SDK/ROS import probe is deferred until ROS and the owner environment exist"
  fi

  if [[ -x "${DAGGER_ENV}/bin/dora" ]] && [[ -x "${DAGGER_ENV}/bin/python" ]]; then
    pass "dora_runtime=${DAGGER_ENV}"
  else
    fail "Dora runtime is incomplete: ${DAGGER_ENV}"
  fi

  if [[ -x "${DAGGER_ENV}/bin/exo_driver_node" ]] && [[ -x "${DAGGER_ENV}/bin/xpolicylab-qnexo-controls" ]]; then
    pass "qnexo_runtime=${DAGGER_ENV}"
  else
    fail "QNexo Dora entry points are missing from ${DAGGER_ENV}"
  fi

  if [[ -x "${RECORDER}" ]]; then
    pass "recorder=${RECORDER}"
  else
    fail "release recorder is missing: ${RECORDER}"
  fi

  if ip -4 -o addr show 2>/dev/null | awk '{print $4}' | grep -Eq '^192\.168\.0\.[0-9]+/'; then
    pass "astribot_network=192.168.0.x"
  else
    warn "no 192.168.0.x interface is configured; this is required on the physical robot host"
  fi

  echo "SUMMARY failures=${failures} warnings=${warnings}"
  ((failures == 0))
}

prepare() {
  local uv_bin
  uv_bin="$(resolve_uv)"
  "${REPO_ROOT}/scripts/setup_robot_sdks.sh" cache astribot
  "${REPO_ROOT}/scripts/setup_robot_sdks.sh" prepare astribot --offline
  "${REPO_ROOT}/scripts/setup_robot_sdks.sh" doctor astribot

  if [[ ! -x "${OWNER_ENV}/bin/python" ]]; then
    "${uv_bin}" venv --python /usr/bin/python3 "${OWNER_ENV}"
  fi
  "${uv_bin}" pip install \
    --python "${OWNER_ENV}/bin/python" \
    --requirements "${OWNER_REQUIREMENTS}"
  "${uv_bin}" pip install \
    --python "${OWNER_ENV}/bin/python" \
    --no-deps \
    --editable "${REPO_ROOT}"

  doctor
}

(($# == 1)) || { usage >&2; exit 2; }
case "$1" in
  prepare) prepare ;;
  doctor) doctor ;;
  --help|-h) usage ;;
  *) usage >&2; exit 2 ;;
esac
