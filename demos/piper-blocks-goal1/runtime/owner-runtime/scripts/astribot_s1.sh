#!/usr/bin/env bash
set -euo pipefail

XPOLICYLAB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASTRIBOT_BUNDLED_SDK_ROOT="${XPOLICYLAB_ROOT}/third_party/robot_sdks/astribot_s1/runtime/astribot_sdk_ros2-master"
ASTRIBOT_RUNTIME_SDK_ROOT="${ASTRIBOT_SDK_ROOT:-${ASTRIBOT_BUNDLED_SDK_ROOT}}"
ASTRIBOT_RUNTIME_ROS_ROOT="${ASTRIBOT_ROS_ROOT:-/opt/ros/humble}"
ASTRIBOT_RUNTIME_PYTHON="${ASTRIBOT_OWNER_PYTHON:-${XPOLICYLAB_ROOT}/.runtime/astribot-owner/bin/python}"

if [[ ! -f "${ASTRIBOT_RUNTIME_SDK_ROOT}/env.sh" ]]; then
  echo "Astribot SDK env.sh not found: ${ASTRIBOT_RUNTIME_SDK_ROOT}/env.sh" >&2
  echo "Run: scripts/setup_robot_sdks.sh prepare astribot_s1" >&2
  exit 1
fi
if [[ ! -f "${ASTRIBOT_RUNTIME_ROS_ROOT}/setup.bash" ]]; then
  echo "ROS 2 Humble setup not found: ${ASTRIBOT_RUNTIME_ROS_ROOT}/setup.bash" >&2
  echo "Run: scripts/setup_astribot_host.sh doctor" >&2
  exit 1
fi
if [[ ! -x "${ASTRIBOT_RUNTIME_PYTHON}" ]]; then
  echo "Astribot owner environment not found: ${ASTRIBOT_RUNTIME_PYTHON}" >&2
  echo "Run: scripts/setup_astribot_host.sh prepare" >&2
  exit 1
fi

# The vendor SDK and ROS2 Humble are tied to the system Python 3.10 install.
set +u
source "${ASTRIBOT_RUNTIME_ROS_ROOT}/setup.bash"
source "${ASTRIBOT_RUNTIME_SDK_ROOT}/env.sh"
set -u
export PYTHONPATH="${XPOLICYLAB_ROOT}:${PYTHONPATH:-}"

exec "${ASTRIBOT_RUNTIME_PYTHON}" -m XPolicyLab.real_world.robots.astribot_s1.cli "$@"
