#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer as root:" >&2
  echo "  sudo scripts/install_astribot_system_deps.sh" >&2
  exit 2
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "Astribot system dependencies require Ubuntu 22.04." >&2
  exit 1
fi
if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "Astribot system dependencies require x86_64." >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y ca-certificates curl software-properties-common
add-apt-repository -y universe

ros_source_version="$(
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | /usr/bin/python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])'
)"
[[ "${ros_source_version}" =~ ^[0-9][0-9.~-]*$ ]] || {
  echo "Could not resolve a valid ros2-apt-source release version." >&2
  exit 1
}

temp_dir="$(mktemp -d)"
trap 'rm -rf -- "${temp_dir}"' EXIT
ros_source_deb="${temp_dir}/ros2-apt-source.deb"
curl -fL -o "${ros_source_deb}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb"
dpkg -i "${ros_source_deb}"

apt-get update
apt-get install -y \
  libgsl27 \
  libmumps-seq-5.4 \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-ros-base \
  ros-humble-tf-transformations

test -f /opt/ros/humble/setup.bash
echo "Astribot ROS 2/system dependencies are installed."
echo "Next: scripts/setup_astribot_host.sh doctor"
