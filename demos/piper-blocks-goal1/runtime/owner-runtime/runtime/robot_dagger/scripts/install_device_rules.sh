#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULES_DIR="${PROJECT_ROOT}/config/udev"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer with sudo." >&2
  exit 1
fi

for name in \
  80-gr3-oak.rules \
  80-gr3-qnbot.rules \
  80-xpolicylab-realsense.rules \
  99-gr3-footswitch.rules; do
  source_path="${RULES_DIR}/${name}"
  if [[ ! -f "${source_path}" ]]; then
    echo "Missing device rule: ${source_path}" >&2
    exit 1
  fi
  install -m 0644 "${source_path}" "/etc/udev/rules.d/${name}"
done

udevadm control --reload-rules
udevadm trigger --subsystem-match=usb
udevadm trigger --subsystem-match=video4linux
udevadm trigger --subsystem-match=tty
udevadm trigger --subsystem-match=input
udevadm settle --timeout=10

echo "XPolicyLab OAK, RealSense, QNexo, and footswitch udev rules installed."
