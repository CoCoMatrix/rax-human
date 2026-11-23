#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sudo install_piper_can_device.sh --serial <USB serial> --name <stable netdev>
  install_piper_can_device.sh --serial <USB serial> --name <stable netdev> --dry-run

The stable name is an adapter identity, not a robot role. Use names such as
can_px_002b001f, then bind that identity to left_arm/right_arm in the robot's
site-local device binding.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LINK_TEMPLATE="${RUNTIME_ROOT}/config/systemd/10-xpolicylab-piper-can.link.in"
SERVICE_SOURCE="${RUNTIME_ROOT}/config/systemd/xpolicylab-piper-can@.service"
UDEV_SOURCE="${RUNTIME_ROOT}/config/udev/90-xpolicylab-piper-can.rules"

can_serial=""
can_name=""
dry_run=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      can_serial="$2"
      shift 2
      ;;
    --name)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      can_name="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "${can_serial}" =~ ^[0-9A-Fa-f]+$ ]]; then
  echo "--serial must be a non-empty hexadecimal USB serial." >&2
  exit 2
fi

if [[ ! "${can_name}" =~ ^can_px_[a-z0-9]+$ ]] || [[ ${#can_name} -gt 15 ]]; then
  echo "--name must match can_px_[a-z0-9]+ and be at most 15 characters." >&2
  exit 2
fi

for required in "${LINK_TEMPLATE}" "${SERVICE_SOURCE}" "${UDEV_SOURCE}"; do
  if [[ ! -f "${required}" ]]; then
    echo "Missing installer asset: ${required}" >&2
    exit 1
  fi
done

link_name="10-xpolicylab-piper-${can_name}.link"
rendered_link="$(
  sed \
    -e "s/@CAN_SERIAL@/${can_serial}/g" \
    -e "s/@CAN_NAME@/${can_name}/g" \
    "${LINK_TEMPLATE}"
)"

if [[ "${dry_run}" == true ]]; then
  printf 'Target: /etc/systemd/network/%s\n%s\n' "${link_name}" "${rendered_link}"
  printf 'Target: /etc/systemd/system/xpolicylab-piper-can@.service\n'
  printf 'Target: /etc/udev/rules.d/90-xpolicylab-piper-can.rules\n'
  exit 0
fi

if [[ ${EUID} -ne 0 ]]; then
  echo "Run this installer with sudo or pkexec." >&2
  exit 1
fi

install -d -m 0755 /etc/systemd/network /etc/systemd/system /etc/udev/rules.d
printf '%s\n' "${rendered_link}" > "/etc/systemd/network/${link_name}"
chmod 0644 "/etc/systemd/network/${link_name}"
install -m 0644 "${SERVICE_SOURCE}" /etc/systemd/system/xpolicylab-piper-can@.service
install -m 0644 "${UDEV_SOURCE}" /etc/udev/rules.d/90-xpolicylab-piper-can.rules

systemctl daemon-reload
udevadm control --reload-rules

echo "Installed Piper CAN identity ${can_name} for serial ${can_serial}."
echo "Replug that USB-CAN adapter; it will be renamed and configured automatically."
