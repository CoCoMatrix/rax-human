#!/usr/bin/env bash
set -euo pipefail
typec_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
source "$typec_repo/policy/Pi05TypeCRTC/environment.sh"
cd "$typec_repo"
typec_command="${1:-check}"
if (( $# )); then shift; fi
case "$typec_command" in
  show|check|onboard|qualify)
    exec /usr/bin/python3 -B policy/Pi_05/piper_deployments.py "$typec_command" typec-task11-rtc-ema10000-instance02 "$@"
    ;;
  serve)
    exec bash policy/Pi05TypeCRTC/serve.sh "$@"
    ;;
  run)
    # The standard launcher defaults to no-write. Operator adds --enable-write
    # only for a separately supervised live run; this script never sends Start.
    exec /usr/bin/python3 -B policy/Pi_05/piper_deployments.py run typec-task11-rtc-ema10000-instance02 \
      --model-server managed --model-port 19031 --policy-url ws://127.0.0.1:19031 --no-pigey --control-source keyboard \
      --operator-ui-port 8098 --no-model-filter "$@"
    ;;
  *) echo "Usage: bash scripts/typec_rtc.sh {show|check|onboard|qualify|serve|run} [options]" >&2; exit 2 ;;
esac
