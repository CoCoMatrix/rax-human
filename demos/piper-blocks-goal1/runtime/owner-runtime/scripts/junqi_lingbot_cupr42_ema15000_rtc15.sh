#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
bundle="/home/ubuntu/code_junqi/model-store/models/lingbot/realcup_cupr42_rtc15_ema_step15000_20261016"
cd "$repo"
# A different profile must receive its own reviewed launcher and qualification.
.xpolicy/envs/piper_qualification/bin/python -B - "$bundle" <<'PY'
import json,sys
from pathlib import Path
from XPolicyLab.policy.LingBotCupR42EMA15KRTC15.verify_bundle import verify
p=Path(sys.argv[1]);verify(p);c=json.loads((p/'runtime_processing.json').read_text())
assert c['native_rtc']==dict(future_budget_steps=12,commit_steps=15,max_prefix_steps=15,horizon=50,frequency_hz=30)
assert c['fallback']['enabled'] is False
PY
export PIPER_TRAINING_RTC_ENABLED=1 PIPER_NATIVE_RTC_FUTURE_STEPS=12 PIPER_NATIVE_RTC_MAX_PREFIX_STEPS=15 PIPER_NATIVE_RTC_WALLCLOCK_DEADLINE=1
export ACTION_RTC_MODE=disabled ACTION_RTC_TRANSITION_STEPS=0 PIPER_MODEL_FILTER_ENABLED=0
unset ACTION_TRAJECTORY_PROFILE PIPER_NATIVE_TRAJECTORY_PROFILE PIPER_NATIVE_ONE_EURO_PROFILE PIPER_NATIVE_JOINT_CLIP_PROFILE PIPER_NATIVE_VELOCITY_SEAM_PROFILE
exec .xpolicy/envs/piper_qualification/bin/python -B policy/Pi_05/piper_deployments.py run "junqi-lingbot-cupr42-ema15000-rtc15-instance01" "$@"
