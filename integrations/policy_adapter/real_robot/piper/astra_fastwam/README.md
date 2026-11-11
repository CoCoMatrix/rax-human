# Astra + FastWAM / PiPER

This package reuses the upstream `CodexPolicy.run()` persistent Codex app-server
event loop. It adapts the robot, student, observation files and action schema.
The original outcome/intent gate is copied byte-for-byte into the dedicated skill.
`pi05_infer` and `robodojo_*` are event-loop compatibility names only: runtime
packets explicitly identify **FastWAM / PiPER**.

`--mode shadow` is the default. The one-cycle sequence is observation →
FastWAM H32 infer → native persistent GPT6/xhigh review → candidate validation →
finish. No POST, SDK, CAN, graph launch, home or enable operation exists here.
Authorized live mode uses `live_io.py`, `live_tools.py` and `relay.py`: reviewed
prefixes travel through the original formal graph and sole owner, with ordered
ACK receipts. The agent has no independent hardware writer.

```bash
cd /home/ubuntu/code_junqi/GPT-as-Policy
PYTHONPATH="$PWD:/home/ubuntu/code_junqi/xpolicylab-robot-runtime" \
  .venv-piper/bin/python -m real_robot.piper.astra_fastwam \
  --owner-url http://127.0.0.1:8091 \
  --student-url ws://127.0.0.1:19023 \
  --output /home/ubuntu/code_junqi/migration/astra-fastwam/NEW_SHADOW \
  --observation-dir /home/ubuntu/code_junqi/migration/astra-fastwam/observation \
  --task '把棕色瓶子放到杯垫上' --mode shadow
```

If the existing ChatGPT login has exhausted its Codex usage allowance, use
`python -m real_robot.piper.astra_fastwam.api_shadow` with the same arguments.
This keeps the original native persistent agent and fixed Astra/xhigh model,
using the already authorized `/home/ubuntu/code_junqi/key.py` API credential via
an environment-key provider. The key is not written to auth.json/config/logs or
command-line arguments. Native analysis shells exclude the API-key variables and
have sandbox network disabled. The dedicated provider configuration is under
`/home/ubuntu/code_junqi/.codex-astra-fastwam-api`.

The parent launcher must already provide the owner and model services. Existing
personal Codex authentication defaults to `/home/ubuntu/code_junqi/.codex-gpt-policy`.
This module never reads, copies or prints credentials. Override `--codex-home`
or `--model-provider` only with an existing authorized configuration.

Lossless capture contract: `--observation-dir` contains `latest.json` (or
`metadata.json`) with timestamp_ns, sequence, arrays_path (default observation.npz),
optional metadata; NPZ contains state14 and camera_head/camera_left/camera_right
uint8 RGB[720,1280,3]. The NPZ is immutable. UI JPEG previews must not be resized
or passed as model inputs. Frozen snapshot age is recorded, not presented as
execution freshness. Default maximum snapshot age is1800s.

The existing owner may have its old write gate enabled, but must stay HOLD with
the same hardware-write count. This process does not command it. The task text
in that old owner's UI does not replace FastWAM's pinned training conditioning.

Outputs: status.json/events.jsonl/result.json, full original observation/proposal
NPZ, raw+effective boundary-normalization distinction, selected candidate and
shadow receipt. Success here means shadow validation, never physical task success.

## Explicit live invocation

The operator must first authorize the run. Use a fresh run-id and fresh directory.
The outer launcher, not this module, starts the existing formal robot graph and
performs its original lifecycle controls. Never start a second owner.

```bash
# All three components must share RUNBASE/RUNID and the same receipt journal.
python -m real_robot.piper.astra_fastwam.relay \
  --exchange-dir "$RUNBASE/relay" --run-id "$RUNID" \
  --authorize-dispatch --port 19026

# Run the existing scripts/launch_robot.sh graph with policy URL19026, canonical
# FastWAM conditioning, and PIPER_REVIEW_RECEIPT_JOURNAL=$RUNBASE/owner-receipts.jsonl.
# Required release profile variables:
# PIPER_REVIEW_PROFILE=policy_prefix PIPER_BOUNDED_REVIEW_RELEASE=1
# PIPER_CONTINUOUS_REVIEW=1 PIPER_GPT_TRAJECTORY_RELEASE=0
# POLICY_PREFETCH_ENABLED=0 ACTION_RTC_MODE=disabled XPOLICY_ALLOW_NO_ACTION=0
# POLICY_EXECUTE_CHUNK_MODE=auto POLICY_EXECUTE_CHUNK_SIZE=15
# POLICY_ACTION_FREQUENCY_HZ=30

python -m real_robot.piper.astra_fastwam.api_shadow \
  --mode live --authorize-dispatch --run-id "$RUNID" \
  --exchange-dir "$RUNBASE/relay" --journal "$RUNBASE/owner-receipts.jsonl" \
  --output "$RUNBASE/agent" --owner-url http://127.0.0.1:8091 \
  --student-url ws://127.0.0.1:19023 --task '把棕色瓶子放到杯垫上'
```

The API helper's compatibility module name does not override `--mode`. Live mode
requires all four explicit authorization/run/relay/journal parameters and uses a
dedicated live skill with the unchanged gate. It repeatedly infers a fresh model
proposal after each observed segment. Student releases contain1–15 steps; edit/eef
releases contain one step each and recompute IK after its actual ACK, up to5 total.
No skipped initial actions, H50 padding, FFS/SAM3, or GPT-Policy planner is added.

The current owner profile/URDF and inward float32 command encoder are reused.
Unused H32 suffix violations remain reported but do not veto a legal selectable
prefix. No physical range is widened. Formal ACK means SDK dispatch, not arrival
or grasp. Missing/uncertain ACK stops this run and never resends that command.
