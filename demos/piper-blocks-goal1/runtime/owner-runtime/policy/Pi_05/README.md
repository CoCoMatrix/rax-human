# Pi_05

**Contributor:** RoboDojo Team | **Paper:** Pi0.5 technical report | **arXiv:** TBD | **Original code:** https://github.com/Physical-Intelligence/openpi

`Pi_05` adapts Physical Intelligence's π0.5 policy to XPolicyLab/RoboDojo through the uv-managed OpenPI stack. Integration scripts live at this directory level; the vendored upstream implementation lives in `openpi/`.

Shared conventions — argument meanings, checkpoint naming, split-machine deployment, `EVAL_ENV_TYPE` — are documented in the [XPolicyLab README](../../README.md). Official results: [RoboDojo LeaderBoard](https://robodojo-benchmark.com/LeaderBoard).

## Installation

```bash
cd XPolicyLab/policy/Pi_05
bash install.sh
source openpi/.venv/bin/activate  # OpenPI is uv-managed; there is no policy conda env
```

`eval.sh` arg 9 is not a conda env: pass `uv` (uses `deploy.yml` `policy_uv_env_path`) or an explicit OpenPI project path.

## Data Processing

Converts RoboDojo demonstrations into the LeRobot repo consumed by training. The optional `expert_data_num` caps episodes for data conversion only (it is not part of checkpoint naming); the optional `raw_task_dirs` is a source task directory or comma-separated task list under `data/<bench_name>/` (defaults to `ckpt_name`). `raw_task_dirs` may also be passed directly as the 5th argument to write a differently named dataset from all of a task's demos, e.g. `bash process_data.sh RoboDojo stack_bowls_ablation arx_x5 joint stack_bowls`.

```bash
cd XPolicyLab/policy/Pi_05
bash process_data.sh <bench_name> <ckpt_name> <env_cfg_type> <action_type> [expert_data_num] [raw_task_dirs]

# Example: convert stack_bowls demos for arx_x5 joint control
bash process_data.sh RoboDojo stack_bowls arx_x5 joint

# Example: create a 50-episode ablation while reading from the original task data
bash process_data.sh RoboDojo stack_bowls_50ep arx_x5 joint 50 stack_bowls
```

## Training

```bash
cd XPolicyLab/policy/Pi_05
bash train.sh <bench_name> <ckpt_name> <env_cfg_type> <action_type> <seed> <gpu_id>

# Example: train a cotrain run on GPU 0 (comma-separated gpu_id for multi-GPU)
bash train.sh RoboDojo cotrain arx_x5 joint 0 0
```

Checkpoints land in `checkpoints/<bench_name>-<ckpt_name>-<env_cfg_type>-<action_type>-<seed>/`; at eval time `ckpt_name` may be the short run name (auto-combined into that directory name), the full run-directory name, or a path to a checkpoint directory. By default training reads the LeRobot repo produced by `process_data.sh` (`<bench_name>-<ckpt_name>-<env_cfg_type>-<action_type>`); override with `OPENPI_LEROBOT_REPO_ID` when reusing an existing dataset. `train.sh` sets `fsdp_devices=1` for one visible GPU and `2` for multi-GPU by default (override with `OPENPI_FSDP_DEVICES`).

## Evaluation

```bash
cd XPolicyLab/policy/Pi_05
bash eval.sh <bench_name> <task_name> <ckpt_name> <env_cfg_type> <action_type> <seed> \
  <policy_gpu_id> <env_gpu_id> <policy_uv_env> <eval_env_conda_env>

# Example: evaluate a trained cotrain checkpoint on stack_bowls
bash eval.sh RoboDojo stack_bowls RoboDojo-cotrain-arx_x5-joint-0 arx_x5 joint 0 0 0 uv <eval_env_conda_env>
```

`EVAL_ENV_TYPE=debug` runs the offline wiring check (no simulator); leave it unset or set `EVAL_ENV_TYPE=sim` for RoboDojo simulation. For split-machine deployment via `setup_eval_policy_server.sh` / `setup_eval_env_client.sh`, follow the [Deployment Flow](../../README.md#-deployment-flow).

## GR3 Deployment

GR3 uses the same XPolicyLab WebSocket path as the other policies. The adapter accepts the
standard 33-dim GR3 `ObservationFrame`, runs OpenPI, and scatters the model's 29 output
dimensions into the standard 37-dim raw `ActionChunk`. RTC is unavailable for this policy;
filter, QP, and Mink remain optional stages in the shared robot-side output pipeline.

```bash
bash install.sh
PI05_CHECKPOINT=/home/fourier/siyuan/pi05-experiments-main/checkpoints/pi05_anygrasp/anygrasp_20hr/28000 \
./serve.sh

# Offline replay; this still uses WsModelClient -> PolicyServer.
../../.venv/bin/python -m XPolicyLab.real_world.robots.gr3.replay \
  /mnt/Data/dagger/gr3/jiaxin_728-v5_20260729_025148_099/episode_000000019
```

For the shared Dora/DAgger runtime, use `runtime/robot_dagger/graphs/gr3-policy-dagger.yml`
with `POLICY_URL=ws://127.0.0.1:19001`, `ACTION_CHUNK_SIZE=25`,
`MODEL_BACKEND=pi05`, and `MODEL_CHECKPOINT` set to the checkpoint above.

## Astribot S1 Deployment

The `astribot_s1_tomato_v3` runtime profile loads the exact self-contained
tomato-v3 EMA bundle through its frozen OpenPI source, normalization assets,
and tokenizer. Start the current XPolicyLab WebSocket policy server with:

```bash
UV_PROJECT_ENVIRONMENT=../../.runtime/pi05-astribot-tomato-v3 \
  GIT_LFS_SKIP_SMUDGE=1 uv sync \
  --project /home/ubuntu/astribot_pick_tomato_v3_pi05_ema_step20000_ckpt/runtime/openpi_source \
  --frozen --no-dev
uv pip install \
  --python ../../.runtime/pi05-astribot-tomato-v3/bin/python \
  -e ../..

PI05_BUNDLE_ROOT=/home/ubuntu/astribot_pick_tomato_v3_pi05_ema_step20000_ckpt \
  bash serve_astribot.sh
```

The model environment is intentionally outside the bundle so installing local
transport dependencies does not invalidate the bundle manifest.

The shared Astribot observation still carries head, left-wrist, and
right-wrist RGB. This checkpoint consumes only `cam_head`; the two wrist images
remain available to other policies but are deliberately ignored. Its input
prompt must exactly equal `pick the tomato`.

The bundle returns 30 absolute full25 targets at 30 Hz. XPolicyLab maps the
core22 fields to its existing full-body Astribot schema and forces the first
three velocity-based chassis fields to exact zero. It never interprets the
observed chassis pose as a velocity command. Server startup verifies the
checkpoint's config, step, EMA semantics, source commit, prompt, and contract
identity before restoring the model.

Run the environment-only robot check separately; it does not construct the
Astribot SDK object:

```bash
bash ../../scripts/astribot_s1.sh doctor
```

The current `owner-shadow` command uses a measured hold chunk and is not a real
model-shadow claim. Connecting cameras/state to the real policy server remains
operator-gated and must keep hardware writes disabled until its no-send report
passes.

## Configuration

`deploy.yml` keys to check before evaluation: `checkpoint_num`, `result_dir`, `obs_transform_pipeline`, `policy_uv_env_path`, `train_config_name` (must match the config used by `train.sh`), `repo_id`.

Environment variables used by the adapter scripts:

| Variable | Notes |
|---|---|
| `OPENPI_LEROBOT_REPO_ID` | Overrides the LeRobot repo id used by `train.sh`; defaults to `<bench_name>-<ckpt_name>-<env_cfg_type>-<action_type>`. |
| `OPENPI_FSDP_DEVICES` | Overrides the FSDP device count passed to OpenPI training. |
| `OPENPI_TRAIN_CONFIG_NAME` | Overrides the training config; defaults to `pi05_base_aloha_full_sim_arx-x5_seed_0`. |
| `OPENPI_DATA_MODE` | Data-processing mode passed to `openpi/scripts/process_data.py`; defaults to `image`. |
| `OPENPI_LOCAL_CACHE_ROOT` | Per-host local cache root for the HF datasets / JAX compilation caches; defaults to `/tmp/openpi-cache-$(hostname)`. |

`OPENPI_ROOT` and `OPENPI_SRC` are additional overrides consumed by the local scripts.

## Piper X dual-arm desktop-cleanup checkpoints

For a repeatable handoff when onboarding any new dual-PiPER bundle, follow
[Dual-PiPER new model bundle setup](../../docs/piper-new-model-bundle-setup.md).
It covers Aliyun/PPU intake, immutable verification, contract comparison,
runtime preparation, adapter registration, dry-run qualification and rollback.

Dual-PiPER launcher choices are declared in
`policy/Pi_05/deployments/*.yml` and loaded by
`policy/Pi_05/piper_deployments.py`; adding a compatible checkpoint no longer
requires a model-specific branch in `scripts/launch_robot.sh`. Useful commands:

```bash
python policy/Pi_05/piper_deployments.py list
python policy/Pi_05/piper_deployments.py show desktop-cleanup-ema15k
python policy/Pi_05/piper_deployments.py onboard build-blocks-ema12k
python policy/Pi_05/piper_deployments.py onboard desktop-cleanup-h32-v3-ema10k
python policy/Pi_05/piper_deployments.py qualify desktop-cleanup-h32-v3-ema10k
python policy/Pi_05/piper_deployments.py qualify desktop-cleanup-h32-v3-ema19999
python policy/Pi_05/piper_deployments.py onboard paperall50-h32-v1-ema4999
python policy/Pi_05/piper_deployments.py qualify paperall50-h32-v1-ema4999
python policy/Pi_05/piper_deployments.py onboard dagger50-h32-v1-ema4999
python policy/Pi_05/piper_deployments.py qualify dagger50-h32-v1-ema4999
python policy/Pi_05/piper_deployments.py qualify piper-close-laptop-bridgewam-step40550
```

`desktop-cleanup-h32-v3-ema10k` freezes the 2026-08-27 h32 v3 EMA step-10000
contract at 30 Hz, 32 predicted actions, and a default 16-action execution
prefix. Its descriptor also selects the publisher-matched 1280x720@30
RealSense capture profile without changing the robot's physical/safety
profile. It uses the cadence-specific 30 Hz keyboard graph and shares the
parameterized 30 Hz DAgger graph with the final step-19999
deployment. Select `--dagger-mode mixed` for model plus QNexo takeover or
`--dagger-mode data` for expert-only collection; see
[Dual-PiPER DAgger collection](../../docs/piper-dagger-collection.md). Pigey
combined mode remains qualified only for
`desktop-cleanup-ema15k`; attempting `--pigey` with the h32 v3 deployment fails
closed until that supervisor path receives a separate timing canary.

`desktop-cleanup-h32-v3-ema19999` uses the same runtime and observation/action
contract for the final EMA step-19999 checkpoint. Its dedicated descriptor,
setup wrapper, server wrapper and verifier pin the final package identity and
cannot be substituted with the step-10000 bundle. Both exact identities are
recognized by the shared H32-v3 model adapter; all other package IDs fail
closed.

`paperall50-h32-v1-ema4999` registers the 2026-08-28 PaperAll50 final EMA
checkpoint through the same descriptor-driven H32 runtime. It preserves the
30 Hz, 32-action, 16-execute, three-camera and state14 contract, while pinning
its distinct dataset, normalization statistics and canonical prompt catalog.
In particular, its paper task is exactly `Pick up all paper balls and place
them in the trash bin.`; the older singular-paper instruction fails closed.

The same launcher can own the complete Pigey stack. Fixed-task mode is
strictly selected from the CLI launch configuration. Deployments may also
declare immutable `pigey.sequence_presets`; `--pigey-preset-ui` exposes only
those preset IDs in the authenticated operator page. The UI never supplies an
arbitrary Prompt or action. Pigey reconstructs the exact ordered rules from the
local deployment descriptor, skips demonstration induction, and still uses the
live VLM for visual progress/completion evidence. Before starting services, the launcher runs Pigey's
hardware-free input validation. The deployment's explicit `pigey` contract
selects a local or authenticated remote VLM provider, exact prompt catalog,
prompt-level object-progress behavior, and optional recovery bound. It then
starts the authoritative XPolicy graph and waits for its loopback operator API,
all three cameras, fresh 14D state, policy startup, and live-write readiness
before starting Pigey.
`--pigey-check-steps` controls the spacing between VLM observations. In the
legacy bounded mode Pigey pauses at that boundary; in non-blocking supervision
it takes the fresh camera snapshots while the same owner-bounded Pi0.5 rollout
keeps running. Each decision uses labeled left/head/right views; after an ACT it
submits the three fixed per-object baseline views plus three current views.
Qwen reports satisfied/unsatisfied object counts and code derives the
ACT/NEXT_OBJECT/DONE transition. It also reports active manipulation and visual
ambiguity; either blocks progress for every `inside`, `on`, or `closed` rule.
Every apparent NEXT_OBJECT or DONE must satisfy its configured independent
tri-camera confirmation count before code accepts the transition. Consecutive
ACT decisions never reinitialize; in non-blocking mode they also never stop the
live rollout. Each deployment explicitly selects its semantic-boundary
initialization. `dataset_pose` holds the completed rollout and uses the nearest
task-conditioned pre-action pose from an immutable dataset library; Pigey
submits only its pose id and the owner rechecks the library and route.
`profile_home` uses the formal XPolicy stop/prepare lifecycle and waits for the
configured robot profile initial pose. The graph switches to the next exact
Prompt only after the selected physical boundary completes. With
`require_act_before_done`, the validated next rule starts its first ACT without
a redundant Qwen request. The driver waits for physical completion—not just
logical `prepared`—before it captures or starts again:

```bash
python policy/Pi_05/piper_deployments.py run desktop-cleanup-ema15k \
  --pigey \
  --pigey-check-steps 120 \
  --control-source keyboard \
  --task "Pick up one paper ball and place it in the trash bin." \
  --enable-write \
  --open-ui
```

The pen-trash step-9999 preset UI is launched with:

```bash
.venv/bin/python policy/Pi_05/piper_deployments.py run \
  pen-trash-h32-v1-ema9999 \
  --pigey \
  --pigey-preset-ui \
  --pigey-check-steps 150 \
  --pigey-max-task-steps 500 \
  --pigey-reinitialize-steps 0 \
  --pigey-rounds 0 \
  --control-source keyboard \
  --task "Close the laptop." \
  --enable-write \
  --open-ui
```

After startup, select Task 1, Task 2, or Task 3 and click **Run selected
sequence**. The selection starts that sequence; no human-demo recording step is
used. Confirmed NEXT_OBJECT, intermediate DONE, and final ALL_DONE all use the
formal XPolicy `profile_home` prepare boundary. This deployment does not use a
dataset transition-pose library.
For this deployment, `150` remains the live VLM observation interval and `500`
is the cumulative dispatch cap for each original Prompt. `NEXT_OBJECT` does not
reset the 500-step count. At the cap the owner holds and Pigey takes one final
VLM observation for audit. This deployment ignores that final result for
control, marks the current original Prompt DONE, returns through `profile_home`,
and advances to the next Prompt.

Scene-level policies may declare exact prompts in
`pigey.continuous_policy_prompts` when confirmed object progress should resume
from the held pose. PaperAll50 deliberately leaves that list empty: its `ACT`
observations run in non-blocking mode, so the owner keeps dispatching model
actions while the remote VLM request is in flight. Its confirmed `NEXT_OBJECT`
uses a current-task dataset pre-action pose, an intermediate `DONE` uses a
next-task pose, and final `ALL_DONE` uses the safe prepare path once. The 9999
pen-trash deployment uses `profile_home` for confirmed NEXT_OBJECT,
intermediate DONE, and final DONE; it loads no dataset transition library. The
generic `pigey.reinitialize_after_steps` watchdog counts control steps since
the last confirmed progress boundary, shortens the final rollout window to hit
the bound exactly, and then invokes the selected initialization mode before
retrying the same rule. The
PaperAll50 contract uses 750 steps. Its remote VLM contract uses
`qwen3.6-flash` for both live tri-camera decisions and full demonstration
induction; the credential remains in a mode-0600 file outside Git. Ordered
demonstrations also enable `require_act_before_done`: the graph
switches to the next exact policy prompt before capturing that task's initial
three-camera baseline. For an intermediate task transition, code directly
starts that validated prompt's first ACT because task-start DONE is forbidden;
this removes one remote-VLM round-trip while still requiring a positive
baseline-to-current completion change afterward.
That transition-grounded DONE is accepted on its first detection in the
remote-Qwen profile; NEXT_OBJECT retains two-detection confirmation.

For `object50-h32-v1-ema6000`, the deployment pins
`transition_poses/object50-h32-v1-preaction-onset-20260902-v3.json` by
SHA-256. It contains 12 actual recorded follower-state medoids per
paper/pen/mouse/drink task. Candidates are anchored eight 30 Hz frames (about
267 ms) before the first sustained joint-motion onset, and are capped at the
first 22% of their source episode; none is selected from the middle or tail of
the rollout. Each has at least 120 future 30 Hz frames and meaningful future
motion, remains separated from the terminal state, and passes flange-height and
vendor joint-boundary checks. Terminal/post-completion frames and
averaged/synthetic poses are rejected by both Pigey and the owner. A prompt
without a qualified library entry still fails closed. If the measured current
pose cannot reach the selected candidate directly (joint bound or flange-height
floor), the sole owner may route through the profile's clear/home arm pose,
split both legs into direct-bound joint waypoints, and validate every
interpolated segment before dispatch; this keeps the transition moving without
disabling the safety guard or starting a new lifecycle prepare. If that bounded
home route is unsafe, the owner fails closed. The current
demonstrations still begin from
nearly the same physical start configuration, so “many” early candidates are
real episode variants but cannot create arbitrary randomized starts; additional
varied-start demonstrations are required for that. This is not a collision
certificate: the checked URDF is kinematics-only, so a low-speed supervised
hardware canary remains required.

```bash
/usr/bin/python3 policy/Pi_05/piper_deployments.py run \
  paperall50-h32-v1-ema4999 \
  --pigey \
  --pigey-check-steps 120 \
  --pigey-reinitialize-steps 750 \
  --control-source keyboard \
  --task "Pick up all paper balls and place them in the trash bin." \
  --enable-write \
  --open-ui
```

To repeat the same fixed task or already-recorded demonstration without
restarting the model server or XPolicy graph, Pigey now keeps running warm
rounds until Ctrl+C. It writes each episode under `round_###/` and performs a
single configured prepare pose at final ALL_DONE, then a fail-closed stop at
the round boundary; it does not repeat the same home trajectory or settle
delay before waiting in `ROUND_READY` for an explicit **Start**. Use
`--pigey-rounds N` for a finite run; `--pigey-rounds 0` (the default) means
unlimited rounds. For a video/manifest demo, visual rule induction runs once
and the validated rules are reused when **Start** launches later rounds. With
`--pigey-record-demo`, `ROUND_READY` also enables **Start recording**: the
robot remains idle at its initialized pose, and finishing the new recording
automatically extracts frames, replaces the cached sequence through fresh
induction, and starts the next round. The remote profile uses `qwen3.6-flash`
for both demo induction and live checks.

```bash
/usr/bin/python3 policy/Pi_05/piper_deployments.py run \
  paperall50-h32-v1-ema4999 \
  --pigey \
  --control-source keyboard \
  --task "Pick up all paper balls and place them in the trash bin." \
  --enable-write \
  --open-ui
```

In combined mode, manual task selection remains locked from graph startup, but
the I/S/X/F lifecycle controls always override or gate Pigey through the
authenticated operator API. **X** reaches the sole hardware owner immediately;
**I** invalidates an in-flight VLM decision, initializes through the formal
prepare pose, and pauses; **S** resumes the current cached sequence. A bounded
command history preserves fast inputs such as X → I → S.
The task selector remains for normal `--no-pigey` operation and is not a live
task switch while the combined Pigey supervisor is active.

To record a visual human demonstration and hand it directly to Pigey, use the
combined recording mode:

```bash
/usr/bin/python3 policy/Pi_05/piper_deployments.py run desktop-cleanup-ema15k \
  --pigey \
  --pigey-record-demo \
  --pigey-check-steps 120 \
  --control-source keyboard \
  --task "Pick up one paper ball and place it in the trash bin." \
  --enable-write \
  --open-ui
```

The launcher first forces a read-only capture graph even though the final
command includes `--enable-write`. In the **Human demonstration · head/top
camera** panel, click **Start recording**, perform the ordered tasks, then click
**Finish recording & run Pigey**. That button finalizes the recording, and the same launcher
automatically validates it, replaces the capture graph with the live graph,
loads the recording into Pigey, infers the ordered task sequence, and starts
execution. There is no path to copy and no second command. The capture phase
never sends a robot command; live writes are not enabled until the saved input
passes validation.

A completed recording keeps chronological JPEGs, `demo_manifest.json`, and
`human_demo.mp4` under the visible `piper_human_demos/<timestamp>/` directory for audit and
re-use. `PIPER_OPERATOR_DEMO_ROOT`,
`PIPER_OPERATOR_DEMO_FPS` (default 5), and
`PIPER_OPERATOR_DEMO_MAX_SECONDS` (default 600) configure capture.

In a normal deployment (`--no-pigey` and no DAgger), the same UI buttons are a
read-only operator-video recorder. Start it before or during the rollout, then
click **Stop & save**. It writes the three views shown in the dashboard as
`camera_left.mp4`, `camera_head.mp4`, and `camera_right.mp4`, plus the
left/head/right horizontal composite `camera_concat.mp4`, under:
`Log/operator_recordings/<deployment>/<session>/`. The videos use the UI preview
stream (10 FPS by default, configured by `PIPER_OPERATOR_VIDEO_FPS`) and do not
alter policy actions, lifecycle control, or the single hardware owner. Temporary
JPEGs are removed after all four H.264 encodes succeed; they remain when an
encoder fails. `recording_manifest.json` indexes the files and composite layout;
`video_index.json` maps each shared frame index and timestamp to all four videos
and the corresponding source-camera metadata. `PIPER_OPERATOR_VIDEO_ROOT` and
`PIPER_OPERATOR_VIDEO_MAX_SECONDS` override the root and duration.

An existing video, image directory, or JSON frame manifest remains a
replaceable alternative to UI recording:

```bash
python policy/Pi_05/piper_deployments.py run desktop-cleanup-ema15k \
  --pigey \
  --pigey-demo /absolute/path/to/human-demo.mp4 \
  --pigey-expected-tasks 2 \
  --pigey-check-steps 120 \
  --control-source keyboard \
  --enable-write \
  --open-ui
```

`--pigey-expected-tasks` is optional; omit it to let Pigey infer the count.
It is also accepted with `--pigey-record-demo` when the human demonstration is
known to contain an exact number of tasks. In recording mode, the demonstrated
visual transitions determine the sequence; the launch `--task` is only the
valid initial XPolicy prompt while the read-only graph starts.
Video evidence scales with duration at `--pigey-demo-sample-fps` (default 1), always
includes explicit first and final frames, and is uniformly capped by
`--pigey-demo-max-frames` (default/max 40). The chosen duration, target count, and
actual count are recorded in `demo_input/video_sampling.json`.
Without `--pigey` (or with explicit `--no-pigey`) the launcher preserves the
normal XPolicy behavior. Add `--dry-run` to either combined command to print
the input-validation, Qwen, XPolicy, and Pigey process plan without running any
of them. A pre-existing matching Qwen service is reused and never stopped by
this launcher; every process it starts is stopped together on exit or Ctrl+C.

`onboard` is the first-time path (optional descriptor-defined download, full
verification, setup, tests, temporary policy-server inference, and launcher
dry-run). `qualify` repeats the checks for an installed runtime without setup.
Both are hardware-free, write stage logs plus `qualification.json` under
`.xpolicy/reports/piper/`, and refuse to compete with an active PiPER runtime.
Use `--quick` to omit server/GPU inference or `--dry-run` to inspect the plan.

The `piper_x_dual_desktop_cleanup_v1` profile loads the immutable step 5000 or
step 10000 deployment bundle from the external model store. It consumes the
harness's `camera_head`, `camera_left`, `camera_right`, and ordered 14D Piper
state, maps them to the checkpoint's packaged OpenPI input contract, and
returns ten absolute 14D targets at 15 Hz. The robot scheduler executes the
complete ten-target chunk before requesting another observation. A task must
exactly match one of the five prompts in the bundle contract.

```bash
PI05_BUNDLE_ROOT=/home/ubuntu/michael_dev/model-store/models/pi05/desktop-cleanup-dual-piper-v1/step-10000 \
  bash setup_piper_desktop_cleanup.sh

bash scripts/launch_robot.sh piper_x_dual \
  --deployment desktop-cleanup \
  --model-server managed \
  --ckpt /home/ubuntu/michael_dev/model-store/models/pi05/desktop-cleanup-dual-piper-v1/step-10000 \
  --control-source keyboard \
  --no-write
```

The unified launcher owns both the local model server and Dora graph. It waits
for the WebSocket port before opening the graph and stops both child processes
on Ctrl+C. Use `--model-server external --policy-url ws://HOST:PORT` only when
the policy service is intentionally managed elsewhere.

The model and robot dependencies are installed into separate environments;
the robot environment gets the harness branch's pinned `pyAgxArm` snapshot.
On this RTX 5090, setup keeps the locked model environment intact and prepends
a selective Torch 2.13/CUDA 13 overlay from the existing Python 3.12 Qwen
runtime. Matching Torchvision/Triton/CUDA packages are included; Qwen's
Transformers and NumPy are excluded. A full step-10000 synthetic inference
returned finite `[10,14]` actions. The checkpoint bundle and frozen source
remain unchanged.
The robot graph also serves loopback operator controls at
`http://127.0.0.1:8091`; QNexo and browser keyboard lifecycle events are merged
before entering the same fail-closed Piper owner.
The UI shows three 16:9 feeds with explicit `camera_left`, `camera_head` and
`camera_right` labels. Its execution selector defaults to **Auto**, which reads
the bundle adapter's `execute_prefix_steps` metadata; a fixed prefix from 1 to
the declared model horizon can be selected only while the episode is stopped.
See [the deployment guide](../../docs/piper-desktop-cleanup-deployment.md) for
the complete offline setup and no-send launch flow.

### Desktop-cleanup EMA step 15000

`piper_x_dual_desktop_cleanup_ema_h16_v1` is a separate profile for the
`pi05_dual_piper_real` EMA export. It keeps the same three RGB camera mapping,
ordered 14D Piper state and five exact prompts as the raw profile, but predicts
`[16,14]` and executes the bundle-recommended first eight targets at 15 Hz
before observing again. RTC and policy prefetch remain disabled.

The bundle is immutable at:

```text
/home/ubuntu/michael_dev/model-store/models/pi05/desktop-cleanup-dual-piper-real-ema-v1/step-15000
```

Its OpenPI dependency lock is byte-identical to the raw desktop bundle, so the
setup wrapper safely reuses `.xpolicy/envs/pi05_desktop_cleanup_py312` while
loading source, weights, normalization statistics and tokenizer from the
selected bundle.

```bash
bash policy/Pi_05/setup_piper_desktop_ema15k.sh

bash scripts/launch_robot.sh piper_x_dual \
  --deployment desktop-cleanup-ema15k \
  --control-source keyboard \
  --task "Pick up one paper ball and place it in the trash bin." \
  --no-write
```

The deployment defaults to the promoted step-15000 path and a managed local
model server, so neither `--ckpt` nor a second server command is required.

## Piper X dual-arm build-blocks checkpoint

### Jiaxin EMA step 12000 bundle

Deployment `build-blocks-ema12k` freezes the comparison package
`piper_build_blocks_jiaxin_v1_pi05_ema_step12000_ckpt` without replacing the
older raw `build-blocks` entry. It restores JAX/Orbax EMA parameters from the
bundle's frozen source at commit `abcb0a93d010aaf4c4c4b02fc772b741b23ec2e5`,
requires the one exact English prompt, and returns `[15,14]` absolute Piper
targets at 15 Hz. The adapter explicitly resizes each required RGB input to the
publisher's 224x224 contract before inference.

The immutable local target and mutable prepared runtime are separate:

```text
/home/ubuntu/michael_dev/model-store/models/pi05/piper-build-blocks-jiaxin-v1-ema-v1/step-12000
.xpolicy/runtimes/piper_build_blocks_jiaxin_v1_ema_step12000
```

Setup and hardware-free qualification use the descriptor-driven path:

```bash
python policy/Pi_05/piper_deployments.py onboard build-blocks-ema12k
python policy/Pi_05/piper_deployments.py qualify build-blocks-ema12k

bash scripts/launch_robot.sh piper_x_dual \
  --deployment build-blocks-ema12k \
  --control-source keyboard \
  --task "Stack the second cube on top of the first cube, then place the cone on top of the second cube." \
  --no-write --dry-run
```

The default execution prefix is the complete 15-step published horizon. The
shared client RTC remains off by default and can be switched to measured-latency
align or blend mode from the operator UI; that does not imply model-native RTC.

### Legacy raw checkpoint

The Piper build-blocks adapter is a real-world, three-camera Pi0.5 profile with a
frozen 14D contract: left 6 joints, left gripper width, right 6 joints, right
gripper width. Joint state is absolute radians and gripper state is absolute
meters. The trained model uses joint deltas for the 12 arm dimensions and
absolute widths for the two grippers; the OpenPI output transform reconstructs
the complete 14D absolute target before XPolicyLab creates an `ActionChunk`.

All three runtime cameras (`camera_head`, `camera_left`, `camera_right`) and the
exact training prompt are required. Missing cameras, prompt drift, schema drift,
non-finite values, or any shape other than `[15,14]` fail closed.

```bash
PI05_CHECKPOINT=/absolute/path/to/2000 bash serve_piper.sh
```

This command starts only the WebSocket PolicyServer. The tracked Piper Dora
graph contains the robot-independent WebSocket/ActionChunk client and routes its
14D output through the same owner and final atomic PASS/HOLD gate used by expert
takeover. Starting the server alone does not authorize CAN access, start the
Piper owner, or enable robot writes. The Piper graph still has no recorder.

For a supervised full-chain run, start the server first and then use the formal
launcher:

```bash
scripts/launch_robot.sh piper_x_dual --enable-write \
  --ckpt /absolute/path/to/2000 \
  --policy-url ws://127.0.0.1:19003
```
