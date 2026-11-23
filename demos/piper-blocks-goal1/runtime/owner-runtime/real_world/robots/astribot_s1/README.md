# Astribot S1 real-world runtime

This integration has its own robot mapping and SDK owner while reusing the
shared Dora control, ActionChunk scheduling, PolicyServer protocol, and
recorder contracts.
The default QNexo profile controls the two 7-DoF arms through joint positions
and controls the two one-DoF grippers independently from the QNexo triggers.
The physical left joystick controls torso yaw and the physical right joystick
controls torso pitch directly through the uppermost pitch joint. The lower two
pitch joints and head are held, and the
chassis is omitted from QNexo SDK writes; the model/Owner path can command the
complete S1 body.

The public deployment action contract is
`astribot_s1.full_body_joint_position_chassis_velocity.v1` with 25 fields in
fixed order: chassis forward/lateral/yaw velocity 3 (`m/s`, `m/s`, `rad/s`),
torso 4, left arm 7, left gripper 1, right arm 7, right gripper 1, and head 2.
All non-chassis fields are absolute joint positions (grippers use percent; the
other joints use the SDK units). The previous 16D arm/gripper schema remains an
explicit QNexo teleoperation compatibility path, not the model action schema.

## Full-body ActionChunk runtime

`AstribotS1FullBodyRuntime` accepts only the public 25D schema at 30 Hz. It
advances the queued target on the 30 Hz execution grid while rate-gating SDK
dispatch to the vendor direct-control rate of 250 Hz. Every SDK tick applies four independent gates: chassis
velocity, torso/head position, arms, and grippers. Each gate validates the exact
candidate against freshness, schema, finite-value and live SDK hard limits. A
complete valid action passes unchanged; any invalid field rejects the complete
action and enters HOLD. No gate clips a target toward measured feedback or
returns an adjusted command. Valid chassis velocity is integrated on the 250 Hz
grid into the SDK's chassis position command, following the vendor joystick
example. The resulting chassis, torso, arms, grippers, and head are submitted in
exactly one fixed-order SDK position write. This avoids leaving a persistent
chassis velocity command when the runtime holds or stops.

The runtime supports `pause`, one complete chunk with `step`, and continuous
`run`. Generation changes clear queued and held targets. Schema/frequency
mismatches and late chunks are rejected; empty queues hold without writing.
Loss of control rights, stale/invalid robot state, stale commands, SDK liveness
failure, and write faults enter HOLD and attempt the SDK stop path. The runtime
does not connect to a model or start a robot loop by itself.

## Three-camera observations

`AstribotS1CameraSource` subscribes through the official SDK to the decoded
color streams `head_rgbd`, `left_wrist_rgbd`, and `right_wrist_rgbd`. They are
exposed to the policy contract as `cam_head`, `cam_left_wrist`, and
`cam_right_wrist`. SDK images arrive as BGR HWC `uint8` and are converted to
contiguous RGB HWC `uint8` only when a synchronized observation is built.

Each stream keeps 16 recent frames. Because the camera computer and local SDK
state use different clocks, the adapter estimates their common offset from the
latest three source/receive timestamp pairs. It preserves both raw and aligned
timestamps, then selects the aligned frame nearest to each 25D state. The
default gate requires every camera to have a recently received frame (250 ms),
no more than 50 ms skew across the selected cameras, and no more than 100 ms
between a selected frame and robot state. Missing, stale, or skewed input
returns HOLD instead of a partial observation. The SDK currently has one global
callback slot, so all three subscriptions deliberately share one dispatcher
and route frames by topic name.

The read-only camera probe does not send arm or gripper commands. Camera
activation remains explicit because this SDK version ignores per-camera
settings and activates the complete camera module:

```bash
scripts/astribot_s1.sh \
  --acknowledge-sdk-connect \
  camera-observe --activate-cameras --samples 3 --timeout 15
```

## Unified owner shadow runtime

`AstribotS1OwnerRuntime` keeps the camera source and 25D action runtime on one
SDK session. It emits synchronized observations at 30 Hz while ticking all four
independent safety paths on the 250 Hz driver grid. An
observation HOLD prevents the action queue from advancing; robot liveness,
control-rights, state-age, command-age, joint-limit, and write-fault handling
remain owned by the action runtime. `run`, one-chunk `step`, `pause`, and
explicit `stop` share the same generation reset behavior.

The first integration probe is deliberately shadow-only. It captures the
measured 22D non-chassis joint position plus zero chassis velocity as a finite
25D dummy ActionChunk, passes every step through synchronization and safety,
and asserts that the SDK write count remains zero:

```bash
scripts/astribot_s1.sh \
  --acknowledge-sdk-connect \
  owner-shadow --samples 5 --timeout 10
```

Every owner shadow run now creates a versioned rollout report under
`Log/astribot_s1_rollouts` by default. `events.jsonl.partial` is flushed after
each lifecycle, status, observation, or action-step event and atomically becomes
`events.jsonl` on finalization. `manifest.json` fixes the state/action schemas;
`summary.json` records counts, maximum synchronization skew, safety/driver
status histograms, reasons, termination, and hardware-write count. Observation
and action records both contain 25 values. Images are not
embedded: only shape, dtype, and SHA-256 are retained for auditability.

Use `--report-root` to change the artifact root and `--rollout-id` when a stable
external run identifier is required. Existing rollout directories are never
overwritten.

## Trainable DAgger episodes

The live Dora chain uses the same `fourier-record` process and
`xpolicylab_dagger_parquet_v1` container as GR3. Every saved episode contains
one Parquet file per asynchronous stream plus `metadata.json` and
`schema.json`. Common stream names cover canonical state, model action, expert
proposed/safe action, executed action, selected source, takeover/control events
and label validity. `astribot_s1_dagger_v1` supplies the robot-specific 25D
orders and the `cam_head`, `cam_left_wrist`, and `cam_right_wrist` RGB streams.

The button/save lifecycle also remains shared: start opens an episode, stop or
finish closes it, and a fresh left-foot press confirms the save. The live
launcher enables recording by default and writes to `Log/dagger/astribot_s1`;
set `ENABLE_DAGGER_RECORDING=0` for a no-record run.

Model inference is deliberately outside the SDK owner. `policy_node.py` runs
in the XPolicyLab environment, converts the three camera streams and 25D state
to an `ObservationFrame`, and uses the common `ActionChunkPolicyRuntime` to
emit one action step at a time. `dora_owner_node.py` runs in the vendor SDK
environment and owns lifecycle, DAgger arbitration, safety, and the sole robot
write path. It never constructs a `WsModelClient` or maintains a model chunk
queue. New robot integrations should keep this process boundary and only add
their profile, observation/action adapter, retargeting, safety, and SDK owner.

The former Python `frames.jsonl + data.npz + PNG` recorder and its direct CLI
replay commands have been removed. All trainable DAgger data now goes through
the shared Parquet recorder, so Astribot and GR3 have one storage lifecycle and
one validation path.

## Shared GR3/Astribot DAgger arbitration

The model/expert state machine is now robot-independent. Both robots use the
same `HOLD`, `MODEL`, and `EXPERT` modes, episode generation changes,
single-chunk step behavior, takeover reference capture, model pause, and
continuous-mode handback. Robot action assumptions remain profiles: GR3 keeps
its 37D joints/base profile, while Astribot uses its 25D full-body profile.

For Astribot, the QNexo adapter itself creates takeover-relative absolute arm
and torso targets, absolute gripper targets, held head targets and zero chassis
velocity. The shared Router selects model/expert mode and records evidence but
does not realign, clip or step-limit those targets. The unique Owner submits the
selected 25D target without a final PASS/HOLD action review. QNexo buttons request model control using
the GR3 mapping. The
footswitch is a model/expert takeover switch in DAgger mode, not
the model-motion deadman: pressing it pauses the model and selects expert
actions; releasing it resumes only continuous model mode. Step mode always
waits for another left-blue press.

`owner-shadow` now exercises this DAgger path with a measured dummy model chunk.
It remains unable to enable hardware writes. The independent policy node accepts
the common `AsyncPolicyInference` transport and validates every model result
against `astribot_s1.full_body_joint_position_chassis_velocity.v1`.


The vendor SDK identity is pinned under `third_party/robot_sdks/astribot_s1`;
the large ZIP is supplied by an artifact store or offline mirror rather than
ordinary Git. Cache and prepare it once after cloning:

```bash
gh auth login  # 已登录可跳过；也可显式设置 GH_TOKEN/GITHUB_TOKEN
scripts/setup_robot_sdks.sh cache astribot
scripts/setup_robot_sdks.sh prepare astribot --offline
```

The launcher then defaults `ASTRIBOT_SDK_ROOT` to the materialized repository
path. An explicit environment override is still possible, but it represents a
different runtime identity and requires new qualification evidence.

On a combined model-and-robot host, prepare the remaining hardware-free owner
runtime with:

```bash
scripts/setup_astribot_host.sh prepare
scripts/setup_astribot_host.sh doctor
```

The setup creates `.runtime/astribot-owner` with the system Python 3.10 ABI and
installs only the XPolicyLab/Dora owner dependencies there. It never constructs
the vendor SDK object or checks robot serial/input devices. ROS 2 Humble remains
a system prerequisite because the vendor SDK uses `rclpy`, Astribot messages and
Fast DDS internally; the PolicyServer and policy node do not use ROS.
Install ROS from the official Humble Ubuntu deb repository. The robot host needs
the `ros-humble-ros-base`, `ros-humble-rmw-fastrtps-cpp`,
`ros-humble-tf-transformations`, `libgsl27`, and `libmumps-seq-5.4` runtime
packages. `setup_astribot_host.sh` deliberately does not invoke `sudo` or mutate
the host's apt sources.

The repository provides a reviewed root-only installer for those system
packages:

```bash
sudo scripts/install_astribot_system_deps.sh
scripts/setup_astribot_host.sh doctor
```

The one formal graph supports two public launcher modes. Both use the same
policy node, owner, lifecycle, stop path and graph file:

```bash
# Model-connected runtime; trainable recording disabled by default.
scripts/launch_robot.sh astribot policy --no-write

# DAgger collection; trainable Parquet recording enabled by default.
scripts/launch_robot.sh astribot dagger --no-write
```

`policy` is a launcher mode, not a second graph. Either non-dry-run command
constructs the Astribot SDK session and therefore remains an operator-gated
robot connection even when `ENABLE_WRITE=0`.

## Gates

- `doctor` never constructs the SDK object.
- All robot connections require `--acknowledge-sdk-connect`, because the SDK
  constructor may acquire control rights and restart or stop the robot.
- Hardware commands are disabled unless both `--enable-write` and
  `--acknowledge-workspace-clear` are present.
- QNexo teleoperation uses the right footswitch as a click toggle: one click
  enters takeover and the next click leaves takeover.
- While takeover is active, each QNexo trigger controls its matching
  gripper at 100 Hz (`1=open` maps to `0`, `0=closed` maps to `100`).
- Teleoperation uses the reviewed arms, torso, head, and grippers stored under
  `prepare.initial_pose` in `profile.yml`. The values are written from an
  explicit live SDK state capture during calibration; Dora startup, white-button
  reset and episode transitions only read them and never overwrite or recapture
  them. Updating the default requires another explicit capture and profile
  change. The chassis remains at zero velocity.
  QNexo's calibrated data-side left/right names preserve the operator's physical
  hands. The collector mapping is physical right white `prepare`, physical
  right blue `start`, and physical right red `finish` (end the episode with an
  unconfirmed outcome and return to the configured Astribot initial pose without
  invoking a GR3 PdStand/RLStand transition). Physical left red is disabled.
  The finish command leaves the owner process alive; after immediate review,
  press physical right white and right blue to start another run. The right
  pedal toggles takeover; episode evidence is retained automatically and the left pedal has no save semantic.
- The footswitch remains the motion gate after `start` or `step`. The second
  click exits takeover and drops QNexo alignment without changing the prepared
  lifecycle state.
- Astribot pose reset is asynchronous, unlike GR3's blocking finish-home call.
  To preserve the same operator-visible ordering, one blue command received
  during reset is queued and executed only after readiness succeeds. A white
  command received during finish is likewise replayed before that blue command.
  The configured head target is still sent, but head feedback residual alone
  cannot block prepare readiness; arms, torso and grippers remain hard gates.
- A stale QNexo stream or SDK write failure calls the SDK stop path.

## Commands

```bash
# Environment and dependency checks only
scripts/astribot_s1.sh doctor

# Read buttons, footswitch, and triggers without constructing a robot SDK
scripts/astribot_s1.sh qnexo-observe --duration 10

# Read state and SDK-reported limits (SDK connection has lifecycle side effects)
scripts/astribot_s1.sh --acknowledge-sdk-connect observe

# Read three synchronized RGB cameras plus the 25D robot state
scripts/astribot_s1.sh \
  --acknowledge-sdk-connect \
  camera-observe --activate-cameras

# Exercise the unified observation/action owner without hardware writes
scripts/astribot_s1.sh \
  --acknowledge-sdk-connect \
  owner-shadow

# Safety-check the current arm joints without writing
scripts/astribot_s1.sh --acknowledge-sdk-connect hold-current-joints

# Supervised 25D full-body smoke: zero chassis velocity, move left joint 0 by
# at most 0.03 rad, then automatically return to the captured initial pose.
scripts/astribot_s1.sh \
  --acknowledge-sdk-connect \
  --acknowledge-workspace-clear \
  --enable-write \
  full-body-smoke --delta-rad 0.01 --phase-seconds 0.3

# Submit exactly the currently measured dual-arm joints once
scripts/astribot_s1.sh \
  --acknowledge-sdk-connect \
  --acknowledge-workspace-clear \
  --enable-write \
  hold-current-joints

# QNexo upper-body joint-delta shadow mode. Press physical left-white prepare,
# then physical left-blue start; right-foot clicks toggle takeover.
scripts/astribot_s1.sh --acknowledge-sdk-connect teleop

# Lab fallback when the footswitch is absent: hold unassigned left-white
scripts/astribot_s1.sh \
  --acknowledge-sdk-connect \
  --button-deadman \
  teleop --duration 10

# QNexo upper-body joint-delta hardware control
scripts/astribot_s1.sh \
  --acknowledge-sdk-connect \
  --acknowledge-workspace-clear \
  --enable-write \
  teleop
```

The joint path matches GR3 takeover alignment. The first pressed-foot sample
captures both the current robot arm joints and QNexo joints and sends nothing.
Later targets are:

```text
robot_reference + joint_scale * wrap(exo_now - exo_reference)
```

The current sign mappings are `[+,+,-,+,+,-,-]` for the left arm and
`[+,-,-,+,+,-,-]` for the right arm. Before SDK dispatch, the exact target is
checked against live SDK joint limits. An out-of-limit target rejects the whole
command and enters HOLD; valid targets are never rewritten from measured state.

The torso has three pitch joints and one yaw joint. Both joystick axes act as
slow three-state switches with a 0.25 dead zone: once deflected, stick magnitude
is ignored. Left joystick X directly commands the dedicated yaw joint, using
fixed `-0.10 rad/s` left and `+0.15 rad/s` right commands to compensate the
measured S1 positive-yaw tracking resistance. Right joystick Y directly
commands only the uppermost pitch joint (SDK torso joint 3 / action index 2).
The lower two
pitch joints remain at their takeover positions. Centering either joystick
latches the measured release pose once and then holds that fixed target. This
path does not continuously rebase through feedback and does not load a
kinematics model or an IK/QP solver.

The formal 25D owner submits arm, torso/head, chassis, and trigger targets
without a final action gate. Because the SDK applies `control_way` to an entire
call, teleop alternates two independent
chassis-free command groups: arms/grippers/head use `filter`, while torso alone
uses `direct`. This follows the torso-only part list in
`examples/206-move_joints_waypoints.py` and the 250 Hz direct online API in
`examples/105-set_joint_position_direct.py`. Both groups use `use_wbc=False`
and disable the implicit default torso; no chassis part name is present. A
blocking waypoint is never used in teleop. Trigger targets are updated at 100
Hz as absolute percentages and pass unchanged when they are inside the live SDK
limits. The SDK filter remains the single arm/gripper smoothing mechanism.
The former Cartesian experiment is never dispatched by `teleop`.
