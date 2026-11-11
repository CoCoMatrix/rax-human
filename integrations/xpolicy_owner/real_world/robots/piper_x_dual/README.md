# Dual Piper X owner

The formal Dora graph defaults to no-send. Its unique owner receives joint,
controller and gripper feedback from both configured CAN channels. It
also validates all three D435 color streams by librealsense serial. The formal
Dora graph uses the robot-independent `real_world.cameras.realsense_dora_node`
to own and continuously publish all three streams; the Piper owner waits for
that node before declaring `READY_NO_WRITE`. Hardware motion is available only
when the formal supervised launcher explicitly sets `ENABLE_WRITE=1`.

For an explicitly authorized software-stop recovery, the formal launcher accepts
`--release-software-stop right_arm --enable-write --duration 10` (or `left_arm`).
This maintenance run checks fresh controller and disabled-joint feedback, sends
one pinned SDK resume packet (`0x150`, `02 00 00 00 00 00 00 00`) to that arm,
checks a newer NORMAL controller status, and exits the owner before motor enable
or operator/model event processing. No joint or gripper targets are sent.
It is not evidence that enabling or model execution works, nor proof of a prior
emergency stop when the initial status was already NORMAL. Use external model
server mode to avoid starting a model service for this operation.

For explicitly authorized zero calibration of an empty, physically fully closed
gripper, set `PIPER_CALIBRATE_CLOSED_GRIPPER=left_arm` (or `right_arm`) only for a
bounded formal launcher maintenance run with `--enable-write --model-server external`.
The existing owner disables that gripper, checks fresh stable feedback, invokes
the vendor `calibrate_gripper()` API, requires its acknowledgement and fresh
zero feedback, then exits before arm enable or operator commands. It sends no
motion targets. Remove this variable before restoring the normal paused Web.

`HOLD_FEEDBACK_UNAVAILABLE` with a gripper range error blocks Prepare before
motion. The error includes the measured width in meters and millimeters as
well as the allowed feedback range. This is measured sensor feedback, not a
model target; do not clamp it or widen the range to dismiss the fault. Inspect
the value and the device's calibration/feedback before retrying live motion.
While feedback is invalid, Prepare/Start explicitly emit a rejected
`PIPER_X_DUAL_CONTROL` event with the measured feedback error. Rejected clicks
do not change the lifecycle or queue motion for later recovery. The UI labels
HTTP command delivery as submitted, not accepted by the robot. Stop/Finish/
Discard still reach the cancellation boundary without issuing a position hold
from invalid feedback. If live startup skipped SDK enable because feedback was
invalid, correct the feedback and restart through the formal launcher.

After startup, one `wait_for_frames` timeout no longer kills the shared camera
node. Only the affected pipeline enters bounded restart while the other streams
continue draining. The node publishes `DEGRADED` during recovery and `ERROR`
after retry exhaustion, so the owner leaves `READY_STREAMING` and holds before
any further write. The operator UI expires a cached JPEG after 2 seconds and
labels it `Stale`; an old image is never presented as live evidence.

## Web 双夹爪张开

普通部署的 Web 控制区提供 **张开左右夹爪**。机器人须处于可写就绪、
暂停、无 Prepare/Finish 边界运动、无 takeover、无监督任务或录制状态。
一次点击仅由正式 owner 向左右夹爪下发各自 profile 的张开上限（当前均为
0.1 m），保持机械臂目标不变；不启动模型。HTTP 成功只表示指令已提交，
页面另外显示 owner 的下发或拒绝结果，不把下发成功当作已到位。

该指令经过既有 UI token、control mux、双臂反馈与 PASS/HOLD 校验。
旧指令、缺失/过期反馈、禁用写入或运动中的请求均拒绝，不自动重试。
张开时清除已 Prepare 状态；随后点击 **Prepare**，按已有流程将双臂移回
初始姿态并将双夹爪闭合至 0 m，完成后才允许 **Start**。
保持当前位置的专用 Prepare 部署和 DAgger 采集模式不支持此手动按钮。

## Web 硬件重连与就地诊断

录像区域的 **打开录像文件夹** 在机器人电脑的桌面文件管理器中打开
当前录制或最近一次录制的目录。运行时重启后按当前模型查找最近保存的
录像；尚无录像时打开录像根目录。Pigey 使用对应的示范录像目录。
按钮不开始/停止录像，也不发送机器人动作；路径由服务端选择，失败原因
直接显示在按钮下方。远程访问 Web 时，打开的是机器人电脑上的文件夹。

**重新连接硬件和相机**复用正式部署切换事务：校验当前部署，停止原
launcher 并等待退出，然后由同一个正式 graph 重新打开 CAN 和三路相机。
保留当前模型、执行步数和读写模式；只读模式重连后仍为只读，不自动
Prepare、Start 或张开夹爪。运行、接管、录制或其他操作未结束时拒绝重连。
若重新启动后设备仍有故障，保留暂停的 Web 展示诊断，不自动反复重启。

左右臂及左右夹爪各自展示反馈新鲜度、驱动故障和使能状态；夹爪显示
实际开度，张开指令发出后超过 5 秒仍未到位会在对应夹爪处显示目标和
实测开度。各相机画面下显示该相机的错误或画面过期信息。诊断只读取
已有 owner 的 SDK 缓存，不建立额外设备连接，不替代动作安全检查。
Web 或 owner 状态过期时显示未知/中断，不继续显示缓存的绿色状态。

## 个人 BAT23 部署包切换

The 8097 operator UI shows **个人部署包** only when the active deployment is
listed in [`policy/Pi_05/junqi_personal_web_deployments.json`](../../../policy/Pi_05/junqi_personal_web_deployments.json).
That catalogue contains the six personal BAT23 packages for instance01,
including the new-prompt EMA2500 package. Additional packages enrolled through
**＋ 接入部署包** / `xpolicy web` are loaded from
`.xpolicy/web-deployments/registered/` and appear automatically. The intake
checks compatibility and requires a current complete qualification receipt;
it is not a general deployment browser. See the personal Web intake section in
[`docs/piper-new-model-bundle-setup.md`](../../../docs/piper-new-model-bundle-setup.md).

Each button is enabled only while the policy is paused, the lifecycle is
`idle`, `prepared`, or `stopped`, and neither operator recording nor a human
demo is active. Selecting another package first validates its release and
bundle, then restarts the one formal `launch_robot.sh` parent with the same
write mode, control source, UI port, model port, filtering choice, and UI-open
choice. It does not send `Prepare` or `Start`.

The switch worker inherits the original launcher's initial environment, so the
old graph's checkpoint, task catalogue, prompt and Dora session cannot leak into
the new deployment. Relative and absolute personal launcher commands are both
supported. The Web entry point installs the switcher at startup; an existing
process must be restarted once to load this feature. While switching, other
mutations are rejected except Hold. The browser automatically reconnects and
reloads when the server session changes, without replaying commands.

The button remains busy until the new UI reports the selected deployment ID,
the expected ready status, a paused policy, and all cameras available. A failed
preflight leaves the current runtime untouched. A failed replacement is stopped
before the switcher restarts and verifies the previous package. Its progress and
failure reason are written under
`/home/ubuntu/code_junqi/migration/piper-deployment-web-switch/`.

2026-10-12 verification: 128 focused tests passed. A real browser clicked all
six buttons on port 8097; each switch took 18–21 seconds, reloaded the browser
session, and reached paused policy / idle owner with all three cameras live.
Each model's `TRAINING_PREFIX_MODEL_READY` manifest hash matched the selected
bundle and reported prefix warmup complete. The original mixed EMA2500 package
was restored. No Prepare/Start was sent and every restarted owner reported zero
hardware writes; this does not qualify task motion or grasp success. Evidence,
screenshots and the browser script are in
`migration/piper-deployment-web-switch/ui-integration-20261012/` under the
personal workspace. Prior release supplements are preserved in its `before/`
directory; only the UI, switcher and catalogue source bindings were updated.

A separate supervised qualification entry point owns both SDK connections but
permits commands to only one selected arm per run. It uses the SDK-smoothed
`move_j`, enables SDK joint limits, fixes the installation position to
`horizontal`, and never commands the other arm or either gripper:

```bash
uv run --extra piper-x-dual xpolicylab-piper-x-dual-write-canary \
  --arm left_arm --mode step
uv run --extra piper-x-dual xpolicylab-piper-x-dual-write-canary \
  --arm left_arm --mode prepare
```

`step` performs the profile-bounded 0.03 rad test and returns to the measured
start. `prepare` moves the selected arm at the profile's 15% speed to the fixed Piper X SDK
Home in `profile.yml`. Passing this tool does not qualify model or continuous
teleoperation dispatch; the formal graph still reports zero hardware writes.

The pinned `pyAgxArm` source is bundled at
`third_party/robot_sdks/pyagxarm`. Prepare/verify robot SDK identities, install
the Piper extra from that local source, and run the probe from the repository
root:

```bash
scripts/setup_robot_sdks.sh verify pyagxarm
uv run --extra piper-x-dual xpolicylab-piper-x-dual-probe
```

To select a different physical robot instance:

```bash
uv run --extra piper-x-dual xpolicylab-piper-x-dual-probe \
  --device-instance runtime/robot_dagger/config/robot_instances/<robot-id>.yml
```

A successful result is `READY_NO_WRITE`. This does not authorize actuator
commands or qualify the candidate action schema for deployment.

The repository-owned Mink FK/IK model is used by the formal teleoperation path
and retained for offline URDF regression:
[`description/dual_piper_x_mink.urdf`](description/dual_piper_x_mink.urdf).
It preserves the provided 0.60 m dual-arm kinematics but intentionally contains
no visual or collision geometry. See `description/README.md` for provenance and
the link-6-versus-TCP limitation.

The teleoperation path uses the common `qnexo-fk` node and fixed takeover
anchors. The Piper retarget node converts QNexo endpoint deltas into absolute
link-6/flange poses and solves warm-started host-side Mink IK. The unique owner
then applies an all-or-nothing freshness, finite-value and joint-limit gate and
streams the accepted 14D absolute joint/gripper target through the official V189
joint controller. The validated Mink target is also the joint-space DAgger label;
firmware Cartesian IK is not used by this path.

The qualified profile uses unit translation and rotation gains. Both cumulative
takeover-relative pose caps are disabled (`max_translation_m: null` and
`max_rotation_rad: null`), so the retargeter does not create an artificial
spherical translation or rotation boundary. Joint validity and stale-input gates
remain fail-closed and never clip or reshape an accepted action.

All model policies, including BridgeWAM, use the same owner-side action path.
There are no model-label-based J2/J3 `abs`/`-abs` overrides or substitutions of
out-of-range model targets with measured-state commands. The shared model
boundary normalization (joint overshoot up to 0.03 rad and physical gripper
clamping) and configurable low-pass filter remain in place. Targets that still
violate the safety gate produce `PIPER_X_DUAL_ACTION_HOLD` with `invalid_targets`
and are not dispatched; bounds, freshness and non-finite checks are unchanged.

## Selecting the DAgger expert source

The authoritative launcher now accepts `--dagger-expert-source qnexo|leader_follower`
(default `qnexo`). `qnexo` keeps the existing exoskeleton FK/Mink path. The
`leader_follower` option reads the pinned pyAgxArm leader-angle feedback inside
the existing Piper owner and maps it into the same canonical 14D absolute action;
the configured follower arm is the only arm replaced, while the other arm is
held at its measured state. No second SDK/CAN process is created.

Example (the QNexo footswitch still supplies the lifecycle/takeover event):

```bash
bash scripts/launch_robot.sh piper_x_dual \
  --deployment desktop-cleanup-h32-v3-ema10k \
  --dagger-mode mixed \
  --dagger-expert-source leader_follower \
  --control-source both \
  --no-write
```

The physical pair can be changed without code using `PIPER_LF_LEADER_ROLE`,
`PIPER_LF_FOLLOWER_ROLE`, and `PIPER_LF_FEEDBACK_ROLE` (`left_arm` or
`right_arm`). The current owner intentionally does not auto-switch native
firmware modes: `native_linkage: true` is rejected until the paired CAN wiring,
model/expert handoff, and stop behavior are qualified. This prevents a native
follower controller from competing with the single owner writer. The SDK mode
methods are exposed in `sdk.py` for that later qualification; the default
feedback-bridge path is hardware-neutral and can be shadow-tested.

The operator UI exposes the same settings under **DAgger expert control**:
choose `QNexo / retarget` or `Leader → follower SDK bridge`, select the leader,
follower and joint-feedback roles, and set the maximum accepted feedback age.
Press **Apply** before Start (the controls lock while an episode is running).
The selection is sent to `piper_x_dual_owner` as `expert_config`; the owner
validates it and keeps QNexo lifecycle buttons/footswitch semantics unchanged.
