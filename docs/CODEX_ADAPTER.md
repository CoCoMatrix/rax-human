# Codex 适配与验收边界

保留上游 RAX 的 workspace generator、Frontend 串行调度、Session 日志、Context 客户端和 memory/经验/自动压缩提示。新增 `harness/codex_cli` 替换 Claude CLI 启动与凭据转发，新增 `codex_vision_generate` 替换辅助 Gemini 服务。未引入新阶段状态机、冻结器、压缩器或评测框架。

- 主代理：沿用先前配置 `gpt-6-astra` / `high`，由 operator config 可修改。
- 辅助视觉：固定 `gpt-6.1-sol` / `medium`，Responses API；输入调用者自己的 prompt 与最多 8 张 inline PNG/JPEG；没有机器人控制工具。
- `key.py` 支持纯 token 文本或简单字符串赋值；只由 host 读取，不执行文件、不放入 agent 环境、配置或日志。
- API endpoint：启动时指定 `--base-url https://sub2api.cocomatrix.cn/v1`，也可来自 key 文件中的 `base_url` 字符串。
- 原生 Codex 自动压缩和会话 resume；没有沿用旧项目的 100000 阈值、自定义压缩 hook 或重复阅读拦截。
- 代理整个进程运行在 bubblewrap 内：新 network namespace、仅 workspace/Codex home 可写；只有 MCP 与模型 relay 两个 host Unix socket。无 host loopback、GPU、机器人 SDK owner 路径或凭据。依赖由 host 预备，当前 agent shell 不开放普通互联网。
- `--agent-runtime-dir` 可只读使用现有独立 agent Python（含 numpy/scipy）；不复制大环境，不修改旧项目。
- Frontend 在独立 host 子进程的主线程串行处理工具；它自行关闭 backend、solver 和 Session。launcher 先结束 agent，再请求 Frontend 退出并记录结果；强杀或清理不完整时记录 `control_state=unknown`。正常进程退出也不证明物理停止，记录始终保留 `physical_stop_confirmed=false`。
- 辅助模型调用记录真实 usage、duration 与调用数；中转站价格未经核实，不套用 Gemini 的美元价格或宣称已执行 $10 成本限制。原 upstream Session 中 Gemini budget 字段仍存在但此次未使用；辅助调用数见 `robot_status.auxiliary_vision`。

## 入口

```bash
RAX_PYTHON=/path/to/host/python bash scripts/start_codex.sh \
  --config /path/to/operator-input.json \
  --key-file /home/ubuntu/code_junqi/key.py \
  --base-url https://sub2api.cocomatrix.cn/v1 \
  --http-proxy http://127.0.0.1:17890 \
  --agent-runtime-dir /home/ubuntu/code_junqi/rax-piper-agent-isolated \
  --prepare-only
```

Prepare 只生成 workspace/config，不启动 backend、不读相机、不运动、不调用模型。首次运行同命令去掉 `--prepare-only`；有历史启动后加 `--resume` 继续同一个 Codex home 中的最近会话。Launcher 从不 arm/reset 机器人，backend 的 operator arming 仍有效。主模型或辅助视觉的网络错误不会被当成机器人成功。

## 2026-11-04 实测

在 5090 上使用原有 host 解释器只读运行：

- 独立 Frontend 进程与关闭相关回归 **14 passed**，见 `verification/codex-process-tests.txt`；全量代码检查另见 `VALIDATION.md`。
- 真实 bubblewrap 内 `/agent-runtime/bin/python` 成功 import numpy/scipy，并通过原 `api.runtime.Context`→Unix MCP→原 Frontend 调用 mock `status`。返回 `robot=mock`，`credential_visible=false`，`host_repo_visible=false`。
- Codex CLI **0.154.0** 在同一 sandbox 内成功创建会话、向 host relay 发出 Responses 请求。
- 最初主/辅助模型遇到 TLS SSL EOF；只读查明全局代理选中节点出站失败。均匀测试现有 12 个节点，5 个可用。在新项目的私有目录启动独立 `127.0.0.1:17890` 任务代理，使用一个已配置可用节点、绑定 `enp6s0`、关闭该任务代理的 TUN；不修改原全局代理、DNS 或规则。原配置 SHA 前后一致。
- 中转站对 `stream=false` 仍返回 SSE。辅助服务现兼容 JSON 与 SSE，只接受 `response.completed`，拒绝部分结果/失败/断流。回归测试先复现原失败，再验证修复。
- **真实归档相机图像 API 通过**：主 `gpt-6-astra/high`（实际返回 model 相同），辅助 `gpt-6.1-sol/medium`（实际返回 model 相同）。两个模型都收到旧成功轮的真实头部相机图像，返回蓝杯及支撑物描述和实际 usage；这是归档图像接口验收，不是当前真机闭环成功。
- **本轮真实三视图 API 通过**：两个指定模型均收到 `frame_id=952a205dd63444abbbebb492e52fcac9` 的头部、左腕、右腕图像；主模型18.519秒、辅助16.781秒。图像 SHA 与来源元数据见 `verification/current-model-check/{primary,vision}.json`，原图见 `verification/current-cameras/`。这些耗时仅是本次请求，不是探索速度保证；没有调用运动工具。
- **独立 Frontend 进程下的原生 Codex + bubblewrap + MCP mock 通过**：exit 0；一次 `status`、一次 `codex_vision_generate`，0 MCP 错误、0运动调用。Frontend 正常清理并退出，没有强杀，`physical_stop_confirmed=false`。证据 `verification/model-smoke/native-codex-process.json`。
- **原生 Codex + MCP + 真实 Piper NO_SEND通过**：32.29秒，status → observe(head/left/right RGB) → 一次辅助vision，0 MCP错误、0运动调用；主 `gpt-6-astra/high` 实际通过view_image查看三图，辅助实际返回 `gpt-6.1-sol/medium`。源 `frame_id=3ea9c8424bb14a05861960620da07ad7`；这次是RGB-only，三图均为HTTP metadata bracket、`metric_depth=false`。头图中的杯子和杯垫分离，腕视图受遮挡或未覆盖目标，不把模型看图当成抓取成功。
- r3观测graph的350条状态全部禁写、最高写计数0；归档验证完成，设备与agent进程均退出，Frontend未强杀且正常清理。启动/退出时的反馈stale及连接拒绝记录保留。证据与实际转发三图在 `verification/live-native-r3/`。这仍是只读整链路验证，不证明运动或任务成功。
- 后续独立受监督脚本已通过新Piper backend完成左J1半度往返与owner Stop验收，见 `VALIDATION.md` 和 `verification/piper-minimal-live-r1/SUMMARY.json`。该物理接口测试不属于上述模型只读调用，也不证明Codex自主抓取、右臂主动运动或全姿态IK执行。
- 真实自动压缩/长会话恢复尚未专门压测；短会话和一次图像理解不能证明长期可靠性。
- 归档图像成功证据在 `verification/model-smoke/{primary-astra-high,vision-sol-medium}.json`；网络变化证据在 `verification/network/task-proxy.json`。此前失败的 mock 会话仍保留在 `runtime/sessions/codex-smoke-20261104/` 与 `codex-smoke-proxy-20261104/`。

## 任务代理的启动与恢复

私有配置 `runtime/private/task-proxy/config.yaml` 保存现有节点配置，权限 0600；目录 0700。该文件包含代理节点凭据，不能展示、提交或打进交付包。`key.py` 仍在原路径，仅由模型 host 读取。

任务代理已作为独立进程运行；它没有修改全局 Mihomo。启动模型时使用上述 `--http-proxy`，代理 URL 仅保存于 operator-owned `codex-relay.json`。agent 自身仍无网络，不持有 API key。

重启服务器后，先确认 17890 未被现有任务代理占用，再从新仓库启动：

```bash
cd /home/ubuntu/code_junqi/rax-piper-franka-20261104
nohup /opt/clash-party/resources/sidecar/mihomo \
  -d "$PWD/runtime/private/task-proxy" \
  -f "$PWD/runtime/private/task-proxy/config.yaml" \
  -ext-ctl-unix "$PWD/runtime/private/task-proxy/controller.sock" \
  > "$PWD/runtime/private/task-proxy/mihomo.log" 2>&1 < /dev/null &
```

该进程没有安装为系统自动启动服务。恢复前可用不带凭据的 `curl --proxy http://127.0.0.1:17890 https://sub2api.cocomatrix.cn/v1/models` 检查 TLS/路由：401 表示已到认证端点，不代表模型通过。真实模型证据必须来自 `/responses` 完整返回。回滚只需停止命令行明确包含这个私有 config 路径的任务 Mihomo；不要停止全局代理或盲信旧 PID 文件。
