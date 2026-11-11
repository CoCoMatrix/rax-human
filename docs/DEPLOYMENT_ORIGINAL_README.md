# RAX Franka 流程的 Piper / Codex 适配

底座：[`real/franka@8858d7f3`](https://github.com/robot-agent-exploration/rax/tree/8858d7f3e34eff382abae738cf0578d2726a9a0a)。保留原 MCP frontend、`api.runtime.Context`、任务技能、经验格式及工作区生成；只替换设备与模型接入。没有旧工作台状态机、自动冻结框架、自定义压缩或阅读去重 hook。

5090 新目录：`/home/ubuntu/code_junqi/rax-piper-franka-20261104`。旧工程、成功候选和原始证据未修改。

## 工作方式

**直接工具探索 → 单函数演练 → `main(ctx)` 闭环组合 → 固定版本、多摆位整轮测试。**

探索不要求预先写完控制器。演练把成功操作写成 `fn(ctx, ...)`；组合时传递同一个 `ctx`，从当前画面重新定位、执行并检查结果。组合的是闭环决策，不是拼接旧坐标。整轮测试中主代理补动作或人工代替关键判断，必须记录为协助。详细边界见 [WORKFLOW.md](docs/WORKFLOW.md)，旧流程教训见 [HISTORY_LESSONS.md](docs/HISTORY_LESSONS.md)。

## 当前实际能力

- 两台 Piper X、三台 D435 身份与 CAD 参数已导入；运动接口为左臂，右臂保持静止。右臂只有只读 FK，没有宣称完成双臂运动适配。
- MoveIt 2：复用已安装的持久 KDL **全姿态 IK**，用独立 FK 验证；不是完整 MoveGroup 场景规划/执行。Piper pose 动作按关节插值，不承诺 Franka 的直线 TCP servo。
- 执行：连接既有 XPolicy 唯一硬件 owner 的 bridge；本项目不再创建 CAN/SDK writer。与 Franka 依赖外部 Polymetis bridges 一样，底层 owner 必须独立就绪。
- 感知：头部同包 RGB + Fast-FoundationStereo 深度，默认7×7有效深度中值和内外参反投影；左右腕为 RGB 辅助视图。没有 SAM 或整圈拟合前置。原深度来源与时间戳保留。
- 外参：按用户选择沿用 all13 候选，LOO 最大约18.08mm，仍标 `provisional`；没有已验证的腕部外参或抓取 TCP。
- 模型：主代理 `gpt-6-astra / high` 继续使用现有 API；辅助视觉改为 Google 官方 `gemini-3.8-flash`，通过 RAX 原生 `gemini_generate` 调用，工作副本默认 low。主／辅助密钥分别由 host 从 `/home/ubuntu/code_junqi/key.py`、`/home/ubuntu/code_junqi/gemini-key.py` 读取，不进入代理工作区。Codex 使用原生压缩。配置及实测见 [Gemini 接入记录](docs/GEMINI_OFFICIAL_CONFIGURATION_20261105.md)。
- 代理隔离：Codex 位于 bwrap 内，只见工作区、专用环境和模型/MCP sockets。当前 shell 不联网；Python依赖由 host 准备。这是与 Claude 原 sandbox 的能力差异，已同步写入代理指令。
- 已验收：任务独立代理 `127.0.0.1:17890`、指定主/辅助模型的真实图像请求、独立 Frontend 进程下的原生 Codex + MCP mock、约5分钟真机只读观测，以及原生 Codex → MCP → 真实 Piper 的NO_SEND整链路。五分钟观测取得589条状态、硬件写计数全为0；原生整链路32.29秒，完成status、三视图observe和辅助视觉各一次。
- 受监督最小关节往返已通过：左J1目标+0.5°、实测+0.486°，返回0°；其余13维测量值保持。独立owner Stop有确认，两个arming文件已撤销，并以8条新反馈验证静止。该轮包含获准的双臂Enable、当前实测双臂目标Prepare及结束持位；没有夹爪命令。这是一次接口测试，尚未抓杯。

## 已准备的会话

`runtime/sessions/20261104T024221Z_blue-cup_a1b397/operator-input.json`

这是最初生成的 Piper 技能、三相机契约、任务和 Codex 配置，状态为 **prepared_no_hardware / NO_SEND**。后续只读观测、模型验收和最小关节测试使用独立会话，见验证记录；它们不等于已通过的杯子控制器。

## 准备新会话（无硬件动作）

在5090新目录执行。现有 Python 环境仅作为依赖只读复用，不修改旧环境：

```bash
export RAX_PYTHON=/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python
export PYTHONDONTWRITEBYTECODE=1
export HTTPS_PROXY=http://127.0.0.1:17890
export HTTP_PROXY=http://127.0.0.1:17890
export NO_PROXY=127.0.0.1,localhost

"$RAX_PYTHON" scripts/configure_session.py \
  --task blue-cup --task-file examples/tasks/blue-cup.md \
  --robot piper --backend-options examples/piper.json \
  --agent-provider codex-relay --agent-model gpt-6-astra --agent-effort high

# 把上一条输出的路径填入 --config。
bash scripts/start_codex.sh --config '<operator-input.json>' \
  --key-file /home/ubuntu/code_junqi/key.py \
  --gemini-key-file /home/ubuntu/code_junqi/gemini-key.py \
  --base-url https://sub2api.cocomatrix.cn/v1 \
  --http-proxy http://127.0.0.1:17890 \
  --agent-runtime-dir /home/ubuntu/code_junqi/rax-piper-agent-isolated \
  --prepare-only
```

正式启动使用同一命令去掉 `--prepare-only`；已启动过的会话再加 `--resume`，保留其Codex对话。启动器不自动 arm/reset，也不自动启动相机或机器人。

真机运动前必须将对应operator配置绑定到新项目独立的 `bridge_root`、`owner_session_root` 和当前 `depth_root`，设置实测工作空间，并完成受监督验收。`examples/piper.json` 默认 `motion_enabled=false`。只读观测服务按会话限时启动和退出；历史临时目录或旧 armed 文件不代表当前就绪。新增代码不把“不知道有没有执行”自动重发；已保存的图可继续分析，但暂停后仍需重新观察现场。独立撤销入口见 [OPERATOR.md](docs/OPERATOR.md)。

## 验证与剩余项

见 [VALIDATION.md](docs/VALIDATION.md) 和 [CODEX_ADAPTER.md](docs/CODEX_ADAPTER.md)。此前 TLS EOF 已通过任务独立代理解决，未修改全局代理配置或更换模型、密钥、中转站。RGB-D模式下头部图像与深度来自同一不可变数据包；腕图仅有 HTTP 元数据前后包围，三视角并非硬同步。原生整链路这次使用RGB-only，三个视角均不宣称深度绑定。单次关节往返不证明全姿态IK运动、抓杯、演练、组合控制器、多摆位成功率或长期原生压缩可靠性。
