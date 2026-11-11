# 已调通 demo：Piper 蓝杯放杯垫，v0.4 `main(ctx)`

这是原 5090 上 **v0.4.0-rc10** 的控制器源码快照，来自代码 commit `b4d3c1720ad635464fe099043ea4fd708870b902`。`code/blue_cup/` 的文件逐字保留，未把多路点/连续轨迹试验写入本 demo。原服务器工作副本与冻结版本也未修改。

入口：[`controller.py:main(ctx)`](code/blue_cup/controller.py)。它使用同一个 RAX `Context` 顺序调用普通 Python 函数，不启动主 Agent；Gemini 只作为按需辅助视觉。**下载源码不启动硬件；本目录是任务控制器，不是完整 Piper 驱动发行包。**

## 1. 已验证到什么程度

2026-11-11 重新读取原服务器 Web 记录与 `REHEARSAL-DONE.json`。以下四轮的 13 个顶层 Python 源文件 SHA256 均与本 demo 相同：

| 2026-11-10 轮次 | 结果 | main 秒数 | 含启动/收尾秒数 | 主 Agent | Gemini |
|---|---|---:|---:|---:|---:|
| 163900-bd92c4 | 成功 | 57.826 | 73.682 | 0 | 0 |
| 164047-5d1fb1 | hover IK 失败，已 Stop | 35.983 | 51.883 | 0 | 0 |
| 164742-2a1b36 | 成功 | 56.975 | 73.668 | 0 | 0 |
| 164932-b53382 | 成功 | 57.820 | 73.802 | 0 | 0 |

三次成功均有任务成功报告、原生停止确认，记录中未标人工辅助。M 轮的 58.661 秒属于更早的 rc9，不能算作本次 rc10 完全相同源码的测试。

[`validation-runs.json`](validation-runs.json) 包含逐阶段耗时、模型次数、成功和失败记录、报告哈希及代码哈希。它是原记录的选定字段摘要，没有相机原图、token、完整审计或持久化摆位坐标。发布时没有重新运动，也没有把离线测试算作新的真机成功。

这说明它已在若干摆位跑通，**不说明任何杯型、任意摆位或整个 500 mm 工作区均可达**。同版本已知失败是固定朝向下 hover IK 超限；位置在工作区内仍可能无完整位姿解。

## 2. main 实际怎样组织

```text
prepare_for_approach：本轮初始 RGBD 自动定位＋张爪＋接近准备
  → approach(hover)
  → approach(entry)
  → grasp：目标开度 3 mm
  → trial_lift：40 mm，普通规则比较夹持结果，必要时 Gemini
  → localize_coaster：从本轮初始三维目标计算搬运目标，不再次语义定位
  → transfer_step：有界搬运
  → lower_to_support → scheduled.lower_to_height：按本轮高度关系下降
  → release：张爪
  → retreat：退开 80 mm，定性确认最终放置
```

这里的合并是 **Python 函数组成 main**，底层仍逐动作下发，不是一条跨阶段连续轨迹。每一步检查前置条件和执行结果；第一个失败保存报告并退出，不自动重试运动、松爪或回零。

- 新摆位重新观察和定位，不手动填写杯子坐标，也不重放历史关节轨迹。
- 同一次接近保留带来源帧、owner、几何和动作绑定的三维目标；预期遮挡不要求每段重测杯沿。
- 普通数值移动用反馈、FK、IK、路径和工作区检查，不固定取三张图或请求 Gemini。
- 头部深度用于初始度量定位；左腕 RGB 补充橙垫/杯壁关系。腕部 FFS 未做手眼标定，不直接产生基座运动目标。
- 预测放置高度为 `entry_flange_z + initial_coaster_z - initial_table_z`；不把该数值描述为测得了支撑力。释放后退开再定性看结果。
- `code/blue_cup/README.md`、部分旧函数与注释保留了当时历史状态，可能写“待验证”；本目录的版本哈希与实测摘要是发布状态依据。不要为了调用 main 而转去执行旧 `rehearsal.py` 的整套路径。

| 文件 | 在当前 main 中的作用 |
|---|---|
| [controller.py](code/blue_cup/controller.py) | 编排 `main(ctx)`、失败退出、汇总耗时和调用数 |
| [scheduled.py](code/blue_cup/scheduled.py) | 接近、抓取、同轮搬运、预测高度放下、释放和退开 |
| [execution.py](code/blue_cup/execution.py) | 单动作状态绑定、IK、路径预检、下发及回执 |
| [placement.py](code/blue_cup/placement.py) | `trial_lift(..., execution_mode='task_scheduled')` |
| [perception.py](code/blue_cup/perception.py)、[checks.py](code/blue_cup/checks.py) | 初始定位、普通定性规则、必要时 Gemini |
| [approach_target.py](code/blue_cup/approach_target.py)、[scene_geometry.py](code/blue_cup/scene_geometry.py) | 同轮目标身份、几何与轮廓 |
| [run_no_agent.py](run_no_agent.py) | 在已启动的 frontend-only 会话中执行一次；默认只打印计划 |

## 3. 运行环境与参数

依赖已经适配好的 RAX/Piper runtime：XPolicy 唯一硬件 owner、双 Piper X、头部/左右腕相机、头部 FFS 与外参、当前 TCP/指尖/桌面模型、MoveIt 2 KDL、FCL、RAX MCP frontend、Linux bubblewrap、隔离 Python。控制器依赖 NumPy、SciPy、OpenCV；`api.runtime.Context` 由 RAX 会话提供，不能用同名 PyPI 包替代。

本机配置：SDK 50%；有效峰值约 17°/s、加速度 40°/s²；3 mm 抓取；40 mm 入杯与试抬；关节状态容差 0.01 rad；持稳 0.3 秒/至少 4 个不同反馈样本；单次位姿位移 ≤400 mm。左法兰工作区 `[55,-400,170]` 到 `[500,45,360]` mm，坐标系 `left_base_link`，受控点 `left_link6`。TCP 140 mm、指尖轴向 170 mm。完整参数见 [`DEMO.json`](DEMO.json)。

这些参数同时存在于控制器与宿主；仅复制本目录不会安装后端或修改 SDK 速度。其他设备需提供同等接口及对应标定，不能直接沿用这些数值。

主模型虽然不启动，当前宿主仍会加载既有模型配置；Gemini 兜底依赖宿主密钥与代理。仓库不包含密钥，复现不得把密钥写入源码。当前辅助视觉接口说明见 [Gemini 文档](../../skills/rax-rehearse-functions/references/gemini-interface.md)。

## 4. 先做离线完整性检查

在 Mac 或服务器的仓库根目录运行，Python 3.9+，不导入控制器、不访问硬件：

```bash
python3 demos/piper-blue-cup-v0.4/verify_demo.py
```

在原 5090 上还能核对已部署代码是否相同：

```bash
python3 demos/piper-blue-cup-v0.4/verify_demo.py \
  --installed-code /home/ubuntu/code_junqi/rax-piper-franka-20261104/workspaces/blue-cup/20261109-v0.4-merged-candidate/code/blue_cup
```

验证哈希、语法和历史报告对应关系；不把它当作当前场景的运动验证。

## 5. 当前 5090：通过已有 Web 复现

1. 使用仓库现有 [一键 Web 入口](../../web-console/Open-RAX-Console.command)。它连接只读相机/反馈，不自动执行任务。
2. 核对页面为 **v0.4.0-rc10**、代码 commit 前缀 **b4d3c17**，且上一步源码核对通过。不要选多路点试验版。
3. 点击 **回到 Prepare**，由 XPolicy 原生回零闭爪；到位后重新摆杯子和杯垫、夹爪空载、手离开工作区，并现场监督。
4. 勾选现场确认，再点击 **执行一次 main(ctx)**。它重新初始定位，无手动摆位坐标、不启动主 Agent。
5. 从页面查看阶段、main/整轮耗时、Gemini 次数和结果。失败记录也保留；Stop 不等于 Prepare，异常后先核对实际状态，不盲重跑。

现有 Web 固定指向原 v0.4 候选。本次发布没有修改其路径或安装新 Web，也没有启动硬件。

## 6. 命令行复现 main：独立副本＋原生 frontend-only 会话

在**已有同版本原部署的 5090**上，将本仓库克隆到任意目录。安装器只从现有宿主复制本地 memory/标定/证据到一个新的工作副本，再放入本 demo 源码；不会把这些本地材料上传，也不会改原候选或冻结版本。没有原部署的机器，应先完成 Piper runtime 适配，不能靠此安装器从零接机。

```bash
RAX_ROOT=/home/ubuntu/code_junqi/rax-piper-franka-20261104
DEMO="$PWD/demos/piper-blue-cup-v0.4"
PY=/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python

# 默认只检查并显示目标目录；--apply 才创建新副本，目标已存在则拒绝覆盖。
python3 "$DEMO/install_demo.py" --root "$RAX_ROOT"
python3 "$DEMO/install_demo.py" --root "$RAX_ROOT" --apply
CANDIDATE="$RAX_ROOT/workspaces/blue-cup/v0.4-rc10-demo"
```

新副本重新建立自己的 Git commit；`demo_original_code_commit` 保存原始身份，源文件字节不变。默认不修改 Web 指向。安装后使用原生 XPolicy 回 Prepare、重新摆位；退出其他硬件会话，让唯一 owner 可用。下面 `--run` 会启动受监督的硬件会话，不能与 Web 的另一轮 Run 同时执行。

终端 A，在同一个 shell 里保留 `RAX_ROOT/PY/CANDIDATE` 变量：

```bash
RUN="$RAX_ROOT/commissioning/demo-v04-$(date +%Y%m%d-%H%M%S)"
printf '%s\n' "$RUN"  # 给终端 B 使用，必须是本次新目录
cd "$RAX_ROOT"
PYTHONPATH="$RAX_ROOT" "$PY" scripts/piper_explore.py \
  --output "$RUN" --rehearsal-candidate "$CANDIDATE" \
  --direct-model --http-proxy http://127.0.0.1:7892 --seconds 1800 --run
```

**不加 `--rehearsal-agent`。** 等待本次 `REHEARSAL-READY.json` 出现；其中 `main_planning_model_started` 必须为 `false`。启动器名称虽为 `piper_explore.py`，此参数组合实际只启动 frontend 和工具服务。

终端 B，把 `RUN` 设置为终端 A 打印的实际新目录，不能复用历史 READY：

```bash
PY=/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python
CANDIDATE=/home/ubuntu/code_junqi/rax-piper-franka-20261104/workspaces/blue-cup/v0.4-rc10-demo
# RUN=/home/ubuntu/code_junqi/rax-piper-franka-20261104/commissioning/demo-v04-实际时间
"$PY" "$CANDIDATE/run_no_agent.py" --ready "$RUN/REHEARSAL-READY.json"
# 看清上面计划后，执行一次：
"$PY" "$CANDIDATE/run_no_agent.py" --ready "$RUN/REHEARSAL-READY.json" --run
```

runner 在隔离的 RAX 工作区执行 `code/blue_cup/controller.py --run`，实际入口是：

```python
# 仅在已创建、已授权的 RAX sandbox 内；不是普通宿主 shell 的独立连接方式。
from api.runtime import Context
from controller import main  # controller 所在目录需在 sys.path 中；CLI 已处理
with Context() as ctx:
    result = main(ctx)
```

正常结束，runner 写入 `REHEARSAL-DONE.json`，原生 supervisor 负责 Stop/disarm。报告位于：

- `$RUN/REHEARSAL-DONE.json`：主结果、阶段耗时、模型次数、源文件哈希。
- `$RUN/STOP-RESULT.json`：停止与静止反馈。
- `$RUN/rax-session/agent-workspace/output/rehearsal/merged-*/report.json`：进行中的阶段报告。
- `$RUN/rax-session/agent-workspace/output/merged-console.log`：控制器输出。

这条手动 CLI 路径保存上述原生报告，**不会自动新增 Web 实验表的一行**；要用 Web 表连续记录，就通过第 5 节按钮执行。

需要中断当前 CLI 会话时，从原 runtime 调用独立 Stop：

```bash
cd /home/ubuntu/code_junqi/rax-piper-franka-20261104
PYTHONPATH="$PWD" /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python \
  scripts/piper_explore.py --stop "$RUN"
```

不要以 `kill` Python 代替停止。失败后不重放 main 来“续跑”；先检查持杯状态、当前反馈和停止结果，再决定恢复。重新测试须新会话、新观测和新的空载初始条件。

## 7. 发布时的离线验证

`verify_demo.py` 校验原样源码和全部四轮代码身份；`tests/` 覆盖 main 正常顺序、首个失败退出、闭爪后失败不释放、安装器不覆盖原版本。测试使用假函数/临时目录，不连接机器人或模型：

```bash
python3 -m pytest demos/piper-blue-cup-v0.4/tests -q
```

测试环境需安装 pytest、NumPy、SciPy、OpenCV。这些离线用例验证打包和编排，不替代真实深度、碰撞或机械臂验收。新多路点执行方法属于单独试验，不在此已调通 demo 的实测范围内。
