# Franka 原生演练与当前函数入口

## 主 Agent、函数和辅助视觉的分工

上游 [real/franka autonomous-control](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/autonomous-control/SKILL.md) 的路线是：成功直接控制 → 保存策略和关键帧 → 一个有用动作函数化 → 在当前场景演练和修复 → 组合 `main(ctx)` → 新摆位整段测试。主 Agent 编写、调用和调试函数；函数运行时使用普通 MCP 工具，也可按需调用 Gemini。

| 阶段 | 主 Agent | Gemini | 产物 |
|---|---|---|---|
| 探索 | 决定尝试、修改代码、解释反馈 | 简单感知不足时辅助 | 策略、经验、关键帧与失败记录 |
| 逐函数演练 | 选择当前函数、传入本轮输入、检查与修复 | 函数按需调用，不主持演练 | 经实测的函数、前置条件和恢复入口 |
| 整段执行 | 控制器内部不再逐步调用主 Agent | 可保留必要的兜底 | `main(ctx)` 的一次运行报告 |
| 多摆位测试 | 在轮次之间分析与修改 | 次数作为实验指标 | 真实成功率、耗时、覆盖范围 |

上游要求按当前观测检查函数条件，并优先用简单感知规则；没有规定每个动作前后必须调用 Gemini。最终图像/日志的检查不等于固定发两次辅助视觉请求。用同一个 `ctx` 向下传递；只在最外层 Python 入口创建 `Context()`，不得在每个子函数另建会话。

本项目把同一次接近目标保留、静止区间内多视角判断、阶段取图计划做成普通 Python 方法。这些是遵循该原则的 Piper 实现，不声称 Franka 上游已经包含同样的蓝杯像素规则或参数。

## v0.4 应演练哪一组函数

先看实际 `code/blue_cup/controller.py` 的 imports。2026-11-10 rc10 的对应关系如下；不要仅因名字一样就使用旧 `rehearsal.py` / `placement.py` 中的版本。

| 顺序 | 实际实现 | 关键输入/输出与检查 |
|---|---|---|
| 准备与本轮定位 | `scheduled.prepare_for_approach(ctx)` | 新场景自动定位，保存准备报告及本轮目标；不是回零 |
| hover | `scheduled.approach(ctx, {'phase': 'hover'})` | 本轮三维目标，完整有界接近及数值路径检查 |
| entry | `scheduled.approach(ctx, {'phase': 'entry'})` | 保留接近目标，杯沿下 40 mm 的本机策略；保存 entry 报告 |
| 闭合 | `scheduled.grasp(ctx)` | 当前 3 mm 目标，读取实际开度/回执；不把目标开度当夹持力 |
| 试抬 | `placement.trial_lift(ctx, {'dz': .040, 'execution_mode': 'task_scheduled'})` | 40 mm，前后定性比较；该 execution_mode 决定采用最新取图策略 |
| 杯垫目标 | `scheduled.localize_coaster(ctx, inputs)` | 传 `preparation_report`、`entry_report`、`lift_report`；读取本轮初始度量目标，不固定重新问模型 |
| 搬运 | `scheduled.transfer_step(ctx, inputs)` | 传 `target_report`、`previous_report`；更新上一报告，直到 `transport_complete`，仍有段数上限 |
| 放下 | `controller.lower_to_support(ctx, inputs)` | 当前目标源为 `initial_same_run_metric_coaster` 时进入 `scheduled.lower_to_height` |
| 释放 | `scheduled.release(ctx, inputs)` | 当前 main 传 12 mm、目标与放下报告；检查真实开度与动作结果 |
| 退开 | `scheduled.retreat(ctx, inputs)` | 当前 main 竖直退 80 mm，最后一次取图定性检查放置 |

表中报告键是当前函数输入契约，使用实际返回路径，不能手工拼接另一轮的文件。调用前读取相关签名/报告即可；完整示例以实际 `main(ctx)` 的参数串接为准，不从本文生成一个新的并行控制器。

**发现的旧入口问题：** 保存的演练提示仍出现 v0.3、3.5 mm 及旧 `rehearsal.py` 调用；部分 README/WORKSPACE 状态也滞后。启动演练时需要把会话简报与上述实际 imports 对齐。此次 skill 发布只记录并解释差异，没有改写运行时提示、启动新会话或重新验收全部函数。

## 一轮演练的最小资料与循环

启动只需版本身份、当前实测状态、任务简报、工具契约和下一函数的条件。先看紧凑报告，只有失败定位到感知/IK/交接时才展开对应源码；不要提前读完放置实现、整个感知模块和所有历史记忆。主 Agent 等待同一长调用时使用合适的有界等待，避免短轮询生成大量模型往返。

每次循环：本轮输入 → 单个有用子目标 → 结果报告 → 有问题才修复 → 从实测状态恢复。正常接近的小分段留在函数内部；闭合、试抬和释放仍是清晰边界。减少 Agent 往返不等于演练时完全去掉 Agent。

函数报告至少说明：`ok`、为什么、测量值、报告路径、动作是否已下发/完成、当前可能持物状态、耗时、Gemini 次数。感知失败与动作失败分开；日志保存完整结果，主 Agent 接收紧凑摘要。

## 失败与交接

- 只有图像不清而数值工具已证明的条件，不再另请模型重复证明。真正影响下一动作的缺失事实，先用其他视角/普通规则；仍不足再通过原生 `request_operator` 请求具体事实。
- 人工答复绑定对应请求和当前边界；保存答复来源。不能把人工事实写成图像模型已经确认，也不能在待答复时开一个新会话重试同一动作。
- 动作后图像失败：检查回执、当前状态和必要新观测，判断已经完成哪段。不要从 `main(ctx)` 重头恢复，尤其可能仍持杯时。
- Python 退出或 TLS 错误不表示机器人没动。沿用原生停止/持位协议；不自动松爪、回零、重播动作。用户明确要求回 Prepare 时使用 XPolicy 原生入口。

## 收尾与后续

记录成功、失败、人工帮助、仍待验证函数及版本。先备份演练副本，再由相同函数组合 `main(ctx)`；新摆位重新定位，完整运行期间不由 Agent 手动补动作。人工补完的轮次记为调试，不计自主成功。多摆位连续通过才扩大泛化声明。

Codex 的此 skill 与 RAX 跨会话可执行技能库不同；后者仍需按 [skills-lib](https://github.com/robot-agent-exploration/skills-lib) 的格式、checker、分支和 library merge 接入，失败/人工辅助经验不能直接晋升为已验证自主技能。

## 上游来源身份

核对日期 2026-11-10：`autonomous-control/SKILL.md` blob `4f86a90e7b9b9fe989df849380e1715f6b226a97`，本地内容 SHA-256 `c5bcdda2c2a4d87b318a1a193ac242c7c4a52e67dc5fd11704ada19d07384457`。设备适配参考 [REAL_ROBOT_ADAPTATION.md](https://github.com/robot-agent-exploration/rax/blob/main/docs/REAL_ROBOT_ADAPTATION.md)，blob `b11edfc6cdf07faf5b4bf9633a2ec3702605cf63`。分支链接会变化，迁移时重新核对当前工具契约。
