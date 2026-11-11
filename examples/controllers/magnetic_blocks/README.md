# 积木逐函数演练：非阻塞工作副本

本版只对应 RAX `real/franka/autonomous-control` 中的逐函数演练，不组合或启动 `main(ctx)`，也不进行整段／规模测试。来自 v0.1 冻结策略及 R1 中断演练；冻结版不改。

## 原生循环

1. 首次读任务、经验索引、当前阶段与真实工具契约。取得新观测。
2. 为下一子目标编写／修正普通函数，明确输入、前置条件、结果与恢复边界。
3. 使用同一个 `ctx` 调用该函数及子函数；函数自行完成正常小分段和数值反馈检查。
4. 函数返回紧凑报告；主 Agent 查看结果和必要图像，修正后从实际状态继续。
5. 所有函数演练后另行进入合并阶段。本次不自动进入该阶段。

主 Agent 在函数调用之间工作。函数内的 `assess` 只能是正常同步感知代码（普通规则优先，必要时 Gemini），**不能轮询等待主 Agent 写批准文件**。没有所需证据时立即返回 `needs_review`；不会将 unknown 写成 true，也不要求照片证明隐藏毫米间隙。

## 函数与恢复边界

| 函数 | 完成内容 | 下一边界 |
|---|---|---|
| `locate_block` | 在指定当前帧的有效深度窗口取点 | 规划当前目标 |
| `open_empty_gripper` | 当前空载张开范围判断后，按实测开度分步张至69mm | 接近；到位用反馈，不再强制模型审核“已张开” |
| `empty_joint_waypoint` | 选定IK解后做完整数值路径检查和空载过渡 | 接近 |
| `approach` | 当前目标的有界接近 | 夹持对准 |
| `close_grasp` | 已确认包覆条件下闭合至50mm | 试抬前边界；不在此函数内抬升 |
| `trial_lift` | 当前姿态试抬10mm并检查保留关系 | 搬运；视觉未解决时不能重闭或重抬 |
| `transport` | 受控负载搬运 | 放置前边界 |
| `lower_to_support` | 下降到本轮计算位置，检查支撑条件 | 释放；函数本身不张爪 |
| `release` | 已确认支撑后松爪 | 退开；不重新下降 |
| `retreat` | 已检查路径退开，检查最终结构 | 任务结果 |

旧合并子函数 `grasp_and_trial_lift`、`place_release_retreat` 保留无动作的迁移提示，避免旧调用误重放。50mm/10mm属于本轮积木经验，不继承蓝杯3mm/40mm。205/265mm是历史法兰高度，不能用作新摆位默认目标。

## 普通 Python 调用与记录

```python
from api.runtime import Context
import sys
sys.path.insert(0, 'code/magnetic_blocks')
import controller
# 此例没有提供场景判断，因此只读取状态/图像并返回needs_review，不闭爪。
with Context() as ctx:
    report = controller.close_grasp(ctx)
# 在函数返回后检查 report 和其实际 frame_id。
```

目录内 `rehearse_step.py` 只是一次函数调用、计时和写报告的薄包装，没有后台服务／命令队列／批准循环。默认只打印计划，不连接硬件：

```bash
python code/magnetic_blocks/rehearse_step.py \
  --function open_empty_gripper --inputs output/open-inputs.json \
  --output output/rehearsal/open-001.json
```

未来获准演练且当前函数的实际同步检查已实现后，增加 `--execute --checks your_checks:assess`。最外层仅创建一个 `Context()`；一次运行只调用一个函数。未提供 checks 时，缺少场景证据立即返回，不能伪装成可无人执行的感知能力。

输出包括独立的调用报告、逐工具回执、`memory/REHEARSAL_STATE.json` 和 `REHEARSAL_CURRENT.md`。输入先按JSON校验并复制；回调不进入JSON。报告已存在时拒绝同ID再执行。

动作已完成但图像失败：`executed=true`、`action_completed=true` 和实测值仍保留，返回补观测请求。真实动作错误仍使用原生hold/Stop规则；不盲重试、不自动松爪。进程被外部中断时，先查原生状态／回执，独立Stop不由本脚本取代。

## 离线验证与限制

`python3 -m unittest discover -s code/magnetic_blocks -p test_workflow.py -v`

21项用例，含真实Piper工具schema参数约束及R1八个实际张爪回执的离线重放。重放保持同样8个命令和68.7mm反馈，函数内Agent等待为0，取图从每小步改成最终一次。它不证明新实机耗时或任何未演练的积木函数已成功。

本轮真实结果：只完成张爪动作，尚未接近或抓取。用户已停止；当前代码修改没有发起新的硬件或模型调用。

## R3 repair checkpoint
R3 added perception.py and rehearsed read-only locate_block for three colors; returns visible interior surface, not grasp center. Motion checks remain to be implemented/rehearsed by the main Agent. Host repair aligns native wall_budget and restores progress/compaction workflow; no frozen assets changed. Test recordings are under references/rehearsal-r3, never live observations.
