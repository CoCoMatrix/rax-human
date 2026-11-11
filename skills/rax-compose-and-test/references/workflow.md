# 上游路线与本次实现

## 已核对来源（2026-11-10）

- [Franka autonomous-control](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/autonomous-control/SKILL.md)，blob `4f86a90e7b9b9fe989df849380e1715f6b226a97`。
- [REAL_ROBOT_ADAPTATION](https://github.com/robot-agent-exploration/rax/blob/main/docs/REAL_ROBOT_ADAPTATION.md)，blob `b11edfc6cdf07faf5b4bf9633a2ec3702605cf63`。
- [Franka 分支](https://github.com/robot-agent-exploration/rax/tree/real/franka)。分支会变化，后续改接口先核对当时源码。

上游没有一个保证任意任务成功的“通用合并脚本”。它规定工作方法：成功手动控制 → 函数化并在当前场景演练 → 顺序组合 controller.py:main(ctx) → 提交代码 → 中立姿态和操作者新摆位 → 无 Agent 介入地跑一次 → 记录失败及修复 → 换摆位重复。没有固定 N 轮或正式 benchmark 的要求，要求多个不同摆位连续干净成功，不能宣称未测试的泛化。

## 合并如何做

保留演练版本，把可写副本里的修复验证后复制成候选。不要把一次探索的全串绝对关节角当控制器：需要每轮取当前物体位置，再用共享几何关系产生目标。

```python
def main(ctx):
    report = {"ok": False, "steps": []}
    # 实际 fn 和 inputs 按已经演练的函数契约传递。
    for name, fn, inputs in stages:
        result = fn(ctx, inputs)
        report["steps"].append({"stage": name, **result})
        if not result["ok"]:
            return report
    report["ok"] = True
    return report
```

示意代码不负责硬件启动、授权和故障停止；这些由宿主原生后端负责。每阶段写入 active_step / elapsed / 回执，异常也保存。恢复到一个函数前重新核对实际状态；不要从可能持杯的状态重跑 main。

## 感知和速度

上游建议普通规则与 MCP 动作工具，简单规则不足时使用 Gemini。它并未承诺完全不需要模型，亦未规定每个动作前后强制辅助视觉。本项目初始定位、试抬前后对比、最终放置检查是任务调度选择，不是上游硬性规则。

同一次接近保留 base-frame 三维杯沿/杯垫目标，后续动作由 FK、路径检查和反馈执行。保留目标不等于把旧像素拿去索引新深度。腕部图像用于遮挡补充；需要尺度时按需 FFS。新摆位必须清空上轮缓存。

函数串联不是跨动作连续轨迹。先分别量化预检、owner 轮询、前动作 ack、交接、SDK 运动、持稳、观测、模型耗时，再减少重复等待。碰撞缓存只能复用完全相同状态/几何/环境下的结果。不能把未验证环境误写成“自碰撞检查通过，所以路径安全”。

## 本地文件定位

Piper 当前合并代码：`workspaces/blue-cup/20261109-v0.4-merged-candidate/code/blue_cup/controller.py`。
主要模块：`scheduled.py` 动作子目标，`checks.py` 普通视觉判定，`perception.py` 初始定位，`placement.py` 试抬/下降，`transport.py` 搬运及目标身份。
宿主入口：`scripts/piper_explore.py` + 工作副本 `run_no_agent.py`；传 `--rehearsal-candidate` 且不传 `--rehearsal-agent`。旧输出名 REHEARSAL-* 不代表运行了主 Agent，应看 `main_planning_model_started=false` 和 `execution_mode=compiled_controller_no_agent`。
