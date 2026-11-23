# magnetic-blocks-l-stack v0.1.0

本次RAX探索受监督成功，最终现场确认。成功策略、失败反例、参数、动作回执、关键帧与停止状态已经收尾归档。`code/magnetic_blocks/controller.py`是从成功策略新整理的可执行函数候选，尚待真机逐函数演练；不是已验证的无Agent整段控制器。

入口：memory/INDEX.md；参数经验：memory/TIPS.md；实测统计：evidence/run-summary.json；控制器使用方法：code/magnetic_blocks/README.md。

保留蓝杯原main及全部冻结版本。本任务可写副本为workspaces/magnetic-blocks-l-stack/v0.1-working；只读快照为versions/magnetic-blocks-l-stack/20261111-r3-v0.1.0。

依据：RAX real/franka的manual-robot-control → experience-memory → autonomous-control。探索完成后先把成功策略整理为函数并逐个演练，之后组合main(ctx)，再做新摆位测试。

- https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/experience-memory/SKILL.md
- https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/autonomous-control/SKILL.md
