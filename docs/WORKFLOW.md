# Agent 所有权、阶段与收尾

原生依据是 RAX real/franka 的 AGENTS.md、experience-memory 和 autonomous-control。这里的“只读冻结目录＋哈希”是本工程的版本保护，不宣称是上游统一冻结机制。

主 Agent 在探索中持续记录策略和失败；成功后整理 TIPS/PROGRESS、memory/skills 与索引。逐函数演练允许它改函数、修代码、重新测试。函数应具有明确输入、可检查前置条件及结果报告，不依赖主机临时传递未记录的坐标。正常动作内部不固定插入主模型／辅助视觉调用。

探索的物理预算只限制有设备的阶段。停止设备、撤销写权限并确认子进程退出后，入口可启动独立的软件收尾：复制原生会话继续由 RAX 主 Agent 更新记忆、候选代码、离线验证和提交。它不继承原时间／调用上限，单独计时计数。沙箱不提供 robot socket、frontend、设备流或 Gemini；不能继续探索动作。显式用户中断、停止不确定时不自动继续；模型错误要准确记录，不能无限重试。

关键实现：`scripts/agent_owned_workflow.py`、`harness/codex_cli/closeout.py`、`harness/codex_cli/launcher.py`、`scripts/piper_explore.py`。产物包括 `rax-session/software-closeout/phase.json`、任务 checkpoint 与工作区 `output/closeout/SUMMARY.json`。本次发布仅离线验证这些机制，未声称真实模型收尾已经成功。

代码写出、编译通过、离线测试通过、函数真机通过、整段新摆位通过是不同资格。人工确认、手动调整和失败必须保留，不得升级为自主技能证据。跨会话 library merge 使用已有检查机制，不能只复制任务 memory 就称完成技能晋升。

组合 main(ctx) 是后续阶段：将临时判断整理成函数输入、普通规则、明确结果；完成真实函数演练后提交版本，再从新摆位整段测试。Mac 可监督和归档，不能替主 Agent 完成工作后冒充它自主完成。
