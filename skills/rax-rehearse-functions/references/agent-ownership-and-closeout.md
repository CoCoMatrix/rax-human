# RAX 主 Agent 负责生成与收尾

核对日期：2026-11-11。适用于后续新会话；不得将流程更新热写入正在运行的工作区，或借修复流程重启正在演练的机器人。

## 谁完成什么

- RAX 会话主 Agent 持续写 TIPS/PROGRESS，按 experience-memory 提炼 `memory/skills/<slug>/SKILL.md` 与 `evidence.json`，更新 INDEX；从成功策略编写、修改和验证 `fn(ctx, explicit_inputs)`，保存 README、结果与 Git 版本。
- Mac Codex 监督模型/函数耗时、处理现场交接、诊断基础设施、核验和归档。正常收尾不由 Mac 静默接管编写任务控制器。用户明确要求宿主修改时可以执行，但需标明来源。
- 已有 Mac 候选可以复用：记录原始路径/哈希/作者来源，由 RAX Agent 核对实际工具契约并决定沿用或修复；不能标成会话独立生成或已经真机验收。

## 阶段范围

探索首次成功后停止继续任务运动，主 Agent 仍完成经验提炼、候选函数生成、相关离线验证和版本保存。旧入口“成功后保存策略，不进入演练”限制的是后续真机阶段，不应让代码收尾转交 Mac。若用户仅要求记录或明确禁止编写代码，以本次用户范围为准。

逐函数演练仍遵循：成功策略 → 一个有用函数 → 当前场景调用 → 返回后检查图像/日志 → 修复 → 下一函数。正常小段在函数内完成，无批准文件轮询，不逐小步等待 Agent。函数已执行但图像不足时保存实测回执，恢复不重放动作。全部相关函数通过后才交付后续合并阶段；不在演练中擅自整段测试。

## 预算与版本

经验和代码随过程记录，但**完整收尾是独立的软件阶段，不占探索／演练预算**。真机阶段沿用原有任务预算和安全停止窗口；它仅需留下简短事实进度。任务成功、阶段结束或任务预算用尽后，宿主先执行原生 Stop/disarm，确认持稳、设备子进程退出，再启动 RAX 主 Agent 的纯软件收尾。

软件收尾复用同一任务工作区和复制的原生 Codex 会话记录；与原任务分别记录耗时和模型调用。不继承任务剩余时间或调用上限，也不配置固定的收尾时长／调用数上限。没有 robot MCP/socket、owner、相机服务或辅助视觉入口；不能因为收尾而重新运动。主模型通过既有中转完成经验、代码与文件工作。任务已耗尽预算不会阻止收尾；上游 API 错误、用户中断仍可终止，发生阻塞保存 partial，不无限重试。

明确的用户中断不自动启动新模型。停止或设备退出未确认也不自动收尾。新任务才应用该入口；不向当前演练注入指令或修改其预算。独立软件阶段是本项目按用户要求做的生命周期隔离，不声称 Franka 上游已经实现同样的独立计费／预算机制。

由 RAX Agent 在项目代码目录用普通 Git 保存候选版本，不改写历史、不推送仓库、不提交凭据或无关文件。保存提交哈希、代码哈希、参数范围、来源、离线命令/退出码/日志、逐函数实测记录、人工帮助和未验证项。身份配置缺失或提交失败如实记录为未完成，不伪造 commit。

状态区分：`partial`、`candidate_offline_checked`、`function_rehearsed`。语法/接口/mock/回放检查不能证明物理成功；Franka 的关键验证是函数真机演练和后续不同摆位整段测试。只读目录与哈希冻结是我们的额外归档保护；不以它替代真机资格。

## 后续新会话的实施位置

5090 `rax-piper-franka-20261104/scripts/piper_explore.py` 在新建主 Agent 会话时安装 `scripts/agent_owned_workflow.py` 的工作约定，产生 `AGENT_OWNERSHIP.md` 和来源记录。它不写入既有会话。无主 Agent 的 main/Web 前端入口保持原路径。

物理阶段结束前的代码和记忆保存在 `rax-session/software-closeout/task-checkpoint/`，不是新冻结版本。独立阶段实现在 `harness/codex_cli/closeout.py`，复用原生 sandbox、会话恢复和 relay，不创建第二套机器人控制器。阶段状态与耗时/调用数保存为 `rax-session/software-closeout/phase.json`。

Agent 输出 `output/closeout/SUMMARY.json`；逐函数阶段同时更新原有 `output/rehearsal/SUMMARY.json`、memory/RUNS/PROGRESS。宿主核验文件、测试日志、代码与提交是否对应，然后归档；报告缺失时记“收尾未完成”，不代填成功控制器。此改动仅完成离线验证，后续主 Agent 是否遵循并完成真实收尾仍需实测监督。

## 上游来源

- [Franka AGENTS](https://github.com/robot-agent-exploration/rax/blob/real/franka/auto_research_agent/AGENTS.md)：主 Agent 探索、生成自动控制器、持续记忆和报告。
- [Franka experience-memory](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/experience-memory/SKILL.md)：成功后即时记忆、提炼策略与证据。
- [Franka autonomous-control](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/autonomous-control/SKILL.md)：逐函数运行与修复、明确输入、后续整段测试前 Git 提交。
- [团队演练 skill](https://github.com/CoCoMatrix/rax-skills/blob/main/skills/rax-rehearse-functions/SKILL.md)：按需观测、紧凑报告、恢复与证据口径。
