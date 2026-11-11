# 后续新任务：RAX 主 Agent 负责经验、函数与独立收尾

2026-11-11，按用户最后确认：**收尾单独运行，不占探索／演练预算。** 此版本撤回先前预留600秒/40次的方案，恢复原任务预算行为。

## 最终流程

1. 新任务会话启动时生成 `AGENT_OWNERSHIP.md`，明确主 Agent 持续记录经验并编写/修复函数；Mac监督、核验和归档。已有Mac候选可复用，保留来源及原始代码哈希。
2. RAX 原生探索/逐函数演练依授权范围执行。一个有用函数通过普通Python调用，返回后看结果/图像/日志；普通工具和规则优先，辅助视觉按需。正常小段不增加批准文件轮询。
3. 任务阶段结束时保存简短事实进度并结束该轮。宿主执行现有独立Stop/disarm，确认持稳及设备子进程全部退出。保留任务预算、原始结果和耗时，不以收尾成功倒改真机结果。
4. 独立软件收尾复用 RAX 原生 Codex 会话记录、同一任务工作区、现有模型relay与sandbox。只接主模型，不创建robot frontend，不挂载robot socket、stream或真实设备，也不接辅助视觉。无任务时间/调用预算继承，无固定收尾总时间或调用数上限。API错误或用户中断仍会结束；不无限重试。
5. RAX主Agent整理TIPS、PROGRESS、INDEX、策略/证据，生成或修复 `fn(ctx, inputs)`，进行相关离线检查，在代码项目内保存Git版本，填写收尾报告。未真机验证的函数标成候选或partial，不因为离线通过而升格。
6. Mac读取交付、核对提交与日志并归档。所有函数实测通过后再进入另行授权的main合并/新摆位整段测试，本次未进入该阶段。

完整收尾职责包括经验、代码、README、测试命令/退出码/日志、实际提交与代码身份、人工帮助、未验证项、当前状态与下一入口。原生跨会话技能库的检查/merge有其自身入口；纯软件收尾未连接该MCP，无法提交时明确待处理，不能声称已合并。

## 源码与运行产物

5090项目根：`/home/ubuntu/code_junqi/rax-piper-franka-20261104`

- `scripts/piper_explore.py`：仅新启动的主Agent会话采用新约定；硬件停止和归档后才进入软件收尾。
- `scripts/agent_owned_workflow.py`：新会话责任/阶段说明与候选来源记录，不编写任务控制器。
- `harness/codex_cli/closeout.py`：独立软件阶段；主模型调用无限制使用单独counter，原任务counter保持不变。
- `harness/codex_cli/launcher.py`：复用原生sandbox与会话恢复，软件模式不暴露robot连接。
- `harness/codex_cli/relay.py`：独立软件阶段接受无调用数上限；普通任务的有限上限仍按原配置执行。
- `rax-session/software-closeout/task-checkpoint/`：收尾前code/memory备份。
- `rax-session/software-closeout/phase.json`：独立阶段耗时、调用数、退出与报告状态。
- `rax-session/agent-workspace/output/closeout/SUMMARY.json`：主Agent交付报告。

软件阶段错误或缺报告记为未完成，不自动让Mac补写控制器。显式用户中断不自动启动收尾模型；停止/退出未确认也不启动。尚未提供自动重试或重新接机入口。

## 验证与当前限制

76项离线测试在隔离目录及部署后均通过，覆盖原任务预算兼容性、停止前禁止启动收尾、旧预算耗尽后独立计数、原生relay无上限分支、软件sandbox无robot socket、原会话/候选代码保护、真实入口与回归。独立阶段模拟超过原任务时长/调用数仍可完成，错误退出不重连硬件。真实新入口plan为 `starts_devices=false`。

本机演练skill与GitHub仓库的本地副本已更新，skill格式校验通过；未推送GitHub。

未向当前演练注入指令、重启服务或发送真机动作。保护核验覆盖724个文件；既有任务代码、配置、冻结版本无变更。三份演练进度文件在本次工作期间被其他流程更新，本次未写入或还原。源码部署有独立备份及哈希记录。

尚未实际启动一次收费主模型软件收尾，亦未做真机验证；后续新任务需验证Agent实际交付质量。这里的独立软件阶段是用户要求的本项目生命周期适配，不宣称Franka上游自带相同预算机制。

## 上游对齐

- [Franka AGENTS](https://github.com/robot-agent-exploration/rax/blob/real/franka/auto_research_agent/AGENTS.md)：主Agent探索、生成控制器、持续记忆。
- [Franka autonomous-control](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/autonomous-control/SKILL.md)：有用函数、明确输入、当前场景演练、检查修复，后续整段测试前Git提交。
- [Franka experience-memory](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/experience-memory/SKILL.md)：即时TIPS、策略/evidence提炼、索引。
- [团队演练skill](https://github.com/CoCoMatrix/rax-skills/blob/main/skills/rax-rehearse-functions/SKILL.md)：按需观测、普通规则、紧凑报告、可恢复边界。

Franka不规定离线测试通过即可完成真机资格，也不规定必须只读冻结目录；本项目分别采用离线工程验证、普通Git提交与额外归档保护。
