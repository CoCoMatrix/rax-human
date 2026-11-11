# R3 演练重复读取修复（2026-11-11）

核对来源：
- https://github.com/robot-agent-exploration/rax/blob/real/franka/auto_research_agent/AGENTS.md
- https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/autonomous-control/SKILL.md
- https://github.com/robot-agent-exploration/rax/blob/real/franka/harness/claude_cli/launcher.py
- https://github.com/CoCoMatrix/rax-skills/blob/main/skills/rax-rehearse-functions/SKILL.md

上游是按步骤读取必要技能，持续写PROGRESS，压缩/恢复后读进度和实时status；逐函数检查修复后继续下一函数。上游Franka运行Claude CLI，不能声称其有相同的Codex参数。

R3实测：总940.320秒；43条模型请求日志累计924.796秒（含压缩等，非独立机械耗时）；本地配置65536自动压缩阈值，运行日志报告258400上下文窗口，共5次压缩。定位红/橙/绿分别0.227/0.210/0.200秒，修订红色0.209秒，Gemini0，任务运动0。原生Stop/撤销授权/持稳通过，左0.3mm、右2mm，双臂关节零位。

修复：
1. 仅积木演练配置移除65536覆盖，使用客户端原生默认压缩，未谎报上下文容量。
2. 原生PROGRESS作为工程进度，自动REHEARSAL_STATE作为独立动作回执；开发/测试阶段也写进度，不等首个函数执行。
3. 明确单个定位成功是检查点，不是整个演练完成。预算允许且无具体物理阻碍时继续下一函数。
4. 保留服务器已有的独立软件收尾更新；将格式/库打包/提交工作推迟至收尾，不在每个物理函数前重读。
5. 修复R3控制器对旧600秒/32次和已移除primary_model_budget字段的硬依赖，直接消费当前wall_budget；动作限位、感知和回执检查不放宽。
6. 保存R3 Agent编写的定位函数和真实证据到可写副本，迁移离线测试证据路径。标注可见表面定位不等于抓取中心；未演练函数不晋升。

验证：部署代码33项测试；启动/closeout/配置回归56项；入口测试1项。原生记忆导入44个证据文件通过。离线计划不启设备；配置生成不再含auto_compact覆盖。积木冻结73文件、蓝杯保护555文件哈希一致。备份：backups/magnetic-blocks/20261111-r3-context-fix。

限制：这证明配置、契约、记录和离线分支修正；实际模型是否减少重复读取，需R4观察。接近/抓取等未因此成为已通过真机演练的能力。
