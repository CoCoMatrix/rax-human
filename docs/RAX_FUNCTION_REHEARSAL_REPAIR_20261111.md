# RAX 逐函数演练修复 — 2026-11-11

范围仅为逐函数演练。未合并main，未启动新真机动作，未修改冻结版本。

## 核对的原生步骤

RAX Franka [autonomous-control](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/autonomous-control/SKILL.md) 要求：读取成功策略和关键帧 → 提取一个接收ctx及明确输入的函数 → 当前场景运行 → 返回后检查图像和日志 → 修复并继续逐函数验证。遇到不适用场景就返回报告，不能临时猜动作；全部函数完成后才组合main。

[Franka AGENTS](https://github.com/robot-agent-exploration/rax/blob/real/franka/auto_research_agent/AGENTS.md) 要求按当前步骤读取资料，持续更新PROGRESS，压缩后恢复当前状态；并未规定反复读取完整源码或在函数中等待Agent批准文件。

团队 [rax-rehearse-functions](https://github.com/CoCoMatrix/rax-skills/blob/main/skills/rax-rehearse-functions/SKILL.md)、[失败与验收](https://github.com/CoCoMatrix/rax-skills/blob/main/skills/rax-rehearse-functions/references/failures-and-validation.md)、[视觉与深度](https://github.com/CoCoMatrix/rax-skills/blob/main/skills/rax-rehearse-functions/references/vision-and-depth.md) 进一步明确：普通规则优先、函数内完成正常小段、动作回执与视觉失败分别记录、恢复不重放。本次没有采用合并阶段skill。

## R1 实际问题与修复

| 问题 | 证据 | 修复 |
|---|---|---|
| 演练误变成函数内等待Agent的审批循环 | pending/review文件轮询，90s超时；张爪函数141.08s中检查等待112.49s | 删除常驻runner和批准文件流程；普通函数立即返回结果／needs_review；Agent在调用之间看报告 |
| 压缩后反复读资料、重复启动 | PROGRESS仍写“尚未运动、接下来启动runner”，实际已经张爪 | 每次调用自动落盘动作意图、最新结果、下一边界；简短启动页和签名索引；压缩只读最新记录和当前状态 |
| 把可调用函数写进JSON | args引用被原地加入assess，保存报告TypeError | JSON输入事先校验复制，运行期callback单独传入；输入错误在工具调用前拒绝 |
| 离线mock未识别真实工具参数错误 | 使用quaternion，真实Piper要求quaternion_xyzw | 修正参数，并从实际后端无初始化导出schema验证；不以Franka示例参数名代替Piper契约 |
| 张爪完成却被结果审核超时判失败 | 8次真实到位，最后68.7mm，函数仍ok=false | 普通到位用原生反馈；动作完成但缺图保留executed；未知语义立即返回，不加hold或重复动作 |
| 恢复入口过粗 | 闭合/试抬和下降/释放/退开绑在同一调用 | 拆成独立函数；返回明确下一入口，旧组合入口无动作提示迁移 |
| 每小步重复取图和审核 | 8次开爪均observe_after=true；一度加入每小步Agent复核 | 小段保留反馈和检查，只在函数结果需要时取图；无固定Gemini |
| 记忆文件无法原生导入 | evidence.json的frames写成目录 | 可写副本改成明确图片路径，并补可导入证据ledger；原冻结版保持 |

## 实测与离线口径

R1总时长909.492s；主模型48次请求，47次完成，最后一次随用户停止而未完成；请求时间累计837.920s。辅助视觉0次，机械臂关节/位姿移动0次。8次开爪RPC合计28.051s，包含预检、交接、SDK、持稳和图像，不等于纯电机运动时间。模型请求和函数等待有重叠，不能相加。

张爪动作实际完成，尚未抓取/搬运/堆叠。不是成功函数演练或自主任务成功。Stop回执、撤销授权及8个持稳样本通过；最后双臂关节为0，左爪68.7mm、右爪2mm，没有额外回零或闭爪。

离线：21项控制/恢复/真实接口及实际回执重放检查、2项简短启动页检查。新执行包装不启动后台或等待Agent，保持同一组真实回执对应的8个开爪命令。无需硬件即可核对分支；实际提速和其余函数成功率仍需下一轮获准真机演练。

机器人相关阈值、50mm抓取/10mm试抬经验、原生工具限位和停止协议未放宽。右臂、蓝杯原main、已有冻结版本未修改。

## 部署后最终验收

5090 上可写工作副本的21项测试、启动简报2项测试、启动器33项回归全部通过。实际代码的简报生成与离线启动计划通过，starts_devices=false。积木冻结版73个文件及蓝杯受保护555个文件哈希一致。本轮演练进程已退出，未重启硬件。详见同目录 REPAIR-ACCEPTANCE.json。
