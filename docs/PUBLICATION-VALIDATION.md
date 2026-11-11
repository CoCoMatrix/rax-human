# 发布验证（2026-11-11）

验证对象：本仓库的独立发布副本；5090 上位于 `verification/rax-human-publication-20261111`，不是正在运行的 RAX 目录。

- `PYTHONPATH=. python -m pytest -q`：**197 passed，6.01 s**。使用现有 5090 Python 环境，未启动硬件或模型。覆盖独立软件收尾、预算／交接、API 转发、观测／几何约定、工作区导入、Web 页面／记录／停止等。
- 新蓝杯工作区实际导入成功，生成本地 Git 身份、归档哈希和可写代码；源 Demo 未修改。
- 探索默认计划：`starts_devices=false`，软件收尾为 `separate_software_only`，`closeout_task_budget=null`。
- 无 Agent 主函数默认计划：`starts_devices=false`，`main_planning_model_started=false`。
- 原 5090 路径样例经 doctor 文件存在性检查通过；relay URL 故意保留占位符，未读取密钥／调用 API。
- v0.4 `DEMO.json` 中全部源文件 SHA-256 匹配。
- 发布目录检查了常见密钥／私钥格式、凭据赋值、大文件和符号链接；一个测试用 key 改为明确的合成 fixture。唯一符号链接是 owner 的 `XPolicyLab -> .` 命名空间别名。
- 无超过 40 MB 的单文件；不上传模型权重、虚拟环境、运行图像／录像、运行目录或私有配置。

发布适配仅调整部署路径、工作副本导入、文档和测试入口。一个历史回归用例仍期望 1 s 持稳，本次改为与**已有** 0.3 s／4 样本设置一致，没有改变控制参数。Web 使用 site 配置中的辅助视觉代理和隔离 Python 路径；v0.4 控制器源码保持原样。

这些结果不是新机器的真机验收，也不是新的任务成功记录。连续动作、多摆位泛化、第三方 API 可用性和真实模型软件收尾，均未由本次离线发布验证证明。历史说明中的耗时、可达性和成功案例仅对应其当时设备与摆位。
