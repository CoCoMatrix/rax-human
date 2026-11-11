# main 之后的规模测试

## 单轮闭环

1. 连接硬件和三路预览，核对当前状态及版本。
2. 点击 XPolicy Prepare，等实际零位与生命周期确认。该按钮闭爪并回零；它不是只持位。
3. 操作者重新摆杯子和杯垫，夹爪空载，确认现场条件；不填写摆位。初始 RGBD 定位自动记录杯口中心估计、杯垫位置、坐标系和来源帧。
4. 点击“执行一次 main(ctx)”。启动器建一个全新会话，加载工作副本、初始定位、串行执行，不启动主 Agent，不在中间插入人工工具调用。
5. 看 active_step、阶段耗时、模型调用次数、三路画面和日志。Stop 走已有 Stop/disarm，异常不自动松爪/回零/重跑。
6. 结果写入独立记录。成功后已停止但仍在退开位置；回 Prepare、换摆位、再运行。不要把同一场景无人复位地循环 N 次当规模测试。

## 测试组织

按杯子左右/前后位置、杯垫相对方向、可达范围内距离和常见遮挡变化分组。保留失败轮次。记录总体成功率和按场景组成功率、耗时分布、各阶段耗时、Gemini/FFS 次数、失败阶段、人工帮助次数。成功率的分母应明确包含失败及中断，不隐去坏轮次；现场中断与算法失败分开解释。数量由任务覆盖目标决定，上游没规定固定轮数。

本 Web 提供每轮记录/CSV 和成功计数，是人工摆位的一次任务操作台；不是自动重置场景的无人值守压测器。当前统计为最近 200 条显示记录，原始 JSON 留在服务器。

## 本项目 Web

源码：`services/dashboard/piper_scale.py`、`static/scale.html`、`static/scale.js`；只读图像入口 `scripts/piper_preview.py`。复用已有 dashboard 的 token-cookie/JSON HTTP 防护，不增加控制协议。

GitHub 配套代码位于 [CoCoMatrix/rax-skills 的 web-console](https://github.com/CoCoMatrix/rax-skills/tree/main/web-console)：完整 Web 三文件、Mac/Linux 一键入口、连接配置示例、备份安装器和离线测试。它需要本机已适配的 RAX/Piper/XPolicy runtime，不是干净系统上的整套机器人安装包。

Mac 双击 `web-console/Open-RAX-Console.command`：检查/启动 Web 服务 → SSH 转发 → 验证 token → 打开页面 → 未连接时只发一次只读 Connect 并等待三路相机；已有 Prepare/运行会话时直接复用。5090 桌面执行 `bash web-console/Open-RAX-Console.sh`，不用 SSH。`--open-only` 仅打开页面，`--no-browser` 用于连接检查。刷新网页不自动动作；启动器不会发送 Prepare/Run/夹爪命令，不自动重试硬件连接。

可把 `console.example.json` 复制为 `console.local.json` 配置另一台服务器；本地文件不入 Git。上传的 Web 是本机 500 mm 工作区快照，修改工作区时应同步后端配置、候选预检、界面及文档。安装器先验证依赖与范围，备份后仅复制 Web 三文件，不覆盖冻结控制器、校准、凭据或运行记录；有 Web 服务持锁时拒绝覆盖。

服务器启动（在 RAX 根目录）：

```bash
PYTHONPATH=. /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python -m services.dashboard.piper_scale --port 8781
```

监听 127.0.0.1:8781。Mac 用 SSH 转发访问；不把可写控制口直接暴露到公网。Mac 启动脚本读仅当前用户可读的访问 token，再打开浏览器；不在终端打印密钥。

- Connect：原生只读 graph，三路相机和状态，不启动 FFS/模型/电机写入。
- Prepare：已有 `piper_prepare.py` + token/session 校验的 `Operator.command('prepare')`，不再额外闭爪。
- Run：已有 `piper_explore.py --rehearsal-candidate ...` + `run_no_agent.py`，禁止并行启动；源候选 commit 必须匹配。
- Stop：先原生 Stop/disarm 和反馈确认，再退出本控制台的进程。别的 owner 会阻止接入，不能抢占。
- 相机画面为低频实时 HTTP 预览，不是度量帧；控制器仍按需读取原本的绑定 RGBD。预览不增加 Gemini 或 FFS 请求。
- 每条实验记录包括版本/commit、自动测得的摆位、各阶段耗时、辅助视觉次数、控制器总耗时和含启动/收尾的整轮总耗时。缺少初始定位时明确标记未取得，不填人工推测坐标。
- Web 显示左法兰 X 55–500 mm、Y −400–45 mm、Z 170–360 mm，以及失败阶段/原因；不要把位置范围内理解为任意夹爪朝向都能求解。
- 数据：`runtime/scale-tests/*.json` 与 `commissioning/web-*`。连续录像使用已有会话录制，Web 不另录一套或无限堆图。

重启控制台不会自动接管遗留写会话；先核对已有进程/原生 Stop 结果。不要删除旧锁后直接重放任务。若未来要换机器人，替换宿主 adapter 和工具配置，不能把 Piper Prepare/路径硬编码推进通用 harness。
