# main(ctx) 整段测试台

当前候选 v0.4.0-rc9，commit a6178516076809acc6f70089662c9a470e11406e。控制台不修改控制器参数，不覆盖 v0.1 / v0.3 冻结版本。

## 使用

在 Mac 双击 `RAX-整段测试台.command`，打开 http://127.0.0.1:8781/。启动脚本建立 SSH 隧道并认证；直接打开未认证浏览器会被拒绝。服务器服务只监听 loopback。连接断开时重开这个文件即可。

1. **连接 / 重新连接硬件**：只读三路相机和反馈，不使能电机、不启动 FFS 或模型。先退出本控制台自己的旧会话；遇到其他 owner 不抢占。
2. **回到 Prepare**：复用 XPolicy 原生回零和闭爪。实际姿态到位后开放任务按钮，不额外下发闭爪。当前若持物先判断回零是否合适。
3. 重新摆放物体并勾选现场就绪。**无需输入摆位**，初始视觉自动定位杯子和杯垫。
4. **执行一次 main(ctx)**：新的独立会话加载可写候选，保持冻结版不变；不启动主 Agent，普通规则不足时才使用现有 Gemini 兜底。
5. **停止本轮**：走原生 Stop/disarm 并确认反馈；不自动松爪、回零或重放任务。结束后的退开位置不等于 Prepare。

Web 显示三路 HTTP 实时预览，约每 1.2 秒更新，不额外请求模型/FFS；它不是绑定 RGBD 的度量接口。连续录像仍由既有任务后端承担。

## 实验记录

每轮一条 JSON，页面列出：版本和 commit、初始视觉定位结果/坐标系/来源帧、各阶段耗时、辅助视觉和主 Agent 调用次数、控制器总耗时、含硬件启动及收尾的整轮耗时、结果和失败阶段。可以展开阶段和导出 CSV；刷新或重启网页不会丢记录。初始定位失败明确记为未获得结果，不伪造位置。

记录在服务器 `runtime/scale-tests/web-main-*.json`；完整日志/状态/录像位于 `commissioning/web-main-*`。页面显示最新 200 条，文件继续保留。人工介入不算自主成功，不自动循环任务。历史 M 轮已从真实报告导入：58.661 秒控制器，83.908 秒 supervisor，0 主 Agent / 0 Gemini，10 个阶段；不是本次网页又做了一轮运动。

## 实现入口

- `services/dashboard/piper_scale.py`：编排与记录，复用现有 dashboard 鉴权。
- `services/dashboard/static/scale.html` / `scale.js`：界面。
- `scripts/piper_preview.py`：只读 owner graph，复用 `piper_observe_only.make_graph`。
- 写入口仍是 `scripts/piper_prepare.py` / `scripts/piper_explore.py` / 候选 `run_no_agent.py`。

启动：在项目根目录用现有服务 Python 执行 `PYTHONPATH=. python -m services.dashboard.piper_scale --port 8781`。没有端口向公网开放或第二套 SDK 控制。浏览器不接收 owner token/API key。

## 2026-11-10 验证范围

本地和 5090 的 18 项离线控制台测试通过；真实只读三视角显示、重连和停止通过，hardware_write_enabled=false、hardware_write_count=0。浏览器无控制台错误，实验记录和 CSV 正常。Prepare/Run 按钮已接入并通过假进程生命周期测试，**未通过这两个新按钮再次发起真机运动**；旧入口此前真机验证仍保留。候选105文件、冻结v0.1的60文件和v0.3的109文件SHA校验不变。
