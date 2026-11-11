# Piper 操作员停止入口

`piper_hold` 是动作边界工具。同一 RAX Frontend 将工具调用串行化；运动工具尚未返回时，后来的 `piper_hold` 会等待，不能作为活动动作的紧急停止入口。需要立即停止时使用物理急停；软件停止使用下面独立于 Frontend 的操作员入口，或实际硬件 owner 的 Stop。

从新项目目录运行，指定本次新会话的配置；若使用自定义 `RAX_ARTIFACT_ROOT`，保持与启动配置一致：

```bash
/path/to/host/python scripts/piper_operator.py \
  --config /path/to/new-session/operator-input.json disarm
```

该命令依次尝试：

1. 将新 RAX 会话的 arming 设为 false。
2. 将配置中 `owner_session_root` 对应的实际 owner 会话 arming 设为 false。
3. 验证 `bridge_root/state.json` 的会话、当前 heartbeat 与 instance，写入该实例的 `cancel.json`。

它不调用模型、不经过 MCP 调度锁、不启动 owner、不 enable、不 Prepare、不移动或自动 rearm。某一步失败仍尝试后续独立停止路径。桥不可用或身份不匹配时不猜测实例，但前面的撤销仍会落盘；输出 `errors` 并返回非零退出码。

`session_disarmed`、`owner_disarmed` 表示对应撤销写入成功；`cancel_requested` 只表示发出了取消请求，**不是桥已消费，更不是机器人已静止**。输出始终保留 `physical_stop_confirmed=false`。操作者随后核对新鲜硬件反馈和现场实物；有危险或软件路径异常时使用物理急停，不等待代理回应。停止后的恢复仍由操作者处理，此脚本没有 arm/enable/start 功能。

## 新会话绑定

本次 live 配置的 `owner_session_root` 和 `bridge_root` 必须来自**新项目下本次创建的 owner 会话与桥实例**，不复用旧成功探索 run 的会话、armed文件、反馈确认或桥目录。沿用的是既有 owner 的协议与经过核对的硬件实现，不是旧轮次的现场许可。

同一硬件 owner 只能有一个任务控制者：启动新会话前确认旧代理、旧workbench自动任务及其他运动脚本均不再提交动作。一个CAN writer仍可能收到多个上层任务的请求；不能只看CAN进程数量就认定互斥成立。

## 从已结束的探索接续

```bash
/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python scripts/piper_explore.py \
  --output /home/ubuntu/code_junqi/rax-piper-franka-20261104/commissioning/NEW_RUN \
  --continue-from /home/ubuntu/code_junqi/rax-piper-franka-20261104/commissioning/FINISHED_RUN \
  --seconds 900 --max-model-calls 80
```

默认只生成计划；`--run` 才启动设备和模型。接续创建独立的有界物理会话与新的控制实例，Prepare 保留当前实测双臂和夹爪，不要求零位。旧 memory 继续使用，但旧 PROGRESS 先保存为历史，再由实际工具日志、停止反馈和帧索引重建交接；旧 code 只放 references，不自动运行。启动会重新读取当前关节、夹爪与三路图像，模型确认当前场景后只做剩余步骤。

`wall_budget` 随每次工具成功或失败返回；父、子进程共用单调时钟预算，最后 60 秒（短会话为总时长三分之一）不再接受新运动，保留 hold、观察和交接。普通 native `--resume` 保留已用时间，不自动延长；预算已耗尽的物理任务使用上面的显式新会话接续。模型最终文本来不及写时，runner 仍从日志生成 `HANDOFF.json` 和 `HANDOFF.md`。退出码、到位或夹爪宽度均不能单独判定任务成功。

探索通常使用 `observe_after=true`，省略 `duration_s` 以使用现有 owner 的自动定时。动作确认后等新采集图像，观察失败不会重放动作。模型 relay 的 `model-usage.jsonl` 记录请求大小、响应头、首字节、首事件、首输出/工具增量和完成时间，用来区分等待发生在哪一段；它不记录密钥或请求正文，也不自动重试。

`motion_enabled=true`、`mode=LIVE_BRIDGE` 是配置状态，不能单独证明接入就绪。还需核对实际 owner/bridge 无错误、实例匹配、当前反馈与三相机、两份 arming，以及本轮现场。工作箱检查的是 `left_link6` 法兰路径；41点FK和旧CAD自碰撞不等于桌面、杯体、线缆等现场环境已验收。不要照搬旧轮工作箱或将历史确认当成当前事实。

## 2026-11-04 用户约定

回零只由 XPolicy 的确定性入口完成，不交给探索模型。实验参数 `--return-to-zero-first` 已禁用。

新探索不带 `--continue-from`，新工作区 `code/` 必须为空，启动时写入 `task-code-at-start.json`。只带用户条件性经验，不导入旧任务代码、轨迹或控制器。底层 XPolicy owner、MoveIt IK、运动桥属于设备适配，不能解释为已加载杯子任务策略。模型可在探索中创建修改 Context helper；首次实物成功后可整理代码，但另一轮真机演练需要新场景，不能把第一次直接探索混算为规模测试。
