# 2026-11-04 验证记录

模型网络、真实图像、真机只读观测与一次受监督最小关节往返已通过；新项目尚未抓杯或完成物理任务验收。下列检查分别提供证据，不相互替代。

| 检查 | 结果与证据 |
|---|---|
| 未修改的Franka源码基线 | 本地150 passed、1 skipped；macOS使用短临时路径避开AF_UNIX长度限制 |
| 当前全量单元测试 | 5090 **206 passed**（21.27s）；macOS **205 passed、1 skipped**（22.37s）。`verification/final-regression.json`；无硬件命令、无模型请求。早期175项记录保留于 `verification/validation-tests.log` |
| 经验格式 | `experience-memory/SKILL.md`及`references/format.md`与固定上游逐字节哈希一致，值在 `UPSTREAM.json` |
| 真实MoveIt2 KDL | 三组CAD目标通过，其中两组扰动seed；独立FK误差最大5.71e-8m/3.59e-7rad。`verification/piper-pure-ik/RESULT.json`；无硬件写入 |
| Codex隔离 | 5090真实bwrap内Python依赖导入、原Context读取mock状态通过；key.py及旧源码不可见。详见 `CODEX_ADAPTER.md` |
| 模型网络 | 任务独立代理 `127.0.0.1:17890` 已通，TLS校验保持开启、全局代理配置SHA未变。`verification/network/task-proxy.json`；此前7890/7891的TLS EOF是历史失败记录 |
| 主模型与辅助视觉 | 指定主 `gpt-6-astra/high`、辅助 `gpt-6.1-sol/medium` 均通过真实归档图像和本轮三视图请求。`verification/model-smoke/{primary-astra-high,vision-sol-medium}.json` 与 `verification/current-model-check/{primary,vision}.json` |
| 原生Codex + 独立Frontend + MCP mock | exit 0；一次status、一次辅助视觉，0 MCP错误、0运动调用，Frontend清理完成、无强杀。`verification/model-smoke/native-codex-process.json`；进程关闭不代表物理停止 |
| 真机只读观测r2 | 300秒配置，589条owner状态覆盖295.63秒，写使能全false、写计数全0，14维测量值范围全0；3组三视图快照、589个头部数据包，147文件归档哈希验证成功。`verification/piper-observe-only-r2/{RESULT,SUMMARY}.json` |
| Piper backend读图与几何 | 本轮NO_SEND状态、三相机原图、元数据和3个图像选点反投影已保存于 `verification/current-cameras/`。几何记录明确 `motion_permit=false`，选点未作为抓取/支撑目标验收 |
| 原生Codex + 真实Piper整链路r3 | **只读通过**，32.29秒；status、三视图observe、辅助vision各一次，0 MCP错误、0运动调用；主模型实际查看三图，辅助返回 `gpt-6.1-sol/medium`。`verification/live-native-r3/summary.json` |
| 最小关节往返 | **通过**，22.17秒；左J1目标+0.5°、实测+0.486°，返回0°；其余13维实测不变。去/回各150个SDK轨迹回执并确认实测到位。`verification/piper-minimal-live-r1/SUMMARY.json` |
| 抓取、函数演练、组合控制器、多摆位测试 | 本次均未执行 |

## 只读观测的范围

r2三台相机各输出约8,800帧，timeout/restart均为0，分别报告10/12/11丢帧。RESULT中的10次连接拒绝来自UI启动等待；Dora graph退出码0，FFS的-15来自launcher结束信号。该轮设备进程已退出、8091/19027端口空闲；这是该次结束时的状态，不是后续运行时的就绪证明。详见 `PIPER_OBSERVE_ONLY.md`。

头部1280×720 RGB与FFS深度绑定到同一不可变数据包，并保留模型、内参、源时间戳和哈希。腕图来自960×540 UI图像，只有HTTP元数据前后包围，不是精确帧绑定；r2三组快照跨视角源时间跨度0.577–0.830秒，不能当成硬同步三视角。已保存图像可以离线分析，其年龄不代表当前现场仍适合执行。

r3原生整链路使用RGB-only，`frame_id=3ea9c8424bb14a05861960620da07ad7`，三图都标 `metric_depth=false`、HTTP metadata bracket，不能沿用r2的头部RGB-D同包声明。其观测graph取得350条状态，写使能全false、最高写计数0，归档验证完成、设备进程退出；保留启动/退出时的stale反馈及连接拒绝记录。Frontend正常清理、没有强杀，仍明确 `physical_stop_confirmed=false`。源图、模型输入哈希与退出证据见 `verification/live-native-r3/`。

## 最小物理接口验收的范围

`minimal-live-20261104-r1` 在现场确认后通过新Piper backend执行受限往返。除左J1目标变化外，两段轨迹的其余目标维度保持原值，实测端点的其余13维不变。启动实际包含双臂Enable、用当前实测双臂目标Prepare、Start及控制模式帧，结束包含Stop/持位；全部夹爪初始化与派发关闭。300是去/回两段的SDK轨迹步数，**不是所有硬件写入的总数**。

独立Stop进程收到实际owner的 `stop/ok=true`，两份arming文件均为false；随后8条新反馈跨0.3697秒验证静止。bridge和graph退出码均为0，控制与传感器临时数据都完成归档，设备进程及8091/19027端口已释放。证据见 `verification/piper-minimal-live-r1/{RESULT,SUMMARY,STOP-RESULT,FRAME-EVIDENCE}.json`。

该记录只验收一次小关节接口和返回后的软件Stop/静止反馈。它没有验证物理急停、夹爪控制、右臂主动运动、全姿态IK目标运动、物体接触或杯子成功率；也不是Codex自主选择并执行抓取动作的证明。

## 本次保留的限制

- 右臂动作和右爪尚未开放；并非双臂运动复现已经完成。
- 头部外参为用户指定暂用候选，LOO最大位置误差约18.08mm；腕部有效外参和抓取TCP尚未验证。
- 已有快照与限时观测归档；Franka连续录像能力尚未适配。
- 新backend使用外部owner/camera/FFS协议，backend本身不启动设备。只读验收不会自动创建或arm运动bridge。
- 纯IK和41点工作空间检查不等于已建模桌面、杯子、线缆等环境的全场景避障。
- Codex原生compaction/resume已接线，尚无长上下文压缩与长期恢复验收；接口短测不能证明这一点。

## 下一项实际工作

从当前实测现场与任务工作箱开始杯子探索；单次关节接口验收不扩大为任意工作空间、抓取TCP或碰撞安全的验证。任务流程仍是Franka的**直接工具探索→单函数演练→`main(ctx)`闭环组合→固定版本多摆位整轮测试**，见 `WORKFLOW.md`。当前没有已成功的杯子控制器，旧混合版本成功记录不替代新版本验证。
