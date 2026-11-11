# Junqi 的 Piper 配置快照（2026-11-10）

这些是本机经验和验收参数，不是 Franka 原生默认值，也不是对其他设备的推荐值。

| 项目 | 当前配置 |
|---|---|
| 服务器 | ubuntu@192.168.200.102，RTX 5090 |
| RAX 根目录 | /home/ubuntu/code_junqi/rax-piper-franka-20261104 |
| 服务 Python | /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python |
| 隔离任务 Python | /home/ubuntu/code_junqi/rax-piper-agent-isolated |
| 机械臂 | 双 Piper X，任务主要左臂/左爪，右臂目标保持 |
| 头部 / 左腕 / 右腕相机序列号 | 262522073779 / 262422071560 / 262522070284 |
| 深度 | Fast FoundationStereo；头部有 base 外参；左腕 FFS 已接通但手眼暂缓 |
| 控制 | sole XPolicy owner，MoveIt 2/KDL IK，installed FCL 路径检查 |
| TCP / 指尖 | 用户确认 A 为 link6、两橙垫中心轴向均 140 mm，最前端 170 mm；执行检查使用安装后的几何包络 |
| 速度 | SDK 50%；有效关节峰值约 17°/s、加速度 40°/s² |
| 抓杯 / 试抬 | 目标 3 mm；试抬 40 mm，实际载荷开度需看回执 |
| 反馈容差 / 持稳 | 0.01 rad；0.3 s 且 ≥4 个不同反馈样本，jaw 稳定范围 0.3 mm |
| 单次位姿范围 | 最大 400 mm，仍需 workspace / IK / 路径验证 |
| 左法兰工作区 | `left_link6` 在 `left_base_link` 下：X 55–500 mm，Y −400–45 mm，Z 170–360 mm；不是 TCP 坐标范围 |
| 原生 Prepare 闭爪反馈 | 左约 0.3 mm，右约 2 mm；由当前 profile/校准决定，不额外下发闭爪 |
| 视觉代理 | XQNetwork http://127.0.0.1:7892；主模型中转直连，本 main 不启动主模型 |

密钥只在服务器已有文件读取，不复制到 skill、网页、日志或分享包。地址和序列号需在新环境重新配置。

## 版本

- 当前可写合并候选 `workspaces/blue-cup/20261109-v0.4-merged-candidate`，v0.4.0-rc10。
- 嵌套代码 commit `b4d3c1720ad635464fe099043ea4fd708870b902`。
- 冻结 v0.1：`versions/blue-cup/20261104-r5-v0.1.0`。
- 冻结 v0.3：`versions/blue-cup/20261104-r12-v0.3.0`。
- 工作区修改前备份：`backups/blue-cup/20261110-rc9-before-workspace500`；此前合并备份继续保留。
- 当前候选不是待修改的冻结副本。任务子会话由启动器复制，保持源候选身份可核对。

## 已测试任务顺序

prepare_for_approach（初始 RGBD 定位与任务姿态准备，不是原生回零）→ hover → entry → grasp → trial_lift(40mm) → 读取本轮初始杯垫目标 → transfer → lower_to_height → release → retreat。

放置高度由入杯时实测法兰高度与杯垫/桌面高度差计算，不盲目下压。正常移动以反馈和路径检查为主；试抬、最终结果保留普通定性图像判断，必要时才辅助视觉。

原生 Prepare 是另外的宿主操作；测试完成后 Stop/disarm 留在退开姿态，不意味着已经回零。相机 Web 是 owner 预览，不保证精确像素/深度绑定；度量仍使用带元数据的 MCP observe。

## 工作区与可达性

配置文件为 `examples/blue-cup-exploration.json`，`workspace_min=[0.055,-0.4,0.17]`、`workspace_max=[0.5,0.045,0.36]`，单位米。rc10 的 `piper_check_path` 与执行前检查共用 41 点关节路径 FK 工作区检查；候选代码要求 `workspace_clear=true` 与原碰撞检查均通过。准备阶段在张爪/移动前先检查整体准备目标是否落在工作区。Web 显示这组范围。

位置包围盒不等于完整位姿可达空间，也不替代自碰撞、桌面和其他已建模环境检查。500 mm 与单次移动 400 mm 是不同约束。0.01 rad 是到位/状态检查容差，不扩大 URDF 关节限位。

2026-11-10 仍发现新摆位的 hover 固定朝向需要 J4≈89.55°，超过模型上限 89°。小段候选只缩短位移，没有解决姿态边界。完整 hover/entry 的前置可达性选择尚待实现；详见 `lessons.md`，不要把它写成 rc10 已修好。
