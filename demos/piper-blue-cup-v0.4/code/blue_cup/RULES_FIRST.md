# v0.4.0-rc2：函数条件检查与按需 Gemini

本次修改当前 v0.4 可写合并副本。原始 R31、v0.3 演练副本、冻结 v0.1/v0.3 不变。
修改前完整备份：`backups/blue-cup/20261110-before-rules-first/candidate.tar.gz`，139 个文件及 SHA256 清单已逐项核验。

## Franka 原生实现的依据

- [autonomous-control](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/task/autonomous-control/SKILL.md)：普通 `fn(ctx, inputs)` 逐函数演练后组合 `main(ctx)`；普通规则不足时使用 Gemini。没有固定的逐动作模型检查要求。
- [相机配置](https://github.com/robot-agent-exploration/rax/blob/real/franka/examples/franka.json)：两路固定外部 ZED `left/right`，一路随手运动的 `wrist`。不是本机的一头两腕布置。
- [观测与取点实现](https://github.com/robot-agent-exploration/rax/blob/real/franka/services/franka/backend.py#L465)：保存当前 RGB、深度、K、T_base_cam。`franka_pixel_to_position` 对有效深度取中值、反投影并变换到基座。
- [腕部坐标变换](https://github.com/robot-agent-exploration/rax/blob/real/franka/services/franka/calibration.py#L66)：由 TCP 位姿和腕部手眼矩阵得到相机到基座变换。后端取采集后 TCP 状态，并记录采集前后位移；不是高精度逐相机运动同步的保证。
- [定位指导](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/perception/depth-franka/object-localization/SKILL.md)：选择当前可见的有效表面点，外部视角交叉核对；腕部提供近距离几何。支持 Python/OpenCV 批量反投影。
- [腕部观察指导](https://github.com/robot-agent-exploration/rax/blob/real/franka/agent_skills/perception/common/wrist-camera-inspection/SKILL.md)：外部视角不清楚时，用空载机械臂调整腕部观察位；不为观察而松开持物。合并后的任务代码仍使用相同接口。

仓库提供这些接口和组合步骤；公开 examples 只有设备配置。未找到一个发布的、可直接照搬的合并版杯子控制器，也没有通用自动多视角融合器。这里的杯子轮廓、橙垫和颜色规则是本任务适配，不冒称上游自带算法。

## 本次行为

1. `move_once` 保留授权、状态、IK、安装指尖几何路径、到位和异常停止检查；条件字典不再等于模型调用指令。`checks.assess` 先执行普通规则，只把未解决的事实交给一次 Gemini。
2. 空载/接近：当前杯沿轮廓与有效头部深度、保留的同次接近目标、当前状态和路径共同检查。完成 hover/entry 由 FK 目标误差判断，不追加隐藏指尖拓扑的模型判定。
3. 环境：把实测桌面任务区域变换到 dual_base_link 后加入现有 FCL 路径检查。避开已知固定安装板的占地，不删除碰撞或放宽容差。可见手部路线使用当前深度；搬运/下降同时检查杯体包络。不是完整环境/线缆建模。
4. 头腕配合：在已绑定、已检查路径的同次入杯/闭合边界，头部提供当前杯沿/桌面几何，左腕识别两侧橙垫和其间可见蓝杯区域。头部遮挡且无可疑前景时，组合证据可支持大体抓取准备状态。不要求证明隐藏的内外夹持拓扑，不把这个结果当作已抓住/已支撑。相关近场操作的头部信息不足时，按需请求一次 `piper_observe(cameras=[head,left,right], require_depth=True, depth_cameras=[head,left])`；核对 owner、last_motion、状态和新鲜度，重新绑定新观测并在动作前复查当前起点路径。对橙垫和杯壁有效像素统计腕部光学坐标中的深度，作为局部证据；异常或不足仍交给 Gemini。头部足够时不请求腕部深度。
5. 试抬/搬运：40 mm 试抬经验不变。用实际运动反馈配合腕部杯体特征的正反光流，判断杯子是否大体随爪移动；无纹理、目标丢失、明显相对位移时回退。到位或闭合 3 mm 本身不是抓取成功。
6. 放置：用当前杯沿深度、同次实测杯高和杯垫对齐关系估计大体支撑；这是定性结果，非力传感器接触证明。不能仅凭目标高度判断成功。松开/退开后重查当前杯子是否仍在杯垫区域。
7. 同一持位边界共享已证明的物体事实，前提是新图像无变化、姿态/owner/last_motion/标定一致且仍在短边界内。不同动作的路线条件不继承。杯垫定位若已有新鲜深度，不再固定再取一次。
8. 记录 `condition_checks`、`fact_sources`、`fallback_reason`、实际 `visual_calls`、各函数耗时。未知不是已失败的物理动作，也不是默认批准；无足够证据时保留原生返回/人工事实入口，不盲目重复动作。

## 离线证据与限制

- 原 B 测试：235.798 秒，主 Agent 0 次、Gemini 14 次；完成 12 条动作回执，在入杯处停止，未闭合/试抬。不是整段成功。
- 原 B 的 12 条路径重放：新增任务桌面模型全部通过。起始开爪指尖包络末端与安装几何差约 1.4e-10 m。只是离线模型一致性。
- 新普通规则能独立解决当前杯沿/桌面、部分路线及合成的抓取/支撑判断；未知深度、外来前景、状态变更、少一个橙垫、视角不同步等负例不会静默通过。
- 25 组原始头腕观测离线对照：近距离入杯末态 1 组同时具有可用杯沿深度及两橙垫之间的杯体区域；其他视角多不满足这个特定近距离规则，不是 24 次任务失败。
- 在该末态上，用新控制器的数值到位绑定构造下一函数的离线条件检查：头部路线可见性单独不足；加入腕部和已通过的几何路径后可支持定性 `grasp_setup`，模型调用 0。这个绑定是离线模拟输入，不改写旧运行、不授权真机运动、不宣称抓取成功。
- 本轮历史记录没有同期腕部深度/原始双目包；按需 FFS 调用、坐标契约、状态重绑定和有效深度统计已通过离线接口/合成数据测试，真实腕部 FFS 时延与质量尚未验证。手眼标定未完成，腕部 RGB 不被伪装为基座坐标系内的毫米间隙。
- 远处空载路线仍可能因遮挡/环境不完整回退 Gemini。固定调用已取消，但不保证下一次整段零调用；速度提升、试抬保持、释放/退开及换位泛化必须通过后续实测。

本次没有启动硬件、相机或运动。最后一次已核实的机器人状态为入杯位置停止、未抓杯，不是 Prepare。下一次真机测试按原生流程先经 XPolicy 复位、重新摆位，运行一次新会话的 `main(ctx)`；不要从旧入杯状态直接重跑 main。

最终离线回归：本地与 5090 各 276 项通过。计数包含腕部按需调用、头部充分时不调用、状态变化拒绝、光学坐标不升级为动作目标等用例。
