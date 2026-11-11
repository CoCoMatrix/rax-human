# Piper v0.4 配置与版本身份

2026-11-10 快照。本文件的绝对路径用于在原部署定位证据；迁移机器时由使用者定位对应环境，不假定这些目录自动存在。skill 不包含密钥、相机原图、控制器完整源码或硬件驱动。

## 原工程与版本链

服务器 `ubuntu@192.168.200.102`，根目录 `/home/ubuntu/code_junqi/rax-piper-franka-20261104`。以下路径均相对此根目录：

| 身份 | 路径 / 状态 |
|---|---|
| 冻结 v0.1 | `versions/blue-cup/20261104-r5-v0.1.0`，保持不变 |
| 冻结 v0.3 | `versions/blue-cup/20261104-r12-v0.3.0`，保持不变 |
| R31c 实际演练副本 | `commissioning/blue-cup-v03-rehearsal-20261109-r31c/rax-session/agent-workspace` |
| 合并前演练备份 | `backups/blue-cup/20261109-r31-before-merge` |
| 演练到合并的可写中间副本 | `workspaces/blue-cup/20261109-v0.3-r31-merge-working` |
| 当前 v0.4 可写候选 | `workspaces/blue-cup/20261109-v0.4-merged-candidate` |
| 500 mm 修改前备份 | `backups/blue-cup/20261110-rc9-before-workspace500` |

当前候选实际代码版本为 **v0.4.0-rc10**，`code/blue_cup` commit **`b4d3c1720ad635464fe099043ea4fd708870b902`**。这是原控制代码的 commit，不是本 skill 仓库 commit。

用户所称“v0.4 保存的演练副本”涉及上述版本链：R31 的逐函数演练产生基础，随后可写 v0.4 在整段测试中持续优化。新一轮逐函数验证应使用 v0.4 实际被 main 导入的函数，而不是退回原 R31 提示。**并非所有 v0.4 改动都已在新的逐函数会话中重新验收。**

`WORKSPACE.json.latest_whole_run`、`merged_candidate_live_validated` 和源码 README 的旧 rc9 标签可能滞后。以实际代码 commit/hash、imports 和对应运行报告核对；既不能仅凭旧标签否认已有实测，也不能只凭新目录名宣称验证完成。

## 当前设备和数值选择

| 项目 | 本机配置与含义 |
|---|---|
| 计算 / 机器人 | RTX 5090，双 Piper X，主要由左臂/左爪完成杯子任务；右侧保持 |
| 相机 | 三路 RealSense：头部 `262522073779`、左腕 `262422071560`、右腕 `262522070284` |
| 深度与标定 | Fast FoundationStereo；头部已有到基座外参但仍属当前项目标定证据；左腕仅 optical-frame 深度，手眼暂缓 |
| 控制与规划 | XPolicy 唯一硬件 owner；MoveIt 2 / KDL 完整位姿 IK；FCL 采样路径碰撞检查 |
| TCP | 用户确认 A 环形接缝对应 link6，两侧接触中心轴向均 140 mm，安装无偏斜；作为当前几何候选/标定输入 |
| 指尖 | 轴向最前端 170 mm；执行路径使用指尖包络和已建模桌面区域，TCP 与碰撞形状分别处理 |
| 指令坐标 | 目标位姿为左法兰 `left_link6` 相对 `left_base_link`，不是直接将 TCP 目标当法兰目标 |
| 入杯 / 抓取 | 杯沿下 40 mm；当前目标开度 3 mm。R31 为 3.5 mm、更早为 4 mm，都是历史经验，不等于当前指令 |
| 试抬 | 至少 40 mm；有效正值 10/20 mm 的旧建议在计算目标前归一为 40 mm，而不是为此直接拒绝整轮 |
| 速度 | SDK 50%；当前有效关节峰值约 17°/s、加速度 40°/s²；动作时长自动计算 |
| 到位 / 观测状态绑定 | 本机用户指定 0.01 rad；不扩大关节硬限位，也不作为精密小角度能力证明 |
| 持稳 | 0.3 s，至少 4 个不同反馈样本；夹爪稳定阈值 0.3 mm |
| 单次位姿移动 | 最大 400 mm；还受函数局部限制、工作区、IK、碰撞检查限制 |
| 左法兰绝对工作区 | 最小 `[0.055, -0.400, 0.170]` m；最大 `[0.500, 0.045, 0.360]` m |
| 模型 | 主模型通过 API，当前采用 gpt-6-astra；辅助视觉为宿主 `gemini_generate` 对接的 Gemini，详见接口文档 |
| 记录 | 连续滑动窗口录像及机器人状态记录；按需观察快照另计，录像不意味着每帧都传模型 |

所有数值都是本机选择；本 skill 不自动写入配置，也不声称另一台 Piper 可直接复用。几何输入来自测量和用户确认，完整指尖/环境碰撞覆盖与手眼精度仍有独立验收范围。

### 工作区 500 mm 与 IK 的区别

500 mm 是法兰 X 的绝对上限；400 mm 是单次位移上限。rc10 已在准备前核对整体准备目标，并让路径预检和执行共用 41 点 FK 工作区检查；碰撞检查继续执行。采样通过不是连续空间内任意物体和任意姿态的证明。

实测新摆位 hover 点约 `(360.752, -147.035, 294.000)` mm 在范围内，但指定朝向的当前解支要求 J4≈89.549°，超过 89° 限位。离线同点绕基座 Y 倾斜 −1°可求得 J4≈88.449°；降低 5 mm 也有可解对照。**这些只是 IK 对照，不是已实施的抓取策略修复或全路径验证。** 完整 hover/entry 位姿候选的提前预检仍待补。

### Prepare 的两个含义

- XPolicy 原生 Prepare：双臂回零和校准闭爪，走独立入口；即使任务主要用左侧也会影响右侧。
- `scheduled.prepare_for_approach`：本轮初始定位和接近准备，可能移动到任务准备姿态；不是回零。

不要将回零交给探索模型，也不要在函数失败后擅自回 Prepare。用户明确要求时按现有入口执行，先依据当前实测持物/设备状态处理已有前置条件。

## 核对过的实现来源

此次发布对照了服务器与 Mac 上以下文件，内容 hash 一致：`services/controller/google_gemini.py`、`services/controller/frontend.py`、`scripts/piper_explore.py`、`harness/codex_cli/launcher.py`，以及当前候选的 `perception.py`、`checks.py`、`scheduled.py`。候选 commit 在服务器也核对一致。

网络关键源码 SHA-256：

- `google_gemini.py`：`9e6d859a9718ee1991ac245d1bc532e79d0156814d9e4ad4d5129bd9a56ca1d3`
- `frontend.py`：`a057f57a5abdd86cda0b739b8a730b32af6e12a4ae6c5f48104c34d2955c0307`

发布只读取这些实现和已有报告，没有启动新的硬件动作或付费模型请求。
