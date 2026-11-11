# v0.6：合并 main 之后的轨迹优化与复现

这是当前5090的 **v0.6.0-continuity-rc1** 源码快照，任务代码commit `0c7e6d58d18c829b1ed4140780e980a46731bb94`。原v0.4 demo、v0.5成功版本、冻结版本均保留；本目录不自动改现有Web或启动硬件。

入口：[controller.py 的 main(ctx)](code/blue_cup/controller.py)。完整控制器及依赖模块逐字保留，宿主关键适配放在[host-overlay](host-overlay)，独立Owner完成事件补丁放在[owner-completion.patch](owner-completion.patch)。这不是干净机器上的完整Piper驱动安装包。

## 1. 处于什么阶段、优化了什么

RAX Franka的成功经验函数化→逐函数演练→合并`main(ctx)`→新摆位测试继续保留。v0.6是合并后基于耗时证据进行的独立Piper轨迹优化，没有增加主Agent或另建控制框架。

```text
首次RGBD自动定位 → 初始场景检查与张爪
→ 提前规划接近、40mm试抬后的搬运候选
→ 轨迹一：准备/接近（指定非接触路点速度衔接）
→ 一次完整闭到3mm → 40mm试抬及普通定性结果确认
→ 根据实测状态核对搬运候选，匹配复用，否则重新求解
→ 轨迹二：水平搬运/下降（杯垫上方转折衔接）
→ 到位持稳 → 释放 → 退开和最终定性判断
```

- Owner完成状态变化即时发布，消除等待下一次1秒周期发布的多余延迟。
- 复用同请求/Owner/实测状态、完成后≤0.5秒内有效的0.3秒/至少4样本持稳证据；失效则重做，捕获失败后的第二次必须重新持稳。
- 试抬观察先RGB；普通规则不足才按需深度/Gemini。首次场景仍重新度量，不沿用历史摆位。
- `piper_close_gripper`只允许从≤20mm开度单调闭合，臂不动；目标3mm，保留原10mm/s夹爪速度与力设置。普通夹爪工具仍限制单步10mm。
- 搬运候选复用要求同目标/Owner/几何、预测试抬关节与实际差≤0.01rad，目标FK误差≤1mm；实际不匹配则重算，不拿预测当测量。
- 指定非接触内点采用五次Hermite，准确经过路点，边界加速度为零，同向关节可以非零速度经过；反向关节速度仍为零。偏离原关节直线超过0.01rad则降低过渡速度，再验证速度、加速度、float32下发路径碰撞和FK工作区。

闭爪、试抬结果、支撑高度/释放仍保留边界，不是一条从准备直接穿过所有接触动作的不停轨迹。改变的是命令组织及部分速度边界，SDK仍接收30Hz位置流；需要另外验证真实运动连续程度。

更完整的问题→证据→修改→验收方法见 [skill经验](../../skills/rax-compose-and-test/references/main-trajectory-optimization.md)。

## 2. 实测与离线证据

两轮源码哈希均与本目录匹配，报告成功、Stop已确认、未标人工辅助：

| 阶段/指标 | 110421-05ae51 | 110617-13c15e |
|---|---:|---:|
| 接近（含初始感知/张爪/规划） | 41.342s | 28.728s |
| 闭爪 | 3.141s | 3.094s |
| 40mm试抬与结果判断 | 2.579s | 2.427s |
| 读取本轮杯垫目标 | 0.013s | 0.012s |
| 搬运与下降 | 5.196s | 6.036s |
| 释放 | 2.556s | 2.595s |
| 退开与最终定性判断 | 3.345s | 3.412s |
| **main总耗时** | **58.178s** | **46.311s** |
| 含启动/收尾 | 73.791s | 64.870s |
| 主Agent/Gemini | 0/1 | 0/0 |
| 搬运预规划复用 | 是 | 是 |

第一轮Gemini调用发生在初始接近准备阶段。两轮试抬与最终放置规则均无需Gemini。与v0.5的50.282/50.674秒相比，一轮较慢、一轮较快；不是严格A/B实验，也不能据此保证所有摆位零模型调用。

v0.5闭爪5.29/5.50秒降到约3.1秒；前动作ack累计等待约2.23/2.16秒在v0.6这两轮基本消失。试抬末SDK到搬运首SDK的间隔由2.310/2.642秒变为1.585/1.652秒。这些是控制调用时间，不是独立测得的物理停顿时间；此前按“运动末样本、不含末尾0.2秒驻留”计算的间隔会大约多0.2秒，不能混用口径。

[validation-runs.json](validation-runs.json)保存v0.5/v0.6的选定字段，包含逐阶段、命令间隔、源码/原报告哈希与状态，不含原图、完整审计或摆位坐标。[OFFLINE-ACCEPTANCE.json](OFFLINE-ACCEPTANCE.json)记录139项宿主检查通过；[INTEGRATED-REVIEW.json](INTEGRATED-REVIEW.json)记录真实安装几何、桌面FCL、工作区及ActionChunk契约验证。原路径轨迹时长仍为16.30/3.367秒，并非通过盲目压缩轨迹时间提速。

源文件中的部分旧注释和`validation='offline_candidate_not_end_to_end_live_validated'`是当时保留的标签；本页、版本/哈希与实测摘要说明本次发布状态。为了保留真实运行源码，本次没有改这些标签。两次成功仍不构成全工作区泛化、带载Stop延迟或物理C2连续性验收。

## 3. 先检查源码和运行环境，不接硬件

Python3.9+即可检查哈希/语法，不导入机器人模块：

```bash
python3 demos/piper-blue-cup-v0.6/verify_demo.py
```

在原5090克隆本仓库后，可进一步核对关键宿主与控制器：

```bash
RAX_V06=/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blue-cup-trajectory-v06-20261111/runtime
python3 demos/piper-blue-cup-v0.6/verify_demo.py \
  --runtime "$RAX_V06" \
  --installed-code "$RAX_V06/workspaces/blue-cup/trajectory-v06/code/blue_cup"
```

核对宿主overlay文件以及独立Owner补丁后的哈希；`XPolicyLab`自别名必须指向该独立Owner。它不校准传感器，也不证明当前现场可运行。

本机必须已有：双Piper X及SDK、sole XPolicy owner、原生review gate/ActionChunk、MoveIt2/KDL、FCL、安装后TCP/指尖/桌面几何、头部FFS/外参、双腕RGB、左腕按需FFS、本地模型配置和隔离RAX Python/MCP frontend。腕部手眼暂未标定，不把腕深度直接当基座坐标。

保持本机50%速度、有效峰值17°/s、40°/s²、3mm夹爪目标、40mm试抬、0.01rad关节状态容差、0.3秒持稳。单次位姿位移400mm，左法兰工作区`[55,-400,170]`到`[500,45,360]`mm；不要把500mm绝对X上限理解为单次移动或任意姿态可达。

辅助视觉仍通过宿主`gemini_generate`，主Agent不启动；本机Gemini代理为127.0.0.1:7892。密钥、模型权重、标定文件和设备凭据不在包内。接口详见 [Gemini说明](../../skills/rax-rehearse-functions/references/gemini-interface.md)。

## 4. 当前5090：通过独立Web复现

Mac仓库根目录：

```bash
python3 web-console/open_console.py --config demos/piper-blue-cup-v0.6/console.v06.json
```

5090桌面：

```bash
python3 web-console/open_console.py --local --config demos/piper-blue-cup-v0.6/console.v06.json
```

使用已有一键入口实现，只改配置：服务器8785，Mac隧道18785，runtime指向v0.6。默认旧配置和v0.4 Web不变。打开入口只读连接，不自动Prepare或Run；已有运行会话不会被替换。

1. 核对标题v0.6及版本`v0.6.0-continuity-rc1`、commit前缀`0c7e6d5`。
2. 若旧v0.5页面仍持有Prepare/相机，先核对其空闲状态，点旧页面Stop，等释放后连接v0.6；其他人的任务需先交接。不删除锁，不kill替代原生Stop。
3. 连接并检查三路相机与反馈，点XPolicy Prepare，等回零闭爪到位；重新摆放杯子/杯垫。
4. 夹爪空载、现场可监督且运动范围空闲时，勾选现场确认，点击一次`main(ctx)`。定位由新观测自动完成。
5. Web记录版本、各阶段、主Agent/Gemini、main及整轮耗时，轨迹样本进度。失败不自动重试/松爪/回零；先检查持物和停止结果。

数据源仍是该runtime的`runtime/scale-tests`与`commissioning`。成功后Stop不等于回Prepare；下一次须重新摆位和新观测。

## 5. 复制新工作副本与CLI复现

此安装器只适用于**已有匹配v0.6宿主**的机器，不升级v0.4宿主，不写Owner，不切换Web。它先校验宿主/原工作副本和冻结源，再复制本机memory/标定/证据及发布代码到新目录。

```bash
RAX_V06=/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blue-cup-trajectory-v06-20261111/runtime
DEMO="$PWD/demos/piper-blue-cup-v0.6"
PY=/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python
python3 "$DEMO/install_demo.py" --root "$RAX_V06"          # 仅检查，不写目录
python3 "$DEMO/install_demo.py" --root "$RAX_V06" --apply  # 仅创建新副本，已存在则拒绝
CANDIDATE="$RAX_V06/workspaces/blue-cup/v0.6-continuity-demo"
```

复制副本建立自己的Git commit，元数据`demo_original_code_commit`保留原身份，源字节不变。原Web继续指向原v0.6副本；不要并发使用Web和CLI控制硬件。

先原生Prepare并重新摆位，退出其他硬件会话。以下`--run`是真实接机入口，只有现场已准备且可监督时才执行。终端A：

```bash
RUN="$RAX_V06/commissioning/demo-v06-$(date +%Y%m%d-%H%M%S)"
printf '%s\n' "$RUN"  # 终端B使用本次目录
cd "$RAX_V06"
PYTHONPATH="$RAX_V06" "$PY" scripts/piper_explore.py \
  --output "$RUN" --rehearsal-candidate "$CANDIDATE" \
  --direct-model --http-proxy http://127.0.0.1:7892 --seconds 1800 --run
```

不要添加`--rehearsal-agent`。等待本次`REHEARSAL-READY.json`，确认`main_planning_model_started=false`。终端B使用相同`PY/CANDIDATE`以及终端A本轮`RUN`：

```bash
"$PY" "$CANDIDATE/run_no_agent.py" --ready "$RUN/REHEARSAL-READY.json"        # 只看计划
"$PY" "$CANDIDATE/run_no_agent.py" --ready "$RUN/REHEARSAL-READY.json" --run  # 执行一次
```

完成看`REHEARSAL-DONE.json`及`STOP-RESULT.json`；阶段报告在`rax-session/agent-workspace/output/rehearsal/merged-*/report.json`。CLI会保存原生报告，但不自动向Web表新增一行；需要Web记录就通过页面Run。

中断当前CLI轮次：

```bash
cd "$RAX_V06"
PYTHONPATH="$RAX_V06" "$PY" scripts/piper_explore.py --stop "$RUN"
```

不要用历史READY或中途轨迹索引续跑。停止后检查实际状态，重新开始须新会话和初始定位。

## 6. 在其他环境复用宿主改动

`host-overlay`包含对应源码，不是完整runtime、直接覆盖安装器或通用兼容补丁。先复制已经适配、可用的Piper RAX到**全新runtime**，保留旧实例，再对照这些文件接入现有接口：

- `piper_move_waypoints(start14, waypoints, obstacles, blend_indices)`：到唯一Owner队列，实际曲线预检与逐进度反馈。
- `piper_close_gripper(width_m)`：完整闭合的独立kind，不能以增大普通夹爪步长代替。
- `installed_settling.py`、`backend.py`：完成时间、证据复用、RGB默认/深度显式请求。
- `scripts/piper_motion_bridge.py`：桥接模块装载；仍需要现有legacy运动/碰撞模块，不是独立SDK入口。
- `scripts/piper_observe_only.py`：`OWNER=ROOT/'owner-runtime'`；其他现有Prepare/Run脚本通过此入口使用新Owner。
- `owner-runtime/real_world`必须是独立副本；其余依赖按原运行环境解析。`XPolicyLab`自别名指回新Owner；根`__init__.py`也是新副本。对[OWNER-PATCH.json](OWNER-PATCH.json)的基准哈希后，再在该新Owner应用补丁，不能修改共享旧Owner。
- Web使用本目录overlay三文件及独立端口；设备独占锁仍跨版本生效，不保证两个页面可以同时接机。

完整重现依赖现有SDK/固件配对、模型/标定和工具特性验收。新设备不能复制本机几何或容差后直接运行。改变后的整条曲线重新检查碰撞/工作区，仍未覆盖未知障碍、线缆和完整带载扫掠体。

## 7. 验证怎么复跑

发布包便携测试需要pytest、NumPy、SciPy、OpenCV，不联网、不接设备：

```bash
python3 -m pytest demos/piper-blue-cup-v0.6/tests -q
```

测试覆盖快照/历史身份、改变文件被拒绝、新副本安装不覆盖、主函数边界与失败退出、曲线非零速度及实际下发动态约束。宿主原139项测试及真实MoveIt/FCL离线对照在原5090可复跑：

```bash
cd /home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blue-cup-trajectory-v06-20261111
/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python verify_all.py
```

[host-validation](host-validation)保存关键宿主集成测试源码供适配参考。这些测试需要完整runtime、`recorded-target.json`、`source-plans.json`、`table.json`及原MoveIt/FCL/Owner依赖；本仓库未发布现场坐标fixture，不能在demo目录直接声称复跑原139项。发布包测试与原139项宿主验证分别报告。
