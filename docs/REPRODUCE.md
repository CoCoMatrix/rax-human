# 复现：从源码到一次受监督运行

以下操作使用新克隆目录，不能覆盖当前任务目录。发布离线测试与真机验收是两件事。原设备可以复用已经安装和验证的依赖；新设备必须重新核对 CAN、序列号、几何、标定及工具特性。

## 1. 主机与路径

先按根 README 安装基础 Python 依赖。Linux 还需 `git`、`bubblewrap`、编译工具以及设备驱动。纯离线测试不需要机器人、API 密钥和 GPU。

原 5090：

```bash
cp site.5090.example.json site.local.json
export RAX_PYTHON=/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python
# 编辑 site.local.json 的 relay_base_url，确认所有路径仍有效
$RAX_PYTHON scripts/doctor.py --strict
PYTHONPATH=. $RAX_PYTHON -m pytest -q
```

其他机器使用 `site.example.json`。路径相对仓库根解析，`RAX_SITE_CONFIG` 可指定另一个 JSON。凭据文件不在仓库内；不要将 `site.local.json`、运行目录或 shell 登录配置提交。

当前导出依赖：

| 组件 | 准备方法 |
|---|---|
| XPolicy owner | `integrations/xpolicy_owner` 是当前设备 owner 源码子集；保留 `XPolicyLab -> .` 命名空间别名。独立 Python 环境安装 `requirements-hardware.txt` 中设备依赖，再验证模块导入。不要直接对原 pyproject 执行 `uv sync`，其中第三方 wheel 路径不是发布附件。 |
| Piper SDK | 当前 pyAgxArm 1.0.0；profile 记录所用 SDK 身份。通过厂商公开渠道安装匹配版本；硬件／固件配对须实测。 |
| FFS | [NVlabs/Fast-FoundationStereo](https://github.com/NVlabs/Fast-FoundationStereo)，当前提交 `476f4249561f7c79ca707326954f9255643412a6`；按上游安装独立 CUDA 环境。当前模型为 `20-30-48`，权重来源见下文。 |
| MoveIt | `integrations/moveit` 提供 C++ IK 服务、URDF/SRDF、CMake 和原 conda 显式环境清单；当前 ROS 2 Humble / MoveIt 2.5.10。源码编译后的二进制不随仓库发布。 |
| 隔离 Agent | 准备独立 Python 环境，安装基础 requirements；`codex` 可执行文件须在 PATH 中（沙箱入口使用 `shutil.which` 查找），按已有部署安装 Codex CLI；详见 `docs/CODEX_ADAPTER.md`。API 可达性要单独做真实调用验证。 |
| 几何／相机 | 当前设备资产位于 `services/piper/assets` 与 `integrations/legacy_piper/assets`。新机器需替换实测外参及内参，并重新验证资产一致性，不能仅改 TCP 数字。 |

FFS 权重上游目录：<https://drive.google.com/drive/folders/1WWT0I8VB1k7cA2LqnjhUHOux8Sz-BZMw>。当前 checkpoint SHA-256 为 `98b5a9acf39fbfa795025de8cea95ce123daa40f6b6234d719167751024cf692`，cfg SHA-256 为 `d45afe99b176454d5aff416edf16c8da6a99579f8f374b927f37907442a7d6bc`。权重许可遵循其来源。

`ffs_manifest` 按 `junqi.piper.ffs_model_manifest.v1` 格式配置，模板见 `examples/deployment/ffs-model-manifest.json`，包含来源、固定提交、路径和权重哈希；可参考 `integrations/policy_adapter/real_robot/piper/ffs_runtime.py` 的实际读取契约。`ffs_warmup_packet` 需要当前设备采集的有效双目包，发布包不包含历史相机帧。只读相机接入也会启动深度工作进程，需先准备这项依赖。

MoveIt 可在已安装的环境执行：

```bash
cmake -S integrations/moveit -B runtime/moveit/build -DCMAKE_PREFIX_PATH=/your/moveit/env
cmake --build runtime/moveit/build --target ik_server -j4
```

将 URDF 的 mesh URI 解析到本机资产，只改 URI 不改运动链；保留原始文件。参考 `services/piper/assets/moveit-runtime.json` 建立自己的 manifest，`root/env` 指向 MoveIt 环境（可用符号链接），`binary/urdf/srdf` 和 `files` 中路径、SHA-256 必须对应新产物。哈希只能证明身份，不代表几何已验收。原设备直接复用已有 manifest 更稳妥。

## 2. 模型与代理

当前入口使用主模型 `gpt-6-astra` / high，辅助视觉 Gemini `gemini-3.8-flash`。这记录当前部署配置，不保证其他账号可用这些模型；需要更换时同时修改入口和 launcher 的模型配置并验证 API。

主 API 凭据文件由 `harness/codex_cli/credentials.py` 读取，接受 Python 字面量字符串赋值，例如 `api_key = "..."`；实际密钥自行保存在配置指向的外部文件中，文件权限设为 0600。Gemini 文件使用单行原始 key（不是 Python 赋值），由 `services/controller/gemini.py:load_key` 读取；该文件必须归当前用户所有且权限 0600。不要把实际内容写进 README、日志或 Git。

`--direct-model` 表示主模型直连中转地址，辅助视觉仍使用 `vision_proxy`。空代理表示不显式指定代理。网络测试是独立的只读模型调用，不能用恢复连接来自动重放已经执行过的机器人动作。

## 3. 只读接入与 Prepare

```bash
PYTHONPATH=. $RAX_PYTHON scripts/piper_observe_only.py \
  --output commissioning/observe-001 --seconds 60
# 核对计划及设备占用后，在同一命令末尾加 --run
```

只读图没有 Prepare/Start/动作输入边，不使能电机。核对头部、左腕、右腕新图像、关节与夹爪反馈以及时间戳。设备被其他任务占用时入口会拒绝，不能绕过 owner。

`piper_prepare.py --output commissioning/prepare-001 --run` 启动原生 owner/UI，等待明确的 Prepare 命令；仅运行该脚本不等于完成回零。建议使用 Web 的 Prepare 按钮，它调用 XPolicy 原生回零闭爪并核对回执。Prepare 不经过探索模型；如果夹持物体，应按当前现场状态处理返回路径。

## 4. 新任务／逐函数演练

根 README 给出积木任务例子。新工作区由 `prepare_workspace.py` 创建，默认只打印计划，加 `--apply` 才导入。它会建立只读源码归档、可写副本、哈希清单和本地导入提交；**导入提交不是新一轮演练成功证明**。已有源记忆可导入，当前机器人状态不会继承。

探索使用任务 `session.json`；演练使用 `rehearsal-session.json`、`--rehearsal-candidate` 和 `--rehearsal-agent`。不带 `--rehearsal-agent` 的候选入口只提供 frontend，不启动主规划模型。

现场满足当次运行条件以后才加 `--run`。监督事件、阶段耗时、模型调用、动作回执和连续录像。需要停止时：

```bash
PYTHONPATH=. $RAX_PYTHON scripts/piper_explore.py --stop commissioning/本轮目录
```

明确停止不会被自动解释为“继续收尾并再启动设备”。不要同时运行两个 owner。

## 5. v0.4 无主 Agent 整段执行

先用 README 中命令导入 v0.4。两个终端分别运行：

```bash
# 终端一：在现场准备完成后启动受限 frontend
PYTHONPATH=. $RAX_PYTHON scripts/piper_explore.py \
  --rehearsal-candidate workspaces/blue-cup/20261109-v0.4-merged-candidate \
  --output commissioning/cup-main-001 --seconds 1800 --direct-model --run
```

等待生成本轮 `REHEARSAL-READY.json`，然后：

```bash
# 终端二：先去掉 --run 看计划；准备好后才执行
$RAX_PYTHON workspaces/blue-cup/20261109-v0.4-merged-candidate/run_no_agent.py \
  --ready commissioning/cup-main-001/REHEARSAL-READY.json \
  --python-runtime /your/isolated-agent-python-env --run
```

这会在沙箱中执行 `controller.py --run`，进入 `main(ctx)`；不启动主 Agent。复用过期 READY、并行运行 Agent 或清掉不确定的执行锁均不可作为恢复方法。异常后先核对当前状态，再决定恢复入口。

## 6. Web 与实验记录

```bash
PYTHONPATH=. $RAX_PYTHON -m services.dashboard.piper_scale --port 8781
# Mac 另开终端；替换服务器地址
ssh -N -L 8781:127.0.0.1:8781 ubuntu@YOUR-SERVER
```

服务只监听 loopback。在服务器本地查看 `runtime/scale-tests/access-token`，浏览器打开 `http://127.0.0.1:8781/?token=本次令牌`，不要公开该地址或令牌。启动 Web 本身不接入设备。依次只读连接、核对现场、Prepare、重新摆位、勾选当次确认、执行一次任务；任务坐标由初始感知自动获得。

默认候选目录为 `workspaces/blue-cup/20261109-v0.4-merged-candidate`。Web 检查可写代码 Git 提交与 WORKSPACE 元数据一致；改过代码需先验证、提交并同步元数据。每条实验保存版本／commit、阶段时间、辅助视觉次数、总耗时、成功或失败原因。记录目录不上传 Git。

`web-console` 是先前独立打包工具，适合原部署；新克隆优先使用这里的根服务，避免覆盖成旧 overlay。
