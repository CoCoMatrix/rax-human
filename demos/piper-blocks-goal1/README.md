# 已调通版本：四色积木 goal1（目标图 + Coal 预筛）

这是 5090 上积木任务 **goal1** 版本的完整快照，即 18797 端口上的 Web（systemd 用户单元 `rax-goal1-trial-web`）当前运行的代码。

- **基线：** 冻结版 `gap68-start1`（控制器提交 `5e6ee54`）。goal1 在它上面增加了：
  - 目标照片识别；
  - 按目标描述规划和执行；
  - 规划器向量化；
  - Coal 碰撞预筛。
- **版本名：** `r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1`
- **控制器提交：** `da2352a170232b7fb910ac60962b88b755950ae6`（阶段 5，`native_collision_prefilter: coal-3.0.3`）
- **原目录：** `/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1-web-20261119`

代码逐字保留。32 个封版主机文件和 102 个控制器文件的 SHA-256 与 [`WEB-VALIDATION.json`](WEB-VALIDATION.json) 一致。因为保持哈希不变，代码里写死的 5090 路径都没有改（见第 4 节）。

## 1. 功能和验证范围

| 内容 | 状态 |
|---|---|
| 默认目标（红橙并排、绿叠红、黄叠绿） | 与 start1 一致，记录回放结果相同 |
| 自定义目标：Web 上传照片 → Gemini 3.8 Flash 读 3 次比对 → 固定格式目标描述 `rax.blocks.goal.v1` | 每轮把确认的目标写成 `GOAL.json`，随代码进入会话，控制器启动前核对 |
| 自定义目标的规划与执行 | **只做了离线验证**（全任务预检和模拟执行），真机验证待做 |
| 规划器 `_check` 向量化 | 证据见 [`VEC-EVIDENCE.json`](VEC-EVIDENCE.json) |
| Coal 3.0.3 碰撞预筛 | 见下文 |
| Prepare | 不再是运行前提；已连接时按实测零位和闭爪检查 |

**Coal 3.0.3 碰撞预筛：** 只跳过肯定比安全距离更远的部件对，可能接触的部件对仍用 FCL 精确计算。

- 346 次记录预览的决策全部一致；
- 原生检查从 178 s 降到 74 s；
- 详见 [`COAL-EVIDENCE.json`](COAL-EVIDENCE.json)；
- 设 `RAX_COAL_PREFILTER=0`，或 vendor 文件和清单不符时，自动退回原 FCL 检查。

**已知问题（未修）：** 规划按爪宽 73 mm 检查，实际张开约 72.4 mm。手指余量接近 0 的规划会在执行时、下发前被拒绝。

## 2. 在 5090 上启动

下面的命令都在 5090 上执行，原目录已经是封版状态，不需要还原。

```bash
V=/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1-web-20261119
PY=/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python
```

**1. 确认 Piper 设备空闲。** 设备被占用时，Web 的"连接"会被拒绝。同机另一个项目的 Astribot 会话（`xpolicylab-astribot-desktop.service`）也会被判为占用，需要先停掉。

```bash
cd $V/runtime && $PY -c "import sys; sys.path.insert(0,'scripts'); from piper_observe_only import no_existing_devices; no_existing_devices(); print('Piper devices free')"
```

**2. 启动 Web。** 只监听本机回环地址，启动时不连接硬件。

```bash
systemd-run --user --collect --unit rax-goal1-trial-web --working-directory=$V/runtime --setenv=PYTHONPATH=$V/runtime --setenv=PYTHONDONTWRITEBYTECODE=1 $PY -m services.dashboard.blocks_scale --port 18797
```

```bash
systemctl --user is-active rax-goal1-trial-web
```

日志里应有 `listening on loopback:18797; no hardware started`：

```bash
journalctl --user -u rax-goal1-trial-web -n 5 --no-pager
```

同一时间只能有一个控制台占用机械臂。start1 的控制台在 8797 端口（`rax-gap68-start1-web`），不要两个同时连接。

**3. 从 Mac 打开页面。** 先建隧道：

```bash
ssh -fN -L 127.0.0.1:18797:127.0.0.1:18797 ubuntu@<5090 地址>
```

再读取访问令牌（只在第一次启动时生成，之后不变）：

```bash
ssh ubuntu@<5090 地址> cat /home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1-web-20261119/runtime/runtime/blocks-scale-tests/access-token
```

浏览器打开 `http://127.0.0.1:18797/?token=<令牌>`。打开后令牌写入 Cookie，之后直接访问 `http://127.0.0.1:18797/` 即可。

**4. 页面操作顺序。** 除了设置目标，其余按钮都会操作硬件，需要有人在现场。

1. 桌面上只放四块积木。上一个任务留下的物体（例如白盒）要先拿走：机械臂停在低位时直接回零，可能会扫到物体。
2. 设置目标（可选）。上传目标照片，点 **用 Gemini 解析**（同一张照片读 3 次比对，不一致时页面让你选或重拍），核对后点 **确认为本轮目标**。不设置时用默认目标；点 **恢复默认目标** 可以撤回。
3. 点 **连接 / 重新连接硬件**。
4. 点 **回到 Prepare**，走 XPolicy 原生 Prepare，机械臂会动。运行要求机械臂在零位、夹爪闭合；已经满足时可以跳过这一步。
5. 点 **执行一次任务**。系统先核对代码和封版哈希，再做全任务预检，然后执行 `assembly_controller.main(ctx)`。不启动主 Agent。
6. 需要中止时点 **停止本轮**。每轮的阶段、耗时和失败原因显示在页面下方，点 **查看更多记录** 可看更早的轮次。
7. 控制台重启或连接中断后，点 **恢复控制台连接**。

**5. 停止 Web。** 先在页面上结束本轮，再执行：

```bash
systemctl --user stop rax-goal1-trial-web
```

## 3. 在别的目录或机器上还原

仓库里没有放这些文件：

- 两个二进制依赖：SSIK 的 194 MB `.so` 超过 GitHub 单文件上限，Coal 的二进制约 117 MB；
- 录制的相机帧：约 740 MB。

[`restore_goal1.py`](restore_goal1.py) 会把它们补回，并重新核对所有封版：

```bash
python3 demos/piper-blocks-goal1/restore_goal1.py --target <新版本目录> --python <Python 3.12，可用 pip> --proxy http://127.0.0.1:7892 --private-from <原 goal1 版本目录>
```

脚本按顺序做这些事：

1. 复制 `runtime/`、证据 JSON 和 `goal1-build/`。
2. 从 PyPI 安装 `coal==3.0.3` 及其依赖、`ssik==8.2.0`、`urchin==0.0.31`，只取仓库缺的二进制。然后按 `coal_vendor_manifest.json`（162 个文件）和 `ssik_vendor_manifest.json`（283 个文件）逐个核对哈希。
3. 从 `--private-from` 复制相机帧：
   - 冻结版 r3 的 `runtime/frames`，按 `SHA256SUMS.json` 核对；
   - 控制器记忆引用的帧，用 Web 自己的检查 `copy_memory_evidence`。
4. 用 [`magnetic_blocks.bundle`](magnetic_blocks.bundle) 重建控制器 git 仓库，确认 HEAD 是 `da2352a` 且工作区干净（Web 每次运行前都检查这一点）。
5. 把 `WORKSPACE.json` 和 `VERIFICATION.json` 里的原目录路径改成新目录。
6. 核对 `WEB-VALIDATION.json` 中 32 个主机文件和 102 个控制器文件的哈希。

不带 `--private-from` 时，代码和依赖照常还原，但 Web 在运行前的来源检查会因为缺帧拒绝执行。还原后，把第 2 节命令里的 `$V` 换成新目录即可启动。目标目录必须为空，不要还原到正在运行的版本上。

## 4. 代码里写死的外部依赖

以下路径写在封版文件里。换机器时，要在相同路径准备好它们（或建同名链接）。改代码会破坏封版，需要重新验证并重新封版。

| 用途 | 路径 |
|---|---|
| Web 和主机进程的 Python | `/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python`（也提供旧版 FFS worker 等模块） |
| XPolicy owner 环境 | `runtime/owner-runtime/.xpolicy/envs/piper_x_dual`、`runtime/owner-runtime/runtime/robot_dagger/.venv`（含 `dora`），仓库里没有，需要单独安装 |
| FFS 深度 | `/home/ubuntu/code_junqi/GPT-as-Policy/.venv-ffs5090`、`/home/ubuntu/code_junqi/migration/ffs-live-20260918`（标定和模型清单） |
| 主模型密钥 | `/home/ubuntu/code_junqi/key.py`（不在仓库里） |
| Gemini 密钥 | `/home/ubuntu/code_junqi/gemini-key.py`（不在仓库里） |
| 外网代理 | `http://127.0.0.1:7892`（Gemini 和 PyPI） |
| Agent 沙箱 | `/home/ubuntu/code_junqi/rax-piper-agent-isolated` |
| 设备与标定 | 原实验的双臂 Piper X、头部双目、双腕相机；标定只对应这套设备 |

## 5. 本次发布做过的检查

在 5090 上，把这个文件夹按第 3 节还原到临时目录 `/tmp/goal1-pubtest/target`，没有启动硬件：

- 还原脚本全部通过：
  - Coal 补回 103 个文件、核对 162 个；
  - SSIK 补回 3 个、核对 283 个；
  - 冻结版 73 个文件、记忆引用 350 个文件；
  - 控制器 HEAD `da2352a` 干净；
  - 封版哈希一致。
- 在还原目录里调用 Web 的 `Manager().validate_run_source()`，通过；`load_coal()` 从还原的 vendor 加载成功；SSIK 启动自检（清单 + FK 比对）通过。
- Web 在临时端口启动后：不带令牌返回 401，带令牌返回 303；启动时没有连接硬件。
- `goal1-build/test_goal.py`：27 passed，和封版时的 `TESTS.log` 相同。
- 控制器里的 goal1 测试：
  - `test_goal_structure.py` 9 passed；
  - `test_goal_execution.py` 3 passed；
  - `test_native_budget.py` 3 passed。
- 控制器目录下的全部历史测试，一次性运行的结果是 385 passed、194 failed、13 errors。原目录用同一命令得到完全相同的结果，失败的是早期版本留下、goal1 没有维护的测试，不是打包造成的。`test_speed_scope.py` 依赖主工程测试目录，单独运行也无法导入，两处相同。

这些只是离线检查，不是新的真机成功记录。

## 6. 目录内容

| 路径 | 内容 |
|---|---|
| `runtime/services`, `runtime/scripts`, `runtime/harness`, … | 主机代码，目录结构与原版本相同（`ROOT` 相对路径不变）。其中 `services/dashboard/blocks_scale.py` 和 `static/blocks/` 是 goal1 Web |
| `runtime/workspaces/magnetic-blocks-l-stack/r9-ssik-speed` | Web 使用的可写工作副本：控制器代码 `code/magnetic_blocks`（入口 `assembly_controller.main`）、记忆、标定、参考资料、`WORKSPACE.json` |
| `runtime/versions/magnetic-blocks-l-stack/20261111-r3-v0.1.0` | 工作副本的冻结来源（不含录制帧） |
| `runtime/owner-runtime` | XPolicy owner 源码（原为链接，已展开）；不含虚拟环境和运行输出 |
| `runtime/vendor/coal-3.0.3` | Coal 许可证和元数据；二进制由还原脚本补回 |
| `goal1-build/` | Web 补丁（`patch_server.py`、`patch_ui.py`、`goal_ui.js`）、测试、封版脚本 `seal*.py`，以及阶段 3/4 和 Coal 的离线工具 |
| `*-EVIDENCE.json`, `WEB-VALIDATION.json`, `DEPLOY-VERIFIED.json` | 各阶段证据和封版记录 |
| `magnetic_blocks.bundle`, `CONTROLLER_COMMIT` | 控制器 git 历史和封版提交 |
| `restore_goal1.py` | 还原和核对脚本 |
