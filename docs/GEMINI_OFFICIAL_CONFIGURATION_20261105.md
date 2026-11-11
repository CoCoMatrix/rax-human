# Gemini 官方辅助视觉接入 — 2026-11-05

已配置到 5090 的当前 RAX 工作工程。主代理继续使用现有 API 的 `gpt-6-astra / high`；
辅助视觉使用 Google 官方 `gemini-3.8-flash`，不切换其他型号。
本次仅进行离线软件检查及保存图像的真实模型请求，没有启动相机或硬件 owner，没有机械臂动作。

## 与 RAX Franka 的对应关系

[Franka 的原生 Gemini 实现](https://github.com/robot-agent-exploration/rax/blob/real/franka/services/controller/gemini.py)
使用 OpenRouter 的 `google/gemini-3.8-flash`。
本工程保留 `gemini_generate`、`api.runtime.Context`、主线程串行 MCP、消息/图片格式、
完整 assistant 消息、函数调用和每会话 $10 预算。只在 host 端增加 Google 官方传输适配。
探索、逐函数演练、组合 main(ctx)、任务记忆及跨会话技能库流程不变。

- 主模型密钥：`/home/ubuntu/code_junqi/key.py`，现有中转配置不变。
- Gemini 密钥：`/home/ubuntu/code_junqi/gemini-key.py`，作为私有 token 读取，不执行该文件。
- 官方接口：`https://generativelanguage.googleapis.com/v1beta/openai/chat/completions`。
- 代理：`http://127.0.0.1:17890`。
- 真实返回模型必须为 `gemini-3.8-flash`；MCP 保持 RAX 的标准名 `google/gemini-3.8-flash`，
  另返回 `provider=google`、`provider_model`。
- 工作副本使用 `reasoning.effort=low`、`max_tokens=4096`、JSON 输出。
  允许 low/medium/high，拒绝 temperature 和不支持的推理选项。
- 不自动重试、不自动替换模型。密钥仅由 host 读取，不进入 agent 工作区或命令行。
- Google 的 `extra_content` / thought signatures 在后续消息中原样保留。
- 文字/JSON 判断只接受 finish_reason=stop、非空内容且无 refusal；截断输出不产生动作判断。

Google API 行为依据：[模型说明](https://aistudio.google.com/docs/models/gemini-3.8-flash)、
[OpenAI 兼容接口](https://ai.google.dev/gemini-api/docs/openai)。

## 预算适配

Google 返回 token 用量，不返回美元账单。成功请求按当前标准单价保守估算，
另记 `session_cost_estimated_upper_bound_usd`；`cost_usd=null`，不伪称已确认费用。
估算包含 thinking：按 total_tokens − prompt_tokens 计输出，不能只用 completion_tokens。
不使用缓存折扣。失败或用量无效保留原预算预留；超出预留后停止后续 Gemini 请求。

[官方价格](https://aistudio.google.com/docs/pricing#gemini-3.8-flash)在 2026-12-31 前为
输入 $0.75/M、输出 $3.75/M，届时需复核 host 价格后继续使用。
保留 RAX 原来的保守图片预留规则。四张完整 1280×720 图可能超出单次 $10 预留，
因此只把“之前”图缩至宽 640；当前图保持原分辨率和像素坐标。
图片标签明确给出实际供模型读取的尺寸。

## 真实接口验证

证据目录：`verification/gemini-20261105/`，在 5090 工程内。

| 测试 | 结果 | 时间 |
|---|---|---|
| 官方模型列表 | 确认目标模型存在 | 0.89 s |
| 直接官方调用，保存的头部＋左腕图 | 完整文本/JSON，模型身份正确 | 5.22 s |
| RAX MCP → 工作副本 visual，头部＋左腕 | 杯子直立、左爪空载的具名事实通过 | 4.69 s |
| RAX MCP → visual，之前/当前两组头部＋左腕图 | 四图请求及具名事实通过 | 8.97 s |
| RAX MCP 客户端函数调用→函数结果续接 | add(2,3) 返回5，完整消息续接成功 | 见 mcp-acceptance.json |

两组时间比较使用相同的保存帧，验证传输、分辨率标签及代码路径；
不等于真机时间连续性或新现场视觉验收。样本不足以给出稳定延迟/P95。
MCP 会话四次请求共8121 tokens，费用保守估算 $0.00753375，
未决预留归零，后台/session 正常退出。主模型本次未重新发起请求。

原来隐藏爪底、两帧之间有无瞬时接触的限制继续保留：
图片只判请求的可见事实，几何、路径和到位分别检查；更换模型不扩大视觉能证明的范围。
J4 / 3mm 位姿问题和工具表中其余未验收项均不因此变成通过。

## 新会话与工作副本

`scripts/piper_explore.py` 默认读取上述 Gemini 密钥；演练/探索新会话生成
Gemini 技能及 `gemini_generate` 契约。通用启动器需明确传
`--gemini-key-file /home/ubuntu/code_junqi/gemini-key.py`，README 示例已更新。
既有历史会话保留原配置，不自动恢复或迁移。

已更新可写的 `workspaces/blue-cup/20261104-v0.3-r12-working/code/blue_cup/perception.py`
及辅助视觉说明。冻结 v0.1、v0.3 不改动。下次演练仍需新鲜三相机观测和现场条件，
不能把本次保存图片当成当前状态。

## 最终软件验证

5090 全部单元测试：393 passed，25.96s；60 条来自既有 yourdfpy 的弃用提示。
MCP 进程重启后正确暴露 Google 支持的参数集合，累计预算保持不变，退出清理成功。
扫描95个工作区、配置及测试日志文件，未发现 Gemini 密钥内容。
冻结 v0.1 的60项、v0.3 的109项文件校验全部一致，manifest 哈希也与原记录一致。
详细机器可读证据：`verification/gemini-20261105/final-verification.json`。

