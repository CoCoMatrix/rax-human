# Gemini 辅助视觉接口与网络

这是 **2026-11-10 已部署的 Piper RAX 接口**，不是照抄 Google SDK 的调用方式。优先复用工作区 `perception.visual` / `checks.assess`；本文示例用于理解和迁移接口，不额外增加必经检查。

## 调用链与主模型的区别

```text
主 Agent / 普通 Python 函数
  → 已有 Context.value("gemini_generate", ...)
  → RAX frontend：参数、预算、有限传输重试
  → GoogleGeminiRouter：宿主密钥和代理
  → Google OpenAI-compatible chat/completions
```

| 项目 | 当前值 |
|---|---|
| MCP 工具 | `gemini_generate` |
| 返回的规范模型名 | `google/gemini-3.8-flash` |
| 宿主实际 provider/model | `google` / `gemini-3.8-flash` |
| Endpoint | `https://generativelanguage.googleapis.com/v1beta/openai/chat/completions` |
| 会话接入 | `api.runtime.Context`，通过原生 `RAX_MCP_SOCKET`，函数沿用传入的 ctx |
| 辅助视觉路由 | 当前 XQNetwork HTTP 代理 `http://127.0.0.1:7892`，不是旧 17890 路由 |
| 主模型路由 | 主模型 API 中转站直连；`--direct-model` 不表示 Gemini 也直连 |
| 凭据 | 宿主读取配置；原机 Gemini 配置路径 `/home/ubuntu/code_junqi/gemini-key.py`，主模型 `/home/ubuntu/code_junqi/key.py` |

只记录配置路径，不把文件内容复制进技能、工作区、日志或 GitHub。调用者不直接创建外网客户端，不自行传 key/model/provider/endpoint。模型更换由宿主配置与兼容性验证处理。

## 实际参数契约

- `messages` 为 OpenAI-compatible 消息数组，1–64 条；不是 Google `contents` / `generationConfig`。
- 图像使用 MCP content block：`{"type":"image","mimeType":"image/jpeg","data":"<base64>"}`，与 text block 交替组成一条 user message。宿主也接受其协议允许的 data URL；不传服务器文件路径让 provider 自行读取，不抓取任意远程图片。
- 当前辅助 helper 使用 `max_tokens=4096`、`reasoning={"effort":"low"}`、`response_format={"type":"json_object"}`。effort 仅 low/medium/high；**不传 temperature**。
- `max_tokens` 允许 1–16384，预算还要考虑 thinking token；调高不是修复反复调用的方法。
- 当前边界：最多 8 张图片，单张 4 MiB、总图像 8 MiB、单张像素总数不超过 `4096 × 4096`；请求上限 12 MiB，非图像文本/结构内容另有上限。重用实际 tool schema，不硬套旧 OpenRouter 注释。
- 每张图标注 current/before、camera、frame ID 和**实际编码尺寸**。历史比较只上传所需图，不固定给三路完整历史。普通定位使用当前像素；试抬 helper 可将前后图等比例缩到同样宽度，坐标不得二次缩放。

## 最小只读调用示例

下面只是一个辅助视觉函数；不包含硬件连接、运动、自动重试或额外 Context。`frames` 应由调用方从现有观测生成，`path` 指向实际 JPEG，`width/height` 是该 JPEG 的真实尺寸；时序、姿态和目标绑定仍由现有控制函数检查。生产控制优先使用已有 `checks.assess`，不要在每个阶段插入这个例子。

```python
import base64
import json
from pathlib import Path


def ask_visible_facts(ctx, question, frames, fact_names):
    """只读辅助示例；fact_names 是确实需要图片判断的可见事实。"""
    if not 1 <= len(frames) <= 8 or not fact_names:
        raise ValueError("Supply relevant frames and named visible facts")
    if len(set(fact_names)) != len(fact_names):
        raise ValueError("Duplicate fact names")
    if any(f["phase"] not in ("current", "before") for f in frames):
        raise ValueError("Unknown frame phase")
    if not any(f["phase"] == "current" for f in frames):
        raise ValueError("A current observation is required")
    camera_ids = sorted({f["camera"] for f in frames})
    prompt = (
        "Judge only the requested visible facts qualitatively across the supplied views. "
        "Partial occlusion or projection overlap alone is not physical contact. "
        "Do not infer hidden millimetre gaps, the full swept path, or absence of transient contact. "
        "Use unknown when combined evidence is insufficient; report clear contradictions. "
        "Return JSON with facts: each named fact has state (supported|contradicted|unknown), "
        "evidence (string), cameras (list of supplied camera IDs); "
        "and visible_hazards (list of concrete relevant hazards). "
        + question + " Fact names: " + json.dumps(fact_names)
        + " Camera IDs: " + json.dumps(camera_ids)
    )
    content = [{"type": "text", "text": prompt}]
    for frame in sorted(frames, key=lambda f: f["phase"] != "current"):
        label = (f'{frame["phase"]} camera={frame["camera"]} '
                 f'frame={frame["frame_id"]} '
                 f'encoded_size={frame["width"]}x{frame["height"]}')
        content.append({"type": "text", "text": "NEXT IMAGE: " + label})
        content.append({"type": "image", "mimeType": "image/jpeg",
                        "data": base64.b64encode(Path(frame["path"]).read_bytes()).decode()})
    raw = ctx.value("gemini_generate", messages=[{"role": "user", "content": content}],
                    max_tokens=4096, reasoning={"effort": "low"},
                    response_format={"type": "json_object"})
    message = raw.get("message", {})
    if (raw.get("finish_reason") != "stop" or message.get("refusal")
            or not isinstance(message.get("content"), str)):
        raise ValueError("Incomplete or refused visual response")
    result = json.loads(message["content"])
    if not isinstance(result, dict) or not isinstance(result.get("facts"), dict):
        raise ValueError("Invalid facts object")
    if not isinstance(result.get("visible_hazards"), list):
        raise ValueError("Missing hazard result")
    for name in fact_names:
        fact = result["facts"].get(name)
        if (not isinstance(fact, dict)
                or fact.get("state") not in ("supported", "contradicted", "unknown")
                or not isinstance(fact.get("evidence"), str)
                or not fact["evidence"].strip()
                or not isinstance(fact.get("cameras"), list)
                or not fact["cameras"]
                or any(c not in camera_ids for c in fact["cameras"])):
            raise ValueError("Missing or invalid evidence for " + name)
    result["ok"] = (all(result["facts"][n]["state"] == "supported" for n in fact_names)
                    and not result["visible_hazards"])
    return {"assessment": result, "visual_calls": 1,
            "provider_request_attempts": raw.get("provider_request_attempts"),
            "duration_s": raw.get("duration_s"), "usage": raw.get("usage"),
            "budget": raw.get("budget")}
```

该示例返回的是视觉证据，不是运动许可。`ok=false` 时先看 unknown/contradicted 及既有几何/人工事实如何补充；不能把模型一句 ok 覆盖缺失事实。示例没有执行模型返回的 tool calls。

## 返回值与统计口径

宿主返回 `message`、`finish_reason`、`model`、`provider`、`provider_model`、`usage`、`budget`、`duration_s`，以及 `provider_request_attempts`、`transport_reconnects`。先验证 finish_reason、拒绝、非空内容和 JSON，再验证业务字段；截断、格式错误或缺失事实不能算成功。

`cost_usd` 为空，因为 provider 未提供实际美元账单；`cost_upper_bound_usd` 是宿主保守估计。会话美元/令牌预算是本地约束，不等于账号剩余额度。失败请求可能已计费，因此保留其预算预留；当前价格边界在代码中有 2026-12-31 到期检查，迁移到之后不能直接删掉检查继续宣称计费准确。

一次逻辑 `gemini_generate` 可能有两次 provider 尝试。分别记逻辑调用数、provider 尝试数和失败/重连次数；不能以一次逻辑调用掩盖重试成本。多轮使用时保留完整 assistant message 的必要扩展字段/thought signatures；本任务通常采用有界、独立图像问题，不堆积全部历史。

## 连接策略与诊断

当前 `_google_http` 每次请求新建 TLS，`Connection: close`，避免长时间动作/模型思考后复用已被代理关闭的连接。发送 HTTP 前最多 3 次连接尝试，连接超时 10 s；已建立连接的读取超时 60 s。证书错误不按普通瞬态错误重试。

外层 `frontend.gemini_call` 对 `SSLEOFError`、`RemoteDisconnected`、`ConnectionResetError` 允许最多 2 次 provider 尝试，每次重新建连、单独记预算，并检查会话收尾预算。认证、配额、证书或普通 HTTP 错误不是这项重试的适用范围。底层旧注释仍写“POST 不重放”，准确行为要结合外层看：**允许有界重试的是只读图像请求，绝不重放机器人动作。**

Gemini TLS 不是动作授权心跳。普通数值移动没有调用 Gemini 时，不应因为不存在的空闲视觉连接而停止；必要视觉边界请求仍失败时，记录未解决事实并返回，不能伪造成功。

排查顺序：

1. 从当前会话配置核对主模型与 Gemini 各自 route、代理端口和 endpoint；不要只看 shell 的代理变量或 `--direct-model`。
2. 看 `gemini_transport` 的 connect/send/headers/read 阶段与耗时、状态码和错误类型，再区分建连慢、上游等候、响应读取或语义判断问题。
3. 经授权需要真实网络验证时，使用同一接口和一张相关真实图像；记录模型身份、usage、耗时和重试次数。普通 TCP 探针不能证明图像请求成功。
4. 对短断连做上述有界请求重试；连续失败则保留原错误。不要反复启动新的运动会话来测试网络。

传输日志不打印密钥、提示词或图像 bytes。原始 MCP 审计可能含图像和场景信息，发布经验时仅取必要脱敏统计；本 skill 没有上传这些原始日志。
