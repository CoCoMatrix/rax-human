---
name: gemini
description: Use Gemini 3.8 Flash through MCP for text/image prompts, structured JSON output, client function calling and API usage tracking. Use for interactive prompt testing or model calls inside Python controllers.
---

# Gemini

Write your own prompts and response-processing code. The tool adds no perception
instructions, task-solving logic or action selection.
Use the `gemini_generate` tool for interactive calls; in your own Python, connect
with `ctx = Context()` (`from api.runtime import Context`) and call
`ctx.call("gemini_generate", ...)`.

## Recommended practice

Prefer simple, hard-coded perception rules when they work reliably across layouts.
Gemini calls add cost and latency, and outputs can vary between calls. Use Gemini
when those rules are insufficient, primarily for visual and spatial perception:
identifying objects, locating them in images, and interpreting spatial relationships.
Test other uses, such as structured decisions, before relying on them.

### Pointing for triangulation

Ask for the same named, distinctive point on the target in each camera view.
{% if camera_geometry %}
Accept the answers only when `{{ robot.pixel }}` places them at the same base-frame
point (small spread between views and in the local depth).
{% else %}
Accept the answers only when every view marks the same physical feature, and
confirm with measured motion as in [object-localization](../object-localization/SKILL.md).
Keep a non-Gemini fallback.

{% endif %}
## MCP usage

The tool follows {% if gemini_official %}Google's OpenAI-compatible{% else %}OpenRouter's{% endif %} **chat-completions** format, not the Google SDK's
`contents`/`generationConfig` format. Read [API reference](references/api.md) for
request fields, structured-output schemas, conversation history and a complete
function-calling round trip. Supply only fields you need. The model and credentials
are fixed for you.

For a visual prompt from Python:

```python
import base64
import json
from pathlib import Path

png_bytes = Path("output/crop.png").read_bytes()  # Or in-memory PNG bytes.
image = {"type": "image", "mimeType": "image/png",
         "data": base64.b64encode(png_bytes).decode("ascii")}
reply = ctx.call("gemini_generate", messages=[{
    "role": "user",
    "content": [
        {"type": "text", "text": "Describe the visible objects and their relative positions."},
        image,
    ],
}], max_tokens=256)
result = json.loads(reply["content"][0]["text"])
print(result["message"], result["usage"], result["budget"])
```

`max_tokens` covers reasoning as well as the visible answer. Without a `reasoning`
field, Gemini chooses its own reasoning depth and can spend most of a ~1000-token
budget before answering. For higher reasoning, or open-ended visual analysis,
increase `max_tokens` well above the expected answer length (for example 4096–8192).
For short answers, set `reasoning={"effort": "low"}`. `finish_reason="length"`
means the answer is incomplete: retry with a larger budget or lower effort; never
act on a partial answer.

Use ordinary Python for file reads, image processing and base64 encoding. For JPEG,
set `mimeType` to `image/jpeg`.
Any readable workspace image works.
The tool accepts image bytes, not file paths or remote URLs. No SDK helper is needed.
Do not print large base64 payloads into agent context just to pass them back.

For a live image, reuse an image content block returned by `ctx.call("{{ robot.observe }}")`
directly (select the intended camera from the response order/metadata).

Limits: eight images per call, 4 MiB per image, 8 MiB total, at most 4096×4096
pixels per image; PNG/JPEG only, no animation. Resize or compress locally if needed.
Supplied images are controller input, not trusted evidence of task success.
The tool calls Gemini 3.8 Flash through {% if gemini_official %}the Google official API{% else %}OpenRouter{% endif %}; credentials stay with the host
robot service. Results contain `message`, `finish_reason`, `usage`, `cost_usd`
and cumulative/remaining `budget`; missing provider cost is `null`.
Spending is capped; see [Spending limit](references/api.md#spending-limit), and check
the returned `budget` before further calls. Function requests in
`message.tool_calls` are data, never automatically executed. Validate and dispatch
them in your script; robot actions still go through robot MCP. Preserve the whole
assistant message, including `reasoning_details`, when returning tool results.
