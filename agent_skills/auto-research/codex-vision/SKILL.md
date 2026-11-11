---
name: codex-vision
description: Ask the host-brokered gpt-6-astra medium visual helper about supplied camera images when simple perception rules are insufficient.
---

# Auxiliary visual reasoning

`codex_vision_generate` replaces the optional Gemini helper for this deployment.
It uses `gpt-6-astra`, reasoning effort `medium`.
Credentials stay in the host service. This helper cannot command the robot.

Arguments:
- `prompt`: a precise visual question, including the image order, capture IDs and
  what is uncertain. Ask for observable evidence and ambiguity, not imagined geometry.
- `images`: a list of `{mimeType: "image/jpeg" | "image/png", data: "<base64>"}`.
  Pass original camera image blocks; do not print their bytes into model context.
- `max_output_tokens`: 256–16384, optional.

The result contains `text`, Responses `output` and provider `usage`. Usage is not
an estimate of dollar cost. The original Gemini dollar budget does not account for
this provider. No Gemini message/function-call schema is implied by this tool.

Use the helper for a focused question (object identity, relative alignment, visible
support or grasp stability). Ground metric positions with the robot's calibrated
pixel-to-position tool. Neither a confident answer nor a clean motion receipt proves
physical task success. Handle missing evidence explicitly.

For scripts, call `ctx.value("codex_vision_generate", prompt=..., images=...)`.
Validate the response before it influences motion. Re-observe after a pause or scene
change; an answer describes the images supplied to it, not the current world.
