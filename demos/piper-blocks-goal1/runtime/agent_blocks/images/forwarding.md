Camera images are tool-result attachments, not MCP resources; you see them directly
in the tool result. Never print or stringify image bytes, call `resources/list` for
cameras, or resize inline images. Save frames you need again under `{{ vocab.frames }}`
and view them selectively with the Read tool rather than loading many at once.
