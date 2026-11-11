import os
"""Persistent Astra + FastWAM PiPER adapter; current dispatch mode is shadow only."""

MODEL = os.environ.get("JUNQI_CODEX_MODEL", "gpt-6-astra")
EFFORT = os.environ.get("JUNQI_CODEX_EFFORT", "xhigh")
CONDITIONING = "Pick up one drink container and place it on the coaster."
STUDENT_MAX = int(os.environ.get("PIPER_POLICY_PREFIX_STUDENT_MAX", "15"))
