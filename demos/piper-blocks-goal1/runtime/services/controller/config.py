"""Operator-owned session configuration. No values are accepted from agent code."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import re

AGENT_EFFORTS = ('medium', 'high', 'xhigh', 'max')
PROVIDERS = ('claude-login', 'openrouter', 'anthropic', 'codex-relay')
PROFILES = ('rgbd', 'rgb-only')


@dataclass(frozen=True)
class Configuration:
    root: Path                      # session directory under the artifact root
    task: str                       # short task id, e.g. "stack_cups"
    task_description: str = ''      # TASK.md body written by the operator
    robot: str = 'franka'           # services.backends registry key
    # Backend-specific operator settings (bridge URLs, cameras, workspace box, ...).
    backend_options: dict = field(default_factory=dict)
    observation_profile: str = 'rgbd'
    agent_provider: str = 'claude-login'
    agent_effort: str = 'xhigh'
    agent_model: str | None = None  # Provider default (Opus 5.5) when unset.
    agent_auto_compact_token_limit: int | None = None  # Native Codex CLI; None retains its default.
    session_seconds: int = 4 * 3600
    max_model_calls: int = 4000
    skill_library_root: str | None = None  # shared base, one repository per robot
    subagents: int = 0              # 0: the Agent tool is removed.

    def __post_init__(self):
        object.__setattr__(self, 'root', Path(self.root).resolve())
        from services.storage_root import ENVIRONMENT, artifact_root
        if not self.root.is_relative_to(artifact_root()):
            raise ValueError(f'Session artifacts must be under {artifact_root()} (set {ENVIRONMENT} to change it)')
        import os
        library = self.skill_library_root or os.environ.get('RAX_SKILL_LIBRARY') or str(artifact_root()/'skill-library')
        object.__setattr__(self, 'skill_library_root', str(Path(library).expanduser().resolve()))
        if not re.fullmatch(r'[A-Za-z0-9_-]+', self.task):
            raise ValueError('Invalid task id')
        if not isinstance(self.task_description, str):
            raise ValueError('task_description must be text')
        from services.backends import BACKENDS
        if self.robot not in BACKENDS:
            raise ValueError(f"Unknown robot; choose one of {', '.join(sorted(BACKENDS))}")
        if not isinstance(self.backend_options, dict):
            raise ValueError('backend_options must be an object')
        if self.observation_profile not in PROFILES:
            raise ValueError(f'observation_profile must be one of {PROFILES}')
        if self.agent_provider not in PROVIDERS:
            raise ValueError(f'agent_provider must be one of {PROVIDERS}')
        if self.agent_effort not in AGENT_EFFORTS:
            raise ValueError(f'agent_effort must be one of {AGENT_EFFORTS}')
        if self.agent_auto_compact_token_limit is not None and (
                type(self.agent_auto_compact_token_limit) is not int
                or self.agent_auto_compact_token_limit < 1):
            raise ValueError('agent_auto_compact_token_limit must be a positive integer or null')
        if type(self.session_seconds) is not int or not 60 <= self.session_seconds <= 7 * 24 * 3600:
            raise ValueError('session_seconds must be 60..604800')
        if type(self.max_model_calls) is not int or self.max_model_calls < 1:
            raise ValueError('max_model_calls must be positive')
        if type(self.subagents) is not int or not 0 <= self.subagents <= 8:
            raise ValueError('subagents must be 0..8')

    @property
    def camera_geometry(self):
        return self.observation_profile == 'rgbd'

    @classmethod
    def from_dict(cls, value):
        return cls(**dict(value))

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text()))

    def to_dict(self):
        return json.loads(json.dumps(asdict(self), default=str))
