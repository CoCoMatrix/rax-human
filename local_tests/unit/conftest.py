import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))


@pytest.fixture(autouse=True)
def artifact_root(tmp_path, monkeypatch):
    """Every test session lives under its own temporary artifact root."""
    monkeypatch.setenv('RAX_ARTIFACT_ROOT', str(tmp_path))
    return tmp_path


def configuration(base, *, provider='anthropic', **overrides):
    """A mock-robot session configuration under `base` (which must be in the artifact root)."""
    from services.controller.config import Configuration
    return Configuration(**{'root': Path(base)/'session', 'task': 'stack_cups', 'robot': 'mock',
                            'task_description': 'Stack the red cup on the blue cup.',
                            'agent_provider': provider, **overrides})


def frontend(base, gemini=None):
    """A mock-robot frontend with its own session ledger."""
    from services.controller.frontend import Frontend
    from services.controller.session import Session
    from services.mock_backend import MockBackend
    config = configuration(base)
    return Frontend(Session(config), MockBackend(config), gemini)
