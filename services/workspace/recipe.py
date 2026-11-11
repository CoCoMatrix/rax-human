"""A recipe says which sources make one kind of workspace (a role)."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

PROJECT = Path(__file__).resolve().parents[2]
SKILL_SOURCE = PROJECT / 'agent_skills'


@dataclass(frozen=True)
class Vocab:
    """Named words a shared source uses instead of spelling out paths and tool names."""
    memory: str             # memory folder
    notes: str              # the agent's running notes file
    frames: str             # where saved camera frames live
    inventory: str          # session budget/status tool


@dataclass(frozen=True)
class SkillRule:
    """Leave a skill out when `when(setup)` holds."""
    skill: str
    when: Callable


@dataclass(frozen=True)
class Recipe:
    name: str
    template: Path                      # document sources, copied then rendered
    # agent_skills/<layer>/ dirs, later layers add or replace files; '{robot}' and
    # '{perception}' are filled from the setup (Setup.axes), and absent layers are skipped.
    skill_layers: tuple
    omit_skills: tuple = ()             # SkillRule values
    vocab: Vocab | None = None
    render_dirs: tuple = ('.agents/skills',)    # besides top-level *.md
    options: frozenset = field(default_factory=frozenset)
