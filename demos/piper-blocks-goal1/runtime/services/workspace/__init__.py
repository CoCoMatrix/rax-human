"""Agent workspace generation: recipe + setup facts -> files."""
from .build import build, install_skills
from .contract_files import describe, write_contract
from .recipes import recipe_for
from .setup import ROBOT_FACTS, Setup
from .templates import render

__all__ = ['ROBOT_FACTS', 'Setup', 'build', 'describe', 'install_skills', 'recipe_for', 'render', 'write_contract']
