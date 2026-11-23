"""One pipeline for every agent workspace: sources + recipe + setup -> files.

build() copies the recipe's template and skill layers, drops skills the setup does not
use, renders every markdown source against the setup, and writes the tool contract.
Callers add the session's extras (TASK.md, CLI entry files) around it.
"""
from pathlib import Path
import shutil

from .contract_files import write_contract
from .recipe import SKILL_SOURCE
from .templates import render

IGNORE = shutil.ignore_patterns('__pycache__', '*.pyc')
# Templates may hold machine-local runtime/ state; never copy it.
TEMPLATE_IGNORE = shutil.ignore_patterns('__pycache__', '*.pyc', 'runtime')


def install_skills(skills, layers, source=SKILL_SOURCE):
    """Copy each layer's skill directories in order; later layers add or replace files."""
    skills = Path(skills)
    for layer in layers:
        root = Path(source) / layer
        if not root.is_dir():
            continue  # A layer may have no skills.
        for skill in sorted(path for path in root.iterdir() if path.is_dir()):
            if skill.is_symlink():
                raise ValueError('Skill sources must not be symlinks')
            shutil.copytree(skill, skills / skill.name, dirs_exist_ok=True, ignore=IGNORE)


def _no_link(path, what):
    if Path(path).is_symlink():
        raise ValueError(f'Workspace templates must not contain symlinked {what}')


def apply_role(recipe, setup, workspace):
    skills = workspace / '.agents/skills'
    install_skills(skills, [layer.format(**setup.axes()) for layer in recipe.skill_layers])
    for rule in recipe.omit_skills:
        if rule.when(setup):
            path = skills / rule.skill
            if path.exists():
                _no_link(path, 'skills')
                shutil.rmtree(path)
    for metadata in sorted(skills.glob('*/agents/openai.yaml')):  # Codex-only skill metadata.
        metadata.unlink()
        if not any(metadata.parent.iterdir()):
            metadata.parent.rmdir()


def render_sources(recipe, setup, workspace):
    """Render every markdown source in place; returns {relative path: block ids used}."""
    paths = list(workspace.glob('*.md'))
    for directory in recipe.render_dirs:
        if (workspace / directory).is_dir():
            paths.extend((workspace / directory).rglob('*.md'))
    blocks = {}
    for path in sorted(paths):
        _no_link(path, 'instructions')
        original = path.read_text()
        used = set()
        result = render(original, setup, vocab=recipe.vocab, used=used)
        if result != original:
            path.write_text(result)
        if used:
            blocks[path.relative_to(workspace).as_posix()] = sorted(used)
    return blocks


def manifest(recipe, setup, workspace, blocks):
    """Provenance of a built workspace (operator side, never in the workspace)."""
    import hashlib
    from .contract_files import manifest as contract
    files = {}
    for path in sorted(Path(workspace).rglob('*')):
        if path.is_file() and not path.is_symlink() and '__pycache__' not in path.parts:
            relative = path.relative_to(workspace).as_posix()
            files[relative] = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                               **({'blocks': blocks[relative]} if relative in blocks else {})}
    return {'schema': 'workspace_build_v2', 'recipe': recipe.name, 'contract': contract(setup), 'files': files}


def build(recipe, setup, workspace, *, copy_template=True):
    """Generate the workspace for `setup` into `workspace` (created if needed)."""
    workspace = Path(workspace)
    skills = workspace / '.agents/skills'
    if skills.is_symlink():
        raise ValueError('Workspace templates must not contain symlinked skills')
    if copy_template:
        shutil.copytree(recipe.template, workspace, dirs_exist_ok=True, ignore=TEMPLATE_IGNORE)
    apply_role(recipe, setup, workspace)
    blocks = render_sources(recipe, setup, workspace)
    write_contract(workspace, setup)
    return manifest(recipe, setup, workspace, blocks)
