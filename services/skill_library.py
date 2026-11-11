"""The agent's persistent skill library: task skills learned across sessions, one git repository per robot.

Separate from the harness's read-only skills (`agent_skills/`, rendered into `.agents/skills/`):
those are written by people; this library is written by agents, one session at a time.

    <artifact root>/skill-library/<robot>/      the library (git, branch main); only the harness writes it
    <session>/agent-workspace/skill-library/    the session's clone, on branch session/<session name>

The agent reads the library, runs its skills, and commits new or improved skills on its
session branch. `merge()` (the harness tool `skill_library_merge`) validates the branch and
merges it into main. The harness never runs git inside the agent's clone (its config and
hooks are the agent's): it copies the clone's object store to a scratch directory with a
fresh config and no hooks, fetches the branch from there into the library, validates the
files of that commit as data (code is only compiled, never run), and merges.

Format (enforced by `validate_tree`, documented in the library README and the agent's
`continual-learning` skill):

    INDEX.md                         one table row per skill: slug link, status, what, when
    skills/<slug>/SKILL.md           frontmatter + the required sections below
    skills/<slug>/code/*.py          the executable procedure; the entry defines main(ctx, ...)
    skills/<slug>/evidence/runs.jsonl  one JSON object per run of the code
    skills/<slug>/evidence/images/   visual references (jpg / png) the runs and SKILL.md cite
"""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

LIBRARY_DIR = 'skill-library'
ENVIRONMENT = 'RAX_SKILL_LIBRARY'
STATUSES = ('draft', 'verified', 'deprecated')
SECTIONS = ('When to use', 'Preconditions', 'Procedure', 'Code', 'Checks', 'Failure modes', 'Evidence')
RUN_KEYS = ('session', 'date', 'commit', 'outcome', 'unattended', 'conditions', 'checks', 'images')
VERIFIED_MIN_RUNS = 2       # unattended successful runs of the committed code
MAX_IMAGE_BYTES = 400_000
MAX_FILE_BYTES = 2_000_000
MAX_SKILL_BYTES = 25_000_000  # saved frames (RGB + depth + meta) let later sessions replay perception offline
SLUG = re.compile(r'^[a-z0-9][a-z0-9-]{1,60}$')
GIT_ENV = {'GIT_CONFIG_NOSYSTEM': '1', 'GIT_TERMINAL_PROMPT': '0', 'GIT_CONFIG_GLOBAL': '/dev/null'}

README = """# {robot} skill library

Skills agents learned on this robot, kept across sessions. Agents read and use them, and
add or improve them on their session branch; the harness tool `skill_library_merge`
checks the format and evidence and merges into `main`. The protocol is the agent skill
`continual-learning`; the rules the merge enforces:

- `INDEX.md`: one row per skill, `| [slug](skills/slug/SKILL.md) | status | what it does | when to use |`.
- `skills/<slug>/SKILL.md` begins with frontmatter (`name` = the folder name, `description`,
  `status`: draft | verified | deprecated, `robot`, `entry`: the code file, e.g.
  `code/run.py`) and has the sections {sections}.
- Code helpers live in `code/<slug_with_underscores>/` with `__init__.py`; every Python file must compile.
- The entry is Python defining `main(ctx, ...)` (ctx: `api.runtime.Context`) that returns
  `{{"ok": bool, "measured": {{...}}, "why": str}}`.
- `evidence/runs.jsonl`: one JSON object per run of the code with {run_keys}; `images` are
  paths under `evidence/images/` (jpg / png, at most {image_kb} kB each).
- `## Checks` holds a `### Perception checkpoints` table: for each checkpoint the step, the
  perception subagent, frames, question, expected answer, pass criterion and action on
  fail (or "None").
- At least one image per skill. `verified` needs at least {verified} unattended successful
  runs. A branch may only change `INDEX.md` and files under `skills/`.
- Never delete another session's skill: set its status to `deprecated` and say why.
"""
INDEX = """# Skill index

| Skill | Status | What it does | When to use |
|---|---|---|---|
"""


class LibraryError(RuntimeError):
    pass


def _git(*args, cwd=None, check=True):
    result = subprocess.run(['git', *args], cwd=cwd, capture_output=True, text=True, timeout=60,
                            env={**os.environ, **GIT_ENV})
    if check and result.returncode:
        raise LibraryError(f"git {' '.join(args)}: {(result.stderr or result.stdout).strip()[:500]}")
    return result


def library_path(config):
    """The robot's library: RAX_SKILL_LIBRARY, else <artifact root>/skill-library/<robot> next to sessions/."""
    configured = getattr(config, 'skill_library_root', None) or os.environ.get(ENVIRONMENT)
    if configured:
        return Path(configured)/config.robot
    sessions = Path(config.root).resolve().parent
    base = sessions.parent if sessions.name == 'sessions' else sessions
    return base/LIBRARY_DIR/config.robot


def ensure_library(path, robot):
    """Create the library repository (README, INDEX, skills/) on first use."""
    path = Path(path)
    if (path/'.git').is_dir():
        return path
    path.mkdir(parents=True, exist_ok=True)
    _git('init', '-q', '-b', 'main', cwd=path)
    _git('config', 'user.name', 'rax harness', cwd=path)
    _git('config', 'user.email', 'harness@rax.local', cwd=path)
    (path/'README.md').write_text(README.format(robot=robot, sections=', '.join(f'`## {s}`' for s in SECTIONS),
                                                run_keys=', '.join(f'`{k}`' for k in RUN_KEYS),
                                                image_kb=MAX_IMAGE_BYTES // 1000, verified=VERIFIED_MIN_RUNS))
    (path/'INDEX.md').write_text(INDEX)
    (path/'skills').mkdir(exist_ok=True)
    (path/'skills'/'.gitattributes').write_text('*/evidence/runs.jsonl merge=union\n')
    _git('add', '-A', cwd=path)
    _git('commit', '-q', '-m', f'Initialize the {robot} skill library', cwd=path)
    return path


def branch_name(config):
    root = Path(config.root).resolve()
    label = re.sub(r'[^A-Za-z0-9_-]', '-', root.name)
    identity = hashlib.sha256(str(root).encode()).hexdigest()[:12]
    return f'session/{label}-{identity}'


def checkout(config, workspace):
    """Clone the library into the new workspace on the session's branch; returns its summary."""
    library = ensure_library(library_path(config), config.robot)
    clone = Path(workspace)/LIBRARY_DIR
    _git('clone', '-q', '--no-hardlinks', str(library), str(clone))
    _git('checkout', '-q', '-b', branch_name(config), cwd=clone)
    _git('config', 'user.name', f'agent {Path(config.root).name}', cwd=clone)
    _git('config', 'user.email', 'agent@rax.local', cwd=clone)
    return {'library': str(library), 'clone': str(clone), 'branch': branch_name(config),
            'skills': len(list((clone/'skills').glob('*/SKILL.md')))}


# The pinned skills-lib checker is host-owned; never import the agent's checker/code.
from .skill_library_check import validate_skill
from . import skill_library_check as checker


def validate_tree(tree, changed):
    tree = Path(tree)
    links = [p for p in tree.rglob('*') if p.is_symlink()]
    if links:
        return ['symlinks are not allowed: ' + str(p.relative_to(tree)) for p in links], []
    slugs = sorted(p.name for p in (tree/'skills').iterdir() if p.is_dir()) if (tree/'skills').is_dir() else []
    deleted = [p for p in changed if not (tree/p).is_file()]
    return checker.validate_tree(tree, slugs, changed, deleted), slugs


# ----- merge (the harness tool) ---------------------------------------------------------------
def _copy_objects(source, target):
    """Copy a git object store without symlinks and without alternates (no reads outside the clone)."""
    source = Path(source)
    for directory, names, files in os.walk(source, followlinks=False):
        here = Path(directory)
        out = Path(target)/here.relative_to(source)
        out.mkdir(parents=True, exist_ok=True)
        names[:] = [n for n in names if not (here/n).is_symlink()]
        for name in files:
            path = here/name
            if path.is_symlink() or (here.name == 'info' and name in ('alternates', 'http-alternates')):
                continue
            shutil.copyfile(path, out/name)



def merge(config, workspace, message):
    """Validate the session branch of the workspace's clone and merge it into the library's main."""
    library = ensure_library(library_path(config), config.robot)
    clone = Path(workspace)/LIBRARY_DIR
    branch = branch_name(config)
    if any(p.is_symlink() for p in (clone, clone/'.git', clone/'.git/objects', clone/'.git/refs')):
        raise LibraryError('symlinked library git metadata is not allowed')
    if not (clone/'.git').is_dir():
        raise LibraryError('the workspace has no skill-library clone')
    lock = open(library/'.git'/'rax-merge.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    scratch = Path(tempfile.mkdtemp(prefix='skill-merge-'))
    try:
        if _git('branch', '--show-current', cwd=library).stdout.strip() != 'main' or _git('status', '--porcelain', cwd=library).stdout.strip():
            raise LibraryError('shared library must be clean on main before a session merge')
        # The agent's objects only: no config, hooks or info from its .git.
        objects = scratch/'agent.git'
        objects.mkdir()
        _copy_objects(clone/'.git'/'objects', objects/'objects')
        refs = clone/'.git'/'refs'/'heads'/Path(branch)
        packed = clone/'.git'/'packed-refs'
        if any(p.is_symlink() for p in (refs, *refs.parents)) or packed.is_symlink():
            raise LibraryError('symlinked branch metadata is not allowed')
        head = refs.read_text().strip() if refs.is_file() else None
        if head is None and packed.is_file():
            for line in packed.read_text().splitlines():
                if line.endswith(f' refs/heads/{branch}'):
                    head = line.split()[0]
        if not head or not re.fullmatch(r'[0-9a-f]{40}', head):
            raise LibraryError(f'no commit on {branch}: commit your skill first (git add, git commit)')
        (objects/'refs'/'heads').mkdir(parents=True)
        (objects/'HEAD').write_text('ref: refs/heads/main\n')
        (objects/'config').write_text('[core]\n\trepositoryformatversion = 0\n\tbare = true\n')
        _git('fetch', '-q', '--no-tags', str(objects), head, cwd=library)
        if _git('merge-base', '--is-ancestor', head, 'main', cwd=library, check=False).returncode == 0:
            return {'merged': False, 'reason': 'no committed changes; branch already in main', 'branch': branch, 'head': head}
        base = _git('merge-base', 'main', head, cwd=library).stdout.strip()
        changed = [p for p in _git('diff', '--name-only', base, head, cwd=library).stdout.splitlines() if p]
        if not changed:
            return {'merged': False, 'reason': 'no committed changes on the session branch since its base',
                    'branch': branch, 'head': head}
        tree = scratch/'tree'
        tree.mkdir()
        listing = _git('ls-tree', '-rz', head, cwd=library).stdout
        for record in filter(None, listing.split('\0')):
            header, name = record.split('\t', 1)
            mode, kind, oid = header.split()
            if mode not in ('100644', '100755') or kind != 'blob':
                return {'merged': False, 'problems': ['symlinks are not allowed (nor submodules): ' + name]}
            target = tree/name
            if not target.resolve().is_relative_to(tree.resolve()):
                raise LibraryError('library path escapes the checked tree')
            target.parent.mkdir(parents=True, exist_ok=True)
            blob = subprocess.run(['git', 'cat-file', 'blob', oid], cwd=library, capture_output=True,
                                  check=True, timeout=60, env={**os.environ, **GIT_ENV}).stdout
            target.write_bytes(blob)
        problems, slugs = validate_tree(tree, changed)
        if problems:
            return {'merged': False, 'branch': branch, 'head': head, 'problems': problems,
                    'do': 'fix these in skill-library/, commit, and call skill_library_merge again'}
        before = _git('rev-parse', 'main', cwd=library).stdout.strip()
        result = _git('merge', '--no-ff', '-m', f'Merge {branch}: {message.strip()[:200]}', head, cwd=library,
                      check=False)
        if result.returncode:
            conflicts = _git('diff', '--name-only', '--diff-filter=U', cwd=library, check=False).stdout.split()
            _git('merge', '--abort', cwd=library, check=False)
            return {'merged': False, 'branch': branch, 'head': head, 'conflicts': conflicts,
                    'do': 'in skill-library/: git fetch origin && git merge origin/main, resolve the conflicts '
                          '(keep both sessions\' evidence), commit, and call skill_library_merge again'}
        after = _git('rev-parse', 'main', cwd=library).stdout.strip()
        return {'merged': True, 'branch': branch, 'head': head, 'library_main': after, 'previous_main': before,
                'skills': slugs, 'changed_files': len(changed),
                'next': 'git fetch origin in skill-library/ to see the merged main'}
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()
