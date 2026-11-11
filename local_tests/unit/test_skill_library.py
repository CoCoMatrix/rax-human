"""The persistent skill library: checkout, format and evidence checks, merges, and the sandbox boundary."""
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from services import skill_library as lib

SECTIONS = '\n\n'.join(f'## {s}\ntext' for s in lib.SECTIONS) + '\n\n### Perception checkpoints\nNone\n'


def git(*args, cwd):
    return subprocess.run(['git', *args], cwd=cwd, check=True, capture_output=True, text=True).stdout


def session(tmp_path, name):
    config = SimpleNamespace(root=tmp_path/'sessions'/name, robot='mock')
    workspace = config.root/'agent-workspace'
    workspace.mkdir(parents=True)
    return config, workspace, lib.checkout(config, workspace)


def write_skill(clone, slug='pick-box', status='draft', runs=(), entry='def main(ctx, size=0.3):\n    return {}\n',
                body=SECTIONS, image=True):
    folder = clone/'skills'/slug
    (folder/'code').mkdir(parents=True, exist_ok=True)
    (folder/'SKILL.md').write_text(f'---\nname: {slug}  # the folder\ndescription: picks a box\nstatus: {status}\n'
                                   f'robot: mock\nentry: code/run.py\n---\n\n# Pick\n\n{body}\n')
    (folder/'code'/'run.py').write_text(entry)
    package=folder/'code'/slug.replace('-', '_');package.mkdir(exist_ok=True)
    (package/'__init__.py').write_text('')
    if image:
        (folder/'evidence'/'images').mkdir(parents=True, exist_ok=True)
        (folder/'evidence'/'images'/'start.jpg').write_bytes(b'\xff\xd8' + b'0' * 1000)
    if runs:
        (folder/'evidence'/'runs.jsonl').write_text('\n'.join(json.dumps(r) for r in runs) + '\n')
    index = clone/'INDEX.md'
    if f'(skills/{slug}/SKILL.md)' not in index.read_text():
        index.write_text(index.read_text() + f'| [{slug}](skills/{slug}/SKILL.md) | {status} | picks | floor |\n')


def commit(clone, message='skill'):
    git('add', '-A', cwd=clone)
    git('commit', '-q', '-m', message, cwd=clone)


def run(outcome='success', unattended=True):
    return {'session': 's', 'date': '2026-10-08', 'commit': 'abc', 'outcome': outcome, 'unattended': unattended,
            'conditions': 'box on floor', 'checks': {'lifted_m': 0.1}, 'images': ['evidence/images/start.jpg']}


def test_checkout_creates_the_library_and_a_session_branch(tmp_path):
    config, workspace, info = session(tmp_path, 'S1')
    library = tmp_path/'skill-library'/'mock'
    assert info['library'] == str(library) and info['branch'] == lib.branch_name(config)
    assert (library/'README.md').is_file() and (library/'INDEX.md').is_file()
    clone = workspace/'skill-library'
    assert git('rev-parse', '--abbrev-ref', 'HEAD', cwd=clone).strip() == lib.branch_name(config)
    assert git('config', 'user.name', cwd=clone).strip() == 'agent S1'


def test_a_valid_draft_merges_into_main_for_the_next_session(tmp_path):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace/'skill-library'
    write_skill(clone)
    commit(clone)
    result = lib.merge(config, workspace, 'add pick-box')
    assert result['merged'] and result['skills'] == ['pick-box']
    _, workspace2, info2 = session(tmp_path, 'S2')
    assert info2['skills'] == 1 and (workspace2/'skill-library'/'skills'/'pick-box'/'code'/'run.py').is_file()


def test_format_and_evidence_problems_are_returned_and_nothing_merges(tmp_path):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace/'skill-library'
    library = Path(lib.library_path(config))
    before = git('rev-parse', 'main', cwd=library)
    write_skill(clone, status='verified', runs=[run()], entry='def run(x):\n    pass\n',
                body=SECTIONS.replace('## Checks', '## Notes'), image=True)
    commit(clone)
    result = lib.merge(config, workspace, 'bad')
    text = ' '.join(result['problems'])
    assert not result['merged'] and git('rev-parse', 'main', cwd=library) == before
    for expected in ('"## Checks"', 'main(ctx', 'at least 2 unattended successful runs'):
        assert expected in text
    write_skill(clone, status='verified', runs=[run(), run(), run('failure')])
    commit(clone)
    assert lib.merge(config, workspace, 'verified now')['merged']


def test_a_branch_may_only_add_or_change_skills_and_never_delete_them(tmp_path):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace/'skill-library'
    write_skill(clone)
    commit(clone)
    assert lib.merge(config, workspace, 'add')['merged']
    (clone/'README.md').write_text('mine now')
    commit(clone)
    assert 'may only change' in ' '.join(lib.merge(config, workspace, 'readme')['problems'])
    git('checkout', '-q', 'HEAD~1', '--', 'README.md', cwd=clone)
    git('rm', '-q', '-r', 'skills/pick-box', cwd=clone)
    commit(clone)
    assert 'deprecated instead' in ' '.join(lib.merge(config, workspace, 'delete')['problems'])


def test_conflicting_sessions_get_conflicts_and_the_library_stays_clean(tmp_path):
    config1, workspace1, _ = session(tmp_path, 'S1')
    config2, workspace2, _ = session(tmp_path, 'S2')
    for workspace, text in ((workspace1, 'one'), (workspace2, 'two')):
        write_skill(workspace/'skill-library', body=SECTIONS.replace('text', text, 1))
        commit(workspace/'skill-library')
    assert lib.merge(config1, workspace1, 'first')['merged']
    result = lib.merge(config2, workspace2, 'second')
    library = Path(lib.library_path(config1))
    assert not result['merged'] and 'skills/pick-box/SKILL.md' in result['conflicts']
    assert git('status', '--porcelain', cwd=library) == '' and 'merge origin/main' in result['do']


def test_the_harness_never_runs_the_agents_git_config_hooks_or_links(tmp_path):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace/'skill-library'
    marker = tmp_path/'pwned'
    script = tmp_path/'evil.sh'
    script.write_text(f'#!/bin/sh\ntouch {marker}\n')
    script.chmod(0o755)
    for hook in ('reference-transaction', 'post-checkout', 'pre-auto-gc', 'post-merge'):
        (clone/'.git'/'hooks'/hook).write_text(script.read_text())
        (clone/'.git'/'hooks'/hook).chmod(0o755)
    git('config', 'core.fsmonitor', str(script), cwd=clone)
    git('config', 'core.hooksPath', str(clone/'.git'/'hooks'), cwd=clone)
    marker.unlink(missing_ok=True)  # the agent's own git commands above may have run them
    (clone/'.git'/'objects'/'info'/'alternates').write_text(str(Path.home()) + '\n')
    write_skill(clone)
    (clone/'skills'/'pick-box'/'leak.md').symlink_to('/etc/hostname')
    commit(clone)
    marker.unlink(missing_ok=True)
    result = lib.merge(config, workspace, 'evil')
    assert not marker.exists()
    assert not result['merged'] and 'symlinks are not allowed' in ' '.join(result['problems'])


def test_nothing_committed_is_reported(tmp_path):
    config, workspace, _ = session(tmp_path, 'S1')
    assert 'no committed changes' in lib.merge(config, workspace, 'nothing')['reason']


def test_uses_must_name_skills_that_exist(tmp_path):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace/'skill-library'
    write_skill(clone, slug='localize-box')
    write_skill(clone, slug='pick-box')
    skill = clone/'skills'/'pick-box'/'SKILL.md'
    skill.write_text(skill.read_text().replace('entry: code/run.py', 'entry: code/run.py\nuses: [localize-box, lift-box]'))
    commit(clone)
    result = lib.merge(config, workspace, 'compose')
    assert not result['merged'] and "uses 'lift-box'" in ' '.join(result['problems'])
    skill.write_text(skill.read_text().replace('uses: [localize-box, lift-box]', 'uses: [localize-box]'))
    commit(clone)
    assert lib.merge(config, workspace, 'compose')['merged']


def test_a_skill_must_document_its_perception_checkpoints(tmp_path):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace/'skill-library'
    write_skill(clone, body=SECTIONS.replace('### Perception checkpoints', '### Notes'))
    commit(clone)
    result = lib.merge(config, workspace, 'no checkpoints')
    assert not result['merged'] and 'Perception checkpoints' in ' '.join(result['problems'])
