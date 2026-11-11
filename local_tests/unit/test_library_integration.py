"""Exercise the real workspace/API/MCP path using only a mock robot."""
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from services.controller.config import Configuration
from services.controller.frontend import Frontend
from services import skill_library as lib
from harness.codex_cli.launcher import prepare, sandbox_command
from test_skill_library import write_skill, commit, git


def config(tmp_path, name):
    return Configuration(root=tmp_path/name/'rax-session', task='arbitrary-task', robot='mock',
                         agent_provider='codex-relay', skill_library_root=str(tmp_path/'shared'))


def prepared(cfg):
    return prepare(cfg, cfg.root/'unused-key.py', 'https://example.invalid/v1')


def test_new_api_sessions_share_checked_library_but_have_distinct_branches(tmp_path):
    cfg = config(tmp_path, 'first')
    workspace = prepared(cfg)
    clone = workspace/'skill-library'
    write_skill(clone)
    commit(clone)
    backend = SimpleNamespace(tools=lambda: [], call=lambda *a: pytest.fail('hardware call'))
    audit = []
    frontend = Frontend(SimpleNamespace(config=cfg, audit=audit.append), backend)
    assert 'skill_library_merge' in {x['name'] for x in frontend.tools()}
    frontend.wall_budget = lambda: {'phase': 'expired'}
    result = json.loads(frontend.call('skill_library_merge', {'message': 'offline fixture'})['content'][0]['text'])
    assert result['merged'] and audit[-1]['event'] == 'mcp_result'
    assert not lib.merge(cfg, workspace, 'again')['merged']
    cfg2 = config(tmp_path, 'second')
    workspace2 = prepared(cfg2)
    assert lib.branch_name(cfg) != lib.branch_name(cfg2)
    assert (workspace2/'skill-library/skills/pick-box/SKILL.md').exists()
    assert 'continual-learning' in (workspace2/'AGENTS.md').read_text()
    assert (workspace2/'.agents/skills/continual-learning/references/library-format.md').exists()
    assert 'skill_library_merge' in (workspace2/'MCP_SESSION.md').read_text()
    settings = json.loads((cfg2.root/'codex-relay.json').read_text())
    assert settings['primary_model'] == settings['vision_model'] == 'gpt-6-astra'


def test_index_only_status_change_is_checked_and_never_published(tmp_path):
    cfg = config(tmp_path, 'one'); ws = prepared(cfg); clone=ws/'skill-library'
    write_skill(clone); commit(clone)
    assert lib.merge(cfg, ws, 'draft')['merged']
    before=git('rev-parse','main',cwd=lib.library_path(cfg))
    p=clone/'INDEX.md';p.write_text(p.read_text().replace('| draft |','| verified |'))
    commit(clone)
    result=lib.merge(cfg,ws,'false promotion')
    assert not result['merged'] and 'INDEX.md lists' in ' '.join(result['problems'])
    assert git('rev-parse','main',cwd=lib.library_path(cfg))==before


def test_helpers_are_compiled_without_executing_entry(tmp_path):
    cfg=config(tmp_path,'one');ws=prepared(cfg);clone=ws/'skill-library'
    write_skill(clone,entry='raise RuntimeError("must never run")\ndef main(ctx):\n    return {}\n')
    p=clone/'skills/pick-box/code/pick_box/helper.py';p.write_text('def broken(:')
    commit(clone)
    result=lib.merge(cfg,ws,'broken helper')
    assert not result['merged'] and 'does not compile' in ' '.join(result['problems'])


def test_shared_repository_is_mounted_read_only_in_codex(tmp_path, monkeypatch):
    from harness.codex_cli import launcher
    cfg=config(tmp_path,'one');prepared(cfg)
    monkeypatch.setattr(launcher.sys,'platform','linux')
    monkeypatch.setattr(launcher.shutil,'which',lambda name:'/usr/bin/'+name)
    command=sandbox_command(cfg,tmp_path/'robot.sock',tmp_path/'model.sock',prompt='offline')
    shared=str(lib.library_path(cfg))
    index=command.index(shared)
    assert command[index-1]=='--ro-bind' and command[index+1]==shared
    assert command[command.index(str(cfg.root/'agent-workspace'))-1]=='--bind'


def test_skill_and_tool_are_absent_without_a_library(tmp_path):
    from services.workspace import Setup, build, recipe_for
    ws=tmp_path/'plain';build(recipe_for(),Setup(robot='mock',gemini=False),ws)
    assert not (ws/'.agents/skills/continual-learning').exists()
    cfg=config(tmp_path,'empty')
    frontend=Frontend(SimpleNamespace(config=cfg),SimpleNamespace(tools=lambda:[]))
    assert 'skill_library_merge' not in {t['name'] for t in frontend.tools()}
