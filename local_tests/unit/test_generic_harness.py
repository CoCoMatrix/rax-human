"""Cross-robot contracts; only fake backends and temporary files."""
import json
from types import SimpleNamespace
import pytest
from services.controller.frontend import Frontend


@pytest.mark.parametrize('robot', ['piper', 'franka', 'mock'])
@pytest.mark.parametrize('provider', ['codex-relay', 'claude-login'])
def test_finish_reserve_is_independent_of_robot_and_provider(tmp_path, robot, provider):
    calls = []
    backend = SimpleNamespace(mutating={'move'}, tools=lambda: [{'name': x} for x in ('move', 'hold')],
                              call=lambda name, args: (calls.append(name) or {'ok': True}, []))
    session = SimpleNamespace(config=SimpleNamespace(root=tmp_path, session_seconds=60,
                              robot=robot, agent_provider=provider), audit=lambda row: None)
    frontend = Frontend(session, backend)
    frontend.wall_budget = lambda: {'phase': 'wind_down'}
    reply = json.loads(frontend.call('move', {})['content'][0]['text'])
    assert reply['error']['stage'] == 'validation' and not calls
    frontend.call('hold', {})
    assert calls == ['hold']


def test_dashboard_uses_registered_telemetry_without_constructing_backend(monkeypatch):
    from services import backends
    from services.dashboard.sessions import Sessions
    seen = []
    monkeypatch.setattr(backends, 'make_telemetry', lambda robot: (
        seen.append(robot) or SimpleNamespace(snapshot=lambda: {'kind': robot, 'ok': True})))
    panel = Sessions(robot='mock')
    assert panel.robot() == {'kind': 'mock', 'ok': True}
    assert seen == ['mock']


def test_no_telemetry_never_falls_back_to_another_robot():
    from services.dashboard.sessions import Sessions
    assert Sessions(robot='mock').robot()['ok'] is False


def test_piper_telemetry_only_reads_supplied_owner():
    from services.piper.telemetry import Telemetry
    calls = []
    reader = SimpleNamespace(state=lambda: (calls.append('state') or {
        'robot_state': {'values': [0.1]*14}, 'robot_status_age_seconds': 0.01}))
    value = Telemetry(reader).snapshot()
    assert value['kind'] == 'piper' and value['ok']
    assert len(value['joints']['left']) == 6 and value['grippers']['right_width_m'] == 0.1
    assert calls == ['state']


def test_task_files_replace_cup_instructions_without_loading_controller(tmp_path):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
    from piper_explore import load_task, task_prompt, rehearsal_prompt
    for name, text in [('task.md', 'Sort parts'), ('bounds.json', '{}'),
                       ('common.md', 'Use task evidence.'), ('fresh.md', 'Fresh parts.'),
                       ('continue.md', 'Continue parts.'), ('practice.md', 'Practise parts.')]:
        (tmp_path/name).write_text(text)
    (tmp_path/'memory').mkdir()
    path=tmp_path/'session.json'
    path.write_text(json.dumps({'id': 'parts', 'title':'Sort parts', 'description':'task.md',
        'backend_options':'bounds.json', 'seed_memory':'memory', 'prompts':{
        'common':'common.md','fresh':'fresh.md','continuation':'continue.md',
        'rehearsal':'practice.md','working_rehearsal':'practice.md'}}))
    task=load_task(path)
    assert task_prompt('Start', task=task) == 'Start\nFresh parts.Use task evidence.'
    assert rehearsal_prompt('', task=task) == 'Practise parts.'
