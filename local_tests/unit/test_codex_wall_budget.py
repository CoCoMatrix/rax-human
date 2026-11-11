"""Budget recovery must not reset elapsed time or dispatch during the finish reserve."""
import json
from types import SimpleNamespace

from services.controller.frontend import Frontend


def make_frontend(monkeypatch, *, elapsed=0):
    from services.controller import frontend as module
    clock = {'wall': 1000., 'mono': 20.}
    monkeypatch.setattr(module.time, 'time', lambda: clock['wall'])
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock['mono'])
    monkeypatch.setattr(module.operator, 'recent', lambda root, limit=5: [])
    calls = []
    backend = SimpleNamespace(
        mutating={'piper_move_pose'},
        tools=lambda: [{'name': n} for n in ('piper_move_pose', 'piper_hold', 'observe')],
        status=lambda: {'measured': True},
        call=lambda name, args: (calls.append(name) or {'ok': True}, []))
    config = SimpleNamespace(session_seconds=900, agent_provider='codex-relay',
                             robot='piper', task='cup', observation_profile='rgbd', root=None)
    session = SimpleNamespace(config=config, audit=lambda value: None, budget=lambda: {})
    return Frontend(session, backend, started_at=1000-elapsed), clock, calls


def value(result):
    return json.loads(result['content'][0]['text'])


def test_resume_retains_elapsed_and_wall_jump_does_not_extend_budget(monkeypatch):
    f, clock, calls = make_frontend(monkeypatch, elapsed=810)
    assert f.status()['wall_budget']['remaining_seconds'] == 90
    clock.update(wall=-10000, mono=51.)
    rejected = value(f.call('piper_move_pose', {}))['error']
    assert rejected['no_action_executed'] and not calls
    assert rejected['wall_budget']['phase'] == 'wind_down'
    assert rejected['wall_budget']['remaining_seconds'] == 59
    assert value(f.call('observe', {}))['ok']
    assert value(f.call('piper_hold', {}))['ok']
    assert calls == ['observe', 'piper_hold']


def test_success_and_failure_carry_real_budget(monkeypatch):
    f, clock, calls = make_frontend(monkeypatch)
    clock['mono'] += 100
    assert value(f.call('piper_move_pose', {}))['wall_budget']['remaining_seconds'] == 800
    clock['mono'] += 850
    result = value(f.call('piper_move_pose', {}))
    assert result['error']['wall_budget']['remaining_seconds'] == 0
    assert result['error']['wall_budget']['phase'] == 'expired'
    assert calls == ['piper_move_pose']


def test_clock_additions_do_not_change_unstarted_upstream_frontend(monkeypatch):
    f, _, _ = make_frontend(monkeypatch)
    f.started_at = None
    assert 'wall_budget' not in value(f.call('piper_move_pose', {}))


def test_launcher_monotonic_anchor_survives_wall_jump_during_child_startup(monkeypatch):
    f, clock, _ = make_frontend(monkeypatch)
    clock.update(wall=-10000., mono=51.)
    resumed = Frontend(f.s, f.backend, started_at=190., budget_started_monotonic=20.,
                       budget_previous_elapsed=810.)
    assert resumed.wall_budget()['remaining_seconds'] == 59
    assert resumed.wall_budget()['phase'] == 'wind_down'
