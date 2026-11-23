"""goal1 unit tests: goal description, answer parsing, image intake, web goal endpoints. No hardware, no network."""
import base64
import io
import json
import threading
import urllib.error
import urllib.request

import pytest

from pathlib import Path

from services.dashboard import goal_gemini, goal_spec as gs


def stacks(*rows):
    return {'schema': gs.SCHEMA, 'stacks': [{'col': c, 'row': r, 'blocks': list(b)} for c, r, b in rows], 'spacing': 'tight'}


def test_default_is_current_fixed_goal():
    c = gs.check(gs.DEFAULT_SPEC)
    assert c['ok'] and c['is_default'] and c['levels'] == 3
    # Supports first; red/orange may swap; exactly the two orders the current planner uses.
    assert sorted(c['placement_orders']) == sorted([['red', 'orange', 'green', 'yellow'], ['orange', 'red', 'green', 'yellow'],
                                                    ['red', 'green', 'orange', 'yellow'], ['red', 'green', 'yellow', 'orange']])


def test_canonical_shifts_and_sorts_and_digest_is_stable():
    a = gs.check(stacks((5, 7, ['orange']), (4, 7, ['red', 'green', 'yellow'])))
    assert a['ok'] and a['spec'] == gs.DEFAULT_SPEC and a['digest'] == gs.digest(gs.DEFAULT_SPEC)


@pytest.mark.parametrize('raw,needle', [
    (stacks((0, 0, ['red', 'green', 'yellow'])), '缺少橙色'),
    (stacks((0, 0, ['red', 'red', 'green']), (1, 0, ['orange', 'yellow'])), '出现了 2 次'),
    (stacks((0, 0, ['red', 'orange', 'green', 'yellow'])), '最多叠 3 层'),
    (stacks((0, 0, ['red']), (0, 0, ['orange']), (1, 0, ['green']), (2, 0, ['yellow'])), '出现了两次'),
    (stacks((0, 0, ['red']), (9, 0, ['orange']), (1, 0, ['green']), (2, 0, ['yellow'])), '超过'),
    ({'stacks': [{'col': 0, 'row': 0, 'blocks': ['blue']}]}, '红橙绿黄'),
    ({'stacks': [{'col': True, 'row': 0, 'blocks': ['red']}]}, '整数'),
    ({'stacks': []}, '缺少 stacks'), ('nonsense', 'JSON 对象'), (None, 'JSON 对象'),
])
def test_invalid_descriptions_are_rejected_with_reason(raw, needle):
    c = gs.check(raw)
    assert not c['ok'] and c['spec'] is None and any(needle in e for e in c['errors']), c['errors']


def test_row_l_shape_and_two_towers_have_orders_but_tight_square_does_not():
    row = gs.check(stacks((0, 0, ['red']), (1, 0, ['orange']), (2, 0, ['green']), (3, 0, ['yellow'])))
    assert row['ok'] and len(row['placement_orders']) == 24 and not row['warnings']
    ell = gs.check(stacks((0, 0, ['red']), (1, 0, ['orange']), (0, 1, ['green', 'yellow'])))
    assert ell['ok'] and ell['placement_orders'] and not ell['warnings']
    towers = gs.check(stacks((0, 0, ['red', 'green']), (1, 0, ['orange', 'yellow'])))
    assert towers['ok'] and towers['placement_orders']
    square = stacks((0, 0, ['red']), (1, 0, ['orange']), (0, 1, ['green']), (1, 1, ['yellow']))
    tight = gs.check(square)
    assert tight['ok'] and not tight['placement_orders'] and '96 mm' in tight['warnings'][0]
    loose = gs.check({**square, 'spacing': 'loose'})
    assert loose['ok'] and len(loose['placement_orders']) == 24 and not loose['warnings']


def test_every_order_puts_supports_first():
    c = gs.check(stacks((0, 0, ['yellow', 'red']), (2, 0, ['green', 'orange'])))
    for order in c['placement_orders']:
        assert order.index('yellow') < order.index('red') and order.index('green') < order.index('orange')


def test_edit_form_round_trip_and_floating_block():
    cells = {'red': {'col': 0, 'row': 0, 'level': 1}, 'green': {'col': 0, 'row': 0, 'level': 2},
             'yellow': {'col': 0, 'row': 0, 'level': 3}, 'orange': {'col': 1, 'row': 0, 'level': 1}}
    assert gs.check(gs.from_cells(cells))['is_default']
    cells['yellow']['level'] = 1
    cells['green']['level'] = 3
    bad = gs.check(gs.from_cells(cells))
    assert not bad['ok'] and '连续' in bad['errors'][0]


def test_schema_is_closed_and_enumerated():
    schema = gs.gemini_schema()
    assert schema['additionalProperties'] is False and set(schema['required']) == set(schema['properties'])
    block = schema['properties']['stacks']['items']
    assert block['additionalProperties'] is False and block['properties']['blocks']['items']['enum'] == list(gs.COLORS)


def jpeg(size=(640, 480), fmt='JPEG'):
    from PIL import Image
    out = io.BytesIO()
    Image.new('RGB', size, (120, 130, 140)).save(out, fmt)
    return out.getvalue()


def test_image_intake_bounds_and_formats():
    data, meta = goal_gemini.load_image('data:image/jpeg;base64,' + base64.b64encode(jpeg((4000, 3000))).decode())
    assert max(meta['width'], meta['height']) == goal_gemini.MAX_SIDE and data[:2] == b'\xff\xd8'
    png, _ = goal_gemini.load_image('data:image/png;base64,' + base64.b64encode(jpeg(fmt='PNG')).decode())
    assert png[:2] == b'\xff\xd8'
    for bad in ('data:image/gif;base64,AAAA', 'http://example.com/a.jpg', None,
                'data:image/jpeg;base64,' + base64.b64encode(b'not an image').decode(),
                'data:image/jpeg;base64,' + base64.b64encode(jpeg((100, 100))).decode()):
        with pytest.raises(Exception):
            goal_gemini.load_image(bad)


def test_answer_parsing_accepts_fenced_json_only_objects():
    assert goal_gemini.parse_answer('```json\n{"a": 1}\n```') == {'a': 1}
    assert goal_gemini.parse_answer(' {"a": 1} ') == {'a': 1}
    for bad in ('', None, '[1]', 'sorry'):
        with pytest.raises(Exception):
            goal_gemini.parse_answer(bad)


def test_analyze_uses_strict_schema_and_no_key_leak(tmp_path):
    key = tmp_path/'key'
    key.write_text('secret-test-key')
    key.chmod(0o600)
    seen = {}

    def send(request):
        seen.update(request)
        answer = {'image_usable': True, 'visible_colors': list(gs.COLORS), 'stacks': gs.DEFAULT_SPEC['stacks'],
                  'spacing': 'tight', 'viewpoint': 'oblique', 'confidence': .9, 'uncertain': [], 'summary': 'x'}
        return {'model': 'gemini-3.8-flash', 'choices': [{'message': {'role': 'assistant', 'content': json.dumps(answer)}, 'finish_reason': 'stop'}],
                'usage': {'prompt_tokens': 900, 'total_tokens': 1000}}
    result = goal_gemini.analyze(jpeg(), key_file=str(key), proxy_url=None, send=send)
    assert gs.check(result['answer'])['is_default'] and result['tokens'] == 1000
    fmt = seen['response_format']
    assert fmt['type'] == 'json_schema' and fmt['json_schema']['strict'] is True and fmt['json_schema']['schema'] == gs.gemini_schema()
    assert seen['model'] == 'gemini-3.8-flash' and 'secret-test-key' not in json.dumps(seen)
    assert seen['messages'][1]['content'][1]['image_url']['url'].startswith('data:image/jpeg;base64,')


@pytest.fixture
def web(tmp_path, monkeypatch):
    """A real server instance on a temp data dir. No devices are started by importing or serving."""
    from services.dashboard import blocks_scale as web
    manager = web.Manager()
    manager.data = tmp_path
    manager.goal_dir = tmp_path/'goal'
    manager.guard_file = tmp_path/'requires-rest.json'
    manager.recovery_required = False  # the live console's guard file must not leak into tests
    server, token = web.make_server(manager, 0, token='t' * 20)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_address[1]}'

    def call(path, body=None, token_value=token):
        headers = {'Cookie': f'{web.COOKIE}={token_value}'}
        data = None
        if body is not None:
            headers['Content-Type'] = 'application/json'
            data = json.dumps(body).encode()
        try:
            with urllib.request.urlopen(urllib.request.Request(base+path, data=data, headers=headers), timeout=10) as r:
                return r.status, json.loads(r.read()) if 'json' in r.headers['Content-Type'] else r.read()
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())
    yield manager, call, monkeypatch, web
    server.shutdown()
    server.server_close()


SQUARE_CELLS = {'red': {'col': 0, 'row': 0, 'level': 1}, 'orange': {'col': 1, 'row': 0, 'level': 1},
                'green': {'col': 0, 'row': 1, 'level': 1}, 'yellow': {'col': 0, 'row': 1, 'level': 2}}


def test_web_goal_flow_and_run_gate(web):
    manager, call, monkeypatch, module = web
    status, state = call('/api/goal')
    assert status == 200 and state['active']['is_default'] and state['runnable'] and state['pending'] is None
    assert call('/api/goal', token_value='wrong')[0] == 401 and call('/api/goal/confirm', {}, token_value='wrong')[0] == 401
    status, checked = call('/api/goal/validate', {'cells': SQUARE_CELLS})
    assert status == 200 and checked['ok'] and not checked['is_default']
    status, state = call('/api/goal/confirm', {'cells': SQUARE_CELLS, 'source': 'manual'})
    assert status == 200 and not state['active']['is_default'] and state['active']['source'] == 'manual' and state['runnable']
    assert call('/api/status')[1]['goal']['runnable'] is True
    # Phase 5: a custom goal is runnable; the run job receives it (devices stubbed out).
    jobs = []
    monkeypatch.setattr(manager, 'validate_run_source', lambda: None)
    monkeypatch.setattr(manager, '_job', lambda action, body: jobs.append(action))
    status, reply = call('/api/action/run', {'scene_ready': True})
    assert status == 202 and jobs == ['run']
    manager.thread.join(2)
    monkeypatch.undo()
    status, state = call('/api/goal/reset', {})
    assert status == 200 and state['active']['is_default'] and state['runnable']
    bad = dict(SQUARE_CELLS, yellow={'col': 3, 'row': 3, 'level': 2})
    status, reply = call('/api/goal/confirm', {'cells': bad})
    assert status == 400 and '连续' in reply['error'] and call('/api/goal')[1]['active']['is_default']


def test_web_analyze_stores_pending_until_confirmed(web):
    manager, call, monkeypatch, module = web
    answer = {'image_usable': True, 'visible_colors': list(gs.COLORS), 'spacing': 'tight', 'viewpoint': 'oblique', 'confidence': .5,
              'uncertain': ['guess'], 'summary': 's', 'stacks': [{'col': 0, 'row': 0, 'blocks': ['red', 'green']}, {'col': 1, 'row': 0, 'blocks': ['orange', 'yellow']}]}
    calls = []

    other = dict(answer, stacks=[{'col': 0, 'row': 0, 'blocks': ['red']}, {'col': 1, 'row': 0, 'blocks': ['orange', 'yellow']}, {'col': 2, 'row': 0, 'blocks': ['green']}])

    def fake(jpeg_bytes, **kw):
        calls.append(kw)
        one = {'model': 'gemini-3.8-flash', 'tokens': 1500, 'cost_upper_bound_usd': .001, 'duration_s': 1.5, 'finish_reason': 'stop'}
        return {'results': [{**one, 'answer': answer}, {**one, 'answer': other}, {**one, 'answer': answer}], 'calls': 3,
                'duration_s': 1.6, 'tokens': 4500, 'cost_upper_bound_usd': .003}
    monkeypatch.setattr(goal_gemini, 'analyze_votes', fake)
    image = 'data:image/jpeg;base64,' + base64.b64encode(jpeg()).decode()
    status, state = call('/api/goal/analyze', {'image': image})
    assert status == 200 and calls[0]['key_file'] == module.GEMINI_KEY_FILE and calls[0]['proxy_url'] == module.GEMINI_PROXY
    pending = state['pending']
    assert pending['check']['ok'] and any('把握度' in n for n in pending['notes']) and state['active']['is_default']
    assert pending['agreement'] == 'majority' and [c['votes'] for c in pending['candidates']] == [2, 1] and any('不一致' in n for n in pending['notes'])
    assert pending['check']['digest'] == pending['candidates'][0]['check']['digest'] and calls[0]['votes'] == module.GOAL_VOTES
    status, raw = call('/api/goal/image/'+pending['image'])
    assert status == 200 and raw[:2] == b'\xff\xd8'
    assert call('/api/goal/image/..%2Factive.json')[0] == 404
    cells = {c: {'col': s['col'], 'row': s['row'], 'level': i+1} for s in pending['check']['spec']['stacks'] for i, c in enumerate(s['blocks'])}
    status, state = call('/api/goal/confirm', {'cells': cells, 'source': 'gemini', 'image': pending['image']})
    assert status == 200 and state['active']['source'] == 'gemini' and state['active']['image'] == pending['image'] and state['pending'] is None
    log = [json.loads(l) for l in (manager.goal_dir/'analyses.jsonl').read_text().splitlines()]
    assert len(log) == 1 and log[0]['ok'] and log[0]['called'] == 3 and 'key' not in json.dumps(log).lower()
    assert call('/api/goal/analyze', {'image': 'data:image/gif;base64,AAAA'})[0] == 400 and len(calls) == 1


def test_web_analyze_failure_is_reported_and_changes_nothing(web):
    manager, call, monkeypatch, module = web

    def boom(jpeg_bytes, **kw):
        return {'results': [{'error': 'TimeoutError: upstream timed out'}] * 3, 'calls': 3, 'duration_s': 60., 'tokens': 0, 'cost_upper_bound_usd': None}
    monkeypatch.setattr(goal_gemini, 'analyze_votes', boom)
    status, reply = call('/api/goal/analyze', {'image': 'data:image/jpeg;base64,' + base64.b64encode(jpeg()).decode()})
    assert status == 409 and 'Gemini 解析失败' in reply['error']
    state = call('/api/goal')[1]
    assert state['pending'] is None and state['active']['is_default']


def test_tally_groups_quarter_turns_and_reports_agreement():
    ell = {'spacing': 'tight', 'stacks': [{'col': 1, 'row': 0, 'blocks': ['orange']}, {'col': 1, 'row': 1, 'blocks': ['green', 'red']}, {'col': 0, 'row': 1, 'blocks': ['yellow']}]}
    turned = {'spacing': 'tight', 'stacks': [{'col': 0, 'row': 0, 'blocks': ['orange']}, {'col': 1, 'row': 0, 'blocks': ['green', 'red']}, {'col': 1, 'row': 1, 'blocks': ['yellow']}]}
    mirror = {'spacing': 'tight', 'stacks': [{'col': 0, 'row': 0, 'blocks': ['orange']}, {'col': 0, 'row': 1, 'blocks': ['green', 'red']}, {'col': 1, 'row': 1, 'blocks': ['yellow']}]}
    wrong = {'spacing': 'tight', 'stacks': [{'col': 0, 'row': 0, 'blocks': ['yellow']}, {'col': 1, 'row': 0, 'blocks': ['orange']}, {'col': 1, 'row': 1, 'blocks': ['green', 'red']}]}
    assert gs.rotation_key(gs.check(ell)['spec']) == gs.rotation_key(gs.check(turned)['spec'])
    assert gs.rotation_key(gs.check(ell)['spec']) != gs.rotation_key(gs.check(mirror)['spec'])
    assert gs.tally([ell, turned, ell])[1] == 'unanimous'
    candidates, agreement = gs.tally([ell, wrong, turned])
    assert agreement == 'majority' and [c['votes'] for c in candidates] == [2, 1]
    assert gs.tally([ell, wrong, mirror])[1] == 'split' and gs.tally([{'stacks': []}, 'x'])[1] == 'none'


def test_contact_list_is_checked_against_the_grid():
    spec = gs.check(gs.DEFAULT_SPEC)['spec']
    said = [{'a': 'green', 'b': 'red', 'relation': 'on_top'}, {'a': 'yellow', 'b': 'green', 'relation': 'on_top'}, {'a': 'red', 'b': 'orange', 'relation': 'side_by_side'}]
    assert gs.contact_mismatch(spec, said) == []
    notes = gs.contact_mismatch(spec, said[:2] + [{'a': 'green', 'b': 'orange', 'relation': 'side_by_side'}])
    assert len(notes) == 2 and any('绿和橙' in n or '橙和绿' in n for n in notes) and any('红' in n and '橙' in n for n in notes)
    assert gs.contact_mismatch(spec, None) == []



def test_run_no_longer_requires_a_prepare_click_and_connecting_keeps_the_goal(web):
    manager, call, monkeypatch, module = web
    assert call('/api/goal/confirm', {'cells': {'red': {'col': 0, 'row': 0, 'level': 1}, 'green': {'col': 0, 'row': 0, 'level': 2},
                                                 'yellow': {'col': 0, 'row': 0, 'level': 3}, 'orange': {'col': 1, 'row': 0, 'level': 1}}, 'source': 'manual'})[0] == 200
    digest = call('/api/goal')[1]['active']['digest']
    jobs = []
    monkeypatch.setattr(manager, 'validate_run_source', lambda: None)
    monkeypatch.setattr(manager, '_job', lambda action, body: jobs.append(action))
    # Never prepared in this server, nothing connected: the run is accepted (the controller checks the start itself).
    assert manager.prepared is False
    status, reply = call('/api/action/run', {'scene_ready': True})
    assert status == 202 and reply['accepted'] and jobs == ['run']
    manager.thread.join(2)
    # Connected, live feedback away from neutral: refused before anything starts, with the measured values.

    class Live:
        def poll(self): return None
    manager.process, manager.out = Live(), manager.data
    monkeypatch.setattr(manager, 'owner', lambda: {'robot_state': {'values': [.3] + [0.] * 5 + [.07] + [0.] * 7}})
    status, reply = call('/api/action/run', {'scene_ready': True})
    assert status == 400 and '零位' in reply['error'] and jobs == ['run']
    # Connected at neutral with a closed jaw, without any Prepare in this server: accepted.
    monkeypatch.setattr(manager, 'owner', lambda: {'robot_state': {'values': [0.002] * 6 + [.002] + [0.] * 7}})
    status, reply = call('/api/action/run', {'scene_ready': True})
    assert status == 202 and jobs == ['run', 'run']
    manager.thread.join(2)
    # Feedback that cannot be read does not block: the controller start check decides.
    monkeypatch.setattr(manager, 'owner', lambda: (_ for _ in ()).throw(RuntimeError('owner busy')))
    assert call('/api/action/run', {'scene_ready': True})[0] == 202
    manager.thread.join(2)
    manager.process = None
    # The operator confirmation is still required.
    assert call('/api/action/run', {'scene_ready': False})[0] == 400
    # Reconnecting hardware (a real connect job with devices stubbed) leaves the confirmed goal alone.
    monkeypatch.undo()
    monkeypatch.setattr(manager, 'launch', lambda *a, **k: None)
    monkeypatch.setattr(manager, 'wait', lambda *a, **k: True)
    manager._job('connect', {})
    assert manager.phase == '只读预览' and manager.error is None
    assert call('/api/goal')[1]['active']['digest'] == digest and call('/api/status')[1]['goal']['digest'] == digest



def test_confirmed_goal_is_handed_to_the_session_and_never_left_behind(web, tmp_path):
    manager, call, monkeypatch, module = web
    from services.dashboard import goal_spec as gs
    manager.candidate = tmp_path/'candidate'; (manager.candidate/'code/magnetic_blocks').mkdir(parents=True)
    session = tmp_path/'session'; (session/'code/magnetic_blocks').mkdir(parents=True)
    manager.out = tmp_path/'out'; manager.active_record = {'id': 'x'}
    call('/api/goal/confirm', {'cells': SQUARE_CELLS})
    goal = manager.goal_active()
    manager.goal_write(goal)
    written = manager.goal_file()
    assert gs.check(json.loads(written.read_text()))['digest'] == goal['digest'] and not goal['is_default']
    # The launcher copies the code tree: simulate it, then verify and clean up.
    (session/'code/magnetic_blocks/GOAL.json').write_text(written.read_text())
    manager.goal_handed_over({'workspace': str(session)}, goal)
    assert not written.exists() and manager.active_record['goal_handed_over'] is True
    # A session copy that differs (or is missing) stops the run before the controller starts.
    manager.goal_write(goal)
    (session/'code/magnetic_blocks/GOAL.json').write_text(json.dumps(gs.DEFAULT_SPEC))
    with pytest.raises(RuntimeError, match='不一致'):
        manager.goal_handed_over({'workspace': str(session)}, goal)
    assert not written.exists()
    (session/'code/magnetic_blocks/GOAL.json').unlink()
    with pytest.raises(RuntimeError, match='不一致'):
        manager.goal_handed_over({'workspace': str(session)}, goal)
    # The default goal is written explicitly too, so every run uses the same path.
    call('/api/goal/reset', {})
    default = manager.goal_active(); manager.goal_write(default)
    assert gs.check(json.loads(written.read_text()))['is_default']
    # A tampered confirmed goal is refused before anything is launched.
    with pytest.raises(RuntimeError, match='校验'):
        manager.goal_write(dict(default, digest='0' * 64))


def test_controller_reads_the_written_goal_file(tmp_path):
    """The file the web writes is exactly what task_spec.py loads."""
    import importlib.util, shutil, sys as _sys
    code = Path(__file__).resolve().parents[1]/'runtime/workspaces/magnetic-blocks-l-stack/r9-ssik-speed/code/magnetic_blocks'
    if not code.exists():
        pytest.skip('controller code not next to this test')
    for f in ('task_spec.py', 'goal_spec.py'):
        shutil.copy(code/f, tmp_path/f)
    spec = gs.check({'stacks': [{'col': 1, 'row': 0, 'blocks': ['orange']}, {'col': 0, 'row': 1, 'blocks': ['yellow']}, {'col': 1, 'row': 1, 'blocks': ['green', 'red']}], 'spacing': 'loose'})
    (tmp_path/'GOAL.json').write_text(json.dumps(spec['spec'], ensure_ascii=False))
    _sys.path.insert(0, str(tmp_path)); _sys.modules.pop('goal_spec', None)
    try:
        m = importlib.util.module_from_spec(importlib.util.spec_from_file_location('ts_probe', tmp_path/'task_spec.py'))
        m.__spec__.loader.exec_module(m)
    finally:
        _sys.path.remove(str(tmp_path)); _sys.modules.pop('goal_spec', None)
    assert m.GOAL_SOURCE == 'file' and m.GOAL_DIGEST == spec['digest'] and m.FINAL_COLOR == 'red' and m.SUPPORTS == {'red': 'green'}
