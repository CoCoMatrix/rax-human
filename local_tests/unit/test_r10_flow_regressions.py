import base64
import io
import json
import tomllib

import pytest
from PIL import Image
from conftest import frontend, configuration
from services.controller import operator
from services.controller.gemini import GeminiRouter
from services.controller.google_gemini import GoogleGeminiRouter
from services.controller.billing import reserve_cost, NANODOLLARS
from harness.codex_cli.launcher import codex_config


def value(result):
    return json.loads(result['content'][0]['text'])


def test_native_compaction_is_operator_configured_without_rewriting_provider(tmp_path):
    c = configuration(tmp_path, agent_auto_compact_token_limit=65536)
    cfg = tomllib.loads(codex_config(c))
    assert cfg['model_auto_compact_token_limit'] == 65536
    assert 'model_context_window' not in cfg  # Do not invent model capacity.
    assert 'compact' not in codex_config(configuration(tmp_path))


@pytest.mark.parametrize('bad', [0, -1, True, 1.5])
def test_invalid_native_compaction_setting_is_rejected(tmp_path, bad):
    with pytest.raises(ValueError, match='compact'):
        configuration(tmp_path, agent_auto_compact_token_limit=bad)


@pytest.mark.parametrize('router_cls', [GeminiRouter, GoogleGeminiRouter])
def test_four_native_images_fit_budget_without_resizing(router_cls):
    f = io.BytesIO(); Image.new('RGB', (1280, 720)).save(f, 'JPEG')
    encoded = base64.b64encode(f.getvalue()).decode()
    image = {'type': 'image', 'mimeType': 'image/jpeg', 'data': encoded}
    args = {'messages': [{'role': 'user', 'content': [image] * 4}], 'max_tokens': 1600}
    payload, tokens = router_cls.__new__(router_cls).prepare(args)
    assert reserve_cost(tokens, 1600) / NANODOLLARS < 1
    for part in payload['messages'][0]['content']:
        assert part['image_url']['url'].split(',', 1)[1] == encoded
    assert tokens > 4 * 2240 + 1600  # Retain generous reservation, not actual billing.


def test_late_answer_arrives_on_next_nonstatus_tool_and_not_repeated(tmp_path):
    f = frontend(tmp_path)
    try:
        r = operator.open_request(f.s.config.root, 'help', 'Did the cup lift?')
        operator.reply(f.s.config.root, r['id'], 'No lift; cup is table supported.', by='onsite')
        out = value(f.call('unknown', {}))['error']
        update = out['operator_updates'][0]
        assert update['answer']['reply'].startswith('No lift')
        assert update['motion_authorized'] is False
        assert update['requires_fresh_observation'] is True
        assert 'operator_updates' not in value(f.call('unknown', {}))['error']
        assert value(f.call('status', {}))['operator_requests'][0]['answer']
        assert f.backend.position == [0, 0, 0]
    finally:
        f.close()


def test_answer_during_tool_is_delivered_without_clearing_control_fault(tmp_path):
    f = frontend(tmp_path)
    try:
        r = operator.open_request(f.s.config.root, 'help', 'support?')
        f.control_uncertain = True
        original = f.backend.status
        def status():
            operator.reply(f.s.config.root, r['id'], 'Supported.', by='onsite')
            return original()
        f.backend.status = status
        out = value(f.call('status', {}))
        assert out['operator_updates'][0]['request_id'] == r['id']
        assert out['control_uncertain'] is True
    finally:
        f.close()


def test_existing_question_supports_zero_wait_poll_no_new_question(tmp_path):
    f = frontend(tmp_path)
    try:
        r = operator.open_request(f.s.config.root, 'help', 'support?')
        out = value(f.call('request_operator', {'request_id': r['id'], 'timeout_s': 0}))
        assert out['status'] == 'timeout' and out['done'] is False
        operator.reply(f.s.config.root, r['id'], 'Supported.', by='onsite')
        out = value(f.call('request_operator', {'request_id': r['id'], 'timeout_s': 0}))
        assert out['status'] == 'answered' and out['late'] is True
        assert len(list(operator.directory(f.s.config.root).glob('????????????.json'))) == 1
        assert f.call('request_operator', {'kind': 'help', 'message': 'x', 'timeout_s': float('nan')})['isError']
    finally:
        f.close()


def test_budget_refusal_explains_reservation_without_spending(tmp_path):
    f = frontend(tmp_path)
    try:
        with pytest.raises(RuntimeError, match=r'required=.*remaining='):
            f.s.charge(20_000_000, 11 * NANODOLLARS)
        assert f.s.budget()['calls_used'] == 0
        assert f.s.budget()['session_cost_remaining_usd'] == 10
    finally:
        f.close()


def test_reply_record_failure_does_not_erase_completed_motion_receipt(tmp_path):
    f = frontend(tmp_path)
    try:
        r = operator.open_request(f.s.config.root, 'help', 'support?')
        path = operator.directory(f.s.config.root) / (r['id'] + '.json')
        path.write_text('{"id": "missing-created"}')
        result = f.call('mock_move', {'position': [0.1, 0, 0]})
        assert not result.get('isError')
        out = value(result)
        assert out['position'] == [0.1, 0, 0]
        assert out['operator_updates_error'] and not f.control_uncertain
        error = value(f.call('unknown', {}))['error']
        assert error['type'] == 'PermissionError' and error['operator_updates_error']
    finally:
        f.close()
