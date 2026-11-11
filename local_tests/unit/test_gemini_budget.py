"""The session's Gemini dollar ledger: admission before I/O, retained reservations, restarts."""
from concurrent.futures import ThreadPoolExecutor
import json

import pytest

from conftest import configuration, frontend
from services.controller.billing import NANODOLLARS, SESSION_LIMIT, nanodollars, reserve_cost
from services.controller.gemini import GeminiRouter, MODEL
from services.controller.session import Session


def response(cost=.001):
    return {'model': MODEL, 'choices': [{'message': {'role': 'assistant', 'content': 'OK'}, 'finish_reason': 'stop'}],
            'usage': {'total_tokens': 10, 'cost': cost}}


def call(front):
    return front.call('gemini_generate', {'messages': [{'role': 'user', 'content': 'Hello'}], 'max_tokens': 128})


def failure(front, stage):
    reply = call(front)
    assert reply['isError'] is True
    error = json.loads(reply['content'][0]['text'])['error']
    assert error['stage'] == stage and error['reason']
    assert not error['control_uncertain']
    return error


def test_admission_prevents_crossing_cap_before_provider_io(tmp_path):
    calls = []
    f = frontend(tmp_path, GeminiRouter('host-key', send=lambda payload: calls.append(payload) or response()))
    try:
        f.s.charge(100, SESSION_LIMIT)
        f.s.refund(90, {'total_tokens': 10, 'cost': 9.999}, SESSION_LIMIT)
        failure(f, 'api_budget')
        assert not calls
        assert f.s.budget()['session_cost_remaining_usd'] == .001
        # The dollar cutoff leaves robot control available.
        assert not f.call('mock_move', {'position': [0.1, 0, 0]}).get('isError')
    finally:
        f.close()


@pytest.mark.parametrize('cost', [None, False, -1, '0.001', float('nan'), float('inf')])
def test_untrusted_cost_retains_dollar_reservation(tmp_path, cost):
    f = frontend(tmp_path, GeminiRouter('host-key', send=lambda _: response(cost)))
    try:
        call(f)
        budget = f.s.budget()
        assert budget['session_cost_reported_usd'] == 0
        assert budget['session_cost_reserved_usd'] > 0
        assert budget['tokens_reported'] == 10
    finally:
        f.close()


def test_timeout_reservation_survives_restart(tmp_path):
    def fail(_):
        raise TimeoutError('unknown billing outcome')
    f = frontend(tmp_path, GeminiRouter('host-key', send=fail))
    assert 'timed out' in failure(f, 'provider_request')['reason']
    before = f.s.budget()
    assert before['session_cost_reserved_usd'] > 0
    f.close()
    restored = Session(configuration(tmp_path))
    try:
        assert restored.budget() == before
    finally:
        restored.close()


def test_reported_cost_bound_violation_closes_gemini(tmp_path):
    calls = []
    f = frontend(tmp_path, GeminiRouter('host-key', send=lambda payload: calls.append(payload) or response(1)))
    try:
        failure(f, 'api_accounting')
        assert f.s.budget()['session_cost_reported_usd'] == 1
        assert f.s.budget()['session_cost_blocked'] is True
        failure(f, 'api_budget')
        assert len(calls) == 1
    finally:
        f.close()


def test_concurrent_reservations_cannot_oversubscribe(tmp_path):
    s = Session(configuration(tmp_path))
    try:
        def reserve(_):
            try:
                s.charge(1, SESSION_LIMIT // 4)
                return True
            except RuntimeError:
                return False
        with ThreadPoolExecutor(16) as pool:
            assert sum(pool.map(reserve, range(16))) == 4
        assert s.budget()['session_cost_remaining_usd'] == 0
    finally:
        s.close()


def test_exact_decimal_arithmetic_and_maximum_price_bound():
    assert nanodollars(0.1) + nanodollars(0.2) == nanodollars(0.3)
    assert 0 < reserve_cost(1128, 128) < reserve_cost(2128, 128) < reserve_cost(2128, 1128)
    assert SESSION_LIMIT == 10 * NANODOLLARS


def test_one_robot_owner_per_session(tmp_path):
    s = Session(configuration(tmp_path))
    try:
        with pytest.raises(RuntimeError, match='robot owner'):
            Session(configuration(tmp_path))
    finally:
        s.close()
