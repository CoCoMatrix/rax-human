import base64
import copy
import io
import json
from pathlib import Path
import urllib.error

import numpy as np
from PIL import Image
import pytest

from real_robot.piper.teacher_client import (PiperTeacherClient, TeacherClientError,
    load_api_key, MODEL, ENDPOINT, PROXY, BOTTLE_TASK, BOTTLE_PLACE_TASK, STUDENT_TASK,
    INSTRUCTIONS, instructions_for_goal)
from real_robot.piper.tests.test_teacher_contract import BINDING, response

KEY = 'sk-' + 'test-fixture-only-' * 3


def inputs():
    binding = dict(BINDING, task=STUDENT_TASK)
    images = {key: np.zeros((32, 64, 3), np.uint8) for key in ('camera_head', 'camera_left', 'camera_right')}
    images['camera_head'][0, 0] = [231, 17, 41]
    context = dict(task=STUDENT_TASK, user_goal=BOTTLE_TASK, step_id=0,
                   measured_state14=[0.0]*14, student_actions=[[0.0]*14 for _ in range(32)],
                   current_flange={}, student_flange_trajectory=[{'step': i} for i in range(32)],
                   previous_result=None)
    return dict(binding=binding, images=images, context=context)


def api_response():
    decision = response(); decision['binding']['task'] = STUDENT_TASK
    decision['binding']['observation_timestamp_ns'] = str(decision['binding']['observation_timestamp_ns'])
    return dict(id='resp_test_123', model=MODEL, status='completed', error=None, incomplete_details=None,
        usage=dict(input_tokens=100, output_tokens=20, total_tokens=120,
                   input_tokens_details=dict(cached_tokens=10), output_tokens_details=dict(reasoning_tokens=5)),
        output=[dict(type='reasoning', summary=[]),
                dict(type='message', role='assistant', status='completed',
                     content=[dict(type='output_text', text=json.dumps(decision))])])


class FakeResponse:
    status = 200
    def __init__(self, value): self.body = json.dumps(value).encode()
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def read(self, size): return self.body[:size]


class FakeOpener:
    def __init__(self, value=None, error=None):
        self.value = api_response() if value is None else value
        self.error, self.requests = error, []
    def open(self, request, timeout):
        self.requests.append((request, timeout))
        if self.error: raise self.error
        return FakeResponse(self.value)


def client(opener, **kwargs):
    return PiperTeacherClient(opener=opener, key_loader=lambda _: KEY, **kwargs)


def test_real_request_shape_rgb_roundtrip_goal_override_and_sanitized_report():
    opener = FakeOpener()
    result = client(opener).review(**inputs())
    request, timeout = opener.requests[0]
    body = json.loads(request.data)
    assert request.full_url == ENDPOINT and request.method == 'POST' and timeout == 60
    assert request.get_header('Authorization') == 'Bearer ' + KEY
    assert body['model'] == MODEL and body['reasoning'] == {'effort': 'low'}
    assert body['max_output_tokens'] == 2048 and body['store'] is False
    assert body['text']['format']['type'] == 'json_schema' and body['text']['format']['strict'] is True
    content = body['input'][0]['content']
    assert len([x for x in content if x['type'] == 'input_image']) == 3
    first_image = next(x for x in content if x['type'] == 'input_image')
    decoded = Image.open(io.BytesIO(base64.b64decode(first_image['image_url'].split(',', 1)[1])))
    assert decoded.mode == 'RGB' and np.asarray(decoded)[0, 0].tolist() == [231, 17, 41]
    assert result['decision']['mode'] == 'student' and 'target' not in result['decision']
    report = result['report']
    assert report['returned_model'] == MODEL and report['response_id'] == 'resp_test_123'
    assert report['usage']['total_tokens'] == 120 and report['api_attempts'] == 1
    assert report['user_goal'] == BOTTLE_TASK and report['student_conditioning'] == STUDENT_TASK
    assert report['identical_task_to_original_method'] is False and report['dispatch_authorized'] is False
    assert KEY not in json.dumps(result) and KEY not in request.data.decode()
    assert 'do not\nexecute placement on a coaster' in INSTRUCTIONS


@pytest.mark.parametrize('mutate,code', [
    (lambda r: r.update(model='gpt-wrong-model'), 'model_identity_mismatch'),
    (lambda r: r.update(status='incomplete', incomplete_details={'reason': 'max_output_tokens'}), 'response_not_completed'),
    (lambda r: r['output'][-1].update(content=[{'type': 'refusal', 'refusal': 'No.'}]), 'model_refusal'),
    (lambda r: r['output'][-1]['content'][0].update(text='```json\n{}\n```'), 'invalid_structured_decision'),
    (lambda r: r['output'].append({'type': 'function_call', 'name': 'exec'}), 'unexpected_output_or_tool_call'),
    (lambda r: r.update(id='wrong id'), 'invalid_response_id'),
])
def test_rejects_wrong_model_truncation_refusal_invalid_json_and_tools(mutate, code):
    api = api_response(); mutate(api)
    opener = FakeOpener(api)
    with pytest.raises(TeacherClientError) as error: client(opener).review(**inputs())
    assert error.value.code == code and len(opener.requests) == 1
    assert KEY not in str(error.value) and KEY not in json.dumps(error.value.report)


def test_http_error_never_reads_or_reports_error_body_or_retries():
    error = urllib.error.HTTPError(ENDPOINT, 401, 'secret=' + KEY, {}, io.BytesIO(KEY.encode()))
    opener = FakeOpener(error=error)
    with pytest.raises(TeacherClientError) as caught: client(opener).review(**inputs())
    assert caught.value.code == 'http_error' and caught.value.report['http_status'] == 401
    assert len(opener.requests) == 1 and KEY not in str(caught.value)
    assert KEY not in json.dumps(caught.value.report)
    assert caught.value.__suppress_context__


def test_transport_error_sanitized_and_not_retried():
    opener = FakeOpener(error=urllib.error.URLError(KEY))
    with pytest.raises(TeacherClientError) as caught: client(opener).review(**inputs())
    assert caught.value.code == 'transport_error' and len(opener.requests) == 1
    assert KEY not in str(caught.value) and KEY not in json.dumps(caught.value.report)


def test_inputs_rejected_before_key_load_or_network():
    opener = FakeOpener()
    def forbidden(_): raise AssertionError('should not load any key')
    candidate = PiperTeacherClient(opener=opener, key_loader=forbidden)
    wrong = inputs(); wrong['images']['camera_head'] = wrong['images']['camera_head'].astype(float)
    with pytest.raises(TeacherClientError, match='invalid_rgb_array'): candidate.review(**wrong)
    wrong = inputs(); wrong['context']['task'] = BOTTLE_TASK
    with pytest.raises(TeacherClientError, match='task_or_step'): candidate.review(**wrong)
    wrong = inputs(); wrong['context']['student_actions'] = [[0.0]*14]
    with pytest.raises(TeacherClientError, match='full_proposal'): candidate.review(**wrong)
    assert not opener.requests


def test_key_loader_reads_plaintext_never_python_code(tmp_path):
    path = tmp_path / 'key.py'
    path.write_text(KEY + '\n')
    assert load_api_key(path) == KEY
    path.write_text("raise RuntimeError('do not execute')")
    with pytest.raises(TeacherClientError, match='invalid_key_format'): load_api_key(path)
    path.write_text(KEY + '\n' + KEY)
    with pytest.raises(TeacherClientError, match='invalid_key_format'): load_api_key(path)
    with pytest.raises(TeacherClientError, match='key_unavailable'): load_api_key(tmp_path / 'missing')


def test_per_client_proxy_is_explicit_and_global_opener_not_replaced(monkeypatch):
    import urllib.request
    calls = []
    original = urllib.request._opener
    sentinel = object()
    def build(*handlers): calls.extend(handlers); return sentinel
    monkeypatch.setattr(urllib.request, 'build_opener', build)
    value = PiperTeacherClient()
    assert value.opener is sentinel
    proxy = next(x for x in calls if isinstance(x, urllib.request.ProxyHandler))
    assert proxy.proxies == {'http': PROXY, 'https': PROXY}
    assert urllib.request._opener is original


def test_response_deadline_overrun_rejected_even_if_http_completed():
    times = iter([0, 61, 61])
    opener = FakeOpener()
    with pytest.raises(TeacherClientError, match='deadline_exceeded'):
        client(opener, monotonic=lambda: next(times)).review(**inputs())
    assert len(opener.requests) == 1


def test_xhigh_long_request_keeps_a_bounded_deadline():
    opener = FakeOpener()
    result = client(opener, reasoning_effort='xhigh', timeout=300).review(**inputs())
    assert opener.requests[0][1] == 300
    assert result['report']['timeout_s'] == 300
    with pytest.raises(TeacherClientError, match='invalid_timeout'):
        client(FakeOpener(), reasoning_effort='xhigh', timeout=301)
    with pytest.raises(TeacherClientError, match='invalid_timeout'):
        client(FakeOpener(), timeout=61)


def test_place_goal_reaches_request_and_report_without_pickup_only_stop():
    opener = FakeOpener()
    request = inputs()
    request['context']['user_goal'] = BOTTLE_PLACE_TASK
    result = client(opener).review(**request)
    body = json.loads(opener.requests[0][0].data)
    assert BOTTLE_PLACE_TASK in body['instructions']
    assert 'PICKUP ONLY' not in body['instructions']
    assert 'do not\nexecute placement' not in body['instructions']
    assert 'supported on the coaster after gripper release and retreat' in body['instructions']
    assert result['report']['user_goal'] == BOTTLE_PLACE_TASK
    assert instructions_for_goal(BOTTLE_TASK) == INSTRUCTIONS


def test_unknown_goal_rejected_before_network():
    opener = FakeOpener()
    request = inputs()
    request['context']['user_goal'] = 'Arbitrary unsupported motion'
    with pytest.raises(TeacherClientError, match='task_or_step'):
        client(opener).review(**request)
    assert not opener.requests
    with pytest.raises(TeacherClientError, match='unsupported_user_goal'):
        instructions_for_goal(request['context']['user_goal'])


def test_large_nanosecond_identity_uses_exact_decimal_string_on_api_wire():
    stamp = 1789706902945987268
    request = inputs()
    request['binding']['observation_timestamp_ns'] = stamp
    api = api_response()
    decision = json.loads(api['output'][-1]['content'][0]['text'])
    decision['binding']['observation_timestamp_ns'] = str(stamp)
    api['output'][-1]['content'][0]['text'] = json.dumps(decision)
    opener = FakeOpener(api)
    result = client(opener).review(**request)
    body = json.loads(opener.requests[0][0].data)
    field = body['text']['format']['schema']['properties']['binding']['properties']['observation_timestamp_ns']
    assert field == {'type': 'string', 'enum': [str(stamp)]}
    assert result['decision']['binding']['observation_timestamp_ns'] == stamp
    assert type(result['decision']['binding']['observation_timestamp_ns']) is int
    assert result['api_decision']['binding']['observation_timestamp_ns'] == str(stamp)


def test_rounded_timestamp_is_rejected_not_tolerated_or_relabelled():
    stamp = 1789706902945987268
    request = inputs()
    request['binding']['observation_timestamp_ns'] = stamp
    api = api_response()
    decision = json.loads(api['output'][-1]['content'][0]['text'])
    decision['binding']['observation_timestamp_ns'] = str(int(float(stamp)))
    api['output'][-1]['content'][0]['text'] = json.dumps(decision)
    opener = FakeOpener(api)
    with pytest.raises(TeacherClientError) as error:
        client(opener).review(**request)
    assert error.value.report['validation_rule'] == 'teacher binding differs from reviewed proposal'
    assert error.value.report['received_binding']['observation_timestamp_ns'] != str(stamp)
