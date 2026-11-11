import http.client
import json
import socket
import threading

import pytest

from harness.codex_cli.relay import ResponseTrace, ResponsesRelay, TRACE_LIMIT


def event(kind, **values):
    return ('event: ' + kind + '\r\ndata: ' + json.dumps(
        {'type': kind, **values}, ensure_ascii=False) + '\r\n\r\n').encode()


def feed(trace, chunk):
    trace.feed(chunk, trace.received(chunk))


def completed():
    return {'id': 'resp_test', 'model': 'gpt-6-astra', 'status': 'completed',
            'output': [{'type': 'reasoning', 'encrypted_content': 'PRIVATE_REASONING'},
                       {'type': 'message', 'content': [{'type': 'output_text', 'text': 'PRIVATE_OUTPUT'}]}],
            'usage': {'input_tokens': 120, 'output_tokens': 7, 'total_tokens': 127,
                      'input_tokens_details': {'cached_tokens': 100},
                      'output_tokens_details': {'reasoning_tokens': 2},
                      'attribution': {'secret': 'PRIVATE_ATTRIBUTION'}}}


@pytest.mark.parametrize('chunk_size', [1, 7, 65536])
def test_sse_chunk_boundaries_timing_and_metadata_only(chunk_size):
    now = [10.]
    trace = ResponseTrace(10., 'sk-private', clock=lambda: now[0])
    trace.request({'input': [{'content': [{'type': 'input_text', 'text': 'PRIVATE_PROMPT'},
                          {'type': 'input_image', 'image_url': 'data:image/png;base64,PRIVATE_IMAGE'}]},
                          {'type': 'reasoning', 'encrypted_content': 'PRIVATE_INPUT_REASONING'}]}, 999, 1050)
    trace.mark('upstream_open_started')
    now[0] = 11.
    trace.headers(200, 'text/event-stream', 'req_provider')
    chunks = [event('response.created', response={'id': 'resp_test', 'model': 'gpt-6-astra'}),
              event('response.output_text.delta', delta='私有PRIVATE_OUTPUT'),
              event('response.function_call_arguments.delta', delta='PRIVATE_ARGUMENTS'),
              event('response.completed', response=completed())]
    for at, raw in zip([12., 13., 14., 15.], chunks):
        now[0] = at
        for i in range(0, len(raw), chunk_size): feed(trace, raw[i:i+chunk_size])
    now[0] = 16.; trace.finish()
    result = trace.result()
    assert result['ok'] and result['transport_complete']
    assert result['timings_s'] == {'upstream_open_started': 0., 'response_headers': 1.,
        'first_byte': 2., 'first_sse_event': 2., 'first_output_delta': 3.,
        'first_tool_delta': 4., 'response_completed': 5., 'stream_end': 6.}
    assert result['response_id'] == 'resp_test' and result['response_model'] == 'gpt-6-astra'
    assert result['provider_request_id'] == 'req_provider'
    assert result['input_items'] == 2 and result['input_images'] == 1
    assert result['request_bytes'] == 999 and result['forwarded_request_bytes'] == 1050
    assert result['usage']['input_tokens_details'] == {'cached_tokens': 100}
    assert result['usage']['output_tokens_details'] == {'reasoning_tokens': 2}
    assert 'PRIVATE_' not in json.dumps(result) and 'sk-private' not in json.dumps(result)


@pytest.mark.parametrize('raw', [
    event('response.output_text.delta', delta='partial') + b'data: [DONE]\n\n',
    event('response.failed', response={'status': 'failed', 'error': {'message': 'PRIVATE_ERROR'}}),
    event('response.incomplete', response={'status': 'incomplete'}),
    b'data: {broken json}\n\n',
    b'data: {"type":"response.completed","response":{"status":"completed"}',
])
def test_partial_failed_or_malformed_stream_is_never_success(raw):
    trace = ResponseTrace(0., 'key', clock=lambda: 1.)
    trace.headers(200, 'text/event-stream'); feed(trace, raw); trace.finish()
    assert trace.result()['ok'] is False
    assert trace.result()['transport_complete'] is True
    assert 'PRIVATE_ERROR' not in json.dumps(trace.result())


def test_complete_event_then_disconnect_is_not_successful_transport():
    trace = ResponseTrace(0., 'key', clock=lambda: 1.)
    trace.headers(200, 'text/event-stream')
    feed(trace, event('response.completed', response=completed()))
    assert trace.result()['response_complete_observed']
    assert trace.result()['ok'] is False and trace.result()['timings_s']['stream_end'] is None


def test_sse_multiline_cr_and_missing_final_separator():
    trace = ResponseTrace(0., 'key', clock=lambda: 1.)
    trace.headers(200, 'text/event-stream')
    raw = b'event: response.completed\rdata: {"type":"response.completed",\rdata: "response":{"status":"completed"}}'
    for byte in raw: feed(trace, bytes([byte]))
    trace.finish()
    assert trace.result()['ok']


def test_oversized_sse_event_is_skipped_but_forward_observer_recovers():
    trace = ResponseTrace(0., 'key', clock=lambda: 1.)
    trace.headers(200, 'text/event-stream')
    feed(trace, b'data: ')
    for _ in range(20):
        feed(trace, b'x' * 65536)
        assert len(trace.pending) <= TRACE_LIMIT and len(trace.data) <= TRACE_LIMIT
    feed(trace, b'\n\n')
    feed(trace, event('response.completed', response=completed()))
    trace.finish()
    assert trace.result()['diagnostic_limit_exceeded'] and trace.result()['ok']


@pytest.mark.parametrize('compact', [False, True])
def test_json_fragments_complete_only_at_eof_and_compaction(compact):
    trace = ResponseTrace(0., 'sk-private', clock=lambda: 1.)
    trace.headers(200, 'application/json', 'sk-private')
    value = completed()
    if compact:
        value.pop('status'); value['object'] = 'response.compaction'
    raw = json.dumps(value).encode()
    for i in range(0, len(raw), 3): feed(trace, raw[i:i+3])
    assert not trace.result()['ok']
    trace.finish()
    assert trace.result()['ok'] and trace.result()['provider_request_id'] is None
    assert trace.result()['timings_s']['first_sse_event'] is None
    assert 'PRIVATE_' not in json.dumps(trace.result())


def test_oversized_json_remains_unverified_with_bounded_memory():
    trace = ResponseTrace(0., 'key', clock=lambda: 1.)
    trace.headers(200, 'application/json')
    for _ in range(20):
        feed(trace, b'x' * 65536)
        assert len(trace.pending) <= TRACE_LIMIT
    trace.finish()
    assert trace.result()['diagnostic_limit_exceeded'] and not trace.result()['ok']


def test_oversized_completion_is_not_inferred_from_sse_event_name():
    trace = ResponseTrace(0., 'key', clock=lambda: 1.)
    trace.headers(200, 'text/event-stream')
    raw = event('response.completed', response={**completed(), 'output': ['x' * (TRACE_LIMIT+1)]})
    for i in range(0, len(raw), 65536): feed(trace, raw[i:i+65536])
    trace.finish()
    assert trace.result()['diagnostic_limit_exceeded']
    assert not trace.result()['response_complete_observed'] and not trace.result()['ok']


def test_provider_metadata_cannot_echo_key_or_arbitrary_usage_payload():
    trace = ResponseTrace(0., 'sk-private', clock=lambda: 1.)
    trace.headers(200, 'application/json', 'req_sk-private')
    value = {**completed(), 'id': 'resp_sk-private', 'model': 'sk-private',
             'usage': {'input_tokens': 'PRIVATE_PROMPT', 'total_tokens': True,
                       'output_tokens': 4, 'output_tokens_details': {'reasoning_tokens': 'PRIVATE_REASONING'}}}
    feed(trace, json.dumps(value).encode()); trace.finish()
    result = trace.result()
    assert result['ok'] and result['response_id'] is None and result['response_model'] is None
    assert result['provider_request_id'] is None
    assert result['usage'] == {'output_tokens': 4, 'output_tokens_details': {}}
    assert 'PRIVATE_' not in json.dumps(result) and 'sk-private' not in json.dumps(result)


def test_relay_forwards_before_diagnostics_and_never_logs_payload(tmp_path, monkeypatch):
    first = event('response.created', response={'id': 'resp_test'})
    second = event('response.completed', response=completed())
    entered, release, logged = threading.Event(), threading.Event(), threading.Event()
    original_feed = ResponseTrace.feed

    def gated_feed(self, chunk, at):
        if chunk == first:
            entered.set()
            assert release.wait(5), 'test did not release diagnostic observer'
        original_feed(self, chunk, at)

    class Reply:
        status = 200
        headers = {'Content-Type': 'text/event-stream', 'x-request-id': 'req_provider'}
        def __init__(self): self.chunks = iter([first, second, b''])
        def __enter__(self): return self
        def __exit__(self, *_): pass
        def read1(self, _): return next(self.chunks)

    class Opener:
        def open(self, request, timeout):
            assert request.get_header('Authorization') == 'Bearer sk-private'
            return Reply()

    monkeypatch.setattr(ResponseTrace, 'feed', gated_feed)
    monkeypatch.setattr('harness.codex_cli.relay.urllib.request.build_opener', lambda *_: Opener())
    relay = ResponsesRelay(tmp_path/'relay.sock', 'sk-private', 'https://relay.example/v1',
                           'gpt-6-astra', 'high', tmp_path, 1)
    original_log = relay.log
    def log(row): original_log(row); logged.set()
    monkeypatch.setattr(relay, 'log', log)
    relay.start()
    client = http.client.HTTPConnection('private', timeout=5)
    client.sock = socket.socket(socket.AF_UNIX); client.sock.settimeout(5)
    client.sock.connect(str(relay.path))
    try:
        client.request('POST', '/v1/responses', json.dumps({'model': 'gpt-6-astra', 'input':
            [{'content': [{'type': 'input_text', 'text': 'PRIVATE_PROMPT'},
                          {'type': 'input_image', 'image_url': 'data:image/png;base64,PRIVATE_IMAGE'}]}]}))
        response = client.getresponse()
        assert entered.wait(5)
        assert response.read(len(first)) == first  # observer is still blocked
        release.set()
        assert response.read() == second
        assert logged.wait(5)
        raw = (tmp_path/'model-usage.jsonl').read_text()
        assert 'PRIVATE_' not in raw and 'sk-private' not in raw and 'data:image' not in raw
        row = json.loads(raw)
        assert row['ok'] and row['input_images'] == 1 and row['request_id']
        assert row['response_bytes'] == len(first)+len(second)
        assert row['timings_s']['first_byte'] <= row['timings_s']['response_completed']
        assert relay.calls == 1  # no retry
    finally:
        release.set(); client.close(); relay.close()


def test_explicit_direct_ignores_environment_proxy(tmp_path,monkeypatch):
    monkeypatch.setenv('https_proxy','http://127.0.0.1:19999')
    relay=ResponsesRelay(tmp_path/'m.sock','test-secret','https://example.invalid/v1',
                         'gpt-6-astra','high',tmp_path,10,proxy_url='')
    try:assert relay.proxy_handler.proxies=={}
    finally:relay.server.server_close()
