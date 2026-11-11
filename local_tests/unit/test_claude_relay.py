"""Claude Messages relay and launcher wiring; local fake provider, no external requests."""
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import socket
import threading
import time
from pathlib import Path

import pytest

from harness.claude_cli import messages_relay
from harness.claude_cli.messages_relay import BUDGET_EXHAUSTED, Credential, MessagesRelay, OAUTH_BETA

MODEL = 'anthropic/claude-opus-5.5'
SSE = (b'event: message_start\ndata: {"type":"message_start","message":{"usage":{"input_tokens":11,'
       b'"cache_read_input_tokens":5,"output_tokens":1}}}\n\n'
       b'event: content_block_delta\ndata: {"type":"content_block_delta","delta":{"text":"ok"}}\n\n'
       b'event: message_delta\ndata: {"type":"message_delta","usage":{"output_tokens":7,"cost":0.002}}\n\n')


def private(path, text):
    path.write_text(text)
    path.chmod(0o600)
    return path


@pytest.fixture
def relay(tmp_path, request):
    received = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            received.append((self.path, dict(self.headers), body))
            status = (body.get('metadata') or {}).get('status', 200)
            self.send_response(status)
            if status == 200 and body.get('stream'):
                self.send_header('Content-Type', 'text/event-stream')
                self.end_headers()
                self.wfile.write(SSE)
            else:
                data = json.dumps({'type': 'error', 'error': {'type': 'invalid_request_error'}} if status != 200
                                  else {'usage': {'input_tokens': 3, 'output_tokens': 2}}).encode()
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

    upstream = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=upstream.serve_forever, daemon=True).start()
    provider, credential = getattr(request, 'param', ('openrouter', None))
    key = private(tmp_path/'key', 'host-secret')
    if provider == 'claude-login':
        credential = Credential('claude-login', key_file=private(tmp_path/'claude-experiment.token', 'login-token'))
    else:
        credential = Credential(provider, key_file=key)
    path = tmp_path/'relay.sock'
    usage = tmp_path/'usage.jsonl'
    router = MessagesRelay(path, credential, base_url=f'http://127.0.0.1:{upstream.server_port}/api',
                           models=[MODEL], max_calls=2, client_write_timeout=5, usage_log=usage)
    router.start()

    def call(body=None, *, route='/v1/messages?beta=true', headers=None):
        connection = http.client.HTTPConnection('localhost', timeout=5)
        connection.sock = socket.socket(socket.AF_UNIX)
        connection.sock.settimeout(5)
        connection.sock.connect(str(path))
        payload = {'model': MODEL, 'max_tokens': 64, 'stream': True, 'messages': [{'role': 'user', 'content': 'hi'}]}
        raw = json.dumps(payload if body is None else body).encode()
        sent = {'Authorization': 'Bearer host-relay', 'X-Agent-Secret': 'do-not-forward',
                'anthropic-version': '2023-06-01', 'anthropic-beta': 'interleaved-thinking-2025-05-14',
                **(headers or {})}
        try:
            connection.request('POST', route, raw, sent)
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()
    try:
        yield call, received, router, usage
    finally:
        router.close()
        upstream.shutdown()
        upstream.server_close()


def test_stream_is_forwarded_with_host_credential_and_usage_logged(relay):
    call, received, router, usage = relay
    assert call() == (200, SSE)
    path, headers, _ = received[0]
    assert path == '/api/v1/messages?beta=true'
    assert headers['Authorization'] == 'Bearer host-secret' and 'X-Agent-Secret' not in headers
    assert headers['anthropic-beta'] == 'interleaved-thinking-2025-05-14'
    row = json.loads(usage.read_text())
    assert row['model'] == MODEL and row['input_tokens'] == 11 and row['cache_read_input_tokens'] == 5
    assert row['output_tokens'] == 7 and row['cost_usd'] == 0.002


def test_budget_counts_messages_but_not_token_counting(relay):
    call, received, router, _ = relay
    assert call(route='/v1/messages/count_tokens')[0] == 200
    assert call()[0] == 200 and call()[0] == 200
    assert call() == (402, BUDGET_EXHAUSTED)
    assert router.calls == 2 and len(received) == 3


@pytest.mark.parametrize('body', [
    {'model': 'anthropic/claude-haiku-4.5', 'max_tokens': 64, 'messages': []},  # Not allowlisted.
    {'model': MODEL, 'max_tokens': 64, 'messages': [], 'tools': [{'type': 'web_search_20250305', 'name': 'web_search'}]},
    {'model': MODEL, 'max_tokens': 64, 'messages': [], 'mcp_servers': [{'url': 'https://x'}]},
    {'model': MODEL, 'max_tokens': 64, 'messages': [{'role': 'user', 'content': [
        {'type': 'image', 'source': {'type': 'url', 'url': 'https://example.com/a.png'}}]}]},
    {'model': MODEL, 'max_tokens': 64, 'messages': [{'role': 'user', 'content': [{'type': 'tool_result', 'content': [
        {'type': 'document', 'source': {'type': 'file', 'file_id': 'f'}}]}]}]},
    {'model': MODEL, 'max_tokens': 10**6, 'messages': []},
])
def test_forbidden_requests_are_refused_without_spending(relay, body):
    call, received, router, _ = relay
    assert call(body)[0] in (400, 403)
    assert not received and router.calls == 0


def test_client_tools_and_inline_images_pass(relay):
    call, received, _, _ = relay
    body = {'model': MODEL, 'max_tokens': 64, 'stream': True,
            'tools': [{'name': 'Bash', 'input_schema': {'type': 'object'}},
                      {'name': 'mcp__auto_research__exploration_status', 'input_schema': {'type': 'object'}}],
            'messages': [{'role': 'user', 'content': [
                {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/png', 'data': 'AA=='}}]}]}
    assert call(body)[0] == 200 and received


def test_provider_errors_keep_their_type(relay):
    call, _, _, _ = relay
    status, body = call({'model': MODEL, 'max_tokens': 64, 'messages': [], 'metadata': {'status': 400}})
    assert status == 400 and json.loads(body)['error']['type'] == 'invalid_request_error'


@pytest.mark.parametrize('relay', [('claude-login', None)], indirect=True)
def test_claude_login_uses_the_oauth_token_and_beta(relay):
    call, received, _, _ = relay
    assert call()[0] == 200
    headers = received[0][1]
    assert headers['Authorization'] == 'Bearer login-token'
    assert headers['anthropic-beta'].split(',') == ['interleaved-thinking-2025-05-14', OAUTH_BETA]


def test_credentials_must_be_private(tmp_path):
    loose = tmp_path/'loose'
    loose.write_text('k')
    loose.chmod(0o644)
    with pytest.raises(ValueError, match='private'):
        Credential('openrouter', key_file=loose)
    with pytest.raises(ValueError, match='Unknown provider'):
        Credential('other')


def test_claude_login_refuses_the_personal_login(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    with pytest.raises(ValueError, match='experiment setup-token'):
        Credential('claude-login')
    personal = tmp_path/'.claude'
    personal.mkdir()
    login = json.dumps({'claudeAiOauth': {'accessToken': 't', 'expiresAt': (time.time()+3600)*1000}})
    for path in (personal/'.credentials.json', personal/'token', tmp_path/'elsewhere/.credentials.json'):
        path.parent.mkdir(parents=True, exist_ok=True)
        with pytest.raises(ValueError, match='personal'):
            Credential('claude-login', key_file=private(path, 'token'))
    # The login JSON shape is refused wherever it is copied.
    with pytest.raises(ValueError, match='credentials file'):
        Credential('claude-login', key_file=private(tmp_path/'copied.token', login))
    assert Credential('claude-login', key_file=private(tmp_path/'experiment.token', 'sk-ant-oat01-x')).headers() == {
        'Authorization': 'Bearer sk-ant-oat01-x'}


def test_relay_listens_on_loopback_tcp(tmp_path):
    hits = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            hits.append(self.headers.get('Authorization'))
            self.rfile.read(int(self.headers['Content-Length']))
            data = b'{"usage":{"input_tokens":1,"output_tokens":1}}'
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    upstream = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=upstream.serve_forever, daemon=True).start()
    credential = Credential('claude-login', key_file=private(tmp_path/'claude-experiment.token', 'login-token'))
    router = MessagesRelay(('127.0.0.1', 0), credential, base_url=f'http://127.0.0.1:{upstream.server_port}',
                           models=[MODEL], client_write_timeout=5)
    router.start()
    try:
        host, port = router.address
        assert host == '127.0.0.1' and port > 0
        connection = http.client.HTTPConnection(host, port, timeout=5)
        connection.request('POST', '/v1/messages', json.dumps(
            {'model': MODEL, 'max_tokens': 8, 'messages': [{'role': 'user', 'content': 'hi'}]}).encode(),
            {'Content-Type': 'application/json', 'Authorization': 'Bearer host-relay'})
        assert connection.getresponse().status == 200
        assert hits == ['Bearer login-token'] and router.calls == 1
    finally:
        router.close()
        upstream.shutdown()
        upstream.server_close()
    with pytest.raises(ValueError, match='loopback'):
        MessagesRelay(('0.0.0.0', 0), credential, base_url='https://api.anthropic.com', models=[MODEL])


def test_an_exhausted_usage_window_is_returned_at_once_and_remembered(tmp_path):
    hits = []
    reset = time.time()+3600

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            self.rfile.read(int(self.headers['Content-Length']))
            hits.append(1)
            data = b'{"type":"error","error":{"type":"rate_limit_error"}}'
            self.send_response(429)
            self.send_header('anthropic-ratelimit-unified-status', 'rejected')
            self.send_header('anthropic-ratelimit-unified-reset', str(int(reset)))
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    upstream = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=upstream.serve_forever, daemon=True).start()
    credential = Credential('claude-login', key_file=private(tmp_path/'claude-experiment.token', 'login-token'))
    router = MessagesRelay(tmp_path/'relay.sock', credential, base_url=f'http://127.0.0.1:{upstream.server_port}',
                           models=[MODEL], client_write_timeout=5)
    router.start()
    try:
        for _ in range(2):
            connection = http.client.HTTPConnection('localhost', timeout=5)
            connection.sock = socket.socket(socket.AF_UNIX)
            connection.sock.connect(str(tmp_path/'relay.sock'))
            connection.request('POST', '/v1/messages', json.dumps(
                {'model': MODEL, 'max_tokens': 8, 'messages': [{'role': 'user', 'content': 'hi'}]}).encode(),
                {'Content-Type': 'application/json'})
            response = connection.getresponse()
            assert response.status == 429 and json.loads(response.read())['error']['type'] == 'rate_limit_error'
            connection.close()
        assert hits == [1] and int(router.limited_until) == int(reset)  # No retry window, no second upstream call.
    finally:
        router.close()
        upstream.shutdown()
        upstream.server_close()


class FakeRelay:
    limited_until = 0.0


def test_the_agent_continues_its_conversation_after_the_usage_reset(tmp_path, monkeypatch):
    from harness.claude_cli import launcher as claude
    monkeypatch.setattr(claude, 'latest_claude_session', lambda home: 'conv')
    relay, runs, waits, slept = FakeRelay(), [], [], []
    def run(prompt, remaining):
        runs.append((prompt, remaining))
        if len(runs) == 1:
            relay.limited_until = time.time()+120  # The window ran out during this run.
        return 1
    deployed = {'claude_home': tmp_path}
    def reopen(seconds):
        slept.append(seconds)
        relay.limited_until = time.time()-1  # Real time passes the reset while sleeping.
    code = claude.until_usage_resets(run, relay, deployed=deployed, prompt='go', budget=1000, interactive=False,
                                     on_wait=lambda until, waited: waits.append(until), sleep=reopen)
    assert code == 1 and [p for p, _ in runs] == ['go', claude.USAGE_PROMPT]
    assert 175 < slept[0] <= 180 and deployed['resume_session'] == 'conv'
    assert runs[1][1] > 990  # The wait is off the session clock.
    assert waits[0] > time.time() and waits[1] is None


@pytest.mark.parametrize('interactive', [True, False])
def test_no_usage_wait_when_interactive_or_not_limited(tmp_path, interactive):
    from harness.claude_cli import launcher as claude
    relay, runs = FakeRelay(), []
    def run(prompt, remaining):
        runs.append(prompt)
        if interactive:
            relay.limited_until = time.time()+120
        return 0
    claude.until_usage_resets(run, relay, deployed={}, prompt='go', budget=100, interactive=interactive,
                              sleep=lambda s: pytest.fail('waited'))
    assert runs == ['go']
