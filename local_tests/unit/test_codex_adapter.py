import base64
import io
import json
from pathlib import Path
import pytest
from PIL import Image
from harness.codex_cli.credentials import load_credentials
from harness.codex_cli.launcher import codex_config, prepare, sandbox_command
from services.controller.codex_vision import VisionRouter
from services.controller.config import Configuration


def test_credentials_raw_and_literals_without_execution(tmp_path):
    key = tmp_path/'key.py'
    key.write_text('sk-test-private')
    assert load_credentials(key, 'https://relay.example/v1') == ('sk-test-private', 'https://relay.example/v1')
    key.write_text("api_key='sk-test-private'\nbase_url='https://relay.example/v1'\n")
    assert load_credentials(key)[1] == 'https://relay.example/v1'
    key.write_text("raise RuntimeError('SECRET-MUST-NOT-ECHO')")
    with pytest.raises(ValueError) as error:
        load_credentials(key, 'https://relay.example/v1')
    assert 'SECRET' not in str(error.value)


def test_vision_uses_responses_and_fixed_model_effort():
    sent = []
    image = Image.new('RGB', (8,8), 'blue')
    stream = io.BytesIO(); image.save(stream, format='PNG')
    def send(payload):
        sent.append(payload)
        return {'status': 'completed', 'output': [{'type': 'message', 'content': [{'type': 'output_text', 'text': 'blue'}]}],
                'usage': {'total_tokens': 20}}
    router = VisionRouter('unused', 'https://relay.example/v1', send=send)
    result = router.call({'prompt': 'Describe', 'images': [{'mimeType': 'image/png', 'data': base64.b64encode(stream.getvalue()).decode()}]})
    assert result['text'] == 'blue'
    assert sent[0]['model'] == 'gpt-6-astra'
    assert sent[0]['reasoning'] == {'effort': 'medium'}
    assert sent[0]['input'][0]['content'][1]['type'] == 'input_image'
    assert 'tools' not in sent[0] and 'temperature' not in sent[0]
    assert result['cost_usd'] is None
    with pytest.raises(ValueError):
        router.call({'prompt': 'Describe', 'images': [{'url': 'http://127.0.0.1/private'}]})


def test_prepare_preserves_native_memory_contract(tmp_path, monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT', str(tmp_path))
    config = Configuration(root=tmp_path/'session', task='cup', robot='mock', agent_provider='codex-relay', agent_effort='high')
    workspace = prepare(config, tmp_path/'key.py', 'https://relay.example/v1')
    assert 'memory/PROGRESS.md' in (workspace/'AGENTS.md').read_text()
    assert (workspace/'.agents/skills/experience-memory/SKILL.md').exists()
    assert 'codex_vision_generate' in json.loads((workspace/'MCP_CONTRACT.json').read_text())['tools']
    assert not (config.root/'state.json').exists()  # No backend or owner was started.
    cfg = codex_config(config)
    assert '100000' not in cfg and 'compact' not in cfg
    assert 'sk-' not in cfg


def test_sandbox_has_no_host_root_or_network(tmp_path, monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT', str(tmp_path))
    monkeypatch.setattr('harness.codex_cli.launcher.sys.platform', 'linux')
    monkeypatch.setattr('harness.codex_cli.launcher.shutil.which', lambda n: '/usr/bin/'+n)
    config = Configuration(root=tmp_path/'s', task='cup', robot='mock', agent_provider='codex-relay')
    cmd = sandbox_command(config, Path('/private/robot.sock'), Path('/private/model.sock'), prompt='status')
    assert '--unshare-all' in cmd and '--clearenv' in cmd
    assert '--share-net' not in cmd
    assert not any(cmd[i:i+3] == ['--ro-bind', '/', '/'] for i in range(len(cmd)-2))
    assert '--dev' in cmd and '--dev-bind' not in cmd


def test_private_relay_pins_model_and_keeps_key_host_side(tmp_path, monkeypatch):
    import http.client
    import socket
    from harness.codex_cli.relay import ResponsesRelay
    captured = []
    class Reply:
        status = 200
        headers = {'Content-Type': 'application/json'}
        def __enter__(self): return self
        def __exit__(self, *_): pass
        def read1(self, _):
            if getattr(self, 'done', False): return b''
            self.done = True
            return b'{"status":"completed","output":[]}'
    class Opener:
        def open(self, request, timeout):
            captured.append(request)
            return Reply()
    monkeypatch.setattr('harness.codex_cli.relay.urllib.request.build_opener', lambda *_: Opener())
    relay = ResponsesRelay(tmp_path/'relay.sock', 'sk-host-only', 'https://relay.example/v1',
                           'gpt-6-astra', 'high', tmp_path, 2)
    relay.start()
    def request(payload):
        client = http.client.HTTPConnection('private')
        client.sock = socket.socket(socket.AF_UNIX)
        client.sock.connect(str(tmp_path/'relay.sock'))
        client.request('POST', '/v1/responses', json.dumps(payload), {'Content-Type': 'application/json'})
        response = client.getresponse(); body = response.read(); status = response.status; client.close()
        return status, body
    try:
        assert request({'model': 'gpt-6-astra', 'input': 'test'})[0] == 200
        assert captured[0].get_header('Authorization') == 'Bearer sk-host-only'
        assert json.loads(captured[0].data)['reasoning']['effort'] == 'high'
        status, body = request({'model': 'unapproved', 'input': 'test'})
        assert status != 200 and b'sk-host-only' not in body
        assert len(captured) == 1
        assert 'sk-host-only' not in (tmp_path/'model-usage.jsonl').read_text()
    finally:
        relay.close()


def test_vision_accepts_relay_sse_completed_response(monkeypatch):
    completed = {'id': 'resp_test', 'status': 'completed', 'output': [
        {'type': 'message', 'content': [{'type': 'output_text', 'text': 'blue'}]}],
        'usage': {'total_tokens': 22}}
    raw = ('event: response.created\ndata: {"type":"response.created"}\n\n'
           'event: response.completed\ndata: ' + json.dumps({'type': 'response.completed', 'response': completed})
           + '\n\ndata: [DONE]\n\n').encode()
    class Reply:
        status = 200
        headers = {'Content-Type': 'text/event-stream'}
        remaining = raw
        def __enter__(self): return self
        def __exit__(self, *_): pass
        def read1(self, n):
            chunk, self.remaining = self.remaining[:n], self.remaining[n:]
            return chunk
    class Opener:
        def open(self, *_args, **_kwargs): return Reply()
    monkeypatch.setattr('services.controller.codex_vision.urllib.request.build_opener', lambda *_: Opener())
    result = VisionRouter('dummy', 'https://relay.example/v1').call({'prompt': 'Color?'})
    assert result['text'] == 'blue' and result['usage']['total_tokens'] == 22
    trace = result['transport_diagnostics']
    assert trace['ok'] and trace['input_images'] == 0
    assert trace['timings_s']['first_byte'] <= trace['timings_s']['response_completed']
    assert trace['timings_s']['response_completed'] <= trace['timings_s']['stream_end']
    assert 'dummy' not in json.dumps(trace)


def test_sse_partial_or_failed_output_is_never_success():
    from services.controller.codex_vision import decode_response
    partial = b'event: response.output_text.delta\ndata: {"type":"response.output_text.delta","delta":"looks safe"}\n\ndata: [DONE]\n\n'
    with pytest.raises(RuntimeError, match='without a completed'):
        decode_response(partial, 'text/event-stream')
    failed = b'event: response.failed\ndata: {"type":"response.failed","response":{"status":"failed"}}\n\n'
    with pytest.raises(RuntimeError, match='failed or was incomplete'):
        decode_response(failed, 'text/event-stream')


def test_main_direct_preserves_auxiliary_proxy(tmp_path, monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT', str(tmp_path))
    config = Configuration(root=tmp_path/'session', task='cup', robot='mock', agent_provider='codex-relay')
    prepare(config, tmp_path/'key.py', 'https://relay.example/v1', http_proxy='http://127.0.0.1:17890', model_http_proxy='')
    settings = json.loads((config.root/'codex-relay.json').read_text())
    assert settings['model_http_proxy'] == ''
    assert settings['http_proxy'] == 'http://127.0.0.1:17890'
