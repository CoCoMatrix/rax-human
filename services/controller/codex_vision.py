"""Host-brokered vision through Responses; no task policy and no robot tools."""
import json
import time
import urllib.request
from .gemini import validate_image

MODEL = 'gpt-6-astra'
TOOL = {'name': 'codex_vision_generate', 'description':
        'Auxiliary visual/spatial analysis using gpt-6-astra medium via Responses. '
        'Supply your own prompt and up to eight inline PNG/JPEG images. No robot actions, '
        'remote image URLs, or server tools. Returns text, raw output items and usage.',
        'inputSchema': {'type': 'object', 'additionalProperties': False,
                        'required': ['prompt'], 'properties': {
            'prompt': {'type': 'string', 'maxLength': 65536},
            'images': {'type': 'array', 'maxItems': 8, 'items': {'type': 'object'}},
            'max_output_tokens': {'type': 'integer', 'minimum': 256, 'maximum': 16384}}}}


def decode_response(raw, content_type="application/json"):
    """Accept standard JSON or relay SSE, only after a completed response event.

    Some compatible relays emit SSE even with stream=false. Never treat partial
    deltas or an interrupted stream as a successful visual observation.
    """
    if "text/event-stream" not in content_type:
        return json.loads(raw)
    completed = None
    data_lines = []
    event_name = None
    for line in [*raw.decode("utf-8").splitlines(), ""]:
        if line.startswith("event:"):
            event_name = line[6:].strip()
        elif line.startswith("data:"):
            data_lines.append(line[5:].lstrip())
        elif not line:
            data = "\n".join(data_lines)
            if data and data != "[DONE]":
                item = json.loads(data)
                kind = item.get("type", event_name)
                if kind in ("error", "response.failed", "response.incomplete"):
                    raise RuntimeError("Responses stream failed or was incomplete")
                if kind == "response.completed":
                    completed = item.get("response")
            data_lines, event_name = [], None
    if not isinstance(completed, dict) or completed.get("status") != "completed":
        raise RuntimeError("Responses stream ended without a completed response")
    return completed


class VisionRouter:
    def __init__(self, key, base_url, *, send=None, proxy_url=None):
        self.key, self.base_url = key, base_url
        self.send = send or self._http
        self.proxy_handler = urllib.request.ProxyHandler({"http": proxy_url, "https": proxy_url}) if proxy_url else urllib.request.ProxyHandler()

    def _http(self, payload):
        from harness.codex_cli.relay import ResponseTrace
        trace = ResponseTrace(time.monotonic(), self.key)
        encoded = json.dumps(payload).encode()
        trace.request(payload, len(encoded), len(encoded))
        req = urllib.request.Request(self.base_url + '/responses', data=encoded,
                                     headers={'Authorization': 'Bearer ' + self.key, 'Content-Type': 'application/json'})
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *args, **kwargs):
                return None
        try:
            trace.mark('upstream_open_started')
            with urllib.request.build_opener(self.proxy_handler, NoRedirect).open(req, timeout=120) as reply:
                content_type = reply.headers.get("Content-Type", "application/json")
                trace.headers(reply.status, content_type, reply.headers.get('x-request-id'))
                raw = bytearray()
                while True:
                    chunk = reply.read1(min(65536, 8 * 1024 * 1024 + 1 - len(raw)))
                    if not chunk:
                        break
                    trace.feed(chunk, trace.received(chunk))
                    raw.extend(chunk)
                    if len(raw) > 8 * 1024 * 1024:
                        raise ValueError('Vision response exceeds size limit')
            trace.finish()
            response = decode_response(bytes(raw), content_type)
            response['_transport_diagnostics'] = trace.result()
            return response
        except Exception:
            raise RuntimeError('Auxiliary vision provider request failed; inspect host diagnostics') from None

    def call(self, args):
        if not isinstance(args, dict) or set(args) - {'prompt', 'images', 'max_output_tokens'}:
            raise ValueError('Unsupported vision arguments')
        prompt = args.get('prompt')
        maximum = args.get('max_output_tokens', 4096)
        images = args.get('images', [])
        if not isinstance(prompt, str) or not 1 <= len(prompt) <= 65536:
            raise ValueError('prompt must be 1..65536 characters')
        if type(maximum) is not int or not 256 <= maximum <= 16384:
            raise ValueError('max_output_tokens must be 256..16384')
        if not isinstance(images, list) or len(images) > 8:
            raise ValueError('Provide at most eight inline images')
        parts = [{'type': 'input_text', 'text': prompt}]
        total = 0
        for image in images:
            raw, mime, _ = validate_image(image)
            total += len(raw)
            if total > 8 * 1024 * 1024:
                raise ValueError('Total images exceed 8 MiB')
            parts.append({'type': 'input_image', 'image_url': f'data:{mime};base64,{image["data"]}'})
        payload = {'model': MODEL, 'reasoning': {'effort': 'medium'}, 'store': False, 'stream': False,
                   'input': [{'role': 'user', 'content': parts}], 'max_output_tokens': maximum}
        response = self.send(payload)
        if response.get('status') not in ('completed', None) or response.get('error'):
            raise RuntimeError('Auxiliary vision did not complete')
        output = response.get('output', [])
        text = '\n'.join(c.get('text', '') for item in output if item.get('type') == 'message'
                         for c in item.get('content', []) if c.get('type') == 'output_text')
        if not text:
            raise RuntimeError('Auxiliary vision returned no text')
        return {'model': MODEL, 'response_model': response.get('model'), 'response_id': response.get('id'),
                'reasoning_effort': 'medium', 'text': text,
                'output': output, 'usage': response.get('usage', {}),
                'transport_diagnostics': response.get('_transport_diagnostics'),
                'cost_usd': None, 'billing': 'Relay price not verified; no Gemini dollar cap is claimed'}


class VisionBackend:
    """Add one perception tool while keeping upstream Frontend serialization/audit."""
    def __init__(self, backend, router, session):
        self.backend, self.router, self.session = backend, router, session
        self.mutating = getattr(backend, 'mutating', set())

    def tools(self):
        return [*self.backend.tools(), TOOL]

    def validate(self, name, arguments):
        if name != TOOL['name']:
            validate = getattr(self.backend, 'validate', None)
            if validate:
                validate(name, arguments)

    def call(self, name, arguments):
        if name != TOOL['name']:
            return self.backend.call(name, arguments)
        state = self.session.state
        used = state.get('codex_vision_calls', 0)
        if used >= self.session.config.max_model_calls:
            raise RuntimeError('Auxiliary vision call budget exhausted')
        state['codex_vision_calls'] = used + 1
        self.session._save()  # Durable reservation before provider I/O; failures consume one call.
        start = time.monotonic()
        result = self.router.call(arguments)
        result['duration_s'] = round(time.monotonic() - start, 3)
        self.session.audit({'event': 'codex_vision_usage', 'usage': result['usage'], 'duration_s': result['duration_s'],
                            'transport_diagnostics': result.get('transport_diagnostics')})
        return result, []

    def status(self):
        return {**self.backend.status(), 'auxiliary_vision': {
            'model': MODEL, 'effort': 'medium', 'calls_used': self.session.state.get('codex_vision_calls', 0),
            'call_limit': self.session.config.max_model_calls, 'cost_usd': None}}

    def close(self):
        self.backend.close()
