"""Google official transport for RAX's existing Gemini MCP contract."""
import copy
import http.client
import io
import ssl
import sys
import urllib.error
import datetime
import json
import time
import urllib.parse
import urllib.request

from .gemini import GeminiRouter, MODEL

GOOGLE_MODEL = 'gemini-3.8-flash'
ENDPOINT = 'https://generativelanguage.googleapis.com/v1beta/openai/chat/completions'
# Published introductory STANDARD prices; cached input is conservatively undiscounted.
PRICE_VALID_UNTIL = datetime.date(2026, 12, 31)


class GoogleGeminiRouter(GeminiRouter):
    def __init__(self, key, *, proxy_url=None, send=None):
        if proxy_url:
            u = urllib.parse.urlsplit(proxy_url)
            if u.scheme not in ('http', 'https') or not u.hostname or u.username or u.password or u.query or u.fragment:
                raise ValueError('Invalid operator Gemini proxy URL')
        self.proxy_url = proxy_url
        self._connection = None
        self._official_send = send or self._google_http
        super().__init__(key, send=self._send_google)

    def tool_schema(self, definition):
        schema = copy.deepcopy(definition)
        schema['description'] = schema['description'].replace('OpenRouter-style', 'Google official OpenAI-compatible').replace('including reasoning_details', 'including extra_content / thought signatures')
        fields = schema['inputSchema']['properties']
        fields.pop('temperature', None)
        fields['reasoning'] = {'type': 'object', 'properties': {'effort': {
            'type': 'string', 'enum': ['low', 'medium', 'high']}},
            'required': ['effort'], 'additionalProperties': False}
        return schema

    def prepare(self, arguments):
        if datetime.date.today() > PRICE_VALID_UNTIL:
            raise ValueError('Google Gemini price bound requires operator revalidation after 2026-12-31')
        payload, reservation = super().prepare(arguments)
        if 'temperature' in arguments:
            raise ValueError('Google Gemini 3.8 Flash does not accept temperature')
        reasoning = arguments.get('reasoning')
        if reasoning is not None and (set(reasoning) != {'effort'} or reasoning['effort'] not in ('low', 'medium', 'high')):
            raise ValueError('Google Gemini requires reasoning effort low, medium or high')
        return payload, reservation

    def close(self):
        if self._connection is not None:
            self._connection.close()
            self._connection = None

    def _google_http(self, payload):
        # One TLS connection per visual request: agent/model pauses can outlive
        # the proxy/upstream keep-alive. Never reuse an idle socket or replay a POST.
        self.close()
        proxy = urllib.parse.urlsplit(self.proxy_url) if self.proxy_url else None
        if proxy and proxy.scheme != 'http':
            return self._google_http_fallback(payload)
        target = urllib.parse.urlsplit(ENDPOINT)
        started = time.monotonic()
        trace = {'event': 'gemini_transport', 'route': 'proxy' if proxy else 'direct',
                 'reused_connection': False, 'connection_policy': 'per_request',
                 'connect_attempts': 0, 'stage': 'connect'}
        try:
            if self._connection is None or self._connection.sock is None:
                # Retry only BEFORE any Gemini HTTP request is sent. Do not replay
                # a POST after send/read failure, HTTP error, or response ambiguity.
                for attempt in range(3):
                    trace['connect_attempts'] += 1
                    conn = http.client.HTTPSConnection(
                        proxy.hostname if proxy else target.hostname,
                        (proxy.port or 80) if proxy else (target.port or 443), timeout=10)
                    if proxy:
                        conn.set_tunnel(target.hostname, target.port or 443)
                    self._connection = conn
                    try:
                        conn.connect()
                        conn.sock.settimeout(60)
                        break
                    except (OSError, http.client.HTTPException) as exc:
                        self.close()
                        if isinstance(exc, ssl.SSLCertVerificationError) or attempt == 2:
                            raise
                        time.sleep(.25)
            conn = self._connection
            trace['connect_s'] = time.monotonic() - started
            trace['stage'] = 'send'
            conn.request('POST', target.path, body=json.dumps(payload).encode(),
                         headers={'Authorization': 'Bearer '+self._key,
                                  'Content-Type': 'application/json', 'Connection': 'close'})
            trace['send_done_s'] = time.monotonic() - started
            trace['stage'] = 'headers'
            response = conn.getresponse()
            trace.update(status=response.status, headers_s=time.monotonic()-started)
            trace['stage'] = 'read'
            raw = response.read(2*1024*1024+1)
            trace['response_bytes'] = len(raw)
            if len(raw) > 2*1024*1024:
                raise ValueError('Provider response too large')
            if not 200 <= response.status < 300:
                raise urllib.error.HTTPError(ENDPOINT, response.status, response.reason,
                                             response.headers, io.BytesIO(raw))
            value = json.loads(raw)
            trace.update(ok=True, stage='complete')
            return value
        except BaseException as exc:
            trace.update(ok=False, error_type=type(exc).__name__,
                         reason=str(exc).replace(self._key, '[redacted]')[:500])
            self.close()
            raise
        finally:
            self.close()
            trace['duration_s'] = time.monotonic()-started
            # Host-only structured diagnostics: never prompts, image bytes or keys.
            print(json.dumps(trace), file=sys.stderr, flush=True)

    def _google_http_fallback(self, payload):
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *args, **kwargs): return None
        handlers = [NoRedirect()]
        if self.proxy_url:
            handlers.append(urllib.request.ProxyHandler({'https': self.proxy_url}))
        req = urllib.request.Request(ENDPOINT, data=json.dumps(payload).encode(),
            headers={'Authorization': 'Bearer '+self._key, 'Content-Type': 'application/json', 'Connection': 'close'})
        with urllib.request.build_opener(*handlers).open(req, timeout=60) as response:
            raw = response.read(2*1024*1024+1)
        if len(raw) > 2*1024*1024: raise ValueError('Provider response too large')
        return json.loads(raw)

    def _send_google(self, payload):
        request = {k:v for k,v in payload.items() if k not in ('provider', 'reasoning')}
        request['model'] = GOOGLE_MODEL
        if 'reasoning' in payload: request['reasoning_effort'] = payload['reasoning']['effort']
        response = self._official_send(request)
        if response.get('model') != GOOGLE_MODEL:
            raise ValueError('Google returned an unexpected model; no fallback accepted')
        # Canonical MCP model name stays the same as RAX; verify identity BEFORE normalization.
        return {**response, 'model': MODEL}

    def execute(self, payload, reservation):
        started = time.monotonic()
        value, used = super().execute(payload, reservation)
        usage = value['usage']; prompt = usage.get('prompt_tokens')
        if type(prompt) is not int or not 0 <= prompt <= used:
            raise ValueError('Invalid Google prompt usage; full reservation retained')
        # total_tokens includes thinking tokens even when completion_tokens omits them.
        # Unknown/non-prompt tokens are charged at the more expensive output rate.
        upper = (prompt*750 + (used-prompt)*3750) / 1_000_000_000
        value.update(provider='google', provider_model=GOOGLE_MODEL,
            duration_s=time.monotonic()-started, cost_usd=None,
            cost_upper_bound_usd=upper,
            cost_basis='Conservative standard-price estimate, not provider-reported billing; thinking included',
            pricing_valid_until=PRICE_VALID_UNTIL.isoformat())
        # Google official responses do not supply actual dollar billing.
        value['usage'] = {**usage, 'cost': None}
        return value, used
