"""Private Unix-socket Responses relay. Credentials remain in this host process."""
import http.server
import json
import os
import re
import socketserver
import threading
import time
import urllib.error
import urllib.request
import uuid

MAX_BYTES = 32 * 1024 * 1024
TRACE_LIMIT = 1024 * 1024


def native_jpeg_transport(body):
    """Encode only native RGB camera PNG attachments as JPEG95.

    No resizing, history removal, text rewriting or custom compaction. Raw files
    and the Codex conversation remain untouched. Depth/alpha/noncamera images
    and existing JPEGs are forwarded unchanged. Runs only on /responses.
    """
    started=time.monotonic();converted=0;saved=0
    stack=[body.get('input')]
    while stack:
        item=stack.pop()
        if isinstance(item,list):stack.extend(item)
        elif isinstance(item,dict):
            if item.get('type')=='input_image' and isinstance(item.get('image_url'),str):
                old=item['image_url'];new=_native_camera_jpeg(old)
                if new!=old:
                    item['image_url']=new;converted+=1;saved+=len(old)-len(new)
            stack.extend(v for v in item.values() if isinstance(v,(dict,list)))
    return {'converted_images':converted,'saved_bytes':saved,'duration_s':round(time.monotonic()-started,6)}


from functools import lru_cache
@lru_cache(maxsize=32)
def _native_camera_jpeg(url):
    prefix='data:image/png;base64,'
    if not url.startswith(prefix) or len(url)<262144:return url
    import base64,struct
    try:
        raw=base64.b64decode(url[len(prefix):],validate=True)
        # Check before decoding. Never decode arbitrary giant images.
        if (raw[:8]!=b'\x89PNG\r\n\x1a\n' or len(raw)<29
                or struct.unpack('>II',raw[16:24])!=(1280,720)
                or raw[24:26]!=bytes([8,2])):return url
        import cv2,numpy as np
        array=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_UNCHANGED)
        if array is None or array.shape!=(720,1280,3) or array.dtype!=np.uint8:return url
        ok,jpg=cv2.imencode('.jpg',array,[cv2.IMWRITE_JPEG_QUALITY,95])
        if not ok:return url
        out='data:image/jpeg;base64,'+base64.b64encode(jpg).decode()
        return out if len(out)<len(url) else url
    except (ValueError,ImportError,RuntimeError):
        return url



def forwarding_headers(headers, *, replace_auth=False):
    """Preserve end-to-end metadata; framing/routing belongs to each HTTP hop."""
    pairs=list(headers.items()) if hasattr(headers, 'items') else list(headers)
    drop={'connection','keep-alive','proxy-authenticate','proxy-authorization',
          'te','trailer','transfer-encoding','upgrade','host','content-length'}
    for name,value in pairs:
        if name.lower()=='connection':drop.update(v.strip().lower() for v in value.split(','))
    if replace_auth:drop.add('authorization')
    return {name:value for name,value in pairs if name.lower() not in drop}

class ResponseTrace:
    """Bounded, metadata-only observer; never changes or reconstructs forwarded bytes."""
    def __init__(self, start, secret, *, clock=time.monotonic):
        self.start, self.secret, self.clock = start, secret, clock
        self.row = {'request_id': uuid.uuid4().hex, 'request_started_at': time.time(),
                    'request_bytes': None, 'forwarded_request_bytes': None,
                    'input_items': None, 'input_images': None, 'input_count_complete': True,
                    'response_bytes': 0, 'response_id': None, 'response_model': None,
                    'provider_request_id': None, 'usage': {}, 'valid_sse_events': 0,
                    'diagnostic_parse_errors': 0, 'diagnostic_limit_exceeded': False,
                    'response_complete_observed': False, 'response_failed_observed': False,
                    'response_status': None, 'transport_complete': False}
        self.row['timings_s'] = {name: None for name in (
            'upstream_open_started', 'response_headers', 'first_byte', 'first_sse_event',
            'first_output_delta', 'first_tool_delta', 'response_completed', 'stream_end')}
        self.protocol = None
        self.pending = bytearray()
        self.data = bytearray()
        self.event_name = None
        self.discard_line = self.discard_event = False

    def mark(self, name, at=None):
        if self.row['timings_s'][name] is None:
            self.row['timings_s'][name] = round((self.clock() if at is None else at)-self.start, 6)

    def identifier(self, value):
        if (isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_.:/-]{1,128}', value)
                and not (self.secret and self.secret in value)):
            return value
        return None

    def request(self, body, size, forwarded_size):
        self.row.update(request_bytes=size, forwarded_request_bytes=forwarded_size)
        items = body.get('input')
        self.row['input_items'] = len(items) if isinstance(items, list) else int(items is not None)
        # Iterate containers only, never inspect or retain prompt/image strings.
        stack, count, images = [iter([items])], 0, 0
        while stack and count < 100000:
            try:
                item = next(stack[-1]); count += 1
            except StopIteration:
                stack.pop(); continue
            if isinstance(item, dict):
                images += item.get('type') in ('input_image', 'image')
                stack.append(iter(item.values()))
            elif isinstance(item, list):
                stack.append(iter(item))
        self.row.update(input_images=images, input_count_complete=not stack)

    def headers(self, status, content_type, provider_request_id=None):
        self.mark('response_headers')
        self.protocol = 'sse' if 'text/event-stream' in content_type.lower() else 'json'
        self.row.update(http_status=status, protocol=self.protocol,
                        provider_request_id=self.identifier(provider_request_id))

    def received(self, chunk):
        at = self.clock()
        self.mark('first_byte', at)
        self.row['response_bytes'] += len(chunk)
        return at

    def metadata(self, response):
        if not isinstance(response, dict):
            return
        for source, target in (('id', 'response_id'), ('model', 'response_model')):
            value = self.identifier(response.get(source))
            if value is not None:
                self.row[target] = value
        usage = response.get('usage')
        if isinstance(usage, dict):
            clean = {k: usage[k] for k in ('input_tokens', 'output_tokens', 'total_tokens',
                     'cached_input_tokens', 'reasoning_output_tokens', 'cache_write_input_tokens')
                     if type(usage.get(k)) is int and usage[k] >= 0}
            for key, names in (('input_tokens_details', ('cached_tokens',)),
                               ('output_tokens_details', ('reasoning_tokens',))):
                detail = usage.get(key)
                if isinstance(detail, dict):
                    clean[key] = {k: detail[k] for k in names if type(detail.get(k)) is int and detail[k] >= 0}
            self.row['usage'] = clean

    def parsed(self, item, at, event_name=None):
        if not isinstance(item, dict):
            return
        kind = item.get('type', event_name)
        if self.protocol == 'sse':
            if not isinstance(kind, str):
                return
            self.mark('first_sse_event', at)
            self.row['valid_sse_events'] += 1
            response = item.get('response', {})
            if kind in ('response.output_text.delta', 'response.refusal.delta'):
                self.mark('first_output_delta', at)
            if kind in ('response.function_call_arguments.delta', 'response.custom_tool_call_input.delta'):
                self.mark('first_tool_delta', at)
            complete = (kind == 'response.completed' and isinstance(response, dict)
                        and response.get('status') == 'completed')
            failed = kind in ('error', 'response.failed', 'response.incomplete')
        else:
            response = item
            # Compaction returns one whole JSON object, without response.status.
            complete = (item.get('status') == 'completed' or
                        (item.get('object') == 'response.compaction' and isinstance(item.get('output'), list)))
            failed = item.get('status') in ('failed', 'incomplete', 'cancelled') or bool(item.get('error'))
        self.metadata(response)
        if complete:
            self.mark('response_completed', at)
            self.row['response_complete_observed'] = True
            self.row['response_status'] = 'completed'
        if failed:
            error=response.get('error') if isinstance(response,dict) else None
            if not isinstance(error,dict):error=item.get('error',item)
            if isinstance(error,dict):
                self.row['provider_error_code']=self.identifier(error.get('code'))
                self.row['provider_error_type']=self.identifier(error.get('type'))
            self.row['response_failed_observed'] = True
            self.row['response_status'] = 'failed_or_incomplete'

    def line(self, line, at):
        if not line:
            if self.data and not self.discard_event:
                try:
                    if self.data.strip() != b'[DONE]':
                        self.parsed(json.loads(self.data), at, self.event_name)
                except (ValueError, RecursionError):
                    self.row['diagnostic_parse_errors'] += 1
            self.data.clear(); self.event_name = None; self.discard_event = False
        elif line.startswith(b'data:') and not self.discard_event:
            part = line[5:]
            if part.startswith(b' '): part = part[1:]
            if len(self.data)+len(part)+1 > TRACE_LIMIT:
                self.data.clear(); self.discard_event = True
                self.row['diagnostic_limit_exceeded'] = True
            else:
                self.data.extend(part); self.data.extend(b'\n')
        elif line.startswith(b'event:'):
            name = line[6:].strip()
            self.event_name = name.decode('ascii', errors='ignore') if len(name) <= 128 else None

    def feed(self, chunk, at):
        if self.protocol != 'sse':
            if not self.row['diagnostic_limit_exceeded']:
                if len(self.pending)+len(chunk) <= TRACE_LIMIT:
                    self.pending.extend(chunk)
                else:
                    self.pending.clear(); self.row['diagnostic_limit_exceeded'] = True
            return
        self.pending.extend(chunk)
        while self.pending:
            positions = [p for p in (self.pending.find(b'\n'), self.pending.find(b'\r')) if p >= 0]
            if not positions:
                break
            pos = min(positions)
            if self.pending[pos] == 13 and pos == len(self.pending)-1:
                break  # The next chunk may supply LF in a CRLF pair.
            end = pos+1
            if self.pending[pos:pos+2] == b'\r\n': end += 1
            line = bytes(self.pending[:pos]); del self.pending[:end]
            if self.discard_line:
                self.discard_line = False
            else:
                self.line(line, at)
        if len(self.pending) > TRACE_LIMIT:
            self.pending.clear(); self.data.clear()
            self.discard_line = self.discard_event = True
            self.row['diagnostic_limit_exceeded'] = True

    def finish(self):
        at = self.clock()
        self.mark('stream_end', at)
        self.row['transport_complete'] = True
        if self.protocol == 'sse':
            if self.pending and not self.discard_line:
                self.line(bytes(self.pending).rstrip(b'\r'), at)
            self.line(b'', at)
        elif self.pending and not self.row['diagnostic_limit_exceeded']:
            try:
                self.parsed(json.loads(self.pending), at)
            except (ValueError, RecursionError):
                self.row['diagnostic_parse_errors'] += 1
        self.pending.clear(); self.data.clear()

    def result(self):
        return {**self.row, 'ok': (self.row['transport_complete'] and
                self.row['response_complete_observed'] and not self.row['response_failed_observed'])}


class UnixHTTPServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


class ResponsesRelay:
    def __init__(self, socket_path, key, base_url, model, effort, root, max_calls, *, proxy_url=None, native_camera_jpeg=False):
        self.path, self.key, self.base_url = socket_path, key, base_url
        self.model, self.effort, self.root, self.max_calls = model, effort, root, max_calls
        self.proxy_handler = urllib.request.ProxyHandler({"http": proxy_url, "https": proxy_url}) if proxy_url else urllib.request.ProxyHandler({} if proxy_url == "" else None)
        self.native_camera_jpeg = native_camera_jpeg
        self.lock = threading.Lock()
        self.count_path = root / 'codex-model-calls.json'
        self.calls = json.loads(self.count_path.read_text()).get('calls', 0) if self.count_path.exists() else 0
        relay = self

        class Handler(http.server.BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.0'
            def log_message(self, *_):
                pass
            def do_POST(self):
                start = time.monotonic()
                trace = ResponseTrace(start, relay.key)
                if self.path not in ('/v1/responses', '/v1/responses/compact'):
                    self.send_error(404, 'Unsupported relay path')
                    return
                try:
                    n = int(self.headers.get('Content-Length', 0))
                    if not 0 < n <= MAX_BYTES:
                        raise ValueError('Invalid request size')
                    body = json.loads(self.rfile.read(n))
                    if body.get('model') != relay.model:
                        raise ValueError('Model is not authorized for this session')
                    if self.path.endswith('/responses'):
                        body['reasoning'] = {**body.get('reasoning', {}), 'effort': relay.effort}
                        body['store'] = False
                    with relay.lock:
                        if relay.max_calls is not None and relay.calls >= relay.max_calls:
                            self.send_error(429, 'Session model call budget exhausted')
                            return
                        relay.calls += 1
                        relay.count_path.write_text(json.dumps({'calls': relay.calls}))
                    if relay.native_camera_jpeg and self.path.endswith('/responses'):
                        trace.row['image_transport']=native_jpeg_transport(body)
                    encoded = json.dumps(body).encode()
                    trace.request(body, n, len(encoded))
                    headers=forwarding_headers(self.headers,replace_auth=True)
                    headers['Authorization']='Bearer '+relay.key
                    request = urllib.request.Request(relay.base_url + self.path[3:],
                        data=encoded, headers=headers)
                    class NoRedirect(urllib.request.HTTPRedirectHandler):
                        def redirect_request(self, *args, **kwargs):
                            return None
                    trace.mark('upstream_open_started')
                    with urllib.request.build_opener(relay.proxy_handler, NoRedirect).open(request, timeout=300) as response:
                        trace.headers(response.status, response.headers.get('Content-Type', 'application/json'),
                                      response.headers.get('x-request-id'))
                        self.send_response(response.status)
                        for name,value in forwarding_headers(response.headers).items():
                            if not (relay.key and relay.key in value):
                                self.send_header(name,value)
                        self.end_headers()
                        while chunk := response.read1(65536):
                            received_at = trace.received(chunk)
                            self.wfile.write(chunk)
                            self.wfile.flush()
                            trace.feed(chunk, received_at)  # Observe only after forwarding this chunk.
                        trace.finish()
                    relay.log({'event': 'response', 'path': self.path, 'duration_s': time.monotonic()-start, **trace.result()})
                except (BrokenPipeError, ConnectionResetError):
                    relay.log({'event': 'client_disconnected', 'path': self.path,
                               'duration_s': time.monotonic()-start, **trace.result()})
                except Exception as exc:
                    status = exc.code if isinstance(exc, urllib.error.HTTPError) else 502
                    if isinstance(exc, urllib.error.HTTPError):
                        trace.headers(status, exc.headers.get('Content-Type', ''), exc.headers.get('x-request-id'))
                    relay.log({'event': 'provider_error', 'status': status, 'kind': type(exc).__name__,
                               'path': self.path, 'duration_s': time.monotonic()-start, **trace.result()})
                    # Do not forward provider error strings, headers or bodies that can echo credentials.
                    try:
                        self.send_error(status, 'Model request failed; inspect host model-usage.jsonl')
                    except (BrokenPipeError, ConnectionResetError):
                        pass

        self.path.unlink(missing_ok=True)
        self.server = UnixHTTPServer(str(self.path), Handler)
        os.chmod(self.path, 0o600)

    def log(self, event):
        with self.lock:
            with (self.root/'model-usage.jsonl').open('a') as f:
                f.write(json.dumps({'time': time.time(), 'model': self.model, **event})+'\n')

    def start(self):
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.path.unlink(missing_ok=True)
