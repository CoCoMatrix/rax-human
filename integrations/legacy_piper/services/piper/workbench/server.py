"""Loopback operator web application, authenticated and protected against replay."""
import argparse
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
import os
from pathlib import Path
import secrets
import signal
import threading
from urllib.parse import urlsplit, parse_qs

from .common import ROOT, DATA
from .core import Workbench, WorkbenchError

STATIC = Path(__file__).parent / 'static'
COOKIE = 'rax_workbench_operator'
ASSETS = {'/': ('index.html', 'text/html; charset=utf-8'), '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
          '/style.css': ('style.css', 'text/css; charset=utf-8'), '/favicon.svg': ('favicon.svg', 'image/svg+xml')}


class Handler(BaseHTTPRequestHandler):
    server_version = 'RAXWorkbench/1.0'

    def log_message(self, fmt, *args):
        # Never log the bootstrap token or arbitrary POST contents.
        if self.command == 'POST': print('POST', urlsplit(self.path).path, flush=True)

    def send(self, status, body, mime='application/json; charset=utf-8', extra=()):
        if not isinstance(body, bytes): body = json.dumps(body, ensure_ascii=False, default=str).encode()
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
        for k,v in extra: self.send_header(k,v)
        self.end_headers()
        try: self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError): pass

    def authorized(self):
        supplied = self.headers.get('X-Operator-Token', '')
        if not supplied:
            try:
                c = SimpleCookie(self.headers.get('Cookie', '')).get(COOKIE)
                supplied = c.value if c else ''
            except Exception: supplied = ''
        return bool(supplied) and hmac.compare_digest(supplied, self.server.token)

    def do_GET(self):
        parsed = urlsplit(self.path); path = parsed.path
        if path == '/':
            token = parse_qs(parsed.query).get('token', [''])[0]
            if token and hmac.compare_digest(token, self.server.token):
                self.send(303, b'', 'text/plain', [('Location', '/'), ('Set-Cookie', f'{COOKIE}={token}; HttpOnly; SameSite=Strict; Path=/')])
                return
        if path in ASSETS:
            filename, mime = ASSETS[path]
            self.send(200, (STATIC / filename).read_bytes(), mime); return
        if not self.authorized():
            self.send(401, {'error': {'code': 'AUTH', 'message': '请使用本次工作台的专属链接打开', 'fix': '链接保存在 commissioning/workbench/access.json'}}); return
        try:
            if path == '/api/state': self.send(200, self.server.core.state())
            elif path.startswith('/api/camera/') and path.endswith('.jpg'):
                self.send(200, self.server.core.image(path.split('/')[-1][:-4]), 'image/jpeg')
            elif path.startswith('/api/segmentation/') and path.endswith('.jpg'):
                self.send(200, self.server.core.segmentation_image(path.split('/')[-1][:-4],
                    parse_qs(parsed.query).get('pack', [None])[0]), 'image/jpeg')
            elif path == '/api/guide':
                self.send(200, {'text': (ROOT / 'docs/piper/WORKBENCH.md').read_text()})
            else: self.send(404, {'error': {'message': 'Not found'}})
        except WorkbenchError as exc: self.send(409, {'error': {'code': exc.code, 'message': str(exc), 'fix': exc.fix}})
        except Exception as exc: self.send(500, {'error': {'message': str(exc)[:500]}})

    def do_POST(self):
        if not self.authorized(): self.send(401, {'error': {'code': 'AUTH', 'message': '操作员认证失败'}}); return
        origin = self.headers.get('Origin')
        if origin and (urlsplit(origin).netloc != self.headers.get('Host') or urlsplit(origin).scheme != 'http'):
            self.send(403, {'error': {'code': 'ORIGIN', 'message': '跨站操作已拒绝'}}); return
        if self.headers.get('Sec-Fetch-Site') == 'cross-site':
            self.send(403, {'error': {'code': 'ORIGIN', 'message': '跨站操作已拒绝'}}); return
        try:
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json': raise ValueError('JSON required')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 16384: raise ValueError('request body size invalid')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict): raise ValueError('JSON object required')
            path = urlsplit(self.path).path
            if not path.startswith('/api/action/'): self.send(404, {'error': {'message': 'Not found'}}); return
            result = self.server.core.command(path.rsplit('/', 1)[-1], body)
            self.send(200, result)
        except WorkbenchError as exc: self.send(409, {'error': {'code': exc.code, 'message': str(exc), 'fix': exc.fix}})
        except (ValueError, TypeError) as exc: self.send(400, {'error': {'code': 'INPUT', 'message': str(exc)}})
        except Exception as exc: self.send(500, {'error': {'code': 'SERVER', 'message': str(exc)[:500], 'fix': '查看运行日志'}})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8780)
    args = parser.parse_args()
    os.environ['RAX_ARTIFACT_ROOT'] = str(ROOT / 'commissioning')
    os.environ['RAX_SKILL_LIBRARY'] = str(ROOT / 'commissioning/task60-20261029/skill-library')
    DATA.mkdir(parents=True, exist_ok=True)
    private = DATA / 'private'; private.mkdir(mode=0o700, exist_ok=True)
    token_file = private / 'web-token'
    token = token_file.read_text().strip() if token_file.exists() else secrets.token_urlsafe(32)
    if len(token) < 32: raise ValueError('Invalid persisted operator token')
    token_file.write_text(token); token_file.chmod(0o600)
    core = Workbench()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.core, server.token = core, token
    def stop(*_): threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    print(f'RAX workbench listening on http://127.0.0.1:{args.port}; access token saved privately', flush=True)
    try: server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()
        core.close()


if __name__ == '__main__': main()
