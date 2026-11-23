"""Loopback-only operator dashboard for robot agent sessions.

Privileged: it launches paid agent sessions, arms robot motion and reads every artifact.
Never give its URL or token to an agent. The agent's sandbox cannot reach host loopback
TCP, so the dashboard is outside the agent's reach by construction.

    bash scripts/run_dashboard.sh [--host 127.0.0.1] [--port 8770]
"""
from __future__ import annotations

import argparse
import errno
from http import HTTPStatus
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
from pathlib import Path
import secrets
import urllib.parse

from services.dashboard.sessions import Sessions

STATIC = Path(__file__).resolve().parent/'static'
ASSETS = {'/': ('index.html', 'text/html; charset=utf-8'),
          '/app.js': ('app.js', 'application/javascript; charset=utf-8')}
COOKIE = 'rax_dashboard_token'
MAX_BODY = 1 << 20


class Handler(BaseHTTPRequestHandler):
    sessions: Sessions
    token: str
    server_version = 'RaxOperator/1.0'

    def log_message(self, format_string, *args):
        if not self.path.startswith(('/api/robot', '/api/session/')) or 'POST' in self.requestline:
            print(f'[dashboard] {self.address_string()} {format_string % args}', flush=True)

    # ----- plumbing -----------------------------------------------------------------------
    def _send(self, status, data, content_type, extra=()):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; img-src 'self'; style-src 'unsafe-inline'; script-src 'self'; "
                         "connect-src 'self'; frame-ancestors 'none'")
        for name, value in extra:
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(data)

    def _json(self, status, value):
        self._send(status, json.dumps(value, ensure_ascii=False, default=str).encode(),
                   'application/json; charset=utf-8')

    def _error(self, status, message):
        self._json(status, {'error': message})

    def _supplied_token(self):
        supplied = self.headers.get('X-Operator-Token', '')
        if not supplied:
            try:
                cookie = SimpleCookie(self.headers.get('Cookie', '')).get(COOKIE)
                supplied = cookie.value if cookie else ''
            except Exception:  # noqa: BLE001 - a malformed cookie is simply unauthorized
                supplied = ''
        return supplied

    def _authorized(self):
        supplied = self._supplied_token()
        return bool(supplied) and hmac.compare_digest(supplied, self.token)

    def _body(self):
        if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
            raise ValueError('POST bodies must be application/json')  # blocks cross-site form posts
        length = int(self.headers.get('Content-Length') or 0)
        if length > MAX_BODY:
            raise ValueError('request too large')
        value = json.loads(self.rfile.read(length) or b'{}')
        if not isinstance(value, dict):
            raise ValueError('expected a JSON object')
        return value

    def _route(self):
        parsed = urllib.parse.urlsplit(self.path)
        return parsed.path, urllib.parse.parse_qs(parsed.query)

    def _guarded(self, action):
        try:
            action()
        except PermissionError as exc:
            self._error(HTTPStatus.FORBIDDEN, str(exc))
        except KeyError as exc:
            self._error(HTTPStatus.NOT_FOUND, str(exc.args[0] if exc.args else exc))
        except (ValueError, TypeError) as exc:
            self._error(HTTPStatus.BAD_REQUEST, str(exc))
        except RuntimeError as exc:
            self._error(HTTPStatus.CONFLICT, str(exc))
        except Exception as exc:  # noqa: BLE001 - surface to the operator
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, f'{type(exc).__name__}: {exc}')

    # ----- GET ------------------------------------------------------------------------------
    def do_GET(self):
        path, query = self._route()
        if path in ASSETS:
            if path == '/':
                supplied = (query.get('token') or [''])[0]
                if supplied and hmac.compare_digest(supplied, self.token):
                    cookie = f'{COOKIE}={self.token}; HttpOnly; SameSite=Strict; Path=/'
                    self._send(HTTPStatus.SEE_OTHER, b'', 'text/plain', [('Location', '/'), ('Set-Cookie', cookie)])
                    return
                if not self._authorized():
                    self._send(HTTPStatus.UNAUTHORIZED, b'Open the tokenized URL printed by run_dashboard.sh.',
                               'text/plain; charset=utf-8')
                    return
            name, content_type = ASSETS[path]
            self._send(HTTPStatus.OK, (STATIC/name).read_bytes(), content_type)
            return
        if not self._authorized():
            self._error(HTTPStatus.UNAUTHORIZED, 'operator token required')
            return
        parts = path.strip('/').split('/')
        sessions = self.sessions

        def handle():
            if path == '/api/sessions':
                self._json(HTTPStatus.OK, {'sessions': sessions.list(), 'running': sessions.running()})
            elif path == '/api/robot':
                self._json(HTTPStatus.OK, sessions.robot())
            elif len(parts) == 4 and parts[:2] == ['api', 'session'] and parts[3] == 'activity':
                self._json(HTTPStatus.OK, sessions.activity(parts[2]))
            elif len(parts) == 4 and parts[:2] == ['api', 'session'] and parts[3] == 'frame':
                self._json(HTTPStatus.OK, {'frame': sessions.latest_frame(parts[2])})
            elif len(parts) == 6 and parts[:2] == ['api', 'session'] and parts[3] == 'frame' \
                    and parts[5].endswith('.jpg'):
                self._send(HTTPStatus.OK, sessions.frame_image(parts[2], parts[4], parts[5][:-4]), 'image/jpeg')
            else:
                self._error(HTTPStatus.NOT_FOUND, 'not found')
        self._guarded(handle)

    # ----- POST ---------------------------------------------------------------------------
    def do_POST(self):
        if not self._authorized():
            self._error(HTTPStatus.UNAUTHORIZED, 'operator token required')
            return
        path, _ = self._route()
        parts = path.strip('/').split('/')
        sessions = self.sessions

        def handle():
            body = self._body()
            if path == '/api/configure':
                self._json(HTTPStatus.OK, sessions.configure(body))
            elif path == '/api/disarm-all':
                self._json(HTTPStatus.OK, sessions.disarm_all())
            elif len(parts) == 4 and parts[:2] == ['api', 'session']:
                session_id, action = parts[2], parts[3]
                if action == 'launch':
                    self._json(HTTPStatus.OK, sessions.launch(session_id))
                elif action == 'resume':
                    self._json(HTTPStatus.OK, sessions.launch(session_id, resume=True))
                elif action == 'stop':
                    self._json(HTTPStatus.OK, sessions.stop(session_id))
                elif action == 'arm':
                    self._json(HTTPStatus.OK, sessions.arm(session_id, confirmed=body.get('estop_confirmed'),
                                                           minutes=body.get('minutes')))
                elif action == 'disarm':
                    self._json(HTTPStatus.OK, sessions.disarm(session_id))
                elif action == 'reply':
                    self._json(HTTPStatus.OK, sessions.reply(session_id, body.get('request_id'),
                                                             str(body.get('text', '')), done=body.get('done', True)))
                else:
                    self._error(HTTPStatus.NOT_FOUND, 'not found')
            else:
                self._error(HTTPStatus.NOT_FOUND, 'not found')
        self._guarded(handle)


def make_server(host, port, sessions=None, token=None):
    handler = type('BoundHandler', (Handler,), {'sessions': sessions or Sessions(),
                                                'token': token or secrets.token_urlsafe(24)})
    return ThreadingHTTPServer((host, port), handler), handler.token


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    from services.backends import BACKENDS
    parser.add_argument('--robot', choices=sorted(BACKENDS), default=None)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8770,
                        help='first port to try; the next free one up to +9 is used if it is taken')
    args = parser.parse_args()
    if args.host not in ('127.0.0.1', 'localhost', '::1'):
        print('WARNING: binding beyond loopback exposes robot arming to the network', flush=True)
    for port in range(args.port, args.port + 10):
        try:
            server, token = make_server(args.host, port, sessions=Sessions(robot=args.robot))
            break
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise
    else:
        raise SystemExit(f'ports {args.port}..{args.port + 9} are all in use; pass --port')
    print(f'Operator dashboard: http://{args.host}:{port}/?token={token}', flush=True)
    print(f'Remote: ssh -L {port}:127.0.0.1:{port} <workstation>, then open the URL above locally.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
