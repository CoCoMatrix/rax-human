"""Runs inside bwrap: loopback HTTP forwarding to the sole host model Unix socket."""
import http.client
import http.server
import os
import signal
import socket
import subprocess
import sys
import threading



def forwarding_headers(headers, *, replace_auth=False):
    """Preserve end-to-end metadata; framing/routing belongs to each HTTP hop."""
    pairs=list(headers.items()) if hasattr(headers, 'items') else list(headers)
    drop={'connection','keep-alive','proxy-authenticate','proxy-authorization',
          'te','trailer','transfer-encoding','upgrade','host','content-length'}
    for name,value in pairs:
        if name.lower()=='connection':drop.update(v.strip().lower() for v in value.split(','))
    if replace_auth:drop.add('authorization')
    return {name:value for name,value in pairs if name.lower() not in drop}

class UnixConnection(http.client.HTTPConnection):
    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX)
        self.sock.connect('/run/model.sock')


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.0'
    def log_message(self, *_):
        pass
    def do_POST(self):
        size = int(self.headers.get('Content-Length', '0'))
        if not 0 < size <= 32 * 1024 * 1024:
            self.send_error(413)
            return
        connection = UnixConnection('relay', timeout=360)
        try:
            connection.request('POST', self.path, self.rfile.read(size), forwarding_headers(self.headers))
            response = connection.getresponse()
            self.send_response(response.status)
            for name,value in forwarding_headers(response.getheaders()).items():
                self.send_header(name,value)
            self.end_headers()
            while block := response.read1(65536):
                self.wfile.write(block)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            connection.close()


server = http.server.ThreadingHTTPServer(('127.0.0.1', 8789), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
child = subprocess.Popen(sys.argv[1:])
def stop(signum, _):
    child.send_signal(signum)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
try:
    raise SystemExit(child.wait())
finally:
    if child.poll() is None:
        child.terminate()
    server.shutdown()
