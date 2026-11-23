"""Small pieces the host model relay shares: call counting, backoff and bounded servers."""
import json
import os
from pathlib import Path
import random
import socketserver
import threading

RETRY_MAX_DELAY_SECONDS = 60


def retry_delay(attempt, retry_after):
    """Provider Retry-After seconds when given, else jittered exponential backoff."""
    try:
        delay = float(retry_after)
    except (TypeError, ValueError):
        delay = min(RETRY_MAX_DELAY_SECONDS, 2**attempt) * random.uniform(.5, 1)
    return max(0., min(delay, RETRY_MAX_DELAY_SECONDS))


class _Bounded:
    """At most eight concurrent requests; extra connections are closed at once."""
    daemon_threads = True

    def __init__(self, *args, **kwargs):
        self.slots = threading.BoundedSemaphore(8)
        super().__init__(*args, **kwargs)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class UnixHTTPServer(_Bounded, socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    pass


class LoopbackHTTPServer(_Bounded, socketserver.ThreadingMixIn, socketserver.TCPServer):
    """Loopback TCP only: the host-run agent CLI reaches the relay at 127.0.0.1:<port>."""
    allow_reuse_address = True

    def __init__(self, address, handler):
        if address[0] not in ('127.0.0.1', 'localhost'):
            raise ValueError('The model relay listens on loopback only')
        super().__init__(address, handler)


class CallCounter:
    """Model calls of a session, persisted so the cap spans every launch (resume)."""

    def __init__(self, path=None):
        self.path = Path(path) if path else None
        self.value = 0
        if self.path is not None and self.path.is_file():
            self.value = int(json.loads(self.path.read_text()).get('calls', 0))

    def increment(self):
        self.value += 1
        if self.path is not None:
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'calls': self.value}))
            os.replace(temporary, self.path)
