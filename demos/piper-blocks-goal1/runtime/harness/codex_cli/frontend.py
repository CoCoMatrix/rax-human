"""Host-owned Codex MCP service, using the upstream main-thread dispatcher.

Signals interrupt blocking tools on the same thread before backend/session cleanup.
The launcher never closes this process's backend or solver from another thread.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time

from services.backends import make_backend
from services.controller.config import Configuration
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.controller.codex_vision import VisionBackend, VisionRouter
from services.controller.gemini import load_key
from services.controller.google_gemini import GoogleGeminiRouter
from services.json_io import write_private_json
from .credentials import load_credentials


def serve(config, socket_path, *, started_at=None, budget_started_monotonic=None, budget_previous_elapsed=0):
    session = backend = frontend = listener = router = None
    stop_signal = None
    report = {'pid': os.getpid(), 'started_at': time.time(), 'stop_requested': False,
              'backend_closed': False, 'session_closed': False,
              'shutdown_completed': False, 'physical_stop_confirmed': False}
    owned_socket = False

    def stop(signum, _):
        nonlocal stop_signal
        stop_signal = signum
        # A repeated stop must not interrupt cleanup halfway through.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise SystemExit(0)

    old = {sig: signal.signal(sig, stop) for sig in (signal.SIGTERM, signal.SIGINT)}
    try:
        settings = json.loads((config.root/'codex-relay.json').read_text())
        provider = settings.get('vision_provider', 'codex')
        if provider not in ('google', 'codex'): raise ValueError('Unknown auxiliary vision provider')
        if provider == 'google':
            router = GoogleGeminiRouter(load_key(settings['gemini_key_file']), proxy_url=settings.get('http_proxy'))
        else:
            key, base_url = load_credentials(settings['key_file'], settings['base_url'])
            router = VisionRouter(key, base_url, proxy_url=settings.get('http_proxy'))
        session = Session(config)
        backend = make_backend(config)
        frontend = Frontend(session, backend if provider == 'google' else VisionBackend(backend, router, session),
            gemini=router if provider == 'google' else None, started_at=started_at,
            budget_started_monotonic=budget_started_monotonic, budget_previous_elapsed=budget_previous_elapsed)
        # Only the process which acquired Session ownership may replace its socket.
        socket_path.unlink(missing_ok=True)
        listener = socket.socket(socket.AF_UNIX)
        listener.bind(str(socket_path))
        owned_socket = True
        os.chmod(socket_path, 0o600)
        listener.listen(16)
        write_private_json(config.root/'frontend-ready.json', {'pid': os.getpid(), 'socket': str(socket_path)})
        print(f'MCP listening on {socket_path}', flush=True)
        frontend.serve_socket(listener)  # deliberately on the main thread
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        report['stop_requested'] = stop_signal is not None
        report['stop_signal'] = stop_signal
        try:
            if backend is not None:
                backend.close()
                report['backend_closed'] = True
        except BaseException as exc:
            report['cleanup_error_type'] = type(exc).__name__
            raise
        finally:
            try:
                if router is not None and hasattr(router, 'close'):
                    router.close()
                if session is not None:
                    session.close()
                    report['session_closed'] = True
            finally:
                if listener is not None:
                    listener.close()
                if owned_socket:
                    socket_path.unlink(missing_ok=True)
                report['shutdown_completed'] = report['backend_closed'] and report['session_closed']
                report['finished_at'] = time.time()
                write_private_json(config.root/'frontend-exit.json', report)
                for sig, handler in old.items():
                    signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--started-at', type=float)
    parser.add_argument('--budget-started-monotonic', type=float)
    parser.add_argument('--budget-previous-elapsed', type=float, default=0)
    args = parser.parse_args()
    serve(Configuration.load(args.config), args.socket, started_at=args.started_at,
          budget_started_monotonic=args.budget_started_monotonic,
          budget_previous_elapsed=args.budget_previous_elapsed)


if __name__ == '__main__':
    main()
