"""Stdlib MCP client for robot and Gemini calls from your own Python.

`Context()` connects to the same MCP tools you call directly, through the session's
private socket (`RAX_MCP_SOCKET`). Calls are serialized with the agent's own
direct tool calls by the host robot service.

    from api.runtime import Context
    with Context() as ctx:
        state = ctx.call('status')
"""
import json
import os
import socket
import sys
import threading

PROTOCOL_VERSION = '2025-06-18'


class Context:
    def __init__(self, path=None):
        path = path or os.environ.get('RAX_MCP_SOCKET')
        if not path:
            raise RuntimeError('RAX_MCP_SOCKET is not set; run inside the agent session')
        self._lock = threading.RLock()
        self._sequence = 0
        self._socket = socket.socket(socket.AF_UNIX)
        try:
            self._socket.connect(path)
            self._reader, self._writer = self._socket.makefile('r'), self._socket.makefile('w')
            self.request('initialize', {'protocolVersion': PROTOCOL_VERSION, 'capabilities': {},
                                        'clientInfo': {'name': 'python-controller', 'version': '3'}})
        except BaseException:
            self._socket.close()
            raise

    def close(self):
        if self._socket:
            self._writer.close()
            self._reader.close()
            self._socket.close()
            self._socket = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def request(self, method, params):
        with self._lock:
            self._sequence += 1
            request = {'jsonrpc': '2.0', 'id': self._sequence, 'method': method, 'params': params}
            self._writer.write(json.dumps(request, allow_nan=False) + '\n')
            self._writer.flush()
            line = self._reader.readline(64 * 1024 * 1024)
            if not line:
                raise ConnectionError('The robot service closed the connection')
            response = json.loads(line)
            if response.get('id') != self._sequence:
                raise RuntimeError('MCP request failed: identity mismatch')
            if 'error' in response:
                raise RuntimeError('MCP tool failed: ' + json.dumps({'error': response['error']}, allow_nan=False))
            return response['result']

    def call(self, tool, **arguments):
        """Call one MCP tool; returns {'content': [...]} or raises on a tool error."""
        result = self.request('tools/call', {'name': tool, 'arguments': arguments})
        # A controller may print only a compact numerical result. Surface new
        # operator messages independently so user steering is not lost in a log.
        # This is notification only; it neither changes checks nor authorizes motion.
        for block in result.get('content', []):
            if block.get('type') != 'text':
                continue
            try:
                value = json.loads(block.get('text', ''))
            except (ValueError, TypeError):
                continue
            updates = value.get('operator_updates') if isinstance(value, dict) else None
            if updates:
                print('RAX_OPERATOR_UPDATES ' + json.dumps(updates, ensure_ascii=False),
                      file=sys.stderr, flush=True)
        if result.get('isError'):
            text = next((b.get('text', '') for b in result.get('content', []) if b.get('type') == 'text'), '')
            raise RuntimeError('MCP tool failed: ' + text)
        return result

    def value(self, tool, **arguments):
        """Call a tool and decode its JSON text block."""
        result = self.call(tool, **arguments)
        text = next(b['text'] for b in result['content'] if b.get('type') == 'text')
        return json.loads(text)

    @staticmethod
    def images(result):
        """The image blocks of a tool result ({'mimeType', 'data'(base64)})."""
        return [b for b in result.get('content', []) if b.get('type') == 'image']
