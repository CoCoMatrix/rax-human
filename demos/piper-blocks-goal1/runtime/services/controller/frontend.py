"""Host-owned robot/API MCP server: the agent's only path to the robot and to Gemini.

One process per session owns the robot backend (services.backends). It speaks
newline-delimited JSON-RPC (MCP tools) on a private Unix socket, multiplexing the
agent CLI's stdio bridge (harness/claude_cli/mcp_bridge.py) and the agent's own
Python scripts (auto_research_agent/api/runtime.py); every call runs on one
dispatcher thread, so two clients never drive the robot at once. Without --socket it
serves a single client on stdin/stdout.
"""
import argparse
import json
import math
import os
from pathlib import Path
import selectors
import signal
import socket
import sys
import threading
import time
import uuid

from services.backends import make_backend
from services.json_io import jsonable
from .billing import reserve_cost
from .config import Configuration
from . import operator
from .errors import public_failure
from .gemini import GeminiRouter, MAX_REQUEST_BYTES, load_key
from .session import Session
from .timing import Timings

PROTOCOL_VERSION = '2025-06-18'


def tool(name, description, properties=None, required=()):
    return {'name': name, 'description': description, 'inputSchema': {'type': 'object',
            'properties': properties or {}, 'required': list(required), 'additionalProperties': False}}


STATUS = tool('status', 'Session task, wall-clock and Gemini budgets, the live robot summary, and whether '
              'an earlier motion failure left control uncertain.')
OPERATOR = tool('request_operator',
    'Ask the human operator for help or for a scene reset, and wait for the answer. kind "reset": the '
    'operator rearranges the scene (e.g. after a cube fell off the table or to start a new attempt) and '
    'replies when done; "help": a question or a problem you cannot solve through the tools (stuck '
    'object, unclear task, robot fault). Say exactly what you need. The robot does nothing while you wait; '
    'observe again after the answer, because the scene may have changed. To wait again or retrieve a late '
    'answer, supply only request_id and optional timeout_s (0 retrieves without waiting). '
    'New answers also accompany the next tool result as operator_updates; status retains recent answers. '
    'Timeout never grants permission; an answer is evidence for its stated question, not a motion command.',
    {'kind': {'type': 'string', 'enum': ['help', 'reset']},
     'message': {'type': 'string', 'maxLength': 4000},
     'request_id': {'type':'string','pattern':'^[0-9a-f]{12}$'},
     'timeout_s': {'type': 'number', 'minimum': 0, 'maximum': 3600,
                   'description': 'How long to wait for the answer (default 900).'}},
    [])
SKILL_MERGE = tool('skill_library_merge',
    'Check committed skill-library session changes and merge into this robot shared library. '
    'No skill code executes; no remote push. Returns problems or conflicts without publishing.',
    {'message': {'type': 'string', 'minLength': 1, 'maxLength': 200}}, ['message'])

GEMINI = {
    'name': 'gemini_generate',
    'description': ('OpenRouter-style chat completion via Gemini 3.8 Flash: text/images, JSON schema output and '
                    'client function calling. Returns the complete assistant message plus usage/cost/API budget; '
                    'Gemini spending is capped for the session. Per-request size/output limits apply. Preserve '
                    'assistant messages including reasoning_details when sending tool results. Tool calls are '
                    'data only: your script validates and executes functions. Images: {type: image, mimeType: '
                    'image/png or image/jpeg, data: base64}, or image_url with an inline data URL. No paths, '
                    'remote URLs or server tools. See the gemini skill references/api.md.'),
    'inputSchema': {'type': 'object', 'additionalProperties': False, 'required': ['messages'], 'properties': {
        'messages': {'type': 'array', 'minItems': 1, 'maxItems': 64, 'items': {'type': 'object'},
                     'description': 'Chat messages: system/user/assistant/tool. Tool results use tool_call_id '
                                    'and string content; preserve complete assistant messages.'},
        'max_tokens': {'type': 'integer', 'minimum': 1, 'maximum': 16384},
        'temperature': {'type': 'number', 'minimum': 0, 'maximum': 2},
        'response_format': {'type': 'object', 'description': 'text, json_object, or json_schema with {name, strict, schema}.'},
        'tools': {'type': 'array', 'maxItems': 64, 'items': {'type': 'object'},
                  'description': 'Client function declarations: {type: function, function: {name, description, parameters, strict?}}.'},
        'tool_choice': {'oneOf': [{'type': 'string', 'enum': ['auto', 'none', 'required']}, {'type': 'object'}],
                        'description': 'Or {type: function, function: {name: declared_name}}.'},
        'reasoning': {'type': 'object', 'description': 'OpenRouter reasoning options: effort or max_tokens, optionally enabled/exclude.'},
    }},
}


class Frontend:
    def __init__(self, session, backend, gemini=None, *, started_at=None,
                 budget_started_monotonic=None, budget_previous_elapsed=0):
        self.s, self.backend, self.gemini = session, backend, gemini
        self.started_at = started_at
        self._clock_started = (time.monotonic() if budget_started_monotonic is None
                               else budget_started_monotonic)
        self._elapsed_at_start = (max(0, time.time()-started_at) if started_at is not None else 0)
        if budget_started_monotonic is not None:
            self._elapsed_at_start = budget_previous_elapsed
        self.timings = Timings()
        # Set when a motion tool raised after dispatch; reported until a motion completes.
        # Interlocks (arming, workspace limits, holds) belong to the backend itself.
        self.control_uncertain = False
        self._delivered_operator_answers = set()
        self._dispatch = threading.Lock()
        self._backend_tools = {t['name']: t for t in backend.tools()}
        root = getattr(self.s.config, 'root', None)
        self.workspace = Path(root)/'agent-workspace' if root is not None else None
        self.library = bool(self.workspace and (self.workspace/'skill-library/.git').is_dir())
        reserved = {'status', 'gemini_generate', 'request_operator', 'skill_library_merge'} & set(self._backend_tools)
        if reserved:
            raise ValueError(f'Backend tools may not reuse harness tool names: {sorted(reserved)}')

    def tools(self):
        gemini_tool = getattr(self.gemini, 'tool_schema', lambda definition: definition)(GEMINI) if self.gemini else None
        return [STATUS, OPERATOR, *([SKILL_MERGE] if self.library else []), *self._backend_tools.values(), *([gemini_tool] if gemini_tool else [])]

    def wall_budget(self):
        if self.started_at is None:
            return None
        elapsed = self._elapsed_at_start + time.monotonic()-self._clock_started
        total = self.s.config.session_seconds
        remaining = max(0, total-elapsed)
        reserve = min(60, total/3)
        return {'session_seconds': total, 'elapsed_seconds': round(elapsed, 3),
                'remaining_seconds': round(remaining, 3), 'finish_reserve_seconds': reserve,
                'phase': 'expired' if remaining <= 0 else 'wind_down' if remaining <= reserve else 'active',
                'instruction': 'Use this measured clock. During wind_down save PROGRESS/INDEX and finish; '
                               'do not begin another physical action. A held object is not released automatically.'}

    def status(self):
        config = self.s.config
        value = {'task': config.task, 'robot': config.robot, 'observation_profile': config.observation_profile,
                 'control_uncertain': self.control_uncertain, 'api_budget': self.s.budget()}
        value['operator_requests'] = operator.recent(config.root)
        if self.started_at is not None:
            value['session_seconds'] = config.session_seconds
            value['seconds_since_launch'] = self.wall_budget()['elapsed_seconds']
            value['wall_budget'] = self.wall_budget()
        try:
            value['robot_status'] = self.backend.status()
        except Exception as exc:
            value['robot_status_error'] = public_failure(exc, operation='status')
        return value

    def gemini_call(self, arguments, failure):
        failure['stage'] = 'api_validation'
        payload, reservation = self.gemini.prepare(arguments)
        cost_reservation = reserve_cost(reservation, payload['max_tokens'])
        from .google_gemini import GoogleGeminiRouter
        import ssl,http.client
        attempts=2 if isinstance(self.gemini,GoogleGeminiRouter) else 1
        reconnects=[]
        for attempt in range(attempts):
            failure['stage'] = 'api_budget'
            wall=self.wall_budget()
            if wall and wall['remaining_seconds']<=wall['finish_reserve_seconds']:
                raise TimeoutError('Visual retry would consume session finish reserve')
            self.s.charge(reservation, cost_reservation)  # EACH attempt may be billable.
            failure['stage'] = 'provider_request'
            try:
                value, used = self.gemini.execute(payload, reservation)
                break
            except (ssl.SSLEOFError,http.client.RemoteDisconnected,ConnectionResetError) as exc:
                # Only retry this read-only visual POST. Never replay a robot tool.
                # Keep the failed reservation: upstream may already have billed it.
                if attempt+1>=attempts:raise
                reconnects.append(type(exc).__name__)
                self.gemini.close()
                print(json.dumps({'event':'gemini_request_reconnect','failed_attempt':attempt+1,
                    'error_type':type(exc).__name__,'next_attempt':attempt+2}),file=sys.stderr,flush=True)
                time.sleep(.25)
        failure['stage'] = 'api_accounting'
        self.s.refund(reservation - used, value['usage'], cost_reservation,
                      cost_upper_bound=value.get('cost_upper_bound_usd'))
        value['budget'] = self.s.budget()
        value['provider_request_attempts']=attempt+1
        value['transport_reconnects']=reconnects
        return value, []

    def _operator_updates(self, value):
        # Reuse the existing durable request/reply files. Restart redelivery is
        # intentional; an answer supplies facts, never arming or fault recovery.
        if not isinstance(value, dict):
            return
        try:
            updates = [r for r in operator.recent(self.s.config.root, limit=None)
                       if r['answer'] is not None and r['request_id'] not in self._delivered_operator_answers]
        except (OSError, ValueError, TypeError, KeyError):
            # Sideband delivery must not replace a completed motion receipt or
            # obscure its original error. No unseen answer is treated as consent.
            value['operator_updates_error'] = (
                'Could not read operator reply records. Retrieve the known request_id '
                'with request_operator; no confirmation is inferred.')
            return
        if updates:
            value['operator_updates'] = updates
            self._delivered_operator_answers.update(r['request_id'] for r in updates)

    def call(self, name, arguments):
        """One tools/call; returns an MCP CallToolResult."""
        call_id = uuid.uuid4().hex
        failure = {'stage': 'request'}
        mutating = False
        with self._dispatch, self.timings.measure(name if isinstance(name, str) else 'invalid'):
            try:
                if not isinstance(arguments, dict):
                    raise ValueError('Arguments must be an object')
                self.s.audit({'event': 'mcp_request', 'call_id': call_id, 'tool': name, 'arguments': arguments})
                if name == 'status':
                    if arguments:
                        raise ValueError('status takes no arguments')
                    value, images = self.status(), []
                elif name == 'request_operator':
                    failure['stage'] = 'validation'
                    if set(arguments) - {'kind', 'message', 'timeout_s', 'request_id'}:
                        raise ValueError('request_operator takes kind/message OR request_id, and timeout_s')
                    timeout = arguments.get('timeout_s', 900)
                    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 <= timeout <= 3600:
                        raise ValueError('timeout_s must be a finite number in 0..3600')
                    if 'request_id' in arguments:
                        if 'kind' in arguments or 'message' in arguments:
                            raise ValueError('Cannot change an existing question')
                        request = operator.get_request(self.s.config.root, arguments['request_id'])
                    else:
                        state=self.backend.status()
                        context={'measured14':state.get('measured_state',{}).get('values'),
                                 'owner_generation':state.get('owner_status',{}).get('generation'),
                                 'bridge_instance':(state.get('bridge') or {}).get('instance')}
                        request = operator.open_request(self.s.config.root, arguments.get('kind'), arguments.get('message'),context=context)
                    print(f"OPERATOR REQUEST [{request['kind']}] {request['message']}", file=sys.stderr, flush=True)
                    failure['stage'] = 'operator_wait'
                    value = operator.wait(self.s.config.root, request, timeout)
                    try:
                        value['robot_status'] = self.backend.status()
                    except Exception as exc:
                        value['robot_status_error'] = public_failure(exc, operation='status')
                    images = []
                elif name == 'gemini_generate' and self.gemini:
                    value, images = self.gemini_call(arguments, failure)
                elif name == 'skill_library_merge' and self.library:
                    failure['stage'] = 'validation'
                    message = arguments.get('message')
                    if set(arguments) != {'message'} or not isinstance(message, str) or not message.strip() or len(message) > 200 or '\n' in message or '\r' in message:
                        raise ValueError('skill_library_merge takes a nonempty one-line message (at most 200 characters)')
                    from services import skill_library
                    value, images = skill_library.merge(self.s.config, self.workspace, message), []
                elif name in self._backend_tools:
                    failure['stage'] = 'validation'
                    validate = getattr(self.backend, 'validate', None)
                    if validate:
                        validate(name, arguments)
                    mutating = name in getattr(self.backend, 'mutating', ())
                    budget = self.wall_budget()
                    if mutating and budget and budget['phase'] != 'active':
                        raise PermissionError('Session is in its finish reserve; no action dispatched. '
                                              'Observe/status if needed, save handoff and finish.')
                    failure['stage'] = 'robot_execution'
                    value, images = self.backend.call(name, arguments)
                    if mutating:
                        self.control_uncertain = False  # A motion completed: control is known again.
                else:
                    raise PermissionError('Unknown tool')
                value = jsonable(value)
                if isinstance(value, dict) and self.started_at is not None:
                    value['wall_budget'] = self.wall_budget()
                    if isinstance(value.get('observation'), dict):
                        value['observation']['wall_budget'] = dict(value['wall_budget'])
                if isinstance(value, dict) and self.control_uncertain:
                    value.setdefault('control_uncertain', True)
                self._operator_updates(value)
                result = {'content': [{'type': 'text', 'text': json.dumps(value, allow_nan=False)}, *images]}
                self.s.audit({'event': 'mcp_result', 'call_id': call_id, 'tool': name,
                              'result': result['content'][0]['text'][:65536], 'images': len(images)})
                return result
            except Exception as exc:
                held = getattr(exc, 'control_certain', False) is True
                # A device adapter may finish expensive IK/owner checks inside call().
                # An explicit pre-dispatch refusal is validation, not a stopped motion.
                if (failure['stage'] == 'robot_execution'
                        and getattr(exc, 'dispatch_started', None) is False):
                    failure['stage'] = 'validation'
                    held = False
                if getattr(exc, 'control_uncertain', False) is True:
                    self.control_uncertain = True
                if mutating and failure['stage'] == 'robot_execution':
                    # A backend-handled stop (refusal, disarm, timeout) leaves the arm holding at a
                    # measured pose; anything else may have left motion in flight.
                    self.control_uncertain = not held
                error = public_failure(exc, operation=name, stage=failure['stage'], uncertain=self.control_uncertain,
                                       held=held and mutating)
                if name in self._backend_tools and not mutating:
                    # This read-only request did not act; an older uncertain
                    # command is still uncertain. Keep the two facts separate.
                    error.update(executed=False, dispatch_started=False,
                                 request_no_action_executed=True)
                    if not self.control_uncertain:error['no_action_executed']=True
                error['episode_ended']=False
                if getattr(exc, 'dispatch_started', None) is False:
                    # This request did not dispatch; an earlier unknown action remains unknown.
                    error.update(executed=False, dispatch_started=False)
                if isinstance(getattr(exc, 'robot_state', None), dict):
                    error['robot_state'] = jsonable(exc.robot_state)
                if self.started_at is not None:
                    error['wall_budget'] = self.wall_budget()
                self._operator_updates(error)
                try:
                    self.s.audit({'event': 'mcp_error', 'call_id': call_id, **error})
                except Exception:
                    error['audit_recording_failed'] = True
                return {'isError': True, 'content': [{'type': 'text', 'text': json.dumps({'error': error})}]}

    def handle(self, request):
        if not isinstance(request, dict):
            return {'jsonrpc': '2.0', 'id': None, 'error': {'code': -32600, 'message': 'Invalid request'}}
        ident = request.get('id')
        try:
            if request.get('jsonrpc') != '2.0':
                raise ValueError('Invalid JSON-RPC')
            method = request.get('method')
            if method == 'initialize':
                result = {'protocolVersion': PROTOCOL_VERSION, 'capabilities': {'tools': {}},
                          'serverInfo': {'name': 'robot', 'version': '1.0'}}
            elif isinstance(method, str) and method.startswith('notifications/'):
                return None
            elif method == 'ping':
                result = {}
            elif method == 'tools/list':
                result = {'tools': self.tools()}
            elif method == 'tools/call':
                params = request['params']
                result = self.call(params['name'], params.get('arguments', {}))
            else:
                raise PermissionError('Unsupported method')
            return {'jsonrpc': '2.0', 'id': ident, 'result': result}
        except Exception as exc:
            failure = public_failure(exc)
            return {'jsonrpc': '2.0', 'id': ident, 'error': {'code': -32000, 'message': failure['reason'], 'data': failure}}

    def _respond(self, line):
        try:
            request = json.loads(line)
        except (ValueError, UnicodeError):
            return {'jsonrpc': '2.0', 'id': None, 'error': {'code': -32700, 'message': 'Invalid request'}}
        return self.handle(request)

    def serve_socket(self, listener, stop=None):
        """Multiplex up to 16 clients; requests run one at a time on this thread."""
        listener.setblocking(False)
        buffers = {}
        with selectors.DefaultSelector() as poll:
            poll.register(listener, selectors.EVENT_READ)
            try:
                while not (stop and stop.is_set()):
                    for key, _ in poll.select(timeout=0.5):
                        connection = key.fileobj
                        if connection is listener:
                            try:
                                client, _ = listener.accept()
                            except BlockingIOError:
                                continue
                            if len(buffers) >= 16:
                                client.close()
                                continue
                            client.setblocking(False)
                            buffers[client] = bytearray()
                            poll.register(client, selectors.EVENT_READ)
                            continue
                        try:
                            block = connection.recv(65536)
                            if not block:
                                raise ConnectionError('Client closed')
                            pending = buffers[connection]
                            pending.extend(block)
                            if len(pending) > MAX_REQUEST_BYTES:
                                raise ConnectionError('Request exceeds wire limit')
                            while b'\n' in pending:
                                line, _, tail = pending.partition(b'\n')
                                pending[:] = tail
                                response = self._respond(line)
                                if response is not None:
                                    connection.settimeout(30)
                                    connection.sendall(json.dumps(response, allow_nan=False).encode() + b'\n')
                                    connection.setblocking(False)
                        except (ConnectionError, OSError):
                            poll.unregister(connection)
                            buffers.pop(connection, None)
                            connection.close()
            finally:
                for client in buffers:
                    client.close()

    def serve(self, reader, writer):
        while line := reader.readline(MAX_REQUEST_BYTES + 1):
            if len(line.encode('utf-8')) > MAX_REQUEST_BYTES:
                writer.write(json.dumps({'jsonrpc': '2.0', 'id': None,
                    'error': {'code': -32600, 'message': 'Request exceeds wire limit'}}) + '\n')
                writer.flush()
                return  # Do not interpret the remaining fragment as another request.
            response = self._respond(line)
            if response is not None:
                writer.write(json.dumps(response, allow_nan=False) + '\n')
                writer.flush()

    def close(self):
        try:
            self.backend.close()
        finally:
            self.s.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--socket', type=Path, help='Serve this private Unix socket instead of stdio')
    parser.add_argument('--key-file', type=Path, help='OpenRouter key for gemini_generate (optional)')
    parser.add_argument('--started-at', type=float, help='Launch wall-clock start (epoch seconds)')
    args = parser.parse_args()
    config = Configuration.load(args.config)
    key = load_key(args.key_file) if args.key_file else None
    session = Session(config)
    try:
        backend = make_backend(config)
    except BaseException:
        session.close()
        raise
    frontend = Frontend(session, backend, GeminiRouter(key) if key else None, started_at=args.started_at)
    def stop(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop)
    try:
        if args.socket:
            args.socket.unlink(missing_ok=True)
            with socket.socket(socket.AF_UNIX) as listener:
                listener.bind(str(args.socket))
                os.chmod(args.socket, 0o600)
                listener.listen(16)
                print(f'MCP listening on {args.socket}', flush=True)
                frontend.serve_socket(listener)
        else:
            frontend.serve(sys.stdin, sys.stdout)
    finally:
        frontend.close()
        if args.socket:
            args.socket.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
