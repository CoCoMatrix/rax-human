"""The robot service MCP: tool listing, dispatch, audit, uncertainty and the socket transport."""
import json
import socket
import threading

import pytest

from conftest import configuration, frontend


def text(result):
    return json.loads(result['content'][0]['text'])


def test_tools_are_status_plus_backend_tools(tmp_path):
    f = frontend(tmp_path)
    try:
        names = [t['name'] for t in f.tools()]
        assert names == ['status', 'request_operator', 'mock_observe', 'mock_move']  # No Gemini without a key.
        status = text(f.call('status', {}))
        assert status['task'] == 'stack_cups' and status['robot_status']['robot'] == 'mock'
        assert status['control_uncertain'] is False
    finally:
        f.close()


def test_validation_failure_moves_nothing_and_is_not_uncertain(tmp_path):
    f = frontend(tmp_path)
    try:
        reply = f.call('mock_move', {'position': [5, 0, 0]})
        error = text(reply)['error']
        assert reply['isError'] and error['stage'] == 'validation' and error['no_action_executed']
        assert f.backend.position == [0.0, 0.0, 0.0] and not f.control_uncertain
        assert text(f.call('mock_move', {'position': [0.1, 0.2, 0.3]})) == {'position': [0.1, 0.2, 0.3]}
    finally:
        f.close()


def test_a_failed_motion_marks_control_uncertain_until_a_motion_completes(tmp_path):
    f = frontend(tmp_path)
    original = f.backend.call
    def broken(name, arguments):
        raise RuntimeError('bridge dropped the connection')
    try:
        f.backend.call = broken
        error = text(f.call('mock_move', {'position': [0.1, 0, 0]}))['error']
        assert error['control_uncertain'] and not error['no_action_executed']
        f.backend.call = original
        assert text(f.call('mock_observe', {}))['control_uncertain'] is True
        assert 'control_uncertain' not in text(f.call('mock_move', {'position': [0, 0, 0]}))
    finally:
        f.close()


def test_unknown_tools_and_bad_arguments_are_refused(tmp_path):
    f = frontend(tmp_path)
    try:
        assert f.call('simulator_step', {})['isError']
        assert f.call('status', {'x': 1})['isError']
        assert f.call('mock_move', [1, 2])['isError']
        assert f.handle({'jsonrpc': '2.0', 'id': 1, 'method': 'resources/list'})['error']
    finally:
        f.close()


def test_every_call_is_audited(tmp_path):
    f = frontend(tmp_path)
    try:
        f.call('mock_move', {'position': [0.1, 0, 0]})
        f.call('mock_move', {'position': [9, 0, 0]})
    finally:
        f.close()
    events = [json.loads(line) for line in (configuration(tmp_path).root/'mcp.jsonl').read_text().splitlines()]
    assert [e['event'] for e in events] == ['mcp_request', 'mcp_result', 'mcp_request', 'mcp_error']


def test_backend_tools_may_not_shadow_harness_tools(tmp_path):
    from services.controller.frontend import Frontend
    from services.controller.session import Session

    class Shadow:
        def tools(self):
            return [{'name': 'status', 'description': '', 'inputSchema': {'type': 'object'}}]
    config = configuration(tmp_path)
    session = Session(config)
    try:
        with pytest.raises(ValueError, match='harness tool names'):
            Frontend(session, Shadow())
    finally:
        session.close()


def test_socket_serves_the_agent_cli_bridge_and_python_scripts(tmp_path, monkeypatch):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'auto_research_agent'))
    from api.runtime import Context
    f = frontend(tmp_path)
    path = tmp_path/'mcp.sock'
    stop = threading.Event()
    with socket.socket(socket.AF_UNIX) as listener:
        listener.bind(str(path))
        listener.listen(16)
        server = threading.Thread(target=f.serve_socket, args=(listener,), kwargs={'stop': stop}, daemon=True)
        server.start()
        try:
            monkeypatch.setenv('RAX_MCP_SOCKET', str(path))
            with Context() as first, Context() as second:
                first.call('mock_move', position=[0.2, 0, 0])
                assert second.value('mock_observe') == {'position': [0.2, 0.0, 0.0]}
                with pytest.raises(RuntimeError, match='MCP tool failed'):
                    second.call('mock_move', position=[3, 0, 0])
                tools = first.request('tools/list', {})['tools']
                assert [t['name'] for t in tools][0] == 'status'
        finally:
            stop.set()
            server.join(timeout=5)
            f.close()


def test_request_operator_waits_for_the_reply(tmp_path):
    from services.controller import operator
    f = frontend(tmp_path)
    root = f.s.config.root

    def answer():
        for _ in range(100):
            waiting = operator.pending(root)
            if waiting:
                operator.reply(root, waiting[0]['id'], 'cubes are back in place')
                return
            threading.Event().wait(0.05)
    helper = threading.Thread(target=answer)
    helper.start()
    try:
        value = text(f.call('request_operator', {'kind': 'reset', 'message': 'red cube fell off', 'timeout_s': 30}))
        helper.join()
        assert value['status'] == 'answered' and value['reply'] == 'cubes are back in place'
        assert value['robot_status']['robot'] == 'mock' and operator.pending(root) == []
        error = text(f.call('request_operator', {'kind': 'coffee', 'message': 'x'}))['error']
        assert error['stage'] == 'validation'
    finally:
        f.close()


def test_a_held_stop_is_not_uncertain(tmp_path):
    f = frontend(tmp_path)

    def refuse(name, arguments):
        exc = RuntimeError('refused; the arm holds')
        exc.control_certain = True
        exc.robot_state = {'tcp_position': [0.4, 0.0, 0.3]}
        raise exc
    f.backend.call = refuse
    try:
        error = text(f.call('mock_move', {'position': [0.1, 0.2, 0.3]}))['error']
        assert error['control_uncertain'] is False and error['robot_state']['tcp_position'] == [0.4, 0.0, 0.3]
        assert 'holds at robot_state' in error['recovery'] and not f.control_uncertain
    finally:
        f.close()
