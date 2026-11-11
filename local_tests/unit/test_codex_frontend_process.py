"""Process-level cancellation/resource checks; mock-only, no network or hardware."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import pytest

from harness.codex_cli.launcher import prepare, stop_process, wait_frontend
from services.controller.config import Configuration
from services.controller.session import Session

BOOTSTRAP = r'''
import sys,time
from pathlib import Path
from harness.codex_cli import frontend as f
config_path,sock,mode=sys.argv[1:]
root=Path(config_path).parent
class Backend:
    mutating=set()
    def tools(self):
        return [{'name':'blocked_ik','description':'Mock blocking IK','inputSchema':{'type':'object','properties':{}}}]
    def status(self):return {'robot':'mock'}
    def call(self,name,arguments):
        (root/'entered').write_text('ik')
        time.sleep(60)
        return {'ok':True},[]
    def close(self):
        (root/'backend-closed').write_text('closed')
        if mode=='close_hang':time.sleep(60)
class Vision:
    def __init__(self,*args,**kwargs):pass
    def call(self,arguments):
        (root/'entered').write_text('vision')
        time.sleep(60)
        return {'text':'unused','usage':{}}
def backend(config):
    if mode=='startup_fail':raise RuntimeError('Expected mock startup failure')
    return Backend()
f.make_backend=backend
f.VisionRouter=Vision
sys.argv=['frontend','--config',config_path,'--socket',sock]
f.main()
'''


@pytest.fixture
def configured(tmp_path, monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT', str(tmp_path))
    config = Configuration(root=tmp_path/'session', task='process_test', robot='mock',
                           agent_provider='codex-relay', agent_effort='high')
    key = tmp_path/'key.py'; key.write_text('sk-private-test')
    prepare(config, key, 'https://relay.example/v1')
    return config


def start(config, mode, sock):
    return subprocess.Popen([sys.executable, '-c', BOOTSTRAP,
                             str(config.root/'configuration.json'), str(sock), mode],
                            cwd=Path(__file__).resolve().parents[2], env=os.environ.copy(),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)


def eventually(predicate, timeout=3):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():return
        time.sleep(.02)
    assert predicate(), 'Expected process state was not reached'


@pytest.mark.parametrize('kind', ['operator', 'vision', 'ik'])
def test_stop_interrupts_main_thread_and_closes_resources(configured, tmp_path, kind):
    c = configured; sock = tmp_path/'service.sock'; p = start(c, kind, sock)
    try:
        wait_frontend(p, sock, c.root, timeout=3)
        with socket.socket(socket.AF_UNIX) as client:
            client.connect(str(sock))
            name,args = {
                'operator': ('request_operator', {'kind':'help','message':'Mock wait','timeout_s':300}),
                'vision': ('codex_vision_generate', {'prompt':'Mock blocked vision'}),
                'ik': ('blocked_ik', {})}[kind]
            client.sendall((json.dumps({'jsonrpc':'2.0','id':1,'method':'tools/call',
                                       'params':{'name':name,'arguments':args}})+'\n').encode())
            eventually(lambda: bool(list((c.root/'operator/requests').glob('*.json'))) if kind=='operator'
                       else (c.root/'entered').exists())
            result = stop_process(p, grace=2)
        report = json.loads((c.root/'frontend-exit.json').read_text())
        assert result['exit_code'] == 0 and not result['forced_kill']
        assert report['stop_requested'] and report['shutdown_completed']
        assert report['backend_closed'] and report['session_closed']
        assert report['physical_stop_confirmed'] is False
        assert not sock.exists()
        # The previous process relinquished the actual owner lock.
        owner = Session(c); owner.close()
    finally:
        stop_process(p, grace=.2)


@pytest.mark.parametrize('kind', ['startup_fail', 'bind_fail'])
def test_startup_failure_releases_acquired_resources(configured, tmp_path, kind):
    c=configured; sock=tmp_path/'service.sock'
    if kind=='bind_fail':sock.mkdir()
    p=start(c, kind, sock)
    try:
        with pytest.raises(RuntimeError, match='exited during startup'):
            wait_frontend(p, sock, c.root, timeout=3)
        assert p.wait(timeout=2) != 0
        report=json.loads((c.root/'frontend-exit.json').read_text())
        assert report['session_closed']
        assert report['backend_closed'] == (kind=='bind_fail')
        owner=Session(c);owner.close()
    finally:
        stop_process(p,grace=.2)


def test_cleanup_timeout_is_unknown_not_physical_stop(configured,tmp_path):
    c=configured;sock=tmp_path/'service.sock';p=start(c,'close_hang',sock)
    try:
        wait_frontend(p,sock,c.root,timeout=3)
        result=stop_process(p,grace=.2)
        assert result['stop_requested'] and result['forced_kill']
        assert result['physical_stop_confirmed'] is False
        assert not (c.root/'frontend-exit.json').exists()
        owner=Session(c);owner.close()
    finally:
        stop_process(p,grace=.2)


def test_launcher_relay_failure_still_stops_host_child(configured,monkeypatch):
    from harness.codex_cli import launcher
    monkeypatch.setattr(launcher,'sandbox_command',lambda *a,**k:[sys.executable,'-c','pass'])
    def fail(*a,**k):raise RuntimeError('Expected relay startup failure')
    monkeypatch.setattr(launcher,'ResponsesRelay',fail)
    with pytest.raises(RuntimeError,match='Expected relay startup failure'):
        launcher.run(configured,prompt='No model call')
    last=json.loads((configured.root/'codex-lifecycle.json').read_text())['launches'][-1]
    assert last['frontend_stop']['exit_code']==0
    assert last['frontend_shutdown']['shutdown_completed']
    assert last['physical_stop_confirmed'] is False
    owner=Session(configured);owner.close()
