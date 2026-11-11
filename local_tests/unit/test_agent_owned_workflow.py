"""New-session authorship and independent software closeout; no hardware/provider calls."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from agent_owned_workflow import install
from harness.codex_cli import closeout
from harness.codex_cli.launcher import codex_config,sandbox_command
from services.controller.config import Configuration


def stopped():
    return {'stop':{'disarmed':True,'stop_acknowledged':{'ok':True},'rest':{'measured_rest':True},'errors':[]},
            'process_exit_codes':{'agent':124,'graph':0,'bridge':0},'passed':False}


@pytest.mark.parametrize('stage',['exploration','function_rehearsal'])
def test_installs_only_new_workspace_preserves_source(tmp_path,stage):
    w=tmp_path/'new';source=tmp_path/'ongoing'
    for p in (w,source):
        (p/'code/task').mkdir(parents=True)
        (p/'code/task/controller.py').write_text('def step(ctx): return {}\n')
        (p/'AGENTS.md').write_text('native motion guards')
    snapshot={str(p):p.read_bytes() for p in source.rglob('*') if p.is_file()}
    provenance=install(w,stage=stage,source=source)
    assert provenance['imported_code_sha256']
    assert (w/'references/before-agent-ownership-AGENTS.md').read_text()=='native motion guards'
    assert all(Path(p).read_bytes()==b for p,b in snapshot.items())
    with pytest.raises(FileExistsError):install(w,stage=stage)


@pytest.mark.parametrize('change',[
    {'error':'KeyboardInterrupt: '}, {'stop':{}}, {'process_exit_codes':{'agent':None}},
    {'stop':{'disarmed':True,'stop_acknowledged':{'ok':True},'rest':{'measured_rest':False}}}])
def test_no_automatic_closeout_on_user_interrupt_or_unknown_shutdown(change):
    r=stopped();r.update(change)
    assert not closeout.eligible(r)[0]


def config_parent(tmp_path,monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT',str(tmp_path))
    p=tmp_path/'session';p.mkdir()
    cfg=Configuration(root=p,task='blocks',robot='mock',agent_provider='codex-relay',session_seconds=60,max_model_calls=1)
    (p/'configuration.json').write_text(json.dumps(cfg.to_dict()))
    (p/'codex-relay.json').write_text(json.dumps({'key_file':'unused','base_url':'https://invalid.example/v1','primary_model':'gpt-6-astra','primary_effort':'high','model_http_proxy':''}))
    (p/'codex-home/sessions').mkdir(parents=True)
    (p/'codex-home/sessions/thread.jsonl').write_text('{"history":"old"}\n')
    (p/'codex-home/config.toml').write_text(codex_config(cfg))
    for d in ('code/task','memory','references'):(p/'agent-workspace'/d).mkdir(parents=True)
    (p/'agent-workspace/code/task/controller.py').write_text('def step(ctx): return {}\n')
    (p/'agent-workspace/memory/PROGRESS.md').write_text('known partial')
    (p/'codex-model-calls.json').write_text('{"calls":1}')
    return cfg,p


def test_prepare_preserves_task_code_and_model_counter_no_live_tools(tmp_path,monkeypatch):
    cfg,p=config_parent(tmp_path,monkeypatch)
    _,root,w,home,report=closeout.prepare(p,stopped())
    assert report['native_resume'] and report['max_model_calls'] is None and report['session_seconds'] is None
    assert (p/'codex-model-calls.json').read_text()=='{"calls":1}'
    assert (root/'task-checkpoint/code/task/controller.py').read_bytes()==(w/'code/task/controller.py').read_bytes()
    assert '[mcp_servers.robot]' not in (home/'config.toml').read_text()
    assert '[mcp_servers.robot]' in (p/'codex-home/config.toml').read_text()
    assert (home/'sessions/thread.jsonl').read_bytes()==(p/'codex-home/sessions/thread.jsonl').read_bytes()
    with pytest.raises(FileExistsError):closeout.prepare(p,stopped())


def test_offline_sandbox_has_model_socket_but_no_robot_stream_or_host_devices(tmp_path,monkeypatch):
    cfg,p=config_parent(tmp_path,monkeypatch)
    monkeypatch.setattr('harness.codex_cli.launcher.sys.platform','linux')
    monkeypatch.setattr('harness.codex_cli.launcher.shutil.which',lambda n:'/usr/bin/'+n)
    cmd=sandbox_command(cfg,None,tmp_path/'model.sock',prompt='closeout',resume=True,offline=True)
    assert '/run/model.sock' in cmd
    for forbidden in ('/run/robot.sock','RAX_MCP_SOCKET','/opt/rax-mcp.py','--share-net','--dev-bind'):
        assert forbidden not in cmd
    assert str(p/'stream') not in cmd
    assert cmd[-3:]==['resume','--last','closeout']


def test_run_after_task_budget_exhaustion_uses_independent_unlimited_counter(tmp_path,monkeypatch):
    _,p=config_parent(tmp_path,monkeypatch);captured={};clock=[0.,5000.]
    monkeypatch.setattr(closeout.time,'monotonic',lambda:clock.pop(0))
    monkeypatch.setattr(closeout,'load_credentials',lambda *a:('offline-test-key','https://invalid.example'))
    monkeypatch.setattr(closeout,'sandbox_command',lambda *a,**k:captured.setdefault('sandbox',k) or [])
    class Relay:
        def __init__(self,*args,**kwargs):captured['relay_root']=args[5];captured['limit']=args[6];self.calls=200
        def start(self):pass
        def close(self):pass
    class Process:
        pid=999
        returncode=0
        def __init__(self,*a,**k):pass
        def wait(self,*a,**k):
            assert not a and not k  # no task timeout forwarded
            s=p/'agent-workspace/output/closeout';s.mkdir(parents=True)
            (s/'SUMMARY.json').write_text('{"status":"candidate_offline_checked"}')
            return 0
        def poll(self):return 0
    monkeypatch.setattr(closeout,'ResponsesRelay',Relay)
    monkeypatch.setattr(closeout.subprocess,'Popen',Process)
    r=closeout.run(p,stopped())
    assert r['status']=='reported' and r['elapsed_s']==5000
    assert captured['limit'] is None and captured['relay_root']==p/'software-closeout'
    assert captured['sandbox']['offline'] and captured['sandbox']['resume']
    assert (p/'codex-model-calls.json').read_text()=='{"calls":1}'
    assert r['model_calls']==200 and not r['task_budget_inherited']


def test_model_start_error_saved_without_hardware_retry(tmp_path,monkeypatch):
    _,p=config_parent(tmp_path,monkeypatch)
    monkeypatch.setattr(closeout,'load_credentials',lambda *a:('offline-test-key','https://invalid.example'))
    monkeypatch.setattr(closeout,'sandbox_command',lambda *a,**k:[])
    class Relay:
        calls=0
        def __init__(self,*a,**k):pass
        def start(self):raise OSError('injected startup error')
        def close(self):pass
    monkeypatch.setattr(closeout,'ResponsesRelay',Relay)
    r=closeout.run(p,stopped())
    assert r['status']=='failed' and not r['robot_connected']
    assert 'injected' in r['error']


def test_real_relay_none_limit_accepts_calls_beyond_old_task_cap(tmp_path,monkeypatch):
    import http.client,socket
    from harness.codex_cli.relay import ResponsesRelay
    (tmp_path/'codex-model-calls.json').write_text('{"calls":999}')
    class Reply:
        status=200;headers={'Content-Type':'application/json'}
        def __enter__(self):return self
        def __exit__(self,*_):pass
        def read1(self,_):
            if getattr(self,'done',False):return b''
            self.done=True;return b'{"status":"completed","output":[]}'
    class Opener:
        def open(self,*a,**k):return Reply()
    monkeypatch.setattr('harness.codex_cli.relay.urllib.request.build_opener',lambda *_:Opener())
    relay=ResponsesRelay(tmp_path/'model.sock','offline-test','https://unused.invalid/v1','gpt-6-astra','high',tmp_path,None)
    relay.start()
    try:
        c=http.client.HTTPConnection('local-only');c.sock=socket.socket(socket.AF_UNIX)
        c.sock.connect(str(tmp_path/'model.sock'))
        c.request('POST','/v1/responses',json.dumps({'model':'gpt-6-astra','input':'synthetic'}),{'Content-Type':'application/json'})
        r=c.getresponse();r.read();assert r.status==200;c.close()
        assert relay.calls==1000
    finally:relay.close()
