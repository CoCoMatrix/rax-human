"""Web lifecycle tests: fake processes only; no CAN, camera or motor access."""
import json
from pathlib import Path
import subprocess
import threading
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError
import pytest
from services.dashboard import piper_scale as m

@pytest.fixture
def manager(tmp_path):
    return m.Manager(tmp_path,python='unused-no-hardware')

def test_import_init_and_get_do_not_launch(manager,monkeypatch):
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:pytest.fail('unexpected process'))
    s=manager.snapshot()
    assert not s['connected'] and not s['prepared'] and s['records']==[]

def test_run_requires_prepare_and_scene(manager):
    with pytest.raises(ValueError,match='确认'): manager.submit('run',{})
    with pytest.raises(ValueError,match='Prepare'): manager.submit('run',{'scene_ready':True,'layout':'new'})
    manager.prepared=True

def test_busy_and_stop_do_not_double_launch(manager):
    class Busy:
        def is_alive(self): return True
    manager.thread=Busy()
    with pytest.raises(RuntimeError,match='重复'): manager.submit('connect',{})
    assert manager.submit('stop',{})['accepted'] and manager.cancel.is_set()

def test_stop_runs_cleanup_and_not_prepare(manager,monkeypatch):
    calls=[];monkeypatch.setattr(manager,'close_owner',lambda:calls.append('stop'))
    manager.submit('stop',{});manager.thread.join(2)
    assert calls==['stop'] and manager.phase=='已停止'

def test_wait_observes_done_even_after_process_exit(manager):
    class Exited:
        def poll(self): return 0
    manager.process=Exited()
    assert manager.wait(lambda:{'ok':True},1,'done')=={'ok':True}

def test_cancel_wait_never_continues(manager):
    manager.cancel.set()
    with pytest.raises(m.Cancelled): manager.wait(lambda:True,1,'done')

def test_finalize_partial_record_persisted(manager):
    out=manager.fresh_output('main');out.mkdir(parents=True)
    m.save(out/'REHEARSAL-DONE.json',{'ok':False,'steps':[{'stage':'grasp','stage_elapsed_s':2.1,'ok':False}],
        'elapsed_s':4.5,'auxiliary_visual_calls':1,'main_agent_calls':0,'failed_step':'grasp'})
    manager.active_record={'id':out.name,'output':str(out),'started':time.time()-10,'outcome':'interrupted'}
    manager.finish_record()
    row=manager.records()[0]
    assert row['outcome']=='interrupted' and row['total_wall_s']>=10
    assert row['stages'][0]['stage_elapsed_s']==2.1 and row['auxiliary_visual_calls']==1
    assert m.Manager(manager.root).records()==manager.records()

def test_failure_does_not_retry_or_become_success(manager,monkeypatch):
    calls=[]
    monkeypatch.setattr(manager,'verify_prepared',lambda:None)
    monkeypatch.setattr(manager,'close_owner',lambda:calls.append('stop'))
    def fail(_): calls.append('run'); raise RuntimeError('uncertain execution')
    monkeypatch.setattr(manager,'run_once',fail)
    manager._job('run',{'layout':'fresh'})
    assert calls==['stop','run','stop'] and manager.phase=='需要处理' and not manager.prepared

def test_read_current_report_from_session_only(manager):
    out=manager.fresh_output('main'); d=out/'rax-session/agent-workspace/output/rehearsal/merged-a';d.mkdir(parents=True)
    m.save(d/'report.json',{'active_step':'trial_lift','elapsed_s':20})
    assert m.controller_report(out)['active_step']=='trial_lift'
    assert m.controller_report(manager.fresh_output('main'))=={}

@pytest.fixture
def http(manager):
    server,token=m.make_server(manager,0,token='test-token')
    t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
    yield 'http://127.0.0.1:'+str(server.server_port),manager
    server.shutdown();server.server_close()

def request(base,path,body=None,token='test-token',ctype='application/json'):
    req=Request(base+path,data=None if body is None else json.dumps(body).encode(),headers={'X-Operator-Token':token,'Content-Type':ctype})
    try:
        with urlopen(req) as r:return r.status,r.read()
    except HTTPError as e:return e.code,e.read()

def test_http_auth_and_json_mutation_guard(http):
    base,manager=http
    assert request(base,'/api/status',token='bad')[0]==401
    assert request(base,'/api/action/prepare',{},token='bad')[0]==401
    assert request(base,'/api/action/prepare',{},ctype='application/x-www-form-urlencoded')[0]==400
    assert request(base,'/api/action/anything',{})[0]==400
    assert manager.thread is None

def test_http_readonly_pages_and_camera_guard(http):
    base,manager=http
    assert request(base,'/')[0]==200
    assert request(base,'/scale.js')[0]==200
    assert request(base,'/api/status')[0]==200
    assert request(base,'/api/camera/head.jpg')[0]==409
    assert manager.thread is None
    with urlopen(Request(base+'/',headers={'X-Operator-Token':'test-token'})) as response:
        assert "img-src 'self' blob:" in response.headers['Content-Security-Policy']

def test_csv_one_row_all_timings_and_formula_escape(http):
    base,manager=http
    m.save(manager.data/'web-main-test.json',{'id':'web-main-test','layout':'=1+1','version':'rc9','commit':'abc','outcome':'success',
        'elapsed_s':58.66,'total_wall_s':83.9,'auxiliary_visual_calls':0,'main_agent_calls':0,
        'stages':[{'stage':'grasp','stage_elapsed_s':5.49,'ok':True}]})
    code,body=request(base,'/api/runs.csv');s=body.decode('utf-8-sig')
    assert code==200 and 'stage_timings_json' in s and '5.49' in s and '83.9' in s and "'=1+1" in s

def test_camera_names_cannot_escape(manager):
    with pytest.raises(ValueError):manager.camera('../../private/owner-token')

def test_prepare_checks_actual_jaws_and_joints():
    profile={'initial_pose':{'left_arm':[0]*6,'right_arm':[0]*6,'left_gripper':.0003,'right_gripper':.0022}}
    state={'robot_status':{'lifecycle_state':'prepared'},'robot_state':{'values':[0]*6+[.0003]+[0]*6+[.002]}}
    assert m.at_prepare(state,profile)
    state['robot_state']['values'][6]=.01
    assert not m.at_prepare(state,profile)
    state['robot_state']['values'][6]=.0003;state['robot_state']['values'][3]=.012
    assert not m.at_prepare(state,profile)

def test_failed_prepare_gate_never_starts_main(manager,monkeypatch):
    def refuse():raise RuntimeError('state moved')
    monkeypatch.setattr(manager,'verify_prepared',refuse)
    monkeypatch.setattr(manager,'close_owner',lambda:None)
    monkeypatch.setattr(manager,'run_once',lambda _:pytest.fail('unsafe start'))
    manager._job('run',{})
    assert 'state moved' in manager.error and not manager.prepared

@pytest.mark.parametrize('stop_ok',[True,False])
def test_one_run_uses_native_launchers_records_metrics_and_stop(manager,monkeypatch,stop_ok):
    manager.candidate.mkdir(parents=True)
    m.save(manager.candidate/'WORKSPACE.json',{'candidate_version':'rc9','code_commit':'abc'})
    monkeypatch.setattr(subprocess,'check_output',lambda cmd,**kw:'abc\n' if 'rev-parse' in cmd else '')
    calls=[]
    class Process:
        def poll(self):return 0
        def wait(self,timeout):return 0
    def launch(script,out,*args):
        calls.append([str(script),*args]);out.mkdir(parents=True);manager.out=out;manager.process=Process()
        m.save(out/'REHEARSAL-READY.json',{'main_planning_model_started':False})
        m.save(out/'REHEARSAL-DONE.json',{'ok':True,'elapsed_s':12,'steps':[{'stage':'grasp','ok':True,'stage_elapsed_s':2}],
            'main_agent_calls':0,'auxiliary_visual_calls':0,'metrics_complete':True})
        m.save(out/'STOP-RESULT.json',{'disarmed':stop_ok,'rest':{'measured_rest':stop_ok},'errors':[]})
        return manager.process
    monkeypatch.setattr(manager,'launch',launch)
    monkeypatch.setattr(subprocess,'Popen',lambda cmd,**kw:calls.append(cmd) or Process())
    manager.run_once({})
    row=manager.records()[0]
    assert len(calls)==2 and '--rehearsal-agent' not in calls[0]
    assert '--rehearsal-candidate' in calls[0] and 'run_no_agent.py' in calls[1][1]
    assert row['outcome']==('success' if stop_ok else 'failed')
    assert row['elapsed_s']==12 and row['stages'][0]['stage_elapsed_s']==2 and row['auxiliary_visual_calls']==0
    assert row['total_wall_s']>=0 and manager.process is None and not manager.prepared

def test_layout_from_this_runs_initial_vision_only(manager):
    out=manager.fresh_output('main');p=out/'rax-session/agent-workspace/output/rehearsal';p.mkdir(parents=True)
    m.save(p/'approach-target.json',{'valid':False,'phase':'grasp_closed','frame':'left_base_link',
        'source_frame':'frame-one','created_ns':123,'geometry':{'cup_center_estimate':[.25,-.1,.1],'coaster_center':[.24,-.32,.028]}})
    manager.active_record={'id':out.name,'output':str(out),'started':time.time(),'outcome':'success','layout':'ignore manual description'}
    manager.finish_record();r=manager.records()[0]
    assert r['initial_localization']['source_frame']=='frame-one'
    assert r['initial_localization']['historical_measurement_only'] is True
    assert '250.0' in r['layout'] and 'manual' not in r['layout']
    assert m.initial_layout(manager.fresh_output('main')) is None
