import importlib,sys
from pathlib import Path
import pytest
D=Path(__file__).resolve().parents[1]
@pytest.fixture
def controller(tmp_path,monkeypatch):
    monkeypatch.chdir(tmp_path);monkeypatch.syspath_prepend(str(D/'code/blue_cup'))
    m=importlib.import_module('controller');out=tmp_path/'output';out.mkdir(exist_ok=True);monkeypatch.setattr(m,'OUT',out);return m

def fake(m,monkeypatch,fail=None):
    calls=[]
    def fn(name):
        def invoke(ctx,*args):
            calls.append(name)
            if name=='trial_lift':assert args[0]=={'dz':.040,'execution_mode':'task_scheduled'}
            return dict(ok=name!=fail,reason='synthetic',report_path='fixture.json',previous_report='fixture.json',visual_calls=0)
        return invoke
    for n in ('prepare_and_enter','grasp','trial_lift','localize_coaster','transfer_and_lower','release','retreat'):monkeypatch.setattr(m,n,fn(n))
    return m.main(object()),calls

def test_normal_order_retains_contact_boundaries(controller,monkeypatch):
    r,calls=fake(controller,monkeypatch)
    assert r['ok'] and r['task_success'] and r['main_agent_calls']==0
    assert calls==['prepare_and_enter','grasp','trial_lift','localize_coaster','transfer_and_lower','release','retreat']
    assert not r['held_object_possible']

@pytest.mark.parametrize('failure',['grasp','trial_lift','transfer_and_lower'])
def test_failure_never_replays_or_auto_releases(controller,monkeypatch,failure):
    r,calls=fake(controller,monkeypatch,failure)
    assert not r['ok'] and r['held_object_possible']
    assert calls[-1]==failure and calls.count(failure)==1 and 'release' not in calls
