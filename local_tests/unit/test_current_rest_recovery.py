"""Real policy recovery logic, fake passive transport only; no SDK or sockets."""
import importlib.util
from pathlib import Path
import sys
import types
from types import SimpleNamespace
import pytest


@pytest.fixture
def Policy(monkeypatch):
    name='_offline_policy_recovery'
    pkg=types.ModuleType(name);pkg.__path__=[str(Path(__file__).resolve().parents[2]/'services/piper')];monkeypatch.setitem(sys.modules,name,pkg)
    def require(condition,reason):
        if not condition:raise ValueError(reason)
    for suffix,attrs in [('controlled_transport',{'require':require}),('owner',{'SERIALS':{},'ORDER':[]}),('dispatch',{'action_schema':lambda:None})]:
        mod=types.ModuleType(name+'.'+suffix);mod.__dict__.update(attrs);monkeypatch.setitem(sys.modules,mod.__name__,mod)
    spec=importlib.util.spec_from_file_location(name+'.controlled_policy',Path(__file__).resolve().parents[2]/'services/piper/controlled_policy.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod.ControlledPolicy


def make(Policy,phase='DONE'):
    clock=[10_000_000_000];mono=[1_000_000_000]
    t=SimpleNamespace(state=phase,general_joint_motion=True,binding=('session',3),result={'final14':[0.]*14},clock=lambda:clock[0],stops=[])
    def stop(reason):t.state='FAULT';t.stops.append(reason)
    t.stop=stop
    p=Policy(None,t,task='test',receipt_tail=None,operator_arm=None,monotonic_ns=lambda:mono[0])
    p.latest={'q':[0.]*14,'generation':3}
    return p,t,clock,mono


def sample(p,c,m,values,**fields):
    c[0]+=50_000_000;m[0]+=50_000_000
    p.feedback={'q':values[:],'timestamp_ns':c[0],'generation':3,'lifecycle_state':'running',**fields}
    p._recovery_feedback()


def test_old_arrival_does_not_override_recent_stable_encoder_pose(Policy):
    p,t,c,m=make(Policy);q=[0.]*14;q[3]=.0015184364492351
    for _ in range(9):sample(p,c,m,q)
    p._begin_timing_recovery('host wall-clock discontinuity: 941407369 ns')
    assert p.timing_recovery['anchor']==q
    assert p.timing_recovery['anchor_source']=='recent_stable_passive_feedback'
    assert p.latest is None
    for _ in range(9):sample(p,c,m,q)
    assert p._recovery_ready({'q':q,'generation':3})
    assert t.state=='DONE' and not t.stops


@pytest.mark.parametrize('field,value',[('generation',4),('lifecycle_state','stopped'),('q',[.011]+[0.]*13)])
def test_new_drift_or_identity_change_still_rejected(Policy,field,value):
    p,t,c,m=make(Policy)
    for _ in range(9):sample(p,c,m,[0.]*14)
    p._begin_timing_recovery('clock')
    with pytest.raises(ValueError):sample(p,c,m,[0.]*14,**{field:value})
    assert p.latest is None and p.timing_recovery is not None


@pytest.mark.parametrize('phase',['STAGED','RUNNING','SETTLING','FAULT'])
def test_active_or_faulted_state_never_recovers(Policy,phase):
    p,t,c,m=make(Policy,phase)
    p._begin_timing_recovery('clock')
    assert t.state=='FAULT' and t.stops and p.timing_recovery is None


def test_stale_history_cannot_replace_old_anchor(Policy):
    p,t,c,m=make(Policy);q=[.0115]+[0.]*13
    for _ in range(9):sample(p,c,m,q)
    m[0]+=1_000_000_000
    p._begin_timing_recovery('clock')
    assert p.timing_recovery['anchor']==[0.]*14
    with pytest.raises(ValueError):sample(p,c,m,q)


def test_current_drift_does_not_accumulate_into_a_moving_anchor(Policy):
    p,t,c,m=make(Policy)
    for _ in range(9):sample(p,c,m,[0.]*14)
    p._begin_timing_recovery('clock')
    sample(p,c,m,[.005]+[0.]*13)
    with pytest.raises(ValueError):sample(p,c,m,[.011]+[0.]*13)


def test_one_frame_and_timeout_never_resume(Policy):
    p,t,c,m=make(Policy);p._begin_timing_recovery('clock')
    sample(p,c,m,[0.]*14)
    assert not p._recovery_ready({'q':[0.]*14,'generation':3})
    m[0]+=3_100_000_000;p._recovery_timeout()
    assert t.state=='FAULT'
