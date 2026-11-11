"""Recorded delayed convergence and post-action observation, no devices opened."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from services.piper.backend import PiperBackend


@pytest.mark.parametrize('error,complete',[(.005,True),(.01,True),(.011,False)])
def test_authorized_arrival_tolerance_keeps_full_rest_window(error,complete):
    import threading
    from services.piper.installed_settling import transport_class

    class Base:
        def __init__(self,*,joint_tolerance_rad=.003):
            assert .001<=joint_tolerance_rad<=.003  # Installed legacy constructor.
            self.joint_tolerance=joint_tolerance_rad
            self.lock=threading.RLock();self.state='SETTLING';self.result=None
            self.plan={'anchor14':[0.]*14,'target14':[0.]*14}
            self.binding=('offline',1);self.receipts=[{}]
        def _validate_obs(self,o):pass
        def _guards(self):pass
        def _lease(self,state):pass
        def stop(self,reason):raise AssertionError(reason)

    clock=[1_000_000_000]
    motion=transport_class(Base)(joint_tolerance_rad=.003,monotonic_ns=lambda:clock[0])
    assert motion.joint_tolerance==.01
    q=[0.]*14;q[4]=error
    for i in range(12):
        clock[0]=1_000_000_000+i*100_000_000
        motion.observe_feedback({'q':q,'timestamp_ns':clock[0]})
        if i<3:assert motion.state!='DONE'
        if motion.state=='DONE':break
    assert (motion.state=='DONE') is complete
    if complete:
        assert motion.result['joint_tolerance_rad']==.01
        assert motion.result['settling']['joint_range_limit_rad']==.01
        assert motion.result['settling']['jaw_range_limit_m']==.0003


def test_post_motion_accepts_authorized_tolerance_but_rejects_excess(tmp_path,monkeypatch):
    b,m,*_=rig(tmp_path,monkeypatch,bad='none')
    m['joint_tolerance_rad']=.01;m['target14'][4]=-.005
    b._check_post_motion_state(m)
    m['target14'][4]=-.011
    with pytest.raises(RuntimeError,match='outside'):b._check_post_motion_state(m)
    m['target14'][4]=0.;m['joint_tolerance_rad']=.011
    with pytest.raises(RuntimeError,match='tolerance invalid'):b._check_post_motion_state(m)


def test_recorded_r8_window_uses_new_policy_without_claiming_subthreshold_drift_detection():
    from services.piper.installed_settling import RestWindow
    rows=json.loads((Path(__file__).parents[1]/'fixtures/r8-j4-state-window.json').read_text())
    w=RestWindow();completed=[]
    for row in rows:
        if w.add(row['q'],row['source_timestamp_ns'],row['t_ms']*1_000_000):
            completed.append(row['t_ms'])
    assert completed and min(completed)-rows[0]["t_ms"]>=300
    assert w.evidence()["max_joint_range_rad"]<=.01


def test_slow_accumulated_drift_and_duplicate_samples_are_not_rest():
    from services.piper.installed_settling import RestWindow
    w=RestWindow()
    for i in range(50):
        assert not w.add([i*.002]+[0.]*13,1_000_000_000+i*50_000_000,1_000_000_000+i*50_000_000)
    w=RestWindow()
    for i in range(50):
        assert not w.add([0.]*14,1_000_000_000,1_000_000_000+i*50_000_000)


def rig(tmp_path,monkeypatch,*,bad='once'):
    clock=[10_000_000_000];calls=[];q=[0.]*14;q[3]=1.497534107
    gen=[1];repeat_stamp=[False]
    def state():
        return {'robot_state':{'values':q.copy(),'feedback_metadata':{'source_timestamp_ns':10_000_000_000 if repeat_stamp[0] else clock[0],'maximum_age_ms':0}},
                'robot_status':{'generation':gen[0],'lifecycle_state':'running'}}
    def sleep(s):clock[0]+=int(s*1e9)
    monkeypatch.setattr('services.piper.backend.time.monotonic_ns',lambda:clock[0])
    monkeypatch.setattr('services.piper.backend.time.time_ns',lambda:clock[0])
    monkeypatch.setattr('services.piper.backend.time.sleep',sleep)
    def capture(*args,**kwargs):
        calls.append(kwargs)
        if bad=='always':q[3]+=.011
        if bad=='once' and len(calls)==1:q[3]+=.0006
        if bad=='generation':gen[0]+=1
        if bad=='out_of_target':q[3]+=.01
        clock[0]+=100_000_000
        return {'frame_id':str(len(calls)),'cameras':{}},[]
    stops=[]
    b=PiperBackend(SimpleNamespace(root=tmp_path,backend_options={},observation_profile='rgb-only'),
                   owner=SimpleNamespace(state=state),cameras=SimpleNamespace(capture=capture))
    b.bridge=SimpleNamespace(cancel=lambda:stops.append(True))
    motion={'executed':True,'measured_arrival':True,'measured_settled':True,'final14':q.copy(),
            'target14':q.copy(),'joint_tolerance_rad':.003,'owner_generation':1,
            'settling':{'window_s':1.0}}
    return b,motion,calls,stops,repeat_stamp


def test_authorized_small_joint_capture_drift_remains_bound_without_extra_capture(tmp_path,monkeypatch):
    b,m,calls,stops,_=rig(tmp_path,monkeypatch)
    result,images=b._observe_after(m)
    assert result['observation']['frame_id']=='1'
    assert result['observation']['state_binding']['rest_consistent']
    assert result['observation']['discarded_frame_ids']==[]
    assert result['observation']['state_binding']['joint_tolerance_rad']==.01
    assert result['observation']['state_binding']['jaw_tolerance_m']==.0005
    assert len(calls)==1
    assert not stops


@pytest.mark.parametrize('bad',['always','generation','out_of_target'])
def test_unrecoverable_state_fault_requests_stop_not_image_recovery(tmp_path,monkeypatch,bad):
    b,m,calls,stops,_=rig(tmp_path,monkeypatch,bad=bad)
    with pytest.raises(RuntimeError):b._observe_after(m)
    assert stops and len(calls)<=2


def test_repeated_feedback_cannot_earn_stability(tmp_path,monkeypatch):
    b,m,calls,stops,repeat=rig(tmp_path,monkeypatch,bad='none');repeat[0]=True
    with pytest.raises(RuntimeError):b._observe_after(m)
    assert stops and not calls


def test_readonly_observe_fault_retains_uncertainty_in_frontend(tmp_path,monkeypatch):
    from test_piper_action_observation import rig as frontend_rig, value
    frontend,bridge,_=frontend_rig(tmp_path)
    from services.piper.backend import PostMotionStateError
    def fault(*args,**kwargs):raise PostMotionStateError('feedback changed')
    monkeypatch.setattr(frontend.backend,'_observe',fault)
    try:
        reply=frontend.call('piper_observe',{})
        assert value(reply)['error']['control_uncertain'] is True
        assert frontend.control_uncertain
    finally:frontend.close()


def test_feedback_gap_resets_full_window_and_reversal_rejected():
    from services.piper.installed_settling import RestWindow
    w=RestWindow()
    for i in range(3):assert not w.add([0.]*14,1_000_000_000+i*100_000_000,1_000_000_000+i*100_000_000)
    assert not w.add([0.]*14,3_000_000_000,3_000_000_000)
    with pytest.raises(ValueError,match='reversed'):w.add([0.]*14,2_000_000_000,3_100_000_000)


@pytest.mark.parametrize('settled',[False,True])
def test_bridge_requires_measured_settling_receipt(tmp_path,monkeypatch,settled):
    from test_piper_bridge_lifecycle import bridge_at
    b=bridge_at(tmp_path);b.instance='offline'
    def status():
        reqs=list((b.root/'requests').glob('*.json'))
        if not reqs:return {'phase':'IDLE','request_id':None}
        req=json.loads(reqs[0].read_text())
        result={'executed':True,'measured_arrival':True,'sdk_completed_steps':10,'final14':[0.]*14}
        if settled:result.update(measured_settled=True,target14=[0.]*14,owner_generation=1,
                                joint_tolerance_rad=.003,settling={'window_s':.3,'samples':4,'max_range':0.})
        (b.root/'results'/reqs[0].name).write_text(json.dumps({'id':req['id'],'instance':'offline','ok':True,'result':result}))
        return {'phase':'DONE','request_id':None}
    monkeypatch.setattr(b,'status',status);monkeypatch.setattr(b,'armed',lambda:None)
    try:
        if settled:assert b.move([0.]*14)['measured_settled']
        else:
            with pytest.raises(RuntimeError,match='settling'):b.move([0.]*14)
            assert (b.root/'cancel.json').exists()
    finally:b.close()


def test_state_fault_latches_backend_even_if_cancel_delivery_fails(tmp_path,monkeypatch):
    b,m,_,_,_=rig(tmp_path,monkeypatch,bad='generation')
    def unavailable():raise OSError('bridge unavailable')
    b.bridge.cancel=unavailable
    with pytest.raises(RuntimeError):b._observe_after(m)
    from services.piper.bridge import RejectedBeforeDispatch
    with pytest.raises(RejectedBeforeDispatch,match='latched'):
        b.validate('piper_move_joints',{'joints':[0.]*6})


@pytest.mark.parametrize('index,delta,bound',[(3,.01,True),(3,.01001,False),(6,.0005,True),(6,.000501,False),(13,.01,False)])
def test_capture_binding_uses_joint_radians_and_jaw_metres(tmp_path,monkeypatch,index,delta,bound):
    b,m,calls,stops,_=rig(tmp_path,monkeypatch,bad='none')
    original=b.owner.state;count=[0]
    def state():
        s=original();count[0]+=1
        # Separate before/after states; no motion or controller is used.
        s['robot_state']['values']=[0.]*14
        if count[0]>1:s['robot_state']['values'][index]=delta
        return s
    b.owner.state=state
    obs,_=b._capture_observation({},motion=None)
    assert obs['state_binding']['rest_consistent'] is bound
    assert obs['state_binding']['joint_tolerance_rad']==.01
    assert obs['state_binding']['jaw_tolerance_m']==.0005
    assert not stops


def test_selected_window_requires_elapsed_time_and_fresh_samples():
    from services.piper.installed_settling import RestWindow, SETTLING_WINDOW_S
    assert SETTLING_WINDOW_S==.3
    w=RestWindow()
    for t in (0,100_000_000,299_999_999):
        assert not w.add([0.]*14,t,t)
    assert w.add([0.]*14,300_000_000,300_000_000)
    assert w.evidence()['window_s']==.3
