"""Offline real adapter loader; no owner, SDK, CAN or camera opened."""
import sys,json,math,copy
from pathlib import Path
import numpy as np
import pytest
ROOT=Path(__file__).parent/'runtime'
sys.path.insert(0,str(ROOT/'scripts'))
from piper_motion_bridge import load_bridge
m=load_bridge(Path('/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025'),'faster')

class Geometry:
    def _vector(self,q):
        a=np.asarray(q,float)
        if a.shape!=(14,) or not np.isfinite(a).all():raise ValueError('shape/finite')
        return a
    def status(self):return {'urdf_sha256':'test'}
class Collision:
    def path(self,points,obstacles=()):return {'cad_clear':True,'whole_path_checked':True}

@pytest.fixture
def rig(tmp_path):
    clock=[1_000_000_000];arm={'armed':True};stops=[]
    t=m.MotionTransport(Geometry(),Collision(),arm_check=lambda:arm,stop_client=lambda:stops.append(True) or {'stop_requested':True},lease_file=tmp_path/'lease.json',max_displacement_rad=math.radians(60),joint_tolerance_rad=.01,allow_left_gripper=True,clock_ns=lambda:clock[0],monotonic_ns=lambda:clock[0],max_velocity_rad_s=math.radians(20),max_acceleration_rad_s2=math.radians(40),terminal_dwell_s=.2,tracking_velocity_ratio=.85)
    t.check_anchor_workspace=lambda a,b:None
    t.check_wire_workspace=lambda points:None
    return t,clock,arm,stops

def plan(t):
    return t.preview_waypoints([0.]*14,[[.15,0,0,0,0,0],[-.05,.08,0,0,0,0]], [{'name':'table'}])
def obs(q,stamp):return {'q':list(q),'timestamp_ns':stamp,'policy_session_id':'test','generation':1}
def receipt(t,i,stamp):
    return dict(event='sdk_dispatch_completed',review_release_id=t.release['release_id'],teacher_request_id=t.plan['sha256'],generation=1,model_chunk_step_index=i,hardware_dispatch_confirmed=True,applied14=t.plan['targets14'][i],recorded_ns=stamp)
def start(t,clock,p=None):
    p=p or plan(t);t.stage(p,obs(p['anchor14'],clock[0]));assert t.tick(obs(p['anchor14'],clock[0]));return p

def test_turning_path_completes_with_measured_settling(rig):
    t,c,arm,stops=rig;p=start(t,c)
    assert max(q[0] for q in p['targets14'])>max(p['anchor14'][0],p['target14'][0])+.015
    for i,q in enumerate(p['targets14']):
        c[0]+=33_333_334;t.receipt(receipt(t,i,c[0]));t.observe_feedback(obs(q,c[0]));assert t.state!='FAULT',t.stop_report
    for _ in range(4):c[0]+=100_000_000;t.observe_feedback(obs(p['target14'],c[0]))
    assert t.state=='DONE' and t.result['measured_settled'] and not stops
    assert t.result['trajectory_progress']['dispatched_samples']==len(p['targets14'])

@pytest.mark.parametrize('fault',['future_point','offpath','wrong_generation','disarm','timeout','cancel','wrong_receipt','modified_plan','reversed_receipt'])
def test_faults_stop_and_never_resume(rig,fault):
    t,c,arm,stops=rig;p=plan(t)
    if fault=='modified_plan':
        p['targets14'][0][0]+=.1
        with pytest.raises(ValueError):t.stage(p,obs(p['anchor14'],c[0]))
        assert not stops;return
    start(t,c,p)
    if fault in ('future_point','offpath','wrong_generation'):
        o=obs(p['anchor14'],c[0]);o['q'][0]=.15 if fault=='future_point' else 0
        if fault=='offpath':o['q'][2]=.02
        if fault=='wrong_generation':o['generation']=2
        t.observe_feedback(o)
    elif fault=='disarm':arm['armed']=False;t.watchdog()
    elif fault=='timeout':c[0]+=30_000_000_000;t.watchdog()
    elif fault=='cancel':t.stop('operator stop')
    elif fault=='wrong_receipt':t.receipt(receipt(t,1,c[0]))
    else:
        t.receipt(receipt(t,0,c[0]));t.receipt(receipt(t,1,c[0]-1))
    assert t.state=='FAULT' and stops
    with pytest.raises(ValueError):t.stage(plan(t),obs(p['anchor14'],c[0]))
    assert t.tick(obs(p['anchor14'],c[0])) is None

@pytest.mark.parametrize('error',[.005,.011])
def test_final_arrival_not_same_as_path_corridor(rig,error):
    t,c,_,_=rig;p=start(t,c)
    for i,q in enumerate(p['targets14']):
        c[0]+=33_333_334;t.receipt(receipt(t,i,c[0]));t.observe_feedback(obs(q,c[0]))
    q=p['target14'].copy();q[0]+=error
    for _ in range(5):c[0]+=100_000_000;t.observe_feedback(obs(q,c[0]))
    assert (t.state=='DONE')==(error<=.01)

def test_missing_progress_receipts_fault(rig):
    t,c,_,stops=rig;p=start(t,c);t.receipt(receipt(t,0,c[0]));c[0]+=400_000_000;t.observe_feedback(obs(p['targets14'][0],c[0]));assert t.state=='FAULT' and stops

def test_fixed_jaws_and_right_side(rig):
    t,c,_,_=rig;p=plan(t);a=np.asarray(p['targets14']);assert np.all(a[:,6:]==0)

def test_native_contract_accepts_wire_without_hardware(rig):
    sys.path.insert(0,'/home/ubuntu/code_junqi/GPT-as-Policy')
    from real_robot.piper.contracts import PiperContract,vector_schema
    owner=ROOT/'owner-runtime'
    contract=PiperContract.from_runtime(owner,horizon=600,action_prefix=600,trajectory_mode=True)
    p=plan(rig[0]);wire=np.asarray(p['targets14'],dtype=np.float32)
    result=contract.validate_candidate(wire,schema=vector_schema(action=True),frequency_hz=30,measured_state14=np.asarray(p['anchor14']),prefix_steps=len(wire))
    assert result is not None

def test_native_owner_gate_accepts_all_samples_and_rejects_replay(rig):
    import importlib.util
    path=ROOT/'owner-runtime/real_world/robots/piper_x_dual/review_release.py'
    spec=importlib.util.spec_from_file_location('offline_owner_gate',path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    t,c,_,_=rig;p=start(t,c);gate=mod.BoundedReviewGate(allow_trajectory=True)
    measured=p['anchor14'];release=t.release
    for i,q in enumerate(p['targets14']):
        gate.reserve(release,generation=1,measured=measured,requested=q,now_ns=c[0]+int(i*1e9/30),step_index=i)
        measured=q
    with pytest.raises(ValueError,match='consumed'):
        gate.reserve(release,generation=1,measured=p['anchor14'],requested=p['targets14'][0],now_ns=c[0],step_index=0)


def test_collision_or_workspace_rejection_has_no_stage(rig):
    t,_,_,stops=rig
    t.collision.path=lambda *a,**k:dict(cad_clear=False)
    with pytest.raises(ValueError):plan(t)
    assert t.state=='IDLE' and not stops
    t.collision=Collision()
    def bad(*a):raise ValueError('workspace')
    t.check_anchor_workspace=bad
    with pytest.raises(ValueError,match='workspace'):plan(t)
    assert t.state=='IDLE' and not stops


def test_blended_knots_keep_velocity_and_recheck_actual_curve(rig):
    t,_,_,_=rig;checked=[]
    t.collision.path=lambda pts,obstacles=(): checked.append(pts) or dict(cad_clear=True,whole_path_checked=True)
    p=t.preview_waypoints([0.]*14,[[.1,.02,0,0,0,0],[.2,.04,0,0,0,0],[.3,0,0,0,0,0]],[{'name':'table'}],blend_indices=[0,1])
    assert len(checked[-1])==len(p['targets14'])+1
    assert p['waypoints'][0]['velocity_at_waypoint'][0]>0
    assert p['waypoints'][1]['velocity_at_waypoint'][0]>0
    assert p['waypoints'][-1]['velocity_at_waypoint']==[0.]*6
    q=np.asarray(p['targets14']);assert np.all(q[:,6:]==0)
    for w in p['waypoints'][:-1]:
        i=w['sample_end']-1
        assert q[i,0]-q[i-1,0]>1e-5 and q[i+1,0]-q[i,0]>1e-5
    assert p['dynamics']['max_joint_line_departure_rad']<=.01

@pytest.mark.parametrize('indices',[[2],[-1],[0,0],[True]])
def test_invalid_blend_does_not_stage(rig,indices):
    t,_,_,_=rig
    with pytest.raises(ValueError):t.preview_waypoints([0.]*14,[[.1,0,0,0,0,0],[.2,0,0,0,0,0]],[{'name':'table'}],blend_indices=indices)
    assert t.state=='IDLE'


def test_full_closure_one_monotonic_wire_with_fixed_arms(rig):
    t,_,_,_=rig;a=[0.]*14;a[6]=.0156;b=a.copy();b[6]=.003
    with pytest.raises(ValueError,match='10 mm'):t.preview(a,b,None)
    p=t.preview(a,b,None,full_closure=True);q=np.asarray(p['targets14'])
    assert np.all(np.diff(q[:,6])<=0) and abs(q[-1,6]-.003)<1e-8
    assert np.all(q[:,:6]==0) and np.all(q[:,7:]==0)
    assert max(abs(np.diff(np.r_[a[6],q[:,6]])))*30<=.01
    assert p['duration_s']<3

@pytest.mark.parametrize('kind',['opening','arm_move','over20mm'])
def test_full_closure_rejects_wrong_scope(rig,kind):
    t,_,_,_=rig;a=[0.]*14;a[6]=.0156;b=a.copy();b[6]=.003
    if kind=='opening':b[6]=.018
    if kind=='arm_move':b[0]=.001
    if kind=='over20mm':a[6]=.021
    with pytest.raises(ValueError):t.preview(a,b,None,full_closure=True)
    assert t.state=='IDLE'


def test_isolated_owner_publishes_completion_change_without_periodic_wait():
    import ast
    p=ROOT/'owner-runtime/real_world/robots/piper_x_dual/dora_owner_node.py'
    tree=ast.parse(p.read_text());found=False
    for node in ast.walk(tree):
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='signature' for t in node.targets):
            if 'rax_motion' in [n.value for n in ast.walk(node) if isinstance(n,ast.Constant)]:found=True
    assert found
    script=(ROOT/'scripts/piper_observe_only.py').read_text()
    assert "OWNER = ROOT / 'owner-runtime'" in script
    assert (ROOT/'owner-runtime/XPolicyLab').resolve()==(ROOT/'owner-runtime').resolve()
