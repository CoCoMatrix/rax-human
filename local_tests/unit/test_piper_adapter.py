import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import time
import numpy as np
import pytest
from services.franka import arming
from services.piper.backend import PiperBackend
from services.piper.bridge import OwnerBridge, RejectedBeforeDispatch
from services.piper.cameras import Cameras, CALIBRATION_SHA, MODEL_SHA
from services.piper.geometry import Kinematics
from services.piper.owner import SERIALS


class Owner:
    def __init__(self): self.q=[0.]*6+[.019]+[0.]*6+[.002];self.reads=0;self.noise=0
    def state(self):
        self.reads+=1;q=self.q.copy()
        if self.reads>1:q[7]+=self.noise
        return {'robot_state':{'values':q},'robot_status':{'generation':1},'cameras':{
            c:{'camera_serial':serial,'available':True,'age_seconds':.01} for c,serial in SERIALS.items()}}
    def camera(self,name):
        stamp=time.time_ns()
        return np.zeros((12,16,3),np.uint8),{'binding':'test bracket','before':{'timestamp_ns':stamp},'after':{'timestamp_ns':stamp}}


class Bridge:
    def __init__(self):self.moves=[]
    def armed(self):pass
    def move(self,q,duration):self.moves.append(q);return {'executed':True,'measured_arrival':True,'final14':q}
    def close(self):pass


def backend(tmp_path,live=False,owner=None,bridge=None):
    c=SimpleNamespace(root=tmp_path,backend_options={'motion_enabled':live,**({'workspace_min':[-1,-1,-1],'workspace_max':[1,1,1]} if live else {})},observation_profile='rgb-only')
    b=PiperBackend(c,owner=owner or Owner(),bridge=bridge)
    if live:arming.write(tmp_path,True,minutes=2)
    return b


def test_no_send_cannot_execute_even_when_armed(tmp_path):
    b=backend(tmp_path);arming.write(tmp_path,True)
    with pytest.raises(RejectedBeforeDispatch,match='NO_SEND'):b.call('piper_move_joints',{'joints':[0.]*6})
    assert b.call('piper_fk',{'joints':[0.]*6})[0]['executed'] is False
    assert b.solver.process is None


def test_true_dual_fk_but_only_left_motion_tools(tmp_path):
    b=backend(tmp_path)
    for side in ('left','right'):
        result,_=b.call('piper_fk',{'joints':[0.]*6,'arm':side})
        assert result['frame']==side+'_base_link'
        assert np.isfinite(result['T_base_flange']).all()
    with pytest.raises(ValueError):b.validate('piper_move_joints',{'joints':[0.]*6,'arm':'right'})


@pytest.mark.parametrize('values',[[float('nan')]*6,[True]*6,[0.]*5,[0,0,0,0,0,3.]])
def test_invalid_or_beyond_cad_joint_values(tmp_path,values):
    with pytest.raises(ValueError):backend(tmp_path).call('piper_fk',{'joints':values})


def test_pre_dispatch_joint_limit_no_move(tmp_path):
    bridge=Bridge();b=backend(tmp_path,True,bridge=bridge)
    with pytest.raises(RejectedBeforeDispatch) as failure:b.call('piper_move_joints',{'joints':[1.1,0,0,0,0,0]})
    assert failure.value.dispatch_started is False and not bridge.moves


def test_stationary_right_encoder_noise_is_preserved(tmp_path):
    owner=Owner();owner.noise=.0001;bridge=Bridge();b=backend(tmp_path,True,owner,bridge)
    b.call('piper_move_joints',{'joints':[.01,0,0,0,0,0]})
    assert len(bridge.moves)==1 and bridge.moves[0][7]==.0001
    assert bridge.moves[0][6]==.019


def test_right_drift_rejects_without_dispatch(tmp_path):
    owner=Owner();owner.noise=.011;bridge=Bridge();b=backend(tmp_path,True,owner,bridge)
    with pytest.raises(RejectedBeforeDispatch):b.call('piper_move_joints',{'joints':[.01,0,0,0,0,0]})
    assert not bridge.moves


@pytest.mark.parametrize('failure',[None,'stale','wrong_serial','missing'])
def test_state_only_motion_never_fetches_jpeg_but_keeps_stream_guard(tmp_path,failure):
    owner=Owner();original=owner.state
    def state():
        d=original()
        if failure=='stale':d['cameras']['left']['age_seconds']=.51
        if failure=='wrong_serial':d['cameras']['left']['camera_serial']='wrong'
        if failure=='missing':d['cameras'].pop('left')
        return d
    owner.state=state
    def forbid(*args):raise AssertionError('Motion must not fetch image JPEGs')
    owner.camera=forbid;bridge=Bridge();b=backend(tmp_path,True,owner,bridge)
    if failure:
        with pytest.raises(RejectedBeforeDispatch,match='Camera'):b.call('piper_move_joints',{'joints':[.01,0,0,0,0,0],'observe_after':False})
        assert not bridge.moves
    else:
        result,images=b.call('piper_move_joints',{'joints':[.01,0,0,0,0,0],'observe_after':False})
        assert result['executed'] and len(bridge.moves)==1 and images==[]


def test_gripper_physical_limit_and_step(tmp_path):
    bridge=Bridge();b=backend(tmp_path,True,bridge=bridge)
    with pytest.raises(ValueError):b.call('piper_gripper',{'width_m':.084})
    with pytest.raises(RejectedBeforeDispatch):b.call('piper_gripper',{'width_m':.04})
    assert not bridge.moves


def depth_packet(root,age_ns=0):
    folder=root/'sample';folder.mkdir(parents=True)
    rgb=np.zeros((720,1280,3),np.uint8);xyz=np.zeros(rgb.shape,np.float32);xyz[...,2]=.7
    valid=np.ones(rgb.shape[:2],bool);np.savez(folder/'observation.npz',rgb=rgb,xyz_rgb_m=xyz,valid=valid)
    stamp=time.time_ns()-age_ns
    report={'schema':'rax.piper.ffs.v1','mode':'live','source':'fast_foundation_stereo','inference_kind':'real_model',
            'serial':'262522073779','calibration_payload_sha256':CALIBRATION_SHA,'checkpoint_sha256':MODEL_SHA,
            'sample_id':'sample','owner_generation':'owner','host_received_unix_ns':stamp,'expires_unix_ns':stamp+1_000_000_000,
            'arrays_sha256':hashlib.sha256((folder/'observation.npz').read_bytes()).hexdigest()}
    (folder/'report.json').write_text(json.dumps(report));(root/'latest.json').write_text(json.dumps({**report,'result_dir':'sample'}))
    return report


def test_live_expired_depth_rejected(tmp_path):
    depth_packet(tmp_path/'depth',2_000_000_000)
    cameras=Cameras(Owner(),tmp_path/'frames',tmp_path/'depth')
    with pytest.raises(ValueError,match='stale/future'):cameras.depth()


def test_saved_depth_analysis_does_not_expire_or_force_hold(tmp_path):
    depth_packet(tmp_path/'depth');c=Cameras(Owner(),tmp_path/'frames',tmp_path/'depth')
    meta,_=c.capture(['head'],True);ident=meta['frame_id'];path=tmp_path/'frames'/ident/'meta.json'
    saved=json.loads(path.read_text());saved['cameras']['head']['source_timestamp_ns']-=10_000_000_000;path.write_text(json.dumps(saved))
    result=c.pixel(ident,'head',[[643.140380859375,379.1578063964844]])
    expected=c.T[:3,:3]@np.array([0,0,.7])+c.T[:3,3]
    np.testing.assert_allclose(result['points'][0]['position'],expected,atol=1e-7)
    assert result['age_s']>10 and result['points'][0]['valid_samples']==49 and result['motion_permit'] is False
    with pytest.raises(ValueError,match='Wrist'):c.pixel(ident,'left',[[1,1]])


def test_depth_payload_tamper_and_path_escape_rejected(tmp_path):
    depth_packet(tmp_path/'depth');c=Cameras(Owner(),tmp_path/'frames',tmp_path/'depth')
    with (tmp_path/'depth/sample/observation.npz').open('ab') as f:f.write(b'changed')
    with pytest.raises(ValueError,match='identity'):c.depth()
    pointer=json.loads((tmp_path/'depth/latest.json').read_text());pointer['result_dir']='../outside'
    (tmp_path/'depth/latest.json').write_text(json.dumps(pointer))
    with pytest.raises(ValueError,match='relative'):c.depth()


def test_bridge_session_and_freshness_pin(tmp_path):
    root=tmp_path/'bridge';root.mkdir();session=tmp_path/'owner';session.mkdir()
    bridge=OwnerBridge(root,session,tmp_path)
    row={'schema':'rax.piper.motion_bridge.v1','session_root':str(session),'timestamp_ns':time.time_ns(),'instance':'first','phase':'IDLE'}
    (root/'state.json').write_text(json.dumps(row));assert bridge.status()['phase']=='IDLE'
    row['instance']='other';(root/'state.json').write_text(json.dumps(row))
    with pytest.raises(ValueError,match='instance'):bridge.status()
    row['instance']='first';row['timestamp_ns']-=2_000_000_000;(root/'state.json').write_text(json.dumps(row))
    with pytest.raises(ValueError,match='stale'):bridge.status()


def test_live_requires_explicit_workspace(tmp_path):
    config=SimpleNamespace(root=tmp_path,backend_options={'motion_enabled':True},observation_profile='rgb-only')
    with pytest.raises(ValueError,match='workspace'):PiperBackend(config,owner=Owner(),bridge=Bridge())


def test_joint_path_leaves_box_even_when_endpoints_inside(tmp_path):
    bridge=Bridge();b=backend(tmp_path,True,bridge=bridge)
    b.box_min=np.array([-1,-1,.19]);b.box_max=np.array([1,1,.22])
    target=[0.,.9,-.3,0.,0.,0.]
    assert .19<b.kin.fk([0.]*6)[2,3]<.22
    assert .19<b.kin.fk(target)[2,3]<.22
    with pytest.raises(RejectedBeforeDispatch,match='workspace'):b.call('piper_move_joints',{'joints':target})
    assert not bridge.moves


def test_pure_path_preflight_includes_same_workspace_guard(tmp_path):
    bridge=Bridge();b=backend(tmp_path,True,bridge=bridge)
    b.box_min=np.array([-1,-1,.19]);b.box_max=np.array([1,1,.22])
    class Clear:
        def path(self,*a,**kw):return {'cad_clear':True,'whole_path_checked':True}
    b.collision=Clear()
    start=[0.]*14;target=start.copy();target[:6]=[0.,.9,-.3,0.,0.,0.]
    result,_=b.call('piper_check_path',{'start14':start,'target14':target})
    assert result['cad_clear'] is True
    assert result['workspace_clear'] is False
    assert result['workspace_check']['first_violation']['axis']=='z'
    assert 'workspace' in result['workspace_check']['reason']
    assert not bridge.moves


def test_recorded_new_layout_workspace_360_vs_500(tmp_path):
    b=backend(tmp_path)
    start=[0.,1.390992507254441,-1.2005421792768196,1.4677695410496714,0.,0.]
    end=[-.07379159580586754,1.7816803797732836,-1.6534347974858792,1.529642744507493,4.869576216240342e-10,-.07190750428242476]
    b.box_min=np.array([.055,-.4,.17]);b.box_max=np.array([.36,.045,.36])
    old=b._workspace_path(start,end)
    assert old['clear'] is False and old['first_violation']['sample_index']==37
    b.box_max[0]=.5
    assert b._workspace_path(start,end)['clear'] is True


def test_bridge_read_preflight_failure_never_enqueues(tmp_path):
    bridge=OwnerBridge(tmp_path/'missing',tmp_path/'owner',tmp_path)
    with pytest.raises(RejectedBeforeDispatch) as exc:bridge.move([0.]*14)
    assert exc.value.dispatch_started is False
    assert not (tmp_path/'missing/requests').exists()


def test_depth_resized_pixels_cannot_use_native_intrinsics(tmp_path):
    report=depth_packet(tmp_path/'depth');folder=tmp_path/'depth/sample'
    np.savez(folder/'observation.npz',rgb=np.zeros((360,640,3),np.uint8),xyz_rgb_m=np.ones((360,640,3),np.float32),valid=np.ones((360,640),bool))
    report['arrays_sha256']=hashlib.sha256((folder/'observation.npz').read_bytes()).hexdigest()
    (folder/'report.json').write_text(json.dumps(report))
    c=Cameras(Owner(),tmp_path/'frames',tmp_path/'depth')
    with pytest.raises(ValueError,match='native intrinsics'):c.depth()


def test_one_rax_client_per_bridge_until_closed(tmp_path):
    root=tmp_path/'bridge';root.mkdir()
    first=OwnerBridge(root,tmp_path/'owner',tmp_path/'session-a')
    second=OwnerBridge(root,tmp_path/'owner',tmp_path/'session-b')
    first._claim()
    try:
        with pytest.raises(ValueError,match='Another RAX session'):
            second._claim()
    finally:
        first.close()
    second._claim()
    second.close()
    with pytest.raises(ValueError,match='closed'):
        first._claim()


@pytest.mark.parametrize('distance,accepted',[(.4,True),(.40001,False)])
def test_pose400mm_bound_precedes_solver_and_preserves_existing_execution_guards(tmp_path,distance,accepted):
    bridge=Bridge();b=backend(tmp_path,True,bridge=bridge);calls=[]
    current=b.kin.fk([0.]*6)[:3,3]
    def solve(*args):calls.append(args);return {'q':[0.]*6}
    b.solver.solve=solve
    args={'position':(current+[distance,0,0]).tolist(),'observe_after':False}
    if accepted:b.call('piper_move_pose',args)
    else:
        with pytest.raises(RejectedBeforeDispatch,match='400mm'):b.call('piper_move_pose',args)
    assert len(calls)==int(accepted) and len(bridge.moves)==int(accepted)
