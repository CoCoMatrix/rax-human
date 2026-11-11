"""Retained geometry is provenance, never a clearance or replay authorization."""
import copy,importlib,time
import numpy as np
import pytest
from test_piper_visual_facts import perception
from test_cup_approach import rehearsal,scene,fk


def observed(q=None,depth=False,ident='new'):
    return {'frame_id':ident,'wall_budget':{'remaining_seconds':900},'measured14':q or [0.]*6+[.016]+[0.]*7,
        'state_binding':{'owner_generation':1,'rest_consistent':True},
        'calibration_sha256':'head-calibration','finger_geometry':{'geometry_identity':'geometry'},
        'cameras':{name:dict(source_timestamp_ns=time.time_ns(),metric_depth=depth,K=np.eye(3).tolist(),T_base_camera=np.eye(4).tolist()) for name in ('head','left','right')}}


def robot_status(q=None,motion=0):
    return {'motion_ready':True,'control_uncertain':False,'owner_status':{'generation':1},
        'bridge':{'instance':'bridge','phase':'IDLE'},'last_motion':{'sequence':motion},
        'measured_state':{'values':q or observed()['measured14']}}


@pytest.fixture
def target(rehearsal):return importlib.import_module('approach_target')


def initial(target):
    loc=scene();loc['observation']=observed(depth=True)
    return target.create(loc,robot_status())


def test_retained_point_has_old_measurement_and_new_execution_frame(target):
    r=initial(target);obs=observed(ident='occluded')
    target.validate(r,obs,robot_status())
    result=target.localization(r,obs)
    assert result['before_frame']=='occluded' and result['measurement_frame']=='new'
    assert result['near_wall']==scene()['near_wall'] and result['clearance_certified'] is False
    assert obs['cameras']['head']['metric_depth'] is False


@pytest.mark.parametrize('change',['future','owner','bridge','control','geometry','calibration','head_transform','joint','gripper','intervening_action','stale','unbound','consumed'])
def test_invalid_target_rejected_without_remeasurement(target,change):
    r=initial(target);obs=observed();robot=robot_status()
    if change=='future':r['created_ns']+=1_000_000_000
    if change=='owner':robot['owner_status']['generation']=2
    if change=='bridge':robot['bridge']['instance']='other'
    if change=='control':robot['control_uncertain']=True
    if change=='geometry':obs['finger_geometry']['geometry_identity']='changed'
    if change=='calibration':obs['calibration_sha256']='changed'
    if change=='head_transform':obs['cameras']['head']['T_base_camera'][0][3]=.02
    if change=='joint':obs['measured14'][0]=.011
    if change=='gripper':obs['measured14'][6]=.005
    if change=='intervening_action':robot['last_motion']['sequence']=1
    if change=='stale':obs['cameras']['head']['source_timestamp_ns']-=2_000_000_000
    if change=='unbound':obs['state_binding']['rest_consistent']=False
    if change=='consumed':target.invalidate('closure');r=__import__('json').loads(target.PATH.read_text())
    with pytest.raises(ValueError):target.validate(r,obs,robot)


@pytest.mark.parametrize('prepared',[False,True])
def test_hover_to_entry_uses_one_target_without_depth(rehearsal,target,monkeypatch,prepared):
    state={'p':np.array([.23,-.06 if prepared else -.16,.29]),'q':observed()['measured14'],'n':0}
    if prepared:state['q'][6]=.0003
    loc=scene();loc['observation']=observed(state['q'],depth=True)
    calls=[];actions=[]
    def localize(*a):calls.append('localize');return loc
    class Rig:
        def value(self,name,**kw):
            if name=='piper_fk':
                f=fk(state['p'])
                if prepared:
                    from scipy.spatial.transform import Rotation
                    f['quaternion_xyzw']=Rotation.from_matrix(np.asarray(f['effective_contact_tcp']['T_base_tcp'])[:3,:3]).as_quat().tolist()
                return f
            if name=='status':return {}
            if name=='piper_status':return robot_status(state['q'],state['n'])
            if name=='piper_observe':
                assert kw['require_depth'] is False
                calls.append('rgb');return observed(state['q'],ident='fresh-'+str(state['n']))
            raise AssertionError(name)
    rig=Rig()
    def move(ctx,e):
        actions.append(e);state['n']+=1;state['q']=state['q'].copy()
        if e['tool']=='piper_gripper':state['q'][6]=e['args']['width_m']
        else:state['p']=np.asarray(e['args']['position']);state['q'][0]+=.01
        obs=observed(state['q'],ident='after-'+str(state['n']))
        return dict(ok=True,executed=True,motion_arrived=True,after_frame=obs['frame_id'],receipt=dict(executed=True,measured_arrival=True,final14=state['q'],observation=obs))
    monkeypatch.setattr(rehearsal,'localize_supported',localize)
    monkeypatch.setattr(rehearsal,'move_once',move)
    monkeypatch.setattr(rehearsal,'visual',lambda *a,**kw:{'ok':True,'perception_s':.01})
    if prepared:
        preparation=rehearsal.prepare_for_approach(rig,{})
        assert preparation['ok'],preparation
    hover=rehearsal.approach(rig,{'phase':'hover'});assert hover['ok'],hover
    if prepared:assert preparation['target_id']==hover['target_id']
    entry=rehearsal.approach(rig,{'phase':'entry'});assert entry['ok'],entry
    assert hover['target_id']==entry['target_id'] and calls.count('localize')==1
    assert len(actions)==(6 if prepared else 2) and all('target_scene_consistent' in e['pre_facts'] for e in actions)
    assert entry['goal']['goal_tcp'][2]==pytest.approx(.06)
    assert state['q'][6]==.016  # no grasp implicitly appended
    assert not rehearsal.approach(rig,{'phase':'entry'})['ok']  # no replay after consumed phase


def test_old_geometry_is_not_relabelled_fresh_but_can_survive_model_pause(target):
    r=initial(target);r['created_ns']-=301_000_000_000
    obs=observed(ident='current-after-pause')
    target.validate(r,obs,robot_status())
    loc=target.localization(r,obs)
    assert loc['target_age_s']>300 and loc['measurement_frame']=='new'
    assert loc['before_frame']=='current-after-pause' and not loc['clearance_certified']


@pytest.mark.parametrize('delta,accepted',[(.006597344572538566,True),(.010001,False)])
def test_r28_j6_completion_feedback_policy(target,delta,accepted):
    r=initial(target);q=observed()['measured14'];q[5]=.006597344572538566
    current=q.copy();current[5]-=delta
    class Rig:
        def value(self,name,**kw):
            assert name=='piper_status'
            return robot_status(current,1)
    step={'ok':True,'after_frame':'actual-after','receipt':{'executed':True,'measured_arrival':True,'final14':q}}
    if accepted:
        target.advance(Rig(),r,step)
        assert r['expected14']==q and current[5]==0. # original values preserved
    else:
        with pytest.raises(ValueError,match='completion changed'):target.advance(Rig(),r,step)


def no_dispatch_report(target_id):
    return {'function':'approach','target_id':target_id,'ok':False,'executed':False,
            'motion_arrived':False,'reason':'provider closed connection','steps':[{
                'executed':False,'motion_arrived':False,'tool_error':{
                    'operation':'gemini_generate','stage':'provider_request',
                    'type':'RemoteDisconnected','no_action_executed':True,'control_uncertain':False}}]}


def test_known_no_dispatch_retains_measurement_for_one_fresh_retry(target):
    r=initial(target);r['phase']='hover';target.save(r);original=copy.deepcopy(r)
    report=no_dispatch_report(r['id'])
    assert target.retain_no_dispatch(r,report,'preparation_verified')
    saved=__import__('json').loads(target.PATH.read_text())
    assert saved['valid'] and saved['phase']=='preparation_verified'
    for key in ('geometry','created_ns','source_frame','source_observation','expected14','last_motion'):
        assert saved[key]==original[key]
    assert saved['pre_dispatch_recoveries']==1
    saved['phase']='hover';target.save(saved)
    assert not target.retain_no_dispatch(saved,report,'preparation_verified')


@pytest.mark.parametrize('change',['dispatched','uncertain','wrong_tool','wrong_stage','missing_proof','postcheck','second_segment','wrong_target','invalid','contact','changed_store','no_verified_phase'])
def test_no_dispatch_recovery_does_not_relax_other_failures(target,change):
    r=initial(target);r['phase']='hover';target.save(r);report=no_dispatch_report(r['id']);step=report['steps'][0];phase='preparation_verified'
    if change=='dispatched':step['dispatch_monotonic']=1.
    if change=='uncertain':step['tool_error']['control_uncertain']=True
    if change=='wrong_tool':step['tool_error']['operation']='piper_move_joints'
    if change=='wrong_stage':step['tool_error']['stage']='robot_execution'
    if change=='missing_proof':step['tool_error'].pop('no_action_executed')
    if change=='postcheck':report['executed']=True;report['motion_arrived']=True
    if change=='second_segment':report['steps'].append(copy.deepcopy(step))
    if change=='wrong_target':report['target_id']='other'
    if change=='invalid':r['valid']=False;target.save(r)
    if change=='contact':r['contact_started']=True;target.save(r)
    if change=='changed_store':other=copy.deepcopy(r);other['id']='changed';target.save(other)
    if change=='no_verified_phase':phase=None
    assert not target.retain_no_dispatch(r,report,phase)


def test_reenter_same_function_after_no_dispatch_gets_new_frame_and_scene_check(rehearsal,target,monkeypatch):
    r=initial(target);r['phase']='preparation_verified';target.save(r)
    calls=[];n=[0]
    class Rig:
        def value(self,name,**kw):
            if name=='status':return {}
            if name=='piper_status':return robot_status()
            if name=='piper_observe':
                n[0]+=1;return observed(ident='new-'+str(n[0]))
            if name=='piper_fk':return fk((.23,-.06,.29))
            raise AssertionError(name)
    def fails(ctx,e):
        calls.append(e)
        return {'ok':False,'reason':'provider closed connection',**no_dispatch_report(r['id'])['steps'][0]}
    monkeypatch.setattr(rehearsal,'move_once',fails)
    monkeypatch.setattr(rehearsal,'localize_supported',lambda *a:pytest.fail('must retain measured target'))
    a=rehearsal.approach(Rig(),{'phase':'hover'})
    assert a['target_retained'] and not a['executed'] and a['next_action']=='retry_same_function_with_fresh_checks'
    b=rehearsal.approach(Rig(),{'phase':'hover'})
    assert not b.get('target_retained')
    assert calls[0]['observation']['frame_id']!=calls[1]['observation']['frame_id']
    assert all('target_scene_consistent' in e['pre_facts'] for e in calls)
    assert not __import__('json').loads(target.PATH.read_text())['valid']
