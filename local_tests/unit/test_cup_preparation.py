"""Preparation owns targets/IK alternatives, never main-agent motion guesses."""
import importlib.util
import json
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_piper_visual_facts import perception,rehearsal_rig
from test_cup_approach import rehearsal,scene,fk


def test_preparation_target_tracks_layout_and_stays_outside_cup(rehearsal):
    a=rehearsal.preparation_goal(scene(),fk())
    assert a['position']==pytest.approx([.23,-.06,.29])
    b=rehearsal.preparation_goal(scene((.30,-.21,.12)),fk())
    assert b['position']==pytest.approx([.30,-.11,.31])
    R=Rotation.from_quat(a['quaternion_xyzw']).as_matrix()
    assert R[:,2]==pytest.approx([0,0,-1])
    assert abs(np.dot(R[:,0],[0,1,0]))==pytest.approx(1)


def test_low_start_lifts_before_rotating_long_fingers(rehearsal):
    f=fk((.10,0,.22));f['quaternion_xyzw']=Rotation.from_euler('y',90,degrees=True).as_quat().tolist()
    goal={'position':[.23,-.06,.29],'quaternion_xyzw':fk()['quaternion_xyzw']}
    c=rehearsal.preparation_candidates(f,goal)
    assert c
    for x in c:
        assert x['position'][2]>.22
        assert x['position'][1]==0
        assert x['quaternion_xyzw']==pytest.approx(f['quaternion_xyzw'])
        assert np.linalg.norm(np.array(x['position'])-f['position'])<=.055001


def test_rotation_candidates_are_bounded_and_deterministic(rehearsal):
    f=fk((.12,0,.29));f['quaternion_xyzw']=Rotation.from_euler('y',90,degrees=True).as_quat().tolist()
    goal=rehearsal.preparation_goal(scene(),f)
    c=rehearsal.preparation_candidates(f,goal)
    assert c==rehearsal.preparation_candidates(f,goal)
    assert any(x['position'][0]>.12 for x in c)
    for x in c:
        angle=(Rotation.from_quat(x['quaternion_xyzw'])*Rotation.from_quat(f['quaternion_xyzw']).inv()).magnitude()
        assert angle<=np.deg2rad(35)+1e-6
        assert np.linalg.norm(np.array(x['position'])-f['position'])<=.055001


def test_prepare_rejects_external_waypoints_before_hardware(rehearsal):
    class NoHardware:
        def value(self,*a,**k):raise AssertionError('No hardware allowed')
    r=rehearsal.prepare_for_approach(NoHardware(),{'delta':[.05,0,0]})
    assert not r['ok'] and not r['executed']


def test_unreachable_candidate_queries_next_without_moving_or_double_solving(rehearsal_rig,monkeypatch):
    m,rig=rehearsal_rig;original=rig.value;ik=[]
    def value(name,**kw):
        if name=='piper_solve_ik':
            ik.append(kw['position'])
            if len(ik)==1:raise RuntimeError('MCP tool failed: '+json.dumps({'error':{'reason':'MOVEIT_NO_IK_SOLUTION','no_action_executed':True,'stage':'validation'}}))
        return original(name,**kw)
    rig.value=value
    r=m.move_once(rig,{'name':'preparation_segment','tool':'piper_move_pose','args':{'position':[.01,0,.3]},
        'pose_candidates':[{'position':[.01,0,.3]},{'position':[.03,0,.3]}], 'pre_facts':{},'post_facts':{}})
    assert r['ok'] and rig.moves==1,r
    assert ik==[[.01,0,.3],[.03,0,.3]]
    assert r['args']['position']==[.03,0,.3]
    assert r['ik_calls']==2


def test_no_candidate_retry_after_uncertain_tool_error(rehearsal_rig):
    m,rig=rehearsal_rig;original=rig.value
    def value(name,**kw):
        if name=='piper_solve_ik':raise RuntimeError('MCP tool failed: '+json.dumps({'error':{'reason':'control_uncertain','stage':'robot_execution'}}))
        return original(name,**kw)
    rig.value=value
    r=m.move_once(rig,{'name':'preparation_segment','tool':'piper_move_pose','args':{'position':[.01,0,.3]},
        'pose_candidates':[{'position':[.01,0,.3]},{'position':[.03,0,.3]}], 'pre_facts':{},'post_facts':{}})
    assert not r['ok'] and not rig.moves
    assert sum(n=='piper_solve_ik' for n,k in rig.calls)==0 # intercepted before fake dispatch


def test_rim_probe_uses_supported_surface_not_foreground_median(perception):
    points=np.array([[0,0,.6]]*9,float);pixels=np.array([[2,2]]*9)
    class SavedDepth:
        def value(self,name,**kw):
            assert name=='piper_pixel_to_position'
            # Nearby robot foreground dominates3x3; selected rim sample is valid.
            return {'points':[{'position':[0,0,.6 if kw['radius']==0 else .3]}],'wall_budget':{}}
    near,pixel,_=perception.supported_rim_probe(SavedDepth(),{'frame_id':'saved'},points,pixels)
    assert near==[0,0,.6] and pixel==[2,2]

@pytest.mark.parametrize('fail_step',[None,2])
def test_prepare_progresses_without_agent_waypoints_and_stops_on_failed_action(rehearsal,monkeypatch,fail_step):
    state={'position':[.10,0,.22],'quaternion_xyzw':Rotation.from_euler('y',90,degrees=True).as_quat().tolist()}
    from test_cup_retained_target import observed,robot_status
    actions=[];localizations=[]
    q=observed()['measured14']
    def obs(depth=False):return observed(q,depth=depth,ident='frame-'+str(len(actions)))
    def localize(ctx,e):
        localizations.append(e);s=scene();s['observation']=obs(True);return s
    class ContextFK:
        def value(self,name,**kw):
            if name=='piper_status':return robot_status(q,len(actions))
            if name=='status':return {}
            if name=='piper_observe':
                assert kw['require_depth'] is False
                return obs()
            assert name=='piper_fk'
            return {**state,'effective_contact_tcp':{'calibration_id':'left-tcp140-tip170-20261104'}}
    def move(ctx,e):
        actions.append(e)
        if len(actions)==fail_step:return {'ok':False,'executed':None,'motion_arrived':False,'reason':'uncertain dispatch','visual_calls':1,'perception_s':.1}
        if e['tool']=='piper_gripper':q[6]=e['args']['width_m']
        else:state.update(e['pose_candidates'][0])
        return {'ok':True,'executed':True,'motion_arrived':True,'visual_calls':1,'perception_s':.1,'planning_s':.1,'motion_s':1,'ik_calls':1,'after_frame':obs()['frame_id'],'receipt':{'executed':True,'measured_arrival':True,'final14':q.copy(),'observation':obs()}}
    monkeypatch.setattr(rehearsal,'localize_supported',localize)
    monkeypatch.setattr(rehearsal,'move_once',move)
    result=rehearsal.prepare_for_approach(ContextFK(),{})
    if fail_step:
        assert len(actions)==2 and not result['ok'] and result['executed'] is None
    else:
        assert result['ok'] and result['task_completed']
        assert 1<len(actions)<=10
        assert state['position']==pytest.approx([.23,-.06,.29])
        assert result['ik_calls']==len(actions)
        assert result['visual_calls']==len(actions)+1
        assert len(localizations)==1
        assert all(a['scene_reference']['frame_id']=='frame-0' for a in actions)


def test_all_unreachable_candidates_never_dispatch(rehearsal_rig):
    m,rig=rehearsal_rig;original=rig.value;attempts=[]
    def value(name,**kw):
        if name=='piper_solve_ik':
            attempts.append(kw)
            raise RuntimeError('MCP tool failed: '+json.dumps({'error':{'reason':'MOVEIT_NO_IK_SOLUTION','stage':'validation','no_action_executed':True}}))
        return original(name,**kw)
    rig.value=value
    r=m.move_once(rig,{'name':'preparation_segment','tool':'piper_move_pose','args':{'position':[.01,0,.3]},
        'pose_candidates':[{'position':[.01,0,.3]},{'position':[.03,0,.3]}],'pre_facts':{},'post_facts':{}})
    assert len(attempts)==2 and not r['ok'] and not r['executed'] and rig.moves==0
    assert r['visual_calls']==0
