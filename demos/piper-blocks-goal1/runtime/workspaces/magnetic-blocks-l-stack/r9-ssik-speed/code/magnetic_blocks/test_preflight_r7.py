"""Search-order coverage; every returned edge still carries full checks."""
from types import SimpleNamespace
import numpy as np
import pytest
import assembly_plan as a
from test_c_four_search import planner


def finish_fixture(direct_ok=True):
    calls=[];start=[0.]*14;start[6]=a.CLOSED
    scene={'blocks':{'green':{'top_center':{'surface_position_m':[.2,-.1,.12]}}}}
    def search(phase,fn):
        if False:yield
        return fn()
    def check(*args,**kw):
        calls.append(('check',kw))
        return {'native':{'cad_clear':True,'workspace_clear':True}}
    def edge(opened,dest,q,scene,**kw):
        calls.append(('direct',list(dest)))
        if not direct_ok or dest[2]>.27:raise a.NoPlan('first endpoint is unreachable')
        end=list(opened);end[0]=.1
        return [{'start14':list(opened),'target14':end,'check':check()}],end
    def route(opened,dest,q,scene,**kw):
        calls.append(('routed',list(dest)))
        if direct_ok:raise AssertionError('detours attempted before cheaper alternative endpoint')
        end=list(opened);end[0]=.2
        return [{'start14':list(opened),'target14':end,'check':check()}]
    p=SimpleNamespace(search_step=search,check=check,route=route,connect_edge=edge,fk=lambda q:{'position':[.2,-.1,.2]})
    gen=a.Planner.finish_steps(p,start,scene,'green',[.2,-.1,.12],{},a.quat(180),'red',None)
    with pytest.raises(StopIteration) as stop:next(gen)
    return stop.value.value,calls


def test_retreat_checks_next_direct_endpoint_before_detours():
    result,calls=finish_fixture()
    assert [x[0] for x in calls].count('direct')==2
    assert not any(x[0]=='routed' for x in calls)
    assert len(result['actions'][-1]['release_feedback_preflight']['checks'])==2
    assert 'next_pickup_check' in result['actions'][-1]


def test_routed_withdrawal_remains_available_after_direct_portfolio():
    result,calls=finish_fixture(False)
    first_routed=next(i for i,v in enumerate(calls) if v[0]=='routed')
    assert sum(v[0]=='direct' for v in calls[:first_routed])>10
    assert result['end14'][0]==.2


def test_clear_stack_lift_keeps_orientation_before_turn_and_checks_both_edges():
    p=planner();p.local[a.OPEN][:,2]=.17;p.search_active=False
    initial=[0.]*14;initial[6]=a.OPEN
    p.scene['blocks']['green']['top_center']['surface_position_m']=[.25,-.1,.14]
    q0=a.quat(180);q1=a.quat(90);pos=np.array([.25,0.,.29]);T=np.eye(4);T[:3,:3]=a.Rotation.from_quat(q0).as_matrix();T[:3,3]=pos
    p.fk=lambda q:{'position':pos.tolist(),'quaternion_xyzw':q0,'T_base_flange':T.tolist()}
    calls=[]
    def edge(start,dest,q,scene,**kw):
        calls.append((list(start),list(dest),list(q)))
        if start[0]==0 and (not np.allclose(q,q0) or dest[2]<=pos[2]):
            raise a.NoPlan('turning before clearing stack rejected')
        end=list(start);end[0]+=.1
        return [{'start14':list(start),'target14':end,'check':{'min_neighbor_m':.03}}],end
    p.connect_edge=edge
    p.check=lambda *args,**kw:(_ for _ in ()).throw(a.NoPlan('shortcut would cross stack'))
    result=p._route(initial,[.32,-.22,.25],q1,p.scene)
    assert len(result)==2 and result[0]['route_strategy']=='clear_stack_up_before_turn'
    assert np.allclose(calls[1][2],q0) and calls[1][1][2]>pos[2]
    assert result[0]['target14']==result[1]['start14']


@pytest.mark.parametrize('yaw',[180.,-90.,0.,90.])
def test_yellow_grasp_priority_follows_incoming_heading_without_removing_poses(yaw):
    p=planner();p.fk=lambda q:{'quaternion_xyzw':a.quat(yaw)}
    p.scene['blocks']['yellow']['top_center']['surface_position_m']=[.4,-.12,.078]
    p.progress=lambda **kw:None
    def family(start,scene,color,target,candidate):
        if False:yield
        return {'candidate':candidate}
    p._family_steps=family
    result=p.transfer([0.]*14,p.scene,'yellow',[.3,-.2,.198])
    candidate=result['candidate']
    assert candidate['yaw']==yaw and candidate['tilt_degrees']==25.
    job=next(iter(p.transfer_jobs.values()))
    retained=[candidate]+[t['candidate'] for t in job['queue']]+job['pending']
    assert None in retained
    assert all(v in retained for v in a.grasp_variants(p.scene,'yellow'))
