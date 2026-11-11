"""Offline outcomes: relaxed image detail does not invent retention or support."""
import copy
import importlib.util
import sys
import time
import pytest
from test_piper_visual_facts import perception, rehearsal_rig, Context, observation
from test_cup_approach import rehearsal, scene, fk

@pytest.fixture
def placement_rig(rehearsal_rig,perception,monkeypatch):
    module,rig=rehearsal_rig
    monkeypatch.setitem(sys.modules,'rehearsal',module)
    spec=importlib.util.spec_from_file_location('qualitative_placement',module.__file__.replace('rehearsal.py','placement.py'))
    p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
    rig.q[6]=.0156
    rig.obs['state_binding']['owner_generation']=1
    rig.vision=[];rig.targets=[];rig.fail_dispatch=False;rig.verdict='supported';rig.hazard=False
    original=rig.value
    def value(name,**kw):
        for c in rig.obs['cameras'].values():c['source_timestamp_ns']=time.time_ns()
        if name=='piper_gripper':
            rig.targets.append(kw['width_m']);rig.moves+=1
            if rig.fail_dispatch:return {'executed':None,'measured_arrival':False}
            rig.q[6]=kw['width_m']+.0003
            obs=copy.deepcopy(rig.obs);obs['frame_id']='after-'+str(rig.moves)
            return {'executed':True,'measured_arrival':True,'final14':rig.q[:],'observation':obs}
        return original(name,**kw)
    def visual(ctx,obs,question,**kw):
        rig.vision.append((question,kw))
        facts={k:{'state':rig.verdict,'evidence':'Overall cup position from combined views; one fine edge hidden','cameras':['head','left']} for k in kw['required_facts']}
        payload={'facts':facts,'visible_hazards':['Cup visibly falling'] if rig.hazard else []}
        return perception.visual(Context(payload),obs,question,**kw)
    rig.value=value;monkeypatch.setattr(module,'visual',visual)
    return p,module,rig


def test_characterized_closure_is_bounded_and_not_pickup_proof(placement_rig):
    p,m,rig=placement_rig
    r=p.grasp(rig)
    assert r['ok'] and r['retention_verified'] is False,r
    assert rig.targets==pytest.approx([.0056,.0035])
    assert len(rig.vision)==r['visual_calls']==1
    assert all('grasp_setup' in kw['required_facts'] for q,kw in rig.vision)
    assert r['next_action']=='trial_lift' and rig.moves==2
    assert not any(n=='piper_move_joints' for n,k in rig.calls)


def test_uncertain_closure_never_retries_or_lifts(placement_rig):
    p,m,rig=placement_rig;rig.fail_dispatch=True
    r=p.grasp(rig)
    assert not r['ok'] and r['executed'] is None
    assert len(rig.targets)==1 and r['next_action']=='observe_only'


@pytest.mark.parametrize('verdict,hazard,expected',[('supported',False,True),('unknown',False,False),('contradicted',False,False),('supported',True,False)])
def test_trial_lift_checks_overall_actual_outcome(placement_rig,verdict,hazard,expected):
    p,m,rig=placement_rig
    original=m.visual
    def result_after(ctx,obs,question,**kw):
        if 'cup_lifted_and_held' in kw['required_facts']:
            rig.verdict=verdict;rig.hazard=hazard
        return original(ctx,obs,question,**kw)
    m.visual=result_after
    r=p.trial_lift(rig)
    assert r['executed'] is True and r['ok'] is expected,r
    assert rig.moves==1 and len(rig.vision)==2
    assert set(rig.vision[-1][1]['required_facts'])=={'cup_lifted_and_held'}
    assert not r.get('operator_assistance')
    assert r['args']['position'][2]-r['before_fk']['position'][2]==pytest.approx(.040)
    assert r['post_control_context']['measured_arrival'] is True
    assert r['post_control_context']['commanded_flange_delta_m']==pytest.approx([0,0,.040])


@pytest.mark.parametrize('dz',[0,-.04,.071,float('nan'),float('inf')])
def test_trial_lift_rejects_invalid_or_unbounded_motion_before_tools(placement_rig,dz):
    p,m,rig=placement_rig
    r=p.trial_lift(rig,{'dz':dz})
    assert r['ok'] is False and r['executed'] is False
    assert rig.moves==0 and not rig.calls and not rig.vision


@pytest.mark.parametrize('requested,effective',[(.010,.040),(.020,.040),(.039,.040),(.040,.040),(.050,.050)])
def test_trial_lift_plans_at_least_experienced_distance(placement_rig,requested,effective):
    p,m,rig=placement_rig
    r=p.trial_lift(rig,{'dz':requested})
    assert r['ok'] and r['executed'] and rig.moves==1,r
    assert r['args']['position'][2]-r['before_fk']['position'][2]==pytest.approx(effective)
    # Full selected motion is checked, not a10mm path followed by a40mm dispatch.
    assert r['candidate_checks'][0]['args']['position']==r['args']['position']
    assert r['post_control_context']['commanded_flange_delta_m']==pytest.approx([0,0,effective])


@pytest.mark.parametrize('verdict',['unknown','contradicted'])
def test_release_without_overall_support_never_opens(placement_rig,verdict):
    p,m,rig=placement_rig;rig.q[6]=.0038;rig.verdict=verdict
    r=p.release(rig,{'width_m':.01})
    assert not r['executed'] and not rig.targets and not rig.moves,r


def test_onsite_accepts_recorded_j6_change_with_same_scope(rehearsal_rig):
    m,rig=rehearsal_rig;args={'delta':[0,0,.003]}
    first=m.approach_step(rig,args);path=rig.confirm(first)
    rig.q[5]+=.0011519173063162
    r=m.approach_step(rig,{**args,'operator_file':path})
    assert r['ok'] and rig.moves==1,r
    assert r['operator_assistance']['reply']==rig.answer['reply']


def test_onsite_still_rejects_changed_jaw(rehearsal_rig):
    m,rig=rehearsal_rig;args={'delta':[0,0,.003]}
    first=m.approach_step(rig,args);path=rig.confirm(first)
    rig.q[6]+=.0006
    r=m.approach_step(rig,{**args,'operator_file':path})
    assert not r['executed'] and rig.moves==0,r


def test_entry_depth_relative_to_current_rim_and_nominal_table(rehearsal):
    for rim in (.1,.12):
        s=scene((.23,-.16,rim));g=rehearsal.approach_goal(s,fk((.23,-.16,rim+.19)),'entry')
        assert g['goal_tcp'][2]==pytest.approx(rim-.040)
    for height in (.05,float('nan')):
        s=scene();s['table'][2]=height
        with pytest.raises(ValueError,match='tip/table'):
            rehearsal.approach_goal(s,fk((.23,-.16,.29)),'entry')


def test_hover_asks_overall_plausibility_not_exact_alignment(rehearsal,monkeypatch):
    questions=[]
    def visual(ctx,obs,question,**kw):
        questions.append((question,kw));return {'ok':True}
    monkeypatch.setattr(rehearsal,'visual',visual)
    obs={'frame_id':'fresh','cameras':{'head':{'source_timestamp_ns':time.time_ns()}}}
    r=rehearsal._verify_approach_goal(None,{'visual_calls':0,'perception_s':0},obs,'hover')
    assert r['ok']
    assert set(questions[0][1]['required_facts'])=={'hover_scene'}
    assert 'not precise 3D alignment' in questions[0][0]
