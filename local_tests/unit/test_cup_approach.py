"""Regression: projection is not contact; fresh targets are not historical deltas."""
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np
import pytest
from test_piper_visual_facts import perception, observation, Context

SOURCE=Path(__file__).resolve().parents[2]/'workspaces/blue-cup/20261104-v0.3-r12-working/code/blue_cup/rehearsal.py'

@pytest.fixture
def rehearsal(perception,monkeypatch):
    monkeypatch.setitem(sys.modules,'perception',perception)
    s=importlib.util.spec_from_file_location('target_rehearsal',SOURCE)
    m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
    return m

def view(camera,relation,cue=None):
    v={'camera':camera,'relation':relation,'evidence':'Current view of both fingers relative to rim and wall'}
    if cue:v['physical_cue']=cue
    return v

def assess(perception,tmp_path,views,claimed='contradicted',hazards=None):
    r={'facts':{'target_separation':{'state':claimed,'evidence':'overlaps/contacts the rim/wall','cameras':['left'],'views':views}},'visible_hazards':hazards or []}
    return perception.visual(Context(r),observation(tmp_path),'Check separation',required_facts={'target_separation':'Both fingers separated from cup'})

def test_projection_only_is_unknown_never_contact_or_clearance(perception,tmp_path):
    r=assess(perception,tmp_path,[view('left','projection_overlap'),view('head','occluded')])
    assert r['facts']['target_separation']['state']=='unknown'
    assert r['contradicted_facts']==[] and r['unresolved_facts']==['target_separation']
    assert not r['ok']

def test_clear_other_view_resolves_projection_overlap(perception,tmp_path):
    r=assess(perception,tmp_path,[view('left','projection_overlap'),view('head','clear_gap')])
    assert r['ok'] and r['facts']['target_separation']['state']=='supported'

@pytest.mark.parametrize('cue',['visible_deformation','resolved_contact_interface'])
def test_direct_contact_cannot_be_overridden_by_other_view(perception,tmp_path,cue):
    r=assess(perception,tmp_path,[view('left','direct_contact',cue),view('head','clear_gap')],claimed='supported')
    assert not r['ok'] and r['contradicted_facts']==['target_separation']

def test_contact_claim_without_resolved_physical_evidence_is_unknown(perception,tmp_path):
    r=assess(perception,tmp_path,[view('left','direct_contact')])
    assert not r['ok'] and r['unresolved_facts']==['target_separation']
    assert not r['contradicted_facts']

def test_projection_diagnostic_is_not_a_physical_hazard(perception,tmp_path):
    r=assess(perception,tmp_path,[view('head','clear_gap'),view('left','projection_overlap')],hazards=[{'basis':'projection_only','evidence':'Left contours overlap','camera':'left'}])
    assert r['ok'] and not r['visible_hazards'] and r['projection_diagnostics']

def test_physical_hazard_remains_blocking(perception,tmp_path):
    r=assess(perception,tmp_path,[view('head','clear_gap')],hazards=[{'basis':'observed_obstruction','evidence':'Cable across route','camera':'head'}])
    assert not r['ok']

def test_legacy_overlap_contact_prose_never_becomes_confirmed_contact(perception,tmp_path):
    r=assess(perception,tmp_path,[])
    assert r['unresolved_facts']==['target_separation'] and not r['contradicted_facts']

def test_normalized_coordinates_are_scaled_once_and_checked(perception):
    assert perception.normalized_pixel([396,545],1280,720)==pytest.approx([506.88,392.4])
    for v in ([1001,0],[float('nan'),0],[1],[-1,0]):
        with pytest.raises(ValueError):perception.normalized_pixel(v,1280,720)

def fk(position=(.2,-.1,.29)):
    T=np.array([[0.,-1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]]);T[:3,3]=np.array(position)+[0,0,-.14]
    return {'position':list(position),'quaternion_xyzw':[1,0,0,0],
            'effective_contact_tcp':{'calibration_id':'left-tcp140-tip170-20261104','T_base_tcp':T.tolist(),'position':T[:3,3].tolist()}}

def scene(wall=(.23,-.16,.1)):
    return {'ok':True,'near_wall':list(wall),'cup_center_estimate':[wall[0],wall[1]-.035,wall[2]],'table':[.2,-.2,.02],
            'coaster_center':[.23,-.3,.025],'rim_support_samples':20,'rim_z_spread_m':.003,'before_frame':'new','after_frame':'new','visual_calls':1,'perception_s':.1,
            'observation':{'frame_id':'new','measured14':[0.]*6+[.016]+[0.]*7}}

def test_targets_follow_new_layout_and_tcp_offset_once(rehearsal):
    a=rehearsal.approach_goal(scene(),fk(),'hover')
    assert a['goal_tcp']==pytest.approx([.23,-.16,.15])
    assert a['goal_flange']==pytest.approx([.23,-.16,.29])
    b=rehearsal.approach_goal(scene((.25,-.19,.105)),fk((.2,-.1,.31)),'hover')
    assert b['goal_flange']==pytest.approx([.25,-.19,.295])

def test_no_horizontal_sweep_below_rim_or_entry_before_alignment(rehearsal):
    with pytest.raises(ValueError,match='staging'):
        rehearsal.approach_goal(scene(),fk((.2,-.1,.24)),'hover')
    with pytest.raises(ValueError,match='align'):
        rehearsal.approach_goal(scene(),fk(),'entry')

def test_entry_requires_downward_known_tcp_and_open_jaw(rehearsal):
    for change in ('rotation','calibration','jaw'):
        s=scene();f=fk((.23,-.16,.29))
        if change=='rotation':f['effective_contact_tcp']['T_base_tcp'][2][2]=1
        elif change=='calibration':f['effective_contact_tcp']['calibration_id']='old'
        else:s['observation']['measured14'][6]=.004
        with pytest.raises(ValueError):rehearsal.approach_goal(s,f,'entry')

def test_approach_does_not_take_historical_position_or_delta(rehearsal):
    class NoHardware:
        def value(self,*a,**k):raise AssertionError('must reject before hardware calls')
    r=rehearsal.approach(NoHardware(),{'delta':[0,0,-.01]})
    assert not r['ok'] and not r['executed']

def test_arrival_does_not_equal_approach_completion(rehearsal,monkeypatch):
    from test_cup_retained_target import observed
    monkeypatch.setattr(rehearsal,'visual',lambda *a,**k:{'ok':False})
    r={'motion_arrived':True,'visual_calls':0,'perception_s':0}
    rehearsal._verify_approach_goal(None,r,observed(),'hover')
    assert r['motion_arrived'] and not r['task_completed'] and not r['ok']
    assert r['visual_calls']==1

def test_wrong_opening_axis_never_enters_wall(rehearsal):
    f=fk((.23,-.16,.29));T=np.diag([1,-1,-1,1.]);T[:3,3]=[.23,-.16,.15]
    f['effective_contact_tcp']['T_base_tcp']=T.tolist()
    with pytest.raises(ValueError,match='opening axis'):rehearsal.approach_goal(scene(),f,'entry')

def test_named_top_level_views_bind_to_same_separation_fact(perception,tmp_path):
    payload={'facts':{'target_separation':{'state':'contradicted','evidence':'overlap','cameras':['left']}},
        'target_separation':{'views':[view('left','clear_gap'),view('head','occluded')]},'visible_hazards':[]}
    r=perception.visual(Context(payload),observation(tmp_path),'Inspect',required_facts={'target_separation':'Both fingers separated'})
    assert r['ok'] and r['facts']['target_separation']['state']=='supported'
