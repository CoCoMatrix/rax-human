import builtins
import numpy as np
import pytest
import assembly_plan as ap
from test_composition import SceneContext
from test_c031_fk import Reference
from pure_fk import local_kinematics


def test_required_bundle_missing_is_not_a_silent_remote_fallback(monkeypatch):
    orig=builtins.__import__
    def missing(name,*a,**kw):
        if name.startswith('native_fk'):raise ImportError('isolated package missing')
        return orig(name,*a,**kw)
    monkeypatch.setattr(builtins,'__import__',missing)
    c=Reference()
    with pytest.raises(RuntimeError,match='Required local FK bundle'):local_kinematics(c,[0.]*6)
    assert not c.calls and not hasattr(c,'_blocks_validated_fk')


def test_implementation_hash_mismatch_rejected_before_rpc(monkeypatch):
    import native_fk
    monkeypatch.setattr(native_fk,'IMPLEMENTATION_SHA256','bad')
    c=Reference()
    with pytest.raises(ValueError,match='implementation identity'):local_kinematics(c,[0.]*6)
    assert not c.calls


def test_native_validation_finishes_preferred_plan_after_search_timeslice(monkeypatch):
    s=SceneContext();p=ap.Planner(s,s.scene,s.robot(),np.zeros((16,3)),np.zeros((16,3)))
    p.search_active=True;p.started=0.;p.ik_mode='fast';p.candidate_deadline=8.;p.candidate_yielded=True
    clock=[7.];monkeypatch.setattr(ap.time,'monotonic',lambda:clock[0])
    visited=[]
    def certify(moves):
        for t in (7.,9.,31.):clock[0]=t;p.progress();visited.append(t)
    monkeypatch.setattr(p,'certify_moves',certify)
    p.certify_complete_plan([])
    assert visited==[7.,9.,31.] and p.candidate_deadline==8. and not p.certifying_complete_plan
    with pytest.raises(ap.FastPassTimeout):p.progress()


def test_global_deadline_and_collision_still_reject_during_certification(monkeypatch):
    s=SceneContext();p=ap.Planner(s,s.scene,s.robot(),np.zeros((16,3)),np.zeros((16,3)))
    p.search_active=True;p.started=0.;p.ik_mode='fast';p.candidate_deadline=8.
    monkeypatch.setattr(ap.time,'monotonic',lambda:91.)
    monkeypatch.setattr(p,'certify_moves',lambda _:p.progress())
    with pytest.raises(ap.PlanningTimeout):p.certify_complete_plan([])
    assert p.candidate_deadline==8. and not p.certifying_complete_plan
    def reject(_):raise ap.NativePathRefusal('real collision')
    monkeypatch.setattr(p,'certify_moves',reject)
    with pytest.raises(ap.NativePathRefusal):p.certify_complete_plan([])
    assert not p.certifying_complete_plan


def test_clear_corridor_layout_prefers_nearer_side_without_hardcoding_image_right():
    s=SceneContext();scene=s.scene
    for c,p in zip(('red','orange','green'),((.25,-.15,.080),(.25,-.30,.080),(.35,-.18,.078))):
        scene['blocks'][c]['top_center']['surface_position_m']=list(p)
    layouts=ap.layout_candidates(scene,s.robot())
    assert next(row for row in layouts if row[0]==[.25,-.15])[1]==-1
    # Far-source corridor ranking may translate the assembly, but preserves
    # both stationary alternatives and their nearer-side preference.
    assert ([.25,-.15],1) in layouts
    # Reflect the scene: same short-path preference now chooses the other side.
    for block in scene['blocks'].values():block['top_center']['surface_position_m'][1]=-.3-block['top_center']['surface_position_m'][1]
    reflected=ap.xyz(scene,'red')[:2].tolist()
    assert next(row for row in ap.layout_candidates(scene,s.robot()) if row[0]==reflected)[1]==1
