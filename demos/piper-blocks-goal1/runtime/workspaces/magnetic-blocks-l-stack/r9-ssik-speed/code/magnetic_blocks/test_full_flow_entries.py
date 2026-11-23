"""Real Execution entry plumbing with synthetic transport, never hardware.
Geometry/recognition are explicit doubles; recorded production replay is separate.
"""
import copy,time
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
import assembly_controller as ac
import assembly_plan as ap
import ordinary_checks as ordinary
import scene_checks as sc
from observations import identity
from test_composition import SceneContext
from test_empty_transition import instrument

@pytest.fixture
def env(monkeypatch):
 f=SceneContext();instrument(f)
 f.scene['blocks']['yellow']=copy.deepcopy(f.scene['blocks']['green'])
 f.scene['blocks']['yellow']['top_center']['surface_position_m']=[.28,-.36,.078]
 f.q[:6]=f.encode([.303,-.155,.206],[1.,0.,0.,0.]);f.q[6]=.0725
 obs0=f.obs
 def obs():
  o=obs0();o['captured_at_ns']=time.time_ns();return o
 f.obs=obs
 value0=f.value
 def value(name,**kw):
  assert name!='gemini_generate','No visual model permitted in normal flow'
  r=value0(name,**kw)
  if name=='piper_gripper_stroke' and 'contact_range_m' in kw:r['contact_range_m']=kw['contact_range_m']
  return r
 f.value=value
 geo=Mock();monkeypatch.setattr(sc,'scene_geometry',geo);monkeypatch.setattr(ac,'scene_geometry',geo)
 monkeypatch.setattr(ordinary,'visible_near_point',lambda *args:{'visible':True})
 monkeypatch.setattr(ordinary,'placed_region_evidence',lambda *args:{'visible':True,'head':{'visible':True},'wrist':{},'wrist_pair':{}})
 displacement=Mock(return_value={'available':True});monkeypatch.setattr('history_scene.reject_visible_displacement',displacement)
 planner=SimpleNamespace(check=Mock(return_value={'offline_geometry_double':True}))
 scene=ap.initial_scene(f.scene);e=ac.Execution(f,planner,{'source_scene':scene,'source_digest':'synthetic'},f.robot(),f.obs())
 return SimpleNamespace(f=f,e=e,geometry=geo,displacement=displacement,planner=planner)

def jaw(v,color,stage,width):
 q=v.f.q.copy();return dict(kind='jaw',stage=color+'-'+stage,color=color,start14=q,width_m=width)

def set_color(v,color):
 p=v.e.scene['blocks'][color]['top_center']['surface_position_m'];v.f.q[:6]=v.f.encode([p[0],p[1],p[2]+.128],[1.,0.,0.,0.]);v.f.q[6]=.0725
 v.e.expected=list(v.f.q);v.e.initial=list(v.f.q);v.e.obs=v.f.obs()

def arm(v,color,stage,pose,**extra):
 q=v.f.q.copy();target=v.f.encode(pose,[1.,0.,0.,0.])+q[6:]
 return dict(kind='arm',stage=color+'-'+stage,color=color,start14=q,target14=target,payload=v.e.payload,**extra)

@pytest.mark.parametrize('color',ap.COLORS)
def test_close_lower_release_retreat_chain(env,color):
 v=env;set_color(v,color)
 close=jaw(v,color,'close',.05);close['start14'][6]=.073
 r=v.e.run(close);assert r['ok'],r['why'];assert v.e.held==color
 # One shared geometry identity check and one measured FK for alignment;
 # the second measured FK constructs the grasp relation, a different purpose.
 assert v.geometry.call_count==1
 assert sum(t=='piper_fk' for t,_ in v.f.calls)==2
 assert v.displacement.call_args.kwargs['associated_colors']==(color,)
 p=v.e.scene['blocks'][color]['top_center']['surface_position_m'];support=.018 if color in ('red','orange') else v.e.scene['blocks'][ap.SUPPORTS[color]]['top_center']['surface_position_m'][2]
 lower=arm(v,color,'lower',[p[0],p[1],support+.06+.128],contact=True,support=ap.SUPPORTS.get(color))
 r=v.e.run(lower);assert r['ok'],r['why'];assert v.e.support['ok']
 before=len(v.f.calls)
 release=jaw(v,color,'release',.073);r=v.e.run(release);assert r['ok'],r['why']
 assert r['facts'][0]['reused_support'] and v.e.held is None
 calls=v.f.calls[before:];assert not any(t=='piper_observe' for t,_ in calls)
 motion=next(k for t,k in calls if t=='piper_gripper_stroke');assert motion['observe_after'] is False
 assert r['measured']['control_feedback_only']
 point=v.e.scene['blocks'][color]['top_center'];assert point['release_request_id'] and point['support_frame'] and point['action_frame'] is None
 retreat=arm(v,color,'retreat',[p[0],p[1],support+.06+.188],withdraw=True)
 r=v.e.run(retreat);assert r['ok'],r['why'];assert r['facts'][0]['ok']
 assert v.f.calls[-1][0]!='gemini_generate'

@pytest.mark.parametrize('invalid',['age','request','state','owner','color'])
def test_changed_support_does_not_reuse_cached_positive(env,invalid):
 v=env;set_color(v,'orange');assert v.e.run(jaw(v,'orange','close',.05))['ok']
 p=v.f.fk(v.f.q[:6])['position'];assert v.e.run(arm(v,'orange','lower',p,contact=True))['ok']
 fact=v.e.support
 if invalid=='age':fact['verified_monotonic']-=31
 elif invalid=='request':fact['completed_endpoint']['request_id']='other'
 elif invalid=='state':fact['completed_endpoint']['measured14'][0]+=.02
 elif invalid=='owner':fact['completed_endpoint']['owner_identity']=None
 else:fact['completed_endpoint']['color']='green'
 before=len(v.f.calls);r=v.e.run(jaw(v,'orange','release',.073));assert r['ok'],r['why']
 assert r['facts'][0]['reused_support'] is False
 assert sum(t=='piper_observe' for t,_ in v.f.calls[before:])==1

@pytest.mark.parametrize('case',['missing_support','no_held_object'])
def test_release_never_moves_without_required_predecessor(env,case):
 v=env;v.f.q[6]=.05;v.e.expected=v.f.q.copy();v.e.held='orange' if case=='missing_support' else None
 v.e.support=None if case=='missing_support' else {'ok':True}
 r=v.e.run(jaw(v,'orange','release',.073));assert not r['ok']
 assert not any(t=='piper_gripper_stroke' for t,_ in v.f.calls)

def test_invalid_cached_close_observation_refreshes_once(env):
 v=env;v.e.obs['measured14'][0]+=.02
 r=v.e.run(jaw(v,'orange','close',.05));assert r['ok'],r['why']
 assert sum(t=='piper_observe' for t,_ in v.f.calls)==1

def test_release_transport_failure_never_replays_or_relabels_success(env):
 v=env;assert v.e.run(jaw(v,'orange','close',.05))['ok'];p=v.f.fk(v.f.q[:6])['position']
 assert v.e.run(arm(v,'orange','lower',p,contact=True))['ok']
 v.f.fail_motion=lambda *args:True;before=len(v.f.calls)
 r=v.e.run(jaw(v,'orange','release',.073));assert not r['ok'] and r['control_uncertain']
 assert sum(t=='piper_gripper_stroke' for t,_ in v.f.calls[before:])==1
 assert v.e.held=='orange'

def test_recheck_failure_does_not_open_gripper(env,monkeypatch):
 v=env;assert v.e.run(jaw(v,'orange','close',.05))['ok'];p=v.f.fk(v.f.q[:6])['position']
 assert v.e.run(arm(v,'orange','lower',p,contact=True))['ok'];v.e.support['verified_monotonic']-=31
 monkeypatch.setattr(ordinary,'placement',lambda *a,**k:{'ok':False,'why':'Unresolved current support'})
 before=len(v.f.calls);r=v.e.run(jaw(v,'orange','release',.073));assert not r['ok']
 assert not any(t=='piper_gripper_stroke' for t,_ in v.f.calls[before:])

def test_retreat_image_must_match_its_completed_receipt(env,monkeypatch):
 v=env;assert v.e.run(jaw(v,'orange','close',.05))['ok'];p=v.f.fk(v.f.q[:6])['position']
 assert v.e.run(arm(v,'orange','lower',p,contact=True))['ok'];assert v.e.run(jaw(v,'orange','release',.073))['ok']
 value=v.f.value
 def inconsistent(name,**kw):
  r=value(name,**kw)
  if name=='piper_move_joints' and r.get('observation'):r['observation']['measured14'][0]+=.02
  return r
 v.f.value=inconsistent
 r=v.e.run(arm(v,'orange','retreat',[p[0],p[1],p[2]+.06],withdraw=True))
 assert not r['ok'] and r['executed'] and 'does not match completed receipt' in r['why']
 assert sum(t=='piper_move_joints' for t,_ in v.f.calls)==2  # lower then retreat, never replay
