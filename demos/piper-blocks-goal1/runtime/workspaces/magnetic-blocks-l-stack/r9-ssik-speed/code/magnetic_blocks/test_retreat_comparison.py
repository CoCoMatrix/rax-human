"""Scoped AABB comparison tests; native responses are explicit test doubles."""
from types import SimpleNamespace
from unittest.mock import Mock
import copy
import numpy as np
import pytest
import assembly_plan as ap

def fixture(overlap=.0005,native_clear=True):
 scene={'table':{'z_ax_by_c':[0,0,0]},'blocks':{'green':{'top_center':{'surface_position_m':[.3,0,.06]},'height_m':.06}}}
 for i,color in enumerate(('red','orange','yellow')):scene['blocks'][color]={'top_center':{'surface_position_m':[.8,-.4+i*.2,.06]},'height_m':.06}
 # Fixed down-facing pair: simplified boxes overlap only along X by overlap.
 fingers=np.concatenate([ap.box_corners(np.array([sign*(.03-overlap),-.005,.13]),np.array([sign*.04,.005,.145])) if sign==1 else ap.box_corners(np.array([-.04,-.005,.13]),np.array([-.03+overlap,.005,.145])) for sign in (1,-1)])
 def fk(q):
  T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=[.3,0,.20+q[0]];return {'T_base_flange':T.tolist(),'position':T[:3,3].tolist()}
 calls=[]
 def value(name,**kw):
  calls.append((name,kw))
  if name=='piper_fk':return fk(kw['joints'])
  if name=='piper_fk_batch':return {'T_base_flange':[fk(q)['T_base_flange'] for q in kw['joints_batch']]}
  assert name=='piper_check_path'
  return {'cad_clear':native_clear,'workspace_clear':True,'contacts':[] if native_clear else ['native collision']}
 robot={'workspace_min':[-1,-1,0],'workspace_max':[1,1,1]}
 p=ap.Planner(SimpleNamespace(value=value),scene,robot,fingers,fingers)
 a=[0.]*14;a[6]=.073;b=a.copy();b[0]=.05
 return p,scene,a,b,calls

@pytest.mark.parametrize('overlap',[.0002,.0005,.000999])
def test_one_mm_scoped_projection_error_requires_native_clearance(overlap):
 p,s,a,b,calls=fixture(overlap)
 result=p.check(a,b,s,active='green',withdraw=True)
 fact=result['released_retreat_comparison'];assert fact['used_m']==pytest.approx(overlap)
 assert result['native']['cad_clear'] and fact['progress']['progressive_departure']
 native=next(kw for name,kw in calls if name=='piper_check_path')
 assert 'green' in [box['name'] for box in native['obstacles']]  # target never removed

def test_more_than_one_mm_still_refuses():
 p,s,a,b,_=fixture(.0011)
 with pytest.raises(ap.NoPlan,match='Finger/green'):p.check(a,b,s,active='green',withdraw=True)

def test_native_collision_is_never_suppressed():
 p,s,a,b,_=fixture(native_clear=False)
 with pytest.raises(ap.NativePathRefusal):p.check(a,b,s,active='green',withdraw=True)

@pytest.mark.parametrize('case',['ordinary','closed','other_object','jaw_motion'])
def test_allowance_does_not_leak_to_other_modes(case):
 p,s,a,b,calls=fixture();opts={'active':'green','withdraw':True}
 if case=='ordinary':opts['withdraw']=False
 elif case=='closed':a[6]=b[6]=.05
 elif case=='other_object':s['blocks']['orange']=copy.deepcopy(s['blocks']['green'])
 else:b[6]=.074
 with pytest.raises(ap.NoPlan):p.check(a,b,s,**opts)

@pytest.mark.parametrize('samples',[
 [[.03,-.0005],[.02,-.0005],[.05,.01]],
 [[.03,-.0005],[.04,-.0002]],
 [[.03,-.0005],[.03,.01]],
 [[.03,-.0005],[float('nan'),.01]],
])
def test_progress_and_clear_final_endpoint_are_required(samples):
 with pytest.raises(ap.NoPlan):ap.retreat_progress_evidence(samples)

from test_full_flow_entries import env,jaw,arm

def prepare_release(v):
 assert v.e.run(jaw(v,'orange','close',.05))['ok'];p=v.f.fk(v.f.q[:6])['position']
 assert v.e.run(arm(v,'orange','lower',p,contact=True))['ok']
 assert v.e.run(jaw(v,'orange','release',.073))['ok']
 return p

def test_execution_requires_release_for_same_object(env):
 v=env;p=v.f.fk(v.f.q[:6])['position'];a=arm(v,'orange','retreat',[p[0],p[1],p[2]+.06],withdraw=True)
 before=len(v.f.calls);r=v.e.run(a);assert not r['ok'] and 'completed open release' in r['why']
 assert not any(t=='piper_move_joints' for t,_ in v.f.calls[before:])

@pytest.mark.parametrize('failure',[False,True])
def test_unexecuted_retreat_can_replan_but_never_bypass_or_replay(env,failure):
 v=env;p=prepare_release(v);action=arm(v,'orange','retreat',[p[0],p[1],p[2]+.06],withdraw=True)
 begin=list(v.f.q);end=action['target14'];mid=((np.array(begin)+end)/2).tolist();calls=[0]
 def check(*args,**kwargs):
  calls[0]+=1
  if calls[0]==1:raise ap.NativePathRefusal('Original curve collides')
  return {'native':{'cad_clear':True},'offline_geometry_double':True}
 v.planner.check=Mock(side_effect=check);v.planner.fk=v.f.fk
 if failure:v.planner.route=Mock(side_effect=ap.NoPlan('No safe route'))
 else:v.planner.route=Mock(return_value=[{'start14':begin,'target14':mid,'check':{}},{'start14':mid,'target14':end,'check':{}}])
 before=len(v.f.calls);r=v.e.run(action)
 if failure:
  assert not r['ok'];assert not any(t=='piper_move_joints' for t,_ in v.f.calls[before:])
 else:
  assert r['ok'],r['why'];assert sum(t=='piper_move_joints' for t,_ in v.f.calls[before:])==2
  assert r['retreat_correction']['rejoins_original_endpoint'];assert v.f.q==end
