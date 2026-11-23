import numpy as np,pytest
import assembly_plan as a
from test_c_four_search import planner

def test_source_pruning_uses_minimum_not_preference():
 s=planner().scene;orange=a.xyz(s,'orange')
 s['blocks']['green']['top_center']['surface_position_m']=(orange+[.15,0.,0.]).tolist()
 s['blocks']['red']['top_center']['surface_position_m']=(orange+[0,.092,0]).tolist()
 assert not a.pickup_source_conflicts(s,'orange')
 s['blocks']['red']['top_center']['surface_position_m']=(orange+[0,.088,0]).tolist()
 assert not a.pickup_source_conflicts(s,'orange')
 s['blocks']['red']['top_center']['surface_position_m']=(orange+[0,.0851,0]).tolist()
 assert not a.pickup_source_conflicts(s,'orange')
 s['blocks']['red']['top_center']['surface_position_m']=(orange+[0,.0849,0]).tolist()
 assert a.pickup_source_conflicts(s,'orange')[0]['required_m']==pytest.approx(.015)

def route_planner(prefer=True):
 p=a.Planner.__new__(a.Planner);p.search_active=prefer;p.robot={}
 p.fk=lambda q:{'position':[q[0],q[1],-q[2]],'quaternion_xyzw':[1.,0.,0.,0.]}
 p.solve=lambda xyz,q,seed:[xyz[0],xyz[1],-xyz[2]]+[0.]*3+seed[6:]
 def check(start,end,*args,**kw):
  delta=np.asarray(end[:3])-start[:3]
  return {'min_payload_neighbor_m':.021 if abs(delta[1])>.001 and abs(delta[2])>.001 else .026}
 p.check=check;p.joint_route=lambda start,end,scene,**kw:[{'start14':start,'target14':end,'check':check(start,end)}]
 return p

def test_planning_prefers_more_clearance_without_shortcutting_it_away():
 p=route_planner();start=[.3,0.,-.25]+[0.]*11
 rows=p._route(start,[.3,.1,.3],[1,0,0,0],{},active='orange',payload={'color':'orange'})
 assert len(rows)==2 and min(r['check']['min_payload_neighbor_m'] for r in rows)>=.023

def test_runtime_accepts_direct_path_above_minimum_without_extra_candidates():
 p=route_planner(False);start=[.3,0.,-.25]+[0.]*11
 rows=p._route(start,[.3,.1,.3],[1,0,0,0],{},active='orange',payload={'color':'orange'})
 assert len(rows)==1 and rows[0]['check']['min_payload_neighbor_m']==.021

def test_no_preferred_candidate_returns_valid_checked_path():
 p=route_planner();start=[.3,0.,-.25]+[0.]*11
 p.check=lambda *args,**kw:{'min_payload_neighbor_m':.021}
 p.joint_route=lambda start,end,scene,**kw:[{'start14':start,'target14':end,'check':p.check(start,end)}]
 rows=p._route(start,[.3,.1,.3],[1,0,0,0],{},active='orange',payload={'color':'orange'})
 assert rows and min(r['check']['min_payload_neighbor_m'] for r in rows)==.021

def test_loaded_split_rejects_last_leg_before_any_motion(monkeypatch):
 from types import SimpleNamespace
 import assembly_batch as b
 q=[0.]*14
 actions=[{'kind':'arm','stage':'red-transfer','start14':q} for _ in range(3)]
 sent=[];checked=[]
 e=SimpleNamespace(ctx=None,guard=lambda:({},q),payload={'color':'red'},run=lambda *args,**kw:sent.append(args))
 monkeypatch.setattr(b.c,'_run',lambda ctx,stage,op:op({}))
 def preview(execution,group,anchor,**options):
  if len(group)>1:raise b.BlendGeometryRefusal('Rejected combined curve')
  checked.append(group[0])
  if len(checked)==3:raise b.BlendGeometryRefusal('Final loaded segment unavailable')
  return dict(target14=q,stages=['red-transfer'],check={'min_payload_neighbor_m':.020})
 monkeypatch.setattr(b,'preview',preview)
 with pytest.raises(b.BlendGeometryRefusal,match='Final loaded segment unavailable'):
  b.run(e,actions)
 assert len(checked)==3 and not sent

@pytest.mark.parametrize('j',range(6))
@pytest.mark.parametrize('side',[0,1])
def test_guard_all_six_joint_limits_and_allow_inward_motion(j,side):
 bounds=a.arm_bounds('left');mid=bounds.mean(axis=1);sign=1 if side else -1
 start=mid.copy();near=mid.copy();near[j]=bounds[j,side]-sign*.0004
 with pytest.raises(a.NoPlan,match='endpoint reserve'):a.check_command_reserve(start,near)
 with pytest.raises(a.NoPlan,match='endpoint reserve'):a.check_command_reserve(start,[start,near,start])
 a.check_command_reserve(near,near);a.check_command_reserve(near,start)
 measured=np.r_[near,[0.],mid,[0.]]
 measured[j]=bounds[j,side]+sign*.00035
 with pytest.raises(RuntimeError,match=f'left J{j+1} measured'):a.measured_limits(measured)
