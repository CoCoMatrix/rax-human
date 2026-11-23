"""Scheduling and IK transport invariants; no hardware needed."""
from types import SimpleNamespace
import numpy as np
import pytest
import assembly_plan as a
from test_c_four_search import planner
from native_fk.geometry import URDF_SHA


def test_seed_bank_exact_dedup_nearest_and_invalidation():
 p=planner();assert p.nearest_seed([0,0,0]) is None
 p.remember_solution([.1,0,0],[1.]*6);p.remember_solution([.1,0,0],[1.]*6)
 assert len(p.seed_bank)==1 and p.nearest_seed([.11,0,0])==[1.]*6
 p.remember_solution([.11,0,0],[2.]*6)
 assert p.nearest_seed([.11,0,0])==[2.]*6
 p.remember_solution([.11+1e-12,0,0],[3.]*6)
 assert len(p.seed_bank)==3 # do not round away geometry or seeds


def test_solve_batches_original_and_fallbacks_and_reuses_exact_results():
 p=planner();calls=[];seed=[0.]*6+[.051]+[0.]*7
 def value(tool,**kw):
  assert tool=='piper_solve_ik_candidates';calls.append(kw)
  return {'executed':False,'hardware_writes':0,'frame':'left_base_link','controlled_link':'left_link6','urdf_sha256':URDF_SHA,
   'ok':True,'attempts':[{'seed':kw['seeds'][0],'ok':False,'reason':'MOVEIT_NO_IK_SOLUTION'},
                       {'seed':kw['seeds'][1],'ok':True,'q':[.2]*6}]}
 p.ctx=SimpleNamespace(_blocks_batch_ik=True,value=value)
 result=p.solve([.2,-.1,.25],a.quat(180),seed)
 assert result==[.2]*6+seed[6:] and len(calls)==1
 assert calls[0]['seeds'][0]==seed[:6]
 assert p.nearest_seed([.2,-.1,.25])==[.2]*6
 assert p._solve_candidates([.2,-.1,.25],a.quat(180),calls[0]['seeds'],seed[6:])==result
 assert len(calls)==1


def test_batch_reply_identity_error_not_treated_as_no_solution():
 p=planner();p.ctx=SimpleNamespace(_blocks_batch_ik=True,value=lambda *a,**kw:{'executed':True,'ok':True})
 with pytest.raises(ValueError,match='model/frame'):p.solve([.2,-.1,.25],a.quat(180),[0.]*14)


def test_near_complete_priority_is_bounded_and_keeps_other_layouts():
 p=planner();p.search_active=False;p.progress=lambda **kw:None
 p.layout_candidates=[];p.layout_pending=[];p.layout_cursor=0;p.layout_resume_count=0
 p.ik_mode='full';order=[]
 def gen(name,completed):
  for i in range(7):
   order.append(name);yield {'phase':'hover','completed':completed}
  return {'ok':True}
 p.layout_queue=[{'generator':gen('early',0),'completed':0,'resumes':0,'anchor':[.2,-.1],'side':1},
                 {'generator':gen('near',3),'completed':3,'resumes':0,'anchor':[.2,-.2],'side':1}]
 p._plan([0.]*14)
 assert order[:3]==['near']*3
 assert 'early' in order[:5] # bounded preference, no starvation


def test_exact_pose_reuse_does_not_reuse_neighbors_or_failure_or_tail():
 p=planner();p.search_active=True;p.progress=lambda **kw:None;p.pose_cache_disabled=False
 point=[.2,-.1,.25];rot=a.quat(180);p.remember_solution(point,[.1]*6,rot)
 p.ctx=SimpleNamespace(_blocks_batch_ik=True)
 calls=[]
 p._solve_candidates=lambda *args,**kw:calls.append(args) or [1.]*6+list(args[3])
 seed=[.12]*6+[.073]+[0.]*7
 assert p.solve(point,rot,seed)==[.1]*6+seed[6:] and not calls
 p.solve([.2+1e-8,-.1,.25],rot,seed);assert len(calls)==1
 p.pose_cache_disabled=True;p.solve(point,rot,seed);assert len(calls)==2
 p.pose_cache_disabled=False;p.solve(point,rot,[1.]*6+seed[6:]);assert len(calls)==3


def test_cached_pose_collision_retries_original_seed_without_disabling_checks():
 p=planner();p.search_active=True;p.progress=lambda **kw:None
 point=[.2,-.1,.25];rot=a.quat(180);seed=[0.]*14
 p.remember_solution(point,[.1]*6,rot);checked=[]
 p.ctx=SimpleNamespace(_blocks_batch_ik=True)
 p._solve_candidates=lambda *args,**kw:[.15]*6+list(args[3])
 def check(start,end,scene,**kw):
  checked.append(end[0])
  if end[0]==.1:raise a.NoPlan('cached solution collides')
  return {'native':{'cad_clear':True,'workspace_clear':True}}
 p.check=check
 parts,end=p.connect_edge(seed,point,rot,p.scene)
 assert end[:6]==[.15]*6 and p.pose_solution_retries==1
 assert .1 in checked and .15 in checked and not p.pose_cache_disabled


def test_cached_alternative_does_not_skip_unresolved_primary_seed():
 p=planner();pos=[.2,-.1,.25];rot=a.quat(180);first=[0.]*6;second=[.3]*6
 p.ik_cache[('full',tuple(pos+rot+second))]=[.4]*6
 calls=[]
 def value(tool,**kw):
  calls.append(kw['seeds'])
  return {'executed':False,'hardware_writes':0,'frame':'left_base_link','controlled_link':'left_link6','urdf_sha256':URDF_SHA,
   'ok':False,'attempts':[{'seed':first,'ok':False,'reason':'MOVEIT_NO_IK_SOLUTION'}]}
 p.ctx=SimpleNamespace(value=value)
 assert p._solve_candidates(pos,rot,[first,second],[.073])==[.4]*6+[.073]
 assert calls==[[first]]
