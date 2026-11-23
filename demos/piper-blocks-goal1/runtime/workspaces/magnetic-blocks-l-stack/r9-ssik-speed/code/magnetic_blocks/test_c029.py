import copy
from unittest.mock import patch
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import assembly_plan as a
from test_c034 import planner


def test_seed_alternatives_are_distinct_bounded_and_preserve_tail():
 p=planner();seen=[]
 def solve(pos,q,seed):
  seen.append(seed)
  if len(seen)<3:raise a.IKUnavailable('local minimum')
  return seed
 p._solve_single=solve
 tail=[.073]+[0.]*6+[.002]
 result=p.solve([.3,-.2,.24],a.quat(180),[0.]*6+tail)
 assert len(seen)==3 and len({tuple(s[:6]) for s in seen})==3
 assert result[6:]==tail


def test_solver_transport_error_is_not_masked_by_seed_fallback():
 p=planner();p._solve_single=lambda *x:(_ for _ in ()).throw(RuntimeError('solver pipe closed'))
 with pytest.raises(RuntimeError,match='pipe closed'):p.solve([.3,0,.2],a.quat(180),[0.]*14)


def test_grasp_family_keeps_tcp_center_and_bounded_tilt():
 p=planner();top=a.xyz(p.scene,'green');rows=list(a.grasp_variants(p.scene,'green'))
 assert any(r['tilt_degrees']!=0 for r in rows)
 assert {r['height_delta_m'] for r in rows}=={0.,-.006,.006}
 for r in rows:
  R=Rotation.from_quat(r['quaternion']);tcp=np.asarray(r['position'])+R.apply([0,0,.140])
  np.testing.assert_allclose(tcp,top+[0,0,-.012+r['height_delta_m']],atol=1e-12)
  assert abs(r['tilt_degrees'])<=5
  assert np.linalg.norm(np.asarray(r['position'])[:2]-top[:2])<.015


def test_tilted_grasp_quarter_turn_preserves_upright_cube_and_target():
 p=planner();src=a.xyz(p.scene,'red');r=next(r for r in a.grasp_variants(p.scene,'red') if r['tilt_degrees']==-5)
 Rg=Rotation.from_quat(r['quaternion']).as_matrix();pg=np.asarray(r['position']);target=np.array([.25,-.12,.08])
 for degrees in (0,90,-90,180):
  Rz=Rotation.from_euler('z',degrees,degrees=True).as_matrix();Rp=Rz@Rg;local=Rg.T@(src-pg)
  position=target-Rp@local
  np.testing.assert_allclose(position+Rp@local,target,atol=1e-12)
  np.testing.assert_allclose(Rp@Rg.T,Rz,atol=1e-12)


def test_stationary_red_needs_no_grasp_query_or_red_motion():
 p=planner()
 for c,pos in {'red':[.3,-.2,.078],'orange':[.3,0,.078],'green':[.17,-.05,.078]}.items():p.scene['blocks'][c]['top_center']['surface_position_m']=pos
 p.check=lambda *x,**kw:{};p.fk=lambda *x:{'position':[.1,0,.22]};queried=[];moved=[]
 def solve(pos,q,seed):
  queried.append(pos)
  if np.linalg.norm(np.asarray(pos)-(a.xyz(p.scene,'red')+[0,0,.128]))<1e-6:raise a.IKUnavailable('red cannot be grasped')
  return seed.copy()
 def transfer(start,scene,color,target):
  moved.append(color);out=copy.deepcopy(scene);out['blocks'][color]['top_center']['surface_position_m']=target
  return {'color':color,'actions':[],'end14':start,'scene_after':out}
 p.solve=solve;p.transfer=transfer;p.certify_complete_plan=lambda moves:None
 anchor=a.xyz(p.scene,'red')[:2].tolist()
 with patch.object(a,'layout_candidates',return_value=[(anchor,1)]):result=p._plan([0.]*14)
 assert result['ok'] and 'red' not in moved and result['order']==['orange','green']
 assert queried
