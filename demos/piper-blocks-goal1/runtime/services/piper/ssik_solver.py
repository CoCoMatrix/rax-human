"""Pure SSIK adapter: identical r9 math contract; never accesses hardware."""
from pathlib import Path
import sys,json,time,uuid,threading,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
PKG=Path(__file__).resolve().with_name('ssik_vendor')
sys.path.insert(0,str(PKG))
import ssik
from ssik.subproblems import sp6
from .geometry import ASSETS,Kinematics,vector
from .moveit import IKNoSolution
from .ssik_rank1_patch import rank_one_conic
sp6.solve_two_ellipse_numeric=rank_one_conic

class SSIK:
 name="ssik_rank1"
 def __init__(self,log_root):
  self.log_root=Path(log_root);self.kinematics=Kinematics();self.arm=None
  self.lock=threading.Lock();self.records=[]
 def start(self):
  if self.arm is not None:return
  self.log_root.mkdir(parents=True,exist_ok=True)
  manifest=json.loads(Path(__file__).with_name('ssik_vendor_manifest.json').read_text())
  for rel,sha in manifest.items():
   if hashlib.sha256((PKG/rel).read_bytes()).hexdigest()!=sha:raise ValueError('SSIK dependency identity changed: '+rel)
  self.arm=ssik.Manipulator.from_urdf(ASSETS/'dual_piper_x.urdf',base='left_base_link',ee='left_link6')
  assert len(self.arm._kb.joints)==6
  for q in ([0.]*6,[.2,.5,-.5,.3,.4,.1]):
   assert np.max(np.abs(self.arm.fk(q)-self.kinematics.fk(q)))<1e-10
  (self.log_root/'identity.json').write_text(json.dumps({'solver':'ssik_rank1','version':'8.2.0','base':'left_base_link','tip':'left_link6','urdf_sha256':hashlib.sha256((ASSETS/'dual_piper_x.urdf').read_bytes()).hexdigest(),'adapter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'patch_sha256':hashlib.sha256(Path(__file__).with_name('ssik_rank1_patch.py').read_bytes()).hexdigest(),'branch_selection':'minimum ordinary bounded-joint squared distance to explicit seed, lexicographic tie break; no modulo wrapping','fallback_to_KDL':False},indent=2))
 def solve(self,position,quaternion_xyzw,seed):return self._solve(position,quaternion_xyzw,seed,.1)
 def solve_fast(self,position,quaternion_xyzw,seed):return self._solve(position,quaternion_xyzw,seed,.005)
 def _solve(self,position,quaternion_xyzw,seed,timeout_s):
  p=vector(position,3);quat=vector(quaternion_xyzw,4);q=self.kinematics.check(seed)
  if abs(np.linalg.norm(quat)-1)>1e-6:raise ValueError('Unit quaternion required')
  T=np.eye(4);T[:3,:3]=Rotation.from_quat(quat).as_matrix();T[:3,3]=p
  with self.lock:
   self.start();t0=time.perf_counter()
   sols=self.arm.solve(T,q_seed=q.tolist(),allow_refinement=True)
   candidates=[]
   for s in sols:
    target=self.kinematics.check(s.q.tolist());F=self.kinematics.fk(target.tolist())
    pe=float(np.linalg.norm(F[:3,3]-p));ae=float(Rotation.from_matrix(T[:3,:3]@F[:3,:3].T).magnitude())
    # A malformed solver result is an integration error, not a silently dropped branch.
    if pe>.0002 or ae>.002:raise ValueError('Independent FK verification failed')
    candidates.append((float(np.sum((target-q)**2)),tuple(target),target,pe,ae))
   candidates.sort(key=lambda c:(c[0],c[1]))
   dt=time.perf_counter()-t0
   rec={'elapsed_s':dt,'budget_s':timeout_s,'branches':len(candidates),'over_budget':dt>timeout_s,'ok':bool(candidates) and dt<=timeout_s}
   if candidates:
    best=candidates[0];rec.update(seed_distance_rad=float(np.sqrt(best[0])),max_seed_delta_rad=float(np.max(np.abs(best[2]-q))),second_distance_rad=float(np.sqrt(candidates[1][0])) if len(candidates)>1 else None)
   self.records.append(rec)
   if not rec['ok']:raise IKNoSolution('SSIK_BUDGET_EXCEEDED' if rec['over_budget'] else 'SSIK_NO_IK_SOLUTION')
   return {'ok':True,'solver':'ssik_rank1','hardware_writes':0,'id':uuid.uuid4().hex,'elapsed_s':dt,'q':best[2].tolist(),'position_error_m':best[3],'orientation_error_rad':best[4],'executed':False,'scene_collision_checked':False,'search_budget_s':timeout_s,'controlled_link':'left_link6','frame':'left_base_link','approximate':False,'branch_count':len(candidates),'selected_seed_distance_rad':float(np.sqrt(best[0]))}
 def close(self):
  self.log_root.mkdir(parents=True,exist_ok=True)
  (self.log_root/'SSIK-QUERIES.json').write_text(json.dumps(self.records))
