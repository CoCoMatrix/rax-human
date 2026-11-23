"""Installed Piper sample check with vectorized AABB broad phase.

Port of our existing legacy CollisionChecker.check, preserving pair order,
mesh/FCL narrow phase, margin validation and opposing-jaw exceptions.
No runtime patching of the legacy checker or vendor SDK. Native path sampling
remains inherited unchanged from the pinned legacy checker.
"""
import math
import numpy as np
import fcl
import hashlib
import json
import os
import sys
from pathlib import Path

COAL_SLACK_M=.001  # Coal and FCL distances differed by at most 0.1 mm over 608,916 recorded pairs.
_VENDOR=Path(__file__).resolve().parents[2]/'vendor/coal-3.0.3/cmeel.prefix/lib/python3.12/site-packages'


def load_coal():
    """Coal from this version's own vendor directory only; None means the plain FCL check."""
    if os.environ.get('RAX_COAL_PREFILTER','1')=='0' or not _VENDOR.is_dir():return None
    try:
        manifest=json.loads(Path(__file__).with_name('coal_vendor_manifest.json').read_text())
        root=_VENDOR.parents[3]  # vendor/coal-3.0.3
        present={str(f.relative_to(root)) for f in root.rglob('*') if f.is_file() and '__pycache__' not in f.parts}
        if not manifest or present!=set(manifest):return None
        for rel,digest in manifest.items():
            if hashlib.sha256((root/rel).read_bytes()).hexdigest()!=digest:return None
    except (OSError,ValueError):return None
    if str(_VENDOR) not in sys.path:sys.path.append(str(_VENDOR))
    try:
        import coal
    except ImportError:return None
    return coal


class CoalPrefilter:
    """Coal copies of the checker objects. Skips only pairs certainly farther than margin+slack."""
    def __init__(self,coal,checker):
        self.coal=coal;self.objects={}
        fingers=('left_gripper_link1','left_gripper_link2')
        for node in checker.objects:
            if checker.links[node] in fingers:
                points=coal.StdVec_Vec3s()
                for v in np.ascontiguousarray(checker.local_bounds[node][:,:3],float):points.append(v)
                geometry=coal.ConvexBase.convexHull(points,True,'Qt')  # the same solid convex finger
            else:
                mesh=checker.scene.geometry[checker.scene.graph[node][1]]
                v=np.asarray(mesh.vertices,float);f=np.asarray(mesh.faces,np.int32)
                geometry=coal.BVHModelOBBRSS();geometry.beginModel(len(f),len(v));geometry.addVertices(v);geometry.addTriangles(f);geometry.endModel()
            self.objects[node]=coal.CollisionObject(geometry,coal.Transform3s())

    def sample(self,poses,boxes,margin_m):
        return _CoalSample(self,poses,boxes,margin_m)


class _CoalSample:
    def __init__(self,prefilter,poses,boxes,margin_m):
        coal=self.coal=prefilter.coal;self.objects=prefilter.objects;self.poses=poses;self.placed=set()
        self.request=coal.CollisionRequest();self.request.security_margin=float(margin_m)+COAL_SLACK_M
        self.env={name:coal.CollisionObject(coal.Box(*s),coal.Transform3s(np.eye(3),c)) for name,(c,s) in boxes.items()}

    def _object(self,name):
        if name in self.env:return self.env[name]
        obj=self.objects[name]
        if name not in self.placed:
            T=self.poses[name];obj.setTransform(self.coal.Transform3s(T[:3,:3],T[:3,3]));self.placed.add(name)
        return obj

    def certainly_apart(self,a,b):
        try:
            result=self.coal.CollisionResult()
            self.coal.collide(self._object(a),self._object(b),self.request,result)
            return not result.isCollision()
        except Exception:return False  # any doubt: compute the exact FCL distance
ORDER=tuple([f'left_joint{i}' for i in range(1,7)]+['left_gripper']+[f'right_joint{i}' for i in range(1,7)]+['right_gripper'])

def sampled_check(self,q, *, margin_m=.003, obstacles=()):
    if type(margin_m) not in (float,int) or not math.isfinite(margin_m) or not .001<=margin_m<=.05: raise ValueError('margin 0.001..0.05m required')
    state=self._state(q)
    self.model.update_cfg(dict(zip(ORDER,state)))
    bounds={};poses={};boxes={}
    for name,obj in self.objects.items():
        T,_=self.scene.graph[name];poses[name]=T
        obj.setTransform(fcl.Transform(T[:3,:3],T[:3,3]))
        xyz=(self.local_bounds[name]@T.T)[:,:3]
        bounds[name]=np.array([xyz.min(0),xyz.max(0)])
    env={};env_tolerance={};env_comparison_error={}
    for box in obstacles:
        if not {'name','center_m','size_m'}<=set(box) or not set(box)<={'name','center_m','size_m','finger_clearance_tolerance_m','finger_clearance_comparison_error_m'} or not isinstance(box['name'],str) or box['name'] in env: raise ValueError('unique named axis-aligned boxes required')
        c,s=np.asarray(box['center_m']),np.asarray(box['size_m'])
        if c.shape!=(3,) or s.shape!=(3,) or c.dtype.kind not in 'fi' or s.dtype.kind not in 'fi' or not np.isfinite(c).all() or not np.isfinite(s).all() or np.any(s<=0): raise ValueError('invalid box')
        tolerance=box.get('finger_clearance_tolerance_m',0.)
        if type(tolerance) not in (float,int) or not math.isfinite(tolerance) or not 0<=tolerance<=.0025:raise ValueError('finger clearance tolerance must be0..2.5mm')
        comparison_error=box.get('finger_clearance_comparison_error_m',0.)
        if type(comparison_error) not in (float,int) or not math.isfinite(comparison_error) or not 0<=comparison_error<=.001:raise ValueError('finger comparison error must be0..1mm')
        name='environment:'+box['name']
        env_tolerance[name]=float(tolerance)
        env_comparison_error[name]=float(comparison_error)
        if name in env: raise ValueError('duplicate obstacle')
        env[name]=fcl.CollisionObject(fcl.Box(*s),fcl.Transform(c.astype(float)))
        boxes[name]=(np.asarray(c,float),np.asarray(s,float))
        bounds[name]=np.array([c-s/2,c+s/2])
    objects={**self.objects,**env}; hits=[]; expected_contacts=[];tolerated=[]
    pairs=self.pairs+[(a,b) for a in self.objects for b in env]
    names=list(bounds);indices={name:i for i,name in enumerate(names)}
    bounds_array=np.asarray([bounds[name] for name in names])
    pair_indices=np.asarray([(indices[a],indices[b]) for a,b in pairs],dtype=int)
    if len(pair_indices):
        A=bounds_array[pair_indices[:,0]];B=bounds_array[pair_indices[:,1]]
        gaps=np.maximum(0,np.maximum(A[:,0]-B[:,1],B[:,0]-A[:,1]))
        nearby=np.flatnonzero(np.linalg.norm(gaps,axis=1)<=margin_m)
    else:nearby=[]
    prefilter=getattr(self,'_coal_prefilter',None)
    if prefilter is not None:
        try:prefilter=prefilter.sample(poses,boxes,margin_m)
        except Exception:prefilter=None
    for index in nearby:
        a,b=pairs[index]
        if prefilter is not None and prefilter.certainly_apart(a,b):continue  # farther than margin+1mm
        distance=fcl.distance(objects[a],objects[b],fcl.DistanceRequest(),fcl.DistanceResult())
        if not math.isfinite(distance): raise ValueError('invalid collision distance')
        if distance<=margin_m:
            links=[self.links.get(a,a),self.links.get(b,b)]
            contact=dict(links=links,distance_m=max(0,float(distance)))
            declared=env_tolerance.get(b,0.) if self.links.get(a) in ('left_gripper_link1','left_gripper_link2') else 0.
            comparison_error=env_comparison_error.get(b,0.) if declared else 0.
            minimum=max(0.,max(.0005,margin_m-declared)-comparison_error)
            if declared and distance>0 and distance>=minimum:
                tolerated.append(dict(**contact,nominal_margin_m=margin_m,tolerance_m=declared,comparison_error_m=comparison_error,minimum_m=minimum))
                continue
            side=next((s for s in ('left','right') if set(links)=={s+'_gripper_link1',s+'_gripper_link2'}),None)
            width=state[6 if side=='left' else 13] if side else None
            # Only the two opposing surfaces of the SAME jaw. Their CAD
            # distance must agree with its full-gap coordinate. Objects,
            # the other arm, fingers vs palm and cable remain unexempted.
            if side and abs(max(0,float(distance))-width)<1e-5:
                expected_contacts.append(dict(**contact,rule='opposing fingers within commanded jaw width',jaw_width_m=float(width)))
            else: hits.append(contact)
    return dict(collision_checked=True,cad_clear=not hits,contacts=hits,expected_gripper_contacts=expected_contacts,margin_m=margin_m,
                tolerated_clearances=tolerated,collision_objects=len(self.objects),checked_pairs=len(pairs),environment_boxes=len(env),
                physical_calibration_validated=False,hardware_execution_approved=False,
                limitation='CAD sampled check only; no cable, payload, unseen obstacle or continuous sweep certification')
