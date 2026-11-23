"""Installed Piper sample check with vectorized AABB broad phase.

Port of our existing legacy CollisionChecker.check, preserving pair order,
mesh/FCL narrow phase, margin validation and opposing-jaw exceptions.
No runtime patching of the legacy checker or vendor SDK. Native path sampling
remains inherited unchanged from the pinned legacy checker.
"""
import math
import numpy as np
import fcl
ORDER=tuple([f'left_joint{i}' for i in range(1,7)]+['left_gripper']+[f'right_joint{i}' for i in range(1,7)]+['right_gripper'])

def sampled_check(self,q, *, margin_m=.003, obstacles=()):
    if type(margin_m) not in (float,int) or not math.isfinite(margin_m) or not .001<=margin_m<=.05: raise ValueError('margin 0.001..0.05m required')
    state=self._state(q)
    self.model.update_cfg(dict(zip(ORDER,state)))
    bounds={}
    for name,obj in self.objects.items():
        T,_=self.scene.graph[name]
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
    for index in nearby:
        a,b=pairs[index]
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
