"""Sampled whole-robot CAD checks, no SDK. NOT continuous/physical certification."""
import hashlib
import json
import math
import numpy as np
import fcl
from yourdfpy import URDF
from .geometry import ASSETS, URDF_SHA, ORDER

class CollisionChecker:
    def __init__(self):
        path = ASSETS/'dual_piper_x_description/urdf/dual_piper_x.urdf'
        if hashlib.sha256(path.read_bytes()).hexdigest() != URDF_SHA:
            raise ValueError('URDF identity changed')
        manifest = ASSETS/'asset-manifest.json'
        if hashlib.sha256(manifest.read_bytes()).hexdigest() != '42d11a2998b7afc835f48a12c24fb7eb6b62f514fa2960fb46ce64967f228b1e':
            raise ValueError('Asset manifest changed')
        for item in json.loads(manifest.read_text())['files']:
            source=ASSETS/item['path']
            if not source.resolve().is_relative_to(ASSETS.resolve()) or hashlib.sha256(source.read_bytes()).hexdigest()!=item['sha256']:
                raise ValueError('Geometry asset SHA mismatch')
        self.model = URDF.load(str(path), load_meshes=False, load_collision_meshes=True, build_collision_scene_graph=True)
        self.scene = self.model.collision_scene
        self.limits = np.array([[self.model.joint_map[n].limit.lower,self.model.joint_map[n].limit.upper] for n in ORDER])
        self.objects, self.local_bounds, self.links = {}, {}, {}
        for node in self.scene.graph.nodes_geometry:
            _, mesh_name = self.scene.graph[node]
            mesh = self.scene.geometry[mesh_name]
            bvh = fcl.BVHModel()
            bvh.beginModel(len(mesh.vertices), len(mesh.faces))
            bvh.addSubModel(np.asarray(mesh.vertices,dtype=np.float64),np.asarray(mesh.faces,dtype=np.int32))
            bvh.endModel()
            self.objects[node] = fcl.CollisionObject(bvh)
            self.local_bounds[node] = np.array([[x,y,z,1] for x in mesh.bounds[:,0] for y in mesh.bounds[:,1] for z in mesh.bounds[:,2]])
            self.links[node] = self.scene.graph.transforms.parents[node]
        # Only same rigid assembly or actual parent-child joints: never learn a
        # collision exemption from a sampled posture (that could hide a real clash).
        parent = {l.name:l.name for l in self.model.robot.links}
        def root(n):
            while parent[n] != n: n=parent[n]
            return n
        adjacent=set()
        for j in self.model.robot.joints:
            adjacent.add(frozenset((j.parent,j.child)))
            if j.type == 'fixed': parent[root(j.child)] = root(j.parent)
        self.excluded = []
        self.pairs = []
        nodes=list(self.objects)
        for i,a in enumerate(nodes):
            for b in nodes[i+1:]:
                la,lb=self.links[a],self.links[b]
                attached_neighbour = any(j.type != 'fixed' and ((la == j.parent and root(lb) == root(j.child)) or (lb == j.parent and root(la) == root(j.child))) for j in self.model.robot.joints)
                reason = 'same fixed assembly' if root(la)==root(lb) else 'joint parent versus child rigid attachment' if attached_neighbour else 'direct joint neighbours' if frozenset((la,lb)) in adjacent else None
                if reason: self.excluded.append(dict(links=[la,lb],reason=reason))
                else: self.pairs.append((a,b))
        self.model_identity = URDF_SHA

    def _state(self,q):
        a=np.asarray(q)
        if a.shape!=(14,) or a.dtype.kind not in 'fi' or not np.isfinite(a).all(): raise ValueError('finite state14 required')
        if np.any(a<self.limits[:,0]) or np.any(a>self.limits[:,1]): raise ValueError('outside URDF limits')
        return a.astype(float)

    def check(self,q, *, margin_m=.003, obstacles=()):
        if type(margin_m) not in (float,int) or not math.isfinite(margin_m) or not .001<=margin_m<=.05: raise ValueError('margin 0.001..0.05m required')
        state=self._state(q)
        self.model.update_cfg(dict(zip(ORDER,state)))
        bounds={}
        for name,obj in self.objects.items():
            T,_=self.scene.graph[name]
            obj.setTransform(fcl.Transform(T[:3,:3],T[:3,3]))
            xyz=(self.local_bounds[name]@T.T)[:,:3]
            bounds[name]=np.array([xyz.min(0),xyz.max(0)])
        env={}
        for box in obstacles:
            if set(box)!={'name','center_m','size_m'} or not isinstance(box['name'],str) or box['name'] in env: raise ValueError('unique named axis-aligned boxes required')
            c,s=np.asarray(box['center_m']),np.asarray(box['size_m'])
            if c.shape!=(3,) or s.shape!=(3,) or c.dtype.kind not in 'fi' or s.dtype.kind not in 'fi' or not np.isfinite(c).all() or not np.isfinite(s).all() or np.any(s<=0): raise ValueError('invalid box')
            name='environment:'+box['name']
            if name in env: raise ValueError('duplicate obstacle')
            env[name]=fcl.CollisionObject(fcl.Box(*s),fcl.Transform(c.astype(float)))
            bounds[name]=np.array([c-s/2,c+s/2])
        objects={**self.objects,**env}; hits=[]; expected_contacts=[]
        pairs=self.pairs+[(a,b) for a in self.objects for b in env]
        for a,b in pairs:
            A,B=bounds[a],bounds[b]
            gap=np.maximum(0,np.maximum(A[0]-B[1],B[0]-A[1]))
            if np.linalg.norm(gap)>margin_m: continue
            distance=fcl.distance(objects[a],objects[b],fcl.DistanceRequest(),fcl.DistanceResult())
            if not math.isfinite(distance): raise ValueError('invalid collision distance')
            if distance<=margin_m:
                links=[self.links.get(a,a),self.links.get(b,b)]
                contact=dict(links=links,distance_m=max(0,float(distance)))
                side=next((s for s in ('left','right') if set(links)=={s+'_gripper_link1',s+'_gripper_link2'}),None)
                width=state[6 if side=='left' else 13] if side else None
                # Only the two opposing surfaces of the SAME jaw. Their CAD
                # distance must agree with its full-gap coordinate. Objects,
                # the other arm, fingers vs palm and cable remain unexempted.
                if side and abs(max(0,float(distance))-width)<1e-5:
                    expected_contacts.append(dict(**contact,rule='opposing fingers within commanded jaw width',jaw_width_m=float(width)))
                else: hits.append(contact)
        return dict(collision_checked=True,cad_clear=not hits,contacts=hits,expected_gripper_contacts=expected_contacts,margin_m=margin_m,
                    collision_objects=len(self.objects),checked_pairs=len(pairs),environment_boxes=len(env),
                    physical_calibration_validated=False,hardware_execution_approved=False,
                    limitation='CAD sampled check only; no cable, payload, unseen obstacle or continuous sweep certification')

    def path(self,samples, *, margin_m=.003, obstacles=()):
        a=np.asarray(samples)
        if a.ndim!=2 or a.shape[1]!=14 or not 1<=len(a)<=901: raise ValueError('path requires 1..901 state14')
        for q in a: self._state(q)
        expanded=[a[0]]
        for p,q in zip(a[:-1],a[1:]):
            n=max(1,int(np.ceil(np.max(abs(q-p)/np.array([.01]*6+[.001]+[.01]*6+[.001])))))
            if len(expanded)+n>4000: raise ValueError('sample budget exceeded')
            expanded.extend(p+(q-p)*i/n for i in range(1,n+1))
        for i,q in enumerate(expanded):
            result=self.check(q,margin_m=margin_m,obstacles=obstacles)
            if not result['cad_clear']:
                return dict(**result,first_failing_sample=i,checked_samples=i+1,total_samples=len(expanded),whole_path_checked=False)
        return dict(**result,checked_samples=len(expanded),total_samples=len(expanded),whole_path_checked=True)
