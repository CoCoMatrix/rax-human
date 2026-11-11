"""Adapt the existing sampled collision checker to the installed LEFT fingers.

No SDK, owner startup or changes to vendor/legacy files. Keep all original robot
objects, pair exclusions, margins and path sampling. Replace only two fingers.
"""
import importlib
import copy
import sys
import types
import threading
from pathlib import Path
import numpy as np
from .finger_geometry import finger_envelopes, geometry_identity


def checker_class(base):
    class InstalledCollisionChecker(base):
        def __init__(self):
            import fcl
            from scipy.spatial import ConvexHull
            super().__init__()
            rows={f"left_gripper_link{r['finger']}":r for r in finger_envelopes([0.]*14)}
            replaced=[]
            world_flange=self.scene.graph['left_link6'][0]
            for node,link in self.links.items():
                if link not in rows:continue
                row=rows[link];world_mesh=self.scene.graph[node][0]
                flange_mesh=np.linalg.inv(world_flange)@world_mesh
                if not np.allclose(flange_mesh,row['T_flange_mesh'],atol=1e-8):
                    raise ValueError('Collision/inspection finger frames disagree')
                vertices=(np.linalg.inv(flange_mesh)@np.c_[row['corners_flange_m'],np.ones(8)].T).T[:,:3]
                hull=ConvexHull(vertices);faces=hull.simplices.copy()
                for face,plane in zip(faces,hull.equations):
                    a,b,c=vertices[face]
                    if np.dot(np.cross(b-a,c-a),plane[:3])<0:face[1],face[2]=face[2],face[1]
                # Solid convex volume, not a triangle soup: a small obstacle
                # wholly inside the envelope must also count as a collision.
                polygons=np.c_[np.full(len(faces),3),faces].astype(np.int32).ravel()
                solid=fcl.Convex(vertices,len(faces),polygons)
                self.objects[node]=fcl.CollisionObject(solid)
                self.local_bounds[node]=np.c_[vertices,np.ones(8)]
                replaced.append(link)
            if sorted(replaced)!=sorted(rows):raise ValueError('Expected exactly two LEFT finger meshes')
            self._right_pair_cache=None
            self._check_lock=threading.RLock()

        def check(self,q,*,margin_m=.003,obstacles=()):
            # The pinned dual-arm URDF has independent left/right chains. Exact
            # right-state equality means RIGHT-vs-RIGHT distances are identical.
            # Reuse those results only; still update every object, check all
            # moving/cross-arm/environment pairs and keep every path sample.
            with self._check_lock:
                state=self._state(q);pairs=self.pairs
                fixed=[(a,b) for a,b in pairs if self.links[a].startswith('right_') and self.links[b].startswith('right_')]
                fixed_set=set(fixed);identity=geometry_identity()
                key=(tuple(state[7:]),margin_m,identity,tuple(fixed),
                     tuple((n,id(self.objects[n])) for n in sorted({n for pair in fixed for n in pair})))
                cached=self._right_pair_cache
                reuse=cached is not None and cached['key']==key
                if reuse:self.pairs=[pair for pair in pairs if pair not in fixed_set]
                try:result=super().check(q,margin_m=margin_m,obstacles=obstacles)
                finally:self.pairs=pairs
                fixed_links={(self.links[a],self.links[b]) for a,b in fixed}
                if reuse:
                    order={(self.links[a],self.links[b]):i for i,(a,b) in enumerate(pairs)}
                    for field in ('contacts','expected_gripper_contacts'):
                        result[field]=sorted(result[field]+copy.deepcopy(cached[field]),
                            key=lambda x:order.get(tuple(x['links']),len(pairs)))
                    result['cad_clear']=not result['contacts']
                    result['checked_pairs']+=len(fixed)
                else:
                    self._right_pair_cache={'key':key,**{field:[copy.deepcopy(x) for x in result[field]
                        if tuple(x['links']) in fixed_links] for field in ('contacts','expected_gripper_contacts')}}
                result['geometry_identity']=identity
                result['environment_checked']=bool(obstacles)
                result['exact_static_pairs_reused']=len(fixed) if reuse else 0
                return result
    return InstalledCollisionChecker


def load_checker(legacy_root):
    """Isolated import of the pinned existing pure checker, without import shadowing."""
    root=Path(legacy_root).resolve()/ 'services/piper'
    name='_rax_legacy_piper_collision'
    if name in sys.modules and Path(sys.modules[name].__path__[0])!=root:
        raise ValueError('Legacy collision root changed in process')
    if name not in sys.modules:
        package=types.ModuleType(name);package.__path__=[str(root)]
        sys.modules[name]=package
    base=importlib.import_module(name+'.collision').CollisionChecker
    return checker_class(base)()
