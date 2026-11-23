"""goal1: Coal pre-filter in the native sampled collision check.

Coal is used only to skip pairs that are clearly farther apart than the margin (with 1 mm
slack). Every pair that could matter still gets the exact FCL distance, so contacts,
tolerated clearances, expected jaw contacts and the clear verdict are unchanged. Without the
vendored Coal, or with RAX_COAL_PREFILTER=0, the check runs exactly as before.
Run in the runtime root of the goal1 version.
"""
from pathlib import Path


def patch(path, pairs):
    p = Path(path); s = p.read_text()
    for old, new in pairs:
        assert s.count(old) == 1, (path, old[:80], s.count(old))
        s = s.replace(old, new)
    p.write_text(s)


patch('services/piper/installed_collision_math.py', [
    ("import fcl\n",
     "import fcl\n"
     "import os\n"
     "import sys\n"
     "from pathlib import Path\n"
     "\n"
     "COAL_SLACK_M=.001  # Coal and FCL distances differed by at most 0.1 mm over 608,916 recorded pairs.\n"
     "_VENDOR=Path(__file__).resolve().parents[2]/'vendor/coal-3.0.3/cmeel.prefix/lib/python3.12/site-packages'\n"
     "\n"
     "\n"
     "def load_coal():\n"
     "    \"\"\"Coal from this version's own vendor directory only; None means the plain FCL check.\"\"\"\n"
     "    if os.environ.get('RAX_COAL_PREFILTER','1')=='0' or not _VENDOR.is_dir():return None\n"
     "    if str(_VENDOR) not in sys.path:sys.path.append(str(_VENDOR))\n"
     "    try:\n"
     "        import coal\n"
     "    except ImportError:return None\n"
     "    return coal\n"
     "\n"
     "\n"
     "class CoalPrefilter:\n"
     "    \"\"\"Coal copies of the checker objects. Skips only pairs certainly farther than margin+slack.\"\"\"\n"
     "    def __init__(self,coal,checker):\n"
     "        self.coal=coal;self.objects={}\n"
     "        fingers=('left_gripper_link1','left_gripper_link2')\n"
     "        for node in checker.objects:\n"
     "            if checker.links[node] in fingers:\n"
     "                points=coal.StdVec_Vec3s()\n"
     "                for v in np.ascontiguousarray(checker.local_bounds[node][:,:3],float):points.append(v)\n"
     "                geometry=coal.ConvexBase.convexHull(points,True,'Qt')  # the same solid convex finger\n"
     "            else:\n"
     "                mesh=checker.scene.geometry[checker.scene.graph[node][1]]\n"
     "                v=np.asarray(mesh.vertices,float);f=np.asarray(mesh.faces,np.int32)\n"
     "                geometry=coal.BVHModelOBBRSS();geometry.beginModel(len(f),len(v));geometry.addVertices(v);geometry.addTriangles(f);geometry.endModel()\n"
     "            self.objects[node]=coal.CollisionObject(geometry,coal.Transform3s())\n"
     "\n"
     "    def sample(self,poses,boxes,margin_m):\n"
     "        return _CoalSample(self,poses,boxes,margin_m)\n"
     "\n"
     "\n"
     "class _CoalSample:\n"
     "    def __init__(self,prefilter,poses,boxes,margin_m):\n"
     "        coal=self.coal=prefilter.coal;self.objects=prefilter.objects;self.poses=poses;self.placed=set()\n"
     "        self.request=coal.CollisionRequest();self.request.security_margin=float(margin_m)+COAL_SLACK_M\n"
     "        self.env={name:coal.CollisionObject(coal.Box(*s),coal.Transform3s(np.eye(3),c)) for name,(c,s) in boxes.items()}\n"
     "\n"
     "    def _object(self,name):\n"
     "        if name in self.env:return self.env[name]\n"
     "        obj=self.objects[name]\n"
     "        if name not in self.placed:\n"
     "            T=self.poses[name];obj.setTransform(self.coal.Transform3s(T[:3,:3],T[:3,3]));self.placed.add(name)\n"
     "        return obj\n"
     "\n"
     "    def certainly_apart(self,a,b):\n"
     "        try:\n"
     "            result=self.coal.CollisionResult()\n"
     "            self.coal.collide(self._object(a),self._object(b),self.request,result)\n"
     "            return not result.isCollision()\n"
     "        except Exception:return False  # any doubt: compute the exact FCL distance\n"),
    ("    bounds={}\n    for name,obj in self.objects.items():\n        T,_=self.scene.graph[name]\n",
     "    bounds={};poses={};boxes={}\n    for name,obj in self.objects.items():\n        T,_=self.scene.graph[name];poses[name]=T\n"),
    ("        env[name]=fcl.CollisionObject(fcl.Box(*s),fcl.Transform(c.astype(float)))\n",
     "        env[name]=fcl.CollisionObject(fcl.Box(*s),fcl.Transform(c.astype(float)))\n        boxes[name]=(np.asarray(c,float),np.asarray(s,float))\n"),
    ("    for index in nearby:\n        a,b=pairs[index]\n        distance=fcl.distance(",
     "    prefilter=getattr(self,'_coal_prefilter',None)\n"
     "    if prefilter is not None:\n"
     "        try:prefilter=prefilter.sample(poses,boxes,margin_m)\n"
     "        except Exception:prefilter=None\n"
     "    for index in nearby:\n        a,b=pairs[index]\n"
     "        if prefilter is not None and prefilter.certainly_apart(a,b):continue  # farther than margin+1mm\n"
     "        distance=fcl.distance("),
])
patch('services/piper/installed_collision.py', [
    ("from .installed_collision_math import sampled_check\n",
     "from .installed_collision_math import sampled_check, load_coal, CoalPrefilter\n"),
    ("            self._right_pair_cache=None\n            self._check_lock=threading.RLock()\n",
     "            self._right_pair_cache=None\n            self._check_lock=threading.RLock()\n"
     "            # Optional speed-up; any problem building it leaves the plain FCL check.\n"
     "            coal=load_coal();self._coal_prefilter=None\n"
     "            if coal is not None:\n"
     "                try:self._coal_prefilter=CoalPrefilter(coal,self)\n"
     "                except Exception:self._coal_prefilter=None\n"),
])
print('coal prefilter patched')
