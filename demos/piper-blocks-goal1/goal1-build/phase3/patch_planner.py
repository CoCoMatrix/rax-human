"""goal1 phase 3: make assembly_plan.py and numeric_policy.py follow the goal description. Run in the controller code dir."""
from pathlib import Path


def patch(path, pairs):
    p = Path(path); s = p.read_text()
    for old, new, *count in pairs:
        n = count[0] if count else 1
        assert s.count(old) == n, (path, old[:90], s.count(old))
        s = s.replace(old, new)
    p.write_text(s)


patch('numeric_policy.py', [
    ('import math\n', 'import math\nfrom task_spec import HIGH_COLORS\n'),
    ("    base=25.1 if color=='yellow' else math.degrees(math.acos(DOWN_COS))",
     "    base=25.1 if color in HIGH_COLORS else math.degrees(math.acos(DOWN_COS))  # third-layer block (yellow in the default goal)"),
])

NEIGHBOR_OLD = "neighbor=('red' if color=='orange' else 'orange' if color in ('red','green') else None)"
patch('assembly_plan.py', [
    ("from task_spec import COLORS, SUPPORTS, BASE_COLORS, VERSION, PREFLIGHT_BUDGET_S, FAST_PASS_BUDGET_S, LAYOUT_SLICE_FAST_S, LAYOUT_SLICE_FULL_S\n",
     "from task_spec import COLORS, SUPPORTS, BASE_COLORS, VERSION, PREFLIGHT_BUDGET_S, FAST_PASS_BUDGET_S, LAYOUT_SLICE_FAST_S, LAYOUT_SLICE_FULL_S\n"
     "import task_spec as TS\n"),
    # --- grasp families: the third-layer block, not the colour yellow ---
    (" if rad<.50 or color=='yellow':return", " if rad<.50 or color in TS.HIGH_COLORS:return"),
    (" src=xyz(scene,color)\n if color=='yellow':\n", " src=xyz(scene,color)\n if color in TS.HIGH_COLORS:\n"),
    # --- goal geometry ---
    ('''def goal_scene(scene,anchor,side=-1):
 s=copy.deepcopy(scene);red=np.asarray([*anchor,0.],float);red[2]=table(scene,red)+height(scene,'red')
 orange=red.copy();orange[1]+=side*SPACING;orange[2]=table(scene,orange)+height(scene,'orange')
 green=red.copy();green[2]+=height(scene,'green')
 yellow=green.copy();yellow[2]+=height(scene,'yellow')
 for color,p in zip(COLORS,(red,orange,green,yellow)):
  s['blocks'][color]['top_center']['surface_position_m']=p.tolist()
 return s

def assert_goal(scene):
 r,o,g=[xyz(scene,c) for c in ('red','orange','green')]
 if not (abs(o[0]-r[0])<1e-8 and abs(abs(o[1]-r[1])-SPACING)<1e-8) or not np.allclose(g[:2],r[:2],atol=1e-8):raise NoPlan('Goal structure/direction changed')
 if abs(g[2]-r[2]-height(scene,'green'))>1e-8:raise NoPlan('Green support relation changed')
 y=xyz(scene,'yellow')
 if not np.allclose(y[:2],g[:2],atol=1e-8) or abs(y[2]-g[2]-height(scene,'yellow'))>1e-8:raise NoPlan('Yellow must be supported on green')
''',
     '''def goal_scene(scene,anchor,side=-1):
 """Top-centre positions of the goal description with its anchor block at `anchor`.
 Table blocks take the measured table height at their own position; a stacked block
 sits on its support by its own measured height."""
 s=copy.deepcopy(scene);tops={}
 for color in TS.PLACE_ORDER:
  dx,dy=TS.offset_xy(color,side);p=np.asarray([*anchor,0.],float);p[0]+=dx;p[1]+=dy
  if color in SUPPORTS:p[2]=tops[SUPPORTS[color]][2]+height(scene,color)
  else:p[2]=table(scene,p)+height(scene,color)
  tops[color]=p
 for color in COLORS:
  s['blocks'][color]['top_center']['surface_position_m']=tops[color].tolist()
 return s

def assert_goal(scene):
 anchor=xyz(scene,TS.ANCHOR)
 def matches(side):
  return all(np.allclose(xyz(scene,c)[:2]-anchor[:2],TS.offset_xy(c,side),atol=1e-8) for c in COLORS)
 if not any(matches(side) for side in TS.SIDES):raise NoPlan('Goal structure/direction changed')
 for color,support in SUPPORTS.items():
  if abs(xyz(scene,color)[2]-xyz(scene,support)[2]-height(scene,color))>1e-8:raise NoPlan(color+' support relation changed')

def placed_neighbors(scene,color,target):
 """Goal neighbours that already stand one tight pitch from this block's target.
 Returns None, one colour, or a list of colours (the former single-neighbour values are unchanged)."""
 found=[];limit=.003+CORRIDOR_ERROR_M
 for other in TS.goal_neighbors(color):
  d=np.asarray(target[:2],float)-xyz(scene,other)[:2]
  for side in TS.SIDES:
   want=np.asarray(TS.offset_xy(color,side))-TS.offset_xy(other,side)
   if np.linalg.norm(want)<=SPACING+1e-9 and abs(d[0]-want[0])<=limit and abs(d[1]-want[1])<=limit:found.append(other);break
 return None if not found else found[0] if len(found)==1 else found

def is_neighbor(color,assembly_neighbor):
 return color==assembly_neighbor or isinstance(assembly_neighbor,(list,tuple)) and color in assembly_neighbor
'''),
    # --- orders ---
    ('''def ranked_orders(scene,target,start_xy):
 """Rank the existing valid base orders by blocking, work and empty travel.
 A heuristic only: every candidate still requires full geometry/IK validation.
 """
 rows=[]
 for order in (('red','orange','green','yellow'),('orange','red','green','yellow')):
''',
     '''def ranked_orders(scene,target,start_xy,orders=None):
 """Rank the valid layer-by-layer orders by blocking, work and empty travel.
 A heuristic only: every candidate still requires full geometry/IK validation.
 """
 rows=[]
 for order in (TS.task_orders(TS.SIDES[0]) if orders is None else orders):
'''),
    # --- layout candidates ---
    (" red=xyz(scene,'red');points=[xyz(scene,c)[:2] for c in COLORS]\n", " red=xyz(scene,TS.ANCHOR);points=[xyz(scene,c)[:2] for c in COLORS]\n"),
    ("  for side in (-1,1):\n   target=goal_scene(scene,a,side)\n", "  for side in TS.SIDES:\n   target=goal_scene(scene,a,side)\n"),
    ('''   green=xyz(scene,'green')[:2]
   separation=min(float(np.max(np.abs(xyz(target,c)[:2]-green))) for c in ('red','orange'))
   access_penalty=max(0.,.13-separation)*3.
''',
     '''   access_penalty=0.
   for upper in TS.SECOND_LAYER:
    green=xyz(scene,upper)[:2]
    separation=min(float(np.max(np.abs(xyz(target,c)[:2]-green))) for c in BASE_COLORS)
    access_penalty=max(access_penalty,max(0.,.13-separation)*3.)
'''),
    ("    for base in ('red','orange'):\n     separation=float(np.min(np.max(abs(corridor-xyz(target,base)[:2]),axis=1)))",
     "    for base in BASE_COLORS:\n     separation=float(np.min(np.max(abs(corridor-xyz(target,base)[:2]),axis=1)))"),
    ("   score=float(travel+reach_penalty+access_penalty+6*corridor_penalty-(.06 if stationary else 0.))\n",
     "   score=float(travel+reach_penalty+access_penalty+6*corridor_penalty-(.06 if stationary else 0.))+TS.ROTATION_PENALTY_M[side]\n"),
    (''' for side in (-1,1):
  a=xyz(scene,'orange')[:2]-[0,side*SPACING]; pair=(a.tolist(),side)
  if pair not in ranked and all(np.all(p-.030>=LAYOUT_MIN-LAYOUT_ERROR_M) and np.all(p+.030<=LAYOUT_MAX+LAYOUT_ERROR_M) for p in (a,xyz(scene,'orange')[:2])):ranked.append(pair)
''',
     ''' for side in TS.SIDES:
  for fixed in BASE_COLORS[1:]:
   a=xyz(scene,fixed)[:2]-TS.offset_xy(fixed,side); pair=(a.tolist(),side)
   if pair not in ranked and all(np.all(p-.030>=LAYOUT_MIN-LAYOUT_ERROR_M) and np.all(p+.030<=LAYOUT_MAX+LAYOUT_ERROR_M) for p in [a+TS.offset_xy(c,side) for c in BASE_COLORS]):ranked.append(pair)
'''),
    # --- deliberate close placement next to declared goal neighbours ---
    ('''     adjacent_assembly=bool(contact and c==assembly_neighbor and
         (active in ('red','orange') and c in ('red','orange') and c!=active
          or active=='green' and support=='red' and c=='orange'))
''',
     '''     adjacent_assembly=bool(contact and is_neighbor(c,assembly_neighbor) and c in TS.goal_neighbors(active))
'''),
    # --- grasp family order for the third-layer block ---
    ("   if color=='yellow':\n    # The fourth pickup starts after a stack retreat.", "   if color in TS.HIGH_COLORS:\n    # The fourth pickup starts after a stack retreat."),
    ("   ordered=[*preferred,None,*[v for v in variants if v not in preferred]] if color=='yellow' else [None,*preferred,*[v for v in variants if v not in preferred]]",
     "   ordered=[*preferred,None,*[v for v in variants if v not in preferred]] if color in TS.HIGH_COLORS else [None,*preferred,*[v for v in variants if v not in preferred]]"),
    # --- only the block placed last may skip the next-pickup check or end released in place ---
    ("    if color!='yellow':\n     # A nonfinal retreat must leave", "    if color!=TS.FINAL_COLOR:\n     # A nonfinal retreat must leave"),
    ("  if color==COLORS[-1] and color=='yellow' and mode=='full':", "  if color==TS.FINAL_COLOR and mode=='full':"),
    # --- neighbours at the three placement call sites ---
    ("       " + NEIGHBOR_OLD + "\n       if neighbor is not None and (abs(abs(target[1]-xyz(scene,neighbor)[1])-SPACING)>.003+CORRIDOR_ERROR_M or abs(target[0]-xyz(scene,neighbor)[0])>.003+CORRIDOR_ERROR_M):neighbor=None\n",
     "       neighbor=placed_neighbors(scene,color,target)\n", 2),
    ("        " + NEIGHBOR_OLD + "\n        # A final red/orange neighbor is intentional only for the goal pair.\n        if neighbor is not None and (abs(abs(target[1]-xyz(scene,neighbor)[1])-SPACING)>.003+CORRIDOR_ERROR_M or abs(target[0]-xyz(scene,neighbor)[0])>.003+CORRIDOR_ERROR_M):\n         neighbor=None\n",
     "        # Close placement is intentional only next to goal neighbours already in place.\n        neighbor=placed_neighbors(scene,color,target)\n"),
    # --- layout screens ---
    ("   for predecessor in COLORS:\n    if predecessor==stacked:break", "   for predecessor in TS.PLACE_ORDER:\n    if predecessor==stacked:break"),
    ("      yield from self.search_step(stacked+'_approach_screen',lambda:self.check(aj,pj,top_scene,active=stacked,payload=load,contact=True,support=SUPPORTS[stacked],assembly_neighbor='orange' if stacked=='green' else None))\n"
     "      yield from self.finish_steps(pj,top_scene,stacked,top_target,load,candidate['quaternion'],SUPPORTS[stacked],'orange' if stacked=='green' else None)\n",
     "      beside=placed_neighbors(top_scene,stacked,top_target)\n"
     "      yield from self.search_step(stacked+'_approach_screen',lambda:self.check(aj,pj,top_scene,active=stacked,payload=load,contact=True,support=SUPPORTS[stacked],assembly_neighbor=beside))\n"
     "      yield from self.finish_steps(pj,top_scene,stacked,top_target,load,candidate['quaternion'],SUPPORTS[stacked],beside)\n"),
    ("  for ranking in ranked_orders(self.scene,target,self.fk(start)['position'][:2]):", "  for ranking in ranked_orders(self.scene,target,self.fk(start)['position'][:2],TS.task_orders(side)):"),
    ("'layout_outer_bounds_m':{'min':LAYOUT_MIN.tolist(),'max':LAYOUT_MAX.tolist()},",
     "'layout_outer_bounds_m':{'min':LAYOUT_MIN.tolist(),'max':LAYOUT_MAX.tolist()},'goal':{'digest':TS.GOAL_DIGEST,'source':TS.GOAL_SOURCE,'spec':TS.GOAL,'side':side,'final_color':TS.FINAL_COLOR},"),
    # --- parallel lanes: try the layout that leaves the second table block where it is early ---
    ("    pairs=list(zip(self.layout_candidates,self.assigned_layout_ids)); orange=xyz(self.scene,'orange')[:2]\n"
     "    pairs.sort(key=lambda p:np.linalg.norm(np.asarray(p[0][0])+[0,p[0][1]*SPACING]-orange)>=.003)\n",
     "    pairs=list(zip(self.layout_candidates,self.assigned_layout_ids)); orange=xyz(self.scene,BASE_COLORS[1])[:2]\n"
     "    pairs.sort(key=lambda p:np.linalg.norm(np.asarray(p[0][0])+TS.offset_xy(BASE_COLORS[1],p[0][1])-orange)>=.003)\n"),
])
print('planner patched')
