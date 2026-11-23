"""Task structure derived from one goal description (goal_spec.py, schema rax.blocks.goal.v1).

Without a GOAL.json next to this file the goal is the original c-Four1.0 structure
(red/green/yellow tower with orange beside red) and every derived value below equals
the former hard-coded constants. One process plans and executes one goal.
"""
import itertools
import json
from pathlib import Path

import goal_spec

COLORS=('red','orange','green','yellow')  # The four physical blocks; perception and scene identity.
VERSION='r9-ssik-sharedik-portfolio-inward1-blend1-carry2'
PREFLIGHT_BUDGET_S=180.0  # Four-block candidate; original three-block budget remains90s.

FAST_PASS_BUDGET_S=30.0
LAYOUT_SLICE_FAST_S=8.0
LAYOUT_SLICE_FULL_S=15.0

GOAL_FILE=Path(__file__).with_name('GOAL.json')
TIGHT_PITCH_M=.068  # 68 mm centre spacing: nominal 8 mm between 60 mm blocks.
WIDE_PITCH_M=.110   # Clear of an open finger (63 mm from the block centre) with margin.


def _load():
    if not GOAL_FILE.exists():
        return json.loads(json.dumps(goal_spec.DEFAULT_SPEC)),'default'
    checked=goal_spec.check(json.loads(GOAL_FILE.read_text()))
    if not checked['ok']:
        raise ValueError('Invalid GOAL.json: '+'; '.join(checked['errors']))
    return checked['spec'],'file'


GOAL,GOAL_SOURCE=_load()
GOAL_DIGEST=goal_spec.digest(GOAL)
IS_DEFAULT_GOAL=GOAL_DIGEST==goal_spec.digest(goal_spec.DEFAULT_SPEC)
CELL={};LEVEL={};_STACK={};_BELOW={}
for _i,_stack in enumerate(GOAL['stacks']):
    for _level,_color in enumerate(_stack['blocks']):
        CELL[_color]=(_stack['col'],_stack['row']);LEVEL[_color]=_level;_STACK[_color]=_i
        if _level:_BELOW[_color]=_stack['blocks'][_level-1]
# Level order: every lower layer before any higher one, stacks in description order.
PLACE_ORDER=tuple(sorted(COLORS,key=lambda c:(LEVEL[c],_STACK[c])))
BASE_COLORS=tuple(c for c in PLACE_ORDER if LEVEL[c]==0)
SUPPORTS={c:_BELOW[c] for c in PLACE_ORDER if LEVEL[c]>0}
ANCHOR=BASE_COLORS[0]          # Layout anchors are positions of this block.
FINAL_COLOR=PLACE_ORDER[-1]    # Always placed last; only it may end released in place.
SECOND_LAYER=tuple(c for c in PLACE_ORDER if LEVEL[c]==1)
TIGHT=GOAL['spacing']=='tight'
# The blended empty move from the released green block to the yellow pickup was rehearsed
# for the original structure only. Other goals run those two moves as separate checked steps.
EMPTY_HANDOFF=('green','yellow') if IS_DEFAULT_GOAL else None
COLOR_NAMES_ZH={'red':'红色','orange':'橙色','green':'绿色','yellow':'黄色'}


def _beside_equal_tower(color):
    """The last block, set on a second layer right beside another finished second-layer block,
    must be carried in above that neighbour: the same high approach as a third-layer block."""
    if not TIGHT or color!=FINAL_COLOR or LEVEL[color]!=1:return False
    cell=CELL[color]
    return any(c!=color and LEVEL[c]==1 and abs(CELL[c][0]-cell[0])+abs(CELL[c][1]-cell[1])==1 for c in COLORS)


# High placements use the tilted grasp family: the third layer, and the case above.
HIGH_COLORS=frozenset(c for c in COLORS if LEVEL[c]>=2 or _beside_equal_tower(c))

# A layout "side" selects how the grid lies on the table. Viewer stands at the robot:
# +X is away from the robot, +Y is to its left. Quarter turns only; never a mirror image.
#   side: (unit XY of +col, unit XY of +row)
_AXES={-1:((0,-1),(1,0)),1:((0,1),(-1,0)),2:((1,0),(0,1)),-2:((-1,0),(0,-1))}


def _feasible(order,wide_cols=False,wide_rows=False):
    """Supports first, and one free pair of faces for the open fingers at the level being placed."""
    height={}
    for color in order:
        cell,level=CELL[color],LEVEL[color]
        if height.get(cell,0)!=level:return False
        if TIGHT:
            def occupied(dc,dr):return height.get((cell[0]+dc,cell[1]+dr),0)>level
            cols=not wide_cols and (occupied(1,0) or occupied(-1,0))
            rows=not wide_rows and (occupied(0,1) or occupied(0,-1))
            if cols and rows:return False
        height[cell]=level+1
    return True


def _level_orders():
    layers=[[c for c in PLACE_ORDER if LEVEL[c]==level] for level in range(max(LEVEL.values())+1)]
    for combo in itertools.product(*[itertools.permutations(layer) for layer in layers]):
        order=tuple(itertools.chain.from_iterable(combo))
        if order[-1]==FINAL_COLOR:yield order


def _pitch_modes():
    if not TIGHT:return (0,)
    if any(_feasible(order) for order in _level_orders()):return (0,)
    # Tight in both directions leaves no room for the fingers: widen one grid direction.
    return tuple(mode for mode,(cols,rows) in ((1,(True,False)),(2,(False,True))) if any(_feasible(o,cols,rows) for o in _level_orders()))


def side_parts(side):
    mode,base=divmod(abs(side),10)
    return (base if side>0 else -base),mode


PITCH_MODES=_pitch_modes()
# Default goal: the two original placements of the pair along Y, with the same codes as before.
SIDES=tuple(sign*(10*mode+base) for mode in PITCH_MODES for base,sign in (((1,-1),(1,1)) if IS_DEFAULT_GOAL else ((1,-1),(1,1),(2,1),(2,-1))))
# The orientation of the photo (base side -1) is preferred; other quarter turns cost a little rank.
ROTATION_PENALTY_M={side:(0. if IS_DEFAULT_GOAL or side_parts(side)[0]==-1 else .03) for side in SIDES}


def pitches(side):
    _,mode=side_parts(side)
    near=TIGHT_PITCH_M if TIGHT else WIDE_PITCH_M
    return (WIDE_PITCH_M if mode==1 else near),(WIDE_PITCH_M if mode==2 else near)


def offset_xy(color,side):
    """Table offset of a block from the anchor block for one layout side."""
    base,_=side_parts(side);(ex,ey)=_AXES[base];pc,pr=pitches(side)
    dc=CELL[color][0]-CELL[ANCHOR][0];dr=CELL[color][1]-CELL[ANCHOR][1]
    return [dc*pc*ex[0]+dr*pr*ey[0],dc*pc*ex[1]+dr*pr*ey[1]]


def task_orders(side):
    _,mode=side_parts(side)
    return [order for order in _level_orders() if _feasible(order,mode==1,mode==2)]


def goal_neighbors(color):
    """Blocks in an edge-sharing cell at this level or the one below: the only ones a
    block being lowered into place can legitimately come close to."""
    cell,level=CELL[color],LEVEL[color]
    return tuple(c for c in PLACE_ORDER if c!=color and abs(CELL[c][0]-cell[0])+abs(CELL[c][1]-cell[1])==1 and LEVEL[c] in (level,level-1))
