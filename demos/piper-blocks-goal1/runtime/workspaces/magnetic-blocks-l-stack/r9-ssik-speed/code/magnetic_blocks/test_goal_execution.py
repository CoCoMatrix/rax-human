"""goal1 phase 4: execution rules follow the goal description. Pure; no services, no hardware.

Each case runs in a fresh interpreter on a private copy of the controller, because the
task structure is fixed when task_spec is imported.
"""
import json
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

PROBE = r'''
import json, sys
import numpy as np
import task_spec as TS, assembly_batch as B, ordinary_checks as O
arm = lambda stage, color, **kw: {**dict(kind='arm', stage=stage, color=color, payload=None, pickup=False, contact=False, withdraw=False,
                                         start14=[0.]*14, target14=[0.]*14, scene={}), **kw}
retreat = arm('green-retreat', 'green', withdraw=True); hover = arm('yellow-hover', 'yellow')
seen = []
O.view_evidence = lambda fn, obs, *a: (seen.append([fn.__name__, *[x for x in a if isinstance(x, (str, tuple, list))][:1]]) or {'visible': False, 'ok': None})
scene = {'blocks': {c: {'top_center': {'surface_position_m': [0.2, -0.2 + 0.02 * i, 0.08]}} for i, c in enumerate(TS.COLORS)}}
pairs = {}
for color in TS.COLORS:
    seen.clear(); O.placed_region_evidence({}, scene, color, scene['blocks'][color]['top_center']['surface_position_m'])
    pairs[color] = [list(x[1]) for x in seen if x[0] == 'wrist_pair']
print(json.dumps({'handoff': TS.EMPTY_HANDOFF, 'final': TS.FINAL_COLOR, 'empty_handoff': B.empty_handoff(retreat),
                  'transition': B.green_yellow_transition([retreat, hover]), 'groups': [len(g) for g in B.groups([retreat, hover])],
                  'pairs': pairs, 'name': TS.COLOR_NAMES_ZH[TS.FINAL_COLOR]}))
'''


def probe(tmp_path, goal=None):
    for f in HERE.glob('*.py'):
        if not f.name.startswith('test_'):
            shutil.copy(f, tmp_path/f.name)
    shutil.copytree(HERE/'native_fk', tmp_path/'native_fk')
    if goal is not None:
        (tmp_path/'GOAL.json').write_text(json.dumps(goal))
    out = subprocess.run([sys.executable, '-c', PROBE], cwd=tmp_path, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr[-1500:]
    return json.loads(out.stdout.strip().splitlines()[-1])


def test_default_goal_keeps_the_rehearsed_green_to_yellow_handoff(tmp_path):
    r = probe(tmp_path)
    assert r['handoff'] == ['green', 'yellow'] and r['final'] == 'yellow' and r['name'] == '黄色'
    assert r['empty_handoff'] is True and r['transition'] is True and r['groups'] == [2]
    # Fallback wrist evidence looks at the support, or at the other table block: as before.
    assert r['pairs'] == {'red': [['orange', 'red']], 'orange': [['red', 'orange']], 'green': [['red', 'green']], 'yellow': [['green', 'yellow']]}


def test_other_goals_run_the_two_moves_separately_and_use_their_own_neighbours(tmp_path):
    flat = {'stacks': [{'col': i, 'row': 0, 'blocks': [c]} for i, c in enumerate(['red', 'orange', 'green', 'yellow'])], 'spacing': 'tight'}
    r = probe(tmp_path, flat)
    assert r['handoff'] is None and r['empty_handoff'] is False and r['transition'] is False and r['groups'] == [1, 1]
    assert r['final'] == 'yellow'
    # A middle block has two table neighbours; the first that is not seen falls through to the second.
    assert r['pairs']['orange'] == [['red', 'orange'], ['green', 'orange']] and r['pairs']['red'] == [['orange', 'red']]


def test_the_block_placed_last_follows_the_goal(tmp_path):
    goal = {'stacks': [{'col': 0, 'row': 0, 'blocks': ['yellow']}, {'col': 1, 'row': 0, 'blocks': ['orange', 'red']}, {'col': 2, 'row': 0, 'blocks': ['green']}], 'spacing': 'tight'}
    r = probe(tmp_path, goal)
    assert r['final'] == 'red' and r['name'] == '红色' and r['handoff'] is None
    assert r['pairs']['red'] == [['orange', 'red']] and r['pairs']['yellow'] == [['orange', 'yellow']]
    source = (HERE/'assembly_controller.py').read_text()
    assert "color!=TS.FINAL_COLOR" in source and "!='yellow'" not in source
