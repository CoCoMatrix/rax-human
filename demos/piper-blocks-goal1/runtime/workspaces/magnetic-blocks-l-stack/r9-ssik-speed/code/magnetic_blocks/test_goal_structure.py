"""goal1 phase 3: the task structure follows the goal description. Pure; no services, no hardware."""
import importlib.util
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent


def load(tmp_path, goal=None, name='ts'):
    """Import a private copy of task_spec with its own GOAL.json (the module reads the file at import)."""
    for f in ('task_spec.py', 'goal_spec.py'):
        shutil.copy(HERE/f, tmp_path/f)
    if goal is not None:
        (tmp_path/'GOAL.json').write_text(json.dumps(goal))
    sys.path.insert(0, str(tmp_path))
    try:
        sys.modules.pop('goal_spec', None)
        spec = importlib.util.spec_from_file_location(name, tmp_path/'task_spec.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(tmp_path))
        sys.modules.pop('goal_spec', None)


def stacks(*rows, spacing='tight'):
    return {'stacks': [{'col': c, 'row': r, 'blocks': list(b)} for c, r, b in rows], 'spacing': spacing}


def test_default_goal_equals_the_former_constants(tmp_path):
    t = load(tmp_path)
    assert t.GOAL_SOURCE == 'default' and t.IS_DEFAULT_GOAL
    assert t.COLORS == ('red', 'orange', 'green', 'yellow') and t.PLACE_ORDER == t.COLORS
    assert t.SUPPORTS == {'green': 'red', 'yellow': 'green'} and list(t.SUPPORTS) == ['green', 'yellow']
    assert t.BASE_COLORS == ('red', 'orange') and t.ANCHOR == 'red' and t.FINAL_COLOR == 'yellow'
    assert t.HIGH_COLORS == {'yellow'} and t.SECOND_LAYER == ('green',)
    assert t.SIDES == (-1, 1) and set(t.ROTATION_PENALTY_M.values()) == {0.}
    assert t.offset_xy('orange', -1) == [0.0, -0.068] and t.offset_xy('orange', 1) == [0.0, 0.068]
    assert all(t.offset_xy(c, s) == [0.0, 0.0] for c in ('red', 'green', 'yellow') for s in t.SIDES)
    assert t.task_orders(-1) == [('red', 'orange', 'green', 'yellow'), ('orange', 'red', 'green', 'yellow')]
    assert {c: t.goal_neighbors(c) for c in t.COLORS} == {'red': ('orange',), 'orange': ('red',), 'green': ('orange',), 'yellow': ()}


def test_an_explicit_default_file_is_the_default(tmp_path):
    t = load(tmp_path, stacks((3, 2, ['red', 'green', 'yellow']), (4, 2, ['orange'])))
    assert t.GOAL_SOURCE == 'file' and t.IS_DEFAULT_GOAL and t.SIDES == (-1, 1)


def test_invalid_goal_file_stops_the_import(tmp_path):
    with pytest.raises(ValueError, match='Invalid GOAL.json'):
        load(tmp_path, stacks((0, 0, ['red', 'green', 'yellow'])))


def test_quarter_turns_keep_handedness_and_never_mirror(tmp_path):
    # L shape: orange near, green far behind it, yellow to the far left.
    t = load(tmp_path, stacks((1, 0, ['orange']), (1, 1, ['green', 'red']), (0, 1, ['yellow'])))
    assert t.SIDES == (-1, 1, 2, -2) and t.ANCHOR == 'orange'
    for side in t.SIDES:
        a = np.array(t.offset_xy('green', side)); b = np.array(t.offset_xy('yellow', side)) - a
        assert np.isclose(np.linalg.norm(a), .068) and np.isclose(np.linalg.norm(b), .068)
        # orange -> green -> yellow turns left when seen from above, in every allowed placement.
        assert a[0] * b[1] - a[1] * b[0] > 0
    assert t.offset_xy('green', -1) == pytest.approx([.068, 0.]) and t.offset_xy('yellow', -1) == pytest.approx([.068, .068])
    assert t.ROTATION_PENALTY_M[-1] == 0. and all(t.ROTATION_PENALTY_M[s] > 0 for s in (1, 2, -2))


def test_orders_go_layer_by_layer_and_end_with_the_final_block(tmp_path):
    t = load(tmp_path, stacks((0, 0, ['yellow']), (1, 0, ['orange', 'red']), (2, 0, ['green'])))
    assert t.BASE_COLORS == ('yellow', 'orange', 'green') and t.SUPPORTS == {'red': 'orange'} and t.FINAL_COLOR == 'red'
    orders = t.task_orders(-1)
    assert len(orders) == 6 and all(o[-1] == 'red' and set(o[:3]) == {'yellow', 'orange', 'green'} for o in orders)
    assert t.HIGH_COLORS == frozenset() and t.goal_neighbors('red') == ('yellow', 'green')
    flat = load(tmp_path, stacks((0, 0, ['red']), (1, 0, ['orange']), (2, 0, ['green']), (3, 0, ['yellow'])), 'flat')
    assert flat.FINAL_COLOR == 'yellow' and len(flat.task_orders(-1)) == 6


def test_the_third_layer_block_uses_the_high_grasp_whatever_its_colour(tmp_path):
    t = load(tmp_path, stacks((0, 0, ['green', 'red', 'orange']), (1, 0, ['yellow'])))
    assert t.HIGH_COLORS == {'orange'} and t.FINAL_COLOR == 'orange' and t.SUPPORTS == {'red': 'green', 'orange': 'red'}


def test_second_tower_top_beside_the_first_is_a_high_placement(tmp_path):
    t = load(tmp_path, stacks((0, 0, ['red', 'green']), (1, 0, ['orange', 'yellow'])))
    assert t.FINAL_COLOR == 'yellow' and t.HIGH_COLORS == {'yellow'}
    assert t.goal_neighbors('yellow') == ('red', 'green') and t.goal_neighbors('green') == ('orange', 'yellow')
    apart = load(tmp_path, stacks((0, 0, ['red', 'green']), (2, 0, ['orange', 'yellow'])), 'apart')
    assert apart.HIGH_COLORS == frozenset()


def test_tight_square_widens_exactly_one_grid_direction(tmp_path):
    t = load(tmp_path, stacks((0, 0, ['red']), (1, 0, ['orange']), (0, 1, ['green']), (1, 1, ['yellow'])))
    assert t.PITCH_MODES == (1, 2) and len(t.SIDES) == 8 and all(abs(s) > 10 for s in t.SIDES)
    for side in t.SIDES:
        cols, rows = t.pitches(side)
        assert sorted((cols, rows)) == [.068, .110] and t.task_orders(side)
    loose = load(tmp_path, stacks((0, 0, ['red']), (1, 0, ['orange']), (0, 1, ['green']), (1, 1, ['yellow']), spacing='loose'), 'loose')
    assert loose.PITCH_MODES == (0,) and loose.pitches(-1) == (.110, .110) and len(loose.task_orders(-1)) == 6


def test_controller_and_web_use_the_same_goal_schema_file():
    web = HERE.parents[4]/'services/dashboard/goal_spec.py'
    if web.exists():
        assert web.read_bytes() == (HERE/'goal_spec.py').read_bytes()
