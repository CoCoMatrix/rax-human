"""Goal description for the four-block task: fixed schema, validation, feasibility hints.

Pure stdlib, no I/O, no robot access. A goal is a set of stacks on a small grid:

    {"schema": "rax.blocks.goal.v1",
     "stacks": [{"col": 0, "row": 0, "blocks": ["red", "green", "yellow"]},
                {"col": 1, "row": 0, "blocks": ["orange"]}],
     "spacing": "tight"}

Grid convention (viewer stands on the robot side of the table):
  col grows from the viewer's left to right, row grows from near the viewer to far.
  "blocks" lists one stack from bottom to top. Neighbouring cells are neighbouring blocks.
The description holds structure only. Millimetre positions are computed by the planner.
"""
from __future__ import annotations

import hashlib
import itertools
import json

SCHEMA = 'rax.blocks.goal.v1'
COLORS = ('red', 'orange', 'green', 'yellow')
COLOR_NAMES = {'red': '红', 'orange': '橙', 'green': '绿', 'yellow': '黄'}
SPACINGS = ('tight', 'loose')
MAX_LEVELS = 3   # A fourth layer needs the flange above the 360 mm workspace limit.
GRID = 4         # Four blocks never need more than a 4x4 footprint.

DEFAULT_SPEC = {
    'schema': SCHEMA,
    'stacks': [{'col': 0, 'row': 0, 'blocks': ['red', 'green', 'yellow']},
               {'col': 1, 'row': 0, 'blocks': ['orange']}],
    'spacing': 'tight',
}


def gemini_schema():
    """Strict JSON schema handed to the model. Every field is closed or enumerated."""
    return {
        'type': 'object', 'additionalProperties': False,
        'required': ['image_usable', 'visible_colors', 'contacts', 'stacks', 'spacing', 'viewpoint',
                     'confidence', 'uncertain', 'summary'],
        'properties': {
            'image_usable': {'type': 'boolean'},
            'visible_colors': {'type': 'array', 'items': {'type': 'string', 'enum': list(COLORS) + ['other']}},
            'contacts': {'type': 'array', 'items': {
                'type': 'object', 'additionalProperties': False, 'required': ['a', 'b', 'relation'],
                'properties': {'a': {'type': 'string', 'enum': list(COLORS)}, 'b': {'type': 'string', 'enum': list(COLORS)},
                               'relation': {'type': 'string', 'enum': ['on_top', 'side_by_side']}}}},
            'stacks': {'type': 'array', 'items': {
                'type': 'object', 'additionalProperties': False, 'required': ['col', 'row', 'blocks'],
                'properties': {
                    'col': {'type': 'integer'}, 'row': {'type': 'integer'},
                    'blocks': {'type': 'array', 'items': {'type': 'string', 'enum': list(COLORS)}}}}},
            'spacing': {'type': 'string', 'enum': list(SPACINGS)},
            'viewpoint': {'type': 'string', 'enum': ['top_down', 'frontal', 'corner_on']},
            'confidence': {'type': 'number'},
            'uncertain': {'type': 'array', 'items': {'type': 'string'}},
            'summary': {'type': 'string'},
        },
    }


def digest(spec):
    core = {'schema': spec['schema'], 'stacks': spec['stacks'], 'spacing': spec['spacing']}
    return hashlib.sha256(json.dumps(core, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def _is_int(value):
    return type(value) is int


def canonical(raw):
    """Return (spec, errors). spec is None when the description cannot be used."""
    errors = []
    if not isinstance(raw, dict):
        return None, ['目标描述必须是一个 JSON 对象']
    stacks = raw.get('stacks')
    if not isinstance(stacks, list) or not stacks:
        return None, ['缺少 stacks（每一柱积木的位置和自下而上的颜色）']
    spacing = raw.get('spacing', 'tight')
    if spacing not in SPACINGS:
        errors.append('spacing 只能是 tight（紧挨）或 loose（分开）')
        spacing = 'tight'
    cells, seen = {}, []
    for i, stack in enumerate(stacks):
        if not isinstance(stack, dict) or not _is_int(stack.get('col')) or not _is_int(stack.get('row')):
            errors.append(f'第 {i+1} 柱缺少整数的 col / row')
            continue
        blocks = stack.get('blocks')
        if not isinstance(blocks, list) or not blocks or any(b not in COLORS for b in blocks):
            errors.append(f'第 {i+1} 柱的 blocks 必须是红橙绿黄中的颜色，至少一块')
            continue
        cell = (stack['col'], stack['row'])
        if cell in cells:
            errors.append(f'格子 (列 {cell[0]}, 排 {cell[1]}) 出现了两次；同一格的积木请写成一柱')
            continue
        if len(blocks) > MAX_LEVELS:
            errors.append(f'一柱最多叠 {MAX_LEVELS} 层：第 4 层超出机械臂的工作高度')
        cells[cell] = list(blocks)
        seen += blocks
    for color in COLORS:
        count = seen.count(color)
        if count == 0:
            errors.append(f'缺少{COLOR_NAMES[color]}色积木')
        elif count > 1:
            errors.append(f'{COLOR_NAMES[color]}色积木出现了 {count} 次，每种颜色只能有一块')
    if cells:
        c0 = min(c for c, _ in cells)
        r0 = min(r for _, r in cells)
        cells = {(c - c0, r - r0): blocks for (c, r), blocks in cells.items()}
        if max(c for c, _ in cells) >= GRID or max(r for _, r in cells) >= GRID:
            errors.append(f'摆放范围超过 {GRID}×{GRID} 格，请把积木摆近一些再拍')
    if errors:
        return None, errors
    ordered = sorted(cells.items(), key=lambda item: (item[0][1], item[0][0]))
    spec = {'schema': SCHEMA,
            'stacks': [{'col': c, 'row': r, 'blocks': blocks} for (c, r), blocks in ordered],
            'spacing': spacing}
    return spec, []


def placement_orders(spec):
    """Orders that respect supports and leave one free pair of faces for the fingers.

    With tight spacing the open gripper needs the two cells on one axis to be free at
    the level being placed (each open finger reaches 63 mm from the block centre, more
    than the 38 mm to a neighbour's face). This is a geometric screen only; the planner
    and collision checker make the real decision.
    """
    where = {}
    for stack in spec['stacks']:
        for level, color in enumerate(stack['blocks']):
            where[color] = ((stack['col'], stack['row']), level)
    tight = spec['spacing'] == 'tight'
    orders = []
    for order in itertools.permutations(COLORS):
        height = {}
        for color in order:
            cell, level = where[color]
            if height.get(cell, 0) != level:
                break
            if tight:
                def occupied(dc, dr):
                    return height.get((cell[0] + dc, cell[1] + dr), 0) > level
                if (occupied(1, 0) or occupied(-1, 0)) and (occupied(0, 1) or occupied(0, -1)):
                    break
            height[cell] = level + 1
        else:
            orders.append(list(order))
    return orders


def contacts(spec):
    """Contact relations implied by a description: (upper, lower) pairs and unordered same-level neighbours."""
    on_top, side = set(), set()
    level_at = {}
    for stack in spec['stacks']:
        for level, color in enumerate(stack['blocks']):
            level_at[(stack['col'], stack['row'], level)] = color
            if level:
                on_top.add((color, stack['blocks'][level - 1]))
    for (col, row, level), color in level_at.items():
        for dc, dr in ((1, 0), (0, 1)):
            other = level_at.get((col + dc, row + dr, level))
            if other:
                side.add(frozenset((color, other)))
    return on_top, side


def contact_mismatch(spec, reported):
    """Compare the model's own contact list with its grid. Returns Chinese notes; empty when they agree."""
    if not isinstance(reported, list):
        return []
    on_top, side = contacts(spec)
    said_top, said_side = set(), set()
    for item in reported:
        if not isinstance(item, dict) or item.get('a') not in COLORS or item.get('b') not in COLORS or item['a'] == item['b']:
            continue
        if item.get('relation') == 'on_top':
            said_top.add((item['a'], item['b']))
        elif item.get('relation') == 'side_by_side':
            said_side.add(frozenset((item['a'], item['b'])))
    notes = []
    name = COLOR_NAMES
    for upper, lower in sorted(said_top ^ on_top):
        notes.append(f'{name[upper]}压在{name[lower]}上' + ('：Gemini 这样说，但格子位置里不是' if (upper, lower) in said_top else '：格子位置里是这样，但 Gemini 没有这样说'))
    for pair in sorted(sorted(p) for p in said_side ^ side):
        a, b = pair
        notes.append(f'{name[a]}和{name[b]}并排挨着' + ('：Gemini 这样说，但格子位置里不相邻' if frozenset(pair) in said_side else '：格子位置里相邻，但 Gemini 没有这样说'))
    return notes


def rotations(spec):
    """The four quarter-turn variants of a description (canonical form each)."""
    out = []
    cells = [((s['col'], s['row']), s['blocks']) for s in spec['stacks']]
    for _ in range(4):
        cells = [((r, -c), blocks) for (c, r), blocks in cells]
        out.append(canonical({'stacks': [{'col': c, 'row': r, 'blocks': b} for (c, r), b in cells], 'spacing': spec['spacing']})[0])
    return out


def rotation_key(spec):
    return min(json.dumps(r['stacks'], sort_keys=True) for r in rotations(spec))


def tally(answers):
    """Group several model readings. Readings that differ only by a quarter turn count as the same.

    Returns (candidates best-first, agreement) with agreement in unanimous / majority / split / none.
    """
    groups = {}
    for answer in answers:
        checked = check(answer)
        if checked['ok']:
            groups.setdefault(rotation_key(checked['spec']), []).append((checked, answer))
    candidates = []
    for items in sorted(groups.values(), key=len, reverse=True):
        exact = {}
        for checked, answer in items:
            exact.setdefault(checked['digest'], []).append((checked, answer))
        checked, answer = max(exact.values(), key=len)[0]
        candidates.append({'check': checked, 'answer': answer, 'votes': len(items)})
    total = len(answers)
    if not candidates:
        agreement = 'none'
    elif candidates[0]['votes'] == total:
        agreement = 'unanimous'
    elif candidates[0]['votes'] * 2 > total:
        agreement = 'majority'
    else:
        agreement = 'split'
    return candidates, agreement


def describe(spec):
    parts = []
    for stack in spec['stacks']:
        names = '→'.join(COLOR_NAMES[b] for b in stack['blocks'])
        parts.append(f"列{stack['col']+1}排{stack['row']+1}：{names}")
    return '；'.join(parts) + ('（紧挨）' if spec['spacing'] == 'tight' else '（分开）')


def check(raw):
    """Validate any incoming description. Never raises on bad content."""
    if isinstance(raw, dict) and isinstance(raw.get('_error'), str):
        return {'ok': False, 'errors': [raw['_error']], 'warnings': [], 'spec': None}
    spec, errors = canonical(raw)
    if spec is None:
        return {'ok': False, 'errors': errors, 'warnings': [], 'spec': None}
    warnings = []
    orders = placement_orders(spec)
    if not orders:
        warnings.append('按紧挨摆放时，至少有一块在两个方向上都有邻块，张开的夹爪放不进去。'
                        '规划时需要把其中一个方向的间距放宽到约 96 mm 以上，或者把间距改为“分开”。')
    return {'ok': True, 'errors': [], 'warnings': warnings, 'spec': spec, 'digest': digest(spec),
            'is_default': digest(spec) == digest(DEFAULT_SPEC), 'summary': describe(spec),
            'placement_orders': orders,
            'levels': max(len(s['blocks']) for s in spec['stacks'])}


def from_cells(rows, spacing='tight'):
    """Build a description from per-colour {col,row,level} rows (the web edit form)."""
    if not isinstance(rows, dict):
        return {'stacks': []}
    cells = {}
    for color in COLORS:
        row = rows.get(color)
        if not isinstance(row, dict) or not all(_is_int(row.get(k)) for k in ('col', 'row', 'level')):
            continue
        cells.setdefault((row['col'], row['row']), []).append((row['level'], color))
    stacks = []
    for (col, row), items in cells.items():
        items.sort()
        if [level for level, _ in items] != list(range(1, len(items) + 1)):
            # A floating or doubled level: keep it visible to validation as a broken stack.
            return {'stacks': [], 'spacing': spacing,
                    '_error': f'列 {col+1} 排 {row+1} 的层数必须从第 1 层开始连续，且每层一块'}
        stacks.append({'col': col, 'row': row, 'blocks': [c for _, c in items]})
    return {'schema': SCHEMA, 'stacks': stacks, 'spacing': spacing}
