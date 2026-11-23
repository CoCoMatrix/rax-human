"""Compare two plan_dump output folders scene by scene. usage: compare.py BASE NEW"""
import json, sys
from pathlib import Path

base, new = Path(sys.argv[1]), Path(sys.argv[2])
static = ('layouts', 'goal_scenes', 'orders', 'grasp_variants', 'source_inward', 'tilt_limits')
totals = {'scenes': 0, 'static_equal': 0, 'both_planned': 0, 'plan_equal': 0, 'same_layout_order': 0, 'outcome_diff': 0}
for fb in sorted(base.glob('*.json')):
    fn = new/fb.name
    if not fn.exists():
        continue
    a, b = json.loads(fb.read_text()), json.loads(fn.read_text())
    totals['scenes'] += 1
    bad = [k for k in static if a.get(k) != b.get(k)]
    totals['static_equal'] += not bad
    pa, pb = a.get('plan') or {}, b.get('plan') or {}
    line = fb.stem[9:] + ' static ' + ('OK' if not bad else 'DIFF ' + ','.join(bad))
    if pa.get('ok') and pb.get('ok'):
        totals['both_planned'] += 1
        ka = {k: v for k, v in pa.items() if k not in ('elapsed_s', 'goal_digest')}; kb = {k: v for k, v in pb.items() if k not in ('elapsed_s', 'goal_digest')}
        same_head = all(pa[k] == pb[k] for k in ('order', 'goal_anchor', 'orange_side', 'goal'))
        totals['same_layout_order'] += same_head
        if ka == kb:
            totals['plan_equal'] += 1; line += ' | plan IDENTICAL'
        else:
            diffs = [k for k in ka if ka[k] != kb.get(k)]
            detail = ''
            if 'moves' in diffs and len(pa['moves']) == len(pb['moves']):
                for ma, mb in zip(pa['moves'], pb['moves']):
                    if ma != mb:
                        keys = [k for k in ma if ma[k] != mb.get(k)]
                        detail += f" {ma['color']}:{keys}"
                        if 'actions' in keys:
                            sa = [x[0] for x in ma['actions']]; sb = [x[0] for x in mb['actions']]
                            detail += f"(stages {'same' if sa == sb else 'differ'}, {len(sa)}/{len(sb)})"
            line += f" | plan DIFF {diffs}{detail} (layout/order {'same' if same_head else 'differ'})"
    elif bool(pa.get('ok')) != bool(pb.get('ok')):
        totals['outcome_diff'] += 1
        line += f" | OUTCOME base={pa.get('ok')} new={pb.get('ok')} {(pa.get('error') or pb.get('error') or '')[:110]}"
    else:
        line += ' | both failed: ' + (pa.get('error') or '')[:60]
    line += f" | t {pa.get('elapsed_s')}/{pb.get('elapsed_s')}s"
    print(line)
print(totals)
