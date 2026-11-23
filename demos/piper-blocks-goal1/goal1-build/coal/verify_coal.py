"""Replay every recorded native trajectory preview through goal1's deployed collision checker
twice: Coal pre-filter on and off (the plain FCL check as before). Compares every per-sample
check result and the final path result field by field, and times both.

usage: verify_coal.py RUNTIME LEGACY OUT.json RUN_DIR...   (production Python)
"""
import glob, json, os, sys, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

R = Path(sys.argv[1]); LEGACY = sys.argv[2]; OUT = Path(sys.argv[3]); RUNS = [Path(p) for p in sys.argv[4:]]
CK = None


def init():
    global CK
    sys.path.insert(0, str(R))
    from services.piper.installed_collision import load_checker
    CK = load_checker(LEGACY)
    assert CK._coal_prefilter is not None, 'Coal pre-filter did not load'


def canon(x):
    return json.dumps(x, sort_keys=True, default=str)


def structure(x, y, worst):
    """True when everything except float values is identical (keys, lists, links, flags, counts,
    sample indices); records the largest float difference. Plain FCL itself varies by ~1e-11 m
    with call history (fcl_history.py), so bit equality of distances is not a property of FCL."""
    if isinstance(x, float) and isinstance(y, float):
        worst[0] = max(worst[0], abs(x - y)); return True
    if isinstance(x, dict) and isinstance(y, dict):
        return x.keys() == y.keys() and all([structure(x[k], y[k], worst) for k in x])
    if isinstance(x, list) and isinstance(y, list):
        return len(x) == len(y) and all([structure(u, v, worst) for u, v in zip(x, y)])
    return type(x) == type(y) and x == y


def same(a, b, worst):
    if a.startswith('EXC') or b.startswith('EXC'): return a == b
    return structure(json.loads(a), json.loads(b), worst)


def run_mode(points, obstacles, prefilter):
    CK._coal_prefilter = prefilter; CK._right_pair_cache = None
    seen = []; original = CK.check
    def recording(q, **kw):
        r = original(q, **kw); seen.append(canon(r)); return r
    CK.check = recording
    t = time.perf_counter()
    try: result = canon(CK.path(points, obstacles=obstacles))
    except Exception as e: result = 'EXC ' + repr(e)
    finally: CK.check = original
    return result, seen, time.perf_counter() - t


def job(item):
    run, index, points, obstacles = item
    pre = CK._coal_prefilter
    try:
        # CONTROL=1: plain FCL against itself, to measure FCL's own call-history variation.
        on, s_on, t_on = run_mode(points, obstacles, None if os.environ.get('CONTROL') == '1' else pre)
        off, s_off, t_off = run_mode(points, obstacles, None)
    finally: CK._coal_prefilter = pre
    worst = [0.]
    decisions = same(on, off, worst) and len(s_on) == len(s_off) and all([same(x, y, worst) for x, y in zip(s_on, s_off)])
    diff = next((i for i, (x, y) in enumerate(zip(s_on, s_off)) if x != y), None)
    return {'run': run, 'preview': index, 'samples': len(s_off), 'on_s': t_on, 'off_s': t_off,
            'path_identical': on == off, 'decisions_identical': decisions, 'identical_to_1nm': decisions and worst[0] <= 1e-9, 'max_float_diff': worst[0], 'samples_identical': s_on == s_off, 'first_sample_diff': diff,
            'clear': '"cad_clear": true' in off and '"whole_path_checked": true' in off}


def jobs():
    sys.path.insert(0, str(R))
    from services.piper.waypoint_plan import compile_arm_path
    out = []
    for run in RUNS:
        for f in sorted(glob.glob(str(run / 'rax-session/agent-workspace/output/composition/runs/*/*.tools.jsonl'))):
            for i, line in enumerate(Path(f).read_text().splitlines()):
                r = json.loads(line)
                if r['tool'] != 'piper_preview_waypoints': continue
                args = r['arguments']; res = r.get('result')
                if isinstance(res, dict) and res.get('targets14'):
                    pts = [args['start14'], *res['targets14']]
                else:   # refused preview: rebuild its wire with the default limits to replay the refusal
                    start = np.asarray(args['start14'], float); targets = []
                    for w in args['waypoints']:
                        q = start.copy(); q[:6] = w; targets.append(q.tolist())
                    box = {}
                    def capture(points): box['p'] = points; return {'cad_clear': True, 'workspace_clear': True}
                    try: compile_arm_path(start.tolist(), targets, check_state=lambda q: None, check_path=capture, blend_indices=args.get('blend_indices') or [], blend_scale=args.get('blend_scale', 1.))
                    except Exception: continue
                    pts = box.get('p')
                    if not pts: continue
                out.append((run.name, f'{Path(f).name}:{i}', pts, args['obstacles']))
    return out


if __name__ == '__main__':
    init()
    work = jobs()
    with Pool(8, initializer=init) as pool:
        rows = pool.map(job, work, chunksize=1)
    on = sum(r['on_s'] for r in rows); off = sum(r['off_s'] for r in rows)
    summary = {'previews': len(rows), 'samples': sum(r['samples'] for r in rows),
               'path_results_identical': sum(r['path_identical'] for r in rows),
               'all_sample_results_identical': sum(r['samples_identical'] for r in rows),
               'decisions_identical': sum(r['decisions_identical'] for r in rows),
               'all_results_identical_to_1nm': sum(r['identical_to_1nm'] for r in rows),
               'max_reported_float_diff_m': max(r['max_float_diff'] for r in rows),
               'refused_previews': sum(not r['clear'] for r in rows),
               'native_check_off_s': round(off, 2), 'native_check_on_s': round(on, 2),
               'speedup': round(off / on, 2), 'saved_fraction': round(1 - on / off, 3)}
    OUT.write_text(json.dumps({'summary': summary, 'rows': rows}))
    print(json.dumps(summary, indent=1))
    for r in rows:
        if not (r['path_identical'] and r['samples_identical']): print('DIFF', r)
        if not r['decisions_identical']: print('DECISION DIFF', r)
