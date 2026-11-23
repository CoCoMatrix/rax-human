"""Plan one lane offline; every native collision sample check the planner causes (path
certification through piper_check_path) runs with the Coal pre-filter AND without it on the
same state. Any difference beyond 1 nm in a reported float, or any other difference in the
result or refusal, is recorded.
usage: coal_dual.py RUNTIME RUN CODE_DIR OUT.json LANE"""
import sys, json, time, collections
OUT_FILE = sys.argv[4]; lane = sys.argv[5]
sys.argv = [sys.argv[0], *sys.argv[1:4], '/dev/null', '--no-plan']
exec(open('/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/goal1-phase3-dev/plan_dump.py').read().split("if '--no-plan' not in sys.argv")[0])

CK = backend.collision
assert CK._coal_prefilter is not None, 'Coal pre-filter did not load'
PRE = CK._coal_prefilter; ORIGINAL = CK.check
stats = collections.Counter(); bad = []; worst = [0.]


def close(x, y):
    if isinstance(x, float) and isinstance(y, float):
        worst[0] = max(worst[0], abs(x - y)); return abs(x - y) <= 1e-9
    if isinstance(x, dict) and isinstance(y, dict): return x.keys() == y.keys() and all(close(x[k], y[k]) for k in x)
    if isinstance(x, (list, tuple)) and isinstance(y, (list, tuple)): return len(x) == len(y) and all(close(u, v) for u, v in zip(x, y))
    return type(x) == type(y) and x == y


def run(q, kw, prefilter):
    CK._coal_prefilter = prefilter
    try: raw = ORIGINAL(q, **kw); return raw, json.loads(json.dumps(raw, default=str)), None
    except Exception as exc: return exc, None, (type(exc).__name__, str(exc))
    finally: CK._coal_prefilter = PRE


def both(q, **kw):
    with CK._check_lock:
        before = CK._right_pair_cache   # both modes start from the same right-arm cache state
        t = time.perf_counter(); raw, ron, eon = run(q, kw, PRE); stats['on_us'] += int(1e6 * (time.perf_counter() - t))
        after = CK._right_pair_cache; collector = getattr(CK, '_path_tolerances', None); CK._path_tolerances = None
        CK._right_pair_cache = before
        t = time.perf_counter(); _, roff, eoff = run(q, kw, None); stats['off_us'] += int(1e6 * (time.perf_counter() - t))
        CK._right_pair_cache = after; CK._path_tolerances = collector
    stats['calls'] += 1
    stats['clear' if ron and ron.get('cad_clear') else 'not_clear'] += 1
    same = eon == eoff if (eon or eoff) else close(ron, roff)
    if not same:
        stats['DIFF'] += 1
        if len(bad) < 5:
            keys = [k for k in (ron or {}) if roff and ron.get(k) != roff.get(k)]
            bad.append({'keys': keys, 'on': str(eon or {k: ron[k] for k in keys})[:600], 'off': str(eoff or {k: roff.get(k) for k in keys})[:600]})
    if eon: raise raw
    return raw   # the planner continues with the deployed (pre-filter on) result


CK.check = both
ctx._blocks_layout_partition = (int(lane), 4)
P = AP.Planner(ctx, scene0, robot, *finger_models(ctx, obs))
t = time.monotonic()
try: res = P.plan(start); ok = True
except Exception as exc: res = {'error': str(exc)[:200]}; ok = False
out = {'run': RUN.name, 'lane': lane, 'ok': ok, 'wall_s': round(time.monotonic() - t, 1), 'stats': dict(stats),
       'max_float_diff_m': worst[0], 'diffs': bad, 'order': res.get('order')}
json.dump(out, open(OUT_FILE, 'w'), ensure_ascii=False)
print(RUN.name[9:], lane, 'plan', ok, out['wall_s'], 's', dict(stats), worst[0])
