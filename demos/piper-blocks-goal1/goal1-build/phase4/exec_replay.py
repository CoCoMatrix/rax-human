"""Strict offline replay of one recorded real run through assembly_controller.main.

usage: exec_replay.py CODE_DIR RECORDED_WORKSPACE OUT.json

No hardware, no network, no services. Every ctx.value() call made by the code under test
must equal the next recorded call (tool and arguments); it then receives the recorded
result. The clock follows the recorded timestamps. Planning itself is not repeated: the
recorded certified plan and its execution cache are restored exactly as production does.
"""
import copy, glob, json, math, os, sys, tempfile, time
from pathlib import Path

CODE, WS, OUT = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve(), Path(sys.argv[3]).resolve()
ARC = Path(sorted(glob.glob(str(WS/'output/composition/runs/*')))[0])
ROWS = []
for f in sorted(ARC.glob('*.tools.jsonl')):
    for line in f.read_text().splitlines():
        ROWS.append((f.name[:-12], json.loads(line)))
RECORDED = json.loads((ARC/'REPORT.json').read_text())
SKIPPED_TOOLS = {'piper_planning_pool_open', 'piper_planning_pool_close'}  # planning is restored, not repeated

work = Path(tempfile.mkdtemp(prefix='exec-replay-'))
(work/'runtime').symlink_to(WS/'runtime')   # recorded frames, read only by the code
(work/'output').mkdir()
os.chdir(work)
sys.path.insert(0, str(CODE))


class Clock:
    base = ROWS[0][1]['started_unix'] - 1.0
    now = base


time.time = lambda: Clock.now
time.time_ns = lambda: int(round(Clock.now * 1e9))
time.monotonic = lambda: Clock.now - Clock.base + 1000.0
time.perf_counter = time.monotonic


def _sleep(seconds):
    Clock.now += max(0., float(seconds))


time.sleep = _sleep


def differ(a, b, path=''):
    """First difference between two JSON values, or None."""
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
        return None if a is b or a == b and type(a) is type(b) else f'{path}: {a!r} != {b!r}'
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return None if math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9) else f'{path}: {a!r} != {b!r}'
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            return f'{path}: keys {sorted(set(a) ^ set(b))}'
        for k in a:
            d = differ(a[k], b[k], path + '.' + str(k))
            if d: return d
        return None
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return f'{path}: length {len(a)} != {len(b)}'
        for i, (x, y) in enumerate(zip(a, b)):
            d = differ(x, y, f'{path}[{i}]')
            if d: return d
        return None
    return None if a == b else f'{path}: {str(a)[:80]!r} != {str(b)[:80]!r}'


class ReplayMismatch(BaseException):
    """Not an Exception: the code under test must not swallow a divergence."""


class Replay:
    def __init__(self):
        self.i = 0; self.mismatch = None; self.local_fk = 0

    def _local_fk(self, joints):
        from native_fk.geometry import Kinematics
        from pure_fk import row
        self.local_fk += 1
        return row(Kinematics().fk(list(joints)))

    def value(self, tool, **kw):
        kw = json.loads(json.dumps(kw, allow_nan=False))
        while self.i < len(ROWS) and ROWS[self.i][1]['tool'] in SKIPPED_TOOLS:
            self.i += 1
        nxt = ROWS[self.i] if self.i < len(ROWS) else None
        problem = 'no recorded call left' if nxt is None else (f"tool {nxt[1]['tool']}" if nxt[1]['tool'] != tool else differ(nxt[1]['arguments'], kw, 'arguments'))
        if problem and tool == 'piper_fk':
            # finger_models() runs between recorded steps; production logs only inside steps.
            return self._local_fk(kw['joints'])
        if problem:
            self.mismatch = {'call_index': self.i, 'step': nxt[0] if nxt else None, 'called': tool, 'problem': problem,
                             'called_arguments': json.dumps(kw)[:600], 'recorded_arguments': json.dumps(nxt[1]['arguments'])[:600] if nxt else None}
            raise ReplayMismatch(problem)
        step, row = nxt; self.i += 1
        Clock.now = max(Clock.now, row['started_unix']) + row['elapsed_s']
        if 'error' in row:
            raise RuntimeError(row['error'].split(': ', 1)[1] if ': ' in row['error'] else row['error'])
        return copy.deepcopy(row['result'])


import assembly_parallel


def restored_plan(planner, start, count=4):
    plan = json.loads((ARC/'WHOLE_TASK_PLAN.json').read_text())
    folder = WS/plan['parallel_search']['artifact']; winner = plan['parallel_search']['winner']
    assembly_parallel.validate_plan(plan, planner.scene, start)
    assembly_parallel.restore_execution_cache(planner, json.loads((folder/str(winner)/'EXECUTION-CACHE.json').read_text()))
    planner.ik_attempts = []; planner.opening_check = plan['opening_check']
    return plan


assembly_parallel.plan_parallel = restored_plan
import assembly_controller

ctx = Replay(); out = {'run': WS.parents[1].name, 'code': str(CODE), 'recorded_ok': RECORDED.get('ok'), 'recorded_calls': len(ROWS)}
try:
    report = assembly_controller.main(ctx, output_dir=work/'output/replay-run')
    out.update(ok=report.get('ok'), status=report.get('status'), failed_step=report.get('failed_step'), why=str(report.get('why'))[:400],
               completion_mode=report.get('completion_mode'),
               steps=[[s['label'], s['ok']] for s in report['steps']])
except ReplayMismatch as exc:
    out.update(ok=None, status='diverged')
remaining = [r['tool'] for _, r in ROWS[ctx.i:] if r['tool'] not in SKIPPED_TOOLS]
recorded_steps = [[s['label'], s['ok']] for s in RECORDED['steps']]
out.update(mismatch=ctx.mismatch, consumed=ctx.i, remaining_calls=len(remaining), local_fk_calls=ctx.local_fk,
           steps_equal=out.get('steps') == recorded_steps, recorded_steps=len(recorded_steps),
           recorded_failed_step=RECORDED.get('failed_step'), recorded_completion_mode=RECORDED.get('completion_mode'))
out['identical'] = bool(ctx.mismatch is None and not remaining and out.get('steps_equal') and out.get('ok') == RECORDED.get('ok')
                        and out.get('failed_step') == RECORDED.get('failed_step') and out.get('completion_mode') == RECORDED.get('completion_mode'))
OUT.write_text(json.dumps(out, ensure_ascii=False, indent=1))
print(out['run'], 'IDENTICAL' if out['identical'] else 'DIFFERENT', 'ok', out.get('ok'), '/', out['recorded_ok'], 'calls', ctx.i, '/', len(ROWS),
      'remaining', len(remaining), 'mismatch', (ctx.mismatch or {}).get('problem'), '|', (out.get('why') or '')[:140] if not out['identical'] else '')
