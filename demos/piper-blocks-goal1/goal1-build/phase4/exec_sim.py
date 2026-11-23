"""Dry execution of one goal through assembly_controller.main. No hardware, no network.

usage: exec_sim.py RUNTIME CODE_DIR RECORDED_RUN_DIR OUT.json

What is real: the recorded initial observation and 3D localization (strict replay of the
recorded initial-state step), the whole-task planner, FK/IK, collision checks and the
native trajectory preview (no-send transport), and every numeric execution check.
What is simulated: the arm reaches each commanded target exactly, the jaw closes to
51.6 mm on a block and opens to 72.4 mm, and the image-based evidence checks (alignment
before closing, placed-body visibility, displacement screening) are stubbed as passing.
So this exercises the control flow and geometry for a goal, not perception or contact.
"""
import collections, copy, glob, json, math, os, sys, tempfile, time, uuid
from pathlib import Path
from types import SimpleNamespace

R, CODE, RUN, OUT = (Path(p).resolve() for p in sys.argv[1:5])
WS = RUN/'rax-session/agent-workspace'
ARC = Path(sorted(glob.glob(str(WS/'output/composition/runs/*')))[0])
INIT_ROWS = [json.loads(l) for l in (ARC/'001-initial-state.tools.jsonl').read_text().splitlines()]
ALL_ROWS = [json.loads(l) for f in sorted(ARC.glob('*.tools.jsonl')) for l in f.read_text().splitlines()]

work = Path(tempfile.mkdtemp(prefix='exec-sim-', dir=os.environ['RAX_ARTIFACT_ROOT']))
(work/'runtime').symlink_to(WS/'runtime'); (work/'output').mkdir(); (work/'bridge').mkdir()
(work/'backend-options.json').write_text((RUN/'backend-options.json').read_text())  # read by the no-send transport
os.chdir(work)
sys.path[:0] = [str(CODE), str(R/'scripts')]

_real_time = time.time


class Clock:
    replaying = True
    now = INIT_ROWS[0]['started_unix'] - 1.0
    offset = 0.0

    @classmethod
    def value(cls):
        return cls.now if cls.replaying else _real_time() + cls.offset


base = Clock.now
time.time = Clock.value
time.time_ns = lambda: int(round(Clock.value() * 1e9))
time.monotonic = lambda: Clock.value() - base + 1000.0
time.perf_counter = time.monotonic


def _sleep(seconds):
    if Clock.replaying: Clock.now += max(0., float(seconds))
    else: Clock.offset += max(0., float(seconds))


time.sleep = _sleep

import numpy as np
import builtins, subprocess
from piper_motion_bridge import load_bridge
from native_fk.geometry import Kinematics
from pure_fk import row as fk_row

opts = json.loads((RUN/'backend-options.json').read_text())
# IK and collision checks run in their own process: the trajectory preview below needs the
# legacy `services` package, which cannot share a process with the runtime's `services`.
math = subprocess.Popen([sys.executable, str(Path(__file__).with_name('math_server.py')), str(R), str(RUN)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
assert json.loads(math.stdout.readline()).get('ready')


def math_call(tool, kw):
    math.stdin.write(json.dumps({'tool': tool, 'args': kw}) + '\n'); math.stdin.flush()
    reply = json.loads(math.stdout.readline())
    if 'error' in reply:
        kind = getattr(builtins, reply['error']['type'], None)
        raise (kind if isinstance(kind, type) and issubclass(kind, Exception) else RuntimeError)(reply['error']['message'])
    return reply['result']


bridge = load_bridge(Path(opts['collision_legacy_root']), 'faster')
from services.piper.motion_profiles import transport_options
transport = bridge.MotionTransport(bridge.PiperNoSendBackend(SimpleNamespace(backend_options={})), bridge.CollisionChecker(),
                                   arm_check=lambda: {'armed': True}, stop_client=lambda: {'offline_stop': True}, lease_file=work/'bridge/lease.json',
                                   **transport_options(json.loads((RUN/'bridge/state.json').read_text())['limits']), allow_left_gripper=False)
KIN = Kinematics()
MATH = ('piper_fk_batch', 'piper_solve_ik', 'piper_solve_ik_fast', 'piper_solve_ik_candidates', 'piper_check_path')


def first(tool, pred=lambda r: True):
    return next((copy.deepcopy(r['result']) for r in ALL_ROWS if r['tool'] == tool and isinstance(r.get('result'), dict) and pred(r)), None)


T_STATUS = first('status')
T_OBS = first('piper_observe')
T_WAYPOINTS = first('piper_move_waypoints', lambda r: r['result'].get('executed') is True)
T_CLOSE = first('piper_gripper_stroke', lambda r: r['arguments'].get('direction') == 'close' and r['result'].get('executed') is True)
T_OPEN = first('piper_gripper_stroke', lambda r: r['arguments'].get('direction') == 'open' and r['result'].get('executed') is True)


def refusal(operation, reason):
    return RuntimeError('MCP tool failed: ' + json.dumps({'error': {
        'type': 'RejectedBeforeDispatch', 'operation': operation, 'stage': 'validation', 'reason': reason, 'control_uncertain': False,
        'no_action_executed': True, 'executed': False, 'dispatch_started': False, 'request_no_action_executed': True, 'episode_ended': False}}))


class Sim:
    def __init__(self):
        self.i = 0; self.q = None; self.last = None; self.counts = collections.Counter(); self.simulated_motion_s = 0.

    def _obs(self):
        obs = copy.deepcopy(T_OBS)
        obs['measured14'] = list(self.q); obs['captured_at_ns'] = time.time_ns()
        if isinstance(obs.get('state_binding'), dict) and 'measured14' in obs['state_binding']: obs['state_binding']['measured14'] = list(self.q)
        return obs

    def _status(self):
        st = copy.deepcopy(T_STATUS); rs = st['robot_status']; now = time.time_ns() - 2_000_000
        rs['measured_state']['values'] = list(self.q)
        meta = rs['measured_state'].setdefault('feedback_metadata', {}); meta['source_timestamp_ns'] = now; meta['timestamp_ns'] = now
        fb = rs.get('bridge', {}).get('feedback')
        if isinstance(fb, dict): fb.update(q=list(self.q), timestamp_ns=now)
        rs['last_motion'] = copy.deepcopy(self.last) if self.last else rs.get('last_motion')
        st['wall_budget'] = {**st.get('wall_budget', {}), 'session_seconds': 1800, 'elapsed_seconds': 100., 'remaining_seconds': 1700., 'finish_reserve_seconds': 60, 'phase': 'active'}
        return st

    def _receipt(self, template, start, final, observe_after, seconds):
        Clock.offset += seconds; self.simulated_motion_s += seconds
        r = copy.deepcopy(template)
        r.update(executed=True, measured_arrival=True, measured_settled=True, final14=list(final), owner_request_id=uuid.uuid4().hex, completed_at_ns=time.time_ns())
        if 'target14' in r: r['target14'] = list(final)
        r['anchor_validation'] = {**(r.get('anchor_validation') or {}), 'measured14': list(start), 'planned_anchor14': list(start)}
        self.q = list(final); self.last = {k: v for k, v in r.items() if k != 'observation'}
        if observe_after: r['observation'] = self._obs()
        else: r.pop('observation', None)
        return r

    def value(self, tool, **kw):
        kw = json.loads(json.dumps(kw, allow_nan=False)); self.counts[tool] += 1
        if Clock.replaying:   # the recorded initial-state step, strictly
            row = INIT_ROWS[self.i]
            if tool == 'piper_fk' and row['tool'] != 'piper_fk': return fk_row(KIN.fk(kw['joints']))
            assert row['tool'] == tool and json.dumps(row['arguments'], sort_keys=True) == json.dumps(kw, sort_keys=True), ('initial-state replay diverged', tool, row['tool'])
            self.i += 1; Clock.now = max(Clock.now, row['started_unix']) + row['elapsed_s']
            if tool == 'piper_observe': self.q = list(row['result']['measured14'])
            if self.i == len(INIT_ROWS):
                Clock.offset = Clock.now - _real_time(); Clock.replaying = False
            return copy.deepcopy(row['result'])
        if tool == 'status': return self._status()
        if tool == 'piper_observe': return self._obs()
        if tool == 'piper_fk': return fk_row(KIN.fk(kw['joints']))
        if tool in MATH: return math_call(tool, kw)
        if tool == 'piper_preview_waypoints':
            try: plan = transport.preview_waypoints(kw['start14'], kw['waypoints'], kw['obstacles'], kw.get('blend_indices') or [], kw.get('blend_scale', 1.))
            except ValueError as exc: raise refusal(tool, str(exc)) from exc
            return dict(json.loads(json.dumps(plan)), review=json.loads(json.dumps(plan['collision'])), executed=False)
        observe_after = kw.pop('observe_after', False)
        if tool in ('piper_move_waypoints', 'piper_move_joints'):
            start = list(self.q); goal = kw['waypoints'][-1] if tool == 'piper_move_waypoints' else kw['joints']
            if tool == 'piper_move_waypoints' and max(abs(a - b) for a, b in zip(kw['start14'], start)) > 1e-4:
                raise refusal(tool, 'Multi-waypoint planning anchor drift')
            final = list(start); final[:6] = [float(x) for x in goal[:6]]
            return self._receipt(T_WAYPOINTS, start, final, observe_after, 2.0)
        if tool == 'piper_gripper_stroke':
            start = list(self.q); final = list(start); closing = kw['direction'] == 'close'
            final[6] = .0516 if closing else .0724
            r = self._receipt(T_CLOSE if closing else T_OPEN, start, final, observe_after, 3.4)
            r['stroke_direction'] = kw['direction']
            if closing: r['contact_range_m'] = kw.get('contact_range_m')
            return r
        raise AssertionError('Simulation does not provide tool ' + tool)


import assembly_parallel, assembly_plan, history_scene, ordinary_checks, qualitative, task_spec as TS

STUBBED = collections.Counter()


def stub(name, value):
    def call(*a, **k):
        STUBBED[name] += 1
        return dict(value, simulated=True)
    return call


qualitative.alignment_evidence = stub('alignment_before_close', {'ok': True, 'why': 'simulated image evidence'})
qualitative.retention_evidence = stub('retention_after_lift', {'ok': True, 'why': 'simulated image evidence'})
history_scene.reject_visible_displacement = stub('displacement_screen', {'available': False, 'unassociated_candidates': []})
ordinary_checks.view_evidence = stub('placed_body_visible', {'visible': True})

plan_info = {}


def offline_plan(planner, start, count=4):
    t = time.monotonic()
    plan = planner.plan(start)
    assembly_parallel.validate_plan(plan, planner.scene, start)
    plan['parallel_search'] = {'workers': 1, 'winner': 0, 'search_wall_s': time.monotonic() - t, 'partition': 'single offline lane (dry execution)', 'artifact': None}
    plan_info.update(elapsed_s=round(time.monotonic() - t, 1), order=plan['order'], side=plan['orange_side'])
    return plan


assembly_parallel.plan_parallel = offline_plan
import assembly_controller

import goal_spec as web_goal

ctx = Sim()
if os.environ.get('LANE'):   # production splits layouts over four lanes; one lane per process here
    ctx._blocks_layout_partition = tuple(int(x) for x in os.environ['LANE'].split('/'))
out = {'run': RUN.name, 'code': str(CODE), 'goal': web_goal.describe(TS.GOAL), 'goal_digest': TS.GOAL_DIGEST, 'final_color': TS.FINAL_COLOR}
report = assembly_controller.main(ctx, output_dir=work/'output/sim-run')
steps = [[s['label'], s['ok']] for s in report['steps']]
out.update(ok=report.get('ok'), status=report.get('status'), failed_step=report.get('failed_step'), why=str(report.get('why'))[:500],
           completion_mode=report.get('completion_mode'), plan=plan_info, steps=len(steps), step_labels=[s[0] for s in steps],
           tool_counts=dict(ctx.counts), stubbed_image_checks=dict(STUBBED), simulated_motion_s=round(ctx.simulated_motion_s, 1))
if report.get('ok'):
    plan = json.loads((work/'output/sim-run/WHOLE_TASK_PLAN.json').read_text()); final = report['final_scene']
    err = {c: float(np.linalg.norm(np.asarray(final['blocks'][c]['top_center']['surface_position_m']) - np.asarray(plan['goal_scene']['blocks'][c]['top_center']['surface_position_m']))) for c in TS.COLORS}
    pos = {c: final['blocks'][c]['top_center']['surface_position_m'] for c in TS.COLORS}; cells = {}
    xs = sorted({round(v[0], 2) for v in pos.values()}); ys = sorted({round(v[1], 2) for v in pos.values()})
    for c, v in pos.items(): cells.setdefault((len(ys) - 1 - ys.index(round(v[1], 2)), xs.index(round(v[0], 2))), []).append((v[2], c))
    got = web_goal.check({'stacks': [{'col': k[0], 'row': k[1], 'blocks': [c for _, c in sorted(v)]} for k, v in cells.items()], 'spacing': TS.GOAL['spacing']})
    out.update(final_position_error_mm={c: round(e * 1000, 2) for c, e in err.items()},
               structure_matches_goal=bool(got['ok'] and web_goal.rotation_key(got['spec']) == web_goal.rotation_key(TS.GOAL)),
               handoff_steps=[s for s in out['step_labels'] if 'empty-transition' in s])
OUT.write_text(json.dumps(out, ensure_ascii=False, indent=1))
math.kill()
out['lane'] = os.environ.get('LANE')
OUT.write_text(json.dumps(out, ensure_ascii=False, indent=1))
print(OUT.stem, 'ok', out['ok'], 'steps', out['steps'], 'plan', plan_info, 'structure', out.get('structure_matches_goal'), 'maxerr_mm', max(out.get('final_position_error_mm', {'-': -1}).values()),
      '' if out['ok'] else ('| ' + str(out['failed_step']) + ' | ' + out['why'][:200]))
