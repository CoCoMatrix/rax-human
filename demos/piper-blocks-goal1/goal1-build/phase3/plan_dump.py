"""Offline planner dump (pure FK/IK/collision math, NoOwner backend, no hardware).

usage: plan_dump.py RUNTIME RUN_DIR CODE_DIR OUT.json [--no-plan]
Writes deterministic planner inputs (layout candidates, goal scenes, orders, grasp families)
and, unless --no-plan, the complete whole-task plan in a normalized form.
"""
import sys, json, copy, os, time, tempfile
from pathlib import Path
import numpy as np

R = Path(sys.argv[1]); RUN = Path(sys.argv[2]); CODE = Path(sys.argv[3]); OUT = Path(sys.argv[4])
sys.path[:0] = [str(CODE), str(R)]
from services.controller.config import Configuration
from services.piper.backend import PiperBackend
from services.piper.installed_collision import load_checker
from native_fk.geometry import Kinematics
import assembly_plan as AP
import numeric_policy as NP
from assembly_controller import finger_models

arc = next((RUN/'rax-session/agent-workspace/output/composition/runs').glob('*'))
init = json.loads((arc/'001-initial-state.json').read_text())['result']
robot = init['start_status']['robot_status']; obs = init['measured']; scene0 = init['facts'][0]['scene']
opts = json.loads((RUN/'backend-options.json').read_text())


class NoOwner:
    def state(self): raise AssertionError('no hardware')


config = Configuration(root=Path(tempfile.mkdtemp(dir=os.environ['RAX_ARTIFACT_ROOT'])), task='parallel_math', robot='piper', backend_options=opts)
backend = PiperBackend(config, owner=NoOwner(), cameras=object()); backend.solver.start()
backend.collision = load_checker(opts['collision_legacy_root'])
PURE = ('piper_fk', 'piper_fk_batch', 'piper_solve_ik', 'piper_solve_ik_fast', 'piper_solve_ik_candidates', 'piper_check_path')


class Ctx:
    _blocks_validated_fk = Kinematics(); _blocks_local_fk = True

    def value(self, name, **kw):
        assert name in PURE, name
        return backend.call(name, kw)[0]


def rnd(v, n=6):
    if isinstance(v, float): return round(v, n)
    if isinstance(v, (list, tuple)): return [rnd(x, n) for x in v]
    if isinstance(v, dict): return {k: rnd(x, n) for k, x in v.items()}
    if isinstance(v, np.ndarray): return rnd(v.tolist(), n)
    if isinstance(v, np.generic): return rnd(v.item(), n)
    return v


ctx = Ctx()
if os.environ.get('LANE'):
    ctx._blocks_layout_partition = tuple(int(x) for x in os.environ['LANE'].split('/'))
P = AP.Planner(ctx, scene0, robot, *finger_models(ctx, obs)); start = obs['measured14']
out = {'run': RUN.name, 'code': str(CODE)}
layouts = AP.layout_candidates(P.scene, robot)
out['layouts'] = rnd(layouts)
out['goal_scenes'] = [rnd({c: AP.xyz(AP.goal_scene(P.scene, a, s), c) for c in AP.COLORS}) for a, s in layouts[:12]]
sxy = P.fk(start)['position'][:2]
out['orders'] = [rnd(AP.ranked_orders(P.scene, AP.goal_scene(P.scene, a, s), sxy)) for a, s in layouts[:6]]
out['grasp_variants'] = {c: rnd(list(AP.grasp_variants(P.scene, c))) for c in AP.COLORS}
out['source_inward'] = {c: rnd(list(AP.source_inward(P.scene, c))) for c in AP.COLORS}
out['tilt_limits'] = {c: rnd(NP.task_orientation_limit_deg(c)) for c in AP.COLORS}
if '--no-plan' not in sys.argv:
    t0 = time.monotonic()
    try:
        res = P.plan(start)
        out['plan'] = {'ok': True, 'order': res['order'], 'requested_order': res['requested_order'], 'goal_anchor': rnd(res['goal_anchor']),
                       'orange_side': res['orange_side'], 'goal_digest': (res.get('goal') or {}).get('digest'), 'layout_rank': res['layout_choice']['rank'],
                       'goal': rnd({c: AP.xyz(res['goal_scene'], c) for c in AP.COLORS}),
                       'moves': [{'color': m['color'], 'yaw': m['yaw_degrees'], 'placement_yaw': m['placement_yaw_degrees'],
                                  'target_top': rnd(m['target_top']), 'grasp': rnd({k: (m.get('grasp_candidate') or {}).get(k) for k in ('yaw', 'tilt_degrees', 'height_delta_m', 'source_family')}),
                                  'actions': [[a['stage'], a['kind'], rnd(a['target14'][:7], 5), a.get('contact'), a.get('withdraw'), a.get('support'), a.get('assembly_neighbor'), bool(a.get('terminal_release_hold'))] for a in m['actions']]}
                                 for m in res['moves']]}
    except Exception as exc:
        out['plan'] = {'ok': False, 'error': type(exc).__name__ + ': ' + str(exc)[:600]}
        counts = sorted(getattr(P, 'screen_counts', {}).items(), key=lambda kv: -kv[1])[:8]
        out['plan']['screens'] = counts
        import collections
        why = collections.Counter(str(d.get('why'))[:170] for d in P.diagnostics if d.get('why'))
        out['plan']['why'] = why.most_common(10)
        out['plan']['last_candidate'] = str(getattr(P, 'last_candidate', None))[:400]
        out['plan']['layout_cursor'] = [getattr(P, 'layout_cursor', None), len(getattr(P, 'layout_candidates', []))]
        out['plan']['last'] = [str({k: v for k, v in d.items() if k in ('anchor', 'orange_side', 'order', 'why', 'phase')})[:260] for d in P.diagnostics[-4:]]
    out['plan']['elapsed_s'] = round(time.monotonic() - t0, 1)
OUT.write_text(json.dumps(out, ensure_ascii=False))
print(RUN.name, 'layouts', len(layouts), 'plan', out.get('plan', {}).get('ok'), out.get('plan', {}).get('elapsed_s'), (out.get('plan') or {}).get('error', '')[:120])
