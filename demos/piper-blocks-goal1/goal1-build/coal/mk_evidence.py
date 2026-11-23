"""Write COAL-EVIDENCE.json into the goal1 version root from the goal1-coal-dev results."""
import json, glob, os, subprocess
from pathlib import Path
B = Path('/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments'); D = B/'goal1-coal-dev'
N = next(B.glob('blocks-*goal1-web-20261119'))
v = json.load(open(D/'verify_all.json'))['summary']; c = json.load(open(D/'control_fcl_fcl.json'))['summary']
dual = [json.load(open(f)) for f in sorted(glob.glob(str(D/'dual/*.json')))]
pairs = []
for f in sorted(glob.glob(str(D/'p_off/*.json'))):
    a = json.load(open(f))['plan']; b = json.load(open(f.replace('p_off', 'p_on')))['plan']
    pairs.append((json.load(open(f)), json.load(open(f.replace('p_off', 'p_on'))), a, b))
strip = lambda d: {k: x for k, x in d.items() if k not in ('elapsed_s', 'screens')}
both = [(a, b) for _, _, a, b in pairs if a.get('ok') and b.get('ok')]
sims = []
for f in sorted(glob.glob(str(D/'sim_on/*.json'))):
    a = json.load(open(f)); b = json.load(open(f.replace('sim_on', 'sim_off')))
    same = (a['ok'], a['failed_step'], a['step_labels'], a['status'], {k: x for k, x in (a['plan'] or {}).items() if k != 'elapsed_s'}) == \
           (b['ok'], b['failed_step'], b['step_labels'], b['status'], {k: x for k, x in (b['plan'] or {}).items() if k != 'elapsed_s'})
    sims.append(dict(case=os.path.basename(f)[:-5], ok=a['ok'], failed_step=a['failed_step'], identical=same,
                     planning_s_on=(a['plan'] or {}).get('elapsed_s'), planning_s_off=(b['plan'] or {}).get('elapsed_s')))
n1 = open(D/'pytest_native2_1.log').read().strip().splitlines()[-1]; n0 = open(D/'pytest_native2_0.log').read().strip().splitlines()[-1]
fail = lambda p: sorted(l.split(' - ')[0] for l in open(p) if l.startswith(('FAILED', 'ERROR')))
ev = {
 'change': 'Native sampled collision check (services/piper/installed_collision*.py): before each exact FCL distance, a Coal 3.0.3 collide query with security margin = margin + 1 mm skips pairs that are certainly farther apart. Every pair that can matter still gets the exact FCL distance, so contacts, tolerated clearances and expected jaw contacts come from the same FCL numbers. Coal is vendored in runtime/vendor/coal-3.0.3 (BSD-2; licenses kept) and loaded only if every file matches services/piper/coal_vendor_manifest.json; otherwise, or with RAX_COAL_PREFILTER=0, the check is the previous plain FCL check. The shared venv and start1 are unchanged.',
 'recorded_preview_replay': dict(previews=v['previews'], samples=v['samples'], refused_previews=v['refused_previews'],
     decisions_identical=v['decisions_identical'], bit_identical=v['all_sample_results_identical'], max_reported_float_diff_m=v['max_reported_float_diff_m'],
     native_check_s_off=v['native_check_off_s'], native_check_s_on=v['native_check_on_s'], speedup=v['speedup'],
     note='Reported distances are always FCL values. Plain FCL against itself on the same 346 previews: %d not bit-identical, max %.1e m (fcl_history.py shows the same sample giving different last digits after different call histories). One tolerated finger/obstacle clearance differs by 1.0e-07 m with the pre-filter, below the 1e-6 m FCL GJK distance tolerance; the closest recorded distance to the 3 mm margin is 3.0e-06 m.' % (c['previews'] - c['all_sample_results_identical'], c['max_reported_float_diff_m'])),
 'planning_dual_check': dict(planning_lanes=len(dual), calls_compared=sum(d['stats']['calls'] for d in dual), differences=sum(d['stats'].get('DIFF', 0) for d in dual),
     max_float_diff_m=max(d['max_float_diff_m'] for d in dual), check_time_s_on=round(sum(d['stats']['on_us'] for d in dual) / 1e6, 1), check_time_s_off=round(sum(d['stats']['off_us'] for d in dual) / 1e6, 1)),
 'paired_23_scene_planning': dict(scenes=len(pairs), static_equal=sum({k: x for k, x in A.items() if k != 'plan'} == {k: x for k, x in Bj.items() if k != 'plan'} for A, Bj, _, _ in pairs),
     both_planned=len(both), plan_equal=sum(strip(a) == strip(b) for a, b in both), outcome_diff=sum(a.get('ok') != b.get('ok') for _, _, a, b in pairs),
     planning_s_both_planned_off_on=[round(sum(a['elapsed_s'] for a, _ in both), 1), round(sum(b['elapsed_s'] for _, b in both), 1)],
     note='Search is time-sliced; with faster checks 2 scenes reached a different (also fully checked) candidate first.'),
 'dry_execution': dict(runs=len(sims), outcome_identical=sum(s['identical'] for s in sims), cases=sims,
     note='exec_sim through the real native preview; g6_square_flat fails identically with and without Coal (180 s planning budget on 154128; known 73 mm vs 72.4 mm jaw margin at green-hover on 164238).'),
 'native_collision_tests': dict(on=n1, off=n0, same_result_on_off=fail(D/'pytest_native2_1.log') == fail(D/'pytest_native2_0.log'),
     note='test_b07.py and test_retreat_feedback_1mm.py run with the RETREAT-ARGS.json fixture (they cannot find it in place); the one failure is an unrelated planner test.'),
 'controller_tests': dict(summary=open(D/'pytest.log').read().strip().splitlines()[-1], same_failure_set_as_vec=fail(D/'pytest.log') == fail(B/'goal1-vec-dev/pytest.log')),
 'tools': 'goal1-build/coal: patch_coal.py, patch_coal2.py, verify_coal.py, coal_dual.py; experiments/goal1-coal-dev has the raw results',
}
(N/'COAL-EVIDENCE.json').write_text(json.dumps(ev, indent=2, ensure_ascii=False) + '\n')
print(json.dumps({k: (x if not isinstance(x, dict) else {kk: vv for kk, vv in x.items() if kk not in ('cases', 'note')}) for k, x in ev.items() if k not in ('change', 'tools')}, indent=1, ensure_ascii=False))
