"""Run one goal on one scene the way production does: four layout lanes in parallel, first complete plan wins.
usage: lanes.py GOAL_NAME RUN_REL OUTDIR"""
import json, os, subprocess, sys, time
B = '/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments'; D = B + '/goal1-phase3-dev'
N = B + '/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1-web-20261119/runtime'
PY = '/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python'
goal, run, out = sys.argv[1:4]
code = D + '/codes/' + goal if goal != 'default' else N + '/workspaces/magnetic-blocks-l-stack/r9-ssik-speed/code/magnetic_blocks'
os.makedirs(out, exist_ok=True); tag = goal + '__' + os.path.basename(run); t0 = time.monotonic(); procs = []
for i in range(4):
    f = f'{out}/{tag}.lane{i}.json'
    if os.path.exists(f): os.remove(f)
    env = {**os.environ, 'LANE': f'{i}/4', 'RAX_ARTIFACT_ROOT': D + '/art', 'PYTHONDONTWRITEBYTECODE': '1'}
    procs.append((f, subprocess.Popen(['nice', '-n', '5', PY, D + '/plan_dump.py', N, B + '/' + run, code, f], env=env, stdout=subprocess.DEVNULL, stderr=open(f[:-5] + '.err', 'w'))))
winner = None; results = {}
while time.monotonic() - t0 < 260 and winner is None and len(results) < 4:
    time.sleep(.5)
    for f, p in procs:
        if f not in results and p.poll() is not None:
            try: results[f] = json.load(open(f))
            except Exception: results[f] = {'plan': {'ok': False, 'error': 'lane crashed'}}
            if results[f]['plan'].get('ok'): winner = results[f]; break
for f, p in procs:
    if p.poll() is None: p.kill()
final = winner or {'plan': {'ok': False, 'error': ' || '.join(sorted({(r['plan'].get('error') or '')[:90] for r in results.values()})) or 'timeout',
                           'screens': sum([r['plan'].get('screens', []) for r in results.values()], [])[:10], 'why': sum([r['plan'].get('why', []) for r in results.values()], [])[:12]}}
final['plan']['wall_s'] = round(time.monotonic() - t0, 1)
json.dump(final, open(f'{out}/{tag}.json', 'w'), ensure_ascii=False)
print(tag, final['plan'].get('ok'), final['plan']['wall_s'], flush=True)
