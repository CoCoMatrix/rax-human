"""Seal goal1: controller unchanged; record host-file hashes and version metadata; validate. No hardware, no web switch."""
from pathlib import Path
import sys, json, hashlib, subprocess as sp

R = Path.cwd(); N = R.parent; B = N.parent
V = 'r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1'
O = B/'blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-web-20261119'
assert R == N/'runtime' and N.name == 'blocks-'+V+'-web-20261119'
read = lambda p: json.loads(p.read_text())
dump = lambda p, v: p.write_text(json.dumps(v, indent=2, ensure_ascii=False)+'\n')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
git = lambda d, *a: sp.check_output(['git', '-C', str(d), *a], text=True).strip()
C = R/'workspaces/magnetic-blocks-l-stack/r9-ssik-speed'; cd = C/'code/magnetic_blocks'
old = read(O/'WEB-VALIDATION.json'); m = read(C/'WORKSPACE.json')
commit = git(cd, 'rev-parse', 'HEAD')
assert not git(cd, 'status', '--porcelain', '--untracked-files=no'), 'controller has uncommitted changes'
assert git(cd, 'rev-parse', 'HEAD~1') == old['commit'], 'expected exactly one commit on top of start1'
hashes = {p.name: sha(p) for p in cd.glob('*.py')}
differs = {k for k in hashes if old['code_hashes'].get(k) != hashes[k]}
assert differs == {'assembly_plan.py', 'numeric_policy.py', 'task_spec.py', 'goal_spec.py', 'test_goal_structure.py'}, differs
assert (cd/'goal_spec.py').read_bytes() == (R/'services/dashboard/goal_spec.py').read_bytes()
assert not (cd/'GOAL.json').exists(), 'the sealed candidate must carry the default goal'
evidence = read(N/'PHASE3-EVIDENCE.json'); assert evidence['default_goal_equivalence']['same_conditions']['plan_equal'] == evidence['default_goal_equivalence']['same_conditions']['both_planned'] >= 12
assert evidence['controller_tests']['same_failure_set_as_start1'] is True
hs = m['required_host_sha256']
changed = {'services/dashboard/blocks_scale.py', 'services/dashboard/static/blocks/scale.html', 'services/dashboard/static/blocks/scale.js',
           'services/dashboard/goal_spec.py', 'services/dashboard/goal_gemini.py'}
actual = {k for k, h in hs.items() if sha(R/k) != h}
assert actual <= changed, actual
hs.update({k: sha(R/k) for k in changed})
for extra in ('services/dashboard/goal_spec.py', 'services/dashboard/goal_gemini.py'):
    hs[extra] = sha(R/extra)
tests = (N/'goal1-build/TESTS.log').read_text()
import re
assert re.search(r'^\d+ passed in ', tests.strip().splitlines()[-1]) and 'failed' not in tests.strip().splitlines()[-1], tests[-300:]
m.update(candidate_revision=V, commit=commit, candidate=str(C), runtime=str(R), optimization_runtime=str(R), runtime_patch=V,
         validation_report=str(N/'WEB-VALIDATION.json'),
         validation_scope='goal1 phase 3: planner follows the goal description (offline, pure math). Default goal plans identical to start1; execution checks and web hand-over of custom goals not done, so custom goals cannot run.',
         goal_description_schema='rax.blocks.goal.v1', planner_supports_custom_goal=False, web_switched=False,
         live_qualified=False, hardware_actions_started=False)
dump(C/'WORKSPACE.json', m)
f = C/'output/composition/VERIFICATION.json'; v = read(f); v.update({k: m[k] for k in v if k in m}); dump(f, v)
dump(N/'WEB-VALIDATION.json', dict(ok=True, passed=True, commit=commit, candidate_revision=V, code_hashes=hashes,
                                   required_host_sha256=hs, report=str(N/'goal1-build/TESTS.log'), offline_only=True))
sys.path.insert(0, str(R))
from services.dashboard.blocks_scale import Manager
Manager(root=R).validate_run_source()
dump(N/'DEPLOY-VERIFIED.json', dict(version=V, commit=commit, base_commit=old['commit'], phase=3))
print(json.dumps(dict(version=V, commit=commit, sealed_host_files=sorted(changed))))
