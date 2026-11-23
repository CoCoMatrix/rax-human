#!/usr/bin/env python3
"""Restore the goal1 snapshot into a runnable version directory and verify every seal.

    python3 restore_goal1.py --target <new version dir> [--private-from <original goal1 version dir>] [--proxy URL]

What it does (no robot, camera or model is touched):
  1. copies runtime/, the evidence JSONs and goal1-build/ from this folder to --target;
  2. installs the exact Coal 3.0.3 and SSIK 8.2.0 wheels from PyPI into a temp dir and copies
     only the binary files that the repository leaves out, then checks every file against
     services/piper/coal_vendor_manifest.json and ssik_vendor_manifest.json;
  3. copies the recorded camera frames that are not published (frozen r3 runtime/frames and
     the frames cited by the controller memory) from --private-from, checking them against
     SHA256SUMS.json and with the Web's own memory-evidence check;
  4. rebuilds the controller's git repository from magnetic_blocks.bundle and checks that
     HEAD is the sealed commit with a clean tree;
  5. points WORKSPACE.json / VERIFICATION.json at the new location;
  6. checks the 32 sealed host files and the controller hashes in WEB-VALIDATION.json.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ORIGINAL = '/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1-web-20261119'
WORKSPACE = 'runtime/workspaces/magnetic-blocks-l-stack/r9-ssik-speed'
FROZEN = 'runtime/versions/magnetic-blocks-l-stack/20261111-r3-v0.1.0'
COAL_WHEELS = ['coal==3.0.3', 'libcoal==3.0.3', 'eigenpy==3.13.0', 'cmeel-boost==1.90.0', 'cmeel-assimp==6.0.5',
               'cmeel-octomap==1.10.0', 'cmeel-qhull==8.0.2.1', 'cmeel-zlib==1.3.2']
SSIK_WHEELS = ['ssik==8.2.0', 'urchin==0.0.31']
EVIDENCE = ['DEPLOY-VERIFIED.json', 'WEB-VALIDATION.json', 'WEB-VALIDATION.start1.json', 'COAL-EVIDENCE.json',
            'VEC-EVIDENCE.json', 'PHASE3-EVIDENCE.json', 'PHASE4-EVIDENCE.json']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def step(message):
    print('==', message, flush=True)


def pip_target(python, wheels, dest, proxy):
    env = dict(os.environ)
    if proxy:
        env.update(HTTPS_PROXY=proxy, HTTP_PROXY=proxy)
    subprocess.run([python, '-m', 'pip', 'install', '-q', '--disable-pip-version-check', '--no-deps',
                    '--only-binary=:all:', '--target', str(dest), *wheels], check=True, env=env)


def fill_vendor(manifest_path, root, installed, rename_licenses):
    """Copy missing manifest files from the pip install, then verify the whole manifest."""
    manifest = json.loads(Path(manifest_path).read_text())
    copied = 0
    for rel in manifest:
        dst = root / rel
        if dst.exists():
            continue
        src = installed / rel
        if rename_licenses and rel.startswith('licenses/'):
            _, dist, rest = rel.split('/', 2)
            src = installed / (dist + '.dist-info') / rest
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        copied += 1
    bad = [rel for rel, digest in manifest.items() if sha(root / rel) != digest]
    if bad:
        raise SystemExit(f'{manifest_path.name}: {len(bad)} files differ, e.g. {bad[:3]}')
    return copied, len(manifest)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--target', type=Path, required=True, help='new, empty version directory')
    p.add_argument('--python', default=sys.executable, help='Python 3.12 used for pip (wheels are cp312 x86_64)')
    p.add_argument('--proxy', help='HTTP(S) proxy for pip, e.g. http://127.0.0.1:7892')
    p.add_argument('--private-from', type=Path,
                   help='an existing copy of the original goal1 version directory; its recorded camera frames '
                        '(frozen r3 runtime/frames, ~124 MB, and the frames the controller memory cites, ~615 MB) '
                        'are not published and are copied from here')
    a = p.parse_args()
    target = a.target.resolve()
    if target.exists() and any(target.iterdir()):
        raise SystemExit(f'{target} is not empty; never restore over a running version')

    step('copy snapshot')
    target.mkdir(parents=True, exist_ok=True)
    shutil.copytree(HERE / 'runtime', target / 'runtime', symlinks=True)
    shutil.copytree(HERE / 'goal1-build', target / 'goal1-build', symlinks=True)
    for name in EVIDENCE:
        shutil.copy2(HERE / name, target / name)
    runtime = target / 'runtime'

    step('restore Coal 3.0.3 and SSIK 8.2.0 binaries from PyPI')
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        pip_target(a.python, COAL_WHEELS, tmp / 'coal', a.proxy)
        pip_target(a.python, SSIK_WHEELS, tmp / 'ssik', a.proxy)
        coal_root = runtime / 'vendor/coal-3.0.3'
        n, total = fill_vendor(runtime / 'services/piper/coal_vendor_manifest.json', coal_root, tmp / 'coal', True)
        present = {str(f.relative_to(coal_root)) for f in coal_root.rglob('*') if f.is_file() and '__pycache__' not in f.parts}
        if present != set(json.loads((runtime / 'services/piper/coal_vendor_manifest.json').read_text())):
            raise SystemExit('Coal vendor directory has files outside its manifest')
        print(f'   coal: copied {n}, verified {total}')
        n, total = fill_vendor(runtime / 'services/piper/ssik_vendor_manifest.json', runtime / 'services/piper/ssik_vendor', tmp / 'ssik', False)
        print(f'   ssik: copied {n}, verified {total}')

    step('frozen r3 origin version')
    frozen = target / FROZEN
    sums = json.loads((frozen / 'SHA256SUMS.json').read_text())
    missing = [rel for rel in sums if not (frozen / rel).is_file()]
    private = a.private_from.resolve() if a.private_from else None
    if missing and private:
        for rel in missing:
            (frozen / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(private / FROZEN / rel, frozen / rel)
        missing = []
    bad = [rel for rel in sums if (frozen / rel).is_file() and sha(frozen / rel) != sums[rel]]
    if bad:
        raise SystemExit(f'frozen version differs from SHA256SUMS.json: {bad[:3]}')
    if missing:
        print(f'   WARNING {len(missing)} recorded frames missing; the Web refuses to run until they are copied in (--private-from)')
    else:
        print(f'   verified {len(sums)} files')
    for path in frozen.rglob('*'):
        if path.is_file():
            path.chmod(0o444)

    step('memory evidence cited by the controller memory')
    sys.path.insert(0, str(runtime / 'scripts'))
    from piper_handoff import copy_memory_evidence  # the same check the Web runs before every run
    workspace = target / WORKSPACE
    try:
        if private:
            result = copy_memory_evidence(private / WORKSPACE, workspace, memory_dir=workspace / 'memory')
        else:
            result = copy_memory_evidence(workspace, workspace, memory_dir=workspace / 'memory')
        print(f'   {result["references_checked"]} cited files present')
    except ValueError as error:
        print(f'   WARNING {error}; the Web refuses to run until the frames are copied in (--private-from)')

    step('controller git repository')
    code = target / WORKSPACE / 'code/magnetic_blocks'
    git = lambda *args: subprocess.run(['git', '-C', str(code), *args], check=True, capture_output=True, text=True).stdout.strip()
    git('-c', 'init.defaultBranch=main', 'init', '-q')
    git('fetch', '-q', str(HERE / 'magnetic_blocks.bundle'), 'HEAD')
    git('update-ref', 'refs/heads/main', 'FETCH_HEAD')
    git('reset', '-q')
    head = git('rev-parse', 'HEAD')
    expected = (HERE / 'CONTROLLER_COMMIT').read_text().strip()
    if head != expected or git('status', '--porcelain', '--untracked-files=no'):
        raise SystemExit(f'controller is not the sealed clean commit (HEAD {head}, expected {expected})')
    print(f'   HEAD {head}, clean')

    step('point workspace metadata at the new location')
    for rel in ('WORKSPACE.json', 'output/composition/VERIFICATION.json'):
        path = target / WORKSPACE / rel
        path.write_text(path.read_text().replace(ORIGINAL, str(target)))

    step('sealed files')
    validation = json.loads((target / 'WEB-VALIDATION.json').read_text())
    host_bad = [rel for rel, digest in validation['required_host_sha256'].items()
                if not (runtime / rel).is_file() or sha(runtime / rel) != digest]
    code_now = {p.name: sha(p) for p in code.glob('*.py')}
    if host_bad or code_now != validation['code_hashes']:
        raise SystemExit(f'seal mismatch: host {host_bad[:3]}, controller equal={code_now == validation["code_hashes"]}')
    print(f'   {len(validation["required_host_sha256"])} host files and {len(code_now)} controller files match WEB-VALIDATION.json')
    print(f'\nRestored to {target}. Start the Web from {runtime} (see README.md).')


if __name__ == '__main__':
    main()
