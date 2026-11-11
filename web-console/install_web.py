#!/usr/bin/env python3
"""Install only the three Web files into an existing Piper RAX runtime, with backup."""
from pathlib import Path
import argparse
import fcntl
import hashlib
import json
import os
import shutil
import time

HERE = Path(__file__).resolve().parent
FILES = ('services/dashboard/piper_scale.py', 'services/dashboard/static/scale.html',
         'services/dashboard/static/scale.js')
REQUIRED = ('services/dashboard/server.py', 'services/piper/owner.py',
            'scripts/piper_preview.py', 'scripts/piper_prepare.py', 'scripts/piper_explore.py',
            'scripts/piper_minimal_live.py', 'examples/blue-cup-exploration.json',
            'workspaces/blue-cup/20261109-v0.4-merged-candidate/run_no_agent.py',
            'workspaces/blue-cup/20261109-v0.4-merged-candidate/WORKSPACE.json')


def validate(root, source):
    missing = [name for name in REQUIRED if not (root / name).is_file()]
    if missing:
        raise ValueError('Requires the existing Piper RAX runtime, missing: ' + ', '.join(missing))
    config = json.loads((root / 'examples/blue-cup-exploration.json').read_text())
    if config.get('workspace_min') != [.055, -.4, .17] or config.get('workspace_max') != [.5, .045, .36]:
        raise ValueError('This Web snapshot displays the rc10 workspace; configure/verify the backend first (X 55..500 mm)')
    for name in FILES:
        data = (source / name).read_bytes()
        if name.endswith('.py'):
            compile(data, name, 'exec')
    return {name: hashlib.sha256((source / name).read_bytes()).hexdigest() for name in FILES}


def install(root, source=HERE / 'runtime-overlay', check=False):
    root, source = Path(root).resolve(), Path(source).resolve()
    manifest = validate(root, source)
    if check:
        return {'check_only': True, 'files': manifest}
    runtime = root / 'runtime/scale-tests'
    runtime.mkdir(parents=True, exist_ok=True)
    # The existing service holds this lock for its entire life. Never restart it
    # or write over an active control session during installation.
    with (runtime / 'server.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Web service is active; finish/stop through its native controls before installing') from None
        backup = root / 'backups/web-console' / (time.strftime('%Y%m%d-%H%M%S') + '-' + str(time.time_ns()))
        backup.mkdir(parents=True)
        previous = {}
        for name in FILES:
            destination = root / name
            if destination.exists():
                saved = backup / name
                saved.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(destination, saved)
                previous[name] = hashlib.sha256(destination.read_bytes()).hexdigest()
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = destination.with_name(destination.name + '.install-tmp')
            shutil.copy2(source / name, temporary)
            os.replace(temporary, destination)
        (backup / 'SHA256SUMS.json').write_text(json.dumps(previous, indent=2) + '\n')
        assert all(hashlib.sha256((root / name).read_bytes()).hexdigest() == sha for name, sha in manifest.items())
        return {'backup': str(backup), 'installed': manifest, 'hardware_commands': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default='/home/ubuntu/code_junqi/rax-piper-franka-20261104')
    parser.add_argument('--check', action='store_true', help='Check prerequisites without changing files')
    args = parser.parse_args()
    print(json.dumps(install(args.root, check=args.check), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
