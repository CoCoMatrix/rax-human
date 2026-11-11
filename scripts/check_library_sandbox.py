#!/usr/bin/env python3
"""Offline Linux check of the real Codex bubblewrap command. No model or robot service.

Only creates temporary mock workspaces and git repositories. Socket mounts are dummy
files, never live endpoints. A failed check leaves a nonzero exit status.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.controller.config import Configuration
from services import skill_library
from harness.codex_cli.launcher import prepare, sandbox_command


def main():
    with tempfile.TemporaryDirectory(prefix='rax-library-sandbox-') as folder:
        base = Path(folder)
        os.environ['RAX_ARTIFACT_ROOT'] = str(base)
        config = Configuration(root=base/'sessions/check', robot='mock', task='offline',
                               agent_provider='codex-relay')
        ws = prepare(config, base/'unused-key.py', 'https://example.invalid/v1')
        for name in ('robot', 'model'):
            (base/name).touch()  # regular files; no network or socket server
        command = sandbox_command(config, base/'robot', base/'model', prompt='unused')
        command = command[:command.index('--chdir')+2]
        library = skill_library.library_path(config)
        probe = r'''
import json, os, pathlib, subprocess
library = pathlib.Path(os.environ['CHECK_LIBRARY'])
clone = pathlib.Path('skill-library')
pathlib.Path('output/writable').write_text('offline')
read_only = False
try:
    (library/'must-not-write').write_text('bad')
except OSError:
    read_only = True
fetch = subprocess.run(['git','-C',str(clone),'fetch','origin'], capture_output=True)
push = subprocess.run(['git','-C',str(clone),'push','origin','HEAD:refs/heads/forbidden'], capture_output=True)
result = {'workspace_writable': True, 'origin_read_only': read_only,
          'fetch_ok': fetch.returncode == 0, 'push_refused': push.returncode != 0,
          'model_started': False, 'hardware_connected': False}
print(json.dumps(result))
assert all(result[k] for k in ('workspace_writable','origin_read_only','fetch_ok','push_refused')), result
'''
        # Environment is explicitly allowlisted by the same real launcher.
        command[command.index('--chdir'):command.index('--chdir')] = [
            '--setenv', 'CHECK_LIBRARY', str(library)]
        result = subprocess.run([*command, '/usr/bin/python3', '-c', probe],
                                text=True, capture_output=True, timeout=30)
        if result.returncode:
            raise RuntimeError(result.stderr + result.stdout)
        print(result.stdout.strip())


if __name__ == '__main__':
    main()
