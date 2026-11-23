"""Pure FK/IK/collision service for the dry execution: JSON lines on stdin/stdout. NoOwner backend, no hardware."""
import json, os, sys, tempfile
from pathlib import Path
R, RUN = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(R))
from services.controller.config import Configuration
from services.piper.backend import PiperBackend
from services.piper.installed_collision import load_checker
opts = json.loads((RUN/'backend-options.json').read_text())


class NoOwner:
    def state(self): raise AssertionError('no hardware')


backend = PiperBackend(Configuration(root=Path(tempfile.mkdtemp(dir=os.environ['RAX_ARTIFACT_ROOT'])), task='parallel_math', robot='piper', backend_options=opts), owner=NoOwner(), cameras=object())
backend.solver.start(); backend.collision = load_checker(opts['collision_legacy_root'])
PURE = ('piper_fk', 'piper_fk_batch', 'piper_solve_ik', 'piper_solve_ik_fast', 'piper_solve_ik_candidates', 'piper_check_path')
out = os.fdopen(os.dup(1), 'w'); os.dup2(2, 1)   # keep library prints away from the protocol stream
out.write(json.dumps({'ready': True}) + '\n'); out.flush()
for line in sys.stdin:
    request = json.loads(line)
    try:
        assert request['tool'] in PURE, request['tool']
        reply = {'result': backend.call(request['tool'], request['args'])[0]}
    except Exception as exc:  # noqa: BLE001 - reported to the caller with its type
        reply = {'error': {'type': type(exc).__name__, 'message': str(exc)}}
    out.write(json.dumps(reply) + '\n'); out.flush()
