"""Write a new session configuration under the artifact root; starts nothing.

    python scripts/configure_session.py --task stack_cups --task-file task.md \
        [--robot franka] [--backend-options franka.json]
    bash scripts/start_agent.sh --config <printed path>

A Franka session needs its cameras in the backend options (see examples/franka.json).
--backend-options defaults to $RAX_BACKEND_OPTIONS, which also serves dashboard sessions.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import secrets
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.controller.config import AGENT_EFFORTS, PROFILES, PROVIDERS, Configuration  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--task', required=True, help='Short task id, e.g. stack_cups')
    parser.add_argument('--task-file', type=Path, help='Markdown task description (TASK.md body)')
    parser.add_argument('--task-text', help='Task description text, instead of --task-file')
    parser.add_argument('--robot', default='franka')
    parser.add_argument('--backend-options', type=Path, default=os.environ.get('RAX_BACKEND_OPTIONS') or None,
                        help='JSON file of backend settings (default: $RAX_BACKEND_OPTIONS)')
    parser.add_argument('--observation-profile', choices=PROFILES, default='rgbd')
    parser.add_argument('--agent-provider', choices=PROVIDERS, default='claude-login')
    parser.add_argument('--agent-effort', choices=AGENT_EFFORTS, default='xhigh')
    parser.add_argument('--agent-model')
    parser.add_argument('--session-hours', type=float, default=4.0)
    parser.add_argument('--max-model-calls', type=int, default=4000)
    parser.add_argument('--subagents', type=int, default=0)
    args = parser.parse_args()
    if args.task_file and args.task_text:
        parser.error('use --task-file or --task-text, not both')
    description = args.task_file.read_text() if args.task_file else (args.task_text or '')
    options = json.loads(args.backend_options.read_text()) if args.backend_options else {}
    if args.robot == 'franka' and not options.get('cameras'):
        parser.error('a franka session needs its cameras: pass --backend-options FILE or set '
                     'RAX_BACKEND_OPTIONS (see examples/franka.json)')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    root = artifact_root()/'sessions'/f'{stamp}_{args.task}_{secrets.token_hex(3)}'
    config = Configuration(
        root=root, task=args.task, task_description=description, robot=args.robot,
        backend_options=options,
        observation_profile=args.observation_profile, agent_provider=args.agent_provider,
        agent_effort=args.agent_effort, agent_model=args.agent_model,
        session_seconds=int(args.session_hours * 3600), max_model_calls=args.max_model_calls,
        subagents=args.subagents)
    root.mkdir(parents=True, mode=0o700)
    path = root/'operator-input.json'
    path.write_text(json.dumps(config.to_dict(), indent=2) + '\n')
    path.chmod(0o600)
    print(path)


if __name__ == '__main__':
    main()
