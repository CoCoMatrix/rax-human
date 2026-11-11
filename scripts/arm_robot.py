"""Operator-only: arm or disarm robot motion for one session.

    python scripts/arm_robot.py <session dir or operator-input.json> arm [--minutes 60]
    python scripts/arm_robot.py <session> disarm
    python scripts/arm_robot.py <session> status

Arm only with a person at the emergency stop and the workspace clear. Disarming stops a
running Cartesian move within one servo period (the arm holds); the agent cannot arm
itself because its sandbox cannot write the session root.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services import arming  # noqa: E402


def session_root(path):
    path = Path(path).resolve()
    return path.parent if path.is_file() else path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('session')
    parser.add_argument('action', choices=('arm', 'disarm', 'status'))
    parser.add_argument('--minutes', type=float, help='arming expires after this long (default: until disarmed)')
    parser.add_argument('--yes', action='store_true', help='skip the e-stop confirmation prompt')
    args = parser.parse_args()
    root = session_root(args.session)
    if not (root/'operator-input.json').is_file():
        parser.error(f'{root} is not a session directory')
    if args.action == 'arm':
        if not args.yes and input('Person at the emergency stop and workspace clear? Type ARM: ').strip() != 'ARM':
            print('not armed')
            return
        arming.write(root, True, minutes=args.minutes)
    elif args.action == 'disarm':
        arming.write(root, False, reason='disarmed by the operator')
    print(json.dumps(arming.read(root), indent=1))


if __name__ == '__main__':
    main()
