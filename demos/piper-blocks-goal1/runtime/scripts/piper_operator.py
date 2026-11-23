"""Independent Piper operator disarm. Never arms, enables, starts or moves hardware.

Writes the new RAX session's disarm, then the configured hardware-owner session's
disarm, then requests cancellation through the existing bridge file protocol.
It does not acquire Frontend's dispatch lock or claim a measured physical stop.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.controller.config import Configuration  # noqa: E402
from services.franka import arming  # noqa: E402


def make_bridge(root, owner, session):
    # Import only after both disarm attempts: a missing optional robot dependency
    # must not prevent revoking the arming files.
    from services.piper.bridge import OwnerBridge
    return OwnerBridge(root, owner, session)


def disarm(config):
    if config.robot != 'piper':
        raise ValueError('This operator command requires a Piper configuration')
    options = config.backend_options
    owner = options.get('owner_session_root')
    bridge_root = options.get('bridge_root')
    result = {'action': 'disarm', 'session_root': str(config.root),
              'owner_session_root': owner, 'session_disarmed': False,
              'owner_disarmed': None if not owner else False,
              'cancel_requested': False, 'physical_stop_confirmed': False,
              'operator_recovery_required': True, 'errors': []}

    def absolute(value, name):
        if value is None:
            return None
        if not isinstance(value, str) or not value or not Path(value).is_absolute():
            result['errors'].append({'stage': 'configuration', 'type': 'ValueError',
                                     'reason': name + ' must be an absolute path'})
            return None
        return value

    owner = absolute(owner, 'owner_session_root')
    bridge_root = absolute(bridge_root, 'bridge_root')

    # Do not stop at the first failed write. The other independent stop path can
    # still revoke motion. Never require an agent response or a model API call.
    for stage, root, field in (
            ('session_disarm', config.root, 'session_disarmed'),
            ('owner_disarm', owner, 'owner_disarmed')):
        if root is None:
            continue
        try:
            arming.write(root, False, reason='independent Piper operator disarm')
            result[field] = True
        except Exception as exc:
            result['errors'].append({'stage': stage, 'type': type(exc).__name__, 'reason': str(exc)})

    if owner and bridge_root:
        try:
            bridge = make_bridge(bridge_root, owner, config.root)
            state = bridge.status()  # Checks session, fresh heartbeat and instance.
            result['bridge_phase_before_cancel'] = state['phase']
            result['bridge_instance'] = state['instance']
            request = bridge.cancel()  # Out-of-band file request, not an MCP call.
            if request.get('stop_requested') is not True:
                raise RuntimeError('Bridge did not report a cancellation request')
            result['cancel_requested'] = True
        except Exception as exc:
            result['errors'].append({'stage': 'bridge_cancel', 'type': type(exc).__name__, 'reason': str(exc)})
    elif owner or bridge_root:
        result['errors'].append({'stage': 'bridge_cancel', 'type': 'ValueError',
                                 'reason': 'Both owner_session_root and bridge_root are required for cancellation'})

    result['message'] = ('Arming revocation/cancellation results only. Physical stop has not been measured; '
                         'use the physical emergency stop when needed and verify fresh owner feedback.')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True, help='New Piper operator-input.json')
    parser.add_argument('action', choices=('disarm',))
    args = parser.parse_args(argv)
    result = disarm(Configuration.load(args.config))
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 1 if result['errors'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
