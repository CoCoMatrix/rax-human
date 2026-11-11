"""JSON clients for the Polymetis arm (:50054) and gripper (:50053) HTTP bridges.

The bridges run inside the DROID container and own the Polymetis connections; this
process only ever talks HTTP to them, so libfranka's realtime threads never live here.
"""
import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np


class BridgeError(RuntimeError):
    pass


class TargetRejected(BridgeError):
    """The bridge refused a target (joint limit, IK, another policy): a wall, not a fault."""


class _Client:
    name = 'bridge'

    def __init__(self, url, timeout=3.0):
        self.url = url.rstrip('/')
        self.timeout = float(timeout)

    def request(self, method, path, payload=None, timeout=None):
        data = None if payload is None else json.dumps(payload).encode()
        request = Request(self.url + path, data=data, method=method, headers={'Content-Type': 'application/json'})
        try:
            with urlopen(request, timeout=timeout or self.timeout) as response:
                value = json.loads(response.read())
        except HTTPError as exc:
            detail = exc.read().decode(errors='replace')[:500]
            kind = TargetRejected if exc.code == 409 else BridgeError
            raise kind(f'{self.name} bridge HTTP {exc.code}: {detail}') from exc
        except (URLError, OSError) as exc:
            raise BridgeError(f'{self.name} bridge unreachable at {self.url}: {exc}') from exc
        if isinstance(value, dict) and 'error' in value:
            raise BridgeError(f"{self.name} bridge error: {value['error']}")
        return value


class ArmBridge(_Client):
    name = 'arm'

    def state(self):
        value = self.request('GET', '/state')
        return {key: np.asarray(value[key], dtype=np.float64) if isinstance(value[key], list) else value[key]
                for key in value}

    def set_target(self, position, quaternion):
        """Link8 (flange) target; the bridge rate-limits and IK-checks it."""
        return self.request('POST', '/target', {'position': np.asarray(position, float).tolist(),
                                                'quaternion': np.asarray(quaternion, float).tolist()})

    def hold(self):
        return self.request('POST', '/hold', {})

    def move_joints(self, positions, time_to_go):
        return self.request('POST', '/move_joints', {'positions': np.asarray(positions, float).tolist(),
                                                     'time_to_go': float(time_to_go)},
                            timeout=float(time_to_go) + 30.0)


class GripperBridge(_Client):
    name = 'gripper'

    def state(self):
        return self.request('GET', '/state')

    def move(self, width, speed, force=0.1, blocking=True):
        return self.request('POST', '/move', {'width': float(width), 'speed': float(speed), 'force': float(force),
                                              'blocking': bool(blocking)}, timeout=30.0)

    def grasp(self, width, speed, force, blocking=True):
        return self.request('POST', '/grasp', {'width': float(width), 'speed': float(speed), 'force': float(force),
                                               'blocking': bool(blocking)}, timeout=30.0)
