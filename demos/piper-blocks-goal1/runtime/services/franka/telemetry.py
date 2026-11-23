"""Read-only Franka dashboard adapter. No control requests."""
import json
import time
from urllib.request import urlopen, build_opener, ProxyHandler

ARM_URL = 'http://127.0.0.1:50054'
GRIPPER_URL = 'http://127.0.0.1:50053'


def _http_get(url, timeout=1):
    with build_opener(ProxyHandler({})).open(url, timeout=timeout) as response:
        return json.loads(response.read())


class Telemetry:
    def __init__(self, http_get=None):
        self.http_get = http_get or _http_get

    def snapshot(self):
        from services.franka.kinematics import link8_to_tcp
        value = {'kind': 'franka', 'time': time.time()}
        try:
            arm = self.http_get(ARM_URL + '/state')
            tcp_p, tcp_q = link8_to_tcp(arm['arm_pos'], arm['arm_quat'])
            value['arm'] = {'ok': True, 'tcp_position': [round(float(v), 4) for v in tcp_p],
                            'tcp_quaternion': [round(float(v), 4) for v in tcp_q],
                            'joints': [round(float(v), 4) for v in arm['arm_joint_position']],
                            'policy_running': bool(arm.get('policy_running')),
                            'backend': arm.get('controller_backend')}
        except Exception as exc:
            value['arm'] = {'ok': False, 'error': str(exc)}
        try:
            grip = self.http_get(GRIPPER_URL + '/state')
            value['gripper'] = {'ok': not grip.get('stale'), **{k: grip.get(k) for k in
                                ('width', 'max_width', 'is_grasped', 'is_moving', 'stale', 'age')}}
        except Exception as exc:
            value['gripper'] = {'ok': False, 'error': str(exc)}
        value['ok'] = value['arm']['ok'] and value['gripper']['ok']
        return value

