"""Piper dashboard reads only the existing owner; no camera, SDK or motion client."""
import time


class Telemetry:
    def __init__(self, reader=None):
        self.reader = reader

    def snapshot(self):
        if self.reader is None:
            from .owner import OwnerReader
            self.reader = OwnerReader()
        data = self.reader.state()
        q = data['robot_state']['values']
        return {'kind': 'piper', 'ok': True, 'time': time.time(),
                'joints': {'left': q[:6], 'right': q[7:13]},
                'grippers': {'left_width_m': q[6], 'right_width_m': q[13]},
                'status': data.get('robot_status', {}),
                'age_seconds': data.get('robot_status_age_seconds')}
