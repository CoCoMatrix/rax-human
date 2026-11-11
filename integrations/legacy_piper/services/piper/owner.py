"""Read-only connection to the existing sole Piper owner. No POST/SDK/CAN path."""
import json
import math
import time
import urllib.request
import numpy as np
import cv2

ORDER = [n for s in ('left', 'right') for n in [*(f'{s}_joint{i}' for i in range(1, 7)), f'{s}_gripper']]
SERIALS = dict(head='262522073779', left='262422071560', right='262522070284')


class OwnerReader:
    def __init__(self, address='127.0.0.1:8091'):
        if address != '127.0.0.1:8091':
            raise ValueError('Only the pinned existing owner read endpoint is supported')
        self.address = 'http://' + address
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def get(self, path, limit=4_000_000):
        if path not in ['/api/state', *[f'/api/camera/{c}.jpg' for c in SERIALS]]:
            raise PermissionError('Read endpoint not allowlisted')
        request = urllib.request.Request(self.address + path, method='GET')
        with self.http.open(request, timeout=2) as response:
            if response.geturl() != request.full_url:
                raise ValueError('Owner redirects are not accepted')
            body = response.read(limit + 1)
        if len(body) > limit:
            raise ValueError('Owner response too large')
        return body

    def state(self):
        data = json.loads(self.get('/api/state'))
        state = data['robot_state']
        age = data.get('robot_status_age_seconds')
        if type(age) not in (float, int) or not math.isfinite(age) or not 0 <= age <= 1.5:
            raise ValueError('Owner feedback is stale or has no age')
        if state.get('available') is not True or state.get('names') != ORDER:
            raise ValueError('Owner joint order or availability mismatch')
        if state.get('units') != ['rad']*6 + ['m'] + ['rad']*6 + ['m']:
            raise ValueError('Owner units mismatch')
        q = state.get('values')
        if not isinstance(q, list) or len(q) != 14 or any(type(x) not in (int, float) or not math.isfinite(x) for x in q):
            raise ValueError('Invalid measured state')
        # Status publishes at 1 Hz; validate independent CAN timing separately.
        meta = state.get('feedback_metadata', {})
        stamp = meta.get('source_timestamp_ns')
        can_age = meta.get('maximum_age_ms')
        if (type(stamp) is not int or type(can_age) not in (float, int)
                or not math.isfinite(can_age) or not 0 <= can_age <= 100):
            raise ValueError('Actual CAN feedback timing missing or invalid')
        received_age = time.time_ns() - stamp
        if received_age < 0 or received_age + math.ceil(can_age * 1e6) > 300_000_000:
            raise ValueError('Actual CAN feedback stale or future')
        return data

    def camera(self, camera):
        if camera not in SERIALS:
            raise ValueError('Unknown camera')
        before = self.state()['cameras'][camera]
        image = self.get(f'/api/camera/{camera}.jpg')
        after = self.state()['cameras'][camera]
        for metadata in (before, after):
            age = metadata.get('age_seconds')
            if (metadata.get('camera_serial') != SERIALS[camera] or metadata.get('available') is not True
                    or type(age) not in (float, int) or not math.isfinite(age) or not 0 <= age <= .5):
                raise ValueError('Camera identity/freshness failure')
        rgb = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
        if rgb is None:
            raise ValueError('Invalid camera JPEG')
        h,w = rgb.shape[:2]
        nh,nw = after['height'],after['width']
        # The existing UI intentionally scales 1280x720 to 960x540. Record it;
        # never apply native intrinsics directly to these display pixels.
        if not (0 < h <= nh and 0 < w <= nw and w*nh == h*nw):
            raise ValueError('Camera preview aspect ratio or size mismatch')
        # This endpoint does not atomically bind a JPEG to state metadata. Do not pretend it does.
        return cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB), dict(
            camera=camera, serial=SERIALS[camera], retrieved_unix_ns=time.time_ns(),
            before=before, after=after, binding='HTTP metadata bracket, not exact frame binding',
            display_shape_hw=[h,w], native_shape_hw=[nh,nw], display_to_native_scale_xy=[nw/w,nh/h],
            depth_aligned=False, pixel_to_robot_allowed=False)
