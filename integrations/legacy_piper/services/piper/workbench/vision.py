"""Read-only bounded-time multiview evidence. Wrist RGB is never metric depth."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib
import time
import uuid
import cv2

from .common import write
from ..depth import DepthExpiredError, DepthNotRecentError

CAMERAS = ('head', 'left', 'right')


def read_recent_depth(depth_reader):
    """Wait boundedly for an original FFS frame before sampling wrist views.

    At 2 Hz capture plus ~0.3 s processing, an arbitrary latest-result read
    can be ~0.9 s old. Do not renew its timestamp or relax bundle gates.
    """
    deadline = time.monotonic() + .65
    attempts = 0
    last_rejection = ''
    while True:
        attempts += 1
        try:
            arrays, depth = depth_reader.read(max_age_s=.55)
        except (DepthExpiredError, DepthNotRecentError) as exc:
            # Ineligible originals are rejected before expensive arrays.
            # Future/provenance faults still escape immediately.
            last_rejection = type(exc).__name__
        else:
            age = (time.time_ns() - depth['host_received_unix_ns']) / 1e9
            if 0 <= age <= .55:
                return arrays, depth
        if time.monotonic() >= deadline:
            raise ValueError(f'No sufficiently fresh FFS frame within bounded wait; attempts={attempts}; last={last_rejection}')
        time.sleep(.025)


def capture_bundle(reader_factory, depth_reader, destination):
    from ..volatile_sensors import writer
    with writer(destination):
        return _capture_bundle(reader_factory, depth_reader, destination)


def _capture_bundle(reader_factory, depth_reader, destination):
    """Bracket each preview independently and retain the original FFS head timestamp."""
    started = time.monotonic()
    arrays, depth = read_recent_depth(depth_reader)
    depth_ready = time.monotonic()
    before = reader_factory().state()
    wrists_started = time.monotonic()
    with ThreadPoolExecutor(max_workers=2) as pool:
        # Head RGB comes from the exact FFS packet below; another HTTP preview
        # is unused and needlessly competes with feedback/wrist reads.
        futures = {c: pool.submit(reader_factory().camera, c) for c in ('left', 'right')}
        views = {c: f.result() for c, f in futures.items()}
    cameras_done = time.monotonic()
    after = reader_factory().state()
    depth_done = time.monotonic()
    if before['robot_status']['generation'] != after['robot_status']['generation']:
        raise ValueError('观察过程中 owner 会话变化；丢弃整组图像')
    q0 = before['robot_state']['values']; q1 = after['robot_state']['values']
    drift = max(abs(a-b) for a, b in zip(q0, q1))
    stamps = [depth['host_received_unix_ns']]
    for c in ('left', 'right'):
        stamps.extend(views[c][1][edge]['timestamp_ns'] for edge in ('before', 'after'))
    now = time.time_ns()
    span = (max(stamps) - min(stamps)) / 1e9
    age = (now - min(stamps)) / 1e9
    if min(stamps) > now or span > .75 or age > 1.0:
        raise ValueError(f'三视角时间差过大：跨度 {span:.3f}s，最旧图 {age:.3f}s')
    ident = uuid.uuid4().hex
    folder = Path(destination) / ident
    folder.mkdir(parents=True)
    images = {}
    for c in CAMERAS:
        rgb = arrays['rgb'] if c == 'head' else views[c][0]
        ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise ValueError('相机图像编码失败：' + c)
        data = encoded.tobytes()
        (folder / f'{c}.jpg').write_bytes(data)
        images[c] = {'file': f'{c}.jpg', 'sha256': hashlib.sha256(data).hexdigest(),
                     'shape_hw': list(rgb.shape[:2]), 'metadata': depth if c == 'head' else views[c][1],
                     'metric_depth': c == 'head', 'role': {'head': '场景、目标位置、路径', 'left': '指尖、杯沿、夹持、释放', 'right': '辅助遮挡与周边检查'}[c]}
    result = {'schema': 'rax.multiview.v1', 'id': ident, 'created_ns': now, 'images': images,
              'oldest_source_ns': min(stamps), 'newest_source_ns': max(stamps), 'span_s': span,
              'measured14': q1, 'joint_drift_rad_m': drift, 'stationary': drift <= .0005,
              'owner_generation': after['robot_status']['generation'],
              'depth_sample_id': depth['sample_id'], 'depth_valid_fraction': float(arrays['valid'].mean()),
              'binding': 'exact FFS head RGB/depth; independently bracketed wrist RGB, bounded temporal span',
              'calibration': 'existing provisional head transform; no wrist metric projection',
              'motion_permit': False}
    result['timing_s'] = {'depth_wait_and_decode': depth_ready - started,
                          'state_before': wrists_started - depth_ready,
                          'wrist_reads': cameras_done - wrists_started,
                          'depth_and_state': depth_done - cameras_done,
                          'encode_and_write': time.monotonic() - depth_done}
    write(folder / 'bundle.json', result)
    write(Path(destination) / 'latest.json', {'id': ident, 'created_ns': now})
    return result, folder
