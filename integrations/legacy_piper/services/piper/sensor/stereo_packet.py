"""Read a head-camera frameset already owned by xpolicy; never open a camera.

The live owner hook remains to be attached after its actual source is available.
Device timestamps are compared only within one common timestamp domain; host
receive timestamps bound age in the same local host. No robot action is emitted.
"""
import time
import uuid
import numpy as np

from .contracts import require
from .export_stereo_calibration import canonical_sha256, profile_record


def calibration_identity(calibration):
    require(calibration.get('schema') == 'junqi.piper.stereo_calibration.v1'
            and calibration.get('role') in ('camera_head','camera_left'), 'expected configured stereo camera calibration')
    payload = {k: v for k, v in calibration.items() if k != 'calibration_payload_sha256'}
    expected = canonical_sha256(payload)
    require(calibration.get('calibration_payload_sha256') == expected,
            'camera calibration payload checksum mismatch')
    profiles = calibration.get('profiles', {})
    require(set(profiles) == {'ir_left', 'ir_right', 'color'}, 'missing stereo camera profiles')
    for role, stream, index, encoding in (('ir_left', 'infrared', 1, 'y8'),
            ('ir_right', 'infrared', 2, 'y8'), ('color', 'color', 0, 'bgr8')):
        p = profiles[role]
        require(str(p.get('stream')).split('.')[-1] == stream
                and p.get('stream_index') == index
                and str(p.get('format')).split('.')[-1] == encoding,
                'wrong stereo stream order or image encoding')
    return expected


def capture_head_packet(frameset, calibration, *, serial, owner_generation, host_received_unix_ns):
    """Call inside the one existing camera owner immediately upon frame receipt.

    Caller binds the frameset to its selected device serial. Profiles are checked
    again here, but frames alone cannot attest the originating device serial.
    """
    identity = calibration_identity(calibration)
    require(serial == calibration['serial'], 'camera owner serial/calibration mismatch')
    frames = {'ir_left': frameset.get_infrared_frame(1),
              'ir_right': frameset.get_infrared_frame(2), 'color': frameset.get_color_frame()}
    arrays, records = {}, {}
    for role, frame in frames.items():
        require(bool(frame), f'missing {role} frame')
        actual = profile_record(frame.get_profile().as_video_stream_profile())
        expected = calibration['profiles'][role]
        require(all(actual[k] == expected[k] for k in actual), f'{role} frame/profile mismatch')
        pixels = np.asanyarray(frame.get_data())
        shape = (actual['height'], actual['width']) + ((3,) if role == 'color' else ())
        require(pixels.dtype == np.uint8 and pixels.shape == shape, f'{role} image layout mismatch')
        key = 'rgb' if role == 'color' else role
        arrays[key] = np.array(pixels[..., ::-1] if role == 'color' else pixels, copy=True)
        arrays[key].setflags(write=False)
        records[role] = {'device_timestamp_ns': int(round(frame.get_timestamp() * 1_000_000)),
                         'frame_number': int(frame.get_frame_number()),
                         'timestamp_domain': str(frame.get_frame_timestamp_domain()),
                         'profile': actual}
    require(isinstance(owner_generation, str) and owner_generation, 'explicit camera owner generation required')
    received = host_received_unix_ns
    require(type(received) is int and received > 0, 'preserve actual frame receipt timestamp')
    metadata = {'schema': 'junqi.piper.head_stereo_packet.v1', 'sample_id': uuid.uuid4().hex,
                'serial': serial, 'camera_role': calibration['role'],
                'calibration_payload_sha256': identity, 'host_received_unix_ns': received,
                'owner_generation': owner_generation,
                'frames': records, 'rgb_encoding': 'rgb8',
                'capture_source': 'existing_xpolicy_camera_owner_frameset',
                'serial_binding_source': 'owner_selected_device', 'robot_action_included': False}
    return arrays, metadata


def validate_packet(arrays, metadata, calibration, *, mode, now_ns=None,
                    max_age_ms=1000.0, max_ir_skew_ms=2.0, max_rgb_skew_ms=34.0):
    """Draft timing limits must be checked against real D435 recordings on host."""
    require(mode in ('live', 'offline_replay'), 'explicit live/offline_replay mode required')
    require(all(type(x) in (float, int) and np.isfinite(x) and x > 0
                for x in (max_age_ms, max_ir_skew_ms, max_rgb_skew_ms)), 'invalid timing limits')
    identity = calibration_identity(calibration)
    require(metadata.get('schema') == 'junqi.piper.head_stereo_packet.v1'
            and metadata.get('camera_role') == calibration['role']
            and isinstance(metadata.get('sample_id'), str) and metadata['sample_id'], 'invalid packet identity')
    require(isinstance(metadata.get('owner_generation'), str) and metadata['owner_generation'],
            'missing camera owner generation')
    require(metadata.get('serial') == calibration['serial']
            and metadata.get('calibration_payload_sha256') == identity, 'packet calibration/serial mismatch')
    require(metadata.get('rgb_encoding') == 'rgb8', 'RGB channel order must be explicit')
    require(set(arrays) == {'ir_left', 'ir_right', 'rgb'}, 'exactly one head IR pair and RGB required')
    frames = metadata.get('frames', {})
    require(set(frames) == {'ir_left', 'ir_right', 'color'}, 'missing frame timing metadata')
    domains, timestamps = set(), {}
    for role, key in (('ir_left', 'ir_left'), ('ir_right', 'ir_right'), ('color', 'rgb')):
        profile = calibration['profiles'][role]
        record = frames[role]
        actual = record.get('profile', {})
        require(all(actual.get(k) == profile[k] for k in
                    ('stream', 'stream_index', 'format', 'width', 'height', 'fps')), 'frame/profile identity differs')
        shape = (profile['height'], profile['width']) + ((3,) if key == 'rgb' else ())
        require(isinstance(arrays[key], np.ndarray) and arrays[key].dtype == np.uint8
                and arrays[key].shape == shape, 'image shape/dtype differs from camera calibration')
        stamp, number = record.get('device_timestamp_ns'), record.get('frame_number')
        require(type(stamp) is int and stamp >= 0 and type(number) is int and number >= 0,
                'invalid device timestamp or frame number')
        domain = record.get('timestamp_domain')
        require(isinstance(domain, str) and bool(domain), 'missing timestamp domain')
        domains.add(domain); timestamps[role] = stamp
    require(len(domains) == 1, 'device timestamps from different domains cannot be subtracted')
    require(frames['ir_left']['frame_number'] == frames['ir_right']['frame_number'], 'IR frame counters differ')
    ir_skew = abs(timestamps['ir_left'] - timestamps['ir_right']) / 1e6
    rgb_skew = max(abs(timestamps['color'] - timestamps[role]) / 1e6 for role in ('ir_left', 'ir_right'))
    require(ir_skew <= max_ir_skew_ms, 'IR frames exceed paired time skew')
    require(rgb_skew <= max_rgb_skew_ms, 'RGB frame does not match IR pair time')
    received = metadata.get('host_received_unix_ns')
    require(type(received) is int and received > 0, 'missing host receive timestamp')
    age_ms = None
    if mode == 'live':
        now = time.time_ns() if now_ns is None else now_ns
        require(type(now) is int and now >= received, 'invalid/future host timestamp')
        age_ms = (now - received) / 1e6
        require(age_ms <= max_age_ms, 'stereo packet is stale')
    return {'mode': mode, 'age_ms': age_ms, 'ir_skew_ms': ir_skew, 'rgb_skew_ms': rgb_skew,
            'freshness_status': 'host_receive_age_checked' if mode == 'live' else 'historical_not_live',
            'timing_limits_status': 'initial_configuration_requires_live_validation'}


class PacketSequenceGuard:
    """One instance per camera-owner generation, at the capture hook and worker.

    A new pipeline generation requires a new guard. Call only for SDK-reported
    new frames; rewrapping a cached frame must never reset its receipt time.
    """
    def __init__(self, owner_generation):
        require(isinstance(owner_generation, str) and owner_generation, 'missing owner generation')
        self.generation = owner_generation
        self.last_number = self.last_device_ns = self.last_receive_ns = -1

    def accept(self, metadata):
        require(metadata.get('owner_generation') == self.generation, 'camera owner generation changed')
        frames = metadata['frames']
        number = frames['ir_left']['frame_number']
        stamp = frames['ir_left']['device_timestamp_ns']
        received = metadata['host_received_unix_ns']
        require(number == frames['ir_right']['frame_number']
                and number > self.last_number and stamp > self.last_device_ns
                and received > self.last_receive_ns, 'duplicate, reordered or cached stereo frame')
        self.last_number, self.last_device_ns, self.last_receive_ns = number, stamp, received
