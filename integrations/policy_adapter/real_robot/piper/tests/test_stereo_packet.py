import copy
from types import SimpleNamespace as NS
import numpy as np
import pytest

from real_robot.piper.export_stereo_calibration import canonical_sha256
from real_robot.piper.stereo_packet import capture_head_packet, validate_packet, PacketSequenceGuard


def sample():
    profiles = {}
    for role, stream, index, encoding in (('ir_left', 'infrared', 1, 'y8'),
            ('ir_right', 'infrared', 2, 'y8'), ('color', 'color', 0, 'bgr8')):
        profiles[role] = {'stream': 'stream.' + stream, 'stream_index': index,
                         'format': 'format.' + encoding, 'width': 64, 'height': 32, 'fps': 30}
    calibration = {'schema': 'junqi.piper.stereo_calibration.v1', 'role': 'camera_head',
                   'serial': '123', 'profiles': profiles}
    calibration['calibration_payload_sha256'] = canonical_sha256(calibration)
    frames = {}
    for role, p in profiles.items():
        profile = NS(**{k: (lambda val=v: val) for k, v in p.items() if k not in ('stream',)})
        profile.stream_type = lambda val=p['stream']: val
        profile.as_video_stream_profile = lambda val=profile: val
        shape = (32, 64, 3) if role == 'color' else (32, 64)
        pixels = np.zeros(shape, dtype=np.uint8)
        if role == 'color': pixels[..., 0] = 7; pixels[..., 2] = 19
        frames[role] = NS(get_profile=lambda val=profile: val,
            get_data=lambda val=pixels: val, get_timestamp=lambda: 1000.0,
            get_frame_number=lambda: 10, get_frame_timestamp_domain=lambda: 'hardware_clock')
    frameset = NS(get_infrared_frame=lambda index: frames['ir_left' if index == 1 else 'ir_right'],
                  get_color_frame=lambda: frames['color'])
    arrays, meta = capture_head_packet(frameset, calibration, serial='123', owner_generation='synthetic_epoch',
                                       host_received_unix_ns=1_000_000_000_000)
    return arrays, meta, calibration


def test_capture_copies_bgr_to_rgb_and_read_only_buffers():
    arrays, meta, calib = sample()
    assert arrays['rgb'][0, 0].tolist() == [19, 0, 7]
    assert not arrays['rgb'].flags.writeable
    status = validate_packet(arrays, meta, calib, mode='live', now_ns=1_000_050_000_000)
    assert status['age_ms'] == 50.0
    assert status['ir_skew_ms'] == 0


@pytest.mark.parametrize('mutation,reason', [
    (lambda m: m.update(serial='456'), 'serial mismatch'),
    (lambda m: m.update(calibration_payload_sha256='0'*64), 'calibration/serial'),
    (lambda m: m['frames']['ir_right'].update(frame_number=11), 'counters'),
    (lambda m: m['frames']['ir_right'].update(device_timestamp_ns=1_005_000_000), 'paired time skew'),
    (lambda m: m['frames']['color'].update(device_timestamp_ns=1_035_000_000), 'RGB frame'),
    (lambda m: m['frames']['color'].update(timestamp_domain='global_time'), 'different domains'),
    (lambda m: m.update(rgb_encoding='bgr8'), 'channel order'),
])
def test_wrong_pair_or_identity_rejected(mutation, reason):
    arrays, meta, calib = sample(); mutation(meta)
    with pytest.raises(ValueError, match=reason): validate_packet(arrays, meta, calib, mode='offline_replay')


def test_stale_and_future_packets_not_accepted_live():
    arrays, meta, calib = sample()
    with pytest.raises(ValueError, match='stale'):
        validate_packet(arrays, meta, calib, mode='live', now_ns=1_002_000_000_000)
    with pytest.raises(ValueError, match='future'):
        validate_packet(arrays, meta, calib, mode='live', now_ns=1)
    assert validate_packet(arrays, meta, calib, mode='offline_replay')['freshness_status'] == 'historical_not_live'


def test_calibration_modification_rejected():
    arrays, meta, calib = sample(); calib['serial'] = 'changed'
    with pytest.raises(ValueError, match='checksum'):
        validate_packet(arrays, meta, calib, mode='offline_replay')


def test_cached_frame_cannot_become_fresh_by_rewrapping():
    _, meta, _ = sample()
    guard = PacketSequenceGuard(meta['owner_generation']); guard.accept(meta)
    replay = copy.deepcopy(meta)
    replay['sample_id'] = 'new_uuid_does_not_make_frame_new'
    replay['host_received_unix_ns'] += 1_000_000
    with pytest.raises(ValueError, match='cached'):
        guard.accept(replay)
    replay['owner_generation'] = 'other_epoch'
    with pytest.raises(ValueError, match='generation changed'):
        guard.accept(replay)


def test_wrong_layout_and_wrong_stream_index_rejected():
    arrays, meta, calib = sample(); arrays['rgb'] = np.zeros((16, 32, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match='shape'):
        validate_packet(arrays, meta, calib, mode='offline_replay')
    arrays, meta, calib = sample(); calib['profiles']['ir_left']['stream_index'] = 2
    calib.pop('calibration_payload_sha256')
    calib['calibration_payload_sha256'] = canonical_sha256(calib)
    with pytest.raises(ValueError, match='stream order'):
        validate_packet(arrays, meta, calib, mode='offline_replay')
