"""Read stored camera calibration for exact profiles, without starting streams.

Run with the personal xpolicy camera interpreter. This script never opens CAN,
creates a RealSense pipeline, changes sensor options, or writes device EEPROM.
The exported intrinsics are device-stored calibration, not a fresh calibration.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

ROLES = ('camera_head', 'camera_left', 'camera_right')


def intrinsics_record(profile):
    intr = profile.get_intrinsics()
    numbers = [intr.fx, intr.fy, intr.ppx, intr.ppy, *intr.coeffs]
    if not all(math.isfinite(x) for x in numbers) or intr.fx <= 0 or intr.fy <= 0:
        raise ValueError('invalid camera intrinsics')
    if (intr.width, intr.height) != (profile.width(), profile.height()):
        raise ValueError('intrinsics/profile resolution mismatch')
    return {'width': intr.width, 'height': intr.height, 'fps': profile.fps(),
            'format': str(profile.format()), 'stream': str(profile.stream_type()),
            'stream_index': profile.stream_index(),
            'K': [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]],
            'distortion_model': str(intr.model), 'distortion_coeffs': list(intr.coeffs),
            'pixel_frame': 'original_full_resolution_not_ui_preview',
            'optical_axes': 'x_right_y_down_z_forward'}


def export(rs, instance, *, width=1280, height=720, fps=30):
    if instance.get('robot_type') != 'piper_x_dual':
        raise ValueError('expected a PiPER owner device instance')
    serials = {role: str(instance['cameras'][role]['serial']) for role in ROLES}
    if len(set(serials.values())) != 3 or not all(s.isdigit() for s in serials.values()):
        raise ValueError('three distinct explicit camera serials are required')
    context = rs.context()
    devices = {dev.get_info(rs.camera_info.serial_number): dev
               for dev in context.query_devices() if dev.supports(rs.camera_info.serial_number)}
    missing = set(serials.values()) - set(devices)
    if missing:
        raise RuntimeError('required camera serials not found: ' + ','.join(sorted(missing)))
    cameras = {}
    for role, serial in serials.items():
        dev = devices[serial]
        profiles = [p.as_video_stream_profile() for sensor in dev.query_sensors()
                    for p in sensor.get_stream_profiles() if p.is_video_stream_profile()]
        matches = [p for p in profiles if p.stream_type() == rs.stream.color
                   and p.format() == rs.format.bgr8
                   and (p.width(), p.height(), p.fps()) == (width, height, fps)]
        if len(matches) != 1:
            raise RuntimeError(f'{role}: expected exactly one {width}x{height}@{fps} BGR profile; found {len(matches)}')
        color = matches[0]
        cameras[role] = {'serial': serial, 'color': intrinsics_record(color),
                         'source': 'device_stored_calibration',
                         'camera_to_robot_extrinsics_included': False}
        for key in ('name', 'firmware_version', 'product_line'):
            info = getattr(rs.camera_info, key)
            if dev.supports(info):
                cameras[role][key] = dev.get_info(info)
    return {'schema': 'junqi.piper.camera_intrinsics.v1', 'robot_id': instance['robot_id'],
            'captured_utc': datetime.now(timezone.utc).isoformat(),
            'sdk_version': getattr(rs, '__version__', 'unavailable'),
            'source': 'device_stored_calibration', 'fresh_calibration_performed': False,
            'hardware_access': 'camera_calibration_metadata_only',
            'streaming_started': False, 'robot_sdk_used': False, 'cameras': cameras}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-instance', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('use a new output path to retain previous calibration')
    import yaml
    import pyrealsense2 as rs
    data = args.device_instance.read_bytes()
    result = export(rs, yaml.safe_load(data))
    result['device_instance_sha256'] = hashlib.sha256(data).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({'output': str(args.output), 'camera_count': len(result['cameras']),
                      'streaming_started': False, 'status': 'exported'}))


if __name__ == '__main__':
    main()
