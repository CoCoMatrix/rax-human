"""Export device-stored selected-camera stereo calibration without opening any stream.

Only context/device/profile metadata and sensor.get_option are read. No pipeline,
sensor.open/start, set_option, robot SDK, reset, or EEPROM write is used. Run this
after reconnecting the robot host, before the camera owner starts. Enumeration
does not prove that all selected profiles can stream concurrently over USB.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

LEFT_SERIAL = '262422071560'


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def profile_record(profile):
    return {'stream': str(profile.stream_type()), 'stream_index': profile.stream_index(),
            'format': str(profile.format()), 'width': profile.width(),
            'height': profile.height(), 'fps': profile.fps()}


def intrinsics_record(profile):
    record = profile_record(profile)
    intr = profile.get_intrinsics()
    values = [intr.fx, intr.fy, intr.ppx, intr.ppy, *intr.coeffs]
    if len(intr.coeffs) != 5 or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid intrinsics coefficients')
    if intr.fx <= 0 or intr.fy <= 0 or (intr.width, intr.height) != (profile.width(), profile.height()):
        raise ValueError('invalid focal length or profile resolution mismatch')
    record.update(K=[[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]],
                  distortion_model=str(intr.model), distortion_coeffs=list(intr.coeffs),
                  optical_axes='x_right_y_down_z_forward', pixel_units='pixels',
                  pixel_frame='original_full_resolution_not_ui_preview')
    return record


def extrinsics_record(extrinsics, source, target):
    """RealSense flat rotation is column-major; output matrices are nested rows."""
    flat, translation = list(extrinsics.rotation), list(extrinsics.translation)
    if len(flat) != 9 or len(translation) != 3 or not all(math.isfinite(v) for v in flat + translation):
        raise ValueError('invalid extrinsic dimensions or nonfinite values')
    rotation = [[flat[row + 3 * col] for col in range(3)] for row in range(3)]
    for i in range(3):
        for j in range(3):
            value = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            if abs(value - (1 if i == j else 0)) > 1e-5:
                raise ValueError('extrinsic rotation is not orthonormal')
    a, b, c = rotation
    det = (a[0] * (b[1] * c[2] - b[2] * c[1])
           - a[1] * (b[0] * c[2] - b[2] * c[0])
           + a[2] * (b[0] * c[1] - b[1] * c[0]))
    if abs(det - 1) > 1e-5:
        raise ValueError('extrinsic rotation is not proper')
    return {'source_frame': source, 'target_frame': target,
            'convention': 'p_target = R_target_source @ p_source + t_target_source',
            'R_target_source': rotation, 't_target_source_m': translation,
            'T_target_source': [rotation[i] + [translation[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]],
            'sdk_rotation_storage': 'column_major', 'serialized_matrix_storage': 'nested_rows',
            'translation_units': 'metres'}


def sensor_option_snapshot(rs, sensors):
    """Emitter state is evidence only: never alter projection or exposure options."""
    records = []
    for index, sensor in enumerate(sensors):
        options = {}
        for name in ('emitter_enabled', 'laser_power', 'enable_auto_exposure', 'exposure', 'gain'):
            option = getattr(rs.option, name, None)
            if option is None or not sensor.supports(option):
                continue
            try:
                value = float(sensor.get_option(option))
                options[name] = value if math.isfinite(value) else {'read_error': 'nonfinite_value'}
            except RuntimeError as error:
                options[name] = {'read_error': str(error)}
        records.append({'sensor_index': index, 'options': options})
    return records


def export(rs, instance, *, expected_serial=LEFT_SERIAL, width=1280, height=720, fps=30):
    if instance.get('robot_type') != 'piper_x_dual':
        raise ValueError('expected PiPER dual owner device instance')
    serial = str(instance['cameras']['camera_left']['serial'])
    if not serial.isdigit() or serial != str(expected_serial):
        raise ValueError('left serial differs from explicit expected serial')
    if any(type(v) is not int or v <= 0 for v in (width, height, fps)):
        raise ValueError('exact positive integer profile dimensions and fps required')
    devices = [device for device in rs.context().query_devices()
               if device.supports(rs.camera_info.serial_number)
               and device.get_info(rs.camera_info.serial_number) == serial]
    if len(devices) != 1:
        raise RuntimeError(f'expected one camera with serial {serial}; found {len(devices)}')
    device = devices[0]
    sensors = list(device.query_sensors())
    profiles = [p.as_video_stream_profile() for s in sensors for p in s.get_stream_profiles()
                if p.is_video_stream_profile()]
    inventory = [profile_record(p) for p in profiles]
    requested = {'ir_left': (rs.stream.infrared, 1, rs.format.y8),
                 'ir_right': (rs.stream.infrared, 2, rs.format.y8),
                 'color': (rs.stream.color, 0, rs.format.bgr8)}
    selected = {}
    for role, (stream, index, fmt) in requested.items():
        matches = [p for p in profiles if p.stream_type() == stream and p.stream_index() == index
                   and p.format() == fmt and (p.width(), p.height(), p.fps()) == (width, height, fps)]
        if len(matches) != 1:
            raise RuntimeError(f'{role}: expected exactly one {width}x{height}@{fps} {fmt} '
                               f'index={index}; found {len(matches)}. Available profiles: '
                               + json.dumps(inventory, sort_keys=True))
        selected[role] = matches[0]
    ir_to_ir = extrinsics_record(selected['ir_left'].get_extrinsics_to(selected['ir_right']), 'ir_left', 'ir_right')
    ir_to_rgb = extrinsics_record(selected['ir_left'].get_extrinsics_to(selected['color']), 'ir_left', 'color')
    translation = ir_to_ir['t_target_source_m']
    baseline = math.sqrt(sum(v * v for v in translation))
    if baseline <= 0:
        raise ValueError('zero stereo baseline')
    result = {'schema': 'junqi.piper.stereo_calibration.v1', 'robot_id': instance['robot_id'],
              'role': 'camera_left', 'serial': serial,
              'captured_utc': datetime.now(timezone.utc).isoformat(),
              'sdk_version': getattr(rs, '__version__', 'unavailable'),
              'source': 'device_stored_calibration', 'fresh_calibration_performed': False,
              'hardware_access': 'camera_calibration_metadata_only',
              'streaming_started': False, 'sensor_options_written': False, 'robot_sdk_used': False,
              'simultaneous_profile_streaming_verified': False,
              'camera_to_robot_extrinsics_included': False,
              'profiles': {role: intrinsics_record(p) for role, p in selected.items()},
              'supported_profiles': inventory,
              'extrinsics': {'ir_left_to_ir_right': ir_to_ir, 'ir_left_to_color': ir_to_rgb},
              'baseline_m': baseline, 'horizontal_baseline_m': abs(translation[0]),
              'baseline_source': 'norm_of_device_stored_ir_left_to_ir_right_translation',
              'sensor_option_snapshot': sensor_option_snapshot(rs, sensors)}
    for name in ('name', 'firmware_version', 'product_line', 'usb_type_descriptor'):
        info = getattr(rs.camera_info, name, None)
        if info is not None and device.supports(info):
            result[name] = device.get_info(info)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-instance', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-left-serial', default=LEFT_SERIAL)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('choose a new output to retain previous calibration')
    import yaml
    import pyrealsense2 as rs
    data = args.device_instance.read_bytes()
    result = export(rs, yaml.safe_load(data), expected_serial=args.expected_left_serial,
                    width=args.width, height=args.height, fps=args.fps)
    result['device_instance_sha256'] = hashlib.sha256(data).hexdigest()
    result['calibration_payload_sha256'] = canonical_sha256(result)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output': str(args.output), 'serial': result['serial'],
                      'baseline_m': result['baseline_m'], 'streaming_started': False,
                      'calibration_payload_sha256': result['calibration_payload_sha256']}))


if __name__ == '__main__':
    main()
