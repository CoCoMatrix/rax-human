"""Metadata-only SDK mocks. These tests do not open a RealSense or CAN device."""
import copy
import json
import math
from types import SimpleNamespace as NS
import unittest

from real_robot.piper.export_stereo_calibration import (
    HEAD_SERIAL, canonical_sha256, export, extrinsics_record, intrinsics_record,
)


class Profile:
    def __init__(self, stream, index, fmt):
        self.kind, self.index, self.fmt = stream, index, fmt
        self.intr = NS(width=1280, height=720, fx=644., fy=645., ppx=639., ppy=359.,
                       coeffs=[0.] * 5, model='distortion.brown_conrady')
        self.extr = NS(rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.], translation=[-0.05031, 0., 0.])

    def stream_type(self): return self.kind
    def stream_index(self): return self.index
    def format(self): return self.fmt
    def width(self): return 1280
    def height(self): return 720
    def fps(self): return 30
    def is_video_stream_profile(self): return True
    def as_video_stream_profile(self): return self
    def get_intrinsics(self): return self.intr
    def get_extrinsics_to(self, target):
        if target.kind == 'stream.color':
            return NS(rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.], translation=[0.0149, 0.0002, 0.])
        return self.extr


class Sensor:
    def __init__(self, profiles): self.profiles = profiles
    def get_stream_profiles(self): return self.profiles
    def supports(self, option): return option == 'emitter_enabled'
    def get_option(self, option): return 1.0
    def set_option(self, *args): raise AssertionError('must not write options')
    def open(self, *args): raise AssertionError('must not open sensor')
    def start(self, *args): raise AssertionError('must not start sensor')


class SDK:
    stream = NS(infrared='stream.infrared', color='stream.color')
    format = NS(y8='format.y8', bgr8='format.bgr8')
    camera_info = NS(serial_number='serial_number', name='name', firmware_version='firmware_version',
                     product_line='product_line', usb_type_descriptor='usb_type_descriptor')
    option = NS(emitter_enabled='emitter_enabled', laser_power='laser_power',
                enable_auto_exposure='enable_auto_exposure', exposure='exposure', gain='gain')
    __version__ = 'test-only-sdk'

    def __init__(self):
        self.profiles = [Profile('stream.infrared', 1, 'format.y8'),
                         Profile('stream.infrared', 2, 'format.y8'), Profile('stream.color', 0, 'format.bgr8')]
        self.sensor = Sensor(self.profiles)
        self.device = NS(supports=lambda info: True,
                         get_info=lambda info: HEAD_SERIAL if info == 'serial_number' else 'mock-' + info,
                         query_sensors=lambda: [self.sensor])
        self.devices = [self.device]

    def context(self): return NS(query_devices=lambda: self.devices)
    def pipeline(self, *args): raise AssertionError('must not create pipeline')


def instance():
    return {'robot_type': 'piper_x_dual', 'robot_id': 'piper_x_dual_01',
            'cameras': {'camera_head': {'serial': HEAD_SERIAL}}}


class StereoCalibrationTests(unittest.TestCase):
    def test_exports_exact_metadata_without_streams_or_writes(self):
        data = export(SDK(), instance())
        self.assertEqual(data['schema'], 'junqi.piper.stereo_calibration.v1')
        self.assertFalse(data['streaming_started'])
        self.assertFalse(data['sensor_options_written'])
        self.assertFalse(data['simultaneous_profile_streaming_verified'])
        self.assertFalse(data['camera_to_robot_extrinsics_included'])
        self.assertEqual(data['profiles']['ir_right']['stream_index'], 2)
        self.assertEqual(data['profiles']['color']['format'], 'format.bgr8')
        self.assertEqual(data['sensor_option_snapshot'][0]['options']['emitter_enabled'], 1.)
        self.assertEqual(data['sdk_version'], 'test-only-sdk')
        json.dumps(data, allow_nan=False)

    def test_uses_device_baseline_not_nominal_fifty_mm(self):
        sdk = SDK()
        sdk.profiles[0].extr.translation = [-0.051, 0.002, 0.003]
        data = export(sdk, instance())
        self.assertAlmostEqual(data['baseline_m'], math.sqrt(0.051**2 + 0.002**2 + 0.003**2))
        self.assertEqual(data['horizontal_baseline_m'], 0.051)

    def test_column_major_rotation_and_transform_direction(self):
        ex = NS(rotation=[0., 1., 0., -1., 0., 0., 0., 0., 1.], translation=[.1, .2, .3])
        result = extrinsics_record(ex, 'ir_left', 'color')
        self.assertEqual(result['R_target_source'], [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        self.assertEqual(result['T_target_source'][0], [0., -1., 0., .1])
        self.assertEqual(result['target_frame'], 'color')

    def test_missing_camera_rejected(self):
        sdk = SDK(); sdk.devices = []
        with self.assertRaisesRegex(RuntimeError, 'expected one camera'): export(sdk, instance())

    def test_wrong_head_identity_rejected(self):
        data = instance(); data['cameras']['camera_head']['serial'] = '123'
        with self.assertRaisesRegex(ValueError, 'serial'): export(SDK(), data)

    def test_exact_profile_missing_does_not_silently_fallback(self):
        sdk = SDK(); sdk.profiles[1].fmt = 'format.y16'
        with self.assertRaisesRegex(RuntimeError, 'ir_right.*Available profiles'): export(sdk, instance())

    def test_duplicate_matching_profile_rejected(self):
        sdk = SDK(); sdk.profiles.append(copy.copy(sdk.profiles[0]))
        with self.assertRaisesRegex(RuntimeError, 'found 2'): export(sdk, instance())

    def test_zero_baseline_rejected(self):
        sdk = SDK(); sdk.profiles[0].extr.translation = [0., 0., 0.]
        with self.assertRaisesRegex(ValueError, 'zero stereo baseline'): export(sdk, instance())

    def test_nonfinite_calibration_rejected(self):
        sdk = SDK(); sdk.profiles[0].intr.fx = math.nan
        with self.assertRaisesRegex(ValueError, 'intrinsics'): export(sdk, instance())

    def test_mismatched_intrinsic_size_rejected(self):
        profile = SDK().profiles[0]; profile.intr.width = 640
        with self.assertRaisesRegex(ValueError, 'resolution mismatch'): intrinsics_record(profile)

    def test_reflection_rejected(self):
        ex = NS(rotation=[-1., 0., 0., 0., 1., 0., 0., 0., 1.], translation=[0., 0., 0.])
        with self.assertRaisesRegex(ValueError, 'not proper'): extrinsics_record(ex, 'a', 'b')

    def test_scaled_rotation_rejected(self):
        ex = NS(rotation=[2., 0., 0., 0., 1., 0., 0., 0., 1.], translation=[0., 0., 0.])
        with self.assertRaisesRegex(ValueError, 'orthonormal'): extrinsics_record(ex, 'a', 'b')

    def test_payload_hash_order_stable_and_value_sensitive(self):
        self.assertEqual(canonical_sha256({'b': 1, 'a': 2}), canonical_sha256({'a': 2, 'b': 1}))
        self.assertNotEqual(canonical_sha256({'a': 1}), canonical_sha256({'a': 2}))


if __name__ == '__main__':
    unittest.main()
