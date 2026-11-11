"""Offline tests: v12 estimate regression and no-stream SDK metadata contract."""
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np

from real_robot.piper.calibration_nominal import head_estimate, _rotation
from real_robot.piper.description_config import mast_configuration
from real_robot.piper.export_realsense_intrinsics import export, intrinsics_record

ROOT = Path(__file__).resolve().parents[3]
ASSETS = ROOT / 'assets/robots/piper/global-d435-fixed-v12'


class Profile:
    def __init__(self, width=1280, height=720, fps=30):
        self.w, self.h, self.f = width, height, fps
        self.intr = NS(width=width, height=height, fx=600.0, fy=601.0, ppx=640.0,
                       ppy=360.0, coeffs=[0.1, 0.02, 0.0, 0.0, 0.0], model='brown_conrady')
    def width(self): return self.w
    def height(self): return self.h
    def fps(self): return self.f
    def format(self): return 'bgr8'
    def stream_type(self): return 'color'
    def stream_index(self): return 0
    def is_video_stream_profile(self): return True
    def as_video_stream_profile(self): return self
    def get_intrinsics(self): return self.intr


class Device:
    def __init__(self, serial, profiles): self.serial, self.profiles = serial, profiles
    def supports(self, field): return field == 'serial'
    def get_info(self, field):
        assert field == 'serial'
        return self.serial
    def query_sensors(self): return [NS(get_stream_profiles=lambda: self.profiles)]


def fixtures():
    instance = {'robot_type': 'piper_x_dual', 'robot_id': 'synthetic_test',
                'cameras': {role: {'serial': str(i)} for i, role in enumerate(
                    ('camera_head', 'camera_left', 'camera_right'), 1)}}
    devices = [Device(str(i), [Profile()]) for i in range(1, 4)]
    rs = NS(context=lambda: NS(query_devices=lambda: devices), stream=NS(color='color'),
            format=NS(bgr8='bgr8'), camera_info=NS(serial_number='serial', name='name',
            firmware_version='firmware', product_line='product'))
    return rs, instance, devices


class MetadataTests(unittest.TestCase):
    def test_device_mapping_exact_profile_and_distortion(self):
        rs, instance, _ = fixtures()
        value = export(rs, instance)
        self.assertFalse(value['streaming_started'])
        self.assertFalse(value['fresh_calibration_performed'])
        self.assertEqual(value['cameras']['camera_right']['serial'], '3')
        self.assertEqual(value['cameras']['camera_head']['color']['K'],
                         [[600.0, 0.0, 640.0], [0.0, 601.0, 360.0], [0.0, 0.0, 1.0]])
        self.assertEqual(value['cameras']['camera_head']['color']['distortion_model'], 'brown_conrady')

    def test_missing_serial_rejected(self):
        rs, instance, devices = fixtures(); devices.pop()
        with self.assertRaisesRegex(RuntimeError, 'serials not found'): export(rs, instance)

    def test_duplicate_binding_rejected(self):
        rs, instance, _ = fixtures(); instance['cameras']['camera_head']['serial'] = '2'
        with self.assertRaisesRegex(ValueError, 'distinct'): export(rs, instance)

    def test_wrong_resolution_not_substituted(self):
        rs, instance, devices = fixtures(); devices[0].profiles = [Profile(960, 540)]
        with self.assertRaisesRegex(RuntimeError, 'exactly one'): export(rs, instance)

    def test_ambiguous_profile_rejected(self):
        rs, instance, devices = fixtures(); devices[0].profiles.append(Profile())
        with self.assertRaisesRegex(RuntimeError, 'exactly one'): export(rs, instance)

    def test_invalid_intrinsics_rejected(self):
        profile = Profile(); profile.intr.fx = float('nan')
        with self.assertRaisesRegex(ValueError, 'invalid'): intrinsics_record(profile)

    def test_intrinsic_resolution_consistency(self):
        profile = Profile(); profile.intr.width = 960
        with self.assertRaisesRegex(ValueError, 'resolution mismatch'): intrinsics_record(profile)

    def test_nominal_head_matches_independent_prior_yourdfpy_audit(self):
        value = head_estimate(ASSETS)
        expected = [[0, -1, 0, 0.29],
                    [-1, 0, 0, 0.015],
                    [0, 0, -1, 0.648],
                    [0, 0, 0, 1]]
        np.testing.assert_allclose(value['T_base_camera'], expected, atol=1e-10, rtol=0)
        self.assertFalse(value['physical_acceptance'])
        self.assertIsNone(value['table_plane_in_base'])
        self.assertIsNone(value['tcp']['left']['position_m'])

    def test_unknown_mast_refused(self):
        mast = mast_configuration(); mast['joint_position_rad'] = None
        with self.assertRaisesRegex(ValueError, 'explicit mast'): head_estimate(ASSETS, mast)

    def test_inconsistent_total_tilt_refused(self):
        mast = mast_configuration(); mast['total_forward_tilt_deg'] = 60
        with self.assertRaisesRegex(ValueError, 'inconsistent'): head_estimate(ASSETS, mast)

    def test_mast_limit_refused(self):
        mast = mast_configuration(); mast.update(joint_position_rad=np.pi,
                                                 total_forward_tilt_deg=230.0)
        with self.assertRaisesRegex(ValueError, 'invalid mast joint'): head_estimate(ASSETS, mast)

    def test_rotation_convention(self):
        np.testing.assert_allclose(_rotation([0, 0, 1], np.pi / 2) @ [1, 0, 0],
                                   [0, 1, 0], atol=1e-15)


if __name__ == '__main__':
    unittest.main()
