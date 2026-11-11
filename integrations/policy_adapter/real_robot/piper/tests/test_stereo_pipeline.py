import numpy as np
import pytest
from real_robot.piper.tests.test_stereo_packet import sample
from real_robot.piper.export_stereo_calibration import canonical_sha256
from real_robot.piper.calibration_nominal import head_estimate
from real_robot.piper.tests.test_calibration_metadata import ASSETS
from real_robot.piper.stereo_pipeline import StereoDepthProcessor


class SyntheticRunner:
    calls = 0
    def infer_rectified(self, left, right):
        self.calls += 1
        assert left.shape == right.shape == (32, 64)
        return {'disparity_px': np.full(left.shape, 8, dtype=np.float32), 'metadata': {
            'test_double': True, 'disparity_definition': 'x_left - x_right',
            'disparity_unit': 'original_input_pixels', 'disparity_reference': 'left_rectified_optical'}}


def setup():
    arrays, packet, calibration = sample()
    for profile in calibration['profiles'].values():
        profile.update(K=[[80.0, 0, 31.5], [0, 80.0, 15.5], [0, 0, 1]],
                       distortion_model='distortion.none', distortion_coeffs=[0]*5)
    stereo = np.eye(4); stereo[0, 3] = -0.05  # Synthetic fixture, never hardware default.
    rgb = np.eye(4); rgb[2, 3] = 0.1
    calibration['extrinsics'] = {'ir_left_to_ir_right': {'T_target_source': stereo.tolist()},
                                 'ir_left_to_color': {'T_target_source': rgb.tolist()}}
    calibration.pop('calibration_payload_sha256')
    calibration['calibration_payload_sha256'] = canonical_sha256(calibration)
    packet['calibration_payload_sha256'] = calibration['calibration_payload_sha256']
    head = head_estimate(ASSETS)
    runner = SyntheticRunner()
    return arrays, packet, calibration, head, runner


def test_complete_synthetic_chain_keeps_rgb_z_separate_from_ir_z():
    arrays, packet, calib, head, runner = setup()
    original_rgb = arrays['rgb'].copy()
    processor = StereoDepthProcessor(calib, head, runner)
    result, report = processor.process(arrays, packet)
    np.testing.assert_allclose(result['depth_left_rectified_m'][np.isfinite(result['depth_left_rectified_m'])], 0.5)
    np.testing.assert_allclose(result['depth_head_rgb_m'][result['valid_head_rgb']], 0.6)
    points = result['xyz_head_rgb_m'][result['valid_head_rgb']]
    T = np.array(head['T_base_camera'])
    np.testing.assert_allclose(result['xyz_dual_base_m'][result['valid_head_rgb']],
                               points @ T[:3,:3].T + T[:3,3], atol=1e-7)
    assert report['model']['test_double'] and not report['physical_acceptance']
    assert report['source'] == report['inference_kind'] == 'synthetic_test_double'
    assert report['mode'] == 'offline_replay' and not report['robot_action_emitted']
    np.testing.assert_array_equal(arrays['rgb'], original_rgb)


def test_live_rejects_stale_before_inference_and_after_slow_inference():
    arrays, packet, calib, head, runner = setup()
    processor = StereoDepthProcessor(calib, head, runner, owner_generation=packet['owner_generation'])
    with pytest.raises(ValueError, match='stale'):
        processor.process(arrays, packet, mode='live', clock_ns=lambda: 1_002_000_000_000)
    assert runner.calls == 0
    clock = iter([1_000_001_000_000, 1_002_000_000_000])
    with pytest.raises(ValueError, match='stale'):
        processor.process(arrays, packet, mode='live', clock_ns=lambda: next(clock))
    assert runner.calls == 1


def test_live_generation_and_frame_replay_are_rejected():
    arrays, packet, calib, head, runner = setup()
    processor = StereoDepthProcessor(calib, head, runner, owner_generation=packet['owner_generation'])
    processor.process(arrays, packet, mode='live', clock_ns=lambda: 1_000_001_000_000)
    with pytest.raises(ValueError, match='cached'):
        processor.process(arrays, packet, mode='live', clock_ns=lambda: 1_000_001_000_000)


def test_missing_disparity_units_and_false_calibration_claim_rejected():
    arrays, packet, calib, head, runner = setup()
    runner.infer_rectified = lambda *a: {'disparity_px': np.ones((32,64), np.float32), 'metadata': {}}
    with pytest.raises(ValueError, match='convention/unit'):
        StereoDepthProcessor(calib, head, runner).process(arrays, packet)
    head['physical_acceptance'] = True
    with pytest.raises(ValueError, match='nominal head'):
        StereoDepthProcessor(calib, head, runner)


def test_v9_head_geometry_is_not_accepted_after_v11_migration():
    from real_robot.piper.calibration_nominal import head_estimate
    from real_robot.piper.tests.test_calibration_metadata import ASSETS
    from real_robot.piper.stereo_pipeline import StereoDepthProcessor
    import pytest
    _, _, calibration, head, runner = setup()
    head['urdf_sha256'] = '87ea54a1d684db2e3be59af8fb2eb3f8780b207491bb05df700e00ac0e30998b'
    with pytest.raises(ValueError, match='pinned v11'):
        StereoDepthProcessor(calibration, head, runner)
