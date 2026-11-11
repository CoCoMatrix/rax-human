"""Prepared observation-only FFS pipeline, with a saved-packet CLI.

This module has no camera/robot SDK or action transport. A future camera-owner
hook supplies stereo_packet arrays. RGB Pi0.5 observations remain unchanged.
"""
import argparse
import copy
import json
from pathlib import Path
import time
import numpy as np

from .contracts import require
from .description import URDF_SHA256
from .export_stereo_calibration import canonical_sha256
from .camera_geometry import validate_head_geometry, ACTIVE_HEAD
from .stereo_packet import calibration_identity, validate_packet, PacketSequenceGuard
from .stereo_geometry import (CameraIntrinsics, StereoRectifier, disparity_to_left_xyz,
                              align_left_xyz_to_rgb, transform_xyz, validate_se3)


class StereoDepthProcessor:
    def __init__(self, calibration, nominal_head, runner, *, owner_generation=None,
                 max_age_ms=1000.0, max_ir_skew_ms=2.0, max_rgb_skew_ms=34.0):
        self.calibration = copy.deepcopy(calibration)
        self.calibration_sha256 = calibration_identity(self.calibration)
        require(nominal_head.get('schema') == 'junqi.piper.tabletop_geometry.v1'
                and nominal_head.get('camera_role') == 'camera_head'
                and nominal_head.get('parent_frame') == 'dual_base_link'
                and nominal_head.get('child_frame') == 'global_d435_color_optical_frame'
                and nominal_head.get('calibration_status') in ('nominal_urdf_not_calibrated', 'table_plane_tilt_height_corrected_xy_yaw_nominal')
                and nominal_head.get('physical_acceptance') is False
                and nominal_head.get('urdf_sha256') == URDF_SHA256,
                'expected pinned v11 nominal head RGB pose')
        require(nominal_head.get('transform_convention') ==
                'p_base = T_base_camera @ p_camera_homogeneous', 'head transform convention mismatch')
        self.nominal_head = copy.deepcopy(nominal_head)
        validate_head_geometry(self.nominal_head)
        self.head_sha256 = canonical_sha256(self.nominal_head)
        self.T_base_rgb = validate_se3(nominal_head['T_base_camera'])
        profiles = self.calibration['profiles']
        intr = {name: CameraIntrinsics.from_dict(value) for name, value in profiles.items()}
        self.rgb_intrinsics = intr['color']
        extrinsics = self.calibration['extrinsics']
        self.rectifier = StereoRectifier(intr['ir_left'], intr['ir_right'],
            extrinsics['ir_left_to_ir_right']['T_target_source'],
            extrinsics['ir_left_to_color']['T_target_source'])
        self.runner = runner
        self.timing = dict(max_age_ms=max_age_ms, max_ir_skew_ms=max_ir_skew_ms,
                           max_rgb_skew_ms=max_rgb_skew_ms)
        self.sequence_guard = PacketSequenceGuard(owner_generation) if owner_generation else None

    def process(self, arrays, packet, *, mode='offline_replay', clock_ns=time.time_ns):
        before = validate_packet(arrays, packet, self.calibration, mode=mode,
                                 now_ns=clock_ns() if mode == 'live' else None, **self.timing)
        if mode == 'live':
            require(self.sequence_guard is not None, 'live processor requires explicit owner generation')
            self.sequence_guard.accept(packet)
        rectified = self.rectifier.rectify_pair(arrays['ir_left'], arrays['ir_right'])
        prediction = self.runner.infer_rectified(rectified['left_rectified'], rectified['right_rectified'])
        disparity = np.asarray(prediction['disparity_px'])
        require(disparity.dtype.kind == 'f' and disparity.shape == arrays['ir_left'].shape,
                'model disparity must be restored to original rectified input resolution')
        predicted_meta = prediction.get('metadata', {})
        require(predicted_meta.get('disparity_definition') == 'x_left - x_right'
                and predicted_meta.get('disparity_unit') == 'original_input_pixels'
                and predicted_meta.get('disparity_reference') == 'left_rectified_optical',
                'model disparity convention/unit missing or incompatible')
        support = self.rectifier.disparity_valid_mask(disparity)
        usable_disparity = np.array(disparity, copy=True)
        usable_disparity[~support] = np.nan
        points = disparity_to_left_xyz(usable_disparity, self.rectifier.stereo)
        aligned = align_left_xyz_to_rgb(points['xyz_left_m'], points['valid_mask'],
                                        self.rectifier.T_rgb_from_leftrect, self.rgb_intrinsics)
        base = transform_xyz(aligned['xyz_rgb_m'], aligned['valid_mask'], self.T_base_rgb)
        after = validate_packet(arrays, packet, self.calibration, mode=mode,
                                now_ns=clock_ns() if mode == 'live' else None, **self.timing)
        result_arrays = {'disparity_px': disparity.astype(np.float32),
                         'depth_left_rectified_m': points['depth_left_m'].astype(np.float32),
                         'depth_head_rgb_m': aligned['depth_rgb_m'].astype(np.float32),
                         'valid_head_rgb': aligned['valid_mask'],
                         'xyz_head_rgb_m': aligned['xyz_rgb_m'].astype(np.float32),
                         'xyz_dual_base_m': base['xyz_m'].astype(np.float32)}
        is_synthetic = predicted_meta.get('test_double') is True
        summary = {'schema': 'junqi.piper.depth_observation.v1', 'mode': mode,
            'source': 'synthetic_test_double' if is_synthetic else 'fast_foundation_stereo',
            'inference_kind': 'synthetic_test_double' if is_synthetic else 'real_model',
            'sample_id': packet['sample_id'],
            'camera_role': 'camera_head', 'serial': packet['serial'],
            'owner_generation': packet['owner_generation'], 'frames': copy.deepcopy(packet['frames']),
            'host_received_unix_ns': packet['host_received_unix_ns'],
            'calibration_payload_sha256': self.calibration_sha256,
            'head_geometry_sha256': self.head_sha256,
            'base_coordinate_status': self.nominal_head['calibration_status'],
            'extrinsics_correction': self.nominal_head.get('correction'),
            'mast_configuration': self.nominal_head['mast_configuration'],
            'model': predicted_meta, 'timing_before': before, 'timing_after': after,
            'rectified_baseline_m': self.rectifier.stereo.baseline_m,
            'rectified_K_left': self.rectifier.stereo.left.matrix.tolist(),
            'rectified_K_right': self.rectifier.stereo.right.matrix.tolist(),
            'T_rgb_from_leftrect': self.rectifier.T_rgb_from_leftrect.tolist(),
            'depth_unit': 'm', 'depth_semantics': 'head_RGB_optical_z_not_IR_z',
            'alignment': 'nearest_pixel_zbuffer_no_hole_filling',
            'valid_rgb_fraction': float(aligned['valid_mask'].mean()),
            'confidence_available': False, 'mask_semantics': 'geometric_validity_not_confidence',
            'pi05_rgb_input_modified': False, 'hardware_access': False,
            'robot_action_emitted': False, 'physical_acceptance': False}
        return result_arrays, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--packet', type=Path, required=True, help='folder containing images.npz and packet.json')
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--nominal-head', type=Path, default=ACTIVE_HEAD)
    parser.add_argument('--model-manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trust-official-pickle', action='store_true')
    parser.add_argument('--valid-iters', type=int, default=4)
    parser.add_argument('--max-disp', type=int, default=192)
    parser.add_argument('--input-scale', type=float, default=1.0)
    args = parser.parse_args()
    require(not args.output.exists(), 'output directory already exists')
    from .ffs_runtime import FastFoundationStereoRunner
    runner = FastFoundationStereoRunner(args.model_manifest, valid_iters=args.valid_iters,
        max_disp=args.max_disp, input_scale=args.input_scale, allow_trusted_pickle=args.trust_official_pickle)
    processor = StereoDepthProcessor(json.loads(args.calibration.read_text()),
                                     json.loads(args.nominal_head.read_text()), runner)
    with np.load(args.packet / 'images.npz', allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files}
    metadata = json.loads((args.packet / 'packet.json').read_text())
    result, report = processor.process(arrays, metadata, mode='offline_replay')
    args.output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(args.output / 'depth.npz', **result)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': str(args.output), 'mode': 'offline_replay',
                      'valid_rgb_fraction': report['valid_rgb_fraction'], 'robot_action_emitted': False}))


if __name__ == '__main__':
    main()
