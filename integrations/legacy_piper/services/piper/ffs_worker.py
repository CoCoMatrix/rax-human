"""Camera-owner packets -> pinned FFS -> RGB optical coordinates only. No robot transform."""
import argparse
from collections import deque
import hashlib
import json
import os
import shutil
from pathlib import Path
import time
import numpy as np
from services.piper.sensor.stereo_packet import calibration_identity, validate_packet, PacketSequenceGuard
from services.piper.sensor.stereo_geometry import CameraIntrinsics, StereoRectifier, disparity_to_left_xyz, align_left_xyz_to_rgb
from services.piper.depth import contained, MODEL_SHA
from services.piper.sensor.rgb_registration import register, library
from services.piper.workbench.ffs_quality import align_surface_footprints, left_right_consistency


def atomic(path, value):
    path = Path(path)
    tmp = path.with_suffix('.partial')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False))
    os.replace(tmp, path)


class Processor:
    def __init__(self, calibration, runner, *, surface_diagnostics=False):
        self.calibration, self.runner = calibration, runner
        self.surface_diagnostics = surface_diagnostics
        library()  # Verify the explicit native build before accepting any frame.
        self.sha = calibration_identity(calibration)
        intr = {k: CameraIntrinsics.from_dict(v) for k, v in calibration['profiles'].items()}
        ext = calibration['extrinsics']
        self.rgb_intrinsics = intr['color']
        self.rectifier = StereoRectifier(intr['ir_left'], intr['ir_right'],
            ext['ir_left_to_ir_right']['T_target_source'], ext['ir_left_to_color']['T_target_source'])
        self.guard = None

    def process(self, packet_dir, output, mode):
        packet_dir, output = Path(packet_dir), Path(output)
        packet = json.loads(contained(packet_dir, 'packet.json').read_text())
        with np.load(contained(packet_dir, 'images.npz'), allow_pickle=False) as z:
            arrays = {k: z[k] for k in z.files}
        def validate():
            return validate_packet(arrays, packet, self.calibration, mode=mode,
                now_ns=time.time_ns() if mode == 'live' else None, max_age_ms=1000,
                max_ir_skew_ms=2, max_rgb_skew_ms=34)
        timing = validate()
        if mode == 'live':
            if self.guard is None:
                self.guard = PacketSequenceGuard(packet['owner_generation'])
            self.guard.accept(packet)  # An owner restart requires a new supervised worker.
        start = time.monotonic()
        rect = self.rectifier.rectify_pair(arrays['ir_left'], arrays['ir_right'])
        prediction = self.runner.infer_rectified(rect['left_rectified'], rect['right_rectified'])
        meta = prediction['metadata']
        if (meta.get('disparity_definition') != 'x_left - x_right' or meta.get('disparity_unit') != 'original_input_pixels'
                or meta.get('disparity_reference') != 'left_rectified_optical' or meta.get('test_double')):
            raise ValueError('FFS convention or real-model provenance failure')
        disparity = np.asarray(prediction['disparity_px']).copy()
        reverse = self.runner.infer_rectified(np.fliplr(rect['right_rectified']).copy(),
                                             np.fliplr(rect['left_rectified']).copy())
        reverse_meta = reverse['metadata']
        if (reverse_meta.get('disparity_definition') != 'x_left - x_right'
                or reverse_meta.get('disparity_unit') != 'original_input_pixels'
                or reverse_meta.get('disparity_reference') != 'left_rectified_optical'
                or reverse_meta.get('test_double')):
            raise ValueError('Reverse FFS convention or real-model provenance failure')
        right_disparity = np.fliplr(reverse['disparity_px']).copy()
        consistent, lr_error = left_right_consistency(disparity, right_disparity)
        geometric_support = self.rectifier.disparity_valid_mask(disparity)
        disparity[~(geometric_support & consistent)] = np.nan
        points = disparity_to_left_xyz(disparity, self.rectifier.stereo)
        registration_start=time.monotonic()
        aligned=register(points['xyz_left_m'],points['valid_mask'],self.rectifier.T_rgb_from_leftrect,
                         self.rectifier.stereo.left,self.rgb_intrinsics)
        registration_s=time.monotonic()-registration_start
        direct_xyz,direct_valid=aligned['xyz_rgb_m'],aligned['valid']
        # Native optical z + rays reproduce each original source point without
        # storing a redundant XYZ image. RGB raster never replaces this geometry.
        extras=dict(native_depth_m=points['xyz_left_m'][...,2].astype(np.float32),
                    native_uv_rgb=aligned['native_uv_rgb'],native_visible=aligned['native_visible'],
                    surface_depth_rgb_m=aligned['surface_depth_rgb_m'],
                    surface_source_index=aligned['surface_source_index'],surface_resampled=aligned['surface_resampled'])
        surface=dict(surface_valid_fraction=float((aligned['surface_source_index']>=0).mean()),
                     surface_resampled_fraction=float(aligned['surface_resampled'].mean()),
                     max_local_span_m=.005,max_footprint_px=4,registration_s=registration_s,
                     registration_source_sha256=aligned['implementation_sha256'])
        si=self.rectifier.stereo.left
        native_meta=dict(schema='rax.native_geometry.v1',shape_hw=[si.height,si.width],
                         coordinate_frame='left_rectified_optical',rgb_coordinate_frame='global_d435_color_optical_frame',
                         intrinsics_fx_fy_cx_cy=[si.fx,si.fy,si.cx,si.cy],
                         T_rgb_from_leftrect=self.rectifier.T_rgb_from_leftrect.tolist(),
                         visibility='same_frame_footprint_zbuffer_1mm',source_index='row_major_original_stereo_grid',
                         rgb_surface_semantics='resampled_source_coverage_not_new_measurements')
        validate()
        output.mkdir(parents=True, exist_ok=False)
        # Only LR-checked original samples are exposed to pixel_to_position.
        np.savez(output/'observation.npz', rgb=arrays['rgb'],
                 xyz_rgb_m=direct_xyz.astype(np.float32), valid=direct_valid, **extras)
        report = dict(schema='rax.piper.ffs.v1', mode=mode, source='fast_foundation_stereo', inference_kind='real_model',
            checkpoint_sha256=MODEL_SHA, calibration_payload_sha256=self.sha, serial=packet['serial'],
            sample_id=packet['sample_id'], owner_generation=packet['owner_generation'], frames=packet['frames'],
            host_received_unix_ns=packet['host_received_unix_ns'], expires_unix_ns=packet['host_received_unix_ns']+1_000_000_000,
            frame='global_d435_color_optical_frame', depth_unit='m', depth_semantics='RGB optical z',
            rgb_binding='same immutable camera-owner packet as stereo', base_transform_applied=False,
            physical_calibration_validated=False, confidence_available=False, alignment='LR-checked direct nearest samples; no hole filling for motion',
            valid_fraction=float(direct_valid.mean()),
            native_geometry=native_meta,
            quality_extension={'version':'ffs-quality-v3','left_right_threshold_px':1.0,
                'source_geometric_fraction':float(geometric_support.mean()),
                'source_lr_consistent_fraction':float((geometric_support & consistent).mean()),
                'surface_diagnostics_computed':True,
                'surface_used_for_motion':False, **surface,
                'right_forward_ms':reverse['metadata']['forward_and_copy_ms'],
                'implementation_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}, model=meta, capture_timing=timing,
            elapsed_s=time.monotonic()-start, arrays_sha256=hashlib.sha256((output/'observation.npz').read_bytes()).hexdigest(),
            hardware_writes=0)
        atomic(output/'report.json', report)
        if mode == 'live':
            validate()  # Expensive serialization also counts against frame age.
        return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--surface-diagnostics', action='store_true', help='Compatibility flag; native registration now always publishes attributed RGB coverage')
    p.add_argument('--calibration', required=True)
    p.add_argument('--manifest', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--packet', required=True, help='Historical packet for explicitly labelled replay/warmup')
    p.add_argument('--packet-root', help='Live camera owner export directory; no camera is opened here')
    p.add_argument('--seconds', type=int, default=600)
    p.add_argument('--replay-count', type=int, default=1)
    p.add_argument('--max-packets', type=int, default=120)
    p.add_argument('--retain-results', type=int, default=0)
    p.add_argument('--retire-packets-after-s', type=int, default=0)
    a = p.parse_args()
    continuous = a.seconds == 0
    if not 0 <= a.seconds <= 7200:
        p.error('runtime must be 0 (continuous) or 1..7200 seconds')
    if not 1 <= a.replay_count <= 5:
        p.error('replay-count must be 1..5')
    if not 0 <= a.max_packets <= 10000 or a.retain_results not in (0,16,32) or a.retire_packets_after_s not in (0,10,20):
        p.error('bounded packet/retention settings invalid')
    if continuous and not (a.packet_root and a.max_packets == 0 and a.retain_results and a.retire_packets_after_s):
        p.error('continuous worker requires live input and rolling packet/result retention')
    if not continuous and a.max_packets == 0:
        p.error('unlimited packets require continuous mode')
    from services.piper.sensor.ffs_runtime import FastFoundationStereoRunner
    runner = FastFoundationStereoRunner(a.manifest, valid_iters=4, max_disp=192, input_scale=1, allow_trusted_pickle=True)
    worker = Processor(json.loads(Path(a.calibration).read_text()), runner, surface_diagnostics=a.surface_diagnostics)
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    if a.retain_results:
        from services.piper.retention import enroll
        enroll(out, 'result')
    report = worker.process(a.packet, out/'historical-warmup', 'offline_replay')
    atomic(out/'ready.json', dict(model_ready=True, live_results=0, hardware_writes=0))
    if not a.packet_root:
        for i in range(1,a.replay_count):
            report = worker.process(a.packet, out/f'historical-repeat-{i}', 'offline_replay')
        print(json.dumps(dict(mode='offline_replay', valid_fraction=report['valid_fraction'], elapsed_s=report['elapsed_s'])))
        return
    root = Path(a.packet_root)
    enrollment_ns = time.time_ns()
    if a.retire_packets_after_s and (root.exists() or root.is_symlink()):
        raise ValueError('Rolling packet retention requires a new, absent owner export root')
    from services.piper.retention import retire_expired
    atomic(out/'live-worker-ready.json', dict(continuous=continuous, packet_root=str(root.resolve())))
    seen = deque(maxlen=128)
    attempted = 0
    published = 0
    last_publish_ns = None
    max_gap_s = 0.0
    end = float('inf') if continuous else time.monotonic()+a.seconds
    while time.monotonic() < end:
        if (a.max_packets and attempted >= a.max_packets) or shutil.disk_usage(out).free < 3*1024**3:
            atomic(out/'stopped.json', dict(reason='bounded packet/disk limit', attempted=attempted))
            break
        try:
            pointer = json.loads((root/'latest.json').read_text())
            relative = pointer['packet_dir']
            if relative in seen:
                time.sleep(.01)
                continue
            seen.append(relative)
            attempted += 1
            result = worker.process(contained(root, relative), out/f'result-{attempted:06}', 'live')
            atomic(out/'latest.json', dict(result_dir=f'result-{attempted:06}', **{k: result[k] for k in
                ('sample_id','owner_generation','host_received_unix_ns','expires_unix_ns')}))
            now_ns = time.time_ns()
            gap_s = None if last_publish_ns is None else (now_ns-last_publish_ns)/1e9
            if gap_s is not None: max_gap_s = max(max_gap_s,gap_s)
            last_publish_ns = now_ns
            published += 1
            atomic(out/'producer-health.json', dict(published=published, attempted=attempted,
                sample_id=result['sample_id'], published_ns=now_ns,
                original_received_ns=result['host_received_unix_ns'],
                publish_age_s=(now_ns-result['host_received_unix_ns'])/1e9,
                processing_s=result['elapsed_s'], input_age_ms=result['capture_timing']['age_ms'],
                gap_s=gap_s, max_gap_s=max_gap_s, hardware_writes=0))
            # Retire only after publication, never on every poll ahead of new work.
            if a.retain_results:
                retire_expired(out, 'result', keep=a.retain_results, grace_s=10, journal=out/'retirement.jsonl')
            if a.retire_packets_after_s and root.exists() and not (root/'retention-owner.json').exists():
                if root.is_symlink():
                    raise ValueError('Owner export root may not be a symlink')
                # This exact root was absent when this bounded worker started.
                # Enrol only this run, without modifying the pinned camera owner.
                with (root/'retention-owner.json').open('x') as stream:
                    json.dump(dict(root=str(root.resolve()),kind='packet',created_ns=enrollment_ns),stream)
            if a.retire_packets_after_s:
                retire_expired(root, 'packet', keep=16, grace_s=a.retire_packets_after_s, journal=out/'retirement.jsonl')
        except Exception as exc:
            atomic(out/'last-error.json', dict(time_ns=time.time_ns(), error=type(exc).__name__+': '+str(exc)))
            time.sleep(.2)


if __name__ == '__main__':
    main()
