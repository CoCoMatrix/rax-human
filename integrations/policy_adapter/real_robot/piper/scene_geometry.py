"""Nominal CAD + recent stereo scene evidence for shadow review, never dispatch.

Head RGB remains the original observation. Separately timestamped depth is
reported as such; photometric agreement is evidence, not semantic identity.
"""
from pathlib import Path
import hashlib
import json
import numpy as np

from .contracts import require
from .description import URDF_SHA256
from .stereo_live import read_fresh_depth, _contained_directory, _load_packet

TCP_SHA256 = '419d61e87ca8fb3b52cd48126e23cc823d11eef3009e575cf54bdaf5fa8afd47'


def nominal_tcp(path):
    raw = Path(path).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == TCP_SHA256, 'nominal TCP identity mismatch')
    value = json.loads(raw)
    require(value['urdf_sha256'] == URDF_SHA256 and value['tcp_measured'] is False
            and value['physical_acceptance'] is False and value['no_dispatch_authority'] is True,
            'nominal TCP must retain CAD-only status')
    return value


def scene_agreement(original, depth_rgb):
    require(original.shape == depth_rgb.shape and original.dtype == depth_rgb.dtype == np.uint8,
            'scene comparison requires same-shape original RGB')
    delta = np.max(np.abs(original.astype(np.int16) - depth_rgb.astype(np.int16)), axis=2)
    metrics = {'mean_max_channel_difference': float(delta.mean()),
               'fraction_over_30_levels': float((delta > 30).mean()),
               'p95_max_channel_difference': float(np.percentile(delta, 95)),
               'test': 'photometric_consistency_not_object_identity_or_collision_check'}
    require(metrics['mean_max_channel_difference'] < 8
            and metrics['fraction_over_30_levels'] < .02,
            'separate depth RGB does not agree with the reviewed scene')
    return metrics


def fit_table(points, mask):
    """Dominant plane within a caller-supplied table ROI; no hardware calibration."""
    cloud = np.asarray(points)[mask]
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    require(len(cloud) >= 1000, 'too few finite points for table-plane preview')
    cloud = cloud[::max(1, len(cloud) // 12000)]
    rng = np.random.default_rng(20260918)
    best = None
    for _ in range(80):
        p = cloud[rng.choice(len(cloud), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        length = np.linalg.norm(n)
        if length < 1e-8:
            continue
        n = n / length
        inliers = np.abs(cloud @ n - p[0] @ n) < .007
        if best is None or inliers.sum() > best.sum():
            best = inliers
    require(best is not None and best.mean() >= .60, 'dominant table plane is ambiguous')
    center = np.mean(cloud[best], axis=0)
    _, _, vt = np.linalg.svd(cloud[best] - center, full_matrices=False)
    n = vt[-1]
    if n[2] < 0:
        n = -n
    d = -float(center @ n)
    residual = np.abs(cloud @ n + d)
    inliers = residual < .007
    require(inliers.mean() >= .60, 'refined table-plane fit is ambiguous')
    return {'normal': n.tolist(), 'offset_m': d, 'equation': 'normal dot p_base + offset_m = 0',
            'inlier_fraction': float(inliers.mean()), 'inlier_rms_m': float(np.sqrt(np.mean(residual[inliers] ** 2))),
            'source': 'FFS_dominant_plane_in_declared_image_ROI_with_nominal_URDF_transform',
            'measured_calibration': False, 'collision_clearance_certified': False}


def region_preview(arrays, plane, regions):
    h, w = arrays['valid_head_rgb'].shape
    result = []
    for region in regions:
        bbox = region['bbox_xyxy_normalized']
        require(len(bbox) == 4 and all(type(x) in (int, float) and np.isfinite(x) for x in bbox)
                and 0 <= bbox[0] < bbox[2] <= 1 and 0 <= bbox[1] < bbox[3] <= 1,
                'invalid scene annotation box')
        x0, y0, x1, y1 = int(bbox[0]*w), int(bbox[1]*h), int(bbox[2]*w), int(bbox[3]*h)
        valid = arrays['valid_head_rgb'][y0:y1, x0:x1].astype(bool)
        points = arrays['xyz_dual_base_m'][y0:y1, x0:x1][valid]
        points = points[np.isfinite(points).all(axis=1)]
        require(len(points) >= 20, 'annotated region has too few depth points')
        median = np.median(points, axis=0)
        heights = points @ np.asarray(plane['normal']) + plane['offset_m']
        result.append({'label': region['label'], 'bbox_xyxy_normalized': bbox,
            'finite_points': len(points), 'valid_fraction': float(valid.mean()),
            'median_xyz_dual_base_m': median.tolist(),
            'xyz_p10_m': np.percentile(points, 10, axis=0).tolist(),
            'xyz_p90_m': np.percentile(points, 90, axis=0).tolist(),
            'height_above_nominal_table_p10_p50_p90_m': np.percentile(heights, [10, 50, 90]).tolist(),
            'meaning': 'surface patch only, not a grasp center or an executable target',
            'object_identity_requires_visual_review': True})
    return result


def geometry_context(observation, *, depth_output, packet_root, tcp_path, regions_path=None, skip_image_comparison=False):
    arrays, report = read_fresh_depth(depth_output)
    require(report['source'] == 'fast_foundation_stereo' and report['inference_kind'] == 'real_model',
            'real stereo model evidence required')
    stamp = report['host_received_unix_ns']
    require(abs(stamp - observation.timestamp_ns) <= 1_000_000_000,
            'depth and observation receipts exceed 1-second shadow association budget')
    # Locate only the packet explicitly named by the depth sample, not latest.
    matches = list(Path(packet_root).glob('packet-*-' + report['sample_id']))
    require(len(matches) == 1, 'depth source packet missing or ambiguous')
    folder = _contained_directory(packet_root, matches[0].name)
    packet_arrays, packet = _load_packet(folder)
    require(packet['owner_generation'] == report['owner_generation']
            and packet['host_received_unix_ns'] == stamp, 'depth source packet identity mismatch')
    agreement = ({'disabled_by_operator': True} if skip_image_comparison else
                 scene_agreement(observation.images['camera_head'], packet_arrays['rgb']))
    valid = arrays['valid_head_rgb'].astype(bool)
    h, w = valid.shape
    roi = np.zeros_like(valid)
    roi[int(h*.22):int(h*.91), int(w*.32):int(w*.93)] = True
    plane = fit_table(arrays['xyz_dual_base_m'], valid & roi)
    tcp = nominal_tcp(tcp_path)
    regions = []
    reference_agreement = None
    if regions_path is not None:
        from PIL import Image
        selected = json.loads(Path(regions_path).read_text())
        reference_path = Path(selected['reference_image'])
        require(hashlib.sha256(reference_path.read_bytes()).hexdigest() == selected['reference_image_sha256'],
                'scene annotation reference identity mismatch')
        reference_agreement = ({'disabled_by_operator': True} if skip_image_comparison else
            scene_agreement(observation.images['camera_head'], np.asarray(Image.open(reference_path).convert('RGB'))))
        regions = region_preview(arrays, plane, selected['regions'])
    return {'schema': 'junqi.piper.shadow_scene_geometry.v1',
        'scope': 'historical_shadow_geometry_preview_only', 'dispatch_authorized': False,
        'observation_timestamp_ns': observation.timestamp_ns,
        'depth_capture_receipt_ns': stamp, 'depth_sample_id': report['sample_id'],
        'depth_camera_generation': report['owner_generation'],
        'depth_publication_ns': report['published_unix_ns'],
        'same_rgb_frame': bool(np.array_equal(observation.images['camera_head'], packet_arrays['rgb'])),
        'head_scene_agreement': agreement,
        'depth_original_camera_age_budget_ms': report['max_age_ms'],
        'association_skew_ms': (stamp - observation.timestamp_ns) / 1e6,
        'depth_semantics': report['depth_semantics'],
        'mask_semantics': report['mask_semantics'], 'confidence_available': False,
        'nominal_table_plane': plane, 'nominal_grasp_tcp': tcp,
        'surface_regions': regions, 'annotation_reference_agreement': reference_agreement,
        'tcp_use': 'Explicit CAD preview selection only; physical geometry match not verified.',
        'depth_limitations': ['Transparent/reflective bottle depth can mix with the table.',
                             'This plane and TCP do not certify a collision-free grasp.'],
        'robot_action_emitted': False}
