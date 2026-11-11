"""Image-bound perception evidence. Segmentation never grants motion permission."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def bind_result(snapshot, result):
    """A result for another frame, camera, or robot anchor cannot be reused."""
    if result.get('frame_id') != snapshot['observation']['frame_id']:
        raise ValueError('SEGMENT_FRAME_MISMATCH')
    if result.get('snapshot_q') != snapshot['q']:
        raise ValueError('SEGMENT_ANCHOR_MISMATCH')
    expected = {'head': snapshot.get('geometry_head') or snapshot['frames']['head'],
                'left': snapshot['frames']['left']}
    for camera, path in expected.items():
        view = result.get('views', {}).get(camera)
        if not view or view.get('source_sha256') != digest(path):
            raise ValueError('SEGMENT_IMAGE_MISMATCH:' + camera)
    if result.get('motion_permit') is not False:
        raise ValueError('SEGMENT_RESULT_IS_NOT_AUTHORIZATION')
    return result


def unique_object(result, camera, concept):
    rows = [r for r in result['views'][camera]['objects'] if r['concept'] == concept]
    if len(rows) != 1:
        raise ValueError(f'SEGMENT_AMBIGUOUS:{camera}:{concept}:{len(rows)}')
    return rows[0]


def rim_quality(snapshot, result, pixels):
    """Check the exact selected rim samples; never replace holes or use cup interior."""
    bind_result(snapshot, result)
    obj = unique_object(result, 'head', 'cup')
    folder = Path(result['directory'])
    mask_path = (folder / obj['mask']).resolve()
    if mask_path.parent != folder.resolve():
        raise ValueError('SEGMENT_MASK_PATH')
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None or digest(mask_path) != obj['mask_sha256']:
        raise ValueError('SEGMENT_MASK_INTEGRITY')
    with np.load(Path(snapshot['observation']['path']) / 'depth.npz', allow_pickle=False) as z:
        valid, xyz = z['valid'], z['xyz_rgb_m']
    if mask.shape != valid.shape:
        raise ValueError('SEGMENT_DEPTH_RESOLUTION')
    if len(pixels) < 3 or len({tuple(p) for p in pixels}) != len(pixels):
        raise ValueError('RIM_REQUIRES_DISTINCT_SAMPLES')
    selected = []
    patches = []
    for p in pixels:
        if len(p) != 2 or any(type(v) is not int for v in p):
            raise ValueError('RIM_PIXEL_TYPE')
        x, y = p
        if not (2 <= x < valid.shape[1]-2 and 2 <= y < valid.shape[0]-2):
            raise ValueError('RIM_PIXEL_BOUNDARY')
        if mask[y, x] == 0:
            raise ValueError('RIM_PIXEL_NOT_TARGET')
        if not valid[y, x] or not np.isfinite(xyz[y, x]).all():
            raise ValueError('RIM_DEPTH_HOLE')
        local = valid[y-2:y+3, x-2:x+3] & (mask[y-2:y+3, x-2:x+3] > 0)
        values = xyz[y-2:y+3, x-2:x+3, 2][local]
        if len(values) < 5:
            raise ValueError('RIM_INSUFFICIENT_LOCAL_SUPPORT')
        spread = float(np.quantile(values, .9)-np.quantile(values, .1))
        if spread > .015:
            raise ValueError('RIM_MIXED_SURFACES')
        selected.append(xyz[y, x].tolist())
        patches.append({'valid_samples': len(values), 'p90_p10_m': spread})
    if np.max(np.ptp(np.asarray(selected), axis=0)) > .025:
        raise ValueError('RIM_INCOHERENT_SAMPLES')
    return {'frame_id': result['frame_id'], 'pixels': pixels, 'camera_points': selected,
            'patches': patches, 'learned_confidence_available': False,
            'physical_accuracy_verified': False, 'motion_permit': False}


def compare_target_rois(reviewed, fresh, result):
    """Small objects must not disappear inside a whole-image change threshold."""
    bind_result(reviewed, result)
    checked = 0
    for camera in ('head', 'left'):
        a = cv2.imread(reviewed['frames'][camera])
        b = cv2.imread(fresh['frames'][camera])
        if a is None or b is None or a.shape != b.shape:
            raise ValueError('TARGET_COMPARISON_IMAGES')
        for obj in result['views'][camera]['objects']:
            if obj['concept'] not in ('cup', 'coaster'):
                continue
            x0, y0, x1, y1 = np.asarray(obj['box_xyxy'], int)
            x0, y0 = max(0, x0-5), max(0, y0-5)
            x1, y1 = min(a.shape[1], x1+6), min(a.shape[0], y1+6)
            if x1 <= x0 or y1 <= y0:
                raise ValueError('TARGET_COMPARISON_BOX')
            delta = np.abs(a[y0:y1, x0:x1].astype(float)-b[y0:y1, x0:x1]).mean(2)
            if float((delta > 35).mean()) > .025:
                raise ValueError('TARGET_CHANGED_AFTER_REVIEW:' + camera + ':' + obj['concept'])
            checked += 1
    if not checked:
        raise ValueError('NO_TARGET_ROI_FOR_COMPARISON')


def approach_clearance(snapshot, parameters, tool=None):
    """Preserve demonstrated approach envelope; do not claim a calibrated TCP.

    The operator retained the 110mm contact-centre experience and declined making
    a new physical measurement a prerequisite. This enforces the actual regression:
    a failed195mm IK must not silently become a170mm contact approach.
    """
    if not .017 <= snapshot['q'][6] <= .0205:
        raise ValueError('OPEN_19MM_IN_CLEAR_SPACE_BEFORE_APPROACH')
    standoff = parameters.get('standoff', .195)
    if isinstance(standoff, bool) or not isinstance(standoff, (int,float)) or not np.isfinite(standoff):
        raise ValueError('APPROACH_STANDOFF_INVALID')
    if not .195 <= standoff <= .25:
        raise ValueError('APPROACH_BELOW_DEMONSTRATED_STANDOFF: choose a reachable side/orientation, not a lower height')
    return {'flange_standoff_m': standoff, 'minimum_demonstrated_standoff_m': .195,
            'contact_center_from_installation_plane_m': .110,
            'fingertip_clearance_m': None, 'physical_clearance_verified': False,
            'basis': '20261102T031727Z-d82df2 successful grasp; 110mm retained as operator experience',
            'requires_current_head_wrist_review': True}
