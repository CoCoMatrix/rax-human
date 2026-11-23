"""Ordinary, read-only RGB/depth perception for compact magnetic blocks.

No model, device, or Context is constructed on import. These measurements never
authorize a motion or establish support, pad alignment, retention or clearance.
"""
import hashlib
import math
from pathlib import Path
import time

import cv2
import numpy as np

# Candidate ranges in native 1280x720 pixels, conditional on this head setup.
PARAMS = dict(area_min=1800, area_max=20000, size_min=55, size_max=180,
              aspect_min=0.55, aspect_max=1.8, fill_min=0.45,
              interior_margin_px=7, max_age_s=30, depth_spread_max_m=0.012,
              min_valid_depth_samples=9)
HSV_RANGES = {'red': [((0, 100, 65), (9, 255, 255)), ((170, 100, 65), (179, 255, 255))],
              'orange': [((10, 110, 85), (18, 255, 255))],
              'yellow': [((19, 100, 90), (34, 255, 255))],
              'green': [((35, 70, 45), (88, 255, 255))]}
# Wider proposals are local to initial component selection. Other users of
# HSV_RANGES (retained-scene/placement masks) retain their existing contract.
GREEN_CANDIDATE_RANGE = ((32, 70, 45), (88, 255, 255))


class PerceptionUnknown(Exception):
    pass


def _mapping(value, label):
    if not isinstance(value, dict):
        raise PerceptionUnknown(f'Missing or malformed {label}')
    return value


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _yellow_green_identity(hsv, mask):
    """Absolute color evidence, not relative ordering or forced assignment.

    Ignore achromatic decoration and dark pixels. Hue 30..34 is a soft
    yellow/green transition; a balanced region remains unknown. Both labels
    use the same scores, so they cannot independently claim the same region.
    """
    samples = hsv[mask & (hsv[:, :, 1] >= 70) & (hsv[:, :, 2] >= 45)]
    if not len(samples):
        return {'identity': None, 'reason': 'no chromatic surface'}
    hue = samples[:, 0].astype(float)
    weights = samples[:, 1].astype(float) / 255.
    green = float(np.average(np.clip((hue-30.)/4., 0., 1.), weights=weights))
    yellow = 1.-green
    identity = ('green' if green >= .60 else 'yellow' if yellow >= .60 else None)
    return {'identity': identity, 'green_score': green, 'yellow_score': yellow,
            'hue_p10_p50_p90': np.percentile(hue, [10, 50, 90]).tolist(),
            'chromatic_samples': len(samples), 'method': 'absolute_surface_hue_distribution_v1'}


def _candidate_mask(hsv, color):
    kernel = np.ones((3, 3), np.uint8)
    if color in ('green', 'yellow'):
        # Establish confidently green regions first, then remove those exact
        # pixels from yellow proposals. This also separates adjacent blocks
        # without merging their touching silhouettes or using a scene quota.
        green = cv2.inRange(hsv, *GREEN_CANDIDATE_RANGE)
        green = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernel)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(green)
        exclusive_green = np.zeros(green.shape, np.uint8)
        for i in range(1, count):
            x,y,w,h,area = map(int, stats[i])
            if area < 9:
                continue
            region = labels[y:y+h, x:x+w] == i
            evidence = _yellow_green_identity(hsv[y:y+h, x:x+w], region)
            if evidence['identity'] == 'green':
                exclusive_green[y:y+h, x:x+w][region] = 255
        if color == 'green':
            return exclusive_green
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in HSV_RANGES['yellow']:
            mask |= cv2.inRange(hsv, lo, hi)
        mask[exclusive_green > 0] = 0
    else:
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in HSV_RANGES[color]:
            mask |= cv2.inRange(hsv, lo, hi)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def _shape_evidence(region):
    """Decoration/fragmentation affects a score, not a standalone fill veto.

    Filled contours are used only for outline quality. Depth sampling below
    still uses the original measured-color mask, never the filled silhouette.
    """
    contours, _ = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)
    outer = max(contours, key=cv2.contourArea)
    outline = float(cv2.contourArea(outer))
    hull = float(cv2.contourArea(cv2.convexHull(outer)))
    fill = float(region.mean())
    rectangularity = outline / max(1, (region.shape[0]-1)*(region.shape[1]-1))
    solidity = outline / max(1., hull)
    score = .55*rectangularity + .25*solidity + .20*fill
    return {'fill': fill, 'rectangularity': rectangularity, 'solidity': solidity,
            'score': score, 'fill_is_diagnostic_only': True}


def component(image, color, candidate_filter=None):
    """Pure detector; ambiguous/absent/truncated candidates fail closed."""
    if not isinstance(color, str) or color not in HSV_RANGES:
        raise PerceptionUnknown('Choose color red, orange, green, or yellow explicitly')
    if not isinstance(image, np.ndarray) or image.shape != (720, 1280, 3) or image.dtype != np.uint8:
        raise PerceptionUnknown('Expected native head image 1280x720 BGR')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = _candidate_mask(hsv, color)
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    candidates = []
    for i in range(1, count):
        x, y, w, h, area = map(int, stats[i])
        if x==0 or y==0 or x+w==image.shape[1] or y+h==image.shape[0]:
            continue  # Border-cut table strips are not complete block candidates.
        if (PARAMS['area_min'] <= area <= PARAMS['area_max'] and
            PARAMS['size_min'] <= w <= PARAMS['size_max'] and
            PARAMS['size_min'] <= h <= PARAMS['size_max'] and
            PARAMS['aspect_min'] <= w/h <= PARAMS['aspect_max']):
            region = labels[y:y+h, x:x+w] == i
            identity = (_yellow_green_identity(hsv[y:y+h, x:x+w], region)
                        if color in ('green', 'yellow') else {'identity': color})
            shape = _shape_evidence(region)
            if identity['identity'] != color or shape['score'] < .60:
                continue
            if candidate_filter is None or candidate_filter([x,y,w,h]):
                candidates.append((i, x, y, w, h, area, identity, shape))
    if len(candidates) != 1:
        raise PerceptionUnknown(f'Expected one {color} block candidate; found {len(candidates)}')
    i, x, y, w, h, area, identity, shape = candidates[0]
    if x == 0 or y == 0 or x+w == image.shape[1] or y+h == image.shape[0]:
        raise PerceptionUnknown('Candidate touches image border')
    distance = cv2.distanceTransform((labels == i).astype(np.uint8), cv2.DIST_L2, 5)
    _, max_distance, _, center = cv2.minMaxLoc(distance)
    offsets = ((0,0),(-6,0),(6,0),(0,-6),(0,6))
    pixels = [[center[0]+dx, center[1]+dy] for dx, dy in offsets]
    if any(distance[v,u] < PARAMS['interior_margin_px'] for u,v in pixels):
        raise PerceptionUnknown('Insufficient interior for five depth windows')
    return {'color': color, 'bbox_xywh': [x,y,w,h], 'area_px': area,
            'color_evidence': identity, 'shape_evidence': shape,
            'pixels': pixels, 'max_interior_distance_px': float(max_distance)}, distance


def supported_depth_pixels(distance, valid, preferred):
    """Select up to five interior 7x7 windows with real native depth support.

    Pure selection only: never fill a hole, synthesize depth, or relax color
    identity. At least three separated windows are required. Native replies
    still have to pass measure_block's frame/calibration/sample/spread checks.
    """
    if distance.shape != (720, 1280) or valid.shape != distance.shape:
        raise PerceptionUnknown('Malformed depth selection arrays')
    counts = cv2.boxFilter(valid.astype(np.float32), -1, (7, 7),
                          normalize=False, borderType=cv2.BORDER_CONSTANT)
    allowed = ((distance >= PARAMS['interior_margin_px']) &
               (counts >= PARAMS['min_valid_depth_samples']))
    if len(preferred)>=3 and all(allowed[v, u] for u, v in preferred):
        pixels = preferred
    else:
        work = np.where(allowed, distance, 0).astype(np.float32)
        pixels = []
        for _ in range(5):
            _, value, _, point = cv2.minMaxLoc(work)
            if value < PARAMS['interior_margin_px']:
                break
            pixels.append(list(point))
            cv2.circle(work, point, 5, 0, -1)
    if len(pixels) < 3:
        raise PerceptionUnknown('Insufficient real depth in colored interior windows')
    return pixels, {'radius_px': 3, 'valid_samples': [int(counts[v, u]) for u, v in pixels],
                    'reselected': pixels != preferred, 'synthesized_depth': False}


def select_block(observation, color, *, interior_pixels=None, now_ns=None, candidate_filter=None,
                 valid_depth_mask=None):
    _mapping(observation, 'observation')
    frame = observation.get('frame_id')
    head = _mapping(_mapping(observation.get('cameras'), 'cameras').get('head'), 'head camera')
    if not isinstance(frame, str) or not frame or any(c not in '0123456789abcdef' for c in frame):
        raise PerceptionUnknown('Missing or invalid native frame id')
    capture = observation.get('captured_at_ns')
    now_ns = time.time_ns() if now_ns is None else now_ns
    if type(capture) is not int or type(now_ns) is not int or not 0 <= (now_ns-capture)/1e9 <= PARAMS['max_age_s']:
        raise PerceptionUnknown('Missing, stale or future observation timestamp')
    if head.get('binding') != 'same immutable stereo/RGB packet' or head.get('metric_depth') is not True:
        raise PerceptionUnknown('Head RGB is not exactly associated with metric depth')
    image_path = head.get('inspection_image')
    if not isinstance(image_path, str) or not image_path:
        raise PerceptionUnknown('Missing head image path')
    path = Path(image_path)
    expected = Path('runtime/frames')/frame/'head.jpg'
    if path.resolve() != expected.resolve() or not path.is_file():
        raise PerceptionUnknown('Head image path/frame mismatch or missing file')
    if head.get('shape_hw') != [720,1280]:
        raise PerceptionUnknown('Unsupported native image shape')
    selected, distance = component(cv2.imread(str(path)), color, candidate_filter)
    if valid_depth_mask is not None and interior_pixels is None:
        selected['pixels'], selected['depth_selection'] = supported_depth_pixels(
            distance, valid_depth_mask, selected['pixels'])
    if interior_pixels is not None:
        if not isinstance(interior_pixels, list) or len(interior_pixels) < 3:
            raise PerceptionUnknown('Supply at least three validated interior pixels')
        for pixel in interior_pixels:
            if (not isinstance(pixel,list) or len(pixel) != 2 or
                any(type(x) is not int for x in pixel) or
                not 0 <= pixel[0] < 1280 or not 0 <= pixel[1] < 720 or
                distance[pixel[1],pixel[0]] < PARAMS['interior_margin_px']):
                raise PerceptionUnknown('Requested pixel lies outside semantic block interior')
        selected['pixels'] = interior_pixels
    return selected | {'frame_id':frame, 'image':str(path),
                        'image_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def _validated_points(observation, selected, depth):
    for label, value in [('observation', observation), ('selection', selected), ('depth', depth)]:
        _mapping(value, label)
    frame = observation.get('frame_id')
    if not isinstance(frame, str) or not frame or selected.get('frame_id') != frame:
        raise PerceptionUnknown('Missing or mismatched selection frame')
    if (depth.get('frame_id') != frame or depth.get('camera') != 'head'
        or depth.get('frame') != 'left_base_link' or depth.get('base_transform_applied') is not True):
        raise PerceptionUnknown('Depth frame/camera/transform mismatch')
    calibration = observation.get('calibration_sha256')
    if (not isinstance(calibration, str) or len(calibration) != 64
        or any(c not in '0123456789abcdef' for c in calibration)
        or depth.get('calibration_sha256') != calibration):
        raise PerceptionUnknown('RGB/depth calibration identity mismatch')
    if depth.get('robot_target_usable') is not True or type(depth.get('calibration_provisional')) is not bool:
        raise PerceptionUnknown('Missing native transform usability or calibration status')
    pixels = selected.get('pixels')
    if (not isinstance(selected.get('color'), str) or selected.get('color') not in HSV_RANGES or not isinstance(pixels, list) or len(pixels) < 3
        or any(not isinstance(px, list) or len(px) != 2 or any(type(v) is not int for v in px) for px in pixels)):
        raise PerceptionUnknown('Malformed selected pixels or color')
    points = depth.get('points')
    if not isinstance(points,list) or len(points) != len(pixels):
        raise PerceptionUnknown('Missing depth points')
    xyz = []
    for expected, point in zip(pixels, points):
        _mapping(point, 'depth point')
        pos = point.get('position')
        samples = point.get('valid_samples')
        depth_m = point.get('depth_m')
        if (point.get('pixel') != expected or type(samples) is not int or samples < PARAMS['min_valid_depth_samples']
            or not _finite(depth_m) or depth_m <= 0
            or not isinstance(pos,list) or len(pos) != 3
            or not all(_finite(v) for v in pos)):
            raise PerceptionUnknown('Invalid or insufficient native depth samples')
        xyz.append(pos)
    return np.array(xyz)


def measure_block(observation, selected, depth):
    points_array = _validated_points(observation, selected, depth)
    xyz = points_array.tolist()
    spread = float(np.ptp(points_array[:,2]))
    if spread > PARAMS['depth_spread_max_m']:
        raise PerceptionUnknown(f'Surface z spread {spread:.4f}m exceeds conditional range')
    return {'color':selected['color'], 'frame_id':observation['frame_id'],
            'surface_position_m':np.median(points_array,axis=0).tolist(),
            'surface_z_spread_m':spread, 'points_m':xyz, 'frame':'left_base_link',
            'calibration_provisional':depth.get('calibration_provisional'),
            'motion_permit':False, 'meaning':'visible interior surface, not geometric center or grasp target'}


class CenterDepthInconsistent(PerceptionUnknown):
    """Bound center measurements disagree; same-frame top evidence may recover."""


def measure_center_surface(observation, selected, depth, patch):
    # Validate every original point and its identity BEFORE excluding outliers.
    points = _validated_points(observation, selected, depth)
    if patch.get('frame_id') != observation['frame_id'] or patch.get('color') != selected['color']:
        raise PerceptionUnknown('Center/top patch identity mismatch')
    z = float(patch['surface_position_m'][2])
    spread = float(np.ptp(points[:, 2]))
    if spread <= PARAMS['depth_spread_max_m'] and abs(float(np.median(points[:, 2]))-z) <= .008:
        return measure_block(observation, selected, depth)
    keep = np.flatnonzero(np.abs(points[:, 2]-z) <= .008).tolist()
    if (len(keep) < max(3, len(points)//2+1)
            or np.ptp(points[keep, 2]) > PARAMS['depth_spread_max_m']):
        raise CenterDepthInconsistent('Center depth has no majority consistent with measured colored top')
    subset = selected | {'pixels': [selected['pixels'][i] for i in keep]}
    result = measure_block(observation, subset, depth | {'points': [depth['points'][i] for i in keep]})
    # Center XY is reconstructed by the caller from the RGB center ray. Only
    # the height combines independent measured colored and center windows.
    result['surface_position_m'][2] = float(np.median(
        [p[2] for p in patch['points_m']] + points[keep, 2].tolist()))
    result['depth_consensus'] = {'method': 'colored top plus consistent center majority',
        'original_spread_m': spread, 'accepted_center_points': len(keep),
        'rejected_pixels': [selected['pixels'][i] for i in range(len(points)) if i not in keep],
        'colored_top_height_m': z, 'synthesized_depth': False}
    return result
