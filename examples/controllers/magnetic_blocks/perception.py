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
              'orange': [((10, 110, 85), (28, 255, 255))],
              'green': [((35, 70, 45), (88, 255, 255))]}


class PerceptionUnknown(Exception):
    pass


def _mapping(value, label):
    if not isinstance(value, dict):
        raise PerceptionUnknown(f'Missing or malformed {label}')
    return value


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def component(image, color):
    """Pure detector; ambiguous/absent/truncated candidates fail closed."""
    if not isinstance(color, str) or color not in HSV_RANGES:
        raise PerceptionUnknown('Choose color red, orange, or green explicitly')
    if not isinstance(image, np.ndarray) or image.shape != (720, 1280, 3) or image.dtype != np.uint8:
        raise PerceptionUnknown('Expected native head image 1280x720 BGR')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    candidates = []
    for i in range(1, count):
        x, y, w, h, area = map(int, stats[i])
        if (PARAMS['area_min'] <= area <= PARAMS['area_max'] and
            PARAMS['size_min'] <= w <= PARAMS['size_max'] and
            PARAMS['size_min'] <= h <= PARAMS['size_max'] and
            PARAMS['aspect_min'] <= w/h <= PARAMS['aspect_max'] and
            area/(w*h) >= PARAMS['fill_min']):
            candidates.append((i, x, y, w, h, area))
    if len(candidates) != 1:
        raise PerceptionUnknown(f'Expected one {color} block candidate; found {len(candidates)}')
    i, x, y, w, h, area = candidates[0]
    if x == 0 or y == 0 or x+w == image.shape[1] or y+h == image.shape[0]:
        raise PerceptionUnknown('Candidate touches image border')
    distance = cv2.distanceTransform((labels == i).astype(np.uint8), cv2.DIST_L2, 5)
    _, max_distance, _, center = cv2.minMaxLoc(distance)
    offsets = ((0,0),(-6,0),(6,0),(0,-6),(0,6))
    pixels = [[center[0]+dx, center[1]+dy] for dx, dy in offsets]
    if any(distance[v,u] < PARAMS['interior_margin_px'] for u,v in pixels):
        raise PerceptionUnknown('Insufficient interior for five depth windows')
    return {'color': color, 'bbox_xywh': [x,y,w,h], 'area_px': area,
            'pixels': pixels, 'max_interior_distance_px': float(max_distance)}, distance


def select_block(observation, color, *, interior_pixels=None, now_ns=None):
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
    selected, distance = component(cv2.imread(str(path)), color)
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


def measure_block(observation, selected, depth):
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
    points_array = np.array(xyz)
    spread = float(np.ptp(points_array[:,2]))
    if spread > PARAMS['depth_spread_max_m']:
        raise PerceptionUnknown(f'Surface z spread {spread:.4f}m exceeds conditional range')
    return {'color':selected['color'], 'frame_id':observation['frame_id'],
            'surface_position_m':np.median(points_array,axis=0).tolist(),
            'surface_z_spread_m':spread, 'points_m':xyz, 'frame':'left_base_link',
            'calibration_provisional':depth.get('calibration_provisional'),
            'motion_permit':False, 'meaning':'visible interior surface, not geometric center or grasp target'}
