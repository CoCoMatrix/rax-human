"""Read side of the sliding-window recording written by camera_server and the robot log."""
import json
from pathlib import Path
import time

import cv2 as cv
import numpy as np


def frame_times(stream, camera, since_ms, until_ms):
    """[(t_ms, path)] of RGB frames of one camera within [since_ms, until_ms], in time order."""
    result = []
    folder = Path(stream)/camera
    if not folder.is_dir():
        return result
    first = time.strftime('%Y%m%dT%H%M', time.gmtime(since_ms / 1000))
    last = time.strftime('%Y%m%dT%H%M', time.gmtime(until_ms / 1000))
    for minute in sorted(p for p in folder.iterdir() if p.is_dir() and first <= p.name <= last):
        for path in minute.iterdir():
            name = path.name
            if name.endswith('.jpg') and not name.startswith('.'):
                t = int(name[:-4])
                if since_ms <= t <= until_ms:
                    result.append((t, path))
    return sorted(result)


def nearest(stream, camera, t_ms, tolerance_ms=1000):
    candidates = frame_times(stream, camera, t_ms - tolerance_ms, t_ms + tolerance_ms)
    return min(candidates, key=lambda item: abs(item[0] - t_ms)) if candidates else None


def robot_at(stream, t_ms):
    """The robot log record closest to t_ms (within 1 s), or None."""
    best = None
    for minute in {time.strftime('%Y%m%dT%H%M', time.gmtime((t_ms + d) / 1000)) for d in (-1000, 0, 1000)}:
        path = Path(stream)/'robot'/f'{minute}.jsonl'
        if not path.is_file():
            continue
        for line in path.read_text().splitlines():
            try:
                record = json.loads(line)
            except ValueError:
                continue
            if 'error' in record:
                continue
            if best is None or abs(record['t_ms'] - t_ms) < abs(best['t_ms'] - t_ms):
                best = record
    return best if best and abs(best['t_ms'] - t_ms) <= 1000 else None


def label(image, text):
    cv.putText(image, text, (8, 22), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
    cv.putText(image, text, (8, 22), cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return image


def contact_sheet(stream, camera, since_ms, until_ms, count, now_ms, tile_width=320, columns=4):
    """One image of `count` evenly spaced frames, each labelled with its age; and their times."""
    selected = frame_times(stream, camera, since_ms, until_ms)
    if not selected:
        return None, []
    count = min(count, len(selected))
    picks = [selected[round(i * (len(selected) - 1) / max(1, count - 1))] for i in range(count)]
    tiles = []
    for t, path in picks:
        image = cv.imread(str(path))
        image = cv.resize(image, (tile_width, int(image.shape[0] * tile_width / image.shape[1])),
                          interpolation=cv.INTER_AREA)
        tiles.append(label(image, f'{camera} -{(now_ms - t) / 1000:.1f}s'))
    while len(tiles) % columns:
        tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[i:i + columns]) for i in range(0, len(tiles), columns)])
    return grid, [t for t, _ in picks]
