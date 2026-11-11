"""Read the live recording; consult index.json for actual rates and availability.

The world changes without you: people move objects, things settle or fall, and time
passes while you think. `runtime/stream/` (read-only) always holds the recent past:

    python -m api.stream latest                  # newest frame per camera and its age
    python -m api.stream sheet left --seconds 60 --count 12   # contact sheet -> output/
    python -m api.stream changes left --seconds 300           # when did the scene change?
    python -m api.stream robot --seconds 30                   # arm / gripper history

or from Python: `from api import stream; stream.latest()`.
Times are epoch milliseconds; see each backend's timestamp binding in index.json.
"""
import argparse
import json
from pathlib import Path
import time

STREAM = Path(__file__).resolve().parents[1]/'runtime'/'stream'
OUTPUT = Path(__file__).resolve().parents[1]/'output'/'stream'


def index():
    return json.loads((STREAM/'index.json').read_text())


def cameras():
    return sorted(index()['cameras'])


def frames(camera, *, seconds=None, since_ms=None, until_ms=None, depth=False):
    """[(t_ms, path)] in time order for one camera, optionally within a time range."""
    now_ms = int(time.time() * 1000)
    if seconds is not None:
        since_ms = now_ms - int(seconds * 1000)
    suffix = '_depth.png' if depth else '.jpg'
    result = []
    folder = STREAM/camera
    for minute in sorted(p for p in folder.iterdir() if p.is_dir()) if folder.is_dir() else []:
        for path in minute.iterdir():
            name = path.name
            if not name.endswith(suffix) or name.startswith('.') or (not depth and name.endswith('_depth.png')):
                continue
            t = int(name[:-len(suffix)])
            if (since_ms is None or t >= since_ms) and (until_ms is None or t <= until_ms):
                result.append((t, path))
    return sorted(result)


def latest(camera=None):
    """{camera: {'t_ms', 'age_s', 'path'}} for the newest recorded frame."""
    now_ms = int(time.time() * 1000)
    out = {}
    for name in [camera] if camera else cameras():
        recent = frames(name, seconds=5) or frames(name)
        if recent:
            t, path = recent[-1]
            out[name] = {'t_ms': t, 'age_s': round((now_ms - t) / 1000, 2), 'path': str(path)}
    return out


def sheet(camera, *, seconds=60, count=12, columns=4, width=480, out=None):
    """Contact sheet of `count` evenly spaced frames over the last `seconds`; returns its path."""
    import cv2 as cv
    import numpy as np
    selected = frames(camera, seconds=seconds)
    if not selected:
        raise RuntimeError(f'no frames for {camera} in the last {seconds}s')
    picks = [selected[round(i * (len(selected) - 1) / max(1, count - 1))] for i in range(min(count, len(selected)))]
    now_ms = int(time.time() * 1000)
    tiles = []
    for t, path in picks:
        image = cv.imread(str(path))
        image = cv.resize(image, (width, int(image.shape[0] * width / image.shape[1])))
        label = f'{camera} -{(now_ms - t) / 1000:.1f}s  {time.strftime("%H:%M:%S", time.localtime(t / 1000))}'
        cv.putText(image, label, (8, 24), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv.putText(image, label, (8, 24), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        tiles.append(image)
    while len(tiles) % columns:
        tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[i:i + columns]) for i in range(0, len(tiles), columns)])
    out = Path(out) if out else OUTPUT/f'sheet_{camera}_{now_ms}.jpg'
    out.parent.mkdir(parents=True, exist_ok=True)
    cv.imwrite(str(out), grid, [cv.IMWRITE_JPEG_QUALITY, 85])
    return out


def changes(camera, *, seconds=120, every_s=0.5, threshold=4.0):
    """When the image changed: [(t_ms, score)] for consecutive samples `every_s` apart whose
    mean absolute grey-level difference (0-255, on a 160x90 thumbnail) exceeds `threshold`.
    Arm motion also counts as change; compare with `robot()` to separate the two."""
    import cv2 as cv
    import numpy as np
    selected = frames(camera, seconds=seconds)
    sampled, last = [], None
    for t, path in selected:
        if last is None or t - last >= every_s * 1000:
            sampled.append((t, path))
            last = t
    events, previous = [], None
    for t, path in sampled:
        grey = cv.resize(cv.imread(str(path), cv.IMREAD_GRAYSCALE), (160, 90)).astype(np.float32)
        if previous is not None:
            score = float(np.mean(np.abs(grey - previous)))
            if score > threshold:
                events.append((t, round(score, 2)))
        previous = grey
    return events


def robot(*, seconds=30):
    """Arm/gripper records (t_ms, tcp_position, tcp_quaternion, joints, gripper_width)."""
    since = int(time.time() * 1000) - int(seconds * 1000)
    records = []
    folder = STREAM/'robot'
    for path in sorted(folder.glob('*.jsonl')) if folder.is_dir() else []:
        for line in path.read_text().splitlines():
            try:
                record = json.loads(line)
            except ValueError:
                continue  # the line being written right now
            if record.get('t_ms', 0) >= since:
                records.append(record)
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('latest')
    p = sub.add_parser('sheet')
    p.add_argument('camera')
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--count', type=int, default=12)
    p = sub.add_parser('changes')
    p.add_argument('camera')
    p.add_argument('--seconds', type=float, default=120)
    p.add_argument('--every', type=float, default=0.5)
    p.add_argument('--threshold', type=float, default=4.0)
    p = sub.add_parser('robot')
    p.add_argument('--seconds', type=float, default=30)
    args = parser.parse_args()
    if args.command == 'latest':
        print(json.dumps(latest(), indent=1))
    elif args.command == 'sheet':
        print(sheet(args.camera, seconds=args.seconds, count=args.count))
    elif args.command == 'changes':
        now_ms = int(time.time() * 1000)
        for t, score in changes(args.camera, seconds=args.seconds, every_s=args.every, threshold=args.threshold):
            print(f'-{(now_ms - t) / 1000:7.1f}s  {time.strftime("%H:%M:%S", time.localtime(t / 1000))}  {score}')
    else:
        for record in robot(seconds=args.seconds):
            print(json.dumps(record))


if __name__ == '__main__':
    main()
