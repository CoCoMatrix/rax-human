"""ZED capture service: a separate process that owns the cameras and records them.

Why a separate process: in NEURAL depth mode each ZED keeps a oneTBB pool sized to the
CPU affinity mask spinning for as long as it is open. Unpinned, three of them starved a
PREEMPT_RT kernel and panicked it. The backend therefore starts this server under
`taskset -c <cpus>` with BLAS/OMP thread caps, keeps it free of any libfranka thread, and
the server exits with its parent.

The real scene changes on its own, so every camera streams continuously into a sliding
window the agent can read but not write:

    <stream>/<camera>/<YYYYmmddTHHMM>/<epoch_ms>.jpg          RGB at the stream rate
    <stream>/<camera>/<YYYYmmddTHHMM>/<epoch_ms>_depth.png    depth (uint16 mm) at depth_hz
    <stream>/index.json                                       refreshed every second

Minute directories (UTC) older than the window are deleted as time goes by, once
services.franka.archive has encoded them into the whole-session MP4 archive.

Protocol: one JSON object per line over a private Unix socket.
  {"op": "grab", "cameras": [names], "depth": bool, "out_dir": path}
      -> {"cameras": {name: {"rgb": file, "depth": file|null, "K": 3x3, "width", "height",
                             "timestamp_ns"}}}       (the newest frames, depth on request)
  {"op": "ping"} -> {"open": [names], "frames": {...}, "dropped": {...}}
"""
import argparse
import json
import os
from pathlib import Path
import queue
import shutil
import signal
import socket
import threading
import time

import cv2 as cv
import numpy as np

cv.setNumThreads(1)


def minute_name(t_s):
    return time.strftime('%Y%m%dT%H%M', time.gmtime(t_s))


def write_atomic(path, data):
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_bytes(data)
    os.replace(temporary, path)


def depth_png(metres):
    millimetres = np.where(np.isfinite(metres) & (metres > 0), np.round(metres * 1000), 0)
    ok, png = cv.imencode('.png', np.clip(millimetres, 0, 65535).astype(np.uint16))
    return png.tobytes()


class CameraWorker:
    """One camera: grabs continuously, keeps the newest frame, feeds the recorder."""

    def __init__(self, sl, spec, *, fps, depth_mode, stream, stream_quality, depth_hz):
        self.sl, self.spec, self.name = sl, spec, spec['name']
        self.stream, self.quality, self.depth_hz = stream, stream_quality, depth_hz
        self.depth_enabled = depth_mode != 'NONE'
        self.camera = sl.Camera()
        init = sl.InitParameters()
        init.set_from_serial_number(int(spec['serial']))
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = fps
        init.coordinate_units = sl.UNIT.METER
        init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
        init.depth_minimum_distance = float(spec.get('depth_min_m', 0.3))
        init.depth_maximum_distance = float(spec.get('depth_max_m', 3.0))
        status = self.camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'camera {self.name} (S/N {spec["serial"]}) failed to open: {status}')
        left = self.camera.get_camera_information().camera_configuration.calibration_parameters.left_cam
        self.K = [[left.fx, 0.0, left.cx], [0.0, left.fy, left.cy], [0.0, 0.0, 1.0]]
        self.lock = threading.Condition()
        self.latest = None          # (timestamp_ns, bgr)
        self.latest_depth = None    # (timestamp_ns, metres)
        self.depth_wanted = 0       # pending on-demand depth requests
        self.frames = self.dropped = self.errors = 0
        self.last_depth_s = 0.0
        self.running = True
        self.records = queue.Queue(maxsize=int(fps * 2))
        threading.Thread(target=self._capture, name=f'cap-{self.name}', daemon=True).start()
        if stream is not None:
            threading.Thread(target=self._record, name=f'rec-{self.name}', daemon=True).start()

    def _capture(self):
        sl = self.sl
        image, measure = sl.Mat(), sl.Mat()
        with_depth, without_depth = sl.RuntimeParameters(), sl.RuntimeParameters()
        without_depth.enable_depth = False
        while self.running:
            now = time.time()
            want_depth = self.depth_enabled and (
                self.depth_wanted > 0 or (self.depth_hz > 0 and now - self.last_depth_s >= 1.0 / self.depth_hz))
            status = self.camera.grab(with_depth if want_depth else without_depth)
            if status != sl.ERROR_CODE.SUCCESS:
                self.errors += 1
                time.sleep(0.01)
                continue
            stamp = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            self.camera.retrieve_image(image, sl.VIEW.LEFT)
            bgr = cv.cvtColor(image.get_data(), cv.COLOR_BGRA2BGR)
            depth = None
            if want_depth:
                self.camera.retrieve_measure(measure, sl.MEASURE.DEPTH)
                depth = measure.get_data().copy()
                self.last_depth_s = now
            with self.lock:
                self.latest = (stamp, bgr)
                if depth is not None:
                    self.latest_depth = (stamp, depth, bgr)
                self.frames += 1
                self.lock.notify_all()
            if self.stream is not None:
                try:
                    self.records.put_nowait((stamp, bgr, depth))
                except queue.Full:
                    self.dropped += 1

    def _record(self):
        root = self.stream/self.name
        while True:
            stamp, bgr, depth = self.records.get()
            folder = root/minute_name(stamp / 1e9)
            folder.mkdir(parents=True, exist_ok=True)
            ms = stamp // 1_000_000
            ok, jpeg = cv.imencode('.jpg', bgr, [cv.IMWRITE_JPEG_QUALITY, self.quality])
            write_atomic(folder/f'{ms}.jpg', jpeg.tobytes())
            if depth is not None:
                write_atomic(folder/f'{ms}_depth.png', depth_png(depth))

    def newest(self, depth, after_ns, timeout=3.0):
        """(timestamp_ns, bgr, depth or None): the newest frame exposed after `after_ns`."""
        with self.lock:
            if depth and self.depth_enabled:
                self.depth_wanted += 1
                try:
                    if not self.lock.wait_for(lambda: self.latest_depth and self.latest_depth[0] > after_ns, timeout):
                        raise RuntimeError(f'camera {self.name}: no depth frame within {timeout}s')
                    stamp, metres, bgr = self.latest_depth
                    return stamp, bgr, metres
                finally:
                    self.depth_wanted -= 1
            if not self.lock.wait_for(lambda: self.latest and self.latest[0] > after_ns, timeout):
                raise RuntimeError(f'camera {self.name}: no frame within {timeout}s')
            stamp, bgr = self.latest
            return stamp, bgr, None

    def close(self):
        self.running = False
        time.sleep(0.1)
        self.camera.close()
        deadline = time.monotonic() + 10
        while not self.records.empty() and time.monotonic() < deadline:  # let the recorder catch up
            time.sleep(0.05)


class Cameras:
    def __init__(self, specs, *, depth_mode, fps, frames_root, stream, window_s, stream_quality, depth_hz,
                 archive=None):
        import pyzed.sl as sl
        self.frames_root = Path(frames_root).resolve()
        self.stream = Path(stream).resolve() if stream else None
        self.window_s, self.fps, self.depth_hz = window_s, fps, depth_hz
        self.workers = {}
        try:
            for spec in specs:  # open everything now: the stream runs for the whole session
                self.workers[spec['name']] = CameraWorker(sl, spec, fps=fps, depth_mode=depth_mode,
                                                          stream=self.stream, stream_quality=stream_quality,
                                                          depth_hz=depth_hz)
        except BaseException:
            self.close()
            raise
        self.archiver = None
        if self.stream is not None and archive:
            from .archive import Archiver
            self.archiver = Archiver(self.stream, archive, list(self.workers), fps=fps,
                                     log=lambda message: print(message, flush=True))
        if self.stream is not None:
            threading.Thread(target=self._maintain, name='stream-maintenance', daemon=True).start()

    def prune(self, now):
        """Delete minutes older than the window, but only once archived (or 5 minutes later)."""
        oldest_kept = minute_name(now - self.window_s)
        last_resort = minute_name(now - self.window_s - 300)
        archive = getattr(self, 'archiver', None)
        for entry in self.stream.iterdir():
            if not entry.is_dir():
                continue
            for minute in entry.iterdir():
                name = minute.stem if minute.suffix == '.jsonl' else minute.name
                if name >= oldest_kept or not (minute.is_dir() or minute.suffix == '.jsonl'):
                    continue
                kept = archive is not None and name >= last_resort and not (
                    archive.archived(entry.name, name) if minute.is_dir()
                    else (archive.archive/'robot'/minute.name).exists())
                if kept:
                    continue
                if minute.is_dir():
                    shutil.rmtree(minute, ignore_errors=True)
                else:
                    minute.unlink(missing_ok=True)

    def _maintain(self):
        previous = {}
        while True:
            now = time.time()
            try:
                self.prune(now)
                cameras = {}
                for name, worker in self.workers.items():
                    frames = worker.frames
                    rate = frames - previous.get(name, frames)
                    previous[name] = frames
                    folder = self.stream/name
                    minutes = sorted(p.name for p in folder.iterdir() if p.is_dir()) if folder.is_dir() else []
                    latest = worker.latest[0] if worker.latest else None
                    cameras[name] = {'latest_ms': latest // 1_000_000 if latest else None,
                                     'oldest_minute': minutes[0] if minutes else None, 'fps_measured': rate,
                                     'dropped_frames': worker.dropped, 'grab_errors': worker.errors}
                index = {'updated_ms': int(now * 1000), 'window_s': self.window_s, 'fps': self.fps,
                         'depth_hz': self.depth_hz,
                         'layout': '<camera>/<UTC YYYYmmddTHHMM>/<epoch_ms>.jpg and <epoch_ms>_depth.png '
                                   '(uint16 mm, 0 = invalid); robot/<UTC YYYYmmddTHHMM>.jsonl',
                         'cameras': cameras,
                         'archive': self.archiver.status() if self.archiver else None}
                write_atomic(self.stream/'index.json', json.dumps(index, indent=1).encode())
            except OSError as exc:
                print(f'stream maintenance: {exc}', flush=True)
            time.sleep(max(0.0, 1.0 - (time.time() - now)))

    def grab(self, names, depth, out_dir):
        out_dir = Path(out_dir).resolve()
        if out_dir.parent != self.frames_root:  # never write anywhere else
            raise ValueError('out_dir must be a direct child of the frames root')
        for name in names:
            if name not in self.workers:
                raise ValueError(f'unknown camera {name!r}; configured: {sorted(self.workers)}')
        out_dir.mkdir(parents=True, exist_ok=True)
        after = time.time_ns() - 50_000_000  # exposed after the request (50 ms slack)
        result = {}
        for name in names:
            worker = self.workers[name]
            stamp, bgr, metres = worker.newest(depth, after)
            rgb_file = out_dir/f'{name}.jpg'
            cv.imwrite(str(rgb_file), bgr, [cv.IMWRITE_JPEG_QUALITY, 95])
            entry = {'rgb': str(rgb_file), 'depth': None, 'K': worker.K, 'width': bgr.shape[1],
                     'height': bgr.shape[0], 'timestamp_ns': stamp}
            if metres is not None:
                depth_file = out_dir/f'{name}_depth.png'
                write_atomic(depth_file, depth_png(metres))
                entry['depth'] = str(depth_file)
            result[name] = entry
        return result

    def close(self):
        for worker in self.workers.values():
            worker.close()
        if self.archiver is not None:
            self.archiver.flush()  # the minute in progress too: keep the end of the session


def serve(listener, cameras):
    while True:
        connection, _ = listener.accept()
        with connection, connection.makefile('r') as reader, connection.makefile('w') as writer:
            for line in reader:
                try:
                    request = json.loads(line)
                    op = request.get('op')
                    if op == 'grab':
                        value = {'cameras': cameras.grab(request['cameras'], bool(request.get('depth', True)),
                                                         request['out_dir'])}
                    elif op == 'ping':
                        value = {'open': sorted(cameras.workers), 'pid': os.getpid(),
                                 'affinity': sorted(os.sched_getaffinity(0)),
                                 'frames': {n: w.frames for n, w in cameras.workers.items()},
                                 'dropped': {n: w.dropped for n, w in cameras.workers.items()}}
                    else:
                        raise ValueError(f'unknown op {op!r}')
                    response = {'ok': True, **value}
                except Exception as exc:  # report, keep serving
                    response = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
                writer.write(json.dumps(response) + '\n')
                writer.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', required=True)
    parser.add_argument('--cameras', required=True, help='JSON list of camera specs')
    parser.add_argument('--depth-mode', default='NEURAL')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--frames-root', required=True)
    parser.add_argument('--stream', help='sliding-window recording directory (omit to disable)')
    parser.add_argument('--window-seconds', type=float, default=600.0)
    parser.add_argument('--stream-quality', type=int, default=80)
    parser.add_argument('--depth-hz', type=float, default=1.0)
    parser.add_argument('--archive', help='whole-session MP4 archive directory (omit to disable)')
    args = parser.parse_args()

    def stop(*_):
        raise SystemExit(0)  # unwinds to the finally below, which closes the cameras
    signal.signal(signal.SIGTERM, stop)
    # Die with the robot service even if it is SIGKILLed: never leave cameras streaming.
    import ctypes
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    if os.getppid() == 1:
        raise SystemExit('parent exited before the camera server started')
    if args.stream:
        Path(args.stream).mkdir(parents=True, exist_ok=True)
    cameras = Cameras(json.loads(args.cameras), depth_mode=args.depth_mode, fps=args.fps,
                      frames_root=args.frames_root, stream=args.stream, window_s=args.window_seconds,
                      stream_quality=args.stream_quality, depth_hz=args.depth_hz, archive=args.archive)
    path = Path(args.socket)
    path.unlink(missing_ok=True)
    try:
        with socket.socket(socket.AF_UNIX) as listener:
            listener.bind(str(path))
            os.chmod(path, 0o600)
            listener.listen(4)
            print(f'camera server on {path}', flush=True)
            serve(listener, cameras)
    finally:
        cameras.close()
        path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
