"""Whole-session archive of the live stream: one H.264 MP4 per camera per minute.

The live stream (camera_server) keeps only a sliding window. As each minute completes,
this archiver encodes its JPEG frames into

    <archive>/<camera>/<UTC YYYYmmddTHHMM>.mp4    H.264, nominal stream fps
    <archive>/<camera>/<UTC YYYYmmddTHHMM>.json   exact capture time of every frame (epoch ms)
    <archive>/robot/<UTC YYYYmmddTHHMM>.jsonl     the arm/gripper log of that minute

so the session can be replayed in sync with the agent's transcript and tool log (all
epoch-timestamped). Frame k of an MP4 is `frames_ms[k]` in its sidecar; playback at the
nominal fps is only approximately real time. Depth is not archived (~10 GB/hour).
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time

CODECS = {
    'h264_nvenc': ['-c:v', 'h264_nvenc', '-preset', 'p5', '-cq', '28', '-b:v', '0'],
    'libx264': ['-c:v', 'libx264', '-preset', 'veryfast', '-crf', '26', '-threads', '4'],
}


def minute_of(t_s):
    return time.strftime('%Y%m%dT%H%M', time.gmtime(t_s))


def pick_codec(ffmpeg='ffmpeg'):
    """NVENC when this GPU/driver accepts a session, else libx264."""
    probe = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i', 'testsrc=size=256x144:rate=30',
             '-t', '0.2', *CODECS['h264_nvenc'], '-f', 'null', '-']
    try:
        return 'h264_nvenc' if subprocess.run(probe, capture_output=True, timeout=30).returncode == 0 else 'libx264'
    except (OSError, subprocess.TimeoutExpired):
        return 'libx264'


def frame_files(minute_dir):
    """[(t_ms, path)] RGB frames of one minute directory, in time order."""
    files = []
    for path in minute_dir.iterdir():
        name = path.name
        if name.endswith('.jpg') and not name.startswith('.'):
            files.append((int(name[:-4]), path))
    return sorted(files)


def encode_minute(minute_dir, out_mp4, *, fps, codec, ffmpeg='ffmpeg'):
    """Encode one minute of JPEGs to out_mp4 (+ .json sidecar). Returns the sidecar dict or None."""
    frames = frame_files(minute_dir)
    if not frames:
        return None
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    temporary = out_mp4.with_name(f'.{out_mp4.name}')
    command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'image2pipe', '-framerate', str(fps),
               '-c:v', 'mjpeg', '-i', '-', *CODECS[codec], '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
               '-f', 'mp4', str(temporary)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for _, path in frames:
            try:
                process.stdin.write(path.read_bytes())
            except FileNotFoundError:  # pruned meanwhile; keep the timeline honest
                frames = [f for f in frames if f[1] != path]
        process.stdin.close()
        error = process.stderr.read().decode(errors='replace')
        if process.wait(timeout=300) != 0:
            raise RuntimeError(f'ffmpeg {codec} failed: {error[-500:]}')
    except BaseException:
        process.kill()
        temporary.unlink(missing_ok=True)
        raise
    sidecar = {'camera': minute_dir.parent.name, 'minute_utc': minute_dir.name, 'fps_nominal': fps,
               'codec': codec, 'frames': len(frames), 'frames_ms': [t for t, _ in frames]}
    out_mp4.with_suffix('.json').write_text(json.dumps(sidecar))
    os.replace(temporary, out_mp4)
    return sidecar


class Archiver:
    """Archives completed minutes in the background; flush() archives everything left."""

    def __init__(self, stream, archive, cameras, *, fps, codec=None, grace_s=3.0, log=print):
        self.stream, self.archive = Path(stream), Path(archive)
        self.cameras, self.fps, self.grace_s, self.log = list(cameras), fps, grace_s, log
        self.codec = codec or pick_codec()
        self.lock = threading.Lock()
        self.encoded = self.failures = 0
        self.failed = set()
        self.running = True
        self.archive.mkdir(parents=True, exist_ok=True)
        self.thread = threading.Thread(target=self._loop, name='archiver', daemon=True)
        self.thread.start()

    def archived(self, camera, minute):
        return (self.archive/camera/f'{minute}.mp4').exists()

    def _pending(self, before_minute):
        for camera in self.cameras:
            folder = self.stream/camera
            for minute in sorted(p for p in folder.iterdir() if p.is_dir()) if folder.is_dir() else []:
                if (before_minute is None or minute.name < before_minute) and not self.archived(camera, minute.name) \
                        and (camera, minute.name) not in self.failed:
                    yield camera, minute
        robot = self.stream/'robot'
        for path in sorted(robot.glob('*.jsonl')) if robot.is_dir() else []:
            if (before_minute is None or path.stem < before_minute) and \
                    not (self.archive/'robot'/path.name).exists():
                yield 'robot', path

    def _archive(self, camera, source):
        if camera == 'robot':
            (self.archive/'robot').mkdir(exist_ok=True)
            shutil.copyfile(source, self.archive/'robot'/source.name)
            return
        try:
            encode_minute(source, self.archive/camera/f'{source.name}.mp4', fps=self.fps, codec=self.codec)
            self.encoded += 1
        except Exception as exc:
            self.failures += 1
            self.failed.add((camera, source.name))
            self.log(f'archive {camera}/{source.name}: {exc}')

    def run_once(self, before_minute):
        with self.lock:
            for camera, source in list(self._pending(before_minute)):
                self._archive(camera, source)

    def _loop(self):
        while self.running:
            self.run_once(minute_of(time.time() - self.grace_s))
            time.sleep(5)

    def flush(self):
        """Archive every remaining minute, including the one in progress (at shutdown)."""
        self.running = False
        self.run_once(None)

    def status(self):
        return {'directory': str(self.archive), 'codec': self.codec, 'minutes_encoded': self.encoded,
                'failures': self.failures}
