"""Whole-session MP4 archive of the live stream (ffmpeg required; no cameras)."""
import json
import shutil
import time

import cv2 as cv
import numpy as np
import pytest

from services.franka.archive import Archiver, encode_minute, minute_of
from services.franka.camera_server import Cameras

pytestmark = pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='ffmpeg not installed')


def fill_minute(folder, start_ms, n=20, step_ms=33):
    folder.mkdir(parents=True, exist_ok=True)
    for k in range(n):
        image = np.zeros((144, 256, 3), np.uint8)
        cv.putText(image, str(k), (20, 100), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        cv.imwrite(str(folder/f'{start_ms + k * step_ms}.jpg'), image)
    cv.imwrite(str(folder/f'{start_ms}_depth.png'), np.zeros((144, 256), np.uint16))  # never archived


@pytest.mark.parametrize('codec', ['libx264', 'h264_nvenc'])
def test_encode_minute_keeps_every_frame_and_its_time(tmp_path, codec):
    from services.franka.archive import pick_codec
    if codec == 'h264_nvenc' and pick_codec() != 'h264_nvenc':
        pytest.skip('no NVENC here')
    minute = tmp_path/'left'/'20261003T2342'
    fill_minute(minute, 1_791_070_920_000)
    sidecar = encode_minute(minute, tmp_path/'archive'/'left'/'20261003T2342.mp4', fps=30, codec=codec)
    video = cv.VideoCapture(str(tmp_path/'archive'/'left'/'20261003T2342.mp4'))
    assert int(video.get(cv.CAP_PROP_FRAME_COUNT)) == 20 == sidecar['frames']
    assert sidecar['frames_ms'][:2] == [1_791_070_920_000, 1_791_070_920_033]
    assert json.loads((tmp_path/'archive'/'left'/'20261003T2342.json').read_text()) == sidecar


def test_archiver_archives_completed_minutes_and_flushes_the_rest(tmp_path):
    now = time.time()
    done, current = minute_of(now - 120), minute_of(now)
    for name in (done, current):
        fill_minute(tmp_path/'stream'/'left'/name, int(now * 1000), n=5)
    (tmp_path/'stream'/'robot').mkdir()
    (tmp_path/'stream'/'robot'/f'{done}.jsonl').write_text('{"t_ms": 1}\n')
    archiver = Archiver(tmp_path/'stream', tmp_path/'archive', ['left'], fps=30, codec='libx264')
    archiver.running = False  # drive it by hand
    archiver.run_once(minute_of(now))
    assert archiver.archived('left', done) and not archiver.archived('left', current)
    assert (tmp_path/'archive'/'robot'/f'{done}.jsonl').exists()
    archiver.flush()
    assert archiver.archived('left', current)


def test_prune_keeps_unarchived_minutes_until_the_last_resort(tmp_path):
    now = time.time()
    cameras = Cameras.__new__(Cameras)
    cameras.stream, cameras.window_s = tmp_path/'stream', 60
    names = {age: minute_of(now - age) for age in (0, 180, 900)}
    for name in names.values():
        (tmp_path/'stream'/'left'/name).mkdir(parents=True)
    archiver = Archiver.__new__(Archiver)
    archiver.archive = tmp_path/'archive'
    cameras.archiver = archiver
    cameras.prune(now)
    kept = {p.name for p in (tmp_path/'stream'/'left').iterdir()}
    # 180 s old: outside the 60 s window but not archived -> kept; 900 s: past the last resort -> deleted.
    assert kept == {names[0], names[180]}
    (tmp_path/'archive'/'left').mkdir(parents=True)
    (tmp_path/'archive'/'left'/f'{names[180]}.mp4').write_bytes(b'x')
    cameras.prune(now)
    assert {p.name for p in (tmp_path/'stream'/'left').iterdir()} == {names[0]}
