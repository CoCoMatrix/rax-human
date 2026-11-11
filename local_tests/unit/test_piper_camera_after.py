"""Post-action observations require source frames after the completed motion."""
import json
import time

import cv2
import numpy as np
import pytest

from services.piper.cameras import Cameras
from test_piper_adapter import Owner, depth_packet


def clock(monkeypatch):
    now = {'s': 0.}
    monkeypatch.setattr('services.piper.cameras.time.monotonic', lambda: now['s'])
    monkeypatch.setattr('services.piper.cameras.time.sleep', lambda seconds: now.update(s=now['s'] + seconds))
    return now


def test_head_waits_for_new_source_and_never_saves_old_candidate(tmp_path, monkeypatch):
    now = clock(monkeypatch)
    cameras = Cameras(Owner(), tmp_path / 'frames')
    attempts = []

    def depth(**kwargs):
        assert kwargs == {'after_ns': 100}
        assert not cameras.frames.exists()
        stamp = 90 if not attempts else 110
        attempts.append(stamp)
        rgb = np.full((12, 16, 3), stamp, np.uint8)
        return {'rgb': rgb, 'xyz_rgb_m': np.ones(rgb.shape), 'valid': np.ones(rgb.shape[:2], bool)}, {'host_received_unix_ns': stamp}

    monkeypatch.setattr(cameras, 'depth', depth)
    meta, images = cameras.capture(['head'], True, after_ns=100)
    assert attempts == [90, 110] and now['s'] == .05
    assert meta['after_ns'] == 100
    assert meta['cameras']['head']['source_timestamp_ns'] == 110
    assert np.all(images[0][1] == 110)
    assert len(list(cameras.frames.iterdir())) == 1
    saved = json.loads((cameras.frames / meta['frame_id'] / 'meta.json').read_text())
    assert saved['cameras']['head']['source']['host_received_unix_ns'] == 110
    frame=cameras.frames/meta['frame_id']
    assert saved['cameras']['head']['inspection_image'].endswith('/head.jpg')
    assert cv2.imread(str(frame/'head.jpg')).shape==(12,16,3)
    assert np.all(cv2.imread(str(frame/'head.png'))==110)


@pytest.mark.parametrize('camera', ['head', 'left', 'right'])
def test_rgb_bracket_before_must_be_after_motion_even_when_after_is_new(tmp_path, monkeypatch, camera):
    clock(monkeypatch)
    owner = Owner()
    cameras = Cameras(owner, tmp_path / 'frames')
    stamps = []

    def capture(name):
        assert name == camera and not cameras.frames.exists()
        before = 90 if not stamps else 110
        stamps.append(before)
        return np.zeros((12, 16, 3), np.uint8), {'binding': 'HTTP bracket',
                                               'before': {'timestamp_ns': before},
                                               'after': {'timestamp_ns': 120}}

    monkeypatch.setattr(owner, 'camera', capture)
    meta, _ = cameras.capture([camera], False, after_ns=100)
    assert stamps == [90, 110]
    assert meta['cameras'][camera]['source']['before']['timestamp_ns'] == 110
    assert meta['cameras'][camera]['source_timestamp_ns'] == 120


def test_wait_timeout_has_no_frame_directory_or_rewritten_source(tmp_path, monkeypatch):
    now = clock(monkeypatch)
    cameras = Cameras(Owner(), tmp_path / 'frames')
    report = {'host_received_unix_ns': 90}
    rgb = np.zeros((12, 16, 3), np.uint8)
    monkeypatch.setattr(cameras, 'depth', lambda **kwargs: ({'rgb': rgb}, report))
    with pytest.raises(TimeoutError, match='post-action'):
        cameras.capture(['head'], True, after_ns=100)
    assert 1.99 <= now['s'] <= 2.01
    assert not cameras.frames.exists() and report['host_received_unix_ns'] == 90


@pytest.mark.parametrize('reason', ['Head depth provenance mismatch', 'Live depth is stale/future'])
def test_invalid_depth_fails_immediately_without_retry_or_fallback(tmp_path, monkeypatch, reason):
    now = clock(monkeypatch)
    cameras = Cameras(Owner(), tmp_path / 'frames')
    attempts = []

    def depth(**kwargs):
        attempts.append(now['s'])
        raise ValueError(reason)

    monkeypatch.setattr(cameras, 'depth', depth)
    with pytest.raises(ValueError, match=reason):
        cameras.capture(['head'], True, after_ns=100)
    assert len(attempts) == 1 and now['s'] == 0
    assert not cameras.frames.exists()


def test_waiting_does_not_relax_ffs_ttl(tmp_path, monkeypatch):
    depth_packet(tmp_path / 'depth', age_ns=2_000_000_000)
    now = clock(monkeypatch)
    cameras = Cameras(Owner(), tmp_path / 'frames', tmp_path / 'depth')
    with pytest.raises(ValueError, match='stale/future'):
        cameras.capture(['head'], True, after_ns=1)
    assert now['s'] == 0
    assert not cameras.frames.exists()


def test_plain_observe_keeps_existing_no_after_boundary_semantics(tmp_path, monkeypatch):
    cameras = Cameras(Owner(), tmp_path / 'frames')
    rgb = np.zeros((12, 16, 3), np.uint8)
    monkeypatch.setattr(cameras, 'depth', lambda: ({'rgb': rgb}, {'host_received_unix_ns': 90}))
    meta, _ = cameras.capture(['head'], True)
    assert 'after_ns' not in meta and meta['cameras']['head']['source_timestamp_ns'] == 90


def test_real_depth_metadata_waits_without_loading_old_arrays(tmp_path, monkeypatch):
    root = tmp_path / 'depth'
    report = depth_packet(root, age_ns=2_000_000_000)
    boundary = time.time_ns()
    now = clock(monkeypatch)
    cameras = Cameras(Owner(), tmp_path / 'frames', root)
    loads = []
    original_load = np.load

    def load(*args, **kwargs):
        loads.append(report['host_received_unix_ns'])
        return original_load(*args, **kwargs)

    def publish_new(seconds):
        assert not cameras.frames.exists() and not loads
        now['s'] += seconds
        report.update(host_received_unix_ns=boundary + 1, expires_unix_ns=boundary + 1_000_000_001)
        (root / 'sample/report.json').write_text(json.dumps(report))
        (root / 'latest.json').write_text(json.dumps({**report, 'result_dir': 'sample'}))

    monkeypatch.setattr('services.piper.cameras.time.sleep', publish_new)
    monkeypatch.setattr('services.piper.cameras.np.load', load)
    meta, _ = cameras.capture(['head'], True, after_ns=boundary)
    assert loads == [boundary + 1]
    assert meta['after_ns'] == boundary
    assert meta['cameras']['head']['source_timestamp_ns'] == boundary + 1


def test_head_save_keeps_original_ttl_and_exact_arrays_without_compression(tmp_path,monkeypatch):
    import zipfile
    root=tmp_path/'depth';report=depth_packet(root)
    cameras=Cameras(Owner(),tmp_path/'frames',root)
    stamp=report['host_received_unix_ns'];now={'ns':stamp+734_177_351}
    monkeypatch.setattr('services.piper.cameras.time.time_ns',lambda:now['ns'])
    compress=np.savez_compressed
    def costly_compress(*args,**kwargs):
        compress(*args,**kwargs);now['ns']+=300_000_000
    monkeypatch.setattr('services.piper.cameras.np.savez_compressed',costly_compress)
    obs,_=cameras.capture(['head','left','right'],True)
    assert obs['captured_at_ns']-stamp<1_000_000_000
    assert obs['cameras']['head']['source_timestamp_ns']==stamp
    assert obs['cameras']['head']['source']['expires_unix_ns']==stamp+1_000_000_000
    path=cameras.frames/obs['frame_id']/'head-depth.npz'
    with zipfile.ZipFile(path) as archive:assert all(x.compress_type==zipfile.ZIP_STORED for x in archive.infolist())
    with np.load(root/'sample/observation.npz') as a,np.load(path) as b:
        for key in a.files:np.testing.assert_array_equal(a[key],b[key])


def test_depth_expiring_during_artifact_save_is_not_returned_as_live(tmp_path,monkeypatch):
    root=tmp_path/'depth';report=depth_packet(root);stamp=report['host_received_unix_ns']
    cameras=Cameras(Owner(),tmp_path/'frames',root);now={'ns':stamp+100_000_000}
    monkeypatch.setattr('services.piper.cameras.time.time_ns',lambda:now['ns'])
    encode=cv2.imencode
    def slow_encode(*args,**kwargs):
        result=encode(*args,**kwargs);now['ns']+=500_000_000;return result
    monkeypatch.setattr('services.piper.cameras.cv2.imencode',slow_encode)
    with pytest.raises(ValueError,match='expired during snapshot'):
        cameras.capture(['head'],True)
    assert report['expires_unix_ns']==stamp+1_000_000_000
