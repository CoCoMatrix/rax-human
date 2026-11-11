import copy
import json
from pathlib import Path

import numpy as np
import pytest

from real_robot.piper.stereo_live import StereoLiveWorker, read_fresh_depth
from real_robot.piper.tests.test_stereo_pipeline import setup


class Clock:
    value = 1_000_020_000_000
    def __call__(self):
        return self.value


def write_packet(root, arrays, packet, *, count=1, directory=None):
    directory = directory or packet['sample_id']
    folder = root / directory
    folder.mkdir(parents=True)
    np.savez(folder / 'images.npz', **arrays)
    (folder / 'packet.json').write_text(json.dumps(packet))
    pointer = {'schema': 'junqi.piper.head_stereo_latest.v1', 'packet_dir': directory,
               'sample_id': packet['sample_id'], 'owner_generation': packet['owner_generation'],
               'host_received_unix_ns': packet['host_received_unix_ns'],
               'exported_unix_ns': packet['host_received_unix_ns'] + 1_000_000,
               'packets_written': count}
    (root / 'latest.json').write_text(json.dumps(pointer))
    return folder, pointer


def make_worker(tmp_path):
    arrays, packet, calib, head, runner = setup()
    root = tmp_path / 'camera'
    warmup, _ = write_packet(root, arrays, packet)
    clock = Clock()
    worker = StereoLiveWorker(calib, head, runner, packet_root=root,
                              output=tmp_path / 'depth', clock_ns=clock, limit=3)
    worker.warmup(warmup)
    return worker, arrays, packet, runner, clock


def advance(packet, *, generation=None, delta=10_000_000):
    packet = copy.deepcopy(packet)
    packet['sample_id'] += '_next'
    packet['host_received_unix_ns'] += delta
    for frame in packet['frames'].values():
        frame['frame_number'] += 1
        frame['device_timestamp_ns'] += delta
    if generation:
        packet['owner_generation'] = generation
        for frame in packet['frames'].values():
            frame['frame_number'] = 0
            frame['device_timestamp_ns'] = 0
    return packet


def test_warmup_is_offline_only_and_never_relabelled_live(tmp_path):
    worker, _, _, runner, _ = make_worker(tmp_path)
    report = json.loads((worker.output / 'warmup-report.json').read_text())
    ready = json.loads((worker.output / 'ready.json').read_text())
    assert report['mode'] == 'offline_replay'
    assert report['source'] == 'synthetic_test_double'
    assert ready['live_results'] == 0
    assert not (worker.output / 'latest.json').exists()
    assert worker.step()['status'] == 'skipped'
    assert worker.step()['status'] == 'unchanged'
    assert runner.calls == 1


def test_latest_published_then_duplicate_or_rewrapped_cached_frame_not_inferred(tmp_path):
    worker, arrays, packet, runner, clock = make_worker(tmp_path)
    packet = advance(packet)
    write_packet(worker.packet_root, arrays, packet, count=2)
    assert worker.step()['status'] == 'published'
    assert worker.step()['status'] == 'unchanged'
    result, report = read_fresh_depth(worker.output, now_ns=clock.value)
    assert report['mode'] == 'live' and report['source'] == 'synthetic_test_double'
    assert result['depth_head_rgb_m'].shape == arrays['ir_left'].shape
    assert runner.calls == 2
    cached = copy.deepcopy(packet)
    cached['sample_id'] = 'rewrapped_cached_frame'
    cached['host_received_unix_ns'] += 1_000_000
    write_packet(worker.packet_root, arrays, cached, count=3)
    assert 'cached' in worker.step()['reason']
    assert runner.calls == 2
    # Existing published result eventually expires even though latest.json persists.
    clock.value += 2_000_000_000
    with pytest.raises(ValueError, match='stale'):
        read_fresh_depth(worker.output, now_ns=clock.value)


def test_restart_resets_frame_sequence_and_retires_old_generation(tmp_path):
    worker, arrays, packet, runner, clock = make_worker(tmp_path)
    first = advance(packet)
    write_packet(worker.packet_root, arrays, first, count=2)
    assert worker.step()['status'] == 'published'
    second = advance(first, generation='new_pipeline')
    clock.value += 10_000_000
    write_packet(worker.packet_root, arrays, second, count=1)
    assert worker.step()['status'] == 'published'
    assert worker.generation == 'new_pipeline'
    assert runner.calls == 3
    retired = advance(first, delta=20_000_000)
    retired['sample_id'] = 'from_old_pipeline'
    clock.value += 10_000_000
    write_packet(worker.packet_root, arrays, retired, count=3)
    assert 'retired' in worker.step()['reason']
    assert runner.calls == 3


def test_stale_packet_and_identity_mismatch_skip_without_retry(tmp_path):
    worker, arrays, packet, runner, clock = make_worker(tmp_path)
    packet = advance(packet)
    _, pointer = write_packet(worker.packet_root, arrays, packet, count=2)
    pointer['sample_id'] = 'wrong_pointer'
    (worker.packet_root / 'latest.json').write_text(json.dumps(pointer))
    assert 'identity mismatch' in worker.step()['reason']
    assert runner.calls == 1
    pointer['sample_id'] = packet['sample_id']
    clock.value += 2_000_000_000
    (worker.packet_root / 'latest.json').write_text(json.dumps(pointer))
    assert 'stale' in worker.step()['reason']
    assert worker.step()['status'] == 'unchanged'
    # A stale sample cannot be retried by editing the pointer export timestamp.
    pointer['exported_unix_ns'] += 1
    (worker.packet_root / 'latest.json').write_text(json.dumps(pointer))
    assert 'already attempted' in worker.step()['reason']
    assert runner.calls == 1


@pytest.mark.parametrize('directory', ['../outside', '/tmp/outside'])
def test_traversal_is_rejected(tmp_path, directory):
    worker, arrays, packet, runner, clock = make_worker(tmp_path)
    packet = advance(packet)
    _, pointer = write_packet(worker.packet_root, arrays, packet, count=2)
    pointer['packet_dir'] = directory
    (worker.packet_root / 'latest.json').write_text(json.dumps(pointer))
    assert 'contained relative' in worker.step()['reason']
    assert runner.calls == 1


def test_symlink_escape_is_rejected(tmp_path):
    worker, arrays, packet, runner, clock = make_worker(tmp_path)
    packet = advance(packet)
    _, pointer = write_packet(worker.packet_root, arrays, packet, count=2)
    external = tmp_path / 'outside'
    external.mkdir()
    (worker.packet_root / 'escape').symlink_to(external)
    pointer['packet_dir'] = 'escape'
    (worker.packet_root / 'latest.json').write_text(json.dumps(pointer))
    assert 'escapes root' in worker.step()['reason']
    assert runner.calls == 1


def test_expiration_during_model_or_output_write_does_not_publish(tmp_path, monkeypatch):
    worker, arrays, packet, runner, clock = make_worker(tmp_path)
    packet = advance(packet)
    write_packet(worker.packet_root, arrays, packet, count=2)
    original = np.savez
    def slow_save(*args, **kwargs):
        original(*args, **kwargs)
        clock.value += 2_000_000_000
    monkeypatch.setattr(np, 'savez', slow_save)
    assert 'stale' in worker.step()['reason']
    assert runner.calls == 2
    assert not (worker.output / 'latest.json').exists()
    assert not list(worker.output.glob('.partial-*'))
    assert not list(worker.output.glob('result-*'))
    assert worker.step()['status'] == 'unchanged'


def test_consumer_rejects_pointer_report_mismatch(tmp_path):
    worker, arrays, packet, _, clock = make_worker(tmp_path)
    packet = advance(packet)
    write_packet(worker.packet_root, arrays, packet, count=2)
    assert worker.step()['status'] == 'published'
    pointer = json.loads((worker.output / 'latest.json').read_text())
    pointer['sample_id'] = 'wrong'
    (worker.output / 'latest.json').write_text(json.dumps(pointer))
    with pytest.raises(ValueError, match='identity mismatch'):
        read_fresh_depth(worker.output, now_ns=clock.value)


def test_saved_npz_object_arrays_are_not_unpickled(tmp_path):
    worker, arrays, packet, _, _ = make_worker(tmp_path)
    packet = advance(packet)
    folder, _ = write_packet(worker.packet_root, arrays, packet, count=2)
    np.savez(folder / 'images.npz', ir_left=np.array([object()], dtype=object),
             ir_right=arrays['ir_right'], rgb=arrays['rgb'])
    assert 'allow_pickle=False' in worker.step()['reason']


def test_duration_and_result_limits_are_bounded(tmp_path):
    worker, arrays, packet, _, _ = make_worker(tmp_path)
    times = iter([0, 0, 0.1, 0.2, 0.3, 0.4])
    report = worker.run(duration=0.3, monotonic=lambda: next(times), sleep=lambda _: None)
    assert report['results_written'] == 0 and report['attempts'] == 1
    assert report['hardware_access'] is False
    worker.results = worker.limit
    assert worker.step()['status'] == 'limit_reached'
