"""Bounded head-depth sidecar consuming the existing camera owner's saved packets.

No camera, CAN or action API is imported. Warmup is explicitly historical;
ready.json is written only after it succeeds. Published depth retains the camera
receipt time and expires even if latest.json remains on disk after this exits.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid

import numpy as np

from .contracts import require
from .stereo_pipeline import StereoDepthProcessor

PACKET_LATEST_SCHEMA = 'junqi.piper.head_stereo_latest.v1'
DEPTH_LATEST_SCHEMA = 'junqi.piper.head_depth_latest.v1'


def _read_json(path):
    value = json.loads(Path(path).read_text())
    require(isinstance(value, dict), 'expected JSON object')
    return value


def _atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _contained_directory(root, relative):
    require(isinstance(relative, str) and relative, 'missing relative packet/result directory')
    path = Path(relative)
    require(not path.is_absolute() and '..' not in path.parts and path.parts,
            'packet/result directory must be a contained relative path')
    root = Path(root).resolve()
    resolved = (root / path).resolve(strict=True)
    require(resolved != root and resolved.is_relative_to(root) and resolved.is_dir(),
            'packet/result directory escapes root or is not a directory')
    return resolved


def _load_packet(directory):
    directory = Path(directory).resolve(strict=True)
    for name in ('packet.json', 'images.npz'):
        require((directory / name).resolve(strict=True).is_relative_to(directory),
                'packet file escapes immutable packet directory')
    metadata = _read_json(directory / 'packet.json')
    with np.load(directory / 'images.npz', allow_pickle=False) as archive:
        require(set(archive.files) == {'ir_left', 'ir_right', 'rgb'}, 'unexpected packet arrays')
        arrays = {key: archive[key] for key in archive.files}
    return arrays, metadata


def _age_ms(received, now, max_age_ms):
    require(type(received) is int and type(now) is int and 0 < received <= now,
            'invalid/future host receipt timestamp')
    age = (now - received) / 1e6
    require(age <= max_age_ms, 'depth packet is stale')
    return age


def read_fresh_depth(output, *, now_ns=None):
    """Consumer helper: never interpret an old latest pointer as current depth.

    Checks expiration before AND after disk IO; consumers must recheck the
    returned expires_unix_ns at their actual use time, on the same host clock.
    """
    output = Path(output)
    pointer = _read_json(output / 'latest.json')
    require(pointer.get('schema') == DEPTH_LATEST_SCHEMA, 'unexpected depth pointer schema')
    max_age = pointer.get('max_age_ms')
    require(type(max_age) in (int, float) and math.isfinite(max_age) and max_age > 0,
            'invalid depth age limit')
    received = pointer.get('host_received_unix_ns')
    require(type(received) is int and pointer.get('expires_unix_ns') == received + int(max_age * 1e6),
            'depth expiry must derive from original camera receipt')
    clock = time.time_ns if now_ns is None else lambda: now_ns
    _age_ms(received, clock(), max_age)
    directory = _contained_directory(output, pointer.get('result_dir'))
    for name in ('report.json', 'depth.npz'):
        require((directory / name).resolve(strict=True).is_relative_to(directory), 'depth file escapes result directory')
    report = _read_json(directory / 'report.json')
    require(report.get('mode') == 'live' and all(report.get(key) == pointer.get(key) for key in
            ('sample_id', 'owner_generation', 'host_received_unix_ns', 'published_unix_ns', 'expires_unix_ns')),
            'depth pointer/report identity mismatch')
    with np.load(directory / 'depth.npz', allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files}
    _age_ms(received, clock(), max_age)
    return arrays, report


class StereoLiveWorker:
    def __init__(self, calibration, nominal_head, runner, *, packet_root, output,
                 limit=10, max_age_ms=1000.0, clock_ns=time.time_ns):
        require(type(limit) is int and 1 <= limit <= 120, 'result limit must be in [1,120]')
        require(type(max_age_ms) in (int, float) and math.isfinite(max_age_ms) and max_age_ms > 0,
                'invalid live age limit')
        self.packet_root, self.output = Path(packet_root).resolve(), Path(output).resolve()
        require(not self.output.exists(), 'live output directory already exists')
        require(not self.output.is_relative_to(self.packet_root), 'output cannot be inside camera packet root')
        self.output.mkdir(parents=True)
        self.calibration, self.nominal_head, self.runner = calibration, nominal_head, runner
        self.limit, self.max_age_ms, self.clock_ns = limit, float(max_age_ms), clock_ns
        self.ready = False
        self.processor = None
        self.generation = None
        self.retired_generations = set()
        self.seen_samples = set()
        self.last_pointer_text = None
        self.last_sequence = -1
        self.last_receive_ns = -1
        self.results = 0
        self.attempts = 0
        self.errors = 0

    def _processor(self, generation=None):
        return StereoDepthProcessor(self.calibration, self.nominal_head, self.runner,
                                    owner_generation=generation, max_age_ms=self.max_age_ms)

    def warmup(self, directory):
        require(not self.ready, 'warmup already completed')
        arrays, packet = _load_packet(directory)
        _, report = self._processor().process(arrays, packet, mode='offline_replay')
        report['purpose'] = 'historical_model_warmup_not_live_output'
        _atomic_json(self.output / 'warmup-report.json', report)
        marker = {'schema': 'junqi.piper.head_depth_ready.v1', 'ready_unix_ns': self.clock_ns(),
                  'warmup_sample_id': packet['sample_id'], 'warmup_mode': 'offline_replay',
                  'live_results': 0, 'hardware_access': False, 'robot_action_emitted': False}
        _atomic_json(self.output / 'ready.json', marker)
        self.seen_samples.add((packet['owner_generation'], packet['sample_id']))
        self.ready = True
        return marker

    def _record(self, status, **fields):
        event = {'status': status, 'recorded_unix_ns': self.clock_ns(), **fields}
        with (self.output / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(event, allow_nan=False) + '\n')
        return event

    def step(self):
        """Try only the newest pointer. A failed sample is never retried."""
        require(self.ready, 'offline warmup must complete before live processing')
        if self.results >= self.limit:
            return {'status': 'limit_reached'}
        # This hard bound limits even a malformed producer's log/identity memory.
        if self.attempts >= 10000:
            return {'status': 'attempt_limit_reached'}
        try:
            raw = (self.packet_root / 'latest.json').read_text()
        except FileNotFoundError:
            return {'status': 'waiting_for_camera'}
        if raw == self.last_pointer_text:
            return {'status': 'unchanged'}
        self.last_pointer_text = raw
        self.attempts += 1
        sample_id = None
        try:
            pointer = json.loads(raw)
            require(isinstance(pointer, dict) and pointer.get('schema') == PACKET_LATEST_SCHEMA,
                    'unexpected camera pointer schema')
            sample_id, generation = pointer.get('sample_id'), pointer.get('owner_generation')
            require(isinstance(sample_id, str) and sample_id and isinstance(generation, str) and generation,
                    'camera pointer identity is missing')
            identity = (generation, sample_id)
            require(identity not in self.seen_samples, 'sample was already attempted')
            self.seen_samples.add(identity)
            received, exported, sequence = (pointer.get(name) for name in
                ('host_received_unix_ns', 'exported_unix_ns', 'packets_written'))
            require(type(sequence) is int and sequence > 0 and type(exported) is int
                    and type(received) is int and 0 < received <= exported <= self.clock_ns(),
                    'invalid camera pointer sequence/timestamps')
            require(generation not in self.retired_generations, 'retired camera generation cannot return')
            require(received > self.last_receive_ns, 'reordered camera receipt')
            if generation == self.generation:
                require(sequence > self.last_sequence, 'reordered camera export sequence')
            _age_ms(received, self.clock_ns(), self.max_age_ms)
            directory = _contained_directory(self.packet_root, pointer.get('packet_dir'))
            arrays, packet = _load_packet(directory)
            require(all(packet.get(key) == pointer[key] for key in
                    ('sample_id', 'owner_generation', 'host_received_unix_ns')),
                    'camera pointer/packet identity mismatch')
            if generation != self.generation:
                if self.generation is not None:
                    self.retired_generations.add(self.generation)
                self.generation = generation
                self.processor = self._processor(generation)
                self.last_sequence = -1
            self.last_sequence, self.last_receive_ns = sequence, received
            result, report = self.processor.process(arrays, packet, mode='live', clock_ns=self.clock_ns)
            # Separate staging keeps failed/stale outputs from appearing as complete results.
            staging = self.output / ('.partial-' + uuid.uuid4().hex)
            staging.mkdir()
            try:
                # Uncompressed NPZ avoids spending the freshness budget on compression.
                np.savez(staging / 'depth.npz', **result)
                published = self.clock_ns()
                _age_ms(received, published, self.max_age_ms)
                report.update(published_unix_ns=published,
                              expires_unix_ns=received + int(self.max_age_ms * 1e6),
                              max_age_ms=self.max_age_ms,
                              consumer_requirement='recheck expires_unix_ns against same-host clock at use time')
                _atomic_json(staging / 'report.json', report)
                # Check once more after report IO, before publishing the directory pointer.
                _age_ms(received, self.clock_ns(), self.max_age_ms)
                relative = 'result-' + str(self.results + 1).zfill(4) + '-' + uuid.uuid4().hex
                destination = self.output / relative
                staging.rename(destination)
                latest = {'schema': DEPTH_LATEST_SCHEMA, 'result_dir': relative,
                          'sample_id': sample_id, 'owner_generation': generation,
                          'host_received_unix_ns': received, 'published_unix_ns': published,
                          'expires_unix_ns': report['expires_unix_ns'], 'max_age_ms': self.max_age_ms,
                          'results_written': self.results + 1, 'robot_action_emitted': False}
                _atomic_json(self.output / 'latest.json', latest)
                self.results += 1
                return self._record('published', sample_id=sample_id, result_dir=relative,
                                    age_ms=(published - received) / 1e6)
            finally:
                if staging.exists():
                    for item in staging.iterdir():
                        item.unlink()
                    staging.rmdir()
        except (ValueError, KeyError, TypeError, OSError, RuntimeError) as error:
            self.errors += 1
            return self._record('skipped', sample_id=sample_id,
                                error_type=type(error).__name__, reason=str(error))

    def run(self, *, duration=60.0, poll_interval=0.02, monotonic=time.monotonic, sleep=time.sleep):
        require(type(duration) in (float, int) and math.isfinite(duration) and 0 < duration <= 600,
                'duration must be in (0,600] seconds')
        require(type(poll_interval) in (float, int) and math.isfinite(poll_interval)
                and 0.001 <= poll_interval <= 1, 'poll interval must be in [0.001,1] seconds')
        start = monotonic()
        while monotonic() - start < duration:
            event = self.step()
            if event['status'] in ('limit_reached', 'attempt_limit_reached'):
                break
            sleep(poll_interval)
        summary = {'schema': 'junqi.piper.head_depth_worker_run.v1', 'duration_s': monotonic() - start,
                   'results_written': self.results, 'attempts': self.attempts, 'skipped': self.errors,
                   'limit': self.limit, 'max_age_ms': self.max_age_ms, 'hardware_access': False,
                   'robot_action_emitted': False, 'pi05_rgb_input_modified': False,
                   'consumer_requirement': 'latest.json expires at original camera receipt plus max_age_ms'}
        _atomic_json(self.output / 'run-report.json', summary)
        return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    from .camera_geometry import ACTIVE_HEAD
    parser.add_argument('--nominal-head', type=Path, default=ACTIVE_HEAD)
    for name in ('calibration', 'model-manifest', 'packet-root', 'warmup-packet', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--limit', type=int, default=10)
    parser.add_argument('--max-age-ms', type=float, default=1000.0)
    parser.add_argument('--trust-official-pickle', action='store_true')
    parser.add_argument('--valid-iters', type=int, default=4)
    parser.add_argument('--max-disp', type=int, default=192)
    parser.add_argument('--input-scale', type=float, default=1.0)
    args = parser.parse_args()
    from .ffs_runtime import FastFoundationStereoRunner
    runner = FastFoundationStereoRunner(args.model_manifest, valid_iters=args.valid_iters,
        max_disp=args.max_disp, input_scale=args.input_scale, allow_trusted_pickle=args.trust_official_pickle)
    worker = StereoLiveWorker(_read_json(args.calibration), _read_json(args.nominal_head), runner,
                              packet_root=args.packet_root, output=args.output, limit=args.limit,
                              max_age_ms=args.max_age_ms)
    worker.warmup(args.warmup_packet)
    print(json.dumps({'ready': str(args.output / 'ready.json')}), flush=True)
    print(json.dumps(worker.run(duration=args.duration)), flush=True)


if __name__ == '__main__':
    main()
