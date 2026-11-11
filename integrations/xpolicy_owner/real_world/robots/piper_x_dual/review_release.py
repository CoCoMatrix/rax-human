"""Bounded review release and passive receipts for the sole PiPER owner.

This module has no SDK, device or motion methods. A dispatch receipt means the
existing SDK calls returned; only later observations can establish arrival.
"""
from pathlib import Path
import hashlib
import json
import math
import os
import re
import time

import numpy as np

JOINT_COLUMNS = (0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12)
GRIPPER_COLUMNS = (6, 13)
SCHEMA = 'junqi.piper.bounded_review_release.v1'
TRAJECTORY_SCHEMA = 'junqi.piper.trajectory_release.v1'
POLICY_PREFIX_SCHEMA = 'junqi.piper.policy_prefix_release.v1'
REVIEW_PROFILES = ('legacy', 'policy_prefix')

def student_prefix_max():
    value = int(os.environ.get('PIPER_POLICY_PREFIX_STUDENT_MAX', '15'))
    _require(1 <= value <= 32, 'student prefix max must be 1..32')
    return value


def _require(ok, message):
    if not ok:
        raise ValueError(message)


def vector(value):
    a = np.asarray(value)
    _require(a.shape == (14,) and a.dtype.kind in 'fi' and np.isfinite(a).all(),
             'review release requires finite state/action14')
    return a.astype(np.float64)


def target_hash(value):
    return hashlib.sha256(np.asarray(vector(value), dtype='<f4').tobytes()).hexdigest()


def release_step_limit(release, *, profile='legacy', allow_trajectory=False):
    """The process configuration, never incoming metadata, selects the contract."""
    _require(profile in REVIEW_PROFILES, 'unknown runtime review profile')
    _require(isinstance(release, dict), 'explicit bounded review release required')
    if profile == 'policy_prefix':
        _require(not allow_trajectory, 'policy_prefix cannot enable GPT trajectory mode')
        _require(release.get('schema') == POLICY_PREFIX_SCHEMA,
                 'explicit policy_prefix review release required')
        kind = release.get('kind')
        _require(kind in ('student', 'edit', 'eef'), 'invalid policy_prefix kind')
        return student_prefix_max() if kind == 'student' else 5
    _require(release.get('schema') == SCHEMA
             or (allow_trajectory and release.get('schema') == TRAJECTORY_SCHEMA),
             'explicit bounded review release required')
    return 600 if release.get('schema') == TRAJECTORY_SCHEMA else 16


def validate_review_runtime(*, profile, bounded_review, continuous_review,
                            trajectory_review, allow_no_action, prefetch_enabled,
                            execute_chunk_size, execute_chunk_mode,
                            action_frequency_hz):
    _require(profile in REVIEW_PROFILES, 'unknown runtime review profile')
    if profile == 'policy_prefix':
        _require(bounded_review and continuous_review and not trajectory_review,
                 'policy_prefix requires bounded continuous review without GPT trajectory mode')
        _require(not allow_no_action and not prefetch_enabled,
                 'policy_prefix uses review_wait with prefetch disabled')
        _require(execute_chunk_mode == 'auto' and execute_chunk_size >= student_prefix_max()
                 and action_frequency_hz == 30,
                 'policy_prefix requires 30 Hz auto scheduling with capacity at least 15')
        return
    # Preserve the pre-existing canary and direct-GPT trajectory configurations.
    if trajectory_review and (not bounded_review or not continuous_review or execute_chunk_size != 600):
        raise ValueError('GPT trajectory release requires bounded continuous review with 600-step maximum')
    if continuous_review and (not bounded_review or allow_no_action or execute_chunk_mode != 'auto' or execute_chunk_size != (600 if trajectory_review else 15)):
        raise ValueError('continuous review requires gated 15-step auto prefixes')
    if bounded_review and (allow_no_action or prefetch_enabled or (execute_chunk_mode != 'fixed' and not continuous_review) or not 1 <= execute_chunk_size <= (600 if trajectory_review else 16)):
        raise ValueError('bounded review release requires actions-only fixed 1..16 steps with prefetch off')


class BoundedReviewGate:
    def __init__(self, *, allow_trajectory=False, profile='legacy'):
        _require(profile in REVIEW_PROFILES, 'unknown runtime review profile')
        _require(not (profile == 'policy_prefix' and allow_trajectory),
                 'policy_prefix cannot enable GPT trajectory mode')
        self.used = {}
        self.allow_trajectory = bool(allow_trajectory)
        self.profile = profile

    def reserve(self, release, *, generation, measured, requested, now_ns, step_index=0):
        maximum = release_step_limit(release, profile=self.profile,
                                     allow_trajectory=self.allow_trajectory)
        policy_prefix = self.profile == 'policy_prefix'
        trajectory = isinstance(release, dict) and release.get('schema') == TRAJECTORY_SCHEMA
        for name in ('release_id', 'teacher_request_id', 'execution_session_id'):
            _require(isinstance(release.get(name), str)
                     and re.fullmatch(r'[A-Za-z0-9_-]{1,128}', release[name]), 'invalid release identity')
        for name in ('generation', 'review_observation_timestamp_ns',
                     'execution_observation_timestamp_ns', 'expires_ns'):
            _require(type(release.get(name)) is int and release[name] >= 0, 'invalid release integer')
        original = release['review_observation_timestamp_ns']
        current = release['execution_observation_timestamp_ns']
        _require(0 < original <= current <= now_ns < release['expires_ns'],
                 'expired/future review release')
        steps = release.get('steps')
        _require(type(steps) is int and 1 <= steps <= maximum, f'release requires 1..{maximum} targets')
        _require(type(step_index) is int and 0 <= step_index < steps, 'invalid release step index')
        budget = 300_000_000 + int((steps - 1) * 1e9 / 30)
        _require(release['expires_ns'] <= current + budget
                 and (step_index > 0 or now_ns - current <= 300_000_000),
                 'review release exceeds fresh execution observation budget')
        _require(generation == release['generation'], 'review release generation mismatch')
        _require(release.get('scene_check') in ('three_camera_photometric_and_state_match',
                 'fresh_three_cameras_and_state_match_image_comparison_disabled_by_operator'),
                 'review release lacks declared scene check')
        _require(len(self.used) < 256, 'review release session limit reached')
        signature = hashlib.sha256(json.dumps(release, sort_keys=True, allow_nan=False).encode()).hexdigest()
        previous = self.used.get(release['release_id'])
        _require((previous is None and step_index == 0)
                 or (previous is not None and previous == (signature, step_index - 1)),
                 'review release consumed, reordered or changed')
        anchor, measured, requested = vector(release.get('anchor14')), vector(measured), vector(requested)
        targets = np.asarray(release.get('targets14'))
        _require(targets.shape == (steps, 14) and targets.dtype.kind in 'fi' and np.isfinite(targets).all(),
                 'release target prefix malformed')
        if policy_prefix:
            # A reviewed policy prefix uses the model's ordinary absolute
            # targets. The owner still applies its real joint/gripper limits,
            # freshness and generation guard before reaching this receipt gate.
            # Only the first sample is anchored to the execution observation;
            # later samples must be allowed to leave that initial pose.
            if step_index == 0:
                _require(np.max(np.abs(anchor - measured)) <= .001,
                         'policy_prefix execution anchor drift')
        elif trajectory:
            deltas = np.diff(np.vstack([anchor, targets]), axis=0)
            _require(np.max(np.abs(deltas[:, JOINT_COLUMNS])) <= .015,
                     'trajectory joint increment exceeds 0.015 rad')
            _require(np.max(np.abs(deltas[:, GRIPPER_COLUMNS])) <= .002,
                     'trajectory gripper increment exceeds 0.002 m')
            reference = anchor if step_index == 0 else targets[step_index-1]
            # RAX-only operator policy; other owner users retain their exact defaults.
            tolerance=float(os.environ.get('RAX_JOINT_STATE_TOLERANCE_RAD','.001')) if os.environ.get('RAX_MOTION_LEASE') else .001
            _require(tolerance in (.001,.006,.01), 'invalid RAX joint-state tolerance')
            delta=np.abs(reference-measured)
            _require(np.max(delta[list(JOINT_COLUMNS)]) <= (tolerance if step_index == 0 else .03)
                     and np.max(delta[list(GRIPPER_COLUMNS)]) <= (.001 if step_index == 0 else .03),
                     'trajectory measured tracking drift')
        else:
            _require(np.max(np.abs(anchor - measured)) <= (.001 if step_index == 0 else .10),
                     'review release anchor drift')
        _require(target_hash(requested) == target_hash(targets[step_index]),
                 'review release target identity mismatch')
        if not policy_prefix:
            _require(np.max(np.abs(requested[list(JOINT_COLUMNS)] - measured[list(JOINT_COLUMNS)])) <= .03,
                     'review release joint step exceeds 0.03 rad')
            _require(np.max(np.abs(requested[list(GRIPPER_COLUMNS)] - measured[list(GRIPPER_COLUMNS)])) <= .02,
                     'review release gripper step exceeds 0.02 m')
        # Consume before any SDK call. A partially dispatched target must not retry.
        self.used[release['release_id']] = (signature, step_index)
        return json.loads(json.dumps(release, allow_nan=False))


class ReceiptJournal:
    def __init__(self, path, *, volatile_root=None):
        root = Path('/home/ubuntu/code_junqi/migration').resolve()
        path = Path(path).resolve()
        allowed = path.is_relative_to(root) and path.parent.is_dir()
        if volatile_root is not None:
            import re, stat
            from .volatile_control import require_tmpfs
            ram = Path(volatile_root).resolve()
            require_tmpfs(ram)
            _require(ram.parent == Path('/run/user') / str(os.getuid())
                     and re.fullmatch(r'rax-motion-[0-9a-f]{24}', ram.name) is not None
                     and stat.S_IMODE(ram.stat().st_mode) == 0o700, 'invalid private RAM control root')
            allowed = path == ram / 'receipts.jsonl'
        _require(allowed, 'receipt journal outside authorized evidence/control roots')
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        self.stream = os.fdopen(fd, 'w', buffering=1)
        self.write({'event': 'journal_opened', 'hardware_dispatch_confirmed': False})

    def write(self, value):
        self.stream.write(json.dumps({'recorded_ns': time.time_ns(), **value}, allow_nan=False) + '\n')
        self.stream.flush()

    def close(self):
        self.stream.close()
