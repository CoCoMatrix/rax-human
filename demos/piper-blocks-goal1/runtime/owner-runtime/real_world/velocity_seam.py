"""Local discrete velocity-continuous bridge; no retiming or gripper edits."""
import json
import time
from pathlib import Path

import numpy as np


class VelocitySeam:
    def __init__(self, profile, schema, frequency_hz):
        self.config = json.loads(Path(profile).read_text())
        cfg = self.config
        if (cfg['schema'] != 'xpolicy.velocity-seam/v1' or cfg['action_schema'] != schema.to_wire()
                or cfg['frequency_hz'] != frequency_hz):
            raise ValueError('Velocity seam schema/cadence mismatch')
        self.indices = np.array(cfg['continuous_indices'], dtype=int)
        expected = [i for i, s in enumerate(schema.semantics)
                    if s == 'absolute_joint' and schema.units[i] == 'rad']
        if self.indices.tolist() != expected:
            raise ValueError('Velocity seam must select exactly the absolute joint fields')
        self.n = cfg['bridge_steps']
        if not isinstance(self.n, int) or not 3 <= self.n <= 9:
            raise ValueError('Velocity seam bridge must fit inside one commit interval')
        self.lower = np.asarray(cfg['lower_rad'], dtype=float)
        self.upper = np.asarray(cfg['upper_rad'], dtype=float)
        if (self.lower.shape != self.indices.shape or self.upper.shape != self.indices.shape
                or not np.all(np.isfinite([self.lower, self.upper]))
                or not np.all(self.lower < self.upper)):
            raise ValueError('Invalid velocity seam position envelope')
        for key in ('max_correction_rad', 'max_bridge_step_rad'):
            if not np.isfinite(cfg[key]) or cfg[key] <= 0:
                raise ValueError('Invalid velocity seam limit')
        # Lagrange interpolation at four discrete boundary samples:
        # t=0: last committed target; t=1: preserve its last increment;
        # t=N,N+1: two unchanged model targets (unchanged exit increment).
        nodes = np.array([0., 1., self.n, self.n + 1.])
        t = np.arange(1., self.n)[:, None]
        weights = np.ones((self.n - 1, 4))
        for j in range(4):
            for k in range(4):
                if j != k:
                    weights[:, j] *= (t[:, 0] - nodes[k]) / (nodes[j] - nodes[k])
        self.weights = weights
        self.hz = frequency_hz

    def process(self, suffix, anchors):
        begin = time.perf_counter()
        raw = np.asarray(suffix)
        history = np.asarray(anchors)
        if (raw.ndim != 2 or len(raw) <= self.n or history.shape != (2, raw.shape[1])
                or not np.all(np.isfinite(raw)) or not np.all(np.isfinite(history))):
            raise ValueError('Invalid velocity seam samples')
        idx = self.indices
        prior = history[:, idx].astype(np.float64)
        joint_raw = raw[:, idx].astype(np.float64)
        increment = prior[-1] - prior[-2]
        boundary = np.stack([prior[-1], prior[-1] + increment,
                             joint_raw[self.n - 1], joint_raw[self.n]])
        candidate = self.weights @ boundary
        # Validate what will actually be emitted, including float32 conversion.
        candidate = candidate.astype(raw.dtype).astype(np.float64)
        correction = np.max(np.abs(candidate - joint_raw[:self.n - 1]), axis=0)
        stitched = np.vstack([prior, candidate, joint_raw[self.n - 1:self.n + 1]])
        step = np.max(np.abs(np.diff(stitched, axis=0)), axis=0)
        original = np.vstack([prior, joint_raw[:self.n + 1]])
        acceleration = np.max(np.abs(np.diff(stitched, n=2, axis=0)), axis=0)
        raw_acceleration = np.max(np.abs(np.diff(original, n=2, axis=0)), axis=0)
        valid = ((correction <= self.config['max_correction_rad'])
                 & (step <= self.config['max_bridge_step_rad'])
                 & (acceleration <= raw_acceleration + 1e-6)
                 & np.all(candidate >= self.lower, axis=0)
                 & np.all(candidate <= self.upper, axis=0))
        # A local fit outside the envelope bypasses that joint unchanged. This
        # is not a new safety policy: the existing owner still judges the raw
        # or bridged command. Never clamp a fit and silently break continuity.
        output = raw.copy()
        output[:self.n - 1, idx[valid]] = candidate[:, valid]
        seam_delta = (output[0, idx].astype(float) - prior[-1] - increment) * self.hz
        report = {
            'trajectory_algorithm': 'discrete_velocity_seam_v1',
            'trajectory_bridge_steps': self.n,
            'trajectory_bridged_joint_count': int(valid.sum()),
            'trajectory_bypassed_joint_indices': idx[~valid].tolist(),
            'trajectory_max_correction_rad': float(np.max(np.abs(output - raw))),
            'trajectory_raw_peak_acceleration_rad_s2': float(np.max(raw_acceleration) * self.hz**2),
            'trajectory_peak_acceleration_rad_s2': float(np.max(np.where(
                valid, acceleration, raw_acceleration)) * self.hz**2),
            'trajectory_raw_seam_velocity_jump_rad_s': float(np.max(np.abs(
                (joint_raw[0] - prior[-1] - increment) * self.hz))),
            'trajectory_seam_velocity_jump_rad_s': float(np.max(np.abs(seam_delta))),
            'trajectory_prepare_ms': (time.perf_counter() - begin) * 1000,
        }
        return output, report
