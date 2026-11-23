"""Optional discrete acceleration matching with the original velocity fallback.

No optimizer, new hard dynamic limits, retiming or new HOLD path. Matching
applies per joint only when the local quintic passes the original envelope
and does not increase peak acceleration over the velocity-seam result.
"""
import json
import time

import numpy as np

from XPolicyLab.real_world.velocity_seam import VelocitySeam


class AccelerationBlendSeam(VelocitySeam):
    anchor_count = 3
    algorithm = 'discrete_acceleration_blend_v1'

    def __init__(self, profile, schema, frequency_hz):
        super().__init__(profile, schema, frequency_hz)
        if self.config.get('acceleration_blend') is not True or self.n < 4:
            raise ValueError('Acceleration blend requires explicit opt-in and at least four bridge steps')
        self.dim = len(schema.units)
        # Six distinct point constraints produce a quintic in sample time.
        # q1 and q(N-1) are chosen to continue backward second differences
        # at entry and exit; the raw qN, q(N+1), q(N+2) remain untouched.
        nodes = np.array([-1., 0., 1., self.n-1., self.n, self.n+1.])
        t = np.arange(1., self.n)
        self.acceleration_weights = np.ones((self.n-1, len(nodes)))
        for j in range(len(nodes)):
            for k in range(len(nodes)):
                if j != k:
                    self.acceleration_weights[:, j] *= (t-nodes[k])/(nodes[j]-nodes[k])

    def process(self, suffix, anchors):
        started = time.perf_counter()
        raw, history = np.asarray(suffix), np.asarray(anchors)
        if (raw.ndim != 2 or raw.shape[1] != self.dim or len(raw) < self.n+2
                or raw.dtype not in (np.dtype('float32'), np.dtype('float64'))
                or history.shape != (3, self.dim) or not np.isfinite(history).all()
                or not np.isfinite(raw).all()):
            raise ValueError('Acceleration blend requires finite targets and three committed anchors')
        # Compute the unchanged, already-tested velocity behavior first.
        output, report = super().process(raw, history[-2:])
        idx, n = self.indices, self.n
        h, q = history[:, idx].astype(float), raw[:, idx].astype(float)
        entry = 3*h[-1]-3*h[-2]+h[-3]
        exit_point = 3*q[n-1]-3*q[n]+q[n+1]
        boundary = np.stack([h[-2], h[-1], entry, exit_point, q[n-1], q[n]])
        candidate = (self.acceleration_weights @ boundary).astype(raw.dtype).astype(float)
        joined = np.vstack([h, candidate, q[n-1:n+2]])
        previous = np.vstack([h, output[:n+2, idx].astype(float)])
        original = np.vstack([h, q[:n+2]])
        correction = np.max(np.abs(candidate-q[:n-1]), axis=0)
        step = np.max(np.abs(np.diff(joined, axis=0)), axis=0)
        acceleration = np.max(np.abs(np.diff(joined, n=2, axis=0)), axis=0)
        old_acceleration = np.max(np.abs(np.diff(previous, n=2, axis=0)), axis=0)
        raw_acceleration = np.max(np.abs(np.diff(original, n=2, axis=0)), axis=0)
        checks = {
            'correction': correction <= self.config['max_correction_rad'],
            'step': step <= self.config['max_bridge_step_rad'],
            'peak_acceleration': acceleration <= np.minimum(old_acceleration, raw_acceleration)+1e-6,
            # As in velocity v1, only modified targets are admitted here.
            # Unchanged tail targets retain the existing owner processing.
            'position': np.all((candidate >= self.lower) & (candidate <= self.upper), axis=0),
        }
        valid = np.logical_and.reduce(list(checks.values()))
        output[:n-1, idx[valid]] = candidate[:, valid]
        fallback = {str(int(idx[j])): [name for name, passed in checks.items() if not passed[j]]
                    for j in np.flatnonzero(~valid)}
        old_bypassed = set(report['trajectory_bypassed_joint_indices'])
        raw_bypassed = [int(i) for i, ok in zip(idx, valid) if not ok and int(i) in old_bypassed]
        velocity_fallback = [int(i) for i, ok in zip(idx, valid) if not ok and int(i) not in old_bypassed]
        actual = np.vstack([h, output[:n+2, idx].astype(float)])
        acceleration_actual = np.diff(actual, n=2, axis=0)*self.hz**2
        report.update({
            'trajectory_algorithm': self.algorithm,
            'trajectory_acceleration_matched_joint_indices': idx[valid].tolist(),
            'trajectory_acceleration_matched_joint_count': int(valid.sum()),
            'trajectory_velocity_fallback_joint_indices': velocity_fallback,
            'trajectory_bypassed_joint_indices': raw_bypassed,
            'trajectory_bridged_joint_count': len(idx)-len(raw_bypassed),
            'trajectory_acceleration_fallback_reasons_json': json.dumps(fallback, separators=(',', ':')),
            'trajectory_peak_acceleration_rad_s2': float(np.max(np.abs(acceleration_actual))),
            'trajectory_raw_peak_acceleration_rad_s2': float(np.max(raw_acceleration)*self.hz**2),
            'trajectory_entry_acceleration_change_rad_s2': float(np.max(np.abs(acceleration_actual[1]-acceleration_actual[0]))),
            'trajectory_matched_entry_acceleration_change_rad_s2': float(np.max(np.abs(
                acceleration_actual[1, valid]-acceleration_actual[0, valid]), initial=0)),
            'trajectory_matched_exit_acceleration_change_rad_s2': float(np.max(np.abs(
                acceleration_actual[-1, valid]-acceleration_actual[-2, valid]), initial=0)),
            'trajectory_seam_velocity_jump_rad_s': float(np.max(np.abs((output[0, idx]-h[-1]-(h[-1]-h[-2]))*self.hz))),
            'trajectory_max_correction_rad': float(np.max(np.abs(output-raw))),
            'trajectory_prepare_ms': (time.perf_counter()-started)*1000,
        })
        return output, report
