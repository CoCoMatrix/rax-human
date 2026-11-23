"""One Euro equations with a per-sample correction cap and explicit plan state.

Reference: https://github.com/casiez/OneEuroFilter (Casiez, Roussel, Vogel).
State is [filtered position, filtered derivative], independently for each joint.
The worker is stateless: only the state at the committed boundary is continued,
never the state at the end of an unexecuted prediction. No SDK or retiming.
"""
import json
import time
from pathlib import Path

import numpy as np


class BoundedOneEuro:
    def __init__(self, profile, schema, frequency_hz):
        self.config = json.loads(Path(profile).read_text())
        cfg = self.config
        if (cfg['schema'] != 'xpolicy.bounded-one-euro/v1'
                or cfg['action_schema'] != schema.to_wire()
                or cfg['frequency_hz'] != frequency_hz):
            raise ValueError('One Euro schema/cadence mismatch')
        self.schema = schema
        self.hz = float(frequency_hz)
        self.indices = np.asarray(cfg['continuous_indices'], dtype=int)
        expected = [i for i in range(schema.dim)
                    if schema.units[i] == 'rad' and schema.semantics[i] == 'absolute_joint']
        if self.indices.tolist() != expected:
            raise ValueError('One Euro must select exactly the radian arm joints')
        self.lower = np.asarray(cfg['lower_rad'], dtype=float)
        self.upper = np.asarray(cfg['upper_rad'], dtype=float)
        if (self.lower.shape != self.indices.shape or self.upper.shape != self.indices.shape
                or not np.isfinite([self.lower, self.upper]).all()
                or not np.all(self.lower < self.upper)):
            raise ValueError('Invalid One Euro position envelope')
        for key in ('min_cutoff_hz', 'derivative_cutoff_hz', 'max_correction_rad'):
            if not np.isfinite(cfg[key]) or cfg[key] <= 0:
                raise ValueError('Invalid One Euro parameter: ' + key)
        if not np.isfinite(cfg['beta']) or cfg['beta'] < 0:
            raise ValueError('Invalid One Euro beta')
        if cfg['cold_start_bypass'] is not True:
            raise ValueError('This experiment preserves the cold-start chunk')

    def alpha(self, cutoff):
        return 1.0 / (1.0 + self.hz / (2.0 * np.pi * cutoff))

    def process(self, raw, state=None):
        begin = time.perf_counter()
        raw = self.schema.coerce(raw, name='One Euro suffix', ndim=2)
        if not len(raw):
            raise ValueError('Empty One Euro suffix')
        cfg, idx = self.config, self.indices
        cold = state is None
        if cold:
            previous = np.stack([raw[0, idx].astype(float), np.zeros(len(idx))])
        else:
            previous = np.asarray(state, dtype=float).copy()
            if previous.shape != (2, len(idx)) or not np.isfinite(previous).all():
                raise ValueError('Invalid One Euro boundary state')
        output = raw.copy()
        states = np.empty((len(raw), 2, len(idx)), dtype=float)
        derivative_alpha = self.alpha(cfg['derivative_cutoff_hz'])
        capped = bypassed = 0
        for i, row in enumerate(raw):
            target = row[idx].astype(float)
            derivative = derivative_alpha * (target - previous[0]) * self.hz + (1 - derivative_alpha) * previous[1]
            cutoff = cfg['min_cutoff_hz'] + cfg['beta'] * np.abs(derivative)
            alpha = self.alpha(cutoff)
            filtered = alpha * target + (1 - alpha) * previous[0]
            # Keep rounding inside the physical correction budget. State uses
            # the capped, emitted float32 target (anti-windup), not hidden output.
            margin = 4 * np.finfo(raw.dtype).eps * np.maximum(1., np.abs(target))
            cap = np.maximum(0., cfg['max_correction_rad'] - margin)
            capped += int(np.count_nonzero(np.abs(filtered-target) > cap)) if not cold else 0
            bounded = target + np.clip(filtered-target, -cap, cap)
            bounded = bounded.astype(raw.dtype).astype(float)
            valid = ((target >= self.lower) & (target <= self.upper)
                     & (bounded >= self.lower) & (bounded <= self.upper))
            # Preserve the original owner decision on out-of-envelope targets.
            # Do not turn this small filter into joint-limit correction.
            bypassed += int(np.count_nonzero(~valid)) if not cold else 0
            output[i, idx] = target if cold else np.where(valid, bounded, target)
            previous = np.stack([output[i, idx].astype(float), derivative])
            states[i] = previous
        deviation = float(np.max(np.abs(output.astype(float) - raw.astype(float))))
        if deviation > cfg['max_correction_rad']:
            raise ValueError('One Euro correction cap exceeded')
        report = {
            'trajectory_algorithm': 'bounded_one_euro_v1',
            'trajectory_cold_start_bypass': cold,
            'trajectory_max_deviation_rad': deviation,
            'trajectory_correction_cap_rad': cfg['max_correction_rad'],
            'trajectory_capped_joint_samples': capped,
            'trajectory_boundary_bypass_joint_samples': bypassed,
            'trajectory_grippers': 'unchanged',
            'trajectory_history_source': 'committed_plan_filter_state',
            'trajectory_prepare_ms': (time.perf_counter() - begin) * 1000,
        }
        return output, states, report
