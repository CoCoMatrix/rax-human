"""Opt-in upper overshoot projection before native RTC commitment.

This changes model targets, never physical limits or the final owner safety
gate. Values outside the explicit correction window pass through unchanged
for rejection by the existing owner. No time interpolation or gripper edits.
"""
import json
import time
from pathlib import Path

import numpy as np


class BoundedJointClip:
    def __init__(self, profile, schema, frequency_hz):
        self.config = json.loads(Path(profile).read_text())
        cfg = self.config
        if (cfg['schema'] != 'xpolicy.bounded-joint-clip/v1'
                or cfg['action_schema'] != schema.to_wire()
                or cfg['frequency_hz'] != frequency_hz):
            raise ValueError('Joint clip schema/cadence mismatch')
        self.schema = schema
        self.index = schema.fields.index(cfg['field'])
        if schema.units[self.index] != 'rad' or schema.semantics[self.index] != 'absolute_joint':
            raise ValueError('Joint clip requires an absolute radian arm joint')
        self.upper = float(cfg['physical_upper_rad'])
        self.ceiling = float(cfg['max_input_rad'])
        if not np.isfinite([self.upper, self.ceiling]).all() or not self.upper < self.ceiling:
            raise ValueError('Invalid bounded joint correction window')

    def process(self, raw):
        begin = time.perf_counter()
        raw = self.schema.coerce(raw, name='joint clip suffix', ndim=2)
        output = raw.copy()
        # Round inward so float32 conversion cannot exceed the physical cap.
        upper = np.float32(self.upper)
        if float(upper) > self.upper:
            upper = np.nextafter(upper, np.float32(-np.inf))
        targets = raw[:, self.index].astype(float)
        # The model wire is float32; include its representation of exactly 93deg.
        mask = (targets > self.upper) & (targets <= float(np.float32(self.ceiling)))
        output[mask, self.index] = upper
        report = {
            'trajectory_algorithm': 'bounded_joint_clip_v1',
            'trajectory_clip_field': self.config['field'],
            'trajectory_clipped_joint_samples': int(mask.sum()),
            'trajectory_above_clip_window_samples': int((targets > float(np.float32(self.ceiling))).sum()),
            'trajectory_max_deviation_rad': float(np.max(np.abs(output.astype(float)-raw.astype(float)))),
            'trajectory_grippers': 'unchanged',
            'trajectory_history_source': 'clipped_committed_plan',
            'trajectory_prepare_ms': (time.perf_counter()-begin)*1000,
        }
        return output, report
