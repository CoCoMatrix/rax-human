"""Opt-in bounded joint mapping before native RTC commitment. No device I/O."""
import json
from pathlib import Path

import numpy as np


def _fault_flag(config):
    value = config.get('fault_invalidation', True)
    if type(value) is not bool:
        raise ValueError('fault_invalidation must be a boolean')
    return value


def fault_invalidation_enabled(profile):
    return _fault_flag(json.loads(Path(profile).read_text()))


class PrecommitLimits:
    def __init__(self, profile, schema):
        self.config = json.loads(Path(profile).read_text())
        c = self.config
        self.strict_commit = _fault_flag(c)
        if c['schema'] != 'xpolicy.precommit-limits/v1' or schema.dim != 14:
            raise ValueError('Wrong precommit contract')
        self.indices = np.array(c['joint_indices'], dtype=int)
        if self.indices.tolist() != [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]:
            raise ValueError('Precommit must exclude both grippers')
        self.lower = np.asarray(c['lower_rad'], dtype=float)
        self.upper = np.asarray(c['upper_rad'], dtype=float)
        self.width = float(c['band_rad'])
        self.maximum = float(c['maximum_raw_overshoot_rad'])
        if (self.lower.shape != (12,) or self.upper.shape != (12,)
                or not np.isfinite([*self.lower, *self.upper, self.width, self.maximum]).all()
                or not 0 < self.width <= .06 or not 0 < self.maximum <= self.width
                or np.any(self.upper - self.lower <= 2*self.width)):
            raise ValueError('Invalid bounded precommit limits')
        # Act on float32-representable *interior* bounds so queue coercion
        # cannot turn a valid saturated endpoint into an owner rejection.
        lo32 = self.lower.astype(np.float32)
        hi32 = self.upper.astype(np.float32)
        self.map_lower = np.where(lo32.astype(float) < self.lower, np.nextafter(lo32, np.float32(np.inf)), lo32).astype(float)
        self.map_upper = np.where(hi32.astype(float) > self.upper, np.nextafter(hi32, np.float32(-np.inf)), hi32).astype(float)

    def process(self, suffix, required_steps):
        values = np.array(suffix, dtype=np.float64, copy=True)
        if values.ndim != 2 or values.shape[1] != 14 or not np.isfinite(values).all():
            raise ValueError('Invalid precommit action')
        if not 0 < required_steps <= len(values):
            raise ValueError('Invalid required commit span')
        x = values[:, self.indices].copy()
        over = np.maximum(self.lower-x, x-self.upper)
        if self.strict_commit and np.any(over[:required_steps] > self.maximum):
            raise ValueError('precommit_gross_joint_violation: required span exceeds correction budget')
        # C2 connection from identity to a flat limit: derivative in [0,1].
        # Gross unused tail values stay invalid; they are never silently repaired.
        eligible = over <= self.maximum
        y = x.copy()
        for bound, sign in ((self.map_upper, 1), (self.map_lower, -1)):
            distance = sign*(x-bound)
            t = np.clip((distance+self.width)/(2*self.width), 0, 1)
            mapped = bound + sign*(-self.width + 2*self.width*(t-t**3+.5*t**4))
            mask = eligible & (distance > -self.width)
            y[mask] = np.broadcast_to(mapped, x.shape)[mask]
        values[:, self.indices] = y
        if self.strict_commit:
            self.validate(values, required_steps)
        report = {'trajectory_precommit_enabled': True,
                  'trajectory_precommit_changed_values': int(np.count_nonzero(y[:required_steps] != x[:required_steps])),
                  'trajectory_precommit_max_correction_rad': float(np.max(np.abs(y[:required_steps]-x[:required_steps]))),
                  'trajectory_precommit_required_steps': required_steps,
                  'trajectory_precommit_fault_invalidation': self.strict_commit,
                  'trajectory_precommit_unrepaired_rows': int(np.count_nonzero(np.any(over[:required_steps] > self.maximum, axis=1)))}
        return values, report

    def validate(self, values, required_steps):
        q = np.asarray(values)[:required_steps, self.indices]
        if not np.isfinite(q).all() or np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError('precommit_final_span_outside_joint_limits')


def check_owner_command(requested, dispatched, names, gripper_limits, metadata):
    """SDK command identity, not encoder arrival. Signed grip mapping is explicit."""
    if metadata.get('native_precommit_version') != 1:
        raise ValueError('precommit step identity missing')
    expected = np.asarray([requested[n] for n in names], dtype=float)
    expected[[6, 13]] = np.clip(expected[[6, 13]], *gripper_limits)
    actual = np.asarray([dispatched[n] for n in names], dtype=float)
    if not np.isfinite(actual).all() or not np.allclose(expected, actual, rtol=0, atol=1e-7):
        raise ValueError('precommit owner changed the committed SDK command')


def guarded_dispatch(dispatch, on_failure):
    """A partial SDK write is a fault too. Never retry motion automatically."""
    try:
        dispatch()
    except Exception as exc:
        on_failure(type(exc).__name__ + ': ' + str(exc))

