"""Retiming of absolute-position actions before wall-clock RTC alignment.

The model cadence and hardware dispatch cadence remain unchanged. All action
channels use one source phase; downstream RTC lengths are dispatch ticks.
"""
from dataclasses import dataclass
import math

import numpy as np

from XPolicyLab.real_world.contracts import VectorSchema


@dataclass(frozen=True)
class ActionTimebase:
    scale: float = 1.0

    def __post_init__(self):
        if isinstance(self.scale, (bool, str)):
            raise ValueError("trajectory time scale must be a number in [0.25, 1]")
        value = float(self.scale)
        if not math.isfinite(value) or not .25 <= value <= 1:
            raise ValueError("trajectory time scale must be in [0.25, 1]")
        object.__setattr__(self, 'scale', value)

    def validate_schema(self, schema: VectorSchema):
        if self.scale == 1:
            return
        supported = {('absolute_joint', 'rad'), ('absolute_gripper_width', 'm'),
                     ('absolute_position', 'rad'), ('absolute_position', 'm')}
        if any(pair not in supported for pair in zip(schema.semantics, schema.units)):
            raise ValueError("retiming requires explicit absolute-position units for every channel")

    def dispatch_steps(self, source_steps: int) -> int:
        return math.ceil(source_steps / self.scale)

    def expand(self, values: np.ndarray) -> np.ndarray:
        if self.scale == 1:
            return values
        # A source row occupies one source period. Keep the last target through
        # its final period; never extrapolate past the supplied execution prefix.
        phase = np.minimum(np.arange(self.dispatch_steps(len(values))) * self.scale,
                           len(values) - 1)
        lower = np.floor(phase).astype(np.intp)
        upper = np.minimum(lower + 1, len(values) - 1)
        weight = (phase - lower)[:, None]
        result = ((1 - weight) * values[lower] + weight * values[upper]).astype(np.float32)
        if not np.isfinite(result).all():
            raise ValueError("retimed action contains non-finite values")
        return result

    def status(self, frequency_hz: float) -> dict:
        return {'trajectory_time_scale': self.scale,
                'dispatch_frequency_hz': frequency_hz,
                'source_phase_steps_per_second': frequency_hz * self.scale,
                'rtc_step_units': 'dispatch_ticks',
                'channels': 'all_absolute_position_channels'}
