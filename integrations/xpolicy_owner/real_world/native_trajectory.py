"""Opt-in finite-difference constrained suffix planning for native prefix RTC.

This is a discrete quadratic trajectory fit, not Ruckig or a guarantee about
motor dynamics. The committed prefix never changes. Three emitted targets
anchor position differences across the seam; grippers remain byte-identical.
"""
import json
import time
from pathlib import Path

import numpy as np


class NativeTrajectory:
    def __init__(self, path, schema, frequency_hz):
        import osqp
        from scipy import sparse
        self.osqp, self.sparse = osqp, sparse
        self.config = json.loads(Path(path).read_text())
        c = self.config
        if c['schema'] != 'xpolicy.native-trajectory/v1' or c['action_schema'] != schema.to_wire():
            raise ValueError('Native trajectory schema mismatch')
        if c['frequency_hz'] != frequency_hz:
            raise ValueError('Native trajectory cadence mismatch')
        self.hz = frequency_hz
        self.indices = np.array(c['continuous_indices'], dtype=int)
        expected = [i for i in range(schema.dim)
                    if schema.units[i] == 'rad' and schema.semantics[i] == 'absolute_joint']
        if list(self.indices) != expected:
            raise ValueError('Native smoothing must select exactly the absolute radian joints')
        self.lower = np.array(c['lower_rad'], dtype=float)
        self.upper = np.array(c['upper_rad'], dtype=float)
        if (self.lower.shape != self.indices.shape or self.upper.shape != self.indices.shape
                or not np.isfinite([self.lower, self.upper]).all()
                or np.any(self.lower >= self.upper)):
            raise ValueError('Invalid native trajectory joint limits')
        for k in ['max_velocity_rad_s', 'max_acceleration_rad_s2', 'max_jerk_rad_s3',
                  'max_deviation_rad', 'max_prepare_ms']:
            if not np.isfinite(c[k]) or c[k] <= 0:
                raise ValueError('Invalid native trajectory setting: ' + k)
        self.last_report = {}

    def process(self, raw, history):
        started = time.perf_counter()
        raw = np.asarray(raw, dtype=np.float32)
        history = np.asarray(history, dtype=np.float64)
        if (raw.ndim != 2 or len(raw) < 4 or history.shape != (3, raw.shape[1])
                or not np.isfinite(raw).all() or not np.isfinite(history).all()):
            raise ValueError('Native smoothing requires finite suffix and three boundary anchors')
        c, sp = self.config, self.sparse
        n, m = len(raw), len(self.indices)
        hist = history[:, self.indices]
        target = raw[:, self.indices].astype(float)
        # Centre the optimization on its seam position to improve conditioning.
        origin = hist[-1]
        target = target - origin
        hist = hist - origin
        matrices = [sp.eye(n * m, format='csc')]
        lower = [np.tile(self.lower + 1e-4 - origin, n)]
        upper = [np.tile(self.upper - 1e-4 - origin, n)]
        for order, key in enumerate(['max_velocity_rad_s', 'max_acceleration_rad_s2',
                                     'max_jerk_rad_s3'], 1):
            d = np.diff(np.eye(n + 3), n=order, axis=0)[-n:]
            offset = (d[:, :3] @ hist).reshape(-1)
            matrices.append(sp.kron(sp.csc_matrix(d[:, 3:]), sp.eye(m), format='csc'))
            # Numerical slack stays INSIDE the declared limits. Independently
            # validate returned float32 samples before any queue mutation.
            bound = 0.98 * c[key] / self.hz ** order
            lower.append(-bound - offset)
            upper.append(bound - offset)
        # Every accepted reserve ends at rest, so the following replan has a
        # feasible continuation even when its new tracking targets disagree.
        terminal = np.zeros((2, n))
        terminal[0, -2:] = [-1, 1]
        terminal[1, -3:] = [-1, 1, 0]
        matrices.append(sp.kron(sp.csc_matrix(terminal), sp.eye(m), format='csc'))
        lower.append(np.zeros(2*m)); upper.append(np.zeros(2*m))
        a = sp.vstack(matrices, format='csc')
        lo, hi = np.concatenate(lower), np.concatenate(upper)
        solver = self.osqp.OSQP()
        solver.setup(P=sp.eye(n*m, format='csc') * 2, q=-2*target.reshape(-1),
                     A=a, l=lo, u=hi, verbose=False, eps_abs=2e-6, eps_rel=2e-6,
                     max_iter=6000, polishing=True, check_termination=10,
                     time_limit=0.7*c['max_prepare_ms']/1000)
        result = solver.solve(raise_error=False)
        if result.x is None or not np.isfinite(result.x).all() or result.info.status_val not in (1, 2, 7, 8):
            raise ValueError('Native trajectory fit failed: ' + result.info.status)
        output = raw.copy()
        output[:, self.indices] = (result.x.reshape(n, m) + origin).astype(np.float32)
        delta = float(np.max(np.abs(output[:, self.indices] - raw[:, self.indices])))
        if delta > c['max_deviation_rad']:
            raise ValueError(f'Native trajectory deviation {delta:.4f} exceeds {c["max_deviation_rad"]}')
        # Validate the actual float32 samples, not merely solver convergence.
        combined = np.vstack([history[:, self.indices], output[:, self.indices]])
        maxima = []
        for order, key in enumerate(['max_velocity_rad_s', 'max_acceleration_rad_s2',
                                     'max_jerk_rad_s3'], 1):
            maximum = float(np.max(np.abs(np.diff(combined, n=order, axis=0)[-n:]))) * self.hz**order
            maxima.append(maximum)
            if maximum > c[key] + 0.04:
                raise ValueError('Native trajectory float32 constraint failed: ' + key)
        if np.any(output[:, self.indices] < self.lower) or np.any(output[:, self.indices] > self.upper):
            raise ValueError('Native trajectory position limit failed')
        if np.max(np.abs(np.diff(output[-3:, self.indices], axis=0))) > 1e-5:
            raise ValueError('Native trajectory terminal hold constraint failed')
        elapsed = (time.perf_counter() - started) * 1000
        if elapsed > c['max_prepare_ms']:
            raise ValueError('Native trajectory preparation deadline exceeded')
        self.last_report = {'trajectory_algorithm': 'native_suffix_qp_v1',
                            'trajectory_solver_status': result.info.status,
                            'trajectory_prepare_ms': elapsed,
                            'trajectory_max_deviation_rad': delta,
                            'trajectory_max_velocity_rad_s': maxima[0],
                            'trajectory_max_acceleration_rad_s2': maxima[1],
                            'trajectory_max_jerk_rad_s3': maxima[2],
                            'trajectory_grippers': 'unchanged',
                            'trajectory_history_source': 'emitted_processed_plan'}
        return output
