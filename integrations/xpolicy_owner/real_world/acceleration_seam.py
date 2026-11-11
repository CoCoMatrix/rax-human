"""Opt-in local discrete acceleration/jerk constrained native RTC bridge.

Only the first 7 or 8 uncommitted arm targets can change. Three fixed history
samples and three unchanged exit samples participate in every difference
constraint. This bounds sampled commands, not continuous motor dynamics.
Exact acceleration matching is preferred; bounded-jerk matching is the only
relaxation. Infeasible candidates raise, so the existing RTC fails closed.
There is no per-joint raw fallback, retiming, terminal rest, or gripper edit.
"""
import json
import time
from pathlib import Path

import numpy as np


class AccelerationSeam:
    anchor_count = 3
    algorithm = 'local_acceleration_seam_qp_v1'
    limit_keys = ('max_velocity_rad_s', 'max_acceleration_rad_s2', 'max_jerk_rad_s3')

    def __init__(self, profile, schema, frequency_hz):
        import osqp
        from scipy import sparse
        self.osqp, self.sp = osqp, sparse
        self.config = c = json.loads(Path(profile).read_text())
        if (c['schema'] != 'xpolicy.acceleration-seam/v1'
                or c['action_schema'] != schema.to_wire() or c['frequency_hz'] != frequency_hz):
            raise ValueError('Acceleration seam schema/cadence mismatch')
        self.dim, self.hz = len(schema.units), frequency_hz
        self.indices = np.asarray(c['continuous_indices'], dtype=int)
        expected = [i for i, (s, u) in enumerate(zip(schema.semantics, schema.units))
                    if s == 'absolute_joint' and u == 'rad']
        if self.indices.tolist() != expected or not expected:
            raise ValueError('Acceleration seam must select exactly the absolute radian joints')
        self.lower, self.upper = np.asarray(c['lower_rad']), np.asarray(c['upper_rad'])
        if (self.lower.shape != self.indices.shape or self.upper.shape != self.indices.shape
                or not np.isfinite([self.lower, self.upper]).all() or not np.all(self.lower < self.upper)):
            raise ValueError('Invalid acceleration seam position limits')
        self.windows = c['bridge_steps_candidates']
        if (not isinstance(self.windows, list) or not self.windows
                or any(type(n) is not int or not 4 <= n <= 9 for n in self.windows)
                or self.windows != sorted(set(self.windows))):
            raise ValueError('Acceleration seam windows must fit inside the ten-step commit')
        for key in (*self.limit_keys, 'max_correction_rad', 'max_prepare_ms', 'solver_time_limit_ms', 'jerk_weight'):
            if not np.isfinite(c[key]) or c[key] <= 0:
                raise ValueError('Invalid acceleration seam setting: ' + key)
        if c['max_correction_rad'] <= 2e-6:
            raise ValueError('Correction budget must exceed numerical interior margin')

    def _differences(self, combined, modified_count):
        # Only differences ending at a new target or one of the three exit
        # samples are validated. Already committed history cannot be changed.
        return [np.diff(combined, n=k, axis=0)[-(modified_count + 3):] * self.hz**k
                for k in (1, 2, 3)]

    def _violations(self, output, raw, history, n, exact):
        m = n - 1
        q = output[:m + 3, self.indices].astype(float)
        stitched = np.vstack([history[:, self.indices], q])
        differences = self._differences(stitched, m)
        reasons = {}
        correction = np.max(np.abs(output[:m, self.indices].astype(float)
                                   - raw[:m, self.indices].astype(float)), axis=0)
        entry = differences[1][0] - np.diff(history[:, self.indices], n=2, axis=0)[0] * self.hz**2
        exit_delta = differences[1][-1] - differences[1][-2]
        for j, idx in enumerate(self.indices):
            failed = []
            if np.any(q[:, j] < self.lower[j]) or np.any(q[:, j] > self.upper[j]):
                failed.append('position')
            if correction[j] > self.config['max_correction_rad']:
                failed.append('correction')
            for values, key in zip(differences, self.limit_keys):
                if np.max(np.abs(values[:, j])) > self.config[key]:
                    failed.append(key)
            # float32 position quantization: 0.002 rad/s² is validation
            # tolerance for matching only; hard derivative limits stay exact.
            if exact and (abs(entry[j]) > .002 or abs(exit_delta[j]) > .002):
                failed.append('acceleration_match')
            if failed:
                reasons[str(int(idx))] = failed
        return reasons, differences, correction, entry, exit_delta

    def _solve(self, raw, history, n, exact, remaining_s):
        if not self._violations(raw, raw, history, n, exact)[0]:
            return raw.copy(), 'raw_already_feasible'
        sp, c = self.sp, self.config
        m, joints = n - 1, len(self.indices)
        origin = history[-1, self.indices].astype(float)
        h = history[:, self.indices].astype(float) - origin
        target = raw[:m, self.indices].astype(float) - origin
        tail = raw[m:m+3, self.indices].astype(float) - origin
        # Unknowns are time-major. Fixed history/exit samples are offsets.
        fixed = np.vstack([h, np.zeros_like(target), tail])
        mats = [sp.eye(m * joints, format='csc')]
        radius = c['max_correction_rad'] - 2e-6
        lo = [np.maximum(target - radius, self.lower + 2e-6 - origin).reshape(-1)]
        hi = [np.minimum(target + radius, self.upper - 2e-6 - origin).reshape(-1)]
        ds, offsets = [], []
        for order, key in enumerate(self.limit_keys, 1):
            d = np.diff(np.eye(m + 6), n=order, axis=0)[-(m + 3):]
            matrix = sp.kron(sp.csc_matrix(d[:, 3:3+m]), sp.eye(joints), format='csc')
            offset = (d @ fixed).reshape(-1)
            ds.append(matrix); offsets.append(offset)
            mats.append(matrix)
            bound = .995 * c[key] / self.hz**order
            lo.append(-bound - offset); hi.append(bound - offset)
        if exact:
            # Equal backward second differences at entry and exit. The
            # bounded mode removes only these equalities, never any limits.
            edge = np.zeros((2, m)); edge[0, 0] = 1; edge[1, -1] = 1
            values = np.stack([3*h[-1]-3*h[-2]+h[-3], 3*tail[0]-3*tail[1]+tail[2]])
            mats.append(sp.kron(sp.csc_matrix(edge), sp.eye(joints), format='csc'))
            lo.append(values.reshape(-1)); hi.append(values.reshape(-1))
        lower, upper = np.concatenate(lo), np.concatenate(hi)
        if np.any(lower > upper):
            return None, 'empty_position_correction_interval'
        weight = c['jerk_weight']
        p = 2 * (sp.eye(m*joints, format='csc') + weight * (ds[2].T @ ds[2]))
        linear = -2*target.reshape(-1) + 2*weight*(ds[2].T @ offsets[2])
        solver = self.osqp.OSQP()
        solver.setup(P=sp.triu(p, format='csc'), q=linear, A=sp.vstack(mats, format='csc'),
                     l=lower, u=upper, verbose=False, eps_abs=1e-8, eps_rel=1e-8,
                     max_iter=3000, polishing=True, check_termination=10,
                     time_limit=min(c['solver_time_limit_ms']/1000, remaining_s))
        solution = solver.solve(raise_error=False)
        if solution.info.status_val not in (1, 2) or solution.x is None or not np.isfinite(solution.x).all():
            return None, solution.info.status
        output = raw.copy()
        output[:m, self.indices] = (solution.x.reshape(m, joints) + origin).astype(raw.dtype)
        return output, solution.info.status

    def process(self, suffix, anchors):
        start = time.perf_counter()
        raw, history = np.asarray(suffix), np.asarray(anchors, dtype=float)
        if (raw.ndim != 2 or raw.shape[1] != self.dim or len(raw) < max(self.windows)+2
                or raw.dtype not in (np.dtype('float32'), np.dtype('float64'))
                or history.shape != (3, self.dim)
                or not np.isfinite(raw).all() or not np.isfinite(history).all()):
            raise ValueError('Acceleration seam requires finite suffix and three committed anchors')
        attempts = []
        diagnostics = self._violations(raw, raw, history, self.windows[0], False)[0]
        for exact in (True, False):
            for n in self.windows:
                remaining = self.config['max_prepare_ms']/1000 - (time.perf_counter()-start)
                if remaining <= .001:
                    raise ValueError('Acceleration seam preparation deadline: ' + json.dumps(attempts))
                output, status = self._solve(raw, history, n, exact, remaining)
                attempt = {'steps': n, 'mode': 'acceleration_match' if exact else 'bounded_jerk', 'status': status}
                attempts.append(attempt)
                if output is None:
                    continue
                failed, differences, correction, entry, exit_delta = self._violations(output, raw, history, n, exact)
                if failed:
                    attempt['violations'] = failed
                    continue
                elapsed = (time.perf_counter()-start)*1000
                if elapsed > self.config['max_prepare_ms']:
                    raise ValueError('Acceleration seam preparation deadline exceeded')
                raw_diffs = self._differences(np.vstack([history[:, self.indices], raw[:n+2, self.indices]]), n-1)
                report = {
                    'trajectory_algorithm': self.algorithm, 'trajectory_bridge_steps': n,
                    'trajectory_boundary_mode': attempt['mode'], 'trajectory_prepare_ms': elapsed,
                    'trajectory_bridged_joint_count': len(self.indices),
                    'trajectory_bypassed_joint_indices': [],
                    'trajectory_max_correction_rad': float(np.max(correction)),
                    'trajectory_peak_acceleration_rad_s2': float(np.max(np.abs(differences[1]))),
                    'trajectory_peak_jerk_rad_s3': float(np.max(np.abs(differences[2]))),
                    'trajectory_peak_velocity_rad_s': float(np.max(np.abs(differences[0]))),
                    'trajectory_raw_peak_acceleration_rad_s2': float(np.max(np.abs(raw_diffs[1]))),
                    'trajectory_raw_peak_jerk_rad_s3': float(np.max(np.abs(raw_diffs[2]))),
                    'trajectory_entry_acceleration_change_rad_s2': float(np.max(np.abs(entry))),
                    'trajectory_exit_acceleration_change_rad_s2': float(np.max(np.abs(exit_delta))),
                    'trajectory_joint_diagnostics_json': json.dumps(diagnostics, separators=(',', ':')),
                    'trajectory_attempts_json': json.dumps(attempts, separators=(',', ':')),
                    'trajectory_grippers': 'unchanged', 'trajectory_history_source': 'committed_processed_plan',
                }
                return output, report
        raise ValueError('Acceleration seam infeasible; no raw fallback: ' + json.dumps(
            {'raw_joint_violations': diagnostics, 'attempts': attempts}, separators=(',', ':')))
