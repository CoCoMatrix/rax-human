"""Opt-in 8/6/4 discrete velocity bridges with gripper-event pose protection.

Only uncommitted targets are touched. A failed fit never clips a target or
creates a new HOLD; the existing owner remains the final safety authority.
"""
import json
import time

import numpy as np

from XPolicyLab.real_world.velocity_seam import VelocitySeam


class TieredVelocitySeam(VelocitySeam):
    anchor_count = 4
    algorithm = 'tiered_velocity_seam_v1'

    def __init__(self, profile, schema, frequency_hz):
        super().__init__(profile, schema, frequency_hz)
        cfg = self.config
        if cfg.get('tiered_velocity') is not True or cfg.get('acceleration_blend'):
            raise ValueError('Tiered velocity requires an exclusive explicit opt-in')
        if self.n != 8 or cfg.get('bridge_steps_candidates') != [8, 6, 4]:
            raise ValueError('Tiered velocity requires the reviewed 8/6/4 windows')
        self.guard = cfg['gripper_event_guard']
        g = self.guard
        self.guard_enabled = g.get('enabled', True)
        if type(self.guard_enabled) is not bool:
            raise ValueError('Gripper-event guard enabled must be a boolean')
        if (g['before_steps'] != 2 or g['after_steps'] != 2
                or not 0 < g['delta_m'] <= .1 or not 0 < g['closed_width_m'] < .1):
            raise ValueError('Invalid gripper-event guard')
        groups = g['groups']
        joints = []
        grippers = []
        for group in groups:
            j = group['gripper_index']
            if (schema.semantics[j] != 'absolute_gripper_width'
                    or schema.units[j] != 'm' or not group['joint_indices']):
                raise ValueError('Invalid gripper-event group')
            joints.extend(group['joint_indices'])
            grippers.append(j)
        if sorted(joints) != self.indices.tolist() or len(set(grippers)) != len(grippers):
            raise ValueError('Gripper groups must cover each arm joint exactly once')
        self.weights_by_n = {self.n: self.weights}
        for n in (6, 4):
            nodes = np.array([0., 1., n, n+1.])
            t = np.arange(1., n)
            weights = np.ones((n-1, 4))
            for j in range(4):
                for k in range(4):
                    if j != k:
                        weights[:, j] *= (t-nodes[k])/(nodes[j]-nodes[k])
            self.weights_by_n[n] = weights

    def _protected(self, raw, history):
        # Clamp only a temporary event detector to the owner's physical width
        # range. Signed policy gripper targets themselves are never changed.
        protected = np.zeros((self.n-1, raw.shape[1]), dtype=bool)
        events = {}
        if not self.guard_enabled:
            return protected, events
        for group in self.guard['groups']:
            grip = group['gripper_index']
            q = np.clip(np.r_[history[:, grip], raw[:self.n+2, grip]], 0., .1)
            closed = q <= self.guard['closed_width_m']
            changed = ((np.abs(np.diff(q)) >= self.guard['delta_m'])
                       | (closed[1:] != closed[:-1]))
            steps = np.flatnonzero(changed)+1-len(history)
            events[str(grip)] = steps.tolist()
            for step in steps:
                start = max(0, int(step)-self.guard['before_steps'])
                end = min(self.n-1, int(step)+self.guard['after_steps']+1)
                if start < end:
                    protected[start:end, group['joint_indices']] = True
        return protected, events

    def process(self, suffix, anchors):
        begin = time.perf_counter()
        raw, history = np.asarray(suffix), np.asarray(anchors)
        if (raw.ndim != 2 or history.shape != (self.anchor_count, raw.shape[1])
                or len(raw) < self.n+2 or raw.dtype not in (np.float32, np.float64)
                or not np.all(np.isfinite(raw)) or not np.all(np.isfinite(history))):
            raise ValueError('Invalid tiered velocity samples')
        baseline, _ = super().process(raw, history[-2:])
        idx = self.indices
        prior, q = history[-2:, idx].astype(float), raw[:, idx].astype(float)
        increment = prior[-1]-prior[-2]
        protected, events = self._protected(raw, history)
        output = raw.copy()
        selected = np.zeros(len(idx), dtype=int)
        attempts = {}
        # Common evaluation span prevents a shorter candidate from hiding a
        # large exit acceleration/jerk just outside its own fit window.
        previous = np.vstack([history[:, idx], baseline[:self.n+2, idx]]).astype(float)
        previous_accel = np.max(np.abs(np.diff(previous, n=2, axis=0)), axis=0)
        previous_jerk = np.max(np.abs(np.diff(previous, n=3, axis=0)), axis=0)
        for n in (8, 6, 4):
            boundary = np.stack([prior[-1], prior[-1]+increment, q[n-1], q[n]])
            candidate = (self.weights_by_n[n] @ boundary).astype(raw.dtype).astype(float)
            stitched = np.vstack([prior, candidate, q[n-1:n+1]])
            original = np.vstack([prior, q[:n+1]])
            checks = {
                'correction': np.max(np.abs(candidate-q[:n-1]), axis=0) <= self.config['max_correction_rad'],
                'step': np.max(np.abs(np.diff(stitched, axis=0)), axis=0) <= self.config['max_bridge_step_rad'],
                'acceleration': np.max(np.abs(np.diff(stitched, n=2, axis=0)), axis=0)
                    <= np.max(np.abs(np.diff(original, n=2, axis=0)), axis=0)+1e-6,
                'position': np.all((candidate >= self.lower) & (candidate <= self.upper), axis=0),
                # Finish the entire bridge before a protected target. Do not
                # overwrite individual samples inside a polynomial afterwards.
                'gripper_guard': ~np.any(protected[:n-1, idx], axis=0),
            }
            if n < 8:
                evaluated = np.vstack([history[:, idx], candidate, q[n-1:self.n+2]]).astype(float)
                checks['fallback_acceleration'] = np.max(np.abs(np.diff(evaluated, n=2, axis=0)), axis=0) <= previous_accel+1e-6
                checks['fallback_jerk'] = np.max(np.abs(np.diff(evaluated, n=3, axis=0)), axis=0) <= previous_jerk+1e-6
            valid = np.logical_and.reduce(list(checks.values()))
            pending = selected == 0
            attempts[str(n)] = {str(int(idx[j])): [key for key, ok in checks.items() if not ok[j]]
                                for j in np.flatnonzero(pending & ~valid)}
            accepted = pending & valid
            output[:n-1, idx[accepted]] = candidate[:, accepted]
            selected[accepted] = n
        actual = np.vstack([history[:, idx], output[:self.n+2, idx]]).astype(float)
        original = np.vstack([history[:, idx], raw[:self.n+2, idx]]).astype(float)
        report = {
            'trajectory_algorithm': self.algorithm,
            'trajectory_bridge_steps': self.n,
            'trajectory_bridged_joint_count': int(np.count_nonzero(selected)),
            'trajectory_bypassed_joint_indices': idx[selected == 0].tolist(),
            'trajectory_selected_windows_json': json.dumps(dict(zip(map(str, idx), map(int, selected)))),
            'trajectory_attempt_failures_json': json.dumps(attempts),
            'trajectory_gripper_events_json': json.dumps(events),
            'trajectory_gripper_guard_enabled': self.guard_enabled,
            'trajectory_gripper_protected_samples': int(protected.sum()),
            'trajectory_protected_max_correction_rad': float(np.max(np.abs(output[:self.n-1][protected]-raw[:self.n-1][protected]), initial=0)),
            'trajectory_max_correction_rad': float(np.max(np.abs(output-raw))),
            'trajectory_peak_acceleration_rad_s2': float(np.max(np.abs(np.diff(actual, n=2, axis=0)))*self.hz**2),
            'trajectory_raw_peak_acceleration_rad_s2': float(np.max(np.abs(np.diff(original, n=2, axis=0)))*self.hz**2),
            'trajectory_peak_jerk_rad_s3': float(np.max(np.abs(np.diff(actual, n=3, axis=0)))*self.hz**3),
            'trajectory_raw_peak_jerk_rad_s3': float(np.max(np.abs(np.diff(original, n=3, axis=0)))*self.hz**3),
            'trajectory_raw_seam_velocity_jump_rad_s': float(np.max(np.abs(q[0]-prior[-1]-increment))*self.hz),
            'trajectory_seam_velocity_jump_rad_s': float(np.max(np.abs(output[0, idx]-prior[-1]-increment))*self.hz),
            'trajectory_prepare_ms': (time.perf_counter()-begin)*1000,
        }
        return output, report
