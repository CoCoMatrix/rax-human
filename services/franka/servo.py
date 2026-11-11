"""Host-side integral outer loop around the Polymetis arm bridge.

The bridge's DROID differential IK re-anchors every command on the measured joints, so
its impedance spring settles short of the goal (about 5 mm in z from an unmodelled
payload, up to 1 cm sideways). This loop sends `goal + bias` and integrates the residual
into the bias, only near the goal or once the arm has stalled, so it does not wind up
while the bridge is still rate-limiting the approach. Verified on the FR3 on 2026-10-03:
10 mm, 3-5 cm and +-15 deg moves settle within 1 mm / 0.5 deg in 1-6 s.
"""
from dataclasses import dataclass
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .bridges import TargetRejected
from .kinematics import link8_to_tcp, tcp_to_link8


class MotionStopped(RuntimeError):
    """The motion was interrupted (operator disarm, workspace guard); the arm holds."""


@dataclass
class ServoSettings:
    ki_position: float = 3.0              # 1/s
    ki_rotation: float = 2.0              # 1/s
    integrate_below_m: float = 0.012
    integrate_below_rad: float = np.radians(3.0)
    stall_speed_mps: float = 0.01
    stall_rate_radps: float = np.radians(2.0)
    max_bias_m: float = 0.03
    max_bias_rad: float = np.radians(8.0)
    settle_s: float = 0.3
    period_s: float = 0.05
    # Measured steady sag with the current payload model (flange frame, metres).
    gravity_bias_m: tuple = (-0.0005, 0.0006, 0.0053)

    @classmethod
    def from_options(cls, options):
        known = {k: v for k, v in (options or {}).items() if k in cls.__dataclass_fields__}
        if 'gravity_bias_m' in known:
            known['gravity_bias_m'] = tuple(float(v) for v in known['gravity_bias_m'])
        return cls(**known)


def rotation_error(target_quat, measured_quat):
    """Rotation vector taking the measured orientation to the target (base frame)."""
    return (Rotation.from_quat(target_quat) * Rotation.from_quat(measured_quat).inv()).as_rotvec()


class PoseServo:
    def __init__(self, arm, settings=None, *, clock=time.monotonic, sleep=time.sleep):
        self.arm = arm
        self.settings = settings or ServoSettings()
        self.clock, self.sleep = clock, sleep
        self.bias_r = np.zeros(3)

    def move_tcp(self, position, quaternion, *, tolerance_m, tolerance_rad, timeout_s, guard):
        """Servo the TCP to a base-frame pose. `guard(state)` raises MotionStopped to abort.

        Returns a summary dict; raises TimeoutError (arm held) if not settled in time.
        """
        st = self.settings
        goal_p, goal_q = tcp_to_link8(position, quaternion)
        goal_R = Rotation.from_quat(goal_q)
        bias_p = np.asarray(st.gravity_bias_m, dtype=np.float64).copy()
        start = self.clock()
        inside_since = previous = None
        try:
            while True:
                now = self.clock()
                state = self.arm.state()
                guard(state)
                err_p = goal_p - state['arm_pos']
                err_r = rotation_error(goal_q, state['arm_quat'])
                dist, angle = float(np.linalg.norm(err_p)), float(np.linalg.norm(err_r))
                if dist <= tolerance_m and angle <= tolerance_rad:
                    inside_since = inside_since or now
                    if now - inside_since >= st.settle_s:
                        self.arm.hold()
                        return self._summary(state, position, quaternion, now - start, bias_p, reached=True)
                else:
                    inside_since = None
                if now - start > timeout_s:
                    self.arm.hold()
                    summary = self._summary(state, position, quaternion, now - start, bias_p, reached=False)
                    raise TimeoutError(f"pose not reached in {timeout_s:.1f}s: position error "
                                       f"{summary['position_error_mm']:.1f} mm, rotation error "
                                       f"{summary['rotation_error_deg']:.2f} deg; the arm holds where it is")
                speed = rate = 1.0
                if previous is not None:
                    dt = max(now - previous[0], 1e-3)
                    speed = float(np.linalg.norm(state['arm_pos'] - previous[1])) / dt
                    rate = (Rotation.from_quat(state['arm_quat']) * previous[2].inv()).magnitude() / dt
                previous = (now, state['arm_pos'], Rotation.from_quat(state['arm_quat']))
                if dist < st.integrate_below_m or speed < st.stall_speed_mps:
                    bias_p = np.clip(bias_p + st.ki_position * err_p * st.period_s, -st.max_bias_m, st.max_bias_m)
                if angle < st.integrate_below_rad or rate < st.stall_rate_radps:
                    self.bias_r = np.clip(self.bias_r + st.ki_rotation * err_r * st.period_s,
                                          -st.max_bias_rad, st.max_bias_rad)
                command_q = (Rotation.from_rotvec(self.bias_r) * goal_R).as_quat()
                self.arm.set_target(goal_p + bias_p, command_q)
                self.sleep(st.period_s)
        except (MotionStopped, TargetRejected, TimeoutError):
            raise
        except BaseException:
            try:
                self.arm.hold()
            finally:
                raise

    @staticmethod
    def _summary(state, position, quaternion, seconds, bias_p, *, reached):
        tcp_p, tcp_q = link8_to_tcp(state['arm_pos'], state['arm_quat'])
        return {'reached': reached, 'seconds': round(seconds, 2),
                'position_error_mm': round(float(np.linalg.norm(np.asarray(position) - tcp_p)) * 1000, 2),
                'rotation_error_deg': round(float(np.degrees(np.linalg.norm(rotation_error(quaternion, tcp_q)))), 2),
                'measured_position': np.round(tcp_p, 5).tolist(), 'measured_quaternion': np.round(tcp_q, 5).tolist(),
                'final_bias_mm': np.round(bias_p * 1000, 2).tolist()}
