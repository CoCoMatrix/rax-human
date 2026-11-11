"""Numerical IK used only to pre-check targets before anything moves.

The bridge solves IK incrementally from the measured joints and refuses any solution
closer than 0.2 rad to a joint limit. Tracking the same straight TCP line here, with
damped least squares continued waypoint by waypoint from the current joints, predicts
those refusals so an unreachable target fails validation instead of mid-motion.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from .kinematics import LOWER, UPPER, joint_frames

JOINT_NAMES = ('J1 base yaw', 'J2 shoulder', 'J3 upper-arm roll', 'J4 elbow', 'J5 forearm roll',
               'J6 wrist pitch', 'J7 flange roll')


def jacobian(q):
    """6x7 geometric Jacobian of the TCP in the base frame, and the TCP pose."""
    frames = joint_frames(q)
    tcp = frames[8]
    J = np.zeros((6, 7))
    for i in range(7):
        z, origin = frames[i][:3, 2], frames[i][:3, 3]
        J[:3, i] = np.cross(z, tcp[:3, 3] - origin)
        J[3:, i] = z
    return J, tcp


def pose_error(T_goal, T):
    dp = T_goal[:3, 3] - T[:3, 3]
    dr = (Rotation.from_matrix(T_goal[:3, :3]) * Rotation.from_matrix(T[:3, :3]).inv()).as_rotvec()
    return np.concatenate([dp, dr])


def solve(q0, T_goal, *, iterations=150, damping=0.05, max_step=0.1, tolerance=(1e-4, 1e-3)):
    """DLS from q0 (no joint clamping, like an unconstrained incremental tracker).

    Returns (q, converged)."""
    q = np.asarray(q0, dtype=np.float64).copy()
    for _ in range(iterations):
        J, T = jacobian(q)
        error = pose_error(T_goal, T)
        if np.linalg.norm(error[:3]) < tolerance[0] and np.linalg.norm(error[3:]) < tolerance[1]:
            return q, True
        dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), error)
        scale = np.max(np.abs(dq)) / max_step
        q = q + (dq / scale if scale > 1 else dq)
    return q, False


def limit_margins(q):
    q = np.asarray(q, dtype=np.float64)
    return np.minimum(q - LOWER, UPPER - q)


def check_line(q0, T_start, T_goal, *, margin, waypoints=12):
    """Follow the straight TCP line start -> goal. Returns None or a refusal message."""
    q = np.asarray(q0, dtype=np.float64)
    r0, r1 = Rotation.from_matrix(T_start[:3, :3]), Rotation.from_matrix(T_goal[:3, :3])
    delta = (r1 * r0.inv()).as_rotvec()
    for k in range(1, waypoints + 1):
        s = k / waypoints
        T = np.eye(4)
        T[:3, 3] = (1 - s) * T_start[:3, 3] + s * T_goal[:3, 3]
        T[:3, :3] = (Rotation.from_rotvec(delta * s) * r0).as_matrix()
        q, converged = solve(q, T)
        if not converged:
            return (f'no IK solution at {s:.0%} of the straight path: the pose is outside the arm\'s reach '
                    'for this orientation')
        margins = limit_margins(q)
        worst = int(np.argmin(margins))
        if margins[worst] < margin:
            side = 'upper' if UPPER[worst] - q[worst] < q[worst] - LOWER[worst] else 'lower'
            return (f'{JOINT_NAMES[worst]} would come within {margins[worst]:.2f} rad of its {side} limit '
                    f'(the controller needs {margin:.2f}) at {s:.0%} of the straight path')
    return None
