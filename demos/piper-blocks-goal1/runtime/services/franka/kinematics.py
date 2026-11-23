"""FR3 forward kinematics and the fixed wrist frames, numpy/scipy only.

Modified-DH parameters are Franka's published ones (identical for Panda and FR3).
`panda_link8` is the flange Polymetis reports; `fr3_hand_tcp` is the point between
the fingertips: link8 --(yaw -45 deg)--> hand --(+0.1034 m along z)--> tcp.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# (a, d, alpha) per joint, Craig convention: T = RotX(alpha) TransX(a) RotZ(q) TransZ(d).
DH = np.array([
    [0.0, 0.333, 0.0],
    [0.0, 0.0, -np.pi / 2],
    [0.0, 0.316, np.pi / 2],
    [0.0825, 0.0, np.pi / 2],
    [-0.0825, 0.384, -np.pi / 2],
    [0.0, 0.0, np.pi / 2],
    [0.088, 0.0, np.pi / 2],
])
FLANGE_D = 0.107
LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
UPPER = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
LINK8_TO_HAND_YAW = -np.pi / 4
HAND_TO_TCP_Z = 0.1034

_T_LINK8_TCP = np.eye(4)
_T_LINK8_TCP[:3, :3] = Rotation.from_euler('z', LINK8_TO_HAND_YAW).as_matrix()
_T_LINK8_TCP[:3, 3] = [0.0, 0.0, HAND_TO_TCP_Z]


def _dh(a, d, alpha, theta):
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array([[ct, -st, 0, a],
                     [st * ca, ct * ca, -sa, -d * sa],
                     [st * sa, ct * sa, ca, d * ca],
                     [0, 0, 0, 1.0]])


def joint_frames(q):
    """Base-frame transforms of joints 1..7, the flange (link8) and the TCP."""
    q = np.asarray(q, dtype=np.float64).reshape(7)
    T = np.eye(4)
    frames = []
    for (a, d, alpha), theta in zip(DH, q):
        T = T @ _dh(a, d, alpha, theta)
        frames.append(T)
    T = T @ _dh(0.0, FLANGE_D, 0.0, 0.0)
    frames.append(T)
    frames.append(T @ _T_LINK8_TCP)
    return frames


def flange_pose(q):
    return joint_frames(q)[7]


def tcp_pose(q):
    return joint_frames(q)[8]


def matrix(position, quaternion):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(np.asarray(quaternion, dtype=np.float64)).as_matrix()
    T[:3, 3] = position
    return T


def _quat(rotation):
    q = rotation.as_quat()
    return -q if q[3] < 0 else q


def link8_to_tcp(position, quaternion):
    T = matrix(position, quaternion) @ _T_LINK8_TCP
    return T[:3, 3], _quat(Rotation.from_matrix(T[:3, :3]))


def tcp_to_link8(position, quaternion):
    T = matrix(position, quaternion) @ np.linalg.inv(_T_LINK8_TCP)
    return T[:3, 3], _quat(Rotation.from_matrix(T[:3, :3]))
