"""Camera extrinsics in the robot base frame (fr3_link0).

Fixed cameras store T_cam_world (P_cam = T @ P_base). The wrist ZED-M stores
T_cam_gripper against `fr3_hand_tcp` (checked by projecting the FK fingertips: the TCP
reading lands on the fingers, link8 misses by ~25 cm).

The cameras are the backend option `cameras` (see examples/franka.json): one entry per
ZED with its name, serial, mount ('fixed' or 'tcp'), optional depth_min_m, and its
extrinsics YAML. A relative extrinsics path resolves under $FRANKA_CALIBRATION_DIR.
"""
from dataclasses import dataclass
import os
from pathlib import Path

import numpy as np
import yaml

CALIBRATION_ENVIRONMENT = 'FRANKA_CALIBRATION_DIR'


def calibration_dir():
    value = os.environ.get(CALIBRATION_ENVIRONMENT)
    return Path(value).expanduser() if value else None


def camera_specs(options):
    cameras = options.get('cameras')
    if not cameras:
        raise ValueError("franka backend option 'cameras' is required: list each ZED camera's name, serial, "
                         "mount and extrinsics file (see examples/franka.json)")
    return cameras


def extrinsics_path(spec):
    path = Path(spec['extrinsics']).expanduser()
    if path.is_absolute():
        return path
    base = calibration_dir()
    if base is None:
        raise ValueError(f"camera {spec['name']}: the relative extrinsics path {path} needs "
                         f"{CALIBRATION_ENVIRONMENT} (or give an absolute path)")
    return base/path


def load_matrix(path):
    with open(path) as handle:
        matrix = np.asarray(yaml.safe_load(handle)['matrix'], dtype=np.float64)
    if matrix.shape != (4, 4) or not np.allclose(matrix[3], [0, 0, 0, 1]):
        raise ValueError(f'{path}: expected a 4x4 homogeneous matrix')
    return matrix


@dataclass(frozen=True)
class CameraModel:
    name: str
    serial: int
    mount: str            # 'fixed' (T_cam_base) or 'tcp' (T_cam_tcp, eye in hand)
    matrix: np.ndarray

    @classmethod
    def from_spec(cls, spec):
        if spec['mount'] not in ('fixed', 'tcp'):
            raise ValueError(f"camera {spec['name']}: mount must be 'fixed' or 'tcp'")
        return cls(spec['name'], int(spec['serial']), spec['mount'], load_matrix(extrinsics_path(spec)))

    def base_from_camera(self, T_base_tcp=None):
        """T_base_cam: the camera's pose in the base frame at the given TCP pose."""
        if self.mount == 'fixed':
            return np.linalg.inv(self.matrix)
        if T_base_tcp is None:
            raise ValueError(f'{self.name} is eye-in-hand; its pose needs the TCP pose')
        return np.asarray(T_base_tcp) @ np.linalg.inv(self.matrix)


def backproject(u, v, depth_m, K):
    """Pixel + metric depth -> camera-frame point (pinhole, rectified left image)."""
    return np.array([(u - K[0][2]) / K[0][0] * depth_m, (v - K[1][2]) / K[1][1] * depth_m, depth_m])


def project(point_base, T_base_cam, K):
    p = np.linalg.inv(T_base_cam) @ np.append(np.asarray(point_base, dtype=np.float64), 1.0)
    if p[2] <= 0:
        return None
    uv = np.asarray(K) @ p[:3]
    return (uv[:2] / uv[2]).tolist()
