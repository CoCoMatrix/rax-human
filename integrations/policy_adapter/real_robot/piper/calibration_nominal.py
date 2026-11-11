"""Export the v12 head-camera pose as an estimate, never as a measured calibration."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from .contracts import _sha256, require
from .description import URDF_RELATIVE_PATH, URDF_SHA256, MAST_JOINT, _joint_map
from .description_config import mast_configuration


def _rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    require(np.isfinite(axis).all() and np.linalg.norm(axis) > 0, 'invalid rotation axis')
    x, y, z = axis / np.linalg.norm(axis)
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def head_estimate(asset_root, mast=None):
    path = Path(asset_root) / URDF_RELATIVE_PATH
    require(_sha256(path) == URDF_SHA256, 'unexpected description URDF identity')
    setting = mast_configuration() if mast is None else mast
    require(setting.get('schema') == 'junqi.piper.mast_setting.v1'
            and setting.get('approximate') is True
            and setting.get('include_in_action14') is False, 'invalid mast metadata')
    q = setting.get('joint_position_rad')
    require(type(q) in (int, float) and np.isfinite(q), 'head estimate requires an explicit mast angle')
    require(q == 0.0 and setting.get('joint_type') == 'fixed', 'invalid mast joint: v12 is fixed')
    total = setting.get('total_forward_tilt_deg')
    require(type(total) in (int, float) and np.isfinite(total)
            and abs(total - 90.0) < 1e-7
            and setting.get('calibration_status') == 'nominal_urdf_not_calibrated',
            'mast estimate angle or calibration status inconsistent')
    root = ET.parse(path).getroot()
    joints = _joint_map(root)
    by_child = {j['child']: j for j in joints.values()}
    frame = 'global_d435_color_optical_frame'
    chain, seen = [], set()
    while frame != 'dual_base_link':
        require(frame not in seen and frame in by_child, 'missing or cyclic head-camera chain')
        seen.add(frame)
        joint = by_child[frame]
        chain.append(joint)
        frame = joint['parent']
    transform = np.eye(4)
    for joint in reversed(chain):
        origin = np.eye(4)
        r, p, y = joint['rpy_rad']
        origin[:3, :3] = _rotation([0, 0, 1], y) @ _rotation([0, 1, 0], p) @ _rotation([1, 0, 0], r)
        origin[:3, 3] = joint['xyz_m']
        motion = np.eye(4)
        if joint['name'] == MAST_JOINT:
            require(joint['type'] == 'fixed' and q == 0.0
                    and abs(joint['rpy_rad'][1] - np.radians(90.0)) < 1e-10, 'invalid mast joint')
            motion[:3, :3] = _rotation(joint['axis'], q)
        else:
            require(joint['type'] == 'fixed', 'unconfigured movable joint in head-camera chain')
        transform = transform @ origin @ motion
    return {'schema': 'junqi.piper.tabletop_geometry.v1',
            'source': 'fixed_v12_urdf_user_geometry',
            'calibration_status': 'nominal_urdf_not_calibrated',
            'physical_acceptance': False, 'hardware_access': False,
            'urdf_sha256': URDF_SHA256, 'mast_configuration': setting,
            'camera_role': 'camera_head', 'parent_frame': 'dual_base_link',
            'child_frame': 'global_d435_color_optical_frame',
            'transform_convention': 'p_base = T_base_camera @ p_camera_homogeneous',
            'translation_unit': 'm', 'T_base_camera': transform.tolist(),
            'camera_forward_axis_in_base': transform[:3, 2].tolist(),
            'camera_optical_axes': 'x_right_y_down_z_forward',
            'table_plane_in_base': None,
            'tcp': {side: {'parent_frame': f'{side}_link6', 'position_m': None,
                           'orientation': None, 'status': 'not_measured'} for side in ('left', 'right')},
            'usage': 'geometry_preview_only; image pixels require actual intrinsics and depth or a measured plane',
            'motion_execution_authorized_by_this_file': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description-assets', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = head_estimate(args.description_assets)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({'output': str(args.output), 'status': result['calibration_status']}))


if __name__ == '__main__':
    main()
