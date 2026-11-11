"""Shared camera-only extrinsics for GPT-as-Policy and GPT-Policy. No hardware."""
import copy
import json
from pathlib import Path
import numpy as np
from .contracts import require
from .description import URDF_SHA256
from .stereo_geometry import validate_se3
from .export_stereo_calibration import canonical_sha256

ACTIVE_HEAD = Path('/home/ubuntu/code_junqi/shared/piper/head_camera_extrinsics.json')
PARTIAL_STATUS = 'table_plane_tilt_height_corrected_xy_yaw_nominal'
ALLOWED_STATUSES = {'nominal_urdf_not_calibrated', PARTIAL_STATUS}

def validate_head_geometry(value):
    require(value.get('schema') == 'junqi.piper.tabletop_geometry.v1'
            and value.get('camera_role') == 'camera_head'
            and value.get('parent_frame') == 'dual_base_link'
            and value.get('child_frame') == 'global_d435_color_optical_frame'
            and value.get('urdf_sha256') == URDF_SHA256
            and value.get('calibration_status') in ALLOWED_STATUSES
            and value.get('physical_acceptance') is False,
            'head geometry identity/status mismatch')
    require(value.get('transform_convention') == 'p_base = T_base_camera @ p_camera_homogeneous',
            'head transform convention mismatch')
    validate_se3(value['T_base_camera'])
    if value['calibration_status'] == PARTIAL_STATUS:
        correction=value.get('correction',{})
        require(correction.get('method') == 'table_plane_normal_and_measured_height'
                and correction.get('full_6dof_calibrated') is False
                and correction.get('xy_yaw_source') == 'nominal_urdf_prior'
                and len(correction.get('source_samples',[])) >= 3,
                'partial head correction provenance missing')
    return value

def load_head_geometry(path=ACTIVE_HEAD):
    return validate_head_geometry(json.loads(Path(path).read_text()))

def apply_head_geometry(context, geometry=None):
    if geometry is None and not ACTIVE_HEAD.exists():
        return context
    value=load_head_geometry() if geometry is None else validate_head_geometry(geometry)
    result=copy.deepcopy(context)
    from scipy.spatial.transform import Rotation
    t=np.asarray(value['T_base_camera'])
    camera=result['rgb_optical_frames']['camera_head']
    camera.update(transform=t.tolist(),position_m=t[:3,3].tolist(),
        quaternion_wxyz=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]].tolist(),
        calibration_status=value['calibration_status'],source=str(ACTIVE_HEAD),
        head_geometry_sha256=canonical_sha256(value))
    result['active_head_geometry_sha256']=canonical_sha256(value)
    result['head_extrinsics_status']=value['calibration_status']
    return result
