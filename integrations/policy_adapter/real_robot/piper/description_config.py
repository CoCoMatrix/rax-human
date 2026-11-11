"""Fixed v12 camera mast metadata; never a robot action."""
import json
import math
from pathlib import Path
from .contracts import require


def mast_configuration(explicit_rad: float | None = None) -> dict:
    if explicit_rad is not None:
        require(type(explicit_rad) in (int, float) and math.isfinite(explicit_rad)
                and explicit_rad == 0.0,
                'v12 mast is fixed; additional mast angle must be zero')
    value = json.loads(Path(__file__).with_name('mast_config.json').read_text())
    require(value.get('schema') == 'junqi.piper.mast_setting.v1'
            and value.get('joint_name') == 'global_camera_mast_upper_joint'
            and value.get('joint_type') == 'fixed'
            and value.get('include_in_action14') is False
            and value.get('joint_position_rad') == 0.0
            and value.get('urdf_zero_total_tilt_deg') == 90.0
            and value.get('total_forward_tilt_deg') == 90.0,
            'invalid fixed v12 mast metadata contract')
    require(value.get('calibration_status') == 'nominal_urdf_not_calibrated'
            and value.get('approximate') is True, 'mast estimate cannot claim camera calibration')
    return value
