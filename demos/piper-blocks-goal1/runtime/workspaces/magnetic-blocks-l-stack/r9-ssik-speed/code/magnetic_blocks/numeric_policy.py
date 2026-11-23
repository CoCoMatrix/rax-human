"""Task measurement tolerances; not joint limits, evidence identity or timeouts."""
import math
from task_spec import HIGH_COLORS
EXTRA_GEOMETRY_ERROR_M=.001
GEOMETRY_ERROR_M=.0015
CORRIDOR_ERROR_M=.002
LOCALIZATION_ERROR_M=.003
LAYOUT_ERROR_M=.002
OPEN_JAW_ERROR_M=.0005
NOMINAL_DOWN_COS=.995
DOWN_COS=math.cos(math.acos(NOMINAL_DOWN_COS)+math.radians(1.))
TASK_ORIENTATION_ERROR_DEG=.5

def task_orientation_limit_deg(color):
    """Shared signed tilt boundary: lower moves outwards, upper upwards.

    Tilt magnitude is nonnegative; +/-25.1deg therefore becomes +/-25.6deg
    for yellow. This does not alter CAD joint limits or grasp pose candidates.
    """
    base=25.1 if color in HIGH_COLORS else math.degrees(math.acos(DOWN_COS))  # third-layer block (yellow in the default goal)
    return base,base+TASK_ORIENTATION_ERROR_DEG

def clearance_minimum(nominal):
    return nominal-GEOMETRY_ERROR_M if nominal>=.003-1e-12 else nominal

def policy_record():
    return {'model_gap_error_m':GEOMETRY_ERROR_M,'contact_corridor_error_m':CORRIDOR_ERROR_M,
            'localization_error_m':LOCALIZATION_ERROR_M,'task_layout_error_m':LAYOUT_ERROR_M,
            'open_jaw_error_m':OPEN_JAW_ERROR_M,'down_angle_extra_deg':1.,
            'task_orientation_comparison_error_deg':TASK_ORIENTATION_ERROR_DEG,
            'unchanged':'joint/workspace/speed/stop limits, table penetration, receipt identity, freshness, native self-collision'}
