"""Operator-selected SI motion bounds. Selection is not physical qualification."""
import math

# Site observation + operator confirmation on 2026-10-29: 75 mm command
# arrived at 74.6 mm; 84 mm timed out at physical maximum 75.6 mm.
# This is the usable command range, not a change to nominal URDF geometry.
LEFT_JAW_COMMAND_MAX_M=.075
LEFT_JAW_ARRIVAL_TOLERANCE_M=.001

TASK60 = dict(max_joint_displacement_rad=math.radians(60),
    max_joint_velocity_rad_s=math.radians(10), max_joint_acceleration_rad_s2=math.radians(20),
    max_eef_translation_m=.1, terminal_dwell_s=.2, joint_tolerance_rad=.003,
    max_left_jaw_step_m=.01,tracking_velocity_ratio=.85)


def transport_options(limits):
    return dict(max_displacement_rad=limits['max_joint_displacement_rad'],
        max_velocity_rad_s=limits['max_joint_velocity_rad_s'],
        max_acceleration_rad_s2=limits['max_joint_acceleration_rad_s2'],
        terminal_dwell_s=limits['terminal_dwell_s'],joint_tolerance_rad=limits['joint_tolerance_rad'],
        tracking_velocity_ratio=limits.get('tracking_velocity_ratio',1.))


def movement_duration(delta, velocity, acceleration):
    """Quintic peak derivatives, rounded UP to the actual 30 Hz wire duration."""
    joint=max(abs(delta[i]) for i in (0,1,2,3,4,5,7,8,9,10,11,12))
    seconds=max(1., 1.875*joint/velocity, math.sqrt((10/math.sqrt(3))*joint/acceleration),
        1.875*abs(delta[6])/.01)
    return math.ceil(seconds*30)/30
