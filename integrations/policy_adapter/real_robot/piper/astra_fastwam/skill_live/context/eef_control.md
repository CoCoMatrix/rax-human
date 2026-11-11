# PiPER action and correction contract

- State/action: left joints1..6 [rad], left gripper width [m], right joints1..6
  [rad], right gripper width [m]. Absolute commands, never deltas or ARX booleans.
- Model: H32, 30Hz; student chooses original prefix1..15; edit/eef1..5.
- Frame: dual_base_link. left_arm names left_link6; right_arm names right_link6.
  These are flange frames, not bottle centers or calibrated grasp TCPs.
- Poses: position_m and unit quaternion_wxyz. Corrections require both arms.
- edit: delta_position_m and delta_rotation_vector_rad, common-base translation
  and left-applied rotation; max Euclidean norm5cm and0.35rad. Zero edit follows
  that arm's student trajectory; it does not freeze the arm.
- eef: absolute flange pose within5cm/0.35rad of current measured flange.
- gripper: "keep" retains current MEASURED width; otherwise {"width_m":number}.
- To keep an arm still use its measured flange pose with measured width, rather
  than assuming a zero edit freezes it.
- Echo review binding exactly, including observation_timestamp_ns as a decimal
  STRING (tool serialization precision). Set unused steps/frame/edit/target to null.

The host first validates an offline correction preview, then recomputes each
actual correction from fresh measured feedback after the preceding owner ACK.
Sending occurs only through the authorized formal XPolicy relay/owner. There is
no extra waypoint planner or independent hardware writer.
