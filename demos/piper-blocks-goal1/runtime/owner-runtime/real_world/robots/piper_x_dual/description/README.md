# Dual Piper X Mink description

`dual_piper_x_mink.urdf` is the repository-owned FK/IK asset for the rigidly
mounted dual Piper X robot.

It was originally imported from the development asset
`pyAgxArm/dual_piper_x_description/urdf/dual_piper_x.urdf`; the control SDK is
now pinned separately at `third_party/robot_sdks/pyagxarm`. The current
repository derivative identity is:

```text
SHA256 9f6dfedb85f2213e05567d6324fbd1f23ffe1e42b6f30f062422d3a6c462b395
```

The source uses the MIT license retained in this directory. Its kinematic
transforms, inertial values and joint axes are preserved. Joint ranges are the
exact `ROBOT_JOINT_LIMIT_PRESET_RAD["piper_x"]` values from the pinned SDK, so
the harness does not narrow the vendor-supported range. The ROS `package://`
visual and collision meshes are intentionally removed so the model can be
loaded directly by MuJoCo/Mink without a ROS package or 28 MB of rendering
assets.

Consequences:

- the two base frames remain at `Y=+0.30 m` and `Y=-0.30 m`;
- both arms retain the same orientation and are not mirrored;
- `left_link6` and `right_link6` are the current IK endpoint frames;
- this asset is suitable for FK/IK only;
- it is not a collision, dynamics, visualization or gripper/TCP model.

A reviewed gripper/TCP fixed transform must be added before claiming that the
link-6 frame is the actual grasp frame. Collision qualification requires a
separate geometry-bearing asset.
