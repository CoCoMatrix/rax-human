# Astribot S1 control description

`astribot_s1_control.urdf` is the repository-owned upper-body kinematics
snapshot for Astribot S1. It is derived from the vendor SDK asset:

- source: `astribot_config/robot_config/astribot_s1/model/astribot_whole_body_with_head.urdf`
- source SHA-256: `da901b5af64b68280cbe67158a49ee2027bc2f1b0548bdbc71b64e8b77a56e23`
- repository asset SHA-256: `c3acb25201fc8acf0b591470c9e6f4a8abea63810784a95b8ffb356834a6268e`
- vendor license: BSD-3-Clause; see `LICENSE`

The repository asset preserves every link, joint, origin, axis, limit and
inertial element. Visual and collision elements were removed because the vendor
mesh tree is not part of this runtime snapshot. This is therefore a kinematics
asset, not a collision-qualified or visual simulation model.

The URDF contains the four torso joints, two head joints and two seven-joint
arms. The planar chassis and both grippers are present in the XPolicyLab 25D
state/action contract but are not modeled by this vendor URDF; they remain
SDK-owned parts and are declared as such in `profile.yml`.

Control indexing is explicit:

- `astribot_torso.joint_0..3` maps to `astribot_torso_joint_1..4`;
- `astribot_head.joint_0..1` maps to `astribot_head_joint_1..2`;
- each `astribot_arm_{left,right}.joint_0..6` maps to vendor joint `1..7`.

Do not silently regenerate this file when the SDK changes. A replacement
requires an explicit source diff and renewed joint, limit, endpoint and FK
validation.
