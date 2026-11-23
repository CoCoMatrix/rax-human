# GR3 control description

`gr3_control.urdf` is the repository-owned kinematics snapshot used by GR3
FK/IK and Predictive Mink. It is derived from:

- package: `fourier-robot-descriptions==0.2.7`
- source asset: `assets/urdf/gr3_fourier_hand_6dof.urdf`
- source SHA-256: `2b87382bbbda834260526b00728e2160529a3cd693267417d3e56683c87dcc9a`
- repository asset SHA-256: `0def579752830df3c86d627b6009c02b613fb9b3718a3f1c9694acd62abe58aa`
- license: Apache-2.0; see `LICENSE`

The repository asset preserves every link, joint, origin, axis, limit, mimic
relationship and inertial element. Visual and collision elements were removed
because their referenced mesh package is not part of the runtime contract.
Consequently this file is suitable for kinematics and the existing Predictive
Mink projector, but it is not a collision-qualified or visual simulation asset.

The controlled GR3 action is the reviewed 31-joint subset declared by
`mapping.JOINT_NAMES`; lower-body and mimic joints remain in the URDF so the
model topology and fixed lower-body configuration are unchanged.

Do not replace this file during a dependency update. A replacement requires an
explicit source diff plus tests for joint membership, limits, end-effector FK
and the GR3 profile contract.
