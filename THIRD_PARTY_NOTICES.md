# Sources and licensing boundaries

- RAX: https://github.com/robot-agent-exploration/rax, real/franka lineage and additional skill-library provenance in `UPSTREAM.json`. Apache License 2.0 is retained in root `LICENSE`. Existing notices remain in source files.
- `integrations/policy_adapter`: exported GPT-as-Policy integration; its MIT license and third-party notices remain alongside the code. Root Apache text does not replace that license.
- `integrations/xpolicy_owner`: exported project-specific XPolicy owner/device source used by this deployment. This is a source subset, not a release of all XPolicy policy implementations, SDK binaries, model checkpoints or datasets. Original file notices are retained. No new license grant is asserted for externally owned dependencies.
- `integrations/legacy_piper` and `integrations/moveit`: deployment support source/assets. Historical path references and original provenance are retained where useful. Meshes/model assets must be used under their original applicable rights; this publication does not grant rights to manufacturer firmware or SDK assets.
- Fast-FoundationStereo, ROS/MoveIt, pyAgxArm, RealSense, Dora and Python dependencies are installed separately under their own licenses. No model weights, firmware, vendor SDK binaries or virtual environments are included.

The root license accompanies upstream RAX-derived code. For files with separate notices, those notices govern. This repository is an engineering reproduction package, not an endorsement from the upstream authors or robot manufacturer.
