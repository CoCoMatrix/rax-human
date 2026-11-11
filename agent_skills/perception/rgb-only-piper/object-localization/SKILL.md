---
name: object-localization
description: Use head and wrist RGB without pretending to have metric depth.
---

Call `piper_observe(require_depth=false)` for head/left/right RGB. Use visible relationships, occlusion and before/after views to reason about objects. RGB-only frames cannot be passed to metric `piper_pixel_to_position`. Do not infer metric camera coordinates or wrist extrinsics from image pixels. Known CAD FK gives flange pose, not object depth or calibrated grasp TCP.
