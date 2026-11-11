---
name: learned-blue-cup-wall-grasp
description: Historical conditional wall-pinch grasp of this blue cup, reported by operator.
---
# Blue cup wall grasp

- Applies when: the same cup wall is visibly between opposing pads below the rim; handle clear; cup supported initially.
- Avoid when: pads are outside the wall, touching only the lip, off-center, or the cup geometry/installed pads differ.
- Executed action: historical closure target0.004m full jaw gap; opening about0.019m. Tool name below is the current equivalent, not a claim it executed in this new session.
- Pose recipe: reacquire the visible cup wall in current head RGB-D and check pad alignment in wrist RGB; no historical world pose is supplied. Historical110mm contact-center offset was an estimate, not a calibrated TCP; CAD nominal137.29mm TCP also is not measured. Do not silently substitute either for validated contact geometry.
- Actions: approach with clearance; ensure wall enters pads; close incrementally toward4mm; make a small initial lift and observe; increase lift only with clear retained contact and clearance.
- Expected effect: cup follows the gripper without sliding; measured loaded gap may exceed4mm (previous successful run about4.7mm).
- Recovery: if empty grasp, keep cup supported, release and reacquire alignment. Do not keep tightening based only on the historical4mm value. Unknown execution is not replayed.
