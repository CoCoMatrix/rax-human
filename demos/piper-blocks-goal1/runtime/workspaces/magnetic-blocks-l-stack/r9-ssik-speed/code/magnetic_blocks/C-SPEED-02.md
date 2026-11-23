# C-speed0.2 — bounded continuity and25mm/s jaw

Independent from C-speed0.1 and C034. SDK80%,600mm per-call translation,120deg per-joint step,0.1rad moving following error and0.01rad arrival/binding remain. No hardware motion in acceptance.

## Recorded cause
2026-11-13 16:02 C034 run: open-to-hover8.411s; orange close-to-lift4.040s; green close-to-lift3.787s. Explicit curve preview plus repeated straight-leg and native curve validation dominated. Model calls0. Max contemporaneous left-joint command/feedback error0.022189rad in2039 samples, at50%, NOT80% motor qualification.

## Changes
- Explicit speed25 jaw profile:25mm/s maximum,20mm/s2 retained acceleration, integrated cosine triangular/cruise profile; short strokes no longer fall back to10mm/s. Force, completion/contact interval and arm fixation unchanged.
- Blended requests omit redundant straight-leg checking in frontend. Actual float32 curve with native collision/workspace checks still required in bridge before staging; fresh owner/anchor guards retained. Nonblended checks unchanged.
- Batched FK checks every original curve endpoint and midpoint; no sample removal.
- Harmonic interior tangent factor0.3 to0.6, automatically reduced to preserve0.01rad curve departure; retain velocity/acceleration and native collision checks. Reversal can still require zero speed.
- Terminal dwell contributes fresh feedback to the same0.3s/4sample rest window. Every SDK receipt is required before completion. Not a shorter rest window.
- Runtime multiwaypoint endpoint cap corrected from stale60deg to selected120deg.
- Only nominal3mm finger/task-block clearance accepts1mm model tolerance (minimum2mm), synchronized in explicit geometry and annotated native environment check. Other nominal margins retain previous tolerance; table/payload/self/right-arm rules unchanged.

## Offline evidence
20 full-task plans including six permutations;533 regression tests+52 subtests; 16 opening/closing stroke wire cases;0.099rad progress error accepted,0.101rad rejected; production arrival/binding remain0.01rad.
Same80% recorded paths: pre-dispatch compute orange-hover5.659 to4.367s, orange-lift2.411 to1.869s, green-hover1.875 to1.431s, green-lift1.586 to1.232s. Descriptive offline timing, two native curve validations retained; not live SDK gaps. Jaw68mm5.767 to4.7s, actual wire peak24.849mm/s. Live physical following still requires a supervised run.

## Reference
[MoveIt Pilz sequence blending](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html): sequence waypoints and explicit blend boundaries. [Ruckig](https://github.com/pantor/ruckig): state/velocity/acceleration constraints; intermediate-waypoint Community implementation uses cloud, so not added to control path. We retain existing local quintic/native pipeline, no framework migration.

## Remaining limits
Closure/release and evidence-dependent endpoints remain stops. No claim that80% hardware following has passed from50% logs or offline wire tests. New start scene is measured per run. Faults stop and require inspection; never blind replay.
