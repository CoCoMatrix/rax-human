# v0.2.5: scene-derived hover and pickup clearance

Failure: web-main-20261112-113605-7e9a65 finished the orange base placement,
release, retreat and two inspection motions. green-hover-inputs failed in 0.154s
on native full-pose IK, before green movement. Total main time 151.74s;
Gemini calls 0; native Stop/rest confirmed. This was not a network wait.

The target [0.335602,-0.267425,0.273256] m used a mandatory top+195mm.
At this XY and retained down orientation, the native solver rejected it.
Top+190mm solves but the full finger envelope fails the existing20mm allowance.
Top+190.5mm passes native IK/FCL/workspace and all sampled finger/scene checks.
The successful candidate is close to J4's original89-degree limit. No limit,
collision margin, state tolerance, speed or grasp opening was changed.

## Adaptive rules

- Hover:195mm is a preference. Search the existing180..240mm corridor, ordered
  nearest the preference, at0.5mm numerical resolution, maximum121 unique
  read-only candidates. Current measured green XY and orientation stay fixed.
  Native refusal may select a different target; stale evidence, changed owner,
  ambiguous transport or other unknown errors stop. Every chosen path uses
  the real GreenPregrasp checker; execution independently rechecks current state.
  Planner and executor share one corridor constant.
- First green raise: do not exceed the actual successful same-run pickup hover.
  Trace completed pickup reports and native receipts. This avoids a fixed55mm
  raise back into the same unreachable posture. It is a preferred-target cap,
  not a claim that the loaded trajectory is clear; LoadedChecks still checks it.
- Insertion segmentation: derive count from actual vertical distance and the
  native400mm displacement bound, instead of rejecting above six20mm segments.
  Keep the existing contact-entry and measured execution checks.

## Other rule audit and limits

Different types of numbers have different roles. Object-local contact height
128mm, open gap and close target encode this60mm block/tool setup, not arbitrary
workspace barriers. They remain task-specific measured experience. Native joint
limits, full-pose IK, workspace and collision requirements remain mandatory.
Observation offsets100mm, the two orientation transitions and the obstacle-side
transfer routing still encode the rehearsed strategy. They already use this run's
object coordinates and retain execution checks, but this change does NOT qualify
arbitrary layouts, rotated blocks or arbitrary reference-image structures. Future
candidate alternatives there must check their full path, not merely remove gates.

## Validation

161 offline tests passed (including several feasible hover heights, bounded
exhaustion, collision refusal and no retry on ambiguous control). Real failed
scene replay chooses190.5mm with18 read-only attempts. Native prospective
insertion/trial/raise probes pass; the old fixed first raise is independently
IK-infeasible. These probes are not physical receipts or proof of grasp retention.
No task motion is issued during validation. Original v0.2.4 and frozen versions
remain intact. All runtime evidence is regular files; native memory references
and launcher dry-run are checked before Web deployment.
