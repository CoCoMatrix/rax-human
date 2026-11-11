# Blue cup to tabletop round coaster — editable exploration

The target in task-target.json is blue-cup-to-table-coaster. The coaster ROI means
the round coaster on the table. This candidate is not a frozen controller.
At a stationary checkpoint with known execution, revise one primitive as needed.
A revision invalidates the old visual decision and requires a new observation;
it never erases executed actions. Mixed-version exploration is not fixed-version validation.

## Next bounded step
Use python code/cup_to_coaster/round_step.py with no arguments to observe.
Otherwise pass ACTION PARAMETERS_JSON CHECKS_JSON EVIDENCE_NOTE.
Wait for completion, parse JSON, and display EVERY review_frames image in the SAME
tool call. Use the returned decision_contract and allowed_next_actions directly.
Post-action views are the next checkpoint while valid. Do not add status/source/
progress/observe round trips unless something is missing, expired, changed or unclear.
Complete analysis, code edits and necessary memory updates before taking action views.

PARAMETERS includes current target_rois: normalized [cx,cy,width,height], e.g.
{"target_rois":{"head":{"cup":[cx,cy,w,h],"coaster":[cx,cy,w,h]},"left":{"cup":[cx,cy,w,h]}}}.
Include only visible objects/views; carry, lower, release and finish need cup and coaster.
CHECKS explicitly states scene_clear and the required observed facts; unknown is not yes.
Supply a short evidence note grounded in the actual head/wrist images.

| Action | Required visual facts | Existing recipe |
|---|---|---|
| open | cup_supported; clear opening sweep | 19mm, <=9mm per segment |
| approach | cup_supported | top-down standoff >=195mm; bounded segments |
| adjust | safe_path | small reviewed alignment/descent |
| close | rim_aligned | pads straddle wall below rim; 4mm target |
| trial_lift | rim_aligned, cup_supported_before_lift | 4mm command, measured gap <=5.5mm; nominal40mm |
| carry | cup_attached_upright and clearance/retention evidence below | measured lift >=38mm; clear path |
| lower | over_support | <=25mm vertical steps over this coaster |
| release | cup_supported | entire cup base supported on this coaster, then open19mm |
| retreat | cup_released | jaw >=18mm; clear path away from stable cup |
| finish | cup_on_coaster, cup_stable, arm_clear | fresh head/wrist evidence after retreat |

Only empty Prepare can use open parameter clear_space_sequence:true, with explicit
empty_jaw_confirmed and jaw_sweep_clear. It chains at most three existing bounded
segments, validates every receipt, and stops on any fault; inspect final views.
Contact, lift, transport, release and retreat retain meaningful visual checkpoints.

## Geometry only for the next primitive
The tabletop-coaster target does not require shelf_preflight or rack-edge heights.
Check actual transport clearance and full coaster support. Do not substitute another target.
For geometry analysis, geometry_batch.py REQUEST_JSON_PATH uses one original FFS frame.
Fields: snapshot, cup_roi, surface_rois (table/coaster as relevant), optional patch_rois.
fit_ring defaults false. Whole-rim fitting is optional, not a startup prerequisite.
A failed global fit requires a different supported hypothesis, not repeated fitting.
The saved geometry-analysis-latest result is diagnostic, never motion authorization.
For raw_pixels approach, supply three exact valid current head rim_pixels and
geometry_mode:"raw_pixels". No nearest-point substitution, filled depth or old coordinates.
cup_rim_model is optional only with sufficient raw planar rim support; provide
grasp_angle_deg and actually observed cup_upright/rim_model_reviewed checks.
Angles are in left-base XY (0=+X,90=+Y); consult cup_geometry.py documentation only if
this particular hypothesis is needed. Existing fit, mixed-face and identity checks remain.
The existing approach accepts top-down yaw only; other orientations need a reviewed
new primitive. A solved IK pose does not prove collision-free physical travel.
delta:[dx,dy,dz] is in left_base_link metres for adjust/lift/carry/lower/retreat.
MoveIt2 KDL and original FFS source points remain the live geometry/IK route.
The camera calibration and 110mm installation-plane contact-centre offset remain provisional.
SAM3 runs initially for cup/coaster only. Later views use fresh raw images; initial
masks/native associations become historical after scene movement.

## Retention, receipts and freshness
Display all paired before/after head/wrist images after trial lift. Verify upright cup,
stable cup-to-pad geometry, no visible slip and the cup following the lift. Measured
flange displacement alone proves none of these. With measured >=38mm rise from a
supported start, paired retention evidence may establish clearance despite base occlusion.
For carry use cup_base_clear, or all wrist_retention_stable/cup_followed_lift/no_visible_slip,
plus cup_attached_upright, only when supported by images. Contradictory views fail.
For another lift use current cup_attached_upright; never claim the raised cup is supported.

Keep150s review and2s pre-dispatch source gates, current scene/anchor/right-arm checks
and all qualified hardware limits. Never renew timestamps or widen limits.
executed:true and measured_arrival:true remains executed if post-observation fails:
observe only, preserving before-lift evidence. Unknown execution or pending_dispatch
blocks further motion until reconciliation. Structured pre-dispatch rejection means
no motion; error text alone does not. Never repeat an unknown or known-arrived action.
LEFT arm only, preserve right arm/jaw; MCP only, sole owner, no direct SDK/legacy IK.
No automatic fault reset or rearm. Use Hold/help for genuine control/contact uncertainty.

## Memory and completion
The controller journals receipts and memory/AUTO_STATE.json automatically. Use its
current contract instead of rewriting the global plan. TIPS records new reusable
lessons; RUNS records outcomes. Notes and old operator replies are historical.
After compaction use injected continuation; context_entry is fallback, not a reason
to reread startup or all code. Read implementation only for a specific unresolved fault.
Only finish produces a real round-result.json after full support, release and retreat.
Complete assisted or mixed-version exploration is learning; it does not establish
unchanged-version autonomous reliability. Freeze follows full-task evidence.
