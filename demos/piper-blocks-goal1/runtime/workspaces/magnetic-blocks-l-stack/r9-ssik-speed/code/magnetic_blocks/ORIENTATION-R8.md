# c-Four1.0 orientation r8

Independent candidate from preflight-r7. Original/frozen versions unchanged. No live motion during repair.

## Root cause and authorized policy
Run20261117-093926 stopped before yellow lift-transfer. FK of measured joints yielded25.121782deg tilt, while the planned tilt was25deg. The old25.1deg absolute limit rejected loaded motion; stationary jaw strokes had a separate1deg planned-reference exception, so closure had passed. Maximum measured joint difference0.001212rad was already within the unchanged0.01rad binding tolerance.

Operator authorizes0.5deg comparison allowance on both sides of the task tilt boundary. A signed lower bound moves outward, not upward toward zero: yellow +/-25.1deg becomes +/-25.6deg. Other colors' existing near-down bound6.731968deg becomes7.231968deg. The nonnegative tilt magnitude is compared with that absolute limit.

## Implementation and active-flow audit
numeric_policy.task_orientation_limit_deg owns the common boundary. assembly_plan.orientation_evidence reports nominal limit, accepted limit, allowance and signed bounds. Its former jaw-only reference exception is removed; the optional legacy reference argument cannot increase the common limit. Finite FK matrices are required. No new grasp angle candidates are introduced.

All active main paths use that helper: entry/contact paths; qualitative pre-close alignment; full jaw close/open; loaded pickup; straight transfer; blended wire plus midpoint validation; standalone batch fallback; loaded lowering; release; contact retreat. Reference14 binding still validates owner/state proximity for stationary jaw actions, but no longer grants a different angular limit or requires a second reference FK. Released lateral retreat keeps its existing geometry-based checks.

Legacy controller/continuous_approach/green_checks/manipulation_checks contain earlier rehearsal-specific guards, but the active assembly_controller.main does not enter those old movement functions. They remain preserved; composition.initial_state is the only initial entry reused. DOWN_COS tests in route generation only rank/add candidate portals and are not another active rejection bound. Receipt binding0.01rad, moving follow error0.1rad, mechanical joint limits, workspace, speeds, collision margins, table and payload checks are unchanged.

## Offline evidence
Same failed record, production Frontend/Session/Unix RPC/bubblewrap and real FK/IK/native path checker: r7 reproduces the exact rejection after closure; r8 passes close,173-sample blended lift+two transfers (plus midpoint checks), standalone rebound pickup, subsequent predicted lower/release/retreat. Uses actual51.3mm closed gap, measured joints, completed placements and grasp-derived payload; no image time falsification. Replay2.070s. Future endpoints are predictions, not fake executed receipts.

Full preflight of this run's initial measured scene passes in19.710s with35 native-certified actions; five batch preview checks pass.67 selected tests+4 subtests pass on host and in production isolation, including both signs/all colors at the exact0.5deg boundary and refusal beyond it with or without a jaw reference. Undefined-name scan clean. No owner access or motion tools allowed in replay; hardware dispatches0.

Offline qualification does not establish physical placement success. No autonomous recovery or replay of the interrupted held-object task is added. The existing Prepare supervisor is preserved across the Web-only restart and adopted read-only; no hardware command is dispatched.
