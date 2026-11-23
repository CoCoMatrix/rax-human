---
name: robot-interface
description: Piper tools for the installed dual-arm machine, with one task arm and three cameras.
---

# Piper interface

Read `piper_status` first (or the robot_status already included in a current `status`/snapshot; do not repeat the same query). `NO_SEND` is read-only: setting a desired goal or asking for IK never starts hardware. Live operation requires an existing XPolicy sole owner and operator arming. Do not start a second SDK/CAN writer or automatically enable/rearm the owner.

`motion_ready` accepts a known idle bridge in either IDLE or DONE; it is not a scene-safety permit. `state_binding` on an observation reports the measured pose bracket and rest consistency; the cameras are not claimed to be hardware synchronized. A receipt's nested `observation.wall_budget` matches its top-level budget. Read-only errors carry `request_no_action_executed=true` without clearing an earlier `control_uncertain`.

Inspection projections and the owner collision checker share the same pinned LEFT finger envelopes, extended to link6 +Z170mm, with the original transverse CAD bounds. `piper_check_path(start14,target14,obstacles=[])` checks the sampled joint/jaw path without dispatch. Its optional axis-aligned obstacle boxes use **dual_base_link**, whereas the pose tools use **left_base_link**. Empty environment means self-collision only; neither a clear CAD path nor approximate head calibration certifies table, cable, payload or cup clearance. Do not upgrade operator-measured geometry to independent physical validation.

Use geometry/feedback for numeric pose, joint/jaw arrival and modeled collision checks. Use combined head/wrist images for the visible task facts needed at this boundary: cup identity/orientation, visible obstacles, pad alignment, retention and support. Do not require RGB to prove millimetre clearance over every hidden surface or absence of transient contact from two endpoint images. Required contact/support facts that remain occluded still block their dependent action; irrelevant occlusions do not add new requirements. Use the recording to investigate suspected intervening disturbances. A pre-motion onsite confirmation applies only to its recorded owner, pose and stated scope; it is not a post-motion fact.

An observation is not automatically an auxiliary-model request. Use measured
feedback for arrival and geometry tools for modeled collision; invoke auxiliary
perception only for the required scene facts that simple rules cannot establish.
Do not add routine before/after model calls to every action. An empty-flight
completion can report measured arrival without claiming object stability, retention
or support. Keep current images available for rehearsal inspection and subsequent
contact decisions. Skipping a model call never supplies an unmodeled scene fact.

Separate image evidence from an actual onsite answer: a hidden fact may remain
visually unknown while a host-recorded, explicitly scoped answer supplies it.
Do not require the images to prove that same hidden fact again. Recheck current
owner/pose, the stated action and visible scene; a changed scene, visible contact,
hazard or conflicting answer still blocks. Fetch/rejoin the original request ID
through `request_operator`; an editable local approval string is not authority.
Record assistance and the source of each fact, consume the answer for that one
dispatch, and never reuse it at the post-motion pose. Read-only localization does
not need a jaw-clearance gate. One clear relevant camera can resolve another
view's projection overlap; do not demand that every view expose the same surface.

All Cartesian tools control **left_link6 flange**, relative to **left_base_link**, in metres; quaternion is `[x,y,z,w]`. LEFT has an operator-measured effective-contact TCP at link6 +Z140mm and tip extent170mm (2026-11-04). This is not independently validated metrology. Motion arguments remain FLANGE; never pass TCP positions directly. `piper_fk` returns `effective_contact_tcp` separately. Candidate `tcp_geometry.flange_target` performs explicit conversion; do not add the offset twice. Right TCP is unavailable. `piper_fk(joints, arm)` is pure CAD FK for either arm. `piper_solve_ik(position, quaternion_xyzw, seed?)` is the installed persistent MoveIt2 KDL solver for the left arm only. An explicit six-radian seed permits offline planning. Full pose residuals are independently checked; IK alone does not certify a collision-free path.

`piper_move_joints(joints, duration_s?)` accepts six LEFT joints in radians and preserves the right arm and both jaws. `piper_move_pose(position, quaternion_xyzw?, duration_s?)` converts a flange pose to left joints; its executed path is joint interpolation, **not Cartesian straight-line servoing**. Omitted orientation preserves the measured flange orientation. The operator must configure a measured flange workspace box;41 FK samples check the joint-interpolated path stays inside it. Existing owner checks the entire interpolated CAD path and trajectory timing. The adapter permits at most 100 mm flange translation / 60 degrees joint change per call; runtime velocity is at most 10 degrees/s and acceleration20 degrees/s². These are device adapter limits, not universal RAX settings. Inspect the physical scene for objects absent from CAD.

Normally omit `duration_s`: the existing owner chooses a duration from its joint velocity/acceleration and jaw limits. Set it only when you have a reason and have checked that it is long enough. A timing rejection before dispatch did not move the robot; it is not evidence that a target was reached.

`piper_gripper(width_m, duration_s?)` commands the LEFT full inner jaw gap (metres), 0..0.075m, at most0.010m change per call. Nominal CAD100mm is not the installed jaw's usable range. Measured arrival is not proof of grasp, contact force, or support. Right gripper and right-arm motion are not exposed in this revision.

`piper_observe` returns head/left/right images. Use before/after observations at meaningful action boundaries; same saved frame may remain useful for analysis while the arm rests. The backend acquires fresh local feedback/cameras at dispatch and does not require a model answer inside a one-second perception TTL. A new frame does not validate stale object geometry automatically: if the scene changed, recompute the affected target.

During direct exploration, normally pass `observe_after=true` to `piper_move_pose`, `piper_move_joints` or `piper_gripper`. One call executes **one** action, checks measured arrival, then saves and returns current images, with their metadata under `observation`. This avoids a model round trip just to request the next observation. If capture fails, the reply retains `executed=true` and `measured_arrival=true` with `observation=null` and `observation_error`: call `piper_observe` only to recover the images, never replay the successful action. Inspect the returned images before deciding a new action; measured arrival does not prove object success. Omit the option or set it false for the original action-only reply.

Ordinary `api.runtime.Context` scripts are available during exploration. Before a
new orientation transition (including turning downward), first compare a bounded
set of read-only FK/IK candidates with explicit measured seeds. Plan translation
and orientation together from the current object location: prefer a reachable
pregrasp region above or adjacent to the object over a fixed remote turning pose
when the complete path is clear. No fixed "turn in place, then translate" sequence
is required. Preserve room for the full fingers, camera and held object; the TCP
point alone does not establish clearance. Do not descend into the object during
this empty transition. Missing object geometry requires observation/localization,
not historical coordinates or guessed heights.

Use `piper_solve_ik` in one short Context script for a small candidate batch (for
example 3–5); save each result/error without model turns between candidates. Use
`piper_fk` to check the candidate's flange/TCP/tip relationship. Prefer solutions
with joint-limit margin. Check the chosen joint-interpolated path using
`piper_check_path`, operating bounds and current scene evidence, including objects
absent from CAD. Keep right-arm and jaw values from the measured start. IK success
alone is not permission to move. Recheck state/scene if changed or stale before
separately dispatching one selected motion. Routine translations with established
reachability need not add an extra IK round trip; `piper_move_pose` still verifies
IK internally. Keep this in native manual exploration and short task functions,
not a new planner framework or preloaded task controller.

Only an error explicitly marked `recovery_action=replan_read_only`,
`dispatch_started=false`, `no_action_executed=true` and `control_uncertain=false`
allows read-only reselection after IK no-solution. Keep the current resting pose;
choose a different legal position/orientation combination, rather than repeating
the same failed query or relaxing limits/tolerances. No legal candidate means
report the block; do not loop indefinitely. Solver transport/protocol errors,
feedback discrepancies, rejected state bindings, tracking failures, owner changes
and unknown execution do not qualify. Stop physical exploration, inspect and
report those faults; do not "solve" them by choosing another motion. Never clear
an earlier uncertain action because a later read-only query failed or succeeded.

A short script may execute one already-decided action and return its observations
for inspection. Do not turn a first, unverified grasp/transfer/place sequence into
a blind batch of motions. Only compose maneuvers with established preconditions
and completion checks, following the upstream autonomous-control skill.

In Python, `ctx.call(...)` retains image blocks and `ctx.images(reply)` extracts them; `ctx.value(...)` returns only the JSON text. The saved files are under `runtime/frames/<observation.frame_id>/`. Return or display those images for the next visual decision; merely printing observation metadata does not show the scene to the model.

`piper_hold` while idle verifies rest and retains the owner. MCP calls are serialized, so this tool cannot interrupt a running motion call. Use the independent operator disarm/owner Stop or physical emergency stop for that. Never automatically replay an uncertain action. A `RejectedBeforeDispatch` error explicitly means no motion was staged. For other motion errors, inspect status/outcome and do not retry blindly.

The task program remains editable during RAX exploration. Use the upstream experiences/consolidation workflow without adding a device-specific task FSM or memory protocol.

## Measured tool characterization

Before selecting a physical tool, read [tool-characterization](references/tool-characterization.md).
It records measured timings, tested ranges, failures and untested capabilities. The 2026-11-05
3 mm pose trial failed arrival; do not count that primitive as accepted or replay it automatically.

### Measured settling and post-motion images (2026-11-05)

The Piper bridge distinguishes SDK dispatch, arrival within the unchanged tolerance, and `measured_settled`. Completion requires a one-second fresh-feedback window with whole-window range <= 0.0003 (existing units per channel), at least ten distinct samples, and the existing motion timeout. This window covers the recorded R8 delayed J4 update; it is not independent physical metrology or proof against every delayed CAN report.

`observe_after=true` retains the one-action/one-model-call interface. The backend additionally verifies current target error, owner identity, fresh feedback and local stability before capture. A drifted image bracket is retained as invalid evidence and can be reacquired once, read-only, after another stable window. No action is replayed and no auxiliary vision is called by these checks. `discarded_frame_ids` and `observation_settling` record this work.

Only image acquisition errors permit ordinary observation recovery. Stale/invalid feedback, owner change, original target tolerance violation, stability timeout or repeated capture drift request Stop and latch a state fault; save evidence and await physical-session recovery. Do not rearm, retry motion, load an old controller, or relax tolerances. `measured_arrival` / `measured_settled` do not mean grasp or task success. See `docs/R8_SETTLING_FIX_20261105.md` for offline evidence and the outstanding hardware qualification.

#### Limited live settling qualification

`docs/J4_SETTLING_LIVE_20261105.md` records one real J4 −0.5° out-and-back at the R8 empty transition pose. Both actions passed measured settling and image binding (4.540 / 4.485 s including acquisition, zero reacquisitions), with unchanged 0.003 rad tolerance and no movement of other joints in sampled feedback. Each passed a further three-second passive-rest check. This is not full pose-tool/3 mm acceptance, arbitrary payload qualification, or a resolved firmware diagnosis. Raw CAN already contained a transient return reversal; SDK/owner followed within ~10.6 ms in this run. Keep existing fault/Stop rules and do not skip settling or invoke vision to prove these numeric checks.


## Operator tolerance update — 2026-11-09

Current RAX joint arrival tolerance is 0.01 rad, explicitly approved by the operator. Historical 0.003 rad references are prior-run evidence, not the current policy. Read the latest tool-characterization entry; all rest, freshness, jaw and Stop guards remain active. Do not independently change the tolerance.

Joint state checks use the operator-selected0.01rad for arrival, anchors, observation/onsite binding and rest/recovery. Jaws keep metre thresholds; one-second rest still needs fresh distinct samples. This is not a firmware repair or proof of subthreshold motion. Read the current tool-table selection, not its full historical chronology.
