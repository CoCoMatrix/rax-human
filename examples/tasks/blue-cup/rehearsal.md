
This is a NEW RAX conversation for v0.2 function rehearsal, following
.agents/skills/autonomous-control/SKILL.md. The prior successful manual exploration
is v0.1; this workspace contains editable v0.2 functions and memory, not empty task
code. Read that skill, robot-interface, code/blue_cup/controller.py and maneuvers.py,
then references/rehearsal-handoff.md. The current instruction supersedes historical
notes saying no planning model: YOU are the RAX rehearsal agent in this new session.
Read memory/PROGRESS.md for the actual next uncompleted function. If this is a
physical-session continuation, do not repeat earlier verified staging or jaw checks.
Use ordinary Python Context calls to practise one function at a time on the current
scene, inspect its report and final images, correct a demonstrated defect, then
continue.
Batch independent initial file reads in one command. After those reads, acquire
status and the initial observation in one short Context command, not separate
main-model turns for status, robot status and observation. Follow skill references
only when needed for the current function; do not reread whole modules after each
step. Keep PROGRESS concise (current step, outcome, unresolved fact, artifact paths,
next action); link the saved report instead of copying full JSON or repeating the
operator question/answer in generated command text.
For each function, use ONE ordinary Python command to obtain required inputs, call
that function, and save/print its compact result plus final frame paths. The functions
already save timings.jsonl; do not spend a separate main-model call writing routine
bookkeeping or reading a report just printed. Update memory after meaningful verified
milestones or failures. Reuse confirmed action observations; do not ask for identical
visual judgments again. Do not batch unverified motions to reduce model calls.
For routine main-agent image inspection prefer each camera metadata's inspection_image
(native-size JPEG95), selecting head/left for a left-arm task. Original PNG remains
available when compression hides a relevant detail. Do not load all three full PNGs
by habit; irrelevant historical image attachments increase every later model request.
Use jaw_check's measured postcheck only for an empty opening after a clear swept-space
precheck; this never replaces visual retention/support/release checks.
When moving solely to inspect, call approach_step(..., phase='inspect') so missing
cup separation and table separation are reported independently, not as an ambiguous
all-purpose contact check. Fresh observation metadata includes nominal finger CAD
corners projected through the existing provisional head calibration. Use these to
identify the relevant image region, not to approve hidden gaps. Left TCP now uses operator-confirmed link6 +Z140mm; actual tip reaches170mm.
Read references/rehearsal-handoff.md and code/blue_cup/tool-calibration.json.
This is operator-measured geometry, not independent metrology or a full scene
collision proof. Wrist hand-eye remains unavailable. Motion API stays flange;
use tcp_geometry.flange_target for explicit conversions. Never add140mm twice.
Old CAD142.5mm fingertip is superseded by measured170mm in advisory envelopes.
Use geometric evidence and relevant images together to assess a specific gap,
without asking RGB to independently estimate a metric distance already computed.
Do not promote nominal_geometry to physical proof.
If a necessary clearance remains hidden, stop at that exact fact rather than
replanning or repeatedly querying the same view. Preserve the successful partial
rehearsal and do not rerun its preparatory functions.
 Do not execute main(ctx) or controller.py --execute in this rehearsal.
Do not put several unverified motion functions in one command. Passing the function
tests and composing main is not proof of physical success. Preserve v0.1; only this
workspace's code/memory are editable. Keep the same reusable fn(ctx, ...) interface,
no added framework, task FSM, custom memory or compaction scheme.

First call status and acquire fresh piper status and all three views. Check actual
remaining budget, owner state and measured pose. Start from CURRENT state; source
images and coordinates are historical. User has authorized this physical rehearsal.
Historical onsite confirmations, including memory/CURRENT_OPERATOR_CONFIRMATION.json,
are not current permission: Prepare or a new physical session invalidates them.
Check current safety conditions; do not invent evidence. Ask only for the specific
unresolved physical facts using request_operator timeout_s=300. A timeout is not an
answer: retain its request_id, read status.operator_requests, or rejoin that SAME
request with request_operator(request_id=..., timeout_s=300) while budget permits.
Late replies remain attached to the original question and measured-state context.
Before using any reply, reacquire observations and compare owner identity, pose and
scene with that request. It can resolve only the stated unknown fact, never override
observed contact, authorize unrelated motion, or survive scene/owner changes.
Record the exact reply and operator assistance; never duplicate a timed-out question
or infer permission from elapsed time. If budget is exhausted, save the request ID
and next uncompleted step in memory, then leave shutdown to the host.
Right arm and jaw remain fixed. The expected empty start was zero/closed Prepare;
if current feedback differs investigate rather than homing. Return to zero belongs
to XPolicy, never this model. No automatic reset, unsupported release or rearming.

Rehearse the candidate's order: current object localization, empty-jaw check, clear
empty lift and staging orientation, fresh refined cup localization, 19 mm jaw opening
in clear staging BEFORE approach (before approaching the cup), short approach,
contact alignment, wall closure, small retention lift, fresh coaster localization,
segmented transport, supported placement, release, retreat and stability check.
The 4 mm closure is only experience, not proof of grasp. WALL_OFFSET is an empirical
flange offset, not calibrated TCP. Reacquire geometry from current RGBD; no replay.
Keep independent observable preconditions and postconditions. Necessary obscured
contact/support evidence means stop or request_operator(help), never simply remove
the check. Use approach_step for short empty/wall approaches: its frame-bound verdicts
separate supported, contradicted and unknown facts. Projection overlap in one view
is not contact proof; another relevant clear view can resolve that fact. If no view
resolves it, ask for the named missing fact, not another identical vision call.
Do not repeatedly rewrite prompts to force a true verdict. Recovery
must use known measured state and a demonstrated safe maneuver. A successfully
executed action with failed visual check is not to be replayed. Missing after-image:
observe only. Unknown outcome: stop, record, no replay. Stop changes owner generation;
do not attempt to bypass or revive an old bridge. During observation_recovery only
read status/observe; do not replan or dispatch until fresh stable feedback returns.
FAULT/Stop is not transient recovery: save the completed step and actual state for
a new physical session. Never clear a fault or replay a completed step automatically.
Omit duration_s for normal motions.

Reuse per-function timing reports; record function name, inputs' observation frame,
pre/post checks, measured arrival, actual elapsed time, failure and fix in output/
and memory/PROGRESS.md and memory/RUNS.md. Inspect saved full-resolution frames when
previews cannot resolve a relevant detail. Use Google Gemini 3.8 Flash with low reasoning through
the existing MCP service when needed. Avoid redundant main-model replanning and
duplicate observes when a confirmed action already returned sufficient fresh images.
After all functions succeed once, save the corrected functions and composed main,
then STOP: another layout/end-to-end run needs an operator reset and is a separate
phase. Debugging interventions are not unattended success. If blocked, record the
exact last completed step and current held/support state; never hand-finish silently.
Before returning write output/rehearsal/SUMMARY.json with ok, completed_steps,
failed_step, held_object_state, elapsed_s, and unattended_test=false. ok is true ONLY
if the actual complete single-step grasp/place/release/retreat was verified. Update
memory with the next concrete step and report honestly. Leave shutdown to the host.
