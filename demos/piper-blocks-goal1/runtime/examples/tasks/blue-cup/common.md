Start direct RAX exploration from current observations. The task code directory starts empty.
Do not load prior cup controllers. You may create and revise short Python Context helpers
from current evidence; do not build a task FSM. XPolicy owns
motion transport and operator return-to-zero, never model-directed homing. The
right arm stays fixed. Acquire current status and all three views before motion.

For normal motion use observe_after=true and omit duration_s for checked automatic
timing. Reuse the receipt and relevant returned head/wrist views; do not add a
separate observe or auxiliary call after every action. Control feedback and
geometry establish their measurable boundaries; visual evidence establishes
visible cup state, grip, support and slip. Neither projection overlap nor an
occluded underside proves contact. Auxiliary vision is optional: ask a specific
unresolved visible question using only relevant current views (and a labelled
before frame when needed), not accumulated history. An API budget refusal is not
a robot fault; inspect its required/remaining amounts and use adequate available
evidence or one justified revised request, not repeated identical requests.

Batch independent read-only depth calculations and FK/IK candidates in one short
Context helper; return a concise summary and save detailed evidence on disk.
Before an orientation transition jointly check cup-relative positions and
orientations, preferring reachable clear space toward/above or adjacent to the
cup. No fixed remote turning station is required. Routine reachable translations
do not need redundant IK. Pose tools use flange coordinates, not contact TCP.
Use fresh supported surface samples, never historical absolute targets or a cup
interior sample as rim height. Task bounds alone do not prove clearance.
Typed replan_read_only with no dispatch/control uncertainty permits bounded
comparison of different legal targets. Actual feedback faults, unknown execution
or state-binding failures stop physical exploration; never replay or relax guards.
If motion succeeded but its image is missing, recover observation only.

Read the R10 deeper-wall-grasp entry linked from memory/INDEX.md. The user confirmed
that its deeper contact region was suitable: about 17.3 mm deeper than that failed
shallow grasp, roughly 26 mm below the then-estimated rim. Recompute from the
current rim/wall, TCP and fingertip envelope; these are references, not coordinates
or a validated universal offset. R10 measured 3.8 mm after a 3.5 mm jaw target;
4.3 mm at the shallower grasp failed. Neither a fixed gap nor arm arrival proves
that the cup is held.

If a test lift does not lift the cup or reveals slip, keep the task active. With
healthy control and the cup fully supported, reopen, inspect wall engagement,
adjust deeper within verified table/rim/handle clearance, close to a justified
smaller legal gap, then retest. If unsupported, first restore verified support;
never open a suspended cup. Inspect at each meaningful contact boundary, do not
blindly replay. After two unsuccessful adjustments, re-localize/reassess the
contact rather than indefinitely tightening or deepening; request only a concrete
missing fact if needed. Real control faults still stop.

Apply the latest 2026-11-06 operator test-lift preference in memory: evaluate ONE
upward displacement of at least 40 mm (0.040 m), rather than repeated tiny lifts.
This is a requested candidate, NOT a physically validated loaded-motion range or
proof of grasp. Check current reachability, tool limits and the complete gripper/
cup path before dispatch; if that distance is infeasible, replan or report the
specific limit, never force it or silently count a shorter lift as this validation.
Before lifting, identify useful before/after evidence. Determine retention from
object motion relative to the table, cup-to-pad stability and available continuous
frames for slip. Direct visibility of the cup bottom is not mandatory if other
validated evidence establishes separation and retention. Use corresponding cup
features/surfaces in a consistent reference frame; do not infer lift from the same
pixel sampling a different surface, arm arrival, jaw width, or wrist imagery alone.
Use current head/wrist views together and saved recording when informative; no
fixed auxiliary-vision call is required. Distinguish confirmed retention, supported
grasp failure, and unknown. For supported failure use the bounded regrasp procedure
above. For unknown, first seek informative existing evidence or a geometrically
checked observation adjustment; do not repeat an unchanged obscured view. Ask the
operator only for a critical fact still unresolved, not routinely after each lift.
Never release a possibly suspended cup. Attribute human confirmation explicitly.
R11's 30 mm arm movement passed, but cup clearance remained unconfirmed; no success
or successful recovery is established by that run.

Consume operator_updates even on error replies. After a help timeout reuse the
same request_id and wait if needed; do not prematurely finalize while a required
reply is pending and active budget remains. An answer is a fact, not automatic
motion authorization; refresh live evidence before acting. Use wall_budget from
tool replies for actual remaining budget, not extra status calls merely to estimate time.

Follow upstream experience-memory continuously: update concise PROGRESS/TIPS with
conditions, relative geometry, outcome and evidence paths; refine existing notes.
Use native CLI compaction, then reread PROGRESS/INDEX and only matching memories.
Do not reload old images except for a specific comparison. During exploration,
retain editable helpers and accurate notes. After visible TASK.md success,
consolidate the understood maneuvers into editable functions; a stop or budget wind-down gets a partial handoff,
never a false success. Then follow autonomous-control for functions and checked
continual-learning for library contributions. Assistance is not unattended success.
No further trial or scale test without a fresh scene/operator reset. No automatic
cup reset, release or homing at shutdown; existing frozen versions remain unchanged.
