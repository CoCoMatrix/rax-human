"""One canonical Rainbow-style workflow contract, also pinned by Freeze."""
import hashlib

TASK_POLICY = '''Workflow: RAX real/rainbow (agent perception checkpoints).
Solve by exploration, turn successful maneuvers into reusable functions, then test an
unchanged candidate end to end. Franka's autonomous-script branch is a different mode.
This session uses the configured native Codex model; no substitute model or controller.

Start once via START_HERE.md: TASK.md, robot-interface and relevant harness skills, memory/INDEX.md,
matching task tips, skill-library/INDEX.md, and code/cup_to_coaster/README.md.
AGENTS.md is already native context; do not print it again. Read other skills/code only
for the current primitive. Follow official memory/PROGRESS, INDEX, TIPS and RUNS roles;
native Codex handles compaction, and the index retrieves relevant evidence only.
Do not bulk-load code/history. ROUND.json and memory/AUTO_STATE.json are measured current
progress; PROGRESS/TIPS are interpreted notes. Historical coordinates, masks and operator
replies are not current evidence. After compaction use the automatically injected RAX continuation. RESUME.md/context_entry are fallback only; reconcile receipts and reuse current post-action views while valid.
Do not repeat the entire bootstrap or dump whole snapshots. Save batched read-only
IK/geometry results to output/preflight-summary.json and print a compact summary.
The ready-to-use ik_preflight.py batches up to64 official piper_solve_ik calls without motion.
For the upper shelf, shelf_preflight.py measures CURRENT table/upper-surface raw depth and
checks five key poses per candidate in one read-only batch. Call it before committing
to a grasp; close requires a recent matching feasible candidate. This is a necessary
task-height check, not all-path approval or frozen planning. Change grasp height,
side/orientation or landing point when needed; yaw alone did not rescue the previous
high rim grasp. Keep immediate collision/contact checks and the provisional TCP caveat.
At empty Prepare only, open can use clear_space_sequence=true with explicit current
empty_jaw_confirmed and jaw_sweep_clear checks. It executes at most three existing
<=9mm jaw segments, checking each measured receipt. Inspect final head/wrist images
before approach. Contact closure, lift, carry and release are still visual checkpoints.
A structured PlanningRejected with executed=false/dispatch_started=false is a
pre-dispatch refusal: preserve it, obtain fresh observation, then change the hypothesis.
Timeouts or missing receipts remain unknown; never infer no execution from error text.

Exploration is incremental: execute one bounded primitive, inspect its result, then
revise or implement the next primitive at a known stationary checkpoint. A complete
controller and IK for every later stage are not prerequisites for a safely recoverable
first step. Keep the short overall task outline and check immediate clearance, support
and a stopping/recovery state before contact/lift. A failed downstream pose is evidence
for a changed hypothesis, not a demand to keep repeating the same global preflight.

Use code/cup_to_coaster/round_step.py (or the selected frozen program).
No arguments: observe. Otherwise: ACTION PARAMETERS_JSON CHECKS_JSON EVIDENCE_NOTE.
Each result provides frames, receipt, decision_contract and allowed next actions.
Use the contract to answer a bounded decision, not to repeat task-wide planning.
Code handles geometry, IK, default jaw/lift parameters, receipts and progress journaling.
The agent identifies targets, chooses scene-dependent corrections, judges contact,
retention, clear paths and supported release. Never infer an unobserved fact as true.
Supply a short factual evidence note with each decision; no separate checkpoint-plan
file or duplicate progress-write command is required. Write TIPS only for a new lesson,
and RUNS at attempt completion. Do this before the next observation, not after it.

Run a command, wait for completion, parse compact JSON and display EVERY path in review_frames (including lift comparison) in
ONE tool call. Returned post-action images are the next checkpoint. No extra cat or
observe unless expired, changed, unclear, or interrupted. Read failure details only on
failure. An executed/arrived receipt remains executed if post-observation fails: observe
only; never replay. Unknown execution or pending dispatch requires reconciliation.

EXPLORATION: revise and debug code at stationary checkpoints with known execution.
The initial controller is an editable candidate, not a frozen source or mandatory
strategy. Fixed top-down orientation, wall side, jaw recipe and stage sequence are
legacy candidate design; alternatives require current geometry and explicit physical
clearance reasoning. Never confuse recipe restrictions with backend hardware limits.
The controller archives source versions and receipts; after a revision it discards old
visual approval and obtains a new observation. Do not change code mid-dispatch or while
execution is unknown. Exploration may complete with multiple versions and assistance.
FROZEN TEST: one unchanged candidate end to end; all source/prompt changes or hand-finishing
invalidate that test. Mixed-version exploration is learning, not fixed-version evidence.
Code may chain only checks/motions with demonstrated reliable guards; retain visual
review before contact, after grasp/lift, before supported release and after retreat.
Do not skip existing review points merely to reduce latency. This is not a requirement
for the LLM to calculate every servo target or rewrite plans after every tool result.

PiPER: LEFT arm only, preserve right arm/jaw. MCP is the only hardware route, one writer.
MoveIt2 KDL is the only live IK; no legacy fallback. Calibration and 110mm contact-centre
offset remain provisional. Keep qualified hardware limits. When reusing the known
wall-pinch approach, preserve its195mm standoff; never silently reduce it to force IK.
A different candidate grasp needs its own justified clearance, not inherited approval.
Use original valid FFS samples, no filled depth. SAM3 once initially for the task object and destination from task-target.json;
later checkpoints use fresh raw head/wrist views, no gripper segmentation or old masks.
Initial SAM3 masks also carry same-frame native point associations; these are historical
after the scene moves and must never be reused as current geometry. Current target_rois
are normalized comparison regions, not SAM prompts. Include cup and
the current destination for transport/placement. For planning, use geometry_batch.py REQUEST_JSON_PATH to inspect cup ROI,
explicit visible local patches, table and upper shelf together from ONE original FFS
frame. It returns measured distributions, surface fits and optional whole-rim fitting;
fit_ring defaults false. An unsupported global rim is a different perception strategy
branch, not an instruction to repeatedly read source or recreate diagnostic scripts.
The result persists in geometry-analysis-latest for context recovery. Identical content
may reuse diagnostics only; fresh action frames and all existing guards remain required.
Whole-rim cup_rim_model is optional when supported; its original support/planarity gates
are unchanged. Local patches are diagnostics, not automatic contact or motion geometry.
raw_pixels remains explicit and must pass exact original sample checks; no nearest-point
substitution or filled depth. Batch candidate key heights with shelf_preflight, which
separates IK feasibility from existing_approach_compatible. The current approach primitive
accepts top-down yaw only; a feasible tilt requires a separately reviewed primitive.
Do not discover this incompatibility by repeatedly attempting the old primitive.
Open19mm in clear space, close4mm, nominal40mm trial lift. Measured flange lift alone
does not establish cup retention/clearance. Trial lift returns paired before/after
head and wrist frames: display all four and judge retained cup geometry, no slip and
actual cup following. Combined with the measured lift from supported start, these
facts can establish separation even if the head view occludes the base. Do not ask the
operator solely because no direct base gap is visible; diagnose contradictory evidence. Keep150s review and2s source gates and
scene-change checks; never renew timestamps or extend limits. Pre-dispatch expiry needs
a fresh observation, not owner reset. Stationary piper_hold(mode=cancel) verifies rest;
active cancellation/stop_command faults require operator recovery, never auto-rearm.

Ask request_operator only for missing physical facts/reset/help, normally timeout_s=300.
After timeout, retain the request. The protected feedback inbox is injected at tool,
compact/resume and final-response boundaries; do not poll repeatedly or repeat the question. Match its request and request round_last_report and answer round_last_report to current state; a late
reply is not a new confirmation after later motion. An expired request is unanswered, not evidence of failure. Then consume the actual
reply, obtain fresh observations and continue if resolved. Do not end an otherwise
healthy exploration just because help was requested and answered. Record assistance;
never label it unattended. Genuine control faults/contact uncertainty require Hold/help.
Only controller finish may generate round-result.json after placement, release and
retreat are verified. Then stop for exact-version Freeze and subsequent reset tests.
An assisted completion or frozen draft is not evidence of autonomous reliability.
'''

def policy_for(task_target):
    if task_target!='table_coaster':return TASK_POLICY
    text=TASK_POLICY
    start=text.index('For the upper shelf, shelf_preflight.py')
    end=text.index('At empty Prepare only',start)
    text=text[:start]+('This task targets the table coaster. No shelf-height/preflight requirement applies; '
                       'establish transport clearance and complete coaster support.\n')+text[end:]
    start=text.index('Batch candidate key heights with shelf_preflight')
    end=text.index('Open19mm',start)
    text=text[:start]+text[end:]
    text=text.replace('table and upper shelf together','table and coaster together')
    return text


def contract():
    return {'schema':'rax.workflow.v3','branch':'real/rainbow',
            'task_policy_sha256':hashlib.sha256(TASK_POLICY.encode()).hexdigest(),
            'coaster_policy_sha256':hashlib.sha256(policy_for('table_coaster').encode()).hexdigest(),
            'agent_effort':'high','auto_compact_token_limit':100000,
            'perception_mode':'agent_checkpoints'}
