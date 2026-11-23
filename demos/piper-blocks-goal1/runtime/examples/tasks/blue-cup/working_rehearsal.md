
CURRENT STAGE: AGENTS.md workflow step 2, autonomous-control FUNCTION REHEARSAL
(not step 1 manual exploration). Run on the editable v0.3 copy.
Frozen versions stay unchanged. Test one function at a time, inspect its measured
result and current images, fix demonstrated defects, then compose tested functions.
Do not execute main(ctx), reset the scene, or run an unattended scale test.

## Startup: required context, then ONLY prepare_for_approach

Follow AGENTS.md: read TASK.md, memory/PROGRESS.md and memory/INDEX.md once. The
START_PROMPT already names AGENTS.md/MCP_SESSION.md; do not reread unchanged files.
Read autonomous-control and robot-interface. The tool-characterization reference is
`.agents/skills/robot-interface/references/tool-characterization.md`;
read its FIRST section, Current selection for startup, and only the linked detail
needed by the next tool. The historical diagnostic chronology is not startup input.
If skill-library exists, read its INDEX.md and continual-learning's At the start
section; follow only listed matching entries/dependencies. Read contribution/merge
details when contributing. Preserve native skill_library_merge checks.

Task experience is at memory/skills/blue-cup-r12-placement/SKILL.md and
memory/skills/blue-cup-r10-deeper-wall-grasp/SKILL.md (current operator-preference
section includes the >=40mm single trial lift). These are TASK MEMORY, not
skill-library/skills entries. Do not guess another path or search the whole workspace.
Read decisive historical evidence only to resolve a specific applicability question.

Run the existing READ-ONLY entry once before task planning:
`PYTHONPATH=. python code/blue_cup/rehearsal.py snapshot '{}'`
It supplies status, current budget, state/owner, image paths and source_sha256 in
one command. Inspect head/left views; inspect right when relevant. Do not repeat
status/observe/hash reads just to reconstruct this output. Its saved JSON contains
{status, observation}; compact stdout contains remaining_seconds and source_sha256.
Snapshot budget/state are historical after a pause; use subsequent tool budgets
and re-observe before motion. The workspace root is not guaranteed to be a Git
repository; startup git status/whole-tree discovery is unnecessary.

Read code/blue_cup/README.md (the short current contract, not its archived history).
The first function is prepare_for_approach(ctx, {}), in rehearsal.py. Read that
function, preparation_goal/preparation_candidates and move_once with targeted
ranges. README describes perception/retained-target interfaces; read their source
only if a SPECIFIC unresolved precondition, interface mismatch or failure requires
it. preparation has NO placement.py or maneuvers.py dependency. Do not search
code/blue_cup/*.py or read grasp/lift/placement before preparation is verified.
Once current requirements and the live snapshot are understood, record the tool
selection and call prepare_for_approach. If blocked, record the concrete missing
fact; do not keep reading downstream code as a substitute for that decision.
Do not separately call status before snapshot: snapshot already calls it. Read
manual-robot-control only if actually switching to manual control. Record inspected
hashes and next function in PROGRESS; after compaction read changed/needed ranges.
These are reading-order instructions, not a file-access restriction or permission
to omit a needed precondition. Code remains writable and inspectable.

## Function rehearsal and evidence

While a function process is still running, wait/poll that same process using the
native terminal tool. Do not reread source or dump intermediate reports simply to
wait; inspect the final compact result and relevant images once it returns.
If approach returns next_action=retry_same_function_with_fresh_checks and
target_retained=true, its first segment had a typed transient visual-provider
failure before ANY dispatch. The measurement is preserved for ONE retry only.
Call the same approach function once; the visual transport creates a new connection,
and the function gets new RGB,
owner/state/path and scene checks. Do not create a separate revalidation script,
edit target validity/timestamps, or apply this rule to any uncertain/executed move.

Use the supplied Context unchanged. Right arm/jaw remain fixed; XPolicy alone owns
Prepare. Rehearse preparation, hover, entry, closure, trial lift, coaster localization,
transport, lowering/support, release and retreat. A failure returns for inspection;
fix its demonstrated cause with offline checks, never replay an uncertain action.
If visual response latency expires the action boundary, re-observe and revalidate
the relevant scene facts before a bounded retry. Replacing only the observation or
timestamp while retaining expired scene evidence does not establish a fresh check.
Preparation/hover/entry retain one measured base-frame target through expected
occlusion, with fresh RGB/state checks; do not remeasure a hidden rim each segment.
Source measurement and current execution frames remain separate. Changed owner,
geometry, unexpected motion, failed rest or control_uncertain stops motion.
TCP is link6 +Z140mm, tip170mm; pose targets are FLANGE. Head calibration is
provisional, left FFS is optical-only without hand-eye. Modeled self-collision alone
does not establish table/cable/payload clearance. Keep necessary scene checks.

Current task policy: enter40mm below the measured rim, close3.5mm conditionally,
then plan ONE trial lift of at least40mm. Positive10/20mm suggestions select40mm
before full path/IK checks; larger legal requests are preserved. This is not
permission for an infeasible route or replaying an already executed lift.
Use head and LEFT wrist before/after cues for practical outcomes. A cup remaining
broadly at the same pad location in the moving wrist view supports retention;
combine it with upward arrival and available scene/background/shadow changes.
Hidden base or fine edges alone do not fail. Arrival/jaw width/shadows alone do
not prove pickup or support. Clear slip/drop/collision blocks. For a genuinely
unresolved critical fact, use one native request_operator and rejoin its original
request_id; preserve actual reply, scope, owner and pose checks. No invented facts.
Never release a suspended cup. No fixed auxiliary-vision pair on every action.
Use automatic duration and receipt observations; no extra depth merely to test FFS.

Functions write output/rehearsal/timings.jsonl. Record total/motion/perception/IK
time, visual_calls, target_id, localization count, explicit wrist FFS requests,
failures and assistance. Do not double-count parent/child times. Main-model waits
come from service logs. Preserve finish reserve and actual held/support state.
After all functions pass, implement main(ctx) and offline validate; do not run it.
Write output/rehearsal/SUMMARY.json with ok, completed_steps, failed_step,
held_object_state, elapsed_s, unattended_test=false and evidence/code paths.
Keep native memory and compaction. Leave final Stop to host; no automatic homing
or shutdown release. Source is writable; frozen versions are immutable.
