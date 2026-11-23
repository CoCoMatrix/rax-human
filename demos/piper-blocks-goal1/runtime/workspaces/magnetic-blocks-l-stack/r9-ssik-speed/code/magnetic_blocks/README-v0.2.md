# Magnetic blocks — R8 composed candidate

`controller.py:main(ctx)` is a complete **COMPOSED CANDIDATE**, checked **offline only**. It chains the R8 functions on the **same supplied context**, calculates inputs from this run, persists each boundary and stops at the first failure. It is not a validated unattended controller or a claim of generalized layout success. No hardware, camera, robot MCP, live Context or Gemini connection was made during composition.

The pre-composition README is preserved byte-for-byte as `README-R8.md`. Its statement that the operator had not answered is superseded by the native reply now supplied in `references/composition-source/operator/`. Original machine reports are unchanged.

## Evidence and qualification

R8 task-stage commit: `c8a3fc6643814ac8d22291a7a5e82137a6ce1875`. Its native offline closeout completed as `bd9713baa7d67281ad62a2499e617324614cf4ff`. The copied baseline already contained those closeout code changes before their commit; original code bytes were verified against the final closeout. Composition additions are identified separately in `output/composition/BASELINE.json` and `references/composition-source/SOURCE-CLOSEOUT.json`. Archived machine reports contain **30 supervised function invocations, 27 passing, 3 refusing before dispatch, 45 motion receipts and 5 auxiliary visual calls**. Green lowering executed; a separate visual support assessment refused release. There is **no archived R8 green-release or green-retreat motion receipt**.

The operator answered actual support request `ff43deacefc7`: **“绿色已稳定落在红块上，大体对齐，可以原位张爪”**. Its native request and late reply are in `references/composition-source/operator/`. The user subsequently stated **“目前这个流程已经演练成功打通了”** and requested composition instead of further motion. Both are preserved as **operator-reported facts**. They do not establish an unattended pass, supply missing motion receipts, or authorize a new run. No historical confirmation, request ID or unconditional visual predicate enters main.

Automatic green release/retreat remains physically unqualified. Their independent support checks and final visual assessment are included unchanged. Positive synthetic tests demonstrate software integration only. New neutral-start entry selection, current-layout calculations, the full chain, varied layouts, provisional depth/TCP accuracy and collision/support assumptions require separately authorized physical qualification.

## Invocation from the workspace

Run from the **workspace root**, because inherited image/history validators use workspace-relative assets.

```bash
# Default preview: no Context construction or connection.
python code/magnetic_blocks/controller.py
python code/magnetic_blocks/controller.py --output-dir output/composition/runs/example
```

Ordinary Python using a context supplied by the authorized native host:

```python
import sys
sys.path.insert(0, "code/magnetic_blocks")
import controller

report = controller.main(ctx, output_dir="output/composition/runs/new-run-id")
```

`main` is the callable and can execute actions. It has no preview flag. The **CLI** previews by default; for a future connected and separately authorized session its explicit gate is:

```bash
python code/magnetic_blocks/controller.py --execute \
  --output-dir output/composition/runs/new-unique-id
```

The CLI creates one outer Context and passes it unchanged to main. Native ownership, arming, Stop/disarm, Prepare, reset and backend lifecycle belong to the host. The controller does not implement them or generate home trajectories. Run only one controller; do not interleave direct actions.

## Expected start and stage ordering

Start at **native XPolicy neutral/Prepare**, with left joints near zero and the **closed empty jaw in the existing 0–5 mm startup range**. The three upright blocks must be separately on the table and observable now. A held 50 mm jaw, an open jaw from a prior task, non-neutral pose, uncertain control, missing evidence, insufficient native budget or ambiguous opening sweep is refused before motion. The fresh guard uses the existing scene/pad/opening-sweep check; main never opens merely to discover whether something is held. Unexpected Prepare settings require review outside main, not a relaxed guard.

1. Fresh empty-state check; bounded empty jaw opening; read-only current-scene entry planning; three distinct empty joint transitions.
2. Orange approach, close, trial, raise; red-relative base planner; transfer; intermediate lowering; final support lowering; independent release/opening; retreat/stability check.
3. Two empty inspection/orientation transitions; green pregrasp; fresh red/orange support measurement; new explicit green empty-history root; approach, close and trial.
4. Green raise; first lateral segment; second raise; second and third lateral segments; stack lowering; independent release/opening; retreat and complete-structure visual check.

Every primitive failure stops the chain. Read-only bounded candidates may be compared before motion, but no failed/ambiguous dispatched action is retried. No trajectories are merged and no checking or observation policy is reduced.

## Inputs and parameters

`main(ctx, *, output_dir=None)` takes only a report-directory option. Default: a new UUID directory under `output/composition/runs/`. An explicit directory must **not exist**. There is no cached-scene, archived-input, resume-index, skip-check or historical operator-approval argument.

Positions are base-frame flange metres, joints radians, quaternions xyzw. `composition.py` contains ordinary calculations; rehearsed function signatures and checkers are unchanged. Every target comes from current observations, validated same-run report history or measured FK. Orange uses `placement_plan.side_by_side` relative to measured red and the actual pickup/table relation. Green uses `stack_checks.stack_target`, refreshed red/orange and the actual green-to-flange pickup relation. Occlusion retains only this run's bound scene through existing validators; old pixels are never queried against new depth.

| Composition choice | R8 evidence and limits |
|---|---|
| Orange downward posture, approximately −3° yaw | `down-candidate` / `down-plan-02`; reusable orientation, no stored XYZ/joints |
| Tilt fractions .65, .45, .30 | `initial-plan` read-only IK candidates; only .65 physically selected. Bounded selection on another layout remains a candidate |
| Two tilted transitions, first at .65 of fresh joint delta | Transitions 1–2; computed from current joints, native 60° segment guard retained |
| Orange top-relative high/down hover +235/+215 mm | Initial entry and successful transition 3 |
| Insertion top+128 mm, maximum 20 mm segment | Orange five segments and green four; six-segment and geometry guards unchanged |
| Orange raise +85 mm; separate final lower 50 mm; retreat +90 mm | `raise-01`, `lower-02`, `retreat-01`; intermediate target is actual table support height plus 50 mm |
| Inspection green Y+100 mm at current clear height; 45° then 90° down orientation | `inspection-01/02`; original whole-finger path checker remains |
| Green hover top+195 mm | Successful `green-pregrasp-02`; failed +215 mm attempt is not replayed |
| Green raises +55 and +25 mm, separated by first lateral move | `green-raise-01/02`; first lateral target preserves current X/Z and uses placed-orange Y−105 mm, followed by the second raise, halfway XY and final XY; each original segment gate remains |
| Green placement away from orange | Existing `stack_target` default 3 mm; actual pickup offset and measured red height determine target |
| Green retreat | Reverse the saved checked lowering to its start height from actual release XY; positive release/retreat remains physically unqualified |

These are documented R8 relative choices, not proven bounds for arbitrary layouts. Unsupported reachability, separation, clearance, heights or visual outcomes return partial. Speeds, tolerances, geometry, visual rules and optional Gemini behavior remain unchanged.

## Reports and failed-action recovery

Each new directory contains `REPORT.json`, numbered step JSON reports, `.tools.jsonl` logs and the existing runner's `REHEARSAL_STATE.json` / `REHEARSAL_CURRENT.md`. `rehearse_step.run_one` is reused. Historical `output/rehearsal/`, memory and other runs are not overwritten.

The report records active step/path before entry; actual monotonic elapsed/wall and per-tool times; tool and auxiliary `gemini_generate` counts; explicit `primary_model_calls=0` for this main runtime; motion attempts, confirmed receipts and ambiguous dispatch counts; complete/partial status; control and held-state uncertainty; failure reason and explicit next recovery boundary. Intent is saved before dispatch. Green pregrasp's implicit `color='green', engage=False` is recorded for provenance, while `invocation_inputs` retains exact public kwargs. Code hashes include composition glue.

A failed/ambiguous action is never automatically replayed. No finally block opens, retreats or homes. Existing functions can request native Hold after an action fault. A lost receipt requires native state inspection. Interrupting Python does not prove hardware stopped. Missing images after confirmed movement stop at that endpoint without replay. A negative green support assessment remains a dispute; never seek another verdict by retrying from an older successful lowering report.

Main has **no automatic resume**. Its supplied context is latched on entry, including failure: a different output directory cannot rerun main on that context. Existing directories are refused. Same-session recovery uses individual functions only after checking actual native status, receipts, current evidence, authorization and exact predecessor. Preserve owner/bridge/calibration and immutable history. `verify_completed_action` supports only its documented boundaries and **does not recover green stack boundaries**. Reset, unrecorded movement/contact, visible displacement, restart, owner change or association loss invalidates retained scene evidence. A new Context/directory is not proof a pending action is safe. A fresh full run requires separate native reset/empty-state resolution and authorization.

If reporting itself fails, inspect the active report and pending dispatch log. Aggregate counters are marked `counters_complete=false` when Python can record that failure. A hard kill can leave `in_progress`; it is not success or permission to replay.

## Individual-function invocation

The callable list is `rehearse_step.FUNCTIONS`. Existing signatures, including optional `assess` where already supported, are unchanged. Use the recorder so later functions have immutable predecessor reports:

```python
from rehearse_step import run_one

# After current startup evidence and separate authorization:
record = run_one(ctx, "open_empty_gripper", {"gap_m": 0.069},
                 "output/rehearsal/NEW-UNIQUE-open.json")
# current_inputs must be calculated from this session, never archived R8 XYZ.
record = run_one(ctx, "transport", current_inputs,
                 "output/rehearsal/NEW-UNIQUE-transport.json")
```

Direct `controller.transport(ctx, **current_inputs)` is also supported; the caller owns persistence before another function consumes it. CLI equivalent (preview unless `--execute` is added in a separately authorized live session):

```bash
python code/magnetic_blocks/rehearse_step.py --function FUNCTION \
  --inputs CURRENT-INPUTS.json --output output/rehearsal/NEW-UNIQUE.json
```

| Functions | Required boundary |
|---|---|
| `locate_block` | Current exactly bound RGB/depth; surface localization, not motion permission |
| `open_empty_gripper`, `empty_joint_waypoint` | Empty startup/open-jaw path evidence; native and complete scene guards |
| `approach`, `close_grasp`, `trial_lift` | Current scene and continuous empty history; insertion; separate retention verification |
| `transport`, `plan_base_placement` | Verified payload, preserved posture, current/retained scene and actual pickup relation |
| `lower_to_support`, `release`, `retreat` | Separate descent, support, opening and withdrawal |
| `empty_inspection`, `green_pregrasp` | Verified orange release/retreat; same-run stationary source and action-updated orange |
| `refresh_stack_support` | Successful empty green pregrasp; exact root hash; same owner/calibration |
| `green_stack_lower`, `green_stack_release`, `green_stack_retreat` | Complete green-only support history; checked descent; independent support assessment; release and withdrawal |
| `verify_completed_action`, `verify_approach_boundary` | Only documented completed-action/empty-approach endpoint recovery; no ambiguous-action replay |

## Offline verification and identity

```bash
python code/magnetic_blocks/verify_offline.py --portable
python code/magnetic_blocks/verify_offline.py --composition
python code/magnetic_blocks/verify_offline.py
python -m compileall -q code/magnetic_blocks
python code/magnetic_blocks/controller.py
```

**79 inherited portable tests and 14 composition tests pass.** Live Context construction and sockets are blocked. Composition tests run the real main, primitives, recorder, receipt continuity and support-history validation, with explicitly synthetic vision/geometry/IK replies. They cover full-chain success, first failure, ambiguous/interrupted dispatch, missing images after completed motion, changed layouts and actual pickup offset, same ctx, startup guards and actual log counters/times. They are not a simulator or model evaluation.

The final host full suite **passes all 119 tests**, including the 14 composition cases. Original R7 reports, tool logs and referenced images were copied with hashes from their source; no evidence was rewritten and no tests were skipped. Earlier missing-fixture failures remain in the development logs. Final commands and results are in `output/composition/HOST-VERIFICATION.json`; local commit, baseline and code hashes are in `SUMMARY.json` and `CODE_HASHES.json`. No remote push or hardware run was performed.

The supplied copy already contained uncommitted closeout edits to controller, stack checks, offline verification, README and closeout tests. The baseline records those differences from frozen R8; the candidate retains them. All 31 existing controller functions/classes and every checker module are unchanged relative to the supplied baseline. Optimization proposals are recorded only in `ANALYSIS.md`; none is claimed as implemented.
