# Magnetic blocks — R8 software closeout

**Software closeout only; hardware is stopped and disconnected.** Do not construct a live `Context`, call robot/Gemini tools, run `r8_plan.py`, or execute physical functions in this phase. Archived observations describe the past, not the current scene.

`controller.py` provides functions accepting a supplied `ctx` and explicit keyword inputs, returning `ok`, `stage`, measured evidence and failure reasons. There is no complete autonomous `main(ctx)` and no unattended whole-task success. The closeout version is **candidate_offline_checked**, not newly hardware-qualified.

## Historical R8 results

The supervised chain completed the red/orange base pair and green pickup/transport. Green lowering executed and its assessment passed, but the separate release assessment disputed support/alignment and made **zero motions**. Green release and retreat did not execute; no completed L-stack was confirmed. The last recorded state held green at 51.3 mm jaw. Hardware state has not been reacquired during closeout.

The historical ledger contains 30 function invocations: 27 report passes, 3 failures, 45 motion receipts, 261.1804919 seconds inside functions and 5 auxiliary Gemini calls. The three failures (higher pregrasp IK, stale approach history, disputed green support) made no motion. Primary-model call count is unavailable. Operator request `ff43deacefc7` had no answer in the archived record; elapsed waiting is not confirmation. These are supervised invocations, not autonomous task runs.

Original evidence: `output/rehearsal/r8-*.json`, matching tool logs, `output/r8-handoff.json`, and `memory/RUNS.md` at workspace root. Original reports were not rewritten. Historical R3 success and R7 evidence remain separate from R8 qualification.

## Function contracts and qualification

| Entry | Preconditions and result | Physical evidence / remaining limits |
|---|---|---|
| `open_empty_gripper`, `empty_joint_waypoint` | Observed empty state; native path and scene guards; measured opening/transition | R8 supervised passes; initial-depth fix exercised |
| `approach`, `close_grasp`, `trial_lift` | Measured current target, continuous owner/action history, insertion and retention checks | Orange and green R8 passes; green used explicit new approach root after stale-history refusal |
| `transport`, `plan_base_placement`, `lower_to_support`, `release`, `retreat` | Retained payload, checked envelope and path; separate support/release/withdrawal boundaries | R8 orange base pair completed; no whole-task qualification |
| `green_pregrasp` | Same-run green source and placement history; checked empty approach | Top +195 mm succeeded in one R8 layout; +215 mm refused at IK; not reusable absolute targets |
| `refresh_stack_support` | Completed empty **green** pregrasp; fresh red/orange head depth; same owner/calibration | R8 measurement passed; new source-hash binding is offline-only |
| `transport(..., color='green', support_report=...)` | Fresh base support provenance plus verified green-only action chain | R8 transport passed; closeout provenance tightening is offline-only |
| `green_stack_lower` | One 5–50 mm vertical segment, preserved grip/orientation, modeled center/neighbor/finger clearance and checked reversal | One R8 lowering executed; support later disputed. Numerical endpoint is not support proof |
| `green_stack_release` | Completed final lowering and independent current support assessment | R8 refused before opening. Positive release remains physically untested |
| `green_stack_retreat` | Completed verified green release, open jaw and rising path; final structure check | Not physically executed |
| `verify_completed_action` | Known completed action, receipt/source continuity and supported recovery stage | Does not recover green stack boundaries; must not promote disputed green support |

Inputs use metres, base-frame flange positions and xyzw quaternions. The 60 mm blocks, depth/TCP and collision envelopes remain provisional. Wrist FFS is camera-optical evidence, never base-frame targets. New-scene targets require new perception; saved inputs are historical records only.

## Closeout repairs

`refresh_stack_support` now pins the exact pregrasp report bytes and rejects a non-green root or a report changed during measurement. `stack_checks.support_scene` requires that binding, rechecks hashes of already retained actions, and rejects a chain that bypasses a previously retained predecessor. Canonical paths detect aliases in cycles. Historical R8 refresh reports lack the new binding and are deliberately refused by the new code; they remain valid records of what the old version did.

The existing placement-history repair preserves orange's old empty approach only as provenance and starts a new active history for green. A new regression checks that it does not mutate its source scene. Further tests verify zero-motion rejection of negative support and exclusion of green lowering from generic endpoint recovery. These checks do not provide a persistent dispute latch across independent calls: callers must retain the failed release report and must not retry from an older successful lowering to seek a different verdict. No automatic dispute recovery or operator-approval path was added.

## Offline verification and preview

From the workspace root:

```bash
python code/magnetic_blocks/verify_offline.py --portable
python -m compileall -q code/magnetic_blocks
python code/magnetic_blocks/rehearse_step.py --function green_stack_release \
  --inputs output/r8-inputs/stack-release-01.json \
  --output output/closeout/preview-only.json
```

The last command prints historical input JSON only: it has no `--execute`, creates no Context and does not validate a live scene. The guarded suite passes **79 tests**, including 16 closeout regressions; live Context construction and socket access are blocked. Tests use synthetic contexts and available saved fixtures, not hardware or Gemini. `output/closeout/` holds command logs and the final `SUMMARY.json`.

The default full suite needs R7 reports/images absent from this workspace. The task-phase attempt ran 80 tests with 28 missing-fixture error records (`output/r8-offline-initial.log`). It was not a passing full-suite run and was not rerun during closeout. Portable success does not replace those missing checks.

For a separately authorized future hardware session only, the single-function invocation is:

```python
# ctx is supplied by the authorized runtime; never construct it in this closeout.
result = controller.green_stack_release(ctx, **current_inputs)
```

The runner equivalent is `python code/magnetic_blocks/rehearse_step.py --function FUNCTION --inputs CURRENT.json --output output/rehearsal/NEW.json --execute`. This command is documentation, not a closeout action. It logs exact calls, receipts and code hashes and refuses existing report paths. There is no whole-controller run command.

## Provenance and handoff

Imported Mac candidate `282a26d4353cb698c82bcc9def3130d7b9af9321` is retained with its authorship. RAX's R8 physical-stage baseline is `c8a3fc6643814ac8d22291a7a5e82137a6ce1875`. Current repair/review is by the RAX main Agent. See `R8_PROVENANCE.json` and `CLOSEOUT-R8.md`. `README-R7.md`, `REPAIR.md`, `PROVENANCE.json` and `CLOSEOUT.json` are inherited historical documents, not current R8 verification claims.

Current task notes are `memory/PROGRESS.md` and `memory/INDEX.md`; the pre-closeout versions are archived under `output/closeout/TASK_PHASE_*.md`. A checkpoint copy of the refined experience is in this repository's `closeout/` directory. Final commit and integrity hashes are recorded in `output/closeout/SUMMARY.json`. The referenced `AGENT_OWNERSHIP.md` and `software-closeout/task-checkpoint` were absent from the supplied workspace; no host backup is claimed. Shared-library publication remains pending because MCP is prohibited in this phase.

Future physical work requires a newly authorized connected session, current observations and resolution of the support discrepancy before release. Positive green release/retreat and an unattended varied-layout run remain unqualified.
