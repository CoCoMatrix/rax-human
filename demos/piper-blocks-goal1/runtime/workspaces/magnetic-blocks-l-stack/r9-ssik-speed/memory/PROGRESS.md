# R8 handoff — stopped holding green, task incomplete

Last physical motion: `output/rehearsal/r8-stack-lower-01.json`. Green remains held at51.3mm jaw. **Do not release/replay lowering or start retreat.** Separate `r8-stack-release-01` refused BEFORE any jaw motion: its visual assessment reported possible front overhang, contradicting the lowering assessment. Control is known/settled, not a motion fault.

Native operator request `ff43deacefc7` remains timeout; reply=None. It asks whether green rests stably on red, is broadly aligned/not bridging, and can safely open at this exact pose. Rejoin this same ID; a timeout is not permission. Fresh status/images and matching owner/pose are required after any reply. If support is not confirmed, do not open.

Current frame `fc0c2ae5a5a348a884df56e9e1b52237`; measured14 `[-0.04291764630654057, 1.5434644707086653, -1.264683029287611, 1.375563796666811, 0.0, -0.04352851154473858, 0.0513, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002]`. Owner generation1; bridge `fc52f55c9e9e417eaba121d0a0e24a4e`. Current measured remaining **3187.626s**, elapsed2812.374s. Native60s finish reserve remains mandatory. Stop is because physical support is unresolved, not budget exhaustion.

## Completed rehearsal
- Orange/red base pair: startup opening, three empty transitions, insertion, closure, trial, raise, repaired65mm-spacing planner, transfer, two lowering segments, release and90mm retreat passed. No onsite assistance for the base pair; three Gemini calls.
- Green: two empty orientation changes, repaired pregrasp at top+195mm, insertion top+128mm, closure50mm ->51mm feedback, measured trial9.25mm, staged loaded rise/transfers passed.
- Fresh support reference `r8-support-refresh-01` measured red[.302170,-.016147,.078554], orange[.299723,-.083360,.079284]; red shifted~6mm from initial, so it was remeasured.
- Green lowering reached a target derived from actual same-run grip/table offset. Nominal model passed; **physical support disputed**, not qualified. Green release and retreat have not executed.

## Results and code
30 supervised function invocations:27 report-level passes,3 failures,45 completed motion receipts,261.180s total inside functions,5 auxiliary visual calls. No unattended whole-task test. Primary-model call count unavailable; invocation count is not a substitute. Native pending operator request is recorded assistance, with no answer yet.

Code commit `c8a3fc6643814ac8d22291a7a5e82137a6ce1875`, branch `r8-supervised-rehearsal`, under `code/magnetic_blocks/`. Added `stack_checks.py` and separate refresh/lower/release/retreat entries. Fixed initial transition depth request and stale orange empty-history leakage into new green approach. Original reports remain unchanged. Read-only helpers `r8_plan.py`, `r8_summary.py`. No complete autonomous main.

Run one NEW current-scene function: `python code/magnetic_blocks/rehearse_step.py --function FUNCTION --inputs CURRENT.json --output output/rehearsal/NEW.json --execute`. Omit execute for preview. **Do not run archived input files as targets.**

63 self-contained offline tests pass with Context/sockets blocked: `python code/magnetic_blocks/verify_offline.py --portable`; log `output/r8-portable-tests.log`. Full inherited suite lacks R7 reports/images; initial80-test attempt produced28 FileNotFoundError records. No full-suite pass claimed.

## Evidence / limits
- `output/r8-handoff.json`: exact latest status, operator record, observation, metrics and commit.
- `memory/RUNS.md`, `output/rehearsal/r8-run-summary.json`: all invocation outcomes and durations; `*.tools.jsonl` exact calls.
- Last support views: `5fbe143183bf4e64a681eaa83e37ef60`, optical wrist `d529d047aa57426db9b94d4ffbfc035e`; failed release frame `0911a4aa530f4e77b37be72f1fbbb38d`.
- Sparse head patch queries at `e5236683cefa43d7822b4660ca06db71` were discarded (2/4 valid samples, background/inconsistent heights). `output/r8-wrist-top-analysis.json` is inconclusive optical analysis, NOT a support fact or base target.
- Calibration/TCP provisional/operator-measured; no force sensing or independently calibrated wrist hand-eye.
- Shared library merge succeeded:main874af8b5304271597d01aafcc9d9d3dc9e438e91. Only bounded empty-hover IK supporting evidence; existing wrapper stays draft.

Next: resolve the native support question with fresh observations. If confirmed at unchanged pose, implement/rehearse an explicitly bound one-use onsite support path for green release, then independent retreat/final check. Otherwise use only a newly justified safe recovery. Keep green held while facts remain unresolved.
