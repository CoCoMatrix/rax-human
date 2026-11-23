# Composition review — optimization analysis only

No optimization proposed here is implemented or physically qualified by this
change. Sources are the supplied composition/autonomous-control/workflow/lessons
references and **magnetic-block R8 evidence only**. No other task's state,
coordinates, timing targets or numeric control constants were imported.

Evidence aggregation: workspace `output/composition/HISTORICAL_ANALYSIS.json`.
Immutable R8 tool logs record 30 supervised invocations, 27 passes, 3 refusals
before dispatch, 45 motion receipts, **261.180492 s inside functions**, and
**5 visual calls totaling 34.563667 s**. Times exclude Agent gaps; they are not
the duration of a composed run. No physical composed run or A/B test occurred.

## Necessary composition glue, implemented

- `main(ctx)` supplies ordering, same-context calls, first-failure return and
  explicit held/recovery state. It does not control native backend lifecycle.
- `composition.py` calculates scene-relative entry/placement inputs previously
  supplied between functions by the Agent. Current observations and actual FK
  replace archived coordinates; this is necessary composition, not optimization.
- `run_one` records the implicit green identity of `green_pregrasp`, separately
  from exact invocation kwargs. The supplied closeout support checker requires
  that identity; its signature, motion and evidence checks stay intact.
- Read-only calculations use the existing recorder through an optional callable.
  Counters distinguish attempted dispatch from confirmed receipts. A new output
  directory, current-run history and a same-context full-run latch prevent
  accidental replay. No observation, check or evidence is removed.

All existing controller function/class bodies and checker modules match the
supplied baseline. That baseline already differs from the R8 task-stage commit; its earlier
closeout repairs are not composition optimizations.

## Prioritized future investigations

| Priority | Evidence | Proposal for a separate change | Qualification / current status |
|---|---|---|---|
| P0 qualification | `r8-stack-lower-01` positive support assessment, followed by `r8-stack-release-01` negative assessment and zero release motions. Native operator reply confirms support only at the old pose. | Resolve automatic support/release evidence and qualify positive release plus retreat on new layouts. Distinguish visible relation from force/support metrology. | **Unresolved.** No future unconditional predicate, automatic retry or deleted checks. |
| P1 duplicate support queries | Orange lowering visual 7.535 s and release visual 9.823 s; green lowering 6.259 s and release 5.926 s. The green assessments disagreed. `LowerChecks`, `VisualReleaseChecks`, `StackLowerChecks`, `StackReleaseChecks` ask related endpoint/current-support questions. | Study referring to unchanged endpoint evidence with explicit freshness, movement and owner validity while retaining contradiction detection and the opening boundary. | **Analysis only.** Do not collapse checks because an earlier answer was yes. Varied-layout support/occlusion evidence and timing comparisons needed. All queries remain. |
| P1 large exception/context output | `r8-green-pregrasp-01` carries native IK error JSON with recovery/budget state. Orange approach report is 492,191 bytes; green approach 430,172 bytes. Facts repeat full scenes/path samples; runner retains full tool results. | Preserve complete immutable artifacts but provide bounded inspection summaries and links. Consider deduplicated evidence IDs only with immutable hash checks. | **Analysis only.** No new truncation, schema reduction or evidence deduplication. Interrupted recovery must remain auditable. |
| P1 observation redundancy | 32 explicit `piper_observe` calls total 17.604 s, in addition to observations inside motion receipts. `_start` refreshes at boundaries. New composition calculations add reads. | Measure whether a following read-only calculation can consume a recent receipt observation with unchanged owner/drift/provenance gates. Separate reacquisition after movement from duplicate full-scene localization. | **Analysis only.** No archived observations reused or existing boundary observations removed. |
| P2 FFS and optional perception | `qualitative.assess_images` may request wrist optical depth after head-color failure. No explicit extra wrist-depth tool invocation appears in these 30 function logs. Five visual calls total 34.564 s. | Instrument missing-fact reason, valid ROI, requested depth and actual consumption before deciding whether requests can be shared or suppressed. | **Analysis only.** Absence in these logs does not prove future FFS is unnecessary. Existing behavior stays; no wrist hand-eye assumption or base XYZ from optical data. |
| P2 planner feasibility | R8 high downward pose and green top+215 mm refused IK; orange down top+215 mm and green top+195 mm worked in this layout. `side_by_side` scans red-relative spacings and samples paths. | Evaluate full read-only feasibility through later approach/transfer/retreat poses; expand bounded candidates only with task-specific evidence. Preserve exact workspace, IK, limits and refusal reasons. | **Analysis only beyond the required entry-selection glue.** No tolerance changes or joint-limit margin relaxation. |
| P2 sampled FK / repeated checks | 864 FK calls total 3.228 s; 24 explicit path checks total 5.295 s. Native dispatch independently checks paths; many checkers sample 41 points. | Profile first. Consider FK batching or exact-state/environment-keyed evidence only when all geometry/owner/environment dependencies are pinned. | **Analysis only.** Call count is not latency rank. No reduced samples, native checks or approximate cache keys; environment completeness remains limited. |
| P2 planner/segment overhead | 23 pose RPCs total 88.594 s, 5 joint RPCs 55.258 s, 17 jaw RPCs 53.850 s. Totals include more than motor travel. R8 retains grasp/trial/lower/release/retreat checkpoints. | Attribute preview, prior-action acknowledgment, staging, SDK travel, settling and response from native timestamps before considering adjacent free-space segment changes. | **Analysis only.** Composition is not continuous trajectory execution. No speed changes, merged segments, altered jaw steps or erased contact boundaries. Any such candidate needs separate physical validation. |
| P2 report I/O / provenance | `read_report`, empty history and `support_scene` reread and hash predecessors. Tightened root binding rejects changed/legacy support records. | Profile I/O against hardware time; consider immutable per-run references only with identical mutation detection. | **Analysis only.** No cached or reduced history validation. Failed/disputed reports must remain available. |
| P3 ordinary checks / model fallback | Existing alignment/retention rules and optional models remain. Full synthetic chain has six fake visual boundaries; archived R8 has five because green retreat did not execute. | Evaluate false refusals and wrong acceptance on saved and newly labeled frames, explicitly defining what broad visual facts can prove. | **Analysis only.** Synthetic yes answers are test doubles, not autonomous support evidence. No claim of zero-model future runs. |

## Evidence reconciliation and limits

Operator statements “绿色已稳定落在红块上，大体对齐，可以原位张爪” and
“目前这个流程已经演练成功打通了” describe their successful rehearsal assessment.
The first has a native saved reply; the second is preserved in the composition
request. Machine records stop at refused release. Both categories remain in the
handoff without fabricating motion receipts or an unattended pass.

The final full suite passes all 119 tests, including 14 composition cases, after importing original R7 fixtures and tool logs with hash verification. This establishes software contracts only. Generalized layouts, positive automatic release/retreat and an unattended full run still require physical tests. Earlier missing-fixture failures remain archived.

## Additional upstream gaps — analysis only

Upstream asks for resuming individual steps and a fallback for each failing motion. Main deliberately never automatically replays a partial run, but green stack endpoint recovery is not yet covered by the existing generic verifier. Add and qualify that boundary in a separate change. Also consolidate the currently scattered task-relative distances into named constants with tested ranges; do not change values during that refactor.
