# Magnetic blocks — R7 Mac offline repair candidate

This writable candidate preserves the RAX R7 closeout at commit `2101a65`, followed by
Mac Codex engineering repairs requested by the user. See `REPAIR.md` for the current
entry contracts, margins, replay results and limits. Historical closeout files remain evidence.
No new physical rehearsal was run. It provides
single functions with explicit inputs and checked boundaries. **There is no complete
L-stack controller or unattended `main(ctx)`.** Green stacking is unimplemented.
The current revision is `candidate_offline_checked`, not physically qualified as a
whole. No robot or Gemini calls were made during software closeout.

## Physical evidence and qualification

R7 completed the orange/red base pair in one supervised function chain. Orange was
closed at a measured top-relative depth of approximately +128 mm with a 50 mm jaw
command (50.9 mm feedback), then trial-lifted 9.27 mm for a 10 mm request; wrist
correlation was 0.977. The chain raised orange 85 mm and moved it beside red.
Nominal 60 mm center spacing caused a **pre-dispatch** finger/red envelope refusal.
The arm reversed the partial descent to its checked retreat, widened the spacing to
65 mm at height, and lowered again. These dimensions are conditional R7 evidence,
not targets for another layout.

Final lowering completed, but its old wrist-template check failed (0.037 correlation)
when red entered the image patch. The original failed report is preserved. A separate
read-only support verification passed; release reached 68.6 mm, and a 90 mm upward
retreat passed broad visual checks. No onsite assistance was used in R7; three
auxiliary Gemini calls supported the support/release/retreat assessments. The agent
inspected and selected functions between invocations: this was not unattended.

Later empty inspections and green orientation transitions completed. Both green
pregrasp attempts stopped **before any motion**: first missing requested depth,
then an orange depth query with no valid samples. R7 did not grasp or stack green.
Historical R3 full-task success was supervised and onsite-confirmed; it does not
qualify these R7 functions.

| Entry in `controller.py` | Preconditions and result | Evidence/status |
|---|---|---|
| `locate_block(ctx, color=...)` | Fresh bound head RGB/depth; returns a visible surface estimate, never motion permission | Imported perception candidate; default locator may refuse sparse depth; no retry |
| `open_empty_gripper(ctx, ...)` | Startup empty state and clear opening sweep; feedback-based jaw steps | R7 passed, final 68.7 mm |
| `empty_joint_waypoint(ctx, joints=..., ...)` | Empty, open jaw; native path and sampled finger/table checks | Three R7 transitions passed |
| `approach(ctx, waypoints=..., quaternion=..., previous_report=..., ...)` | Same-run scene/empty history or supplied real checker; bounded vertical descent and endpoint check | Orange approach passed; not an unrestricted planner |
| `close_grasp(ctx, previous_report=..., color='orange')` | Aligned supported block; closes to 50 mm, without lifting | Orange passed; green variant untested in R7 |
| `trial_lift(ctx, quaternion=..., previous_report=..., color='orange')` | Verified closure; measured motion, jaw and wrist retention checks | Orange passed; green variant untested |
| `transport(ctx, waypoints=..., quaternion=..., previous_report=..., color='orange')` | Verified retained payload, matching color and same orientation; one modeled segment | Orange raise/transfer and bounded partial-placement reversal passed; color guard added offline |
| `plan_base_placement(ctx, previous_report=..., quaternion=..., ...)` | Read-only candidate selection in a 60–75 mm spacing range, preferred 65 mm; full native preflight and at least 3 mm nominal finger/red margin | Offline native FK/IK/collision checked; no dispatch |
| `lower_to_support(ctx, descent=..., retreat=..., quaternion=..., previous_report=..., support_required=True)` | Orange only, front/back grip; modeled red neighbor, same-run table/grip relation, checked retreat | Partial segments passed; final endpoint recovered separately after template failure. Green is explicitly rejected |
| `verify_completed_action(ctx, previous_report=..., assess=None)` | Completed receipts, unchanged owner/state/last motion; no dispatch | R7 final support recovered; source report hash preserved |
| `release(ctx, previous_report=...)` | Verified final orange support and fresh support assessment | R7 passed; no automatic retreat |
| `retreat(ctx, waypoints=..., quaternion=..., previous_report=...)` | Completed supported orange release; open jaw and rising path | R7 passed; old width-ratio helper retired |
| `empty_inspection(ctx, position=..., quaternion=..., previous_report=...)` | Verified empty retreat/inspection, current or retained same-run block-height bounds | R7 inspection and two green orientation transitions passed |
| `green_pregrasp(ctx, position=..., quaternion=..., previous_report=..., stationary_scene_report=..., scene_invalidated=False)` | Empty inspection predecessor; receipt-updated orange plus original stationary red/green; RGB-first, optional wrist optical depth; one green-relative hover | Both R7 attempts had zero motion. Depth/provenance repairs offline only; candidate 12 mm finger/top, 65 mm finger/table limits untested |

All positions are metres in `left_base_link`, controlling `left_link6`; quaternion
order is xyzw, joints are radians. Functions use the supplied `ctx` unchanged and
return JSON-compatible reports with `ok`, `why`, `measured`, `facts`, `receipts`,
`executed`, `action_completed`, `dispatch_attempted`, and `needs_review`. Failure
never automatically opens a held object. Unknown evidence returns for review;
ambiguous dispatch triggers a hold attempt. A completed action with missing endpoint
vision is not replayed. Combined legacy grasp/place entries refuse execution.

## Offline repairs in this revision

- `current_arrays` selects colored interior windows with at least nine real valid
  depth samples per 7x7 window. It never fills missing depth. Fewer than three usable
  windows returns unknown before querying the tool.
- The saved orange frame also has a center-depth hole. A spatially spread, flat
  measured colored patch (z spread at most 4 mm, pixel span at least 10) may supply
  height for the existing RGB-center ray intersection. The report explicitly labels
  this a geometric estimate, **not measured center depth**. This assumes an upright
  cube with approximately horizontal top; it has no new accuracy qualification.
- Retained green carries its original report hash, frame and unchanged measurement.
  Validation rejects changed source hashes, coordinates, frame labels or owner.
  Moved orange is updated from actual verified placement feedback; it is labeled action-derived, not fresh depth. The caller must set `scene_invalidated=True`
  after reset/contact/displacement. Wrist color cannot prove stationary hidden XYZ;
  omitted history remains a limitation.
- New reports hash `green_checks.py` and `qualitative.py` along with the existing
  control/perception modules. Native motion, IK, path and budget checks remain.
- Regressions preserve the real recovery path and original finger-overlap refusal,
  reject excessive XY bow, neighbor-facing drift and downward motion, and distinguish
  broad endpoint support from in-flight wrist retention.

## Run offline

From the workspace root, using Python 3.12 with NumPy, OpenCV and the existing test
fixtures:

```bash
python code/magnetic_blocks/verify_offline.py
```

This blocks live Context construction and socket access, uses workspace `tmp/`, and
runs the current offline regression suite (80 tests at this revision). Saved-depth tests use an explicitly offline median-depth projection
adapter; they do not validate native API execution or physical clearance. Saved FK
replays reject dispatch calls. Current logs: `output/repair/offline-tests.log`; `output/closeout/` is the unchanged earlier RAX closeout.

A single-function preview also opens no Context:

```bash
python code/magnetic_blocks/rehearse_step.py --function green_pregrasp \
  --inputs output/r7-inputs/green-pregrasp.json \
  --output output/closeout/preview-only.json
```

Those inputs are historical evidence, not current targets. The preview prints a
plan and does not create an invocation report.

## Future authorized rehearsal only

Hardware is disconnected for this closeout. A later session must acquire fresh
state, scene, calibration/owner identities, and scene-relative inputs; never execute
archived R7 poses. The invocation API is:

```python
from rehearse_step import run_one
# ctx supplied by the authorized session; inputs contains current-scene arguments.
record = run_one(ctx, function, inputs, output="output/rehearsal/new-unique-name.json")
```

The equivalent CLI is `rehearse_step.py --function NAME --inputs CURRENT.json
--output NEW.json --execute`; `--execute` constructs one outer Context. Existing
report paths are refused. `plan_rehearsal.py` and `r7_plan.py` are historical **live
read-only** planning scripts, not offline checks; running them contacts services.
Final support/release/retreat default assessments may call Gemini in an authorized
session. No real scene reset or new test was performed during closeout.

Next authorized session starts with new localization and owner binding. Test the
base-pair planner and retained-scene inspection/green entry, then green approach/closure/
trial separately. A green-specific stack descent, red support model, orange neighbor
clearance, release and retreat are still required before any complete task test.

## Provenance and artifacts

The imported code was the R6 rehearsal candidate with earlier RAX and Mac-authored
repairs (documented by the imported README and `references/rehearsal-handoff.md`).
R7 task-stage changes and the original closeout were authored by RAX main Agent;
this subsequent engineering revision is authored by Mac Codex. The imported
code arrived without a project git repository; do not attribute all inherited lines
to the closeout author. `PROVENANCE.json` records initial file hashes and source
identities. `CLOSEOUT.json` records offline verification and physical limitations.

Authoritative immutable reports: `output/rehearsal/r7-*.json` and `.tools.jsonl`.
The regenerated derived ledger has 31 invocations: `memory/RUNS.md` and
`output/rehearsal/r7-run-summary.json`. Decisive base-pair frame:
`runtime/frames/07d0a1bd4a6a44f28a5c927d84ff64d0/`; later clear head view:
`runtime/frames/6be6431d81254c6e90e51fe2ba1ac856/head.jpg`. Last failed pregrasp frame:
`runtime/frames/ed11dd3c4e224bda89363ae91324d679/`.

Closeout summary, hashes and logs: `output/closeout/`. Current handoff:
`memory/PROGRESS.md`, `memory/INDEX.md`. Reusable local lesson:
`memory/skills/magnetic-block-placement-boundary/`. Skill-library publication is
pending; no MCP merge was available or attempted during software-only closeout.
The referenced `AGENT_OWNERSHIP.md` and advertised `software-closeout/task-checkpoint`
were absent from the supplied workspace; their contents were not invented.
