# Blue cup v0.4 merged candidate — rc9

Status: rc8(13fdeee) completed live L in64.398s with16motions and1Gemini call5.115s. It stopped released/retracted, notPrepare. rc9 fixes the missing-head-depth gate and unused wrist-depth request; host collision checks reuse exact unchanged right-arm internal distances. Offline validated only; no new live test. Same ordinary Python main(ctx), no main Agent or added framework.

rc6 backup: backups/blue-cup/20261110-rc5-before-handoff. Single pose goal limit400mm, still subject to60deg joint step, workspace, IK, installed collision checks and dynamic limits. Full translation subgoal is attempted first; half/quarter alternatives are read-only planning fallbacks before one selected dispatch. Orientation preparation remains staged. Local accepted review-idle polls at50ms; errors/other no_action keep original backoff. Operator selected0.3s measured settling, minimum4 distinct samples and unchanged0.01rad joint/0.3mm jaw range bounds. K measured settling windows0.312–0.317s; single-run evidence only. SDK50%, effective joint peak17deg/s, acceleration40deg/s², grip target3mm remain selected.

## Observation schedule

1. Acquire initial head RGB-D and wrist views. Identify cup/coaster once, measure near-wall grasp target, cup center, table and coaster in left_base_link. Initial empty-jaw/scene facts use ordinary rules, Gemini only if needed.
2. Prepare, hover, descend into the measured wall region and close to3mm. Each bounded maneuver checks current state, owner, IK, whole-path installed collision geometry and measured arrival/settling. No per-segment image, depth query or Gemini. Initial three-dimensional targets remain the same-run measurement; never relabel them as current images.
3. Take a comparison view for the40mm trial lift and an after-lift view; ordinary rules judge likely pickup. Gemini is an optional fallback. No fixed pre-lift auxiliary vision call.
4. Build the transfer goal from the INITIAL metric coaster and measured pickup transform; no coaster re-localization. Transfer using bounded numeric motions.
5. Descend to the computed placement height, with no intermediate observation or support-image gate. The height is `entry_flange_z + initial_coaster_z - initial_table_z`; equivalently return the actual lift minus the coaster/table height difference. Do not subtract40mm blindly or push below the prediction. The result is labelled predicted height reached, NOT visually confirmed support.
6. Open to12mm at the computed XY/Z pose, then retreat80mm. Acquire one final view after retreat and qualitatively verify cup placement. Simple rules first, optional Gemini if unresolved. Numeric opening alone is not recorded as task success.

Nominal perception boundaries are initial, before/after trial lift, and final placement. Extra observations are permitted only for an unresolved boundary or fault diagnosis. No implicit restart or failed-action replay. Continuous recording remains independent of controller snapshots.

## Scope and safeguards

The initial snapshot assumes the supervised tabletop scene stays stationary except for the intended cup motion during this bounded run. It does not cover externally moved objects or unmodelled dynamic obstacles. A scene change requires stopping and fresh localization. Initial depth/calibration is still provisional; code tests do not establish physical placement accuracy.

Unchanged:0.01rad joint checks, jaw tolerances, measured CAN arrival and operator-selected0.3-second settling, single owner, arming/Stop, IK and sampled collision checks with installed140mm TCP/170mm finger envelope and measured table patch. Numeric actions use `observe_after=False`; actual observations retain their age/state/geometry checks. Backend motion checks stream identity/heartbeat from metadata without downloading/decoding unused JPEGs.

Preparation retains orientation staging. Hover/entry, transfer (220mm total scope), computed descent and80mm retreat use complete goals where their checked paths pass; no speed/acceleration increase over rc5. Descent retains max8steps for planning fallbacks; placement position error<=2mm. Final support is assessed qualitatively AFTER release/retreat per operator instruction. These are candidate software bounds, not physical contact proof or untested generalization.

## Offline evidence

Run E stopped on a report contract bug after pickup, before transfer. Retained localization now preserves source contours and measurement frame; legacy fresh-coaster localization also checks this provenance explicitly. rc4 main uses the initial metric coaster and no longer depends on reacquiring it after pickup.

Real E image/depth + FK + installed FCL replay passed preparation, hover, entry and closure against all14 recorded action replies. All IK targets matched recorded requests exactly. Observation operations fell from29 explicit +14 action-attached to1 initial in that prefix, with no model call. Arrival feedback/IK answers were replay fixtures: this is NOT a new physical test or speed measurement. Initial-coaster target construction also passed the real E report replay with zero new images/models; computed descent31.318mm.

See verification/whole-e-performance-20261110 for timing breakdown, replay scripts and test logs. Run G completed one whole pipeline; varied-layout repeatability remains unvalidated.

## Ordinary Python / one function

From the active session workspace:

```python
from api.runtime import Context
import sys
sys.path.insert(0, 'code/blue_cup')
from controller import main
with Context() as ctx:
    report = main(ctx)
```

For supervised step tests call the same function with explicit inputs, e.g. `approach(ctx, {'phase':'hover'})`, `trial_lift(ctx, {'dz':0.040})`, or `lower_to_support(ctx, {'target_report': CURRENT_TARGET_REPORT, 'previous_report': LAST_SUCCESSFUL_REPORT})`. Do not invent report paths from another run. Later steps validate current owner/bridge/geometry/state. A restarted physical session invalidates old report bindings. On a partially executed or ambiguous failure, read current status/observe, diagnose, then re-enter the relevant function with current evidence; rerunning main is not recovery from a held cup.

## No-Agent launch (future live test only; not run during composition)

First use existing piper_explore.py with `--rehearsal-candidate` and WITHOUT `--rehearsal-agent`. This starts the native robot/frontend services but no Codex conversation. Use a NEW output directory each run. Wait for REHEARSAL-READY.json. Then the host runs the thin run_no_agent.py helper: it uses the existing frontend socket, an isolated Python runtime and bubblewrap, mounts no model socket/credentials/vendor stack, and executes controller.py once. The helper writes REHEARSAL-DONE.json for the existing supervisor to Stop. It does not start or arm hardware itself.

```bash
# On5090, AFTER reset, live authorization and final-function qualification:
cd /home/ubuntu/code_junqi/rax-piper-franka-20261104
PYTHONPATH=. /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python scripts/piper_explore.py \
  --output commissioning/NEW_UNIQUE_RUN \
  --rehearsal-candidate workspaces/blue-cup/20261109-v0.4-merged-candidate \
  --seconds 1800 --direct-model --http-proxy http://127.0.0.1:7892 --run
# Another terminal, once NEW_UNIQUE_RUN/REHEARSAL-READY.json exists:
/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python \
  workspaces/blue-cup/20261109-v0.4-merged-candidate/run_no_agent.py \
  --ready commissioning/NEW_UNIQUE_RUN/REHEARSAL-READY.json --run
```

The existing launcher currently labels the service session single_step_rehearsal/unattended_test=false. The compiled runner/report records execution_mode=compiled_controller_no_agent. Do not count the launcher's legacy label as proof of autonomous success. Check controller report, images and operator intervention log. Both CLIs above are future motion commands; none was executed while building this candidate. On interruption, use existing `scripts/piper_explore.py --stop RUN` and verify Stop/rest, not merely Python exit. Never home/open a held object automatically.

## Offline verification

```bash
# From candidate workspace on5090 (no motion/network):
PYTHONPATH=code/blue_cup:/home/ubuntu/code_junqi/rax-piper-franka-20261104:/home/ubuntu/code_junqi/rax-piper-franka-20261104/local_tests/unit \
 /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python -m pytest -q tests
```

Copied regression fixtures are redirected to this candidate code; the original R31 and frozen files are untouched. Tests cover pipeline stop/uncertain outcomes, changed owner/state, rule success without Gemini, selective fallback, current depth, wrist tracking, support estimates, conflicting/absent evidence and scoped head/wrist entry fusion. There is no blanket approve/disable-check switch. Archived image replay is offline evidence only and never renews timestamps or dispatches a motion.

## 2026-11-10 first whole-run findings
The first no-Agent run stopped before any task motion because a27.7s successful visual query outlived its20s observation boundary. Duplicate reference/current captures are now uploaded once. A slow successful precheck may acquire one fresh RGB observation within a bounded60s query window, verify unchanged owner, last motion, measured pose, geometry/calibration and full-view image differences, then recheck the current-start path. Original semantic frame/timestamps remain explicit; any visible change/occlusion, stale capture or changed control stops. This is endpoint association, not proof of events between frames, hidden clearance, or a motion retry. Onsite answers are not renewed this way.
Whole-run reports include stage_elapsed_s, auxiliary_visual_calls, timing totals, code hashes and metrics_complete; an uncaught stage exception marks counts incomplete. Motion target, arrival tolerance and depth freshness limits are unchanged.

## rc5 physical-speed candidate (2026-11-10)

WORKSPACE.json selects `motion_speed_profile=faster`. The Piper launcher makes a session-local owner profile with SDK50%; the existing bridge selects20deg/s nominal joint velocity with the unchanged0.85 tracking reserve (17deg/s peak) and40deg/s² acceleration. Ordinary automatic trajectory durations shrink; this is not just a change in model waiting. Shared XPolicy, native Prepare15%, frozen versions, collision paths, displacement bounds,0.01rad arrival tolerance, jaw speed/force and settling stay unchanged. Use `standard` in WORKSPACE.json to restore10% SDK and8.5deg/s effective/20deg/s² trajectory bounds.

The grasp default is3mm,0.5mm smaller than run G. This is a target gap, not extra force or proof of retention. The40mm trial lift remains. On G's23 recorded paths, offline collision/trajectory replay reduced summed planned movement51.83→32.80s. This is not measured total runtime; delays, settling and image calls remain. Next supervised run must qualify faster following, pickup/retention and final placement before accepting rc5.

## rc6 offline evidence — 2026-11-10

Recorded I scene, real KDL/FK/FCL and workspace checks passed the numeric sequence:16 moves (11 arm+5 jaw), versus24 in live I. Transfer4→1, lower2→1, retreat2→1; preparation8→7, hover2→1, entry2→1. Pure trajectory previews passed for all16; planned movement30.2s, terminal dwell3.2s. Arrival and qualitative visual results were test stubs; this is not a live speed or perception test. See verification/rc6-handoff-20261110.

## rc7 wrist pickup / visual transport — 2026-11-10

If pale cup colour/texture tracking is unresolved during trial lift, combine measured>=30mm lift, current head rim rise consistent with that lift, and chronological wrist cup contours remaining near the same pad relation. Orange/dark finger edges are excluded; no exact per-pixel match is demanded. Head rise alone, measured motion alone or jaw width alone cannot establish pickup. This adds qualitative pickup evidence only, not environment clearance/support proof. Recorded J replay passes without Gemini; missing wrist, shifted pad reference, no head rise and duplicate frames reject.

Host auxiliary vision now defaults to the actual XQNetwork HTTP proxy127.0.0.1:7892. The old independent17890 route failed during TLS; main-model direct mode stays unchanged. For Google vision, the host retries one read-only request on TLS EOF/remote disconnect/reset with a fresh connection and separate durable billing reservation; failed reservations are retained. Existing pre-POST connect attempts remain bounded at3, so at most2 request attempts/6 connects. No retry of auth/quota/certificate failures, no robot-tool replay, and no assumption that idle API TLS controls motion. At a required visual boundary the controller holds while reconnecting; persistent failure remains an unresolved task result.

trial_lift executed provenance now retains its successful motion receipt if only the subsequent vision request fails. Never replay an already-executed lift based on a failed observation. Reference verification/rc7-vision-20261110.

## rc8 qualitative outcomes

Trial lift: measured upward motion plus independent head rim rise and a coarse wrist cup region near the same visible pads. Partial pad occlusion and small seating changes are accepted; precise edges/texture are not required. This task-specific pale-blue appearance rule does not establish grasp force or invisible contact.

Final result: ordinary scheduled functions already validate placement XY/Z, measured opening and withdrawal. Their receipts now reach the result checker. A current wrist image showing the cup broadly remaining away from open pads completes the qualitative outcome, without demanding hidden head depth or exact centering. It does not set support_state=resting. Missing or contradictory evidence can still invoke the optional Gemini tool. No nominal intermediate image/model stages were added.

## rc9 evidence and motion handoff

Trial outcome now evaluates coarse wrist evidence independently after confirmed measured lift/loaded jaw feedback. Head depth can corroborate or contradict; missing precision head depth alone no longer blocks the wrist rule. Reliable contradictory head height, missing cup or substantial relative change still leaves the outcome unresolved. No depth request is made unless an unresolved fact has a usable wrist ROI that actually consumes depth.

The host collision adapter caches exact static RIGHT-vs-RIGHT results, keyed by unrounded right state, margin and geometry/pair/object identity. Every path sample, left/cross-arm/environment pair and the independent execution-bridge check remain. This saves computation before dispatch; it does not join trajectories or remove arrival/settling. See verification/rc9-continuity-20261110 for real FCL differential tests and recorded L vision replay. Runtime host identity is stored there separately from this controller Git commit.
