# r9-ssik-speed-r2

Independent offline-verified candidate based on r9-ssik-speed commit
`efb573593bcd5bc466f0f124da3efc5608456930`. This candidate has not been physically
qualified. The original runtime, controller and frozen versions remain unchanged.

## Changes

- Preserve a fixed gripper command through small feedback changes. The task-level
  connector check now distinguishes measured feedback from an arm-plus-jaw
  command. It checks the swept finger envelope spanning both gap values, uses
  the exact float32 wire gap, and retains the existing 0.5 mm feedback bound.
  Native owner/state/collision/dynamics checks remain authoritative.
- Retry a task-envelope-rejected blend at tangent scales 0.35 and then zero,
  before using separately checked fallback legs. Each changed curve is checked
  again. Dispatch failures are never retried automatically.
- Route ordinary single arm legs through the same preview/certificate path as
  multi-waypoint legs. Native stage reuses a matching, unexpired, single-use
  certificate; changed inputs, scene, owner or invalid current state still fail.
  Contact/withdraw curves are restricted to their original monotonic straight
  joint segment. Closing and release remain separate actions.
- Retiming shortens whole-leg allocations under the existing speed/acceleration
  limits. Straight legs select the faster admissible quintic or cosine-ramp
  profile. Blends use a bounded search and recheck all coupled polynomial extrema
  and the final 30 Hz float32 stream.

## Unchanged operating limits

SDK 80%; effective joint cap 27.2 deg/s; joint acceleration 40 deg/s²; gripper
25 mm/s; settling 0.3 s; terminal trajectory dwell 0.2 s; arrival/binding 0.01 rad;
tracking 0.1 rad. No additional geometric overlap allowance or visual-model call.

## Offline evidence

Experiment root: `experiments/blocks-r9-ssik-speed-r2-20261117`.

- Current recorded run `web-main-20261117-134457-8cb5fe` and two historical
  layouts: 61/61 action groups pass real task geometry and native FCL/FK/trajectory
  checks. This is saved-state replay, not new motion or a grasp-success test.
- Current run, identical saved actions/feedback: preparation computation totals
  18.250 s before and 13.058 s after; planned arm-trajectory durations total
  63.233 s before and 60.667 s after. These exclude jaw strokes, whole-task initial
  search, real IPC/control latency and live execution.
- Green lift/transfer retains one curve across 51.2→51.1→51.0 mm feedback;
  yellow lift/transfer requires one full preview rather than two.
- Green approach's original −0.039698 mm task-envelope gap is not accepted as
  overlap. Lower tangent scale 0.35 passes the original geometry check and keeps
  the move in one trajectory.
- 21 real native prepare/stage certificate reuses pass without recompilation.
  Negative tests cover changed inputs, expiration, consumption, disabled owner,
  owner-generation change, scene/blend changes, anchor/jaw drift, connector
  dynamics and wire hash mutation. Owner/policy are offline substitutes; no SDK
  tick or hardware dispatch occurs.
- Real isolated frontend/MCP execution passes complete initial preflight and
  six planned batches with zero hardware dispatches. Search time is not claimed
  improved by this patch. Nine existing search regressions pass.

Full measurements and source hashes are sealed in `WEB-VALIDATION.json` at the
experiment root. `OFFLINE-NATIVE.json`, `offline/before.json`, `offline/after.json`
and the two historical `offline/after-web-main-*.json` contain raw evidence.

## Remaining qualification

The offline results demonstrate fewer redundant checks and shorter commanded
trajectories. They do not establish the actual live pause reduction or motor
tracking performance. A supervised new run must measure those before calling
this version physically faster. Web remains on the previous version until a
separate switch.
