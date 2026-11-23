# v0.2.4 endpoint reuse repair

2026-11-12: web-main-20261112-101118-53cdb4 completed orange closure and trial lift.
The following raise received the same cached immutable frame as the trial endpoint.
LoadedChecks mistakenly called a two-frame comparison and stopped before dispatch.
There were zero Gemini calls in that failed run. This is a controller boundary bug,
separate from the later 10:47 host reboot and disconnected XQNetwork client.

## Change

Use retention_at_endpoint at loaded transport, orange lowering and green stack
lowering entry. Callers retain native completed-action/request, owner, state and
scene checks. Only a positive loaded endpoint with the same immutable observation,
matching held color and the existing 0.75-second receipt-cache age may be reused.
New frames still use the existing retention comparison. Actual motion outcomes
still require distinct ordered frames. Native IK/workspace/collision and payload
checks remain unchanged. No thresholds, speed or gripper settings are changed.
No forced capture, model call, automatic retry or motion recovery is introduced.

## Verification

155 offline tests passed with live Context/network forbidden. Historical failure
reproduced; same recorded boundary now passes with native MoveIt IK, installed
FCL collision, workspace and sampled payload/finger checks. Intervening action
still rejected. No physical run performed; this is not whole-task qualification.
Original v0.2.3 and frozen blocks/blue-cup assets remain unchanged.

Web points to this independent writable candidate. Packaging correction: the
runtime fixture symlink was rejected by native memory import. All 350 referenced
evidence files are now independent regular files with matching source hashes.
Native launcher dry-run and memory import into a fresh workspace passed. Web
validates source/evidence before releasing Prepare. No live task was run.
