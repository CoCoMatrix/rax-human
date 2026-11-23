# C-speed0.3.1: resumable preflight and early placement/withdrawal checks

Independent offline candidate. Original C-speed0.2.9 Web, C-speed0.3.0 candidate and prior frozen versions remain unchanged. No hardware motion, no Web switch.

## Changes
- Family search is a generator with explicit stage checkpoints. A slice expiration yields the unfinished stage; completed stages and loop positions remain in memory.
- Route search retains the candidate route index, completed legs, last joint endpoint and best valid route. If preferred-clearance optimization runs out of a family slice, keep the valid route already found.
- Near-finish stages receive at most two additional family turns; existing layout slices, fast-pass30s and global90s budgets remain. Full-pass retries revisit unresolved IK; this is not an unlimited extension.
- Placement pose, jaw opening and checked withdrawal are screened before pickup path construction, using the candidate rigid grasp relation. Actual pickup/lower joint endpoints rebind the finish contract; only identical scene, payload and joint endpoints reuse it.
- Native collision refusals invalidate derived transfer/finish/route caches. Genuine geometry and service errors are not classified as resumable budget exhaustion.
- State lives only in one Planner/run. A full KDL pass rebuilds unresolved candidate traversal and reuses exact solved/check results; it does not pretend a short-budget refusal proves infeasibility. No physical task auto-resume or hardware command replay is introduced.

## Validation
- 651 tests and52 subtests passed.
- Additional injected native refusal verified that transfer, finish and route-resume caches are invalidated.
- Current failed layout:17.53s, all33 action segments native-certified (34 native path calls including opening), blended-group previews pass. The former C-speed0.3.0 candidate timed out at90s.
- Three historical layouts:7.89s,14.33s,6.55s; full native and blended previews pass.
- Earlier implementation replay of current layout:17.54s; final replay17.53s. No new physical-layout/real-execution qualification is claimed.
- Prechecking exits adds work to some easy layouts; results do not establish a speedup for every placement. These four measured layouts are not exhaustive workspace coverage.

## Preserved controls
80% SDK speed,25mm/s gripper,existing joint tolerances,workspace,geometry margins,placement direction and native collision/trajectory checks are unchanged. No IK backend replacement, new model call or sampling-based planning framework was added.
