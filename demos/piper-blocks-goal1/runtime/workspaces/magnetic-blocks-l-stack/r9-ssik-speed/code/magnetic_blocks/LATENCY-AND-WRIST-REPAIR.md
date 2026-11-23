# v0.3.1 independent blending test: wrist endpoint evidence and batched FK

Preserves the original v0.2.9 segmented runtime, frozen versions and v0.3's prior commit. Only the independent blending candidate and its two host API/contract files change. No robot test was started during this repair.

## Failure and evidence

Source run `web-main-20261112-154200-023190`: orange release and withdrawal completed, but the head camera could not see red. Wrist RGB clearly contains adjacent red/orange surfaces. The ordinary checker previously required red in the head view; the wrapper then obscured that reason with “auxiliary vision disabled”.

The revised orange retreat check combines the successful release receipt, actual placement FK and saved scene, orange at the expected fixed head location, and broad red/orange adjacency in the wrist's central region. No precise edge coincidence, area ratio, wrist-to-base coordinate conversion or Gemini is required. Incomplete release, changed owner, closed jaw, absent/clearly separated wrist regions or visible displacement remain unresolved/rejected. This is qualitative placement evidence, not force/support sensing. Green stacking uses its existing evidence semantics; a red/orange pair alone cannot certify green on red.

## Measured latency before this change

Entire interrupted main:104.830s; primary model0; Gemini0.

- Last opening SDK receipt to first empty-entry SDK receipt:24.039s.
- Trial to raise:8.386s; raise to transfer:7.151s.
- Explicit FK:2256 calls,20.125s total; empty-entry alone1411 calls,4.607s.
- Explicit path preflight:29 calls,10.480s; curve preview3.930s.
- Explicit observe:7 calls,3.132s. Receipt-attached observations are inside motion RPCs.
- Opening SDK receipt stream12.935s is commanded jaw execution, not idle function gap.
- Native previous-action acknowledgment is microseconds; no repeated Agent code-reading loop exists in this main.

## Optimization

`piper_fk_batch` is a pure bounded (512 samples/call) left-arm FK API using the exact same pinned Kinematics model. Sample loops in entry, insertion, payload paths and route search use it only in `main_blended`. All unique wire endpoints and midpoints remain checked; duplicated neighboring endpoints are evaluated once. No path tolerances, collision checks, sampling density, right-arm live feedback, 50% speed or0.3s settling were relaxed. The original default main retains scalar calls and segmented functions.

The existing collision adapter already reuses56 exact right/right pairs when right state, margin and geometry match. Left/right and environment collisions still depend on left motion and cannot be treated as static. This repair does not suppress them or substitute historical right feedback.

Logs now expose FK and curve-preview times separately, plus each last-SDK-receipt to next-first-receipt interval.

## Verification and limits

210 task tests,8 actual-evidence/backend regressions,47 applicable host tests passed. Five inherited blue-cup full-closure cases were excluded because that tool is not part of the blocks adapter. All2256 recorded FK transforms match exactly (max matrix difference0). A bulk offline evaluation used5 calls and about0.126s, which is not a measured live speedup: normal controller calls are grouped per path and actual RPC/collision/vision time remains. Next live test must measure full inter-motion gaps and task success.

Additional infrastructure finding: an old dedicated Chrome process produced61.33GiB of repeated Mojo/GPU errors. A1MiB sample was retained, repeated log content removed, and that browser relaunched with GPU/logging disabled and output directed to `/dev/null`. Robot controller, experiment receipts, video and frozen versions were untouched. This removes a potential I/O/CPU disturbance; its contribution to the previous run cannot be quantified retrospectively.

Evidence: experiment `repairs/latency-20261112/`; standalone no-network replay `verify_latency_real.py`; prior source/host copies in `repairs/latency-20261112/before/`.
