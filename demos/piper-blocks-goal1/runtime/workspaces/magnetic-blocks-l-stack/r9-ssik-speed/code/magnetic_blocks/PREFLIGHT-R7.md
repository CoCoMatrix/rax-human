# c-Four 1.0 preflight r7

Independent candidate derived from retreat-r6; original and frozen controllers preserved. The live main still measures a fresh scene. Offline scene files are evidence, never a live target source.

## Diagnosis
The180919 Web attempt spent151.68s in preflight and was stopped before a task motion. Its final BrokenPipe followed control shutdown. Replaying the recorded four-block scene with r6 found a complete native-certified plan in156.016s; the scene was not proven unreachable. Old search made10522 IK RPCs. It repeatedly expanded detours to poor retreat endpoints and favored fixed-90deg yellow grasps while departing a newly built green stack.

## Changes
- Across the unchanged retreat endpoint portfolio, try direct checked connections first; preserve the original routed alternatives. Every chosen retreat still validates both72/74mm jaw bounds, the next free-flight endpoint, and native collisions.
- For an empty down-facing hand changing heading beside a stack, add current-orientation upward clearance portals before the turn. Candidate height derives from modeled stack tops and finger envelope, with workspace bounds. Original XY portals remain; every actual edge is checked. These are options, not an unconditional lift.
- Rank all yellow jaw-axis grasp variants by incoming orientation continuity instead of a fixed-90deg preference. Keep the established25deg/high-grasp preference and all alternate poses. No coordinates from this failed run are hardcoded.
- No changes to collision margins, workspace, IK solver, motion speed, tolerances, grasp/close/release rules or stop contracts. Released-target AABB comparison remains1mm with the original scope; native collision remains mandatory.

## Verification
- Current recorded scene: r6 156.016s; candidate25.836s and final-repeat25.773s. Final repeat37 native-certified actions; all5 proposed motion batches passed preview. IK RPCs1710 (1295 single fast+415 candidate batches), versus10522 in r6.
- Injecting3ms per IK RPC:60.958s, complete native-certified plan. Wall-time slices change search order, so this is a robustness test, not a claim of constant25s runtime.
- Historical successful scene:14.459s,38 native-certified actions (previous r6 evidence26.815s).
-60 regression tests+4 subtests passed both on host and in the production bubblewrap runtime. New regression tests fail on r6 (6 failed,1 passed). Static undefined-name scan clean.
- All replay uses production Frontend/Session/Unix RPC and actual FK/IK/native checker, with owner access forbidden and a pure-tool whitelist. Zero hardware dispatches; no live qualification claimed.
- A broader legacy suite still contains3-color fixtures incompatible with c-Four (29 fixture-related failures); these are retained and excluded from the60-test qualification, not represented as passing.

Evidence paths and timing records: /home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blocks-c-Four10-fast-20261116/preflight-r7. This candidate changes planning choices, so the next live test must start with fresh localization through the Web's normal execution entry.
