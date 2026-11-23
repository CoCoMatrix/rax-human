---
name: learned-magnetic-block-placement-boundary
description: Same-run orange table placement beside red, bounded partial-descent retreat and separate final-support verification.
---

# R7 placement boundary and recovery

- Applies when: approximately 60 mm upright blocks, installed R7 pads, verified orange trial retention, same-run table/grasp relation and unchanged red location. Front/back grip leaves the joining side exposed. Fresh native checks remain mandatory.
- Avoid when: green stacking, changed geometry/owner/calibration, unknown support, unverified payload, observed neighbor displacement, uncertain control or incomplete receipts. R7 had one supervised layout; no unattended qualification.
- Executed action: `transport`, `lower_to_support`, read-only `verify_completed_action`, `release`, `retreat`; exact native receipts/arguments remain in the linked reports. Release command 0.069 m reached 0.0686 m. Upward withdrawal was 0.090 m.
- Pose recipe: derive payload bottom from same-run supported grasp and table, place orange beside measured red. Nominal 60 mm spacing failed the sampled finger/red envelope; 65 mm succeeded in R7. These are observations, not generic clearances or new-scene targets.
- Actions: verify trial retention → raise/transport → one checked partial descent → final support descent → judge broad visible support → release in place → withdraw and verify stable separation. Never release merely because a template or budget ended.
- Recovery: a pre-dispatch final-descent refusal left a verified partial-descent endpoint. Reverse upward only to that function's prechecked retreat, preserving orientation and modeled neighbor separation. The observed joint interpolation bowed 1.12 mm; code bounds XY bow at 2 mm with independent neighbor-facing and monotonic-rise guards. Widen only after returning to height. Reject finger overlap.
- Endpoint lesson: stationary red entered the wrist patch during final placement, giving correlation 0.037 despite supported orange. Retention templates remain useful in flight; final support uses broad current views. Recover a known completed endpoint read-only, with owner/state/last-motion and source-hash checks. Preserve the original failure report.
- Expected effect: orange and red upright/adjacent on table, released and gripper withdrawn. Green remains unstacked. Confirmation is local `observed`, not whole-task confirmed.
- Code: `code/magnetic_blocks/controller.py`, `manipulation_checks.py`; closeout regressions in `test_r7_closeout.py`. Software changes after R7 are offline candidates. Geometry is provisional; no independent clearance metrology.

R8 adds one further supervised base-pair chain with the repaired planner:65mm spacing,4.19mm nominal minimum finger/red separation, successful final support/release/90mm retreat. Three auxiliary image calls, no operator assistance for the base pair. This strengthens only the base-pair lesson. New green lowering/support was disputed at release; green release and full L-stack are not confirmed. See `output/rehearsal/r8-base-plan-01.json` through `r8-retreat-01.json`, and `r8-stack-release-01.json` for the negative case.

For the separate R8 green support refusal and offline provenance repairs, see `../magnetic-block-stack-support-boundary/SKILL.md`. Do not extend this base-pair recovery to disputed green support.
