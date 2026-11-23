---
name: learned-magnetic-block-stack-support-boundary
description: Preserve a disputed green-on-red support boundary and provenance without claiming release or task success.
---

# R8 disputed stack support boundary

- Applies when: retained green has reached a modeled red-top endpoint, but subsequent current support evidence contradicts an earlier positive assessment. This is an observed refusal boundary, not a successful stacking procedure.
- Avoid when: treating historical coordinates/images as current targets, interpreting model agreement as measured force, using wrist optical depth as base XYZ, or assuming an unanswered operator question authorizes release.
- Executed action: R8 `green_stack_lower` issued one checked pose move; the next `green_stack_release` issued **zero motions**. Its disputed-support result is preserved. No green release/withdrawal is evidenced.
- Pose recipe: same-run measured green grasp/table relation and newly measured red/orange support positions, with verified green-only action history. Red had shifted about 6 mm after base placement. Historical numbers are not new targets; nominal clearance does not establish support.
- Actions: retain both the earlier positive lowering assessment and the later negative release assessment; stop the release path. Do not replay lowering or retry from the older report to obtain a different verdict. Keep the payload controlled. In any future authorized physical session, resolve support from new current evidence and appropriate onsite inspection first.
- Expected effect: avoid unverified opening; no claim that green is stably supported or the L-stack complete.
- Recovery: none demonstrated for this R8 discrepancy. Generic completed-endpoint recovery intentionally does not handle green stack stages. Operator request `ff43deacefc7` was unanswered in the final historical record.
- History handoff: after orange release/retreat, retain its old empty-approach chain as provenance only; start a new green approach chain. The explicit new root worked in R8; automatic history reset has offline regression coverage, not a separate physical replay.
- Software refinement: support refresh now pins its green pregrasp source; subsequent consumers reject changed/bypassed retained histories. Legacy refreshes without this binding require fresh evidence in a future session. New checks are offline candidates. There is no persistent cross-call dispute latch; orchestration must preserve the negative result.
- Evidence: `evidence.json`, original `output/rehearsal/r8-stack-lower-01.json` and `r8-stack-release-01.json`, `output/r8-handoff.json`. Current phase is software only; this skill grants no motion authority.

Web packaging preserves original evidence metadata and reports at:
- references/web-evidence-original/magnetic-block-stack-support-boundary/r8-stack-lower-01.json
- references/web-evidence-original/magnetic-block-stack-support-boundary/r8-stack-release-01.json
- references/web-evidence-original/magnetic-block-stack-support-boundary/r8-support-refresh-01.json
- references/web-evidence-original/magnetic-block-stack-support-boundary/r8-retreat-01.json
- references/web-evidence-original/magnetic-block-stack-support-boundary/evidence.json
