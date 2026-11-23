# C-speed0.2.7 — phase-aware placement evidence

The 2026-11-16 orange-first run completed support, open release and retreat, then failed because the head target was hidden and the wrist check demanded both red and orange. Red had not yet moved. This was a task result rejection followed by the normal Stop/disarm, not a network disconnect.

Current-stage evidence now accepts the recognizable placed body in head OR wrist, combined with verified support, open-jaw release, same-owner complete motion receipts and distinct measured retreat. Neighbor appearance is optional corroboration only when the current scene already places that neighbor nearby. No future assembly relation is required at an intermediate endpoint. The legacy withdrawn placement/recovery path uses the same evidence helper and no longer requires head-only visibility or a visible neighbor. Support height, image binding, receipt validity and motion/collision checks are unchanged; unknown views still require inspection and completed actions are never replayed.

Audit: assembly_controller uses this endpoint for red/orange/green. Legacy after_release uses the shared placement check. Remaining fixed-order text in old rehearsal prompts belongs to the original fixed task sequence and is not invoked by C-speed main; it is not a new condition on this independent candidate.

Validation: 588 tests + 52 subtests; actual failed frame replay reproduces old rejection and passes new rule without image-function mocks, using original capture time and pinned native FK. No network, motion or model calls. Bad image binding still rejects. 12 new tests cover colors, recovery, missing support, closed jaw, incomplete/wrong-owner receipts, no retreat, bad binding and absent visual evidence. Earlier planning-layout validation belongs to parent0.2.6: planning, motion host and geometry are unchanged except version string. No new live-success claim.

Independent working copy and git archive; parent0.2.6 and frozen source hashes verified unchanged. 80% arm speed,25mm/s gripper and all physical limits retained.
