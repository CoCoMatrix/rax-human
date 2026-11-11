---
name: experience-memory
description: Save successful {{ robot.name }} action tips and retrieve them for future similar actions with matching observable conditions within this task.
---

# Experience memory

Memory under `{{ vocab.memory }}/` belongs to this task and persists across sessions.
Retrieve matching tips when controlling the robot or building automation. At start or
after compaction, read `{{ vocab.memory }}/INDEX.md` if present and retrieve only
matching learned skills. Open linked evidence when applicability or outcome is
uncertain; do not load the entire history. Keep learned procedures under
`{{ vocab.memory }}/skills/`, separate from the provided skills.

## Record successful tips immediately

- As soon as an action produces a visibly successful, useful result, add or update a
  concise row in `{{ vocab.memory }}/TIPS.md`.
- Keep evidence as file paths and short factual descriptions, not embedded images
  or copied tool transcripts. Retrieve only the few frames needed to resolve a
  current comparison. Update the current PROGRESS summary instead of appending
  the same plan after every tool call; CLI compaction preserves continuity through
  these files, not through repeatedly loading all historical images.
- Record the observable preconditions, task phase, tool and arguments or reusable pose
  relation, visible effect, and the saved frame paths. State what made it work so
  later attempts avoid repeating exploration or a superseded failure.
- Update the matching tip when the same maneuver is refined.
- The real scene drifts between attempts (objects are moved back by hand, lighting
  changes): record the conditions under which a tip held, not only its numbers.

## Distill

Promote a tip to `{{ vocab.memory }}/skills/<slug>/SKILL.md` plus `evidence.json` when
it shows a reusable motion, contact strategy, recovery, or completed procedure. Keep
procedures short; keep attempt-specific values and evidence links in `evidence.json`.
Update `{{ vocab.memory }}/INDEX.md` with the skill path, tags, confirmation level and
date. Current perception and the live tool schemas take precedence over a learned
skill; revise the skill when later evidence contradicts it.

Read [references/format.md](references/format.md) only when creating or updating a
learned skill.
