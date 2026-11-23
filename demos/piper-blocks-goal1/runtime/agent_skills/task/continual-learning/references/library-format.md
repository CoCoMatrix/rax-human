# Skill library format

```
skill-library/
  INDEX.md                    one row per skill
  skills/<slug>/
    SKILL.md                  frontmatter + the sections below
    code/run.py               the entry: main(ctx, **params) -> {ok, measured, why}
    code/<package>/           its modules: a package named after the slug (`-` -> `_`),
                              with params.py, the hyper-parameter list
    evidence/runs.jsonl       one JSON object per run of the code
    evidence/images/*.jpg     downscaled visual references
```

`INDEX.md` row: `| [<slug>](skills/<slug>/SKILL.md) | <status> | what it does | when to use |`.
Name folders by what they do, in lowercase words joined by hyphens.

## `SKILL.md`

```markdown
---
name: <slug>                 # equals the folder name
description: <what it does and when to use it>
status: draft                # draft | verified | deprecated
robot: <robot name>
entry: code/run.py
version: 1                   # bump when behaviour changes
uses: [<other-slug>]         # optional: library skills whose packages it imports
---

# <Title>

## When to use
Observable conditions (object kind and size range, where it is, the scene, the robot's
start posture), the variations tested, and when not to use it.

## Preconditions
What the code checks before moving, and how.

## Procedure
The steps in order, with their tool calls and why each parameter has its value.

## Code
How to call the entry; every parameter with its tested range and default; where the
hyper-parameter list is and which values to calibrate in a new setup; what `measured`
returns; where it logs.

## Checks
Each step's observable completion check and the final success check, with thresholds.

### Perception checkpoints
| Step | Frames | Question | Expected answer | Pass if | On fail |
|---|---|---|---|---|---|
(write "None" if the skill has none)

## Failure modes
Each failure seen: how it shows, how the code detects it, what it does, and what a
later session should do.

## Evidence
A table summarizing `evidence/runs.jsonl`, with images of the start and success states.
```

## The code

- Re-localizes everything it acts on from fresh observations.
- Checks preconditions first and each step before the next. When a check fails or is
  ambiguous it throws an exception, never failing silently: it ends in a safe state and
  returns `ok: False` with a clear `why` and the frames to look at.
- Reaches the robot only through `ctx` and reads the setup from it; keeps its
  hyper-parameters in one list; uses no paths outside the workspace.

## `evidence/runs.jsonl`

Append one line per run; never rewrite other lines:

```json
{"session": "<session>", "date": "<YYYY-MM-DD>", "commit": "<code commit>",
 "outcome": "success | failure", "unattended": true,
 "conditions": "<object, placement, robot start state>",
 "checks": {"<measurement>": 0.0},
 "images": ["evidence/images/<start>.jpg", "evidence/images/<end>.jpg"]}
```

`unattended` is true only when the code ran from start to end with no help (a scene
reset before the run is fine). Record failures too: they define the skill's limits.

## Status

| Status | Needs |
|---|---|
| `draft` | every section, an entry with `main(ctx, ...)`, at least one image |
| `verified` | also at least two unattended successful runs with images, ideally on different layouts or sessions |
| `deprecated` | the reason, and what replaces it |

A verified skill whose behaviour changes stays verified only if new unattended runs of
the new code meet the bar.
