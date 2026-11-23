---
name: continual-learning
description: Read, run and grow a persistent skill library of verified, executable sub-task skills that carries what agents learn across sessions and tasks on the same robot. Use at the start of every session, and contribute and merge before the session ends.
---

# Continual learning: the skill library

Where the harness provides one, `skill-library/` is a git clone of this robot's skill
library: sub-task skills that earlier sessions learned, each an executable procedure
with evidence that it works. You work on your own branch; the harness's merge tool
(`skill_library_merge`) checks your branch and merges it into the library, where the next
session finds it.

| Where | Written by | Holds | Lifetime |
|---|---|---|---|
| `.agents/skills/` | the harness | how to use the robot, tools and workflow; read-only | every session |
| `memory/` | you | this task's notes, attempts and frame links | this task |
| `skill-library/` | agents, through merges | verified, executable procedures | every later session |

The harness skills and the live tool schemas take precedence: a library skill never
changes a limit, a safety rule or what a tool does.

## At the start: read and reuse

1. Read `skill-library/INDEX.md`, then fetch and merge the library's main branch to
   pick up anything merged since your workspace was made.
2. Load only the skills whose "When to use" matches a part of `TASK.md`, plus the
   skills they declare in `uses:`. Prefer `verified` over `draft`; read `deprecated`
   only to learn why it was retired.
3. Check a skill's preconditions against a fresh observation and its tested ranges
   against your scene before running it. In a setup it was not tested in, calibrate the
   values its hyper-parameter list marks for calibration.
4. Run its entry, read the returned `{ok, measured, why}`, and confirm the result with
   your own observation. In a scene its evidence does not cover, look at its perception
   checkpoints yourself and adapt the code where the scene differs, until its own checks
   have proved reliable there. A failure here is information: record it as a run.
5. A skill may predate the current tools. Where a newer tool does a step better,
   upgrade the skill on your branch, bump its version and re-verify it.

```python
import importlib.util, json
from api.runtime import Context

spec = importlib.util.spec_from_file_location("skill", "skill-library/skills/<slug>/code/run.py")
skill = importlib.util.module_from_spec(spec)
spec.loader.exec_module(skill)
with Context() as ctx:
    print(json.dumps(skill.main(ctx, **params), indent=1))
```

Your controller may run library skills the same way, or put a skill's `code/` on
`sys.path` and import its package.

## What to contribute

- A skill for each procedure that **worked on the robot** and is useful beyond this
  exact scene: one sub-task with a clear start and end state.
- Improvements: wider tested conditions, better parameters, a new failure mode and how
  the code now detects it.
- A `draft` of a procedure that is automated but not yet verified, so the next session
  can finish it.
- Deprecation of a skill that proved wrong or was replaced, with the reason and its
  replacement. Never delete another session's skill or evidence.

## Make skills composable

- Cut a task into primitives (perception, positioning, grasping, transport, placing,
  release), split where the robot is in a stable, describable state, never mid-contact.
- Give each primitive a contract: inputs as parameters in base-frame units with tested
  ranges; the start state it checks and the end state it leaves (the next primitive's
  start state); its `{ok, measured, why}` return; and a safe stop on failure with
  enough in `measured` to retry or recover.
- One function per primitive, `primitive(ctx, ...) -> dict`, with no globals or hidden
  state. Import other library skills instead of copying them, and declare them in
  `uses:`. Keep a changed primitive's signature and contract, or add a new function.
- Prefer the robot's own tools (IK, planners, Cartesian moves) inside primitives over
  hand-written solvers.
- Expect reuse on the same robot with other camera setups, in other environments and
  for other tasks. Keep the manipulation logic in the skill, with the motion in the
  object's frame and the scene description as input from a perception function callers
  may replace; read the environment setup (camera parameters, the robot's postures,
  limits and workspace) from the harness at run time, never from values your session
  saw, and perceive what the harness does not report; say which setups and variations
  the evidence covers.
- Put the modules in a package named after the skill (the slug with `_` for `-`), importing
  each other relatively; `code/` holds only the entry and stand-alone scripts. Python keeps
  one module per name in a process, so plain module names from two skills collide. To
  reuse a skill, list it in `uses:`, have your package's `__init__.py` put its `code/` on
  `sys.path`, and import its package; never load its files by path.
- Keep the hyper-parameters in one list, separate from the logic, in the package's
  `params.py`, each with what it controls, its unit, its tested range and how it was set.
  Mark the values that depend on the setup with a `# CALIBRATE:` comment saying how to
  measure them, so a new task or setup knows what to check and where to adjust it.
- Leave room for the agent's perception: each primitive re-observes what it acts on,
  returns the frames it used, throws an exception instead of guessing when a check fails
  or is ambiguous, and documents its perception checkpoints for running it in a new
  scene ([autonomous-control](../autonomous-control/SKILL.md)).

## Format, evidence and merging

Follow [references/library-format.md](references/library-format.md) for the folder
layout, the `SKILL.md` sections, the run log and the status rules. Commit on your
branch, merge the library's main branch, then call `skill_library_merge`; fix every problem
or conflict it reports and call it again. Merge when a skill is first automated (as
`draft`), when it becomes `verified` or changes, and always before the session ends.
