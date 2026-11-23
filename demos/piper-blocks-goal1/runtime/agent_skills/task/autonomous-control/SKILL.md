---
name: autonomous-control
description: Run and debug Python robot control, then turn what worked by hand into an end-to-end autonomous controller and test it on the real scene with operator resets. Use for closed-loop scripts, switching between your script and direct control, and test runs.
---

# Autonomous control

Follow the workflow in AGENTS.md. The robot and Gemini are external services,
reachable only through MCP. The deliverable is a controller that completes the task
from a fresh scene **without you in the loop**.

## Start from your successful manual run

Read the saved successful strategy and decisive frames in `memory/`. Create a project
under `code/<project>/` and turn one useful maneuver into a Python function that
receives `ctx` and explicit inputs. Test it on the current scene using the ordinary
Python example below. Inspect its final images and logs, correct it, and repeat before
composing the full `main(ctx)`.

Separate reusable motion logic from scene-specific inputs: reacquire object locations
from current observations instead of embedding one layout's positions.

## Turn successful control into automated scripts

After succeeding through direct robot control, reproduce the decisions that made it
work in an automated observation–decision–action script that runs without you. Use
current observations to adapt to new layouts, rather than blindly replaying one run's
actions.

- Hard-code reusable motion functions and decision rules on the MCP tools
  (`{{ robot.move }}`, `{{ robot.joints }}`, `{{ robot.gripper }}`) when suitable;
  [robot-interface](../robot-interface/SKILL.md) has the frames and limits.
{% if gemini %}
- Use Gemini when simple perception rules are insufficient. Write your own prompts and
  response-processing code; see the [gemini skill](../gemini/SKILL.md).
{% endif %}
{% if vision %}
- Use [codex-vision](../codex-vision/SKILL.md) when simple perception rules are insufficient.
{% endif %}
- Practise and debug functions on the current scene, using direct robot calls to
  rearrange objects into the test conditions you need where that is safe, rather than
  asking for a reset every time.

Combine these approaches as needed, and test the resulting closed-loop script on the
current scene before the end-to-end test runs below.

## Pressure-test across layouts

Do not stop at one success. The operator can reset the scene to new layouts on
request; reacquire objects and adapt approach poses instead of reusing layout-specific
coordinates. Refresh observations after anything moved. Record each tested layout,
outcome, failure and fix in memory; you stepping in is debugging, not an autonomous
success. Add observation-driven recovery for expected perception or grasp failures
(re-observe, re-localize, retry from a safe height). Do not claim untested
generalization.

## Code you can read and resume

- Split the pipeline into steps, each with an observable precondition and completion
  check.
- Give each step a function `fn(ctx, ...)` that checks its preconditions from fresh
  observations, acts, verifies the result, and returns a small dict with `ok`, what it
  measured and, on failure, why. It returns instead of improvising when the scene is
  outside what it was tested on, leaving the robot in a safe state (gripper not
  dropping a held object mid-air, arm lifted clear).
- Make the pipeline resumable at any step, and give every motion that can fail a
  fallback.
- Keep tunable values as named constants at the top of a module, with the range you
  tested.

`controller.py:main(ctx)` chains the same steps end to end and stops at the first one
that fails, returning a report. `code/<project>/README.md` says how to run the whole
controller and each step from ordinary Python, with their preconditions; record
reusable tips as [experience-memory](../experience-memory/SKILL.md) describes.

## Ordinary Python

Python runs in this workspace with normal files, stdout and stderr. Write reusable
functions against a supplied context `ctx`; the entrypoint is the same for single
steps and full runs:

```python
# code/stack/controller.py
def main(ctx):
    reply = ctx.value("{{ robot.observe }}")
    # Your observation–decision–action loop, e.g.:
    # ctx.value("{{ robot.move }}", position=[x, y, z], quaternion=q)
    # ctx.value("{{ robot.gripper }}", action="grasp", width=0.0, force=30)
    return {"ok": True}


if __name__ == "__main__":
    import json
    from api.runtime import Context
    with Context() as session:
        print(json.dumps(main(session), indent=1))
```

Run it from the workspace root (`python code/stack/controller.py`, with stdout and
stderr redirected to a log under `output/runs/<timestamp>/`). `main(ctx)` and its
helpers use the supplied context unchanged, passing `ctx` down: no new `Context()`.
`Context()` connects to the same robot service as your direct tool calls. Return
whenever you want to inspect images and choose new actions yourself; the scene keeps
the script's actions.

Robot calls run one at a time, but a script does not hold the robot: other calls can
come between its calls. Run one controller at a time, and stop it before acting
directly. Interrupting Python does not cancel a motion already dispatched: after an
interruption or an ambiguous motion error, check `{{ robot.status }}` and a fresh
observation before continuing.

## MCP and files

`ctx.call(tool_name, **arguments)` uses the live tool schemas and returns MCP content
blocks; `ctx.value(...)` decodes the JSON text block. Images remain image blocks.
Inspect current tool schemas (`MCP_SESSION.md`) instead of inventing arguments. Calls
reporting an MCP error raise `RuntimeError` with the error JSON (stage, reason,
`robot_state` where the arm stopped); handle refusals in code rather than blindly
retrying the same motion. Observations save full-resolution frames under
`{{ vocab.frames }}<frame_id>/`; save script replies, decisions and prints to files under
`output/` for debugging.

## End-to-end test runs

There is no formal evaluation: you judge the controller by running it end to end on
the real scene.

1. Bring the robot to a neutral state following [robot-interface](../robot-interface/SKILL.md)
   (never open a gripper holding an unsupported object), then call `request_operator` with kind `reset` and say what you need, e.g.
   "please put the cubes back at new positions on the table". Wait for the answer.
2. Commit the controller version (`git` inside `code/<project>/`) and run it once,
   unattended: no direct robot calls while it runs. If it fails, read its report, log
   and frames, fix the code and go back to step 1; do not hand-finish a run and count
   it as a success.
3. Record each run in `memory/RUNS.md`: code version, scene description, outcome,
   duration, failure step and cause.

Aim for several consecutive clean successes on varied layouts before calling it done.
Use `request_operator` kind `help` when you are blocked by something outside your
control (an object out of reach, a robot fault, unclear task wording).
