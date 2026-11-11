---
name: manual-robot-control
description: Solve the task on the real {{ robot.name }} by choosing robot actions directly from observations. Use for solving the task by hand, recovery, and preparing the scene for script tests.
---

# Manual robot control

This is a real robot with people nearby. Every motion is physical and cannot be
undone; move slowly, in small inspectable steps, and stop when unsure. First learn how
to complete the task yourself through the robot tools, then automate what worked.

## First look

1. Read `TASK.md` and identify the required object relationships, orientation,
   placement and robot end state. Write a short completion checklist in your notes.
2. Call `status` and read `MCP_SESSION.md`. Inspect the robot with
   `{{ robot.status }}` and the scene with `{{ robot.observe }}`. If the robot reports
   that motion is not armed, do not try to work around it: the operator arms motion.
3. View the current camera images. Identify the relevant objects and a short sequence
   of manipulation stages.
4. Ground the next target in current observations with
   [object-localization](../object-localization/SKILL.md), choose a small action, and
   pick the motion tool for it as [robot-interface](../robot-interface/SKILL.md)
   describes.
5. Inspect the reply and new images, revise the next action, and save useful
   parameters and evidence. Work through the checklist until every condition holds.

## Observe and choose actions

Use these skills as needed rather than loading every reference up front:

- [Robot interface](../robot-interface/SKILL.md) for this robot's motion tools,
  frames, limits and conventions. Read it before composing actions.
{% if camera_geometry %}
- [Object localization](../object-localization/SKILL.md) for metric positions of
  visible object points from the current images and depth.
{% else %}
- [Object localization](../object-localization/SKILL.md) to locate visible object
  points from the current images.
{% endif %}
- [Wrist-camera inspection](../wrist-camera-inspection/SKILL.md) for a closer view
  with an empty gripper when the {{ robot.overview }} is insufficient.
- [Experience memory](../experience-memory/SKILL.md) to record and reuse tested
  motion strategies under matching observable conditions.

{{ block("images.forwarding") }}

## Motion and feedback

Distinguish commanded targets from measured states: the reply's measured pose tells
you what physically happened. Real tracking is imperfect; check the measured error
before the next step rather than assuming the target was reached.

Observe the result of each meaningful action and adapt. If a motion tool fails or
reports `control_uncertain`, do not repeat the motion blindly: observe, reason about
what happened, and continue only from a known state. If the robot refuses a target
(workspace or joint limit), preserve that boundary. A pure planning failure
explicitly marked `recovery_action=replan_read_only`, with no dispatch and no
control uncertainty, allows bounded read-only comparison of different legal
targets per robot-interface. It does not authorize motion, guard changes or
replaying the rejected target. Real feedback or execution faults still stop
physical exploration. If localization fails or
is uncertain, take a new view; never replace missing geometry with guessed
coordinates.

Batch independent read-only localization calculations, FK/IK candidates and
reachability comparisons in one short Context script when useful; save full
results on disk and return a concise candidate summary. Choose a meaningful,
bounded motion from the characterized tool range. Do not split an already checked
empty-space segment into tiny moves solely to add model turns. Inspect new
evidence at contact and task-stage boundaries; this is not permission to execute
an unobserved manipulation chain.

A failed task effect with healthy, known control (for example an object did not
follow a grasp) is different from a motion fault. Inspect what happened, restore a
known supported state when needed, revise a justified parameter and test again
within the task's bounded recovery procedure. Do not end the whole task merely
because one grasp failed. Unknown support, unsafe geometry or real control faults
still require resolving the uncertainty before dependent motion.

## Operator answers

Ask only for the specific fact that current tools and views cannot establish.
`request_operator` waits for an answer; after a timeout reuse its `request_id`
instead of creating a new question. `timeout_s=0` retrieves without waiting; avoid
repeated short polls that require extra model turns. New answers also appear as
`operator_updates` on the next robot tool reply (including errors), and recent
answers remain available in `status`. Read the actual reply, including negative
answers; `done` alone is not a positive confirmation. An answer supplies the
stated fact, never arming or fault clearance. Refresh state/observations after a
wait and apply it only to the question and state it describes. If the question
remains unanswered, do not perform the dependent motion. A timeout is neither
task completion nor a reason to discard a later answer and finalize prematurely.

## Confirm success

Partial visible progress or a clean tool return is not completion. Check every
condition in `TASK.md` against fresh images from more than one camera. Record the
successful strategy and evidence in `memory/`, then automate it as AGENTS.md
describes.
