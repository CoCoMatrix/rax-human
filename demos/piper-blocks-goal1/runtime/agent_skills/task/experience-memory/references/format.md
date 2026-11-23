# Learned memory format

Store learned skills only below `{{ vocab.memory }}/skills/`, each under a lowercase
hyphenated slug.

## `TIPS.md`

One row per reusable local result; update a matching row instead of adding variants
of the same lesson:

```markdown
| Task phase | Preconditions | What worked | Visible effect | Evidence |
|---|---|---|---|---|
| <phase> | <observable conditions> | <tool and arguments> | <visible change> | <saved frame path> |
```

## `INDEX.md`

```markdown
| Skill | Tags | Confirmation | Last confirmed |
|---|---|---|---|
| `skills/<slug>/SKILL.md` | <tags> | observed | <date> |
```

`Confirmation` is `observed` for a clear local visual effect, or `task-confirmed`
when the procedure contributed to an attempt in which every `TASK.md` condition was
checked in fresh images.

## Generated `SKILL.md`

```markdown
---
name: learned-<slug>
description: <maneuver and when this experience applies>
---

# <name>

- Applies when: <observable conditions>
- Avoid when: <observable mismatch>
- Executed action: <tool and exact arguments>
- Pose recipe: <anchor, frame, offset>
- Actions: <compact ordered procedure>
- Expected effect: <visible result>
- Recovery: <evidence-backed alternative>
```

Do not copy tool documentation or global limits into a learned skill, and do not
encode inferred task success or an unobserved object pose.

## `evidence.json`

```json
{
  "schema_version": 1,
  "skill": "learned-<slug>",
  "examples": [
    {
      "task": "<task>",
      "date": "<ISO date>",
      "confirmation": "observed | task-confirmed",
      "frames": ["{{ vocab.frames }}<file>.png"],
      "executed_action": {"tool": "<tool>", "arguments": {}},
      "pose_recipe": "<anchor, frame, offset>",
      "point_reacquisition": "<visible feature, offset, {% if camera_geometry %}the `{{ robot.pixel }}` call{% else %}the RGB alignment procedure{% endif %} and preferred views>",
      "observed_effect": "<visible change>"
    }
  ]
}
```
