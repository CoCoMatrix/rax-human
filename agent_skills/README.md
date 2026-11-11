# Agent skills

Every skill an agent can receive lives here, once. The workspace template
(`auto_research_agent/`) holds only the instructions and the agent-side MCP client;
`services/workspace/build.py` builds each new session's `.agents/skills/`.

## Layout: by axis

Each workspace takes the layers of its setup, in this order (`SKILL_LAYERS` in
`services/workspace/recipes.py`); a later layer adds to or replaces files of the same
skill, and an absent layer is skipped:

| Layer | Contents |
| --- | --- |
| `task/` | `manual-robot-control`, `autonomous-control`, `experience-memory` |
| `perception/common/` | `live-stream`, `wrist-camera-inspection` |
| `perception/<mode>/` | perception for one observation mode, any robot (none yet) |
| `perception/<mode>-<robot>/` | `object-localization` for that mode and robot (`depth-franka`, `rgb-only-franka`) |
| `control/common/` | control for any robot (none yet) |
| `control/<robot>/` | the robot's `robot-interface` and motion skills (`franka`) |
| `auto-research/` | `gemini` |

`<mode>` is `depth` or `rgb-only` (from the observation profile) and `<robot>` is the
backend key (`franka`). Agents see one flat `.agents/skills/` with stable, robot-neutral
names, so task skills link to `robot-interface` or `object-localization` without
branching on the robot or the observation mode.

Robot-specific facts a shared source needs come from `robot.*` (`ROBOT_FACTS` in
`services/workspace/setup.py`): tool names, cameras, the overview camera.

## Templates

Every `.md` is rendered with Jinja (`services/workspace/templates.py`) against
`Setup.context()` plus `vocab.*`; unknown names fail the build. Reusable passages are
registered blocks in `agent_blocks/blocks.toml`, inserted with `{{ block("<id>") }}`.
