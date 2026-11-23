# Imported observations — reacquire the current scene

| Task phase | Preconditions | What worked | Visible effect | Evidence |
|---|---|---|---|---|
| Grasp | Same blue cup; opposing pads actually straddle the cup wall below the rim, handle clear | Historical opening about19mm; closure target4mm, approached in bounded increments | Operator previously observed cup retained without falling; actual loaded gap may differ | skills/blue-cup-wall-grasp/evidence.json |
| Retention | Cup follows pads and contact stays stable | Small initial lift, inspect head/wrist before further lifting; historical40mm lift succeeded | No slip in historical trials; not guaranteed in this run | skills/blue-cup-wall-grasp/evidence.json |
| Approach | Final grasp pose not yet reached | Check both final translation and orientation after segmented moves | Avoid treating an intermediate waypoint as final alignment | references/history-lessons.md |

Same19/4mm values previously also produced empty grasps. They are conditional experience, not proof of capture. Use fresh observations, never old world coordinates. Record measured width separately from command width.

## Current tool measurements and rehearsal lessons

- User confirmed A=left_link6, both effective pad centers at axial140mm, black tip at170mm, no offset or tilted mounting. Use the installed operator-measured TCP metadata from status/FK; independent physical accuracy remains unverified. Cartesian actions still target the flange. Historical110mm and CAD137.29mm do not replace these measurements.
- A new exploration starts with editable, empty task code. Do not load or replay the v0.1/v0.2 controller or historical scene coordinates. Derive short functions from current evidence; keep code writable.
- When assessing an opening, specify current gap, intended gap and per-finger travel. Use current geometry and relevant images; a single image cannot demonstrate temporal scene stability. Supply the reference frame for before/after comparison instead of repeating a judgment on the same insufficient input.
- Prior onsite confirmations were specific to their owner, pose and scene. They expired on the subsequent XPolicy Prepare; none authorizes this new session's actions.
