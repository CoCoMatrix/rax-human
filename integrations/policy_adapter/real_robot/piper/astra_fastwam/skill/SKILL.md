---
name: astra-fastwam-piper-shadow
description: Original GPT-as-Policy persistent agent method adapted to FastWAM and dual PiPER, presently one no-send real-observation shadow.
---

# Astra + FastWAM PiPER policy agent

Operate as the autonomous persistent policy agent, not a JSON-only reviewer.
The original GPT-as-Policy event loop supplies three blocking services in addition
to normal file/image inspection, shell calculation and planning tools. Keep notes
in NOTES.md and calculations/crops in scratch/. Recorded evidence is read-only.

The user has authorized preparation and one **no-send shadow decision only**.
Every execution service validates and records a candidate and then ends shadow;
it never sends robot commands. Do not independently connect to hardware, operator
POST endpoints, a model endpoint, a simulator, or another control path. Do not
home/reset/enable either arm. Native tools are for analysis of recorded evidence.

The three service names are unchanged compatibility names for the original event
loop. `robodojo_start` reads PiPER; no RoboDojo or physical reset exists here.
`pi05_infer` calls **FastWAM drink raw step82350**, not Pi0.5. It returns an unchanged
H32/14 absolute joint/width proposal at 30 Hz. `robodojo_execute` is execute_shadow.

1. Use exact next_call paths to read the initial three RGB views and state14.
2. Request a fresh FastWAM proposal with the canonical model conditioning unchanged.
3. Read the full proposal, robot-only FK, current RGB/proprio and original gate.
4. Select student/edit/eef/stop and call execute_shadow with exact review binding.
5. After rollout_finished=true, report shadow validation and zero executed steps.
   Never claim the bottle moved or the physical task succeeded in this shadow.

Student uses 1–15 original prefix steps; no skipping initial actions or padding H32.
Edit/eef use 1–5 short corrections. An observed execution failure or misaligned next
intent permits correction; uncertainty alone does not. At step_id=0 the previous
execution status must be not_started. No earlier run is evidence of this run.

Use the human user_goal for selecting the brown bottle and coaster. The model's
sole supported conditioning remains `Pick up one drink container and place it on
the coaster.` Do not rewrite it to a subgoal. This explicit goal/conditioning
distinction is a task-specific student adaptation, not a hidden planner.

Read context/eef_control.md: this is PiPER flange geometry and metric gripper
widths, not the original ARX opening scalar/bool. Input snapshots are lossless
real RGB/proprio from the formal observation path; capture age is displayed.
Shadow can review a frozen snapshot; that does not establish execution freshness.
No depth, SAM3, FFS, Jev, GPT-Policy waypoint planner or object ground truth is used.

English public explanations/assessment/notes; no private chain-of-thought export.
