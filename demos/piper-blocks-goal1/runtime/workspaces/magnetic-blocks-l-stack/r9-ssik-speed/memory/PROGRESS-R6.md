# R6 active — base pair placed, arm open/clear

Latest completed physical action r6-retreat-01: upward from~205 to300mm after supported orange release; jaw68.5mm; control healthy. Wrist shows red/orange side-adjacent, upright, fingers separated. Endpoint verification r6-retreat-verify passed READ-ONLY; do not repeat retreat. Green remains separate. Need fresh scene/active view to re-localize red after magnetic contact before green stacking. Current source coordinates for red/orange invalidated by placement; no historical reset target reuse.

Reports output/rehearsal/r6-*.json, inputs output/r6-inputs/. Latest endpoint frame9ca645ad961f48a3ada6dfaefdeea2f6. Native budget ~3964s at physical retreat; use status actual clock next.

Completed R6 functions: open,2empty transitions,approach/insertion,50mm closure,10mm trial(actual9.334mm),85mm raise,lateral transport,intermediate descent,final table descent,release,retreat. One operator support fact request9864a2c233f8 confirmed:已由桌面支撑，两块直立，夹指未卡住，可以原位张爪. Reply was late but fresh pose/owner/last-motion checks validated it; consumed only for release. No physical manual reposition assistance, noGemini yet. This is supervised rehearsal, NOT autonomous whole-task success.

Fixes: RGB-only missing finger projections now use pinned source envelope transformed viaFK at same jaw.54tests passed prior to later additions. New manipulation_checks.py adds loaded/path,lowering,supported-release,retreat checks. Retreat area gate falsely failed due clipping; replaced with visible body width decrease,verified endpoint without motion. Final lower endpoint wrist template was occluded by red; report kept needs_review,operator supplied missing support. No movement replayed.

Next: empty inspect move/transition to reveal both base blocks, current RGBD localization; green approach/grasp/trial/stack functions still need rehearsal. Do not compose unattended main yet. Final60s reserve:stop motions,save notes;Prepare host only. Keep task evidence in memory, library shared draft not a qualification.
