# Head + wrist evidence for this session

## Plan first, acquire the action checkpoint last

1. Before capturing a checkpoint, read the task, relevant memory and
   `references/workbench-experience/` (if present). Review historical success AND
   failure images, finish diagnosis, code/import checks and bounded action planning.
   Save `output/checkpoint-plan.json` with phase, relevant experience, proposed
   action, visual pass criteria and stop conditions. History is not current geometry.
2. Acquire new head/depth and wrist evidence only when ready for visual review.
   Check the current scene, alignment and measured anchor, then invoke the existing
   reviewed controller promptly. Its fresh pre-dispatch comparison still applies.
3. If the images reveal a need for further research, code edits or long analysis,
   complete that work, then replace the checkpoint with NEW images and review them.
   Never reuse an old yes/no decision with new frame IDs, relabel old timestamps,
   or increase the 120-second controller review limit / hardware freshness limits.
4. A pre-dispatch expiry has executed no motion: reacquire and review instead of
   repeatedly submitting the same stale request. A true execution/feedback fault
   remains a Hold and operator-recovery condition.

For the same blue cup wall-pinch task, retain the evidence-backed 19 mm opening /
4 mm closing targets described in the curated handoff, within the existing 10 mm
per-request jaw limit. Also retain its visual centering, below-rim pad engagement
and trial-lift checks. Do not copy historical absolute poses or infer grasp from jaw
arrival; failed attempts and incomplete retreat records remain labelled as such.

## Current multiview evidence

The workbench publishes read-only bundles in `runtime/stream/multiview/`. Use
`from code.rax_support.rax_multiview import observe, fuse` only if `code` is a package;
otherwise add `code/rax_support` to `sys.path` and import `rax_multiview`.
Call `observe(ctx)` at approach, precontact, grasp, lift, place and finish checkpoints.
Open both the returned **head and left wrist images**. Right view is supplemental.

Head: object identity, global position, route, cup/coaster relationship. Wrist:
fingertip alignment, rim engagement, slip, release and retreat. Judge the specific
facts listed for the current phase. Do not require tabletop support while carrying.
Record each view's frame path, `facts` (`yes`, `no`, `unsure`) and a short reason;
use `fuse(phase, evidence)`. Save results in `output/multiview-checkpoints.jsonl`.
Unknown or contradictory evidence is a request for a better observation, not a yes.
Do not claim 3D fusion: wrist extrinsics are not calibrated. For metric targets,
call the existing `piper_observe(require_depth=True)` and select pixels in ITS head
frame only, then use `piper_pixel_to_position`. The existing provisional transform
is accepted by the operator for this work; do not repeat a calibration project.

A bundle is a bounded-time collection, not a hardware-synchronized capture. Head
RGB/depth is exactly paired; wrist RGB is bracketed by owner timestamps. Use current
state and a fresh observation before motion, including after model inference.
No extra motion is authorized by a mask, a fusion result or a web connection.
`latest_segments()` reads an optional operator-requested SAM3 head/wrist snapshot.
Use its source image and mask for localization hints; it is historical evidence,
not a current clearance test. Missing segmentation does not replace original views.
Keep normal Rainbow-style primary-agent checkpoints. Preserve all corrections and
recoveries in the eventual program; do not only copy the successful motion sequence.

Each full trial must write `output/round-result.json` with `task_success` (boolean),
`retreat_verified` (boolean), `failure_phase`, `reason`, `checkpoint_frames` (list of
real saved frame paths) and `controller_sha256`. Software completion alone is not
physical success. Record the actual operator interventions and controller changes.

## Fast initial-only segmentation workflow

SAM3 identifies cup and coaster only at initialization. It never identifies the gripper.
The first `code/cup_to_coaster/checkpoint.py` runs text prompts once; later calls are
raw observations only. Initial low-score candidates remain explicitly uncertain and
need one visual identity check. Initial masks are historical references, not live evidence.
Call `step.py` with `target_rois` from current raw head/wrist views. These normalized
boxes only detect image changes before dispatch; they do not call SAM3. Preserve
freshness, measured anchors, exact-source depth and independent motion receipts.
Run each compact command and display its returned two frames in the same tool call.
Do not reread history or generate segmentation between reviewing and dispatching.
