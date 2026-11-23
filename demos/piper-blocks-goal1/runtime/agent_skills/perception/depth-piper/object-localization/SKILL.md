---
name: object-localization
description: Localize objects in saved, exactly associated head RGB and FFS depth.
---

# Head RGBD localization

1. Call `piper_observe(cameras=["head","left","right"], require_depth=true)`. Head RGB and FFS depth come from one immutable stereo packet. Wrist images default to independently bracketed RGB. Left FFS is opt-in as described below.
2. Read returned camera `shape_hw` and `preview_shape_hw`. Convert preview coordinates into saved-image coordinates by the ratio of width and height; the stored head RGB is1280×720. Wrist HTTP previews may already be960×540. Never silently apply native intrinsics to resized pixels.
3. As in the Franka workflow, choose visible object surface pixels away from silhouettes/occlusion boundaries, then call `piper_pixel_to_position(frame_id, camera="head", pixels=[[u,v],...], radius=3)`. Default7×7 median valid optical depth rejects windows without real depth support. It does not fill holes. Reduce radius for thin parts only when adequate valid surface samples remain; a median that includes background is not an object measurement.
4. Returned positions are in **left_base_link**, metres, using the existing all13 head transform. They are provisional: cross-validation max error18.08mm, TCP uses the operator-measured140mm candidate; full independent metrology remains pending. A cup's visible surface is not automatically its center or contact point. Keep the reference frame and assumed geometry explicit.
5. Use `piper_fk`/`piper_solve_ik` for geometry and reachability, then current observations for the next action. Saved frames remain analyzable after model latency; their `age_s` is reported and never rewritten. New movement/scene changes invalidate affected assumptions, rather than causing a global replan solely because the saved analysis image is old.

No wrist hand-eye, synthesized fallback depth, or calibrated force/contact evidence is provided. The head depth engine is the already installed Fast-FoundationStereo model, not ZED depth or D435 hardware depth. This is the camera adaptation to Franka's localization workflow.

## Retain geometry through expected occlusion

A measured base-frame point can persist through one planned approach when the
object is stationary and fresh images/state show no contrary evidence. Keep the
original measurement frame/time/calibration and current observation separate.
Do not reuse old pixels on new images or relabel old measurements as fresh.
Expected robot occlusion is not itself object motion. Invalidate on changed scene,
contact that can move the object, uncertain control, owner/calibration change or
expired task scope. Check each proposed path; saved points do not prove clearance.

## Optional left wrist FFS

Request piper_observe(cameras=["head","left"],require_depth=False,
depth_cameras=["left"]). This uses left D435 serial262422071560's IR1/IR2 and
factory stereo-to-RGB calibration, not the head intrinsics. Use the returned native
shape and saved frame. Call piper_pixel_to_position(camera="left",frame_id=...,
pixels=[[u,v]],radius=3,output_frame="camera_optical"). Result frame is
left_d435_color_optical_frame, metres. Hand-eye is unavailable; base-frame requests
are refused. Camera-local depth does not by itself locate the tool or certify
clearance. A historical wrist point remains in its ORIGINAL camera pose; never
reuse it as a current camera point after wrist motion without a validated transform.
Right wrist remains RGB only. All depth uses LR-checked direct samples; no filled
holes or semantic model estimates replace depth. Current camera-only validation
establishes the pipeline, not near-cup accuracy or full grasp success.
