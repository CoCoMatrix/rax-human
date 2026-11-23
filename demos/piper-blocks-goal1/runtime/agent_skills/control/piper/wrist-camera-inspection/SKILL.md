---
name: wrist-camera-inspection
description: Inspect left and right Piper wrist RGB at manipulation checkpoints.
---

The cameras are named `head`, `left`, `right`; `left` and `right` are wrists, not fixed external cameras. There is no `wrist` camera name. Use `piper_observe(cameras=["head","left"])` for a left-arm grasp, adding `right` when its view resolves occlusion. Inspect finger/object relative motion across a bounded lift, slippage, support before release, and separation after release. Do not certify grasp merely from jaw closure or IK success. Left wrist has optional FFS optical depth (see object-localization); right wrist is RGB only. Neither has validated hand-eye, so use head RGBD for base-frame geometry. Use the enabled live-stream recording for between-frame events; two snapshots alone cannot prove them.

Follow the upstream active-view workflow: with an empty gripper and a verified clear
path, use a short ordinary motion to obtain a different physical view when the head
is occluded. Keep the fingers clear of the cup/rim/table while inspecting. Inspect
the returned views after each move; this is not digital zoom and not permission to
move through an unresolved gap. Do not repeatedly ask the same static images to
resolve projection overlap. A tiny gap hidden in both views remains unknown.

Inspect the saved 1280x720 images. The RAX owner requests native width and JPEG
quality 95; validate actual shape metadata, since old runs used 960x540 UI images.
Full resolution cannot reveal a surface hidden behind the gripper. Re-localize the
cup from visible head RGBD at an unobstructed inspection pose before entering the
occluded contact region. Retain that measured base-frame target through this same bounded approach when fresh scene/state checks remain consistent. Do not require remeasurement of the now-hidden rim at every segment. Do not move to a previous-trial pose or synthesize missing depth.
