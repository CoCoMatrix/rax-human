---
name: live-stream
description: Current Piper observations when resuming or inspecting a pause.
---

# Current views

This adapter records the existing owner's three RGB cameras, robot feedback and available head depth into the native RAX `runtime/stream/` layout. `piper_status.recording` and `index.json` report availability, errors, actual rates and newest timestamps. Requested rates are RGB 5 Hz, robot 10 Hz, depth 1 Hz, with a ten-minute rolling window; never claim the requested rate was achieved without checking. Disabled or failed recording is not evidence of an unchanged scene.

Use `python -m api.stream latest`, `sheet left --seconds 30 --count 8`, or `robot --seconds 30`. `piper_stream(camera, since_ms, until_ms, count=8)` returns a contact sheet directly. The stream is read-only to agent code. It supports reviewing change, slip and scene disturbance between endpoints; it is not a continuous collision proof. HTTP camera timestamps bracket acquisition, while each saved head depth PNG has an exactly matching RGB. Depth PNG is uint16 millimetres, zero invalid. Use `piper_observe`/`piper_pixel_to_position` for calibrated metric analysis.

Use `piper_observe(cameras=["head","left","right"])` to inspect the current scene after a pause, or request selected cameras for a focused check. `require_depth=false` requests RGB only; in RGBD sessions head defaults to its exact FFS RGB/depth packet. Images are saved under `runtime/frames/<frame_id>/`; original source timestamps remain in metadata.

A snapshot is not a recording of the intervening motion. Review the relevant recorded interval when available. Gaps, occlusion or a missing stream remain uncertainty; do not invent video evidence.
