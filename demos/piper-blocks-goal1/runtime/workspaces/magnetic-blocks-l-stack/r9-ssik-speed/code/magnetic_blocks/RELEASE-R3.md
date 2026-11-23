# c-Four-1.0 release r3

Recorded failure: yellow-release-28, 2026-11-16 16:12 run. Planned tilt25°, measured25.136028°, old limit25.1°. Arrival max joint error0.001213495rad; no release dispatch. Support estimate height error0.7405mm. Session cleanup caused the UI disconnection, not network failure.

Stationary contact jaw checks may use their own preflighted start pose, only within the unchanged0.01rad per-joint binding and1° full rotation discrepancy. Unreferenced planning and moving arm paths retain original orientation limits. Actual finger/jaw swept envelopes, full-arm collision/workspace, support evidence and receipts remain required. Closing alignment uses the same bounded reference. Error now records measured tilt and limit.

Real production sandbox replay: original failure reproduced; actual release geometry and retreat at72/73/74mm all pass. Motion tools forbidden. 34 tests +14 subtests pass. Full cold preflight26.87s,38 native-certified actions. The legacy test_c034.py still assumes3 colors: its8 fixture failures reproduce identically on unmodified r2 and are not claimed as passed. No robot actions performed. Live qualification pending.
