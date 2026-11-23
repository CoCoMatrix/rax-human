# C-speed0.2.1 — single-move scope and rejection reporting

Source failure: web-main-20261113-164307-a7fe0f, red-hover-2. Planning21.217s passed; empty-open6.251s completed153SDK samples; red-hover rejected before arm dispatch1.492s. J2 target delta95.271332573deg; J3/4/6 about81deg. Backend _execute still enforced60deg although planning, owner and multiwaypoint allowed120deg. This was an integration omission in C-speed0.2, not a vision/IK/following-error failure.

Ordinary move_joints and IK-produced move_pose now share the same adapter120deg constant as multiwaypoints. Cartesian cap remains600mm. SDK80%, jaw25mm/s, following0.1rad and arrival/binding0.01rad unchanged. Native collision, workspace and mechanical limits remain enforced.

Preserve explicit native RejectedBeforeDispatch validation errors through task Context's JSON wrapper. Only exact boolean executed=false, dispatch_started=false, no_action_executed=true and control_uncertain=false qualify. No automatic retry. Such RPC attempts are counted separately from dispatched or ambiguous actions; no unnecessary Hold solely for a proven no-dispatch rejection. Timeout, malformed or missing evidence still triggers Hold and uncertainty.

Acceptance:552 tests+52 subtests. Includes exact recorded command through frontend/backend, move_joints and move_pose at60/95.27/120deg and refusal above120deg; retained multiwaypoint90/120/121deg tests; recorded rejection counters and adverse error cases. Exact original95.27deg motion passed production bridge trajectory compilation, installed FCL collision and FK workspace offline:6.6s movement,27.064deg/s wire peak. No hardware commands sent. C-speed0.2's20-scene geometry results remain baseline evidence, not rerun or relabeled as new live success.

Original C-speed0.2 and frozen archives preserved. Web candidate uses an independent runtime. Fresh scene/Prepare and operator start still required; no automatic replay after failure.
