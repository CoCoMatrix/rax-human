# C-speed0.2.8 — bounded fresh RGBD acquisition

Parent0.2.7 controller and frozen artifacts are unchanged. Independent candidate uses the same perception, planning, geometry, motion profiles and qualitative placed-body checks.

The failed initial observation selected a head FFS packet aged about904ms; snapshot persistence consumed another106ms and violated the original1s TTL. No color recognition or motion had started. A ValueError ended main rather than refreshing read-only evidence.

Changes:
- Capture selects verified packets with at least250ms remaining for payload read, camera gathering and snapshot persistence. Near-expiry or naturally expired packets wait for new sources before payload decoding. Original1s TTL and acquisition timestamps are not modified.
- Save checks require100ms remaining for downstream preview/state work. An expired/near-expiry save is marked rejected and its frame retained for diagnosis. Reacquire a newer packet with a stricter source boundary. At most3 snapshot attempts, sharing a2s head or5s wrist-depth deadline.
- Initial observations have a wait budget even without an after-motion boundary. Post-action source boundaries are retained. Wrist retries issue a new request identity; no fabricated hand-eye transform. RGB-only captures do not force depth.
- Backend checks freshness again after preview/state/geometry binding. It reacquires read-only data with new state brackets within the SAME deadline and remaining attempt allowance. It never replays a motion or relaxes binding/rest/owner checks.
- Bad provenance, payload hashes, calibration, future timestamps, malformed metadata and camera-frame errors still fail immediately. Unavailable fresh sources still yield a bounded actionable timeout. This does not hide a dead camera or network fault.
- Recorder already polls independently and records transient read failures without ending the task; no blocking recorder retry added. Saved-frame pixel analysis intentionally uses immutable bound observations, not a new capture; its source identity and the task's observation/binding checks remain unchanged.
- Capture metadata records wait_count, attempts, discarded frame IDs, source stamps, reason and elapsed time. Backend records any final-stage discarded frame IDs.

Validation:
632 tests +52 subtests, including real payload and image serialization with injected timing, near-expiry pre-read selection, save expiration, head+wrist request renewal, post-action boundaries, preview expiry, exhausted budgets and nonretryable failures. Original failed race reproduced using two actual archived FFS packets and original timestamps with deterministic replay timing; next-packet publication is simulated, not a live FFS test. No hardware/model calls. Source validator verifies host patch hashes.

Perception study: see PERCEPTION-REVIEW.md and PERCEPTION-BENCHMARK.json. Existing three-color RGB segmentation about10–11ms; RGBD/table/selection about80–84ms on two archived successful initial frames (10 measured repetitions each). Heavy-model alternatives researched but not installed or benchmarked. No unproven new detector deployed.

New runtime host patches: services/piper/cameras.py and services/piper/backend.py. Defaults remain80% arm speed,25mm/s jaw speed. Deployment only updates Web and optionally readonly preview, never Prepare/Run/motion.
