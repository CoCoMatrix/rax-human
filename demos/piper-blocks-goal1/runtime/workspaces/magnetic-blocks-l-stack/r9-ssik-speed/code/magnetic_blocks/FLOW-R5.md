# c-Four1.0 full-flow audit r5

Preserves r4 and all frozen controllers. Main entry remains assembly_controller.main(ctx), no added runtime framework, no autonomous recovery.

## Findings and changes
- No remaining undefined globals/locals/exports in128 Python files spanning candidate and active Piper/controller/dashboard host modules (Ruff F821/F822/F823).
- Close: remove duplicate scene_geometry call; reuse the positive active-object association in displacement diagnostics; calculate reference FK only if the measured tilt needs that reference. Cached observation reuse also requires current allowed state binding/owner and nonnegative age.
- Lower to release: support reuse now binds color, completed request, owner/model identity, actual state and30-second lifetime. Stale/mismatched evidence gets one fresh observation and support check. Reuse flags distinguish a refreshed result from a reused one.
- Release: no unused post-release RGB request. Use measured completion/settling/open-gap receipt; preserve support_frame and release_request_id separately, never fabricate a release image. Retreat still obtains the next image and checks the qualitative placed body.
- Retreat: require its observation to match its own completed receipt using the existing binding tolerances. No numerical tolerances changed. Original geometry, table/collision, owner, status and ambiguous-dispatch checks remain.

## Offline evidence
62 selected regression tests +14 subtests pass. Includes15 new real Execution-entry tests covering four colors' close/lower/release/retreat chains, cached support invalidation, missing support, stale observations, ambiguous release, and mismatched retreat evidence. Original r4 fails the four duplicate-work regression cases; r5 passes.
24 reference/entry tests also pass inside the production bubblewrap runtime with test-only pytest libraries; explicitly synthetic transport/geometry, zero host robot RPCs. No packages installed into production runtime.
11 historical image/state endpoint replays pass without Gemini. Recorded orange-close failure replay is covered by r4 tests. Actual yellow release and retreat at72/73/74mm pass native checks in the isolated environment. Full historical-scene preflight26.771s,38 native-certified actions. All real hardware dispatch counts0.

One legacy test_b02 low-hover fixture still provides only3 colors, incompatible with c-Four's4-color input; it is recorded/excluded, not represented as passing. No full legacy-suite or live qualification claim. Native geometry checks are sampled model evidence, not physical validation.
