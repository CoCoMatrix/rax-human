## Current selection for startup — 2026-11-09

Read this section before choosing tools; open the linked evidence only for the
selected tool or a concrete failure. Historical entries below retain their original
limits/results; later dated operator policy supersedes old policy, not old evidence.
This table remains PARTIAL characterization, not blanket autonomous qualification.

| Tool / concern | Current use and evidence | Limits / next detail |
|---|---|---|
| status, piper_status, RGB observe | Read-only current state; early stationary medians3.5ms/status and306.5ms/3-view observe | Timings are historical samples, not an SLA; [earlier measurements](#earlier-evidence-and-diagnostic-history) |
| Head FFS / pixel query | Valid calibrated head optical samples transformed to left_base_link; source TTL1s unchanged | Saved-point analysis is not a motion permit; external depth accuracy provisional. Uncompressed arrays retain original TTL; R23 live target creation passed |
| Left wrist FFS | Explicit optical depth available;5/5 camera-only checks, processing0.261–0.327s | No hand-eye or base-frame target; [wrist evidence](#left-wrist-ffs--retained-target-update--2026-11-09) |
| FK / IK / path | Numerical planning tools; path uses installed170mm finger envelope | No external metrology/full environment coverage; empty obstacles mean self-collision only; [measurements](#earlier-evidence-and-diagnostic-history) |
| Left joint/pose motion | Small J1 pass; R21 task preparation/hover/entry and10mm lift reached measured targets under historical0.006rad policy | Original3mm/0.003rad failures are NOT repaired or passed. General repeatability/path deviation remain incomplete; see arrival policy and historical failure sections |
| Left gripper | Empty native width matrix and conditional task closures; R21 pickup required onsite confirmation | Commanded width alone is not retention; [native matrix](#left-empty-gripper-repeatability) |
| Stop / hold | Stop acknowledgments and8sample rest confirmation observed, including R22 | No measured in-motion cancellation latency/overrun; serialized piper_hold is not preemption |
| Visual / operator facts | Head + active LEFT wrist for practical visible outcomes; current scoped onsite answers may supplement missing facts | Hidden gaps/transient contact not provable from endpoint images; [visual results](#visual-repair-result) |
| Right side / Prepare | Right task motion not exposed; retain state/collision monitoring. Home is host XPolicy only | Never home through task/agent motion code |

Current JOINT state tolerance is0.01rad across arrival, observation/onsite
binding, planning/execution anchors, one-second rest and recovery. Jaws remain
separate: anchor/capture0.0005m, rest0.0003m, arrival1mm. Original source timestamps,
owner, geometry, lease and dynamic/path limits remain active. No zero filtering or unvalidated motor-angle conversion.
R21 reference:commissioning/blue-cup-v03-rehearsal-20261109-r21; this was a partial
rehearsal, NOT unattended placement success. No40mm trial or new-layout full pass.
Record the selected tool's applicable evidence/limits in PROGRESS, not the whole
chronology. Inspect further details for untested ranges, contradictions or faults.

## R29 joint-state policy update — current, 2026-11-09

Operator explicitly raised all joint STATE checks to0.01rad after R28 J6
alternated0/0.0065973446rad immediately after a successful approach segment.
The old0.006rad check rejected that single state comparison. Arrival, observation
binding, target advance, anchors, rest/recovery and right-state preservation now
use0.01rad. Jaws, source freshness, owner/lease, path/collision and dynamic limits
are unchanged; raw zero feedback is retained. This is an acceptance-policy change,
not a firmware correction or independent measurement of physical drift.
Offline verification includes this recorded J6 case, exact threshold and excess,
plus unchanged jaw/freshness/owner checks. Live0.01rad rehearsal is pending.
R28 preparation passed183.739s/9visualcalls; first approach motion arrived but
parent returned after the J6 feedback mismatch. No closure/lift/placement executed.

## R23 joint-state policy update — latest operator instruction, 2026-11-09

This supersedes OLD smaller joint drift/rest/anchor limits in the history below.
R23 failed before trajectory release because left J1 alternated0/-0.00132645rad
while the bridge anchor limit was0.0005rad. Recorded hardware writes stayed744;
no release/SDK receipt exists for that failed action. This is measured feedback
variation, not independent proof of physical zero drift or a firmware diagnosis.

All RAX joint STATE comparisons now use0.006rad. Jaw metre thresholds remain
separate; time windows/freshness, collision, owner identity and Stop remain.
The actual-start-to-first-target connector is rechecked against installed CAD and
workspace when feedback differs. Planned targets are unchanged; no zero filtering.
Owner first-trajectory-anchor uses0.006rad only when the RAX lease and explicit
RAX_JOINT_STATE_TOLERANCE_RAD are present. Other task profiles retain their defaults.
Offline actual-class replay accepts the recorded J1 case; >0.006rad, jaw excess,
stale feedback, disarm, changed owner, collision and workspace faults still refuse.
This changes acceptance sensitivity; it does not establish subthreshold physical
accuracy, in-motion Stop latency or a completed cup task. Live task validation of
this update has not been run. Evidence:verification/r23-anchor-startup-20261109.

## Left wrist FFS / retained target update — 2026-11-09

Supersedes historical wrist-RGB-only entries below: left D435 serial262422071560
now supports explicitly requested FFS optical depth; right remains RGB only.
Camera-only real validation:5/5 head+left requests and left pixel queries passed;
left FFS processing0.261–0.327s. Warm combined capture/save/query
median0.474s after removing dual-depth compression, previously0.799s.
These are5 camera-only samples, not a full backend/agent timing distribution.
No robot SDK, owner motion process, or arm action was started. Three RGB streams
were healthy; head FFS remained active. No near-cup accuracy, metric reference
object, motion-time wrist transform or hand-eye validation is claimed.

Request piper_observe(...,depth_cameras=["left"]); left pixel queries require
output_frame="camera_optical" and return left_d435_color_optical_frame with
robot_target_usable=false. Base-frame requests are explicitly rejected. The existing
1s acquisition TTL, LR consistency and direct-only pixel samples remain intact.
The writable cup approach retains its measured BASE target through empty preparation/hover/entry
with fresh RGB/state/path checks. This is distinct from reusing old wrist camera
points after motion; the latter needs a known transform, which is not available.
Evidence:verification/target-wrist-20261109/camera-only-fast/RESULT.json.

# Piper tool characterization — 2026-11-05

## Current observation binding — explicit operator update, 2026-11-09

Observation-to-current-pose matching and before/after capture joint binding use
**0.006 rad** for each joint. Jaw binding stays **0.0005 m**, not0.006m.
This accepts the recorded0.001780235837rad J4 change without invalidating a fresh
frame solely for that joint difference. Freshness, same-owner, one-second rest
range0.0003, geometry and required visible contact facts remain enforced.
A bound frame is not proof of physical clearance. The scoped onsite-fact pose guard uses the same joint/jaw limits; original
host reply, action scope, owner and one-use checks remain. The backend local
pre-dispatch anchor remains unchanged. Source working copy only; frozen task versions remain immutable.

## Current arrival tolerance — operator update, 2026-11-09

The operator explicitly changed the RAX joint arrival tolerance to **0.006 rad**
(about 0.344 degrees), accepting reduced sensitivity to very small angle moves.
This supersedes all historical statements below requiring unchanged 0.003 rad.
The RAX installed-settling adapter and post-motion observation validation use
the same configured limit. Receipts/status expose the applied tolerance.

This is an acceptance-policy change, not a repair of firmware filtering or a
switch to motor-angle feedback. An unchanged 0-degree report can be within
tolerance of a -0.28-degree target; do not claim that such a check proves the
small move occurred. Historical failures remain recorded, not retroactively
marked successful. Live acceptance with this policy is still pending.

The one-second fresh-feedback rest window, 0.0003-rad joint / 0.0003-m jaw drift ranges, 1-mm jaw
arrival limits, dispatch receipts, freshness, geometry, speed, lease and Stop
checks are unchanged. Only left-arm task motion is in scope. Agents must not
change tolerance further without operator instruction. Frozen v0.1/v0.3 files
remain unchanged; this applies to the writable RAX runtime/workspace.

## Motor-angle candidate — left arm only, 2026-11-06

The operator relayed vendor advice to read motor angle for higher precision and
authorized continuing with the left arm while the right CAN adapter is absent.
This supports evaluating the motor channel; it does not yet qualify a coordinate
conversion or change the original arrival tolerance.

Passive SDK reads: left filtered joint angles were all zero, while motor
positions were [0, 0.006, -0.008, 0.004, 0, 0] rad, at approximately 100 Hz.
Historical nonzero-pose raw CAN comparison also shows inter-channel differences:
J2 median motor-minus-joint -0.010391 rad, J3 -0.007085 rad, J4 +0.003299 rad.
Do not derive a fixed zero offset from these filtered samples or assume
one-to-one equivalence for all six joints.

Offline replay of archived J5 motor values, using raw * 0.001 rad and unchanged
0.003-rad error / 0.3-second stable-window criteria:
- target -0.335 deg: first qualifying window at 0.362683 seconds;
- target -0.28 deg: first qualifying window at 0.367865 seconds.
This is candidate replay evidence, not a new live pass or independently verified
output-shaft accuracy. The full RAX settling check and original 3 mm pose have
not passed with a motor-backed state source.

Next: obtain the mapping between move_j command coordinates and motor position
(zero, sign, ratio and calibration compensation), qualify LEFT channels only,
then integrate a consistent measured state into arrival and geometry.
No live motor-angle replacement, motion or right-arm change has been made.

### Vendor follow-up: fixed 0.3-degree change threshold

The operator relayed a further vendor answer on 2026-11-06: the 0.3-degree
threshold applies to angle change and currently cannot be disabled. The exact
reference sample/reset/hysteresis rules and any alternative output-position
interface are still unspecified. Do not pursue a filter-disable patch without
new vendor support.

Installed SDK source audit: get_motor_states(5) passively reads cached 0x255;
it does not send requests or change mode. decode_high_spd converts signed
bytes 4:8 to position using raw * 1e-3, with API output documented in radians.
That establishes SDK scaling, not equivalence to calibrated output-joint angle.
Vendor confirmation of shaft/reference, gear ratio, sign, zero and filtering
is still required before adopting it for original 0.003-rad arrival checks.

## Vendor clarification — 2026-11-06 (latest; acceptance still pending)

The operator relayed after-sales support's statement: joint-angle changes smaller
than 0.3 degrees are filtered from 0x2A7 feedback, while the motor executes the
angle command. This is vendor information relayed by the operator, not a new
independent measurement of the output shaft. It supersedes treating unchanged
0x2A7 alone as proof that J5 did not move.

The original arrival tolerance is 0.003 rad = 0.171887 degrees, smaller than the
reported 0.3-degree feedback threshold (0.00523599 rad). A fresh frame timestamp
does not guarantee sub-threshold motion is observable. The threshold reference
(last reported angle, another sample, or zero), reset/hysteresis behavior and
whether the filter is configurable remain to be confirmed.

Do not widen tolerance, filter zeros, substitute command/previous/motor values,
or declare arrival from this explanation. Actual accuracy at the original
tolerance still needs a qualified feedback channel. Also ask why the archived
failed interval's 0x2A1 motion_status stayed 1 (not reached); support's feedback
filter explanation alone does not establish that completion flag's semantics.

Preferred next step: obtain the vendor-supported way to disable/reduce filtering
or read unfiltered actual joint/output position, then validate that channel at the
original tolerance. No firmware, control parameters, zero calibration, arrival
logic or hardware motion was changed for this clarification. Original 3 mm pose
regression and cup rehearsal remain pending; frozen versions remain unchanged.

Status: PARTIAL acceptance, with a FAILED pose-motion case. Never interpret this table
as blanket hardware acceptance. Source sessions: commissioning/tool-acceptance-20261105-readonly,
tool-acceptance-20261105-tools, and
tool-acceptance-20261105-motion. Full logs and frame paths remain on the 5090.

The new-session workflow reads this table with robot-interface before choosing a tool.
Re-run affected tests after a tool, model, firmware, TCP, camera or owner change.

## Feedback/time diagnosis — blocking update, 2026-11-05

New task motion is **not qualified** while the near-zero joint feedback discrepancy remains.
`commissioning/feedback-clock-20261105-r5-hold-fixed-time` logged raw SocketCAN,
public SDK return values, owner samples and HTTP publication on one monotonic timeline.
Over92.542s, J5 CAN0x2A7 contained18449 packets at−0.335deg and14 at0deg;
SDK/owner samples contained8 zeros and HTTP contained6. The first zero was already
present in raw CAN before the SDK read. J1 remained stable in this particular test;
the earlier J1 symptom is not thereby fixed. Motor0x255 raw position around the first
zero was−6/−5; these counts were not substituted for measured joint angles.

The XPolicy adapter previously captured now_s before reading asynchronous SDK
feedback, incorrectly rejecting a new frame received during those reads as future.
Live owner snapshots now take time after both reads. Explicit timestamps remain
available for offline replay. Original100ms freshness and all motion/state tolerances
are unchanged. Four regression tests pass, including stale and genuinely future
rejections. Vendor SDK/firmware and frozen task controllers are unchanged.

System time source changed from ntp.ubuntu.com to measured lower-delay sources
(time.cloudflare.com, ntp.aliyun.com). During the92.542s retest, max wall/monotonic
sample discrepancy was6889ns, no50ms clock step. This is a finite observation window,
not a guarantee against every future NTP step. The bridge clock protection remains.

Current outcome: time read-order repaired; **raw near-zero feedback unresolved**.
No3mm pose regression, no homing and no new cup exploration after this failure.
Next: obtain a vendor-supported explanation/fix for Piper X S-V1.9-0 / pyAgxArm1.0.0
V189, then repeat synchronized hold and the original motion regression at unchanged
limits. Do not mask zeros, swap in target/previous values, relax guards or infer a
firmware fix from unrelated Piper-L results. Evidence: verification/feedback-clock-20261105/.

## Auxiliary vision update — Google Gemini official API

Current new-session auxiliary model: gemini-3.8-flash via native gemini_generate;
main model remains gpt-6-astra/high. Saved-image MCP acceptance passed: head+left
4.69s, before/current head+left 8.97s; function-call follow-up also passed.
Current images retain native coordinates; only prior views use 640px-wide thumbnails.
Four API calls: 8121 tokens, conservative estimate $0.00753375 (not reported billing),
no unresolved reservation. Evidence: verification/gemini-20261105/mcp-acceptance.json.
These are archived-image software/API checks, not live grasp/clearance acceptance or
stable latency statistics. Hidden-gap/transient-contact limitations still apply.
Earlier Codex vision measurements below are historical, not this provider's latency.
No hardware opened for this update. The failed J4/3mm case remains unresolved.

## Current result after right-jaw zero calibration

The operator confirmed the right jaw fully closed and explicitly authorized zeroing.
The existing XPolicy `calibrate_closed_gripper` was called once, with no joint target
or other-jaw command. Vendor ACK preceded applied feedback: the immediate post-ACK
check still saw -0.10 mm; subsequent fresh samples stably showed 0 mm. Keep both
`right-jaw-zero.json` and `right-jaw-zero-verification.json`; the former's immediate
verification failure is not silently overwritten. No retry/reset of joint zero occurred.
RAX startup then passed without changing any jaw-range or arrival guard.

Evidence: `commissioning/j4-followup-20261105/RAX-LEFT-TOOLS-SUMMARY.json`,
`left-tools-live/`, `left-tools-finish/`. These are actual MCP/backend/bridge/owner
executions with new three-camera observations, automatic duration and sensor archives.
They supersede the earlier passive-right startup blocker and native-only limitations
for the specific tests listed here, not the unresolved 3 mm pose failure.

| LEFT RAX tool / case | Result | Timing / limits |
|---|---|---|
| Joint J4: 0 -> 0.5 -> 0.75 -> 0.5 -> 0 deg | 4/4 passed; final feedback 0.436/0.767/0.543/0 deg; max error0.064deg | RPC3.237/3.312/3.044/3.116s, each followed by3s in-band observation; tolerance0.003rad unchanged |
| Empty gripper0.3mm | 3/3; feedback0.7mm each, error0.4mm | RPC3.197–3.524s |
| Empty gripper4mm | 3/3; feedback4.2mm each, error0.2mm | RPC3.026–3.789s |
| Empty gripper5mm | 3/3; feedback4.7mm each, error0.3mm | RPC3.175–3.663s |
| Empty gripper10mm | 3/3; feedback9.6mm each, error0.4mm | RPC3.974–4.329s |
| Idle hold and independent Stop | Passed measured-rest checks; final Stop8samples/0.360s | Rest window is not stop latency; no in-motion stop measurement |
| 3mm pose / J4~0.2845deg | **FAILED / unresolved** | Reproduced below RAX with one native target; no tolerance widening or feedback substitution |

All 12 RAX jaw actions passed the existing 1mm arrival tolerance, path checks and
post-action three-camera capture. First-cycle targets each had3s further observation;
repeat2/3 used0.3s, except the final close had3s. Sampled post-arrival width range was0mm
in those MCP windows; native longer observations separately found up to0.1mm. Report
the windows separately. Images were acquired and manually inspected for empty-jaw
setup; this batch did not rerun auxiliary-model visual classification for every move.
No payload/slip test or external metrology was performed.

The first live session completed11 jaw moves and stopped before the12th because it
entered wind_down. Its RESULT remains partial/false for completeness, not a motion
failure. A fresh physical session completed the remaining close. No budget override
or action replay was used. Both sessions Stop-confirmed rest, closed children, and
verified control/sensor archives. Final measured joints bothzero, leftgap0.7mm,
rightgap0mm. No active Prepare lifecycle is claimed after process exit.

Full left-task acceptance remains PARTIAL: other joint repeatability, original3mm
pose regression, loaded retention, external geometry/depth validation and moving
Stop/overrun are still missing. Do not infer success from the tools merely existing.

## Earlier evidence and diagnostic history

| Tool | Observed result | Acceptance boundary / remaining work |
|---|---|---|
| `status` | 10 reads; median 3.5 ms, max 4.0 ms | Only stationary state tested |
| `piper_status` | 10 reads; median 3.5 ms, max 4.3 ms | Only stationary state tested |
| `piper_observe` | 4 successful captures, all three cameras 1280×720; zero state-bracket drift; median 306.5 ms, max 320.9 ms | Bracketing is not hardware synchronization; moving-frame binding remains untested |
| `piper_fk` | 10 calls across both arms; median 3.2 ms, max 3.9 ms | No independent external TCP accuracy measurement |
| `piper_solve_ik` | 6 current-pose solutions; unreachable target correctly refused; median 3.9 ms, max 261.4 ms | Need varied poses/seeds; FK residual is not external physical accuracy |
| `piper_check_path` | 3 clear zero-pose checks, installed 170 mm geometry; median 33.9 ms, max 1332.2 ms | No environment boxes; only self-collision check, not table/cable clearance |
| `piper_pixel_to_position` | head query succeeds; wrist metric query correctly refused; median 14.7 ms, max 26.2 ms | No known-size target measurement yet; wrist hand-eye unavailable |
| `piper_stream` | all three contact sheets returned; median 14.4 ms, max 15.8 ms | Observed RGB ≈5 Hz/camera, robot ≈10 Hz, depth 1 Hz; no recorder errors in sampled window |
| `piper_move_joints` | One left J1 +0.5° and return; outward error 0.048°, return feedback zero; right arm/jaws unchanged | Only this small J1 case accepted; J2–J6 and repeatability matrix NOT completed |
| `piper_gripper` | One empty opening: command 5 mm, feedback 4.7 mm; measured arrival and visual pre/post checks passed | Not grasp or retention acceptance; repeated widths/closing/load tests remain |
| `piper_move_pose` | **FAILED**: upward 3 mm command timed out after dispatch | Do not use as an accepted autonomous primitive. Investigate small-target tracking before further physical trials; never loosen tolerance to hide failure |
| `piper_hold` | Boundary calls succeeded after joint/opening tests; NO_SEND call truthfully reports no hardware stop | Does not preempt a running serialized MCP call; in-motion hold latency not measured |
| Independent Stop / disarm | After pose timeout: acknowledged and measured rest confirmed, 8 samples across ≈0.36 s | This is a rest observation window, NOT a measured stop latency; e-stop and overrun characterization remain |
| `codex_vision_generate` | Fresh head+left empty-opening check 26.93 s; physical opening precheck 23.47 s, postcheck 35.53 s | First real visual regression passed; no hidden-underside/transient-contact requirement; not a broad false-positive/negative benchmark |
| `request_operator` | Current supervised batch confirmation received before live motion | Offline timeout/late-answer regression covered separately; no new physical reset test here |
| `skill_library_merge` | Offline multi-session and sandbox checks passed in previous revision | No new physical skill promoted; manual/tool tests do not count as unattended task success |

## Visual repair result

The writable v0.3 `move_once` helper executed the empty-opening test with named visible
facts and the real gpt-6-astra medium API, using current head and left-wrist images.
Precheck, geometry, measured arrival and postcheck passed. Total 62.879 s:
vision 58.994 s, motion including returned observation 3.444 s, explicit geometry 0.112 s.
Thus the old hidden-gap/transient-contact demand did not block this case. API vision
latency remains the dominant cost; this result does not establish low latency.
The current scene contains tabletop assembly pieces, not the cup/coaster. The test's
scene-unchanged fact referred to those visible objects; no cup grasp/approach was tested.

## Pose-motion failure and stop

All 36 SDK target receipts were recorded. Target left J4 was 0.004965634 rad (0.2845°),
while stopped feedback remained 0; discrepancy 0.004965634 exceeds the enforced
0.003 rad arrival tolerance. J2/J3 moved. This identifies the measured reason arrival
could not pass, not the underlying cause of J4 tracking failure. Deadband, friction,
command processing and dwell require further diagnosis; none is proven here.
No target replay, tolerance/gain change, or model-directed recovery was attempted.
Independent Stop confirmed rest; the session then closed frontend, bridge, graph and depth
worker and archived evidence. Final left q=[0,0.005532694,-0.009424778,0,0,0], jaw=4.7 mm;
right q=zeros, jaw=2.1 mm. This is a stopped measured pose, not zero Prepare.

## Tool selection until further characterization

- Use status/observe and fresh head/wrist evidence; inspect actual depth validity.
- Use FK, IK and check_path as numeric evidence within their documented scope.
- Never describe `move_pose` as Cartesian straight-line motion or as accepted after
  this failure. Do not start an autonomous task that depends on the failed primitive.
- For unknown outcomes, stop and inspect measured state; do not retry a completed or
  partially completed action because its postcheck failed.
- Prepare/return-to-zero remains an operator XPolicy operation.

## Instrumented follow-up, 2026-11-05

Installed pair: Piper X S-V1.9-0, pyAgxArm 1.0.0, python-can 4.6.1, firmware driver v189.
No vendor SDK/firmware/gain/tolerance change. Joint arrival tolerance remains **0.003 rad**.
Evidence root: `commissioning/tool-diagnosis-20261105/` on the 5090.

- Readonly: 170 state samples, three complete three-camera observations, zero hardware writes; archive verified.
- Repeated the original 3 mm upward full-pose request once through the real MCP/backend.
  It failed again; no successful return move was claimed. Stop acknowledged, eight fresh
  samples across 0.3704 s confirmed rest. This window is not stop latency.
- Raw SocketCAN `0x2A6` J4 reported approximately 0.310–0.317 degrees briefly, then zero.
  J4 motor `0x254` position changed from raw 4 to 9 and stayed there during the zero
  joint feedback. These are distinct feedback streams; motor position is NOT substituted
  for joint angle. Conversion, zero offsets and independent physical reference are not
  calibrated. This rules out a purely RAX JSON/SDK decoding explanation for the zero
  joint feedback, but does not prove a specific firmware defect or exact physical angle.
- First trial receipts likewise contained J4 0.330 degrees before the zero value.
  The earlier phrase “never followed” is therefore incorrect. The arrival timeout is
  valid under the current feedback contract; raising tolerance or filling zeros would
  conceal the unresolved issue.
- Passive capture did not contain TX loopback command frames. SDK receipts establish
  requested targets, not physical CAN transmission confirmation; do not claim otherwise.
- A J4-only comparison was not run: another XPolicy owner acquired the same arms/cameras.
  It was left untouched. Current robot pose/control belongs to that task; our last pose
  is historical, not a claim about its current Prepare state.

## Required measurement matrix

“Pending” is not zero error, not N/A, and not acceptance. Tests must use the actual
backend/guards; disarm tests use the independent operator path, never queued MCP hold.

| Mode/tool | Final error / external check | Repeats | Duration / settle / overhead | Path deviation | Stop/overrun/3 s drift | Failures / next check |
|---|---|---|---|---|---|---|
| Left joint move | J1 +0.5 deg: 0.048 deg; no external metrology | N=1 round trip; pending N>=3 and J2–J6 | RPC 3.090 / 3.451 s; plan 1.2 s; separate settle pending | Joint path documented; measured FK sampling pending | Pending in-motion disarm | J4 near-zero feedback unresolved; compare J4 alone before more pose trials |
| Left pose move | 3 mm upward: FAILED twice; no verified endpoint | N=2 failures, not repeatability | Timeout; separate bridge dispatch/settling evidence retained | Joint interpolation, not Cartesian line; measure later | Postfault rest only, not stop characterization | Keep unavailable as accepted autonomous primitive |
| Left gripper width | target5 mm / feedback4.7 mm; empty-image check passed | N=1; closing/multiple widths pending | 3.444 s RPC; whole visual helper62.879 s | N/A for free jaw width; object envelope still required | Pending | Position width is not force grasp/retention proof |
| Hold / independent disarm | Postfault rest confirmed | One stop per failed case | Rest window only; true cancel latency pending | N/A | All three metrics pending | Do not claim MCP hold interrupts serialized calls |
| FK / IK / check_path | Numerical checks only; external TCP/obstacles unverified | Counts above | Counts/timings above | Dry-run checks sampled planned path | N/A (read-only) | Add varied seeds, boundary rejection and explicit environment boxes |
| Observation / pixel | Three RGB views; head depth returns provisional coordinates | Counts above | 306.5 ms observe median | N/A | N/A | Known metric target, extrinsic residual and wrist hand-eye pending |
| Stream | Actual RGB5 Hz/state10 Hz/depth1 Hz | One measured window | Recorder errors absent in measured window | N/A | Coverage insufficient to exclude brief contact | Verify gaps/retention and higher cadence for transient investigations |
| Vision / operator | One visible-fact empty-opening pass | Narrow regression only | See timings below | N/A | Never physical stop evidence | Add occlusion/visible-hazard negatives and scoped handoff replay |

A suggested repeat count is a test plan, not a new runtime requirement. Right-arm motion,
right gripper, force grasp, vendor Cartesian/CPV/MIT are not exposed/qualified here; no
attempt to enable them is part of this table.

## API latency evidence (diagnostic only)

The same saved pre/post visual prompts and images were replayed without hardware writes:
2 images/588705 bytes:18.432 s total, headers18.007 s, remaining download~0.399 s;
4 images/1177985 bytes:16.692 s total, headers16.426 s, remaining~0.256 s.
A streaming 2-image comparison: proxy20.840 s (headers13.054 s, first output13.710 s),
direct17.741 s (headers8.763 s, first output9.257 s). One sample per condition cannot
justify changing routes. Completed-event to EOF was <1 ms for both streaming samples;
no long tail after completion was observed. Server-side queue/inference/network components
are not separately exposed. Provider reports reasoning_tokens=0 despite requested medium;
this is provider telemetry, not proof that no reasoning occurred. Model/effort/production
streaming setting remain unchanged. Timing diagnostics now reuse the existing metadata-only
Responses trace and never log credentials or image/prompt bodies.

## How sessions consume this table

New AGENTS.md requires robot-interface and its selection guidance before choosing motion,
with applicable evidence/range and unresolved items recorded in PROGRESS.md. Piper status
returns this table's relative path, SHA256, partial acceptance and the unresolved J4 case.
A generated-workspace regression checks the copied table against the actual backend status
hash. This proves delivery to the agent; it does not prove a future model obeys instructions.
Review the next live session's transcript/PROGRESS before crediting behavioral compliance.

## Second handover / J4-only preflight, 2026-11-05

Evidence: `commissioning/j4-followup-20261105/` on the 5090. With explicit user
handover authorization, the previous Michael XPolicy graph was checked prepared,
not recording, no active rollout, and stationary, then stopped with SIGINT. Its
owner/camera processes and port 8091 were gone before RAX acquired devices.

The new readonly session passed: 81 state samples, three complete three-camera
snapshots, hardware_write_count=0, and verified sensor archive. This new scene is
a cup/coaster scene; earlier assembly-piece images remain historical evidence.
Both arms reported zero joints; left jaw=0.30 mm and right jaw=-0.10 mm.

The subsequent live host refused `checked_current_pose` because right jaw feedback
was outside the declared 0..75 mm range. Startup had enabled the owner, but no
Prepare, Start, or J4 test action was reached. Independent Stop acknowledged;
eight fresh samples over 0.35017633 s confirmed rest. All child services exited,
and control/sensor archives were verified. No zero calibration, clamp, range or
arrival-tolerance change was made. The J4-only comparison is **NOT RUN**, and the
3 mm pose failure remains unresolved. A prepared probe is not acceptance evidence.

Right-jaw movement was outside the earlier left-only motion scope. The separate
2 mm request was subsequently authorized and attempted once; feedback stayed at
-0.10 mm and arrival failed. Later read-only driver flags reported right gripper
disabled. No zero reset, software-stop release, or further right-jaw retry followed.
The user then selected left-side-only characterization.
Do not send the negative measured width as a return target. After valid state is
restored, run J4-only comparisons and inspect raw CAN plus 3 s post-arrival feedback
before repeating any 3 mm pose motion. Joint arrival tolerance remains 0.003 rad.

## Left-side scope and native comparison results

User scope: left arm and left gripper for cup placement. Right-arm/right-jaw motion
is not a required capability or acceptance target. Fresh right-state monitoring and
whole-robot collision checks remain required because the right hardware is present.

**Source distinction:** all results below use the existing XPolicy public single-arm
canary API and the pinned SDK, without a RAX planning model. These are bottom-layer
comparisons, not successful RAX MCP executions. The original MCP 3 mm failure and
right-jaw preflight blocker remain open. No arrival tolerance, vendor SDK, firmware,
zero calibration, or frozen controller was changed.

### J4 isolation

`native-j4.json` and `native-j4-can.json` record one sequence:
0 -> 0.5 -> 0.75 -> 0.5 -> 0 degrees, speed 5%, tolerance 0.003 rad, continuous
in-band arrival for 0.3 s, then 3 s further observation at each endpoint. All four
steps passed; stable-arrival times were 0.383/0.383/0.363/0.362 s. Endpoint feedback
was 0.521/0.698/0.521/0 degrees, maximum endpoint error 0.052 degrees. No right/jaw
commands. This qualifies only these native small-motion cases, not all J4 targets.

`native-j4-small.json` repeated J4=0.284509884 degrees with a single SDK target.
It FAILED after 5 s with the same 0.003 rad tolerance. In the approximately 5 s
observation, 228 of 249 sampled joint values were zero; the others were mostly
0.336 degrees. Raw CAN 0x2A6 likewise showed zero while motor-position 0x254 was
mostly raw 9. Thus the zero feedback is emitted below RAX and its streaming
transport, not introduced by model waiting, IK calculation, Python JSON conversion,
or repeated RAX mode frames. Motor counts are not independently calibrated joint
angles. Exact physical motion and the controller's internal zero/quantization rule
remain unverified; do not claim a specific firmware patch or substitute motor counts.

A current-measured hold after this failed test ended at J4 feedback 0.336 degrees.
A subsequent XPolicy diagnostic return to zero passed before left-jaw tests. This
was the small-test return, not an autonomous home or a cup task recovery.

### Left empty-gripper repeatability

`native-left-jaw5.json`: initial 5 mm opening, measured 4.6 mm, stable arrival 0.423 s.
`native-left-jaw-matrix.json`: 3 repeats per target below, force command 1 N,
unchanged 1 mm arrival tolerance and 0.3 s stable window, plus 3 s hold observation.
Each cycle was 10 -> 4 -> 5 -> 0.3 mm; maximum commanded change stayed under 10 mm.

| Target full gap | Repeats | Final feedback | Max final error | Stable-arrival time | Max sampled 3 s drift |
|---|---:|---|---:|---|---:|
| 0.3 mm | 3/3 passed | 0.7 / 0.7 / 0.7 mm | 0.4 mm | 0.402–0.403 s | 0 mm |
| 4 mm | 3/3 passed | 4.2 / 4.2 / 4.2 mm | 0.2 mm | 0.402–0.423 s | 0 mm |
| 5 mm | 3/3 passed | 4.7 / 4.7 / 4.7 mm | 0.3 mm | 0.301–0.302 s | 0.1 mm |
| 10 mm | 3/3 passed | 9.6 / 9.6 / 9.6 mm | 0.4 mm | 0.423–0.463 s | 0 mm |

These are encoder-feedback results at approximately 50 Hz, not external dimensional
accuracy or calibrated force. No payload, cup retention, grasp force, or slip test
was performed. The historical “4 mm holds the cup” remains conditional task experience,
not a newly verified result. Native times exclude RAX interpolation, MCP, vision and
camera overhead, so they cannot be advertised as RAX end-to-end latency.

Final measured joints: both arms zero. Left gap 0.7 mm; right gap -0.10 mm unchanged.
No graph/agent/control service remained active. This is a measured final pose, not
an assertion that an operator lifecycle is still in Prepare after process exit.

### Required left-task tool table / remaining acceptance

| Needed tool | Current evidence | What still prevents full acceptance |
|---|---|---|
| State, three-view observation, stream | Earlier RAX live characterization plus new readonly handover | External timing/occlusion and continuous-video gap characterization remain scoped limits |
| FK / IK / depth / path | Earlier RAX numerical checks; installed left finger envelope | External TCP/depth accuracy, environment obstacle coverage, and varied IK poses/seeds remain pending |
| Left joint motion | RAX J1 narrow pass; native J4 sequence passes above | Native J4 small target fails; J2/J3/J5/J6 and RAX repeatability not fully characterized |
| Left pose motion | Two RAX 3 mm failures; native J4 isolation reproduces discrepancy | NOT accepted; valid joint feedback and original 3 mm end-to-end regression required |
| Left gripper width | One RAX 5 mm empty opening; native 4-width x 3-repeat matrix above | RAX multiwidth replay blocked by current passive-right state contract; loaded retention not tested |
| Hold / independent Stop | Earlier RAX postfault rest; native 3 s endpoint observations | Moving cancellation latency/overrun not measured; never equate rest samples with stop latency |
| Visual checks / operator facts | Earlier RAX visible-fact empty-opening pass | More occlusion/hazard negative cases; images cannot prove hidden gaps or absence of transient contact |
| Right motion / right gripper | Not selected, not exposed as autonomous tools | Outside left-task acceptance scope; retain fresh state/collision monitoring |

Do not unblock left RAX execution by simply deleting the right-jaw check: startup,
legacy full-state geometry, transport, owner target normalization and collision
semantics must agree. Preserve raw feedback; a CAD-only nominal closed-jaw model
must declare its uncertainty and must not become a fabricated measured/commanded gap.
The isolated J4 test used nominal closed right-jaw CAD and an additional 0.1 mm
geometric margin for the observed negative offset, without changing raw feedback
or either motion tolerance; this does not qualify a general runtime conversion.

## Minimal J5 diagnosis — 2026-11-06 (still BLOCKING)

Evidence: `commissioning/j5-minimal-20261106/` and its `nearzero/` subdirectory.
Firmware readback: Piper X H-V1.2-1 / S-V1.9-0; pyAgxArm1.0.0 with V189,
python-can4.6.1. This is a supervised existing-XPolicy native diagnostic,
NOT a RAX motion-tool pass and NOT an autonomous cup success.

- Passive zero-pose capture, no SDK/owner:4000 J5 CAN frames over20s, all zero.
- One J5 target−0.335deg at5%: arrival0.3824s, measured−0.394deg;
  30s settled hold contained6002 joint-feedback frames, all−394mdeg.
  A separate return to0 passed in0.3426s, then measured rest.
- One J5 target−0.28deg at5%: **FAILED** arrival after5.0194s at the original
  0.003 rad tolerance. SDK/owner arrival samples:248 zero,1 at−0.394deg.
  During the settled-target interval (1s after dispatch to timeout), all804 raw
  0x2A7 frames reported J5=0; 0x255 motor-position raw values were−4(401 frames)
  and−5(1 frame), versus0 at the initial baseline.
- All804 anomalous-interval joint frames had DLC8, no remote/error flags and no
  MSG_DONTROUTE local-origin flag. SocketCAN RX/TX errors and bus-error counters
  were0 at inspection. This rules out host-side zero insertion in this case;
  it does not identify the controller's internal algorithm.
- Failed target was not resent. The existing native current-measured hold was
  issued once;10 fresh samples confirmed rest with both arms reporting zero,
  left jaw0.3mm and right jaw0. No right/jaw commands were issued.

The contrast is consistent with device-side near-zero feedback processing, but
a precise deadband threshold, actual physical angle and firmware defect cause
are NOT independently proven. Motor counts are diagnostic evidence only;
they do not replace joint feedback. No zero filtering, target substitution,
tolerance widening, gain/zero calibration, SDK change or firmware update was made.
No original3mm pose regression or cup practice was started: the requested
prerequisite (root-cause repair) has not passed. A model-specific vendor fix or
validated explanation is required before treating this as repaired.

### Follow-up protocol audit — 2026-11-06 (still BLOCKING)

Offline XPolicy -> installed Piper X V189 -> in-memory transport verified that
-0.28 degrees encodes as -280 millidegrees (0x157 fffffee800000000); no target
clamping, zero rounding or duplicate suppression occurred. AF_CAN sockets were
forbidden. This is offline software-path evidence, not historical on-wire TX.

All 9,853 archived 0x2A7 frames replayed with zero SDK/independent decode
mismatches. During the failed target's settled interval, all 804 status frames
also reported MOVE J + motion_status=1 (not reached), err_code=0. The device
itself did not declare arrival. No root-cause repair or 3mm regression pass.

Diagnostic API caveat: installed get_cpv_pos() calls _maybe_set_motion_mode('cpv')
through _get_cpv(). With automatic mode setting enabled, it is NOT a passive
read. It was not called live. Do not add it to observation/arrival checks as a
read-only substitute without verifying mode side effects and signal semantics.
