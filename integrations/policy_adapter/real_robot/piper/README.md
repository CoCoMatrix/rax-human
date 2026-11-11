# Pure PiPER contract and kinematics adapter

This package consumes the existing XPolicy dual PiPER `ObservationFrame` and
policy-result contract. It never creates a robot/camera SDK instance, opens a
device, starts a graph, or owns hardware writes. The formal XPolicy launcher,
graph, arbitrator and sole owner remain responsible for observation production
and execution. RoboDojo/ARX observations and actions cannot be relabeled as PiPER.

## Observation and candidate API

```python
from real_robot.piper.contracts import PiperContract, CandidateEnvelope

contract = PiperContract.from_runtime(
    "/home/ubuntu/code_junqi/xpolicylab-robot-runtime",
    robot_id="piper_x_dual_01", horizon=32, action_prefix=16,
    # Deployment can additionally pin expected_profile_sha256 / expected_urdf_sha256.
)
frame = contract.validate_observation(
    observation_frame_or_wire_mapping,
    max_age_ms=explicit_application_age_limit,
    previous_sequence=previous_sequence_or_none,
)
```

The input is either the actual selected runtime's `ObservationFrame` instance,
or exactly its `to_policy_observation()` wire format:

- `data_format_version: xpolicylab.real_world.v1` and nonempty `instruction`.
- `state.vector`: finite `float32[14]`; no list conversion, flattening, padding,
  dimensional truncation or unit conversion. Field order is left joints 1–6,
  left gripper, right joints 1–6, right gripper. Joints use radians; grippers
  use metres and absolute width. Schema ID is `xpolicy.piper_x_dual.state.v1`.
- `vision`: exactly `camera_head`, `camera_left`, `camera_right`, each containing
  nonempty `uint8[H,W,3]` color. Images are already RGB at the formal policy
  boundary: the owner's observation adapter converts camera BGR there. A
  supplied encoding must be `rgb8`; this backend never guesses or converts it.
- `additional_info`: exact robot type/id, positive integer `timestamp_ns`,
  nonnegative integer `sequence`, and optional mapping `metadata`. Existing
  `inference_context`, including queued actions, survives the round trip.

`PiperObservation` provides immutable copies of `state14`, `images`, task,
timestamp, sequence, identity and metadata. Profile-defined feedback tolerance
is accepted for measurements without clipping them. `max_age_ms` is an explicit
caller setting; omitted or future timestamps fail. Passing `previous_sequence`
also rejects replay/nonadvancing frames.

The selected checkout's profile, URDF and physical device configuration are
validated by the authoritative profile module and pinned by SHA256. The module
checks those files have not changed before returning a validated observation
or candidate. A previously imported different XPolicy checkout is rejected.

**Evidence limits:** the current formal observation does not contain individual
camera capture timestamps or profile/URDF hashes. This adapter checks the
observation's state timestamp, RGB array contract and locally selected robot
assets. It cannot independently certify image freshness, cross-camera time
alignment, camera serial identity or physical robot identity from pixels. Those
remain the formal owner's device-binding/readiness responsibility. Attached
`piper_backend_identity` metadata, if present, must exactly match the local
contract; it is a contract record, not hardware attestation.

For a strict executable candidate:

```python
checked = contract.validate_candidate(
    actions,                         # floating ndarray[N,14], 1 <= N <= 32
    schema=policy_result["action_schema"],
    frequency_hz=policy_result["action_frequency_hz"],  # exactly 30 Hz
    measured_state14=frame.state14,
    prefix_steps=16,                  # <= configured 16 and <= N
    envelope=CandidateEnvelope(       # optional, explicitly caller declared
        max_joint_delta_from_measured_rad=0.35,
        max_joint_velocity_rad_s=0.15,
    ),
)
formal_result = checked.to_policy_result()
```

The example envelope is an application choice, not a new robot default. With no
envelope, only the actual profile command ranges and stated contract apply.
All N rows are validated and returned unchanged; the formal runtime owns prefix
scheduling. No candidate slicing or conversion to an executable prefix occurs
here. Checks additionally consider float32 transport quantization. Metrics
include the first step from the measured state. `to_policy_result()` does not
authorize dispatch or change any write gate.

Raw learned predictions and owner-normalized effective commands are distinct.
The original owner may snap small joint overshoots and clamp gripper widths
using `live_takeover.model_boundary_normalization`. A shadow caller should keep
raw model output unchanged, separately record every owner-normalizer adjustment,
and validate an effective **float64** row with `validate_command_state()` before
preview FK. An effective endpoint such as exactly 0.1 m must not be quantized
again and presented as the original raw candidate: converting it to float32 can
cross the mathematical boundary by one rounding unit. This strict candidate
validator intentionally refuses such transport candidates; it never silently
applies the owner's normalization.

## FK and IK

```python
from real_robot.piper.kinematics import PiperKinematics, transform_from_pose

kin = PiperKinematics(contract)
pose = kin.forward(frame.state14)
# pose.left_transform, pose.right_transform: [4,4]
# pose.positions_m: [left/right, xyz]
# pose.quaternions_wxyz: [left/right, wxyz]
# pose.state14: exact input measurement, preserved

candidate = kin.inverse(
    frame.state14,
    left_target=left_flange_transform,
    right_target=right_flange_transform,
    coordinate_frame="dual_base_link",
)
# candidate.action14 keeps both measured gripper widths unchanged.
```

Both arms use the profile's translated base mounts and endpoint links
`left_link6`/`right_link6`; there is no mirroring or ARX base transform. Targets
are **flange** transforms in `dual_base_link`, with `x` forward, `y` left and
`z` up. They are not an inferred fingertip/tool-center pose. All positions are
metres and all joints are radians. Quaternion helpers explicitly use `wxyz`;
they reject nonunit quaternions rather than silently normalize them. Flat
16-element transforms, reflections and malformed homogeneous matrices fail.

Mink, MuJoCo and SciPy are imported only when kinematics is used. The wrapper
reuses `DualPiperMinkIK` and the repository URDF, with an internal lock for
concurrent preview calls. FK refuses measured joints requiring hidden clipping.
IK requires exact commandable seeds/grippers, converged solves, profile joint
limits and the profile's solution-jump bound. It independently rechecks the
returned joint vector with FK and repeats acceptance for its float32 wire form.
Optional tighter residual tolerances may be supplied; relaxing profile
tolerances is rejected. Any failure raises an error and returns no fallback
movement. The URDF is **kinematics only**; no collision certification is claimed.

## Offline verification

With the local package root on `PYTHONPATH`, set `PIPER_RUNTIME_ROOT` to the
selected authoritative checkout and run `pytest -q real_robot/piper/tests`.
The tests use synthetic observations and actual local profile/URDF/Mink
kinematics. They never connect to hardware, cameras, model services or a GPU.
They cover wire schema/units/roles/identity/freshness, immutable round trips,
candidate rejection and envelope/float32 checks, measured-noise preservation,
flange mount geometry, wxyz round trips, actual small dual-arm IK, and solver
failure/jump/false-convergence rejection. Without `PIPER_RUNTIME_ROOT`, tests
requiring the authoritative checkout are skipped; without Mink, actual IK
tests are skipped. A passing software test is not a live hardware qualification.
