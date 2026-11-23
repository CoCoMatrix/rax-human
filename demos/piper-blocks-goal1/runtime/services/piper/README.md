# Piper adapter against the Franka RAX backend contract

The new module implements `tools/status/validate/call/close` without copying the old workbench, task state machine, experience implementation, or orchestration. It is read-only by default. It reuses **device interfaces**: the already installed camera/robot sole owner, its bounded motion bridge, and the pinned persistent MoveIt2 KDL executable. It does not itself open CAN, enable motors, start cameras, or launch a second owner.

`examples/piper.json` is a read-only operator configuration. `piper_status` honestly reports an absent owner; offline `piper_fk` works without it. `piper_solve_ik` accepts an explicit seed and uses the5090's existing MoveIt dependencies. Configure `depth_root` to the currently running FFS producer's directory for RGBD; do not point it at old archived samples. `piper_observe(require_depth=false)` can consume the existing owner's RGB endpoints without FFS.

Live connection requires **all** operator settings below, plus operator arming of both the new RAX session and the already running owner session:

- `motion_enabled: true`
- `bridge_root`: the existing motion bridge directory containing its live `state.json`, requests/results folders
- `owner_session_root`: the session identity in that bridge's state
- `workspace_min`, `workspace_max`: explicit three-element measured workspace bounds, metres, **left_base_link → left_link6 flange**. Do not copy Franka's box or guess the table height. No grasp TCP calibration is implied.

The adapter checks41 FK samples on the joint-interpolated path against the explicit box, like Franka's sampled check. It limits left joint changes to60degrees, flange translation to100mm and left jaw increments to10mm; the existing bridge independently checks whole-path CAD self-collision, trajectory velocity/acceleration, sole-owner lease, every dispatch receipt and measured arrival. Neither the sampled box nor old CAD checks certify the unmodelled table, cup, payload, cables or unseen scene. `piper_move_pose` is **joint interpolation to a flange pose**, not Franka's straight Cartesian TCP servo. Right-arm motion and right-gripper dispatch are not exposed in this revision, although read-only FK supports both arms.

Sensor defaults are the actual head/left/right RealSense D435 identities. Head intrinsics and all13 eye-to-hand calibration are copied with checksums; head calibration remains provisional (LOO max18.08mm /2.05deg), per user instruction. Left wrist optionally supplies on-demand FFS optical depth using its own IR pair and factory intrinsics; no hand-eye transform is available. Right wrist remains RGB only. Head RGB/FFS depth is consumed from one immutable source packet with original timestamps and hashes, then saved for analysis; median7×7 valid-depth backprojection follows the Franka method. Historical analysis is not rejected solely because model latency exceeds a live-frame TTL. Before dispatch the backend checks fresh local feedback and three camera views without changing the timestamp of prior analysis.

`stream_enabled=true` enables a read-only rolling recorder consuming the existing owner and FFS readers. The exploration launcher sets it for new sessions. It writes the native RAX minute layout under `<session>/stream/`: three RGB cameras, robot JSONL, and head uint16-mm depth with an exactly paired RGB. Requested defaults are 5 Hz RGB /10 Hz robot /1 Hz depth and a600s window. `index.json` and `piper_status.recording` expose actual rates, errors and timestamps; no achieved rate is assumed. Agent files are mounted read-only and `piper_stream` reuses the Franka contact-sheet reader. Low disk space pauses recording and exposes an error. There is no new camera owner or video framework.

`finger_geometry.finger_envelopes` is shared by inspection and `installed_collision`: LEFT link6 +Z170mm tip, +Z140mm effective TCP, with the retained transverse CAD bounds. The bridge substitutes two solid convex finger volumes in the existing whole-robot checker; vendor SDK, trajectory generator, margins, exclusions and right geometry remain unchanged. Solid volumes are intentional: an obstacle entirely inside the new extension must be rejected. `collision_legacy_root` points the read-only `piper_check_path` tool at the pinned existing meshes. Environment boxes use dual_base_link; no supplied environment boxes means self-collision checks only. Measurements and provisional camera calibration remain explicitly unvalidated for physical clearance.

This module cannot start a clean hardware run by itself. The external owner/camera/FFS processes must already be provisioned and running with the documented existing contracts. Its live bridge path is real code, but this revision has not yet been exercised on physical hardware. Software/IK checks are not a claim of new device qualification or end-to-end task reproduction.

`assets/dual_piper_x.urdf` is the prior CAD snapshot, used only for FK/limits; mesh resources are not loaded. Its joint6 ±120degree limit is intentionally retained despite the generic owner profile's ±180degree value. `assets/moveit-runtime.json` pins shared read-only executable/model/plugin inputs. Logs and ROS temporary files are redirected to the **new session** directory.

Left FFS is launched by piper_explore with the existing camera owner exporting
left IR1/IR2. It reuses the installed FFS Processor, max_disp384, four iterations,
LR consistency, no hole filling. Default head depth is unchanged. Explicit
`depth_cameras=["left"]` requests inference; `output_frame="camera_optical"` is
required for left pixel queries. Left points are never marked robot-target usable.
Shared camera hook changes are opt-in via REALSENSE_LEFT_STEREO_*; original head
callers and head health fields remain compatible. Packets/results retain16 entries
with the existing retention mechanism. Dual-depth arrays use uncompressed NPZ to
avoid compression consuming acquisition freshness; source timestamps are preserved.
