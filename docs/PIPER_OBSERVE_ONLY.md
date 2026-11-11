# Piper observation-only bring-up

`scripts/piper_observe_only.py` reuses the existing device modules in the pinned
XPolicy owner runtime. Its derived graph contains only the three-camera source,
Piper owner, and localhost observation UI. The owner has only timer and camera
status inputs; UI commands, Prepare, Start, model actions, and transition commands
have no route to it. `ENABLE_WRITE=0` is explicit on every node.

The default command prints the full plan and source hashes without creating a
session or opening hardware:

```sh
cd /home/ubuntu/code_junqi/rax-piper-franka-20261104
PYTHONDONTWRITEBYTECODE=1 \
  /home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python \
  scripts/piper_observe_only.py \
  --output /home/ubuntu/code_junqi/rax-piper-franka-20261104/commissioning/observe-only-20261104-r2 \
  --seconds 300
```

Adding `--run` opens the cameras and read-only CAN feedback for that bounded
session. The launcher refuses an existing owner, camera, or UI process and refuses
occupied ports 8091 or 19027. It does not stop other sessions or perform stale
cleanup. Its shutdown signals only its own child process groups.

Existing SDK initialization constructs the arm/gripper readers, opens CAN sockets,
and starts receive/FPS threads. With `ENABLE_WRITE=0`, the owner skips startup
enable, gripper initialization, Prepare, and final hardware hold. SDK disconnect
stops threads and closes sockets; it does not disable motors or change mode.
Consequently this is an observation run, not a way to change an already enabled
robot's state.

The existing `volatile_sensors` helper creates private tmpfs packet, depth, and
snapshot bindings. FFS is warmed with a labelled archived packet before cameras
start. Live samples retain original timestamps and the existing one-second TTL.
All child producers stop before SHA-verified archival into the new session
directory. Failed archival preserves the tmpfs evidence. No task FSM, workbench,
policy node, motion bridge, or model agent is started.

The run writes `backend-options.json` with the actual observation address and
`depth_root`. Use those values with the new Piper backend. `motion_enabled` remains
false. A new RAX session takes its own immutable snapshots for model inspection;
the bounded launcher also records three camera groups and sampled owner state.
`hardware_writes` is unknown (`null`) until an owner status was actually read; the
launcher rejects any write-enabled status or nonzero hardware write count.

Current validation, 2026-11-04:

- Graph and reporting unit tests: 3 passed.
- On the 5090 host, existing tmpfs helper create/write/capacity/archive round trip
  passed with verified content hashes; no devices were opened by that test.
- First run `observe-only-20261104-r1` failed before owner samples because the
  reused UI requires TASK membership in its catalog. The launcher now creates a
  single-entry observation catalog. The old UI constructor reproduced the error
  and passed with that catalog in a hardware-free test. The failed run's evidence
  was archived successfully; its hardware write count remains unknown.
- An earlier `r2` attempt was refused before directory/device creation because a separate
  deployment, `piper-blue-cup-holder-only-rtc6-step1600`, had started from
  `/home/ubuntu/michael_dev/xpolicylab-robot-runtime` with `ENABLE_WRITE=1` and UI
  port 8091. This launcher did not stop, replace, or attach to that deployment.
  Observation acceptance resumed only after the devices became available.
- The completed five-minute `r2` observation run passed: 589 owner samples over
  295.63 seconds, all write-disabled with hardware write counts zero, and no change
  in the 14 measured joint/gripper values. It captured three camera snapshot groups
  and exported 589 head stereo packets. The three cameras each delivered about
  8,800 frames with no timeout or restart; they reported 10/12/11 dropped frames.
  The ten connection-refused entries occurred while the UI was starting. The
  graph exited with code 0; FFS received the launcher's termination signal (-15).
  All device processes exited, ports 8091/19027 were free, and archival verified
  hashes for 147 files. See `verification/piper-observe-only-r2/{RESULT,SUMMARY}.json`.
- Head RGB and depth are bound to the same immutable stereo/RGB packet. Wrist RGB
  images use an HTTP metadata bracket, not an exact frame binding. The three
  snapshot groups span 0.577–0.830 seconds across their source timestamps: these
  are not hardware-synchronized multiview observations. Head images are
  1280×720; the wrist UI images are 960×540 and have no calibrated metric depth.
- Both selected models successfully inspected actual images from this observation
  run through the API. A separate `r3` native Codex + MCP + real Piper NO_SEND
  integration also passed in 32.29 seconds: one status, one three-view RGB observe,
  and one auxiliary vision call, with no MCP errors or motion calls. The primary
  model inspected all three images. That check used RGB-only, so all three images
  have HTTP metadata brackets and `metric_depth=false`, not exact RGB-D binding.
  Its graph recorded 350 all-write-disabled samples, maximum hardware write count
  zero, verified archival and complete device-process exit. Startup/shutdown stale
  feedback and connection-refused warnings remain in the record. See
  `verification/live-native-r3/summary.json` and the forwarded camera images there.

Motion bring-up is a separate step. This graph cannot be promoted by sending UI
commands. A live motion session needs its explicitly configured sole-owner bridge
and operator arming, plus a measured workspace box in `left_base_link` for the
`left_link6` flange. Camera calibration and CAD reach do not define table clearance
or a collision-free work area. The current backend checks sampled FK along joint
interpolation and preserves existing owner limits, but has no scene collision
model for the table, cup, or payload. No motion or workspace validation has been
performed by this observation launcher.

A separate supervised minimal live test subsequently passed one left-J1
half-degree round trip through the new Piper backend, with measured arrival,
independent owner Stop acknowledgement and fresh rest feedback. Its initialization
included dual-arm Enable and Prepare at the measured current dual-arm targets;
jaw dispatch was disabled. This does not change the observation launcher's
NO_SEND contract. See `VALIDATION.md` and
`verification/piper-minimal-live-r1/SUMMARY.json` for the exact scope and evidence.
