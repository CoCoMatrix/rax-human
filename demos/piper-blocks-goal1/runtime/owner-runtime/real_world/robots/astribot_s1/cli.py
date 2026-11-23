from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from XPolicyLab.real_world.contracts import ActionChunk
from XPolicyLab.real_world.robots.astribot_s1.camera import (
    AstribotS1CameraSource,
    AstribotS1ObservationAdapter,
)
from XPolicyLab.real_world.robots.astribot_s1.dagger import (
    AstribotS1DaggerOwnerRuntime,
)
from XPolicyLab.real_world.robots.astribot_s1.driver import (
    AstribotS1Driver,
    AstribotS1FullBodyDriver,
    AstribotS1JointDriver,
    AstribotS1UpperBodyDriver,
)
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ARM_NAMES,
    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
    ASTRIBOT_S1_STATE_SCHEMA,
    GRIPPER_NAMES,
    arm_joint_action_dict,
    current_arm_joint_action,
    current_auxiliary_action,
    current_full_body_hold_action,
    current_pose_action,
    full_body_action_vector,
    gripper_action_dict,
    pose_to_matrix,
)
from XPolicyLab.real_world.robots.astribot_s1.owner import (
    AstribotS1OwnerRuntime,
    build_hold_action_chunk,
)
from XPolicyLab.real_world.robots.astribot_s1.qnexo_bridge import (
    QnexoEndEffectorSource,
)
from XPolicyLab.real_world.robots.astribot_s1.rollout import (
    AstribotS1RolloutRecorder,
)
from XPolicyLab.real_world.robots.astribot_s1.runtime import (
    AstribotS1CartesianRuntime,
    AstribotS1FullBodyRuntime,
)
from XPolicyLab.real_world.robots.astribot_s1.safety import (
    AstribotS1CartesianSafetyGuard,
    AstribotS1AuxiliarySafetyGuard,
    AstribotS1GripperSafetyGuard,
    AstribotS1JointSafetyGuard,
)
from XPolicyLab.real_world.robots.astribot_s1.sdk import (
    AstribotS1SdkSession,
    RobotSnapshot,
)
from XPolicyLab.real_world.robots.astribot_s1.teleop import (
    QnexoCartesianRetargeter,
    QnexoGripperRetargeter,
    QnexoJointDeltaRetargeter,
    build_qnexo_torso_retargeter,
)
from XPolicyLab.real_world.teleop import (
    TeleopControlLifecycle,
    validate_teleop_profile,
)

DEFAULT_PROFILE = Path(__file__).with_name("profile.yml")


def _profile(path: str | Path) -> dict[str, Any]:
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if value["robot_type"] != "astribot_s1":
        raise ValueError("profile is not for Astribot S1")
    validate_teleop_profile(value)
    return value


def _require_connect_ack(args: argparse.Namespace) -> None:
    if not args.acknowledge_sdk_connect:
        raise RuntimeError(
            "SDK construction can acquire/restart/stop the robot; pass "
            "--acknowledge-sdk-connect with the operator present"
        )
    if args.enable_write and not args.acknowledge_workspace_clear:
        raise RuntimeError("hardware write requires --acknowledge-workspace-clear")


def _session(args: argparse.Namespace, profile: dict[str, Any]) -> AstribotS1SdkSession:
    _require_connect_ack(args)
    return AstribotS1SdkSession(
        frequency_hz=float(profile["sdk"]["frequency_hz"]),
        high_control_rights=args.high_control_rights,
    )


def _current_action(
    snapshot: RobotSnapshot,
    joint_limits: dict[str, Any],
) -> dict[str, float]:
    action = current_pose_action(
        snapshot.arm_poses,
        snapshot.joints,
        snapshot.desired_grippers,
    )
    for side, part in zip(("left", "right"), GRIPPER_NAMES, strict=True):
        lower, upper = joint_limits[part]
        action[f"{side}.gripper"] = float(
            np.clip(action[f"{side}.gripper"], lower[0], upper[0])
        )
    return action


def _home_reached(
    snapshot: RobotSnapshot,
    home: dict[str, float],
    *,
    position_tolerance_m: float,
    rotation_tolerance_rad: float,
    gripper_tolerance_percent: float,
) -> bool:
    for side, arm, gripper in zip(
        ("left", "right"), ARM_NAMES, GRIPPER_NAMES, strict=True
    ):
        current = pose_to_matrix(snapshot.arm_poses[arm])
        target = pose_to_matrix(
            [home[f"{side}.{name}"] for name in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        )
        if np.linalg.norm(target[:3, 3] - current[:3, 3]) > position_tolerance_m:
            return False
        relative = target[:3, :3] @ current[:3, :3].T
        angle = float(np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)))
        if angle > rotation_tolerance_rad:
            return False
        if (
            abs(float(snapshot.joints[gripper][0]) - home[f"{side}.gripper"])
            > gripper_tolerance_percent
        ):
            return False
    return True


def doctor(args: argparse.Namespace) -> int:
    checks: dict[str, Any] = {
        "python": sys.version.split()[0],
        "ros_distro": os.getenv("ROS_DISTRO"),
        "ros_domain_id": os.getenv("ROS_DOMAIN_ID"),
        "robot_type": os.getenv("ROBOT_TYPE"),
        "sdk_root": os.getenv("ASTRIBOT_SDK_ROOT"),
        "rmw_implementation": os.getenv("RMW_IMPLEMENTATION"),
    }
    imports = {}
    for name in ("rclpy", "astribot_msgs", "astribot_sdk", "serial"):
        try:
            module = importlib.import_module(name)
            imports[name] = getattr(module, "__version__", "ok")
        except Exception as error:
            imports[name] = f"ERROR: {type(error).__name__}: {error}"
    checks["imports"] = imports
    checks["qnexo_port"] = Path(args.qnexo_port).exists()
    checks["footswitch"] = Path(args.footswitch_device).exists()
    checks["teleop_ready"] = bool(
        checks["qnexo_port"] and (checks["footswitch"] or args.button_deadman)
    )
    expected = {
        "ros_distro": "humble",
        "ros_domain_id": "25",
        "robot_type": "S1",
    }
    errors = [
        f"{name}={checks[name]!r}, expected {value!r}"
        for name, value in expected.items()
        if checks[name] != value
    ]
    errors.extend(
        f"import {name}: {value}"
        for name, value in imports.items()
        if str(value).startswith("ERROR:")
    )
    checks["status"] = "PASS" if not errors else "FAIL"
    checks["errors"] = errors
    print(json.dumps(checks, indent=2, sort_keys=True))
    return 0 if not errors else 1


def observe(args: argparse.Namespace) -> int:
    profile = _profile(args.profile)
    with _session(args, profile) as session:
        print(
            json.dumps(
                {
                    "status": "CONNECTED",
                    "joint_limits": session.joint_limits,
                },
                sort_keys=True,
            )
        )
        for _ in range(args.samples):
            snapshot = session.snapshot()
            print(json.dumps(asdict(snapshot), sort_keys=True))
            time.sleep(args.interval)
    return 0


def qnexo_observe(args: argparse.Namespace) -> int:
    """Observe the shared QNexo controls without constructing a robot SDK."""
    if args.duration <= 0:
        raise ValueError("duration must be positive")
    source = QnexoEndEffectorSource(
        port=args.qnexo_port,
        footswitch_device=args.footswitch_device,
        footswitch_key_code=args.footswitch_key_code,
        save_footswitch_key_code=args.save_footswitch_key_code,
        footswitch_toggle_on_press=args.footswitch_toggle_on_press,
        button_deadman=args.button_deadman,
        urdf_path=args.exo_urdf,
    )
    deadline = time.monotonic() + float(args.duration)
    last_operator_enabled: bool | None = None
    next_trigger_report = 0.0
    samples = 0
    with source:
        print(json.dumps({"status": "QNEXO_CONTROL_OBSERVE_STARTED"}))
        while time.monotonic() < deadline:
            sample = source.poll()
            if sample is None:
                time.sleep(0.004)
                continue
            samples += 1
            if sample.operator_enabled != last_operator_enabled:
                print(
                    json.dumps(
                        {
                            "status": "OPERATOR_ENABLE_CHANGED",
                            "operator_enabled": sample.operator_enabled,
                        },
                        sort_keys=True,
                    )
                )
                last_operator_enabled = sample.operator_enabled
            if sample.control_event is not None:
                print(
                    json.dumps(
                        {
                            "status": "BUTTON_EVENT",
                            "button": sample.control_button,
                            "event": sample.control_event,
                            "timestamp_ns": sample.timestamp_ns,
                        },
                        sort_keys=True,
                    )
                )
            monotonic_now = time.monotonic()
            if monotonic_now >= next_trigger_report:
                print(
                    json.dumps(
                        {
                            "status": "QNEXO_CONTROL_SAMPLE",
                            "buttons": sample.buttons,
                            "left_joystick_x": sample.payload["left_joystick_x"],
                            "left_joystick_y": sample.payload["left_joystick_y"],
                            "right_joystick_x": sample.payload["right_joystick_x"],
                            "right_joystick_y": sample.payload["right_joystick_y"],
                            "left_trigger": sample.payload["left_trigger"],
                            "right_trigger": sample.payload["right_trigger"],
                        },
                        sort_keys=True,
                    )
                )
                next_trigger_report = monotonic_now + 0.5
        print(
            json.dumps(
                {"status": "QNEXO_CONTROL_OBSERVE_COMPLETE", "samples": samples},
                sort_keys=True,
            )
        )
    return 0


def camera_observe(args: argparse.Namespace) -> int:
    """Read synchronized three-camera/robot observations without dispatching."""
    if args.samples <= 0 or args.interval < 0 or args.timeout <= 0:
        raise ValueError(
            "samples and timeout must be positive; interval cannot be negative"
        )
    profile = _profile(args.profile)
    activate = bool(
        args.activate_cameras or profile["cameras"].get("activate_on_start", False)
    )
    with _session(args, profile) as session:
        source = AstribotS1CameraSource.from_profile(args.profile, session)
        adapter = AstribotS1ObservationAdapter(source)
        try:
            camera_info = source.start(activate=activate)
            print(
                json.dumps(
                    {
                        "status": "CAMERAS_SUBSCRIBED",
                        "activated_by_command": activate,
                        "camera_info": camera_info,
                        "streams": dict(source.camera_streams),
                    },
                    sort_keys=True,
                    default=str,
                )
            )
            deadline = time.monotonic() + float(args.timeout)
            ready_samples = 0
            sequence = 0
            last_hold: tuple[str, ...] | None = None
            while ready_samples < args.samples and time.monotonic() < deadline:
                snapshot = session.snapshot()
                observation, decision = adapter.build(
                    snapshot,
                    "",
                    sequence=sequence,
                    robot_id=args.robot_id,
                    now_ns=time.time_ns(),
                )
                if observation is None:
                    if decision.reasons != last_hold:
                        print(
                            json.dumps(
                                {
                                    "status": decision.status,
                                    "reasons": decision.reasons,
                                },
                                sort_keys=True,
                            )
                        )
                        last_hold = decision.reasons
                else:
                    last_hold = None
                    camera_metadata = observation.metadata["camera_sync"]["cameras"]
                    print(
                        json.dumps(
                            {
                                "status": decision.status,
                                "sequence": sequence,
                                "state_shape": list(observation.state.shape),
                                "image_shapes": {
                                    name: list(image.shape)
                                    for name, image in observation.images.items()
                                },
                                "state_timestamp_ns": snapshot.timestamp_ns,
                                "camera_source_timestamps_ns": {
                                    name: metadata["source_timestamp_ns"]
                                    for name, metadata in camera_metadata.items()
                                },
                                "camera_aligned_timestamps_ns": dict(
                                    decision.aligned_timestamps_ns
                                ),
                                "camera_clock_offset_ms": (
                                    decision.camera_clock_offset_ns / 1_000_000
                                ),
                                "max_camera_skew_ms": (
                                    decision.max_camera_skew_ns / 1_000_000
                                ),
                                "max_state_skew_ms": (
                                    decision.max_state_skew_ns / 1_000_000
                                ),
                            },
                            sort_keys=True,
                        )
                    )
                    ready_samples += 1
                    sequence += 1
                if ready_samples < args.samples:
                    time.sleep(args.interval)
            if ready_samples != args.samples:
                print(
                    json.dumps(
                        {
                            "status": "TIMEOUT",
                            "ready_samples": ready_samples,
                            "requested_samples": args.samples,
                            "last_reasons": last_hold or (),
                        },
                        sort_keys=True,
                    )
                )
                return 4
            return 0
        finally:
            source.close()


def owner_shadow(args: argparse.Namespace) -> int:
    """Run the unified owner with a measured hold chunk and no SDK writes."""
    if args.enable_write:
        raise RuntimeError("owner-shadow never permits --enable-write")
    if args.samples <= 0 or args.timeout <= 0:
        raise ValueError("samples and timeout must be positive")
    profile = _profile(args.profile)
    chunk_steps = (
        int(args.chunk_steps)
        if args.chunk_steps is not None
        else int(profile["owner"]["dummy_chunk_steps"])
    )
    if chunk_steps <= 0:
        raise ValueError("chunk-steps must be positive")
    activate = bool(
        args.activate_cameras or profile["cameras"].get("activate_on_start", False)
    )
    generation = 1
    with _session(args, profile) as session:
        owner = AstribotS1OwnerRuntime.from_profile(
            args.profile,
            session,
            enable_write=False,
            robot_id=args.robot_id,
            instruction=args.instruction,
        )
        dagger = AstribotS1DaggerOwnerRuntime(owner)
        report_root = Path(
            args.report_root or profile["rollout"]["output_root"]
        ).expanduser()
        if not report_root.is_absolute():
            report_root = Path(__file__).resolve().parents[3] / report_root
        recorder = AstribotS1RolloutRecorder(
            report_root,
            mode="shadow",
            robot_id=args.robot_id,
            instruction=args.instruction,
            generation=generation,
            profile_id=profile["profile_id"],
            state_schema=ASTRIBOT_S1_STATE_SCHEMA,
            action_schema=ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
            rollout_id=args.rollout_id,
        )
        finalized = False
        try:
            camera_info = dagger.start(activate_cameras=activate)
            initial_snapshot = session.snapshot()
            dagger.controller.update_snapshot(initial_snapshot)
            dagger.request_model("step")
            episode = dagger.set_episode(True)
            generation = episode.generation
            hold_chunk = build_hold_action_chunk(
                initial_snapshot,
                steps=chunk_steps,
                frequency_hz=owner.action_runtime.action_frequency_hz,
                generation=generation,
            )
            recorder.record_lifecycle(
                "owner_shadow_started",
                camera_info=camera_info,
                activated_by_command=activate,
                chunk_steps=chunk_steps,
                consumed_steps=0,
            )
            print(
                json.dumps(
                    {
                        "status": "OWNER_SHADOW_STARTED",
                        "camera_info": camera_info,
                        "activated_by_command": activate,
                        "generation": generation,
                        "chunk_steps": chunk_steps,
                        "consumed_steps": 0,
                        "observation_frequency_hz": (owner.observation_frequency_hz),
                        "driver_frequency_hz": (
                            owner.action_runtime.driver_frequency_hz
                        ),
                        "hardware_write_enabled": False,
                        "report_dir": str(recorder.rollout_dir),
                    },
                    sort_keys=True,
                    default=str,
                )
            )

            started = time.monotonic()
            deadline = started + float(args.timeout)
            period = 1.0 / owner.action_runtime.driver_frequency_hz
            next_tick = time.monotonic()
            next_model = next_tick
            model_period = 1.0 / dagger.controller.action_frequency_hz
            tick_count = 0
            shadow_tick_count = 0
            observation_count = 0
            seen_steps: set[int] = set()
            step_paused = False
            last_status: tuple[Any, ...] | None = None
            while time.monotonic() < deadline:
                monotonic_now = time.monotonic()
                routed = None
                if owner.camera_ready and monotonic_now >= next_model:
                    step_index = len(seen_steps)
                    if step_index < chunk_steps:
                        routed = dagger.update_model_action(
                            hold_chunk.values[step_index],
                            generation,
                            model_step_index=step_index,
                        )
                    elif not step_paused:
                        dagger.update_model_status("paused", generation)
                        step_paused = True
                    while next_model <= monotonic_now:
                        next_model += model_period
                    if routed is not None and routed.model_step_index is not None:
                        seen_steps.add(routed.model_step_index)
                decision = dagger.tick(operator_enabled=True)
                recorder.record_tick(decision)
                tick_count += 1
                if decision.wrote:
                    raise RuntimeError("owner-shadow unexpectedly wrote hardware")
                if decision.status == "SHADOW":
                    shadow_tick_count += 1
                signature = (
                    decision.status,
                    decision.reasons,
                    decision.action.chunk_step_index,
                )
                if signature != last_status and decision.action.arm_safety.reasons != (
                    "driver_tick_not_due",
                ):
                    print(
                        json.dumps(
                            {
                                "status": decision.status,
                                "reasons": decision.reasons,
                                "chunk_step_index": (decision.action.chunk_step_index),
                                "queue_depth": len(owner.action_runtime.queue.actions),
                            },
                            sort_keys=True,
                        )
                    )
                    last_status = signature

                if decision.observation is not None:
                    observation_count += 1
                    print(
                        json.dumps(
                            {
                                "status": "OBSERVATION_READY",
                                "sequence": decision.observation.sequence,
                                "state_shape": list(decision.observation.state.shape),
                                "image_shapes": {
                                    name: list(image.shape)
                                    for name, image in (
                                        decision.observation.images.items()
                                    )
                                },
                                "max_camera_skew_ms": (
                                    decision.camera_sync.max_camera_skew_ns / 1_000_000
                                ),
                                "max_state_skew_ms": (
                                    decision.camera_sync.max_state_skew_ns / 1_000_000
                                ),
                            },
                            sort_keys=True,
                        )
                    )

                complete_steps = seen_steps == set(range(chunk_steps))
                if (
                    step_paused
                    and not owner.action_runtime.enabled
                    and complete_steps
                    and observation_count >= args.samples
                ):
                    elapsed = time.monotonic() - started
                    summary = recorder.finalize(
                        outcome="pass",
                        termination_reason="shadow_contract_complete",
                    )
                    finalized = True
                    print(
                        json.dumps(
                            {
                                "status": "PASS",
                                "hardware_writes": 0,
                                "observations": observation_count,
                                "executed_chunk_steps": sorted(seen_steps),
                                "owner_ticks": tick_count,
                                "shadow_action_ticks": shadow_tick_count,
                                "elapsed_seconds": elapsed,
                                "effective_owner_hz": tick_count / elapsed,
                                "report_dir": str(recorder.rollout_dir),
                                "report_summary": summary,
                            },
                            sort_keys=True,
                        )
                    )
                    return 0

                next_tick += period
                sleep_seconds = next_tick - time.monotonic()
                if sleep_seconds > 0:
                    time.sleep(sleep_seconds)

            summary = recorder.finalize(
                outcome="timeout",
                termination_reason="shadow_contract_timeout",
            )
            finalized = True
            print(
                json.dumps(
                    {
                        "status": "TIMEOUT",
                        "hardware_writes": 0,
                        "observations": observation_count,
                        "executed_chunk_steps": sorted(seen_steps),
                        "owner_ticks": tick_count,
                        "report_dir": str(recorder.rollout_dir),
                        "report_summary": summary,
                    },
                    sort_keys=True,
                )
            )
            return 4
        except KeyboardInterrupt:
            if not finalized:
                recorder.finalize(
                    outcome="aborted",
                    termination_reason="operator_interrupt",
                )
                finalized = True
            print(
                json.dumps(
                    {
                        "status": "ABORTED",
                        "report_dir": str(recorder.rollout_dir),
                    },
                    sort_keys=True,
                )
            )
            return 130
        except Exception as error:
            if not finalized:
                recorder.finalize(
                    outcome="fault",
                    termination_reason=(f"{type(error).__name__}: {str(error)[:160]}"),
                )
                finalized = True
            raise
        finally:
            if not finalized:
                recorder.finalize(
                    outcome="aborted",
                    termination_reason="owner_closed_without_result",
                )
            dagger.close()


def hold_current(args: argparse.Namespace) -> int:
    profile = _profile(args.profile)
    guard = AstribotS1CartesianSafetyGuard.from_profile(args.profile)
    with _session(args, profile) as session:
        snapshot = session.snapshot()
        action = _current_action(snapshot, session.joint_limits)
        now_ns = time.time_ns()
        safety = guard.evaluate(
            action,
            snapshot.arm_poses,
            snapshot.joints,
            session.joint_limits,
            enabled=True,
            robot_alive=snapshot.robot_alive,
            have_control_rights=snapshot.have_control_rights,
            generation=1,
            expected_generation=1,
            state_timestamp_ns=snapshot.timestamp_ns,
            command_timestamp_ns=now_ns,
            now_ns=now_ns,
        )
        if safety.status != "PASS":
            print(json.dumps({"safety": asdict(safety)}, sort_keys=True))
            return 2
        driver = AstribotS1Driver(
            session,
            enabled=args.enable_write,
            control_way=profile["sdk"]["control_way"],
            use_wbc=bool(profile["sdk"]["use_wbc"]),
            include_grippers=False,
        )
        decision = driver.send(
            safety.action,
            safety_status=safety.status,
            operator_enabled=True,
            robot_ready=(
                snapshot.robot_alive
                and snapshot.driver_alive
                and snapshot.have_control_rights
            ),
            generation=1,
            expected_generation=1,
        )
        print(
            json.dumps(
                {"safety": asdict(safety), "driver": asdict(decision)},
                sort_keys=True,
            )
        )
        return 0 if decision.status in {"SENT", "HOLD"} else 3


def hold_current_joints(args: argparse.Namespace) -> int:
    """Submit the currently measured arm joints once through the joint gate."""
    profile = _profile(args.profile)
    guard = AstribotS1JointSafetyGuard.from_profile(args.profile)
    with _session(args, profile) as session:
        snapshot = session.snapshot()
        action = current_arm_joint_action(snapshot.joints)
        command_timestamp_ns = time.time_ns()
        safety = guard.evaluate(
            action,
            snapshot.joints,
            session.joint_limits,
            enabled=True,
            robot_alive=snapshot.robot_alive,
            have_control_rights=snapshot.have_control_rights,
            generation=1,
            expected_generation=1,
            state_timestamp_ns=snapshot.timestamp_ns,
            command_timestamp_ns=command_timestamp_ns,
            now_ns=time.time_ns(),
        )
        if not safety.accepted:
            print(json.dumps({"safety": asdict(safety)}, sort_keys=True))
            return 2
        driver = AstribotS1JointDriver(
            session,
            enabled=args.enable_write,
            control_way=profile["sdk"]["control_way"],
            use_wbc=False,
            include_grippers=False,
        )
        decision = driver.send(
            safety.action,
            safety_status=safety.status,
            operator_enabled=True,
            robot_ready=(
                snapshot.robot_alive
                and snapshot.driver_alive
                and snapshot.have_control_rights
            ),
            generation=1,
            expected_generation=1,
        )
        print(
            json.dumps(
                {"safety": asdict(safety), "driver": asdict(decision)},
                sort_keys=True,
            )
        )
        return 0 if decision.status in {"SENT", "HOLD"} else 3


def full_body_smoke(args: argparse.Namespace) -> int:
    """Supervised zero-base 25D hold, optional tiny arm motion, and return."""
    if not args.enable_write:
        raise RuntimeError("full-body-smoke requires --enable-write")
    delta_rad = float(args.delta_rad)
    phase_seconds = float(args.phase_seconds)
    if not np.isfinite(delta_rad) or abs(delta_rad) > 0.02:
        raise ValueError("delta-rad must be finite and no greater than 0.02")
    if not np.isfinite(phase_seconds) or not 0.1 <= phase_seconds <= 1.0:
        raise ValueError("phase-seconds must be between 0.1 and 1.0")

    profile = _profile(args.profile)
    with _session(args, profile) as session:
        driver = AstribotS1FullBodyDriver(
            session,
            enabled=True,
            control_way=profile["sdk"]["control_way"],
            use_wbc=False,
            include_grippers=True,
        )
        runtime = AstribotS1FullBodyRuntime.from_profile(args.profile, driver)
        initial = session.snapshot()
        initial_action = current_full_body_hold_action(initial.joints)
        motion_action = dict(initial_action)
        motion_field = "astribot_arm_left.joint_0"
        motion_action[motion_field] += delta_rad

        for action in (initial_action, motion_action):
            if any(
                action[name] != 0.0
                for name in ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.fields[:3]
            ):
                raise RuntimeError("full-body-smoke refuses non-zero chassis velocity")

        def execute_phase(
            label: str,
            action: dict[str, float],
            generation: int,
        ) -> int:
            steps = max(
                1,
                int(phase_seconds * runtime.action_frequency_hz) + 2,
            )
            row = full_body_action_vector(action)
            chunk = ActionChunk(
                schema=ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
                values=np.repeat(row[None, :], steps, axis=0),
                frequency_hz=runtime.action_frequency_hz,
                generation=generation,
                metadata={"source": "supervised_full_body_smoke", "phase": label},
            )
            runtime.control("run", generation)
            pending = runtime.begin_inference()
            if runtime.accept(chunk, pending) is None:
                raise RuntimeError("full-body-smoke ActionChunk was rejected")
            deadline = time.monotonic() + phase_seconds
            next_tick = time.monotonic()
            writes = 0
            while time.monotonic() < deadline:
                decision = runtime.tick(
                    session.snapshot(),
                    session.joint_limits,
                    operator_enabled=True,
                    now_ns=time.time_ns(),
                )
                writes += int(decision.wrote)
                if decision.driver.status == "FAULT" or decision.stop is not None:
                    raise RuntimeError(
                        f"full-body-smoke {label} failed: "
                        f"driver={decision.driver.reasons}, "
                        f"stop={None if decision.stop is None else decision.stop.reasons}"
                    )
                next_tick += 1.0 / runtime.driver_frequency_hz
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            runtime.control("pause", generation)
            if writes == 0:
                raise RuntimeError(f"full-body-smoke {label} produced no SDK writes")
            print(
                json.dumps(
                    {
                        "status": "FULL_BODY_PHASE_COMPLETE",
                        "phase": label,
                        "writes": writes,
                        "chassis_velocity": [0.0, 0.0, 0.0],
                    },
                    sort_keys=True,
                )
            )
            return writes

        writes = execute_phase("hold_current", initial_action, 1)
        if delta_rad:
            writes += execute_phase("tiny_motion", motion_action, 2)
            writes += execute_phase("return_initial", initial_action, 3)
        final = session.snapshot()
        print(
            json.dumps(
                {
                    "status": "FULL_BODY_SMOKE_COMPLETE",
                    "motion_field": motion_field,
                    "delta_rad": delta_rad,
                    "writes": writes,
                    "initial_chassis": initial.joints["astribot_chassis"],
                    "final_chassis": final.joints["astribot_chassis"],
                    "initial_joint": initial.joints["astribot_arm_left"][0],
                    "final_joint": final.joints["astribot_arm_left"][0],
                },
                sort_keys=True,
            )
        )
        return 0


def cartesian_teleop(args: argparse.Namespace) -> int:
    profile = _profile(args.profile)
    teleop_profile = profile["teleop"]
    guard = AstribotS1CartesianSafetyGuard.from_profile(args.profile)
    retargeter = QnexoCartesianRetargeter(
        axis_map=teleop_profile["axis_map"],
        translation_scale=teleop_profile["translation_scale"],
        hand_open_value=teleop_profile["hand_open_value"],
        hand_closed_value=teleop_profile["hand_closed_value"],
    )
    source_timeout_ns = int(float(teleop_profile["source_timeout_ms"]) * 1_000_000)
    period = 1.0 / float(profile["sdk"]["frequency_hz"])
    with _session(args, profile) as session:
        home_snapshot = session.snapshot()
        home_action = _current_action(home_snapshot, session.joint_limits)
        driver = AstribotS1Driver(
            session,
            enabled=args.enable_write,
            control_way=profile["sdk"]["control_way"],
            use_wbc=bool(profile["sdk"]["use_wbc"]),
            # Mixed Cartesian-arm/joint-gripper streaming moved both arms even
            # with a measured zero QNexo endpoint delta. Keep this first live
            # profile arm-only until the vendor WBC coupling is isolated.
            include_grippers=False,
        )
        runtime = AstribotS1CartesianRuntime(guard, driver)
        runtime.control("run", 1)
        lifecycle = TeleopControlLifecycle()
        active = False
        resetting = False
        reset_event: str | None = None
        reset_started_at = 0.0
        last_source_ns = 0
        last_status: Any = None
        source = QnexoEndEffectorSource(
            port=args.qnexo_port,
            footswitch_device=args.footswitch_device,
            footswitch_key_code=args.footswitch_key_code,
            save_footswitch_key_code=args.save_footswitch_key_code,
            footswitch_toggle_on_press=bool(
                teleop_profile["footswitch_toggle_on_press"]
            ),
            button_deadman=args.button_deadman,
            urdf_path=args.exo_urdf,
        )
        started_at = time.monotonic()
        stream_seen = False
        print(
            json.dumps({"status": "HOME_CAPTURED", "home": home_action}, sort_keys=True)
        )

        def emit_decision(decision: Any, phase: str) -> None:
            nonlocal last_status
            status_key = (
                phase,
                decision.safety.status,
                decision.safety.reasons,
                decision.driver.status,
                decision.driver.reasons,
            )
            if status_key == last_status:
                return
            print(
                json.dumps(
                    {
                        "phase": phase,
                        "safety": {
                            "status": decision.safety.status,
                            "reasons": decision.safety.reasons,
                        },
                        "driver": asdict(decision.driver),
                    },
                    sort_keys=True,
                )
            )
            last_status = status_key

        try:
            with source:
                while True:
                    if (
                        args.duration > 0
                        and time.monotonic() - started_at >= args.duration
                    ):
                        print(json.dumps({"status": "DURATION_COMPLETE"}))
                        return 0
                    sample = source.poll()
                    now_ns = time.time_ns()
                    if sample is None:
                        if (
                            active
                            and last_source_ns
                            and now_ns - last_source_ns > source_timeout_ns
                        ):
                            retargeter.release()
                            active = False
                            decision = driver.stop_on_fault()
                            print(
                                json.dumps(
                                    {
                                        "status": "QNEXO_STALE",
                                        "driver": asdict(decision),
                                    }
                                )
                            )
                            return 4
                        time.sleep(period)
                        continue
                    last_source_ns = sample.timestamp_ns
                    if not stream_seen:
                        stream_seen = True
                        print(json.dumps({"status": "QNEXO_STREAMING"}))
                    if sample.control_event is not None:
                        transition = lifecycle.handle(sample.control_event)
                        retargeter.release()
                        active = False
                        last_status = None
                        print(
                            json.dumps(
                                {
                                    "status": "BUTTON_EVENT",
                                    "button": sample.control_button,
                                    "event": sample.control_event,
                                    "accepted": transition.accepted,
                                    "lifecycle_state": transition.state,
                                    "reason": transition.reason,
                                },
                                sort_keys=True,
                            )
                        )
                        if not transition.accepted:
                            time.sleep(period)
                            continue
                        if sample.control_event == "stop":
                            resetting = False
                            reset_event = None
                            print(
                                json.dumps(
                                    {
                                        "status": "CONTROL_INPUT_STOPPED",
                                        "lifecycle_state": lifecycle.state,
                                    },
                                    sort_keys=True,
                                )
                            )
                            # GR3 stop ends model/expert input but keeps the owner
                            # process alive so the next prepare/start sequence can
                            # begin without restarting the vendor SDK session.
                            time.sleep(period)
                            continue
                        if sample.control_event in {"finish", "prepare"}:
                            resetting = True
                            active = True
                            reset_event = sample.control_event
                            reset_started_at = time.monotonic()
                            print(
                                json.dumps(
                                    {
                                        "status": "HOME_RESET_STARTED",
                                        "event": reset_event,
                                    },
                                    sort_keys=True,
                                )
                            )
                    if resetting:
                        snapshot = session.snapshot()
                        runtime.submit(
                            home_action,
                            timestamp_ns=sample.timestamp_ns,
                            generation=1,
                        )
                        decision = runtime.tick(
                            snapshot,
                            session.joint_limits,
                            operator_enabled=True,
                            now_ns=time.time_ns(),
                        )
                        emit_decision(decision, "reset")
                        if decision.driver.status == "FAULT":
                            driver.stop_on_fault()
                            return 5
                        if _home_reached(
                            snapshot,
                            home_action,
                            position_tolerance_m=float(
                                teleop_profile["reset_position_tolerance_m"]
                            ),
                            rotation_tolerance_rad=float(
                                teleop_profile["reset_rotation_tolerance_rad"]
                            ),
                            gripper_tolerance_percent=float(
                                teleop_profile["reset_gripper_tolerance_percent"]
                            ),
                        ):
                            resetting = False
                            active = False
                            completed_event = reset_event
                            reset_event = None
                            if completed_event == "finish":
                                lifecycle.finish_complete()
                            retargeter.release()
                            print(
                                json.dumps(
                                    {
                                        "status": "HOME_RESET_COMPLETE",
                                        "event": completed_event,
                                        "lifecycle_state": lifecycle.state,
                                    },
                                    sort_keys=True,
                                )
                            )
                            time.sleep(period)
                            continue
                        elif time.monotonic() - reset_started_at > float(
                            teleop_profile["reset_timeout_seconds"]
                        ):
                            decision = driver.stop_on_fault()
                            print(
                                json.dumps(
                                    {
                                        "status": "RESET_TIMEOUT",
                                        "driver": asdict(decision),
                                    }
                                )
                            )
                            return 6
                        time.sleep(period)
                        continue
                    if not sample.operator_enabled:
                        if active:
                            print(json.dumps({"status": "FOOTSWITCH_RELEASED"}))
                        active = False
                        retargeter.release()
                        time.sleep(period)
                        continue
                    if not lifecycle.motion_requested:
                        if active:
                            print(json.dumps({"status": "CONTROL_PAUSED"}))
                        active = False
                        retargeter.release()
                        time.sleep(period)
                        continue
                    snapshot = session.snapshot()
                    target = retargeter.update(sample.payload, snapshot.arm_poses)
                    active = True
                    if target.calibrated:
                        print(json.dumps({"status": "CALIBRATED"}))
                        time.sleep(period)
                        continue
                    runtime.submit(
                        target.action,
                        timestamp_ns=sample.timestamp_ns,
                        generation=1,
                    )
                    # snapshot() timestamps the state after its SDK reads.  Sample
                    # the evaluation clock afterwards so state is never treated
                    # as coming from the future.
                    now_ns = time.time_ns()
                    decision = runtime.tick(
                        snapshot,
                        session.joint_limits,
                        operator_enabled=True,
                        now_ns=now_ns,
                    )
                    emit_decision(decision, "teleop")
                    if decision.driver.status == "FAULT":
                        driver.stop_on_fault()
                        return 5
                    if lifecycle.consume_step():
                        active = False
                        retargeter.release()
                        print(json.dumps({"status": "STEP_COMPLETE"}))
                    time.sleep(period)
        except KeyboardInterrupt:
            if active:
                print(json.dumps({"status": "OPERATOR_STOP"}))
                driver.stop_on_fault()
        finally:
            if active:
                driver.stop_on_fault()
    return 0


def teleop(args: argparse.Namespace) -> int:
    """GR3-style reference-aligned dual-arm joint teleoperation."""
    profile = _profile(args.profile)
    teleop_profile = profile["teleop"]
    joint_profile = profile["joint_teleop"]
    gripper_profile = profile["gripper_teleop"]
    torso_profile = profile["torso_teleop"]
    guard = AstribotS1JointSafetyGuard.from_profile(args.profile)
    auxiliary_guard = AstribotS1AuxiliarySafetyGuard.from_profile(args.profile)
    gripper_guard = AstribotS1GripperSafetyGuard.from_profile(args.profile)
    retargeter = QnexoJointDeltaRetargeter(joint_scales=joint_profile["joint_scales"])
    gripper_retargeter = QnexoGripperRetargeter()
    source_timeout_ns = int(float(teleop_profile["source_timeout_ms"]) * 1_000_000)
    period = 1.0 / float(profile["sdk"]["frequency_hz"])
    gripper_period = 1.0 / float(gripper_profile["frequency_hz"])
    with _session(args, profile) as session:
        home_snapshot = session.snapshot()
        home_auxiliary_action = current_auxiliary_action(home_snapshot.joints)
        torso_retargeter = build_qnexo_torso_retargeter(
            profile,
            torso_limits=session.joint_limits["astribot_torso"],
            torso_velocity_limits=session.joint_velocity_limits["astribot_torso"],
        )
        configured_home = joint_profile.get("home_joints")
        home_action = (
            current_arm_joint_action(home_snapshot.joints)
            if configured_home is None
            else arm_joint_action_dict(
                [*configured_home["left"], *configured_home["right"]]
            )
        )
        driver = AstribotS1UpperBodyDriver(
            session,
            enabled=args.enable_write,
            control_way=torso_profile["arm_control_way"],
            torso_control_way=torso_profile["torso_control_way"],
            use_wbc=False,
            include_grippers=True,
        )
        generation = 1
        lifecycle = TeleopControlLifecycle()
        active = False
        resetting = False
        reset_event: str | None = None
        reset_started_at = 0.0
        last_source_ns = 0
        last_status: Any = None
        last_gripper_status: Any = None
        last_gripper_safety: Any = None
        next_gripper_send_at = 0.0
        source = QnexoEndEffectorSource(
            port=args.qnexo_port,
            footswitch_device=args.footswitch_device,
            footswitch_key_code=args.footswitch_key_code,
            save_footswitch_key_code=args.save_footswitch_key_code,
            footswitch_toggle_on_press=bool(
                teleop_profile["footswitch_toggle_on_press"]
            ),
            button_deadman=args.button_deadman,
            urdf_path=args.exo_urdf,
        )
        started_at = time.monotonic()
        stream_seen = False
        print(
            json.dumps(
                {
                    "status": (
                        "JOINT_HOME_CAPTURED"
                        if configured_home is None
                        else "JOINT_HOME_CONFIGURED"
                    ),
                    "home": home_action,
                    "home_auxiliary": home_auxiliary_action,
                    "torso_solver": torso_profile["solver"],
                },
                sort_keys=True,
            )
        )

        def evaluate_and_send(
            action: dict[str, float],
            auxiliary_action: dict[str, float],
            gripper_safety: Any,
            snapshot: RobotSnapshot,
            command_timestamp_ns: int,
            phase: str,
        ) -> Any:
            nonlocal last_status
            now_ns = time.time_ns()
            safety = guard.evaluate(
                action,
                snapshot.joints,
                session.joint_limits,
                enabled=True,
                robot_alive=snapshot.robot_alive,
                have_control_rights=snapshot.have_control_rights,
                generation=generation,
                expected_generation=generation,
                state_timestamp_ns=snapshot.timestamp_ns,
                command_timestamp_ns=command_timestamp_ns,
                now_ns=now_ns,
            )
            auxiliary_safety = auxiliary_guard.evaluate(
                auxiliary_action,
                snapshot.joints,
                session.joint_limits,
                enabled=True,
                robot_alive=snapshot.robot_alive,
                have_control_rights=snapshot.have_control_rights,
                generation=generation,
                expected_generation=generation,
                state_timestamp_ns=snapshot.timestamp_ns,
                command_timestamp_ns=command_timestamp_ns,
                now_ns=now_ns,
            )
            if (
                safety.accepted
                and auxiliary_safety.accepted
                and gripper_safety.accepted
            ):
                decision = driver.send(
                    auxiliary_safety.action,
                    safety.action,
                    gripper_safety.action,
                    auxiliary_safety_status=auxiliary_safety.status,
                    arm_safety_status=safety.status,
                    gripper_safety_status=gripper_safety.status,
                    operator_enabled=True,
                    robot_ready=(
                        snapshot.robot_alive
                        and snapshot.driver_alive
                        and snapshot.have_control_rights
                    ),
                    generation=generation,
                    expected_generation=generation,
                )
            else:
                decision = driver._hold("action_not_safety_approved")
            status_key = (
                phase,
                safety.status,
                safety.reasons,
                auxiliary_safety.status,
                auxiliary_safety.reasons,
                gripper_safety.status,
                gripper_safety.reasons,
                decision.status,
                decision.reasons,
            )
            if status_key != last_status:
                print(
                    json.dumps(
                        {
                            "phase": phase,
                            "arm_safety": asdict(safety),
                            "auxiliary_safety": asdict(auxiliary_safety),
                            "gripper_safety": asdict(gripper_safety),
                            "driver": asdict(decision),
                        },
                        sort_keys=True,
                    )
                )
                last_status = status_key
            return decision

        def evaluate_grippers(
            action: dict[str, float],
            snapshot: RobotSnapshot,
            command_timestamp_ns: int,
        ) -> Any:
            nonlocal last_gripper_status
            now_ns = time.time_ns()
            safety = gripper_guard.evaluate(
                action,
                snapshot.joints,
                session.joint_limits,
                enabled=True,
                robot_alive=snapshot.robot_alive,
                have_control_rights=snapshot.have_control_rights,
                generation=generation,
                expected_generation=generation,
                state_timestamp_ns=snapshot.timestamp_ns,
                command_timestamp_ns=command_timestamp_ns,
                now_ns=now_ns,
            )
            status_key = (
                safety.status,
                safety.reasons,
            )
            if status_key != last_gripper_status:
                print(
                    json.dumps(
                        {
                            "phase": "gripper_teleop",
                            "safety": asdict(safety),
                        },
                        sort_keys=True,
                    )
                )
                last_gripper_status = status_key
            return safety

        def current_gripper_hold(snapshot: RobotSnapshot) -> dict[str, float]:
            values = []
            for part in GRIPPER_NAMES:
                lower, upper = session.joint_limits[part]
                values.append(
                    float(np.clip(snapshot.joints[part][0], lower[0], upper[0]))
                )
            return gripper_action_dict(values)

        def release_retargeters() -> None:
            retargeter.release()
            torso_retargeter.release()

        try:
            with source:
                while True:
                    if (
                        args.duration > 0
                        and time.monotonic() - started_at >= args.duration
                    ):
                        print(json.dumps({"status": "DURATION_COMPLETE"}))
                        return 0
                    sample = source.poll()
                    now_ns = time.time_ns()
                    if sample is None:
                        if (
                            active
                            and last_source_ns
                            and now_ns - last_source_ns > source_timeout_ns
                        ):
                            release_retargeters()
                            decision = driver.stop_on_fault()
                            print(
                                json.dumps(
                                    {
                                        "status": "QNEXO_STALE",
                                        "driver": asdict(decision),
                                    }
                                )
                            )
                            return 4
                        time.sleep(period)
                        continue
                    last_source_ns = sample.timestamp_ns
                    if not stream_seen:
                        stream_seen = True
                        print(json.dumps({"status": "QNEXO_STREAMING"}))
                    if sample.control_event is not None:
                        transition = lifecycle.handle(sample.control_event)
                        release_retargeters()
                        active = False
                        last_status = None
                        last_gripper_safety = None
                        next_gripper_send_at = 0.0
                        print(
                            json.dumps(
                                {
                                    "status": "BUTTON_EVENT",
                                    "button": sample.control_button,
                                    "event": sample.control_event,
                                    "accepted": transition.accepted,
                                    "lifecycle_state": transition.state,
                                    "reason": transition.reason,
                                },
                                sort_keys=True,
                            )
                        )
                        if not transition.accepted:
                            time.sleep(period)
                            continue
                        if sample.control_event == "stop":
                            resetting = False
                            reset_event = None
                            print(
                                json.dumps(
                                    {
                                        "status": "CONTROL_INPUT_STOPPED",
                                        "lifecycle_state": lifecycle.state,
                                    },
                                    sort_keys=True,
                                )
                            )
                            # GR3 stop ends model/expert input but keeps the owner
                            # process alive so the next prepare/start sequence can
                            # begin without restarting the vendor SDK session.
                            time.sleep(period)
                            continue
                        if sample.control_event in {"finish", "prepare"}:
                            resetting = True
                            active = True
                            reset_event = sample.control_event
                            reset_started_at = time.monotonic()
                            print(
                                json.dumps(
                                    {
                                        "status": "JOINT_HOME_RESET_STARTED",
                                        "event": reset_event,
                                    },
                                    sort_keys=True,
                                )
                            )
                    if resetting:
                        snapshot = session.snapshot()
                        reset_gripper_safety = evaluate_grippers(
                            current_gripper_hold(snapshot),
                            snapshot,
                            sample.timestamp_ns,
                        )
                        decision = evaluate_and_send(
                            home_action,
                            home_auxiliary_action,
                            reset_gripper_safety,
                            snapshot,
                            sample.timestamp_ns,
                            "joint_reset",
                        )
                        if decision.status == "FAULT":
                            driver.stop_on_fault()
                            return 5
                        arm_errors = [
                            abs(
                                home_action[name]
                                - current_arm_joint_action(snapshot.joints)[name]
                            )
                            for name in home_action
                        ]
                        auxiliary_now = current_auxiliary_action(snapshot.joints)
                        auxiliary_errors = [
                            abs(home_auxiliary_action[name] - auxiliary_now[name])
                            for name in home_auxiliary_action
                        ]
                        reset_tolerance = float(
                            joint_profile["reset_joint_tolerance_rad"]
                        )
                        if (
                            max(arm_errors, default=0.0) <= reset_tolerance
                            and max(auxiliary_errors, default=0.0)
                            <= reset_tolerance
                        ):
                            resetting = False
                            active = False
                            completed_event = reset_event
                            reset_event = None
                            if completed_event == "finish":
                                lifecycle.finish_complete()
                            release_retargeters()
                            print(
                                json.dumps(
                                    {
                                        "status": "JOINT_HOME_RESET_COMPLETE",
                                        "event": completed_event,
                                        "lifecycle_state": lifecycle.state,
                                    },
                                    sort_keys=True,
                                )
                            )
                            time.sleep(period)
                            continue
                        elif time.monotonic() - reset_started_at > float(
                            joint_profile["reset_timeout_seconds"]
                        ):
                            decision = driver.stop_on_fault()
                            print(
                                json.dumps(
                                    {
                                        "status": "JOINT_RESET_TIMEOUT",
                                        "driver": asdict(decision),
                                    }
                                )
                            )
                            return 6
                        time.sleep(period)
                        continue
                    if not sample.operator_enabled:
                        if active:
                            print(json.dumps({"status": "FOOTSWITCH_RELEASED"}))
                        active = False
                        release_retargeters()
                        last_gripper_safety = None
                        next_gripper_send_at = 0.0
                        time.sleep(period)
                        continue
                    if not lifecycle.motion_requested:
                        if active:
                            print(json.dumps({"status": "CONTROL_PAUSED"}))
                        active = False
                        release_retargeters()
                        last_gripper_safety = None
                        next_gripper_send_at = 0.0
                        time.sleep(period)
                        continue
                    snapshot = session.snapshot()
                    target = retargeter.update(sample.payload, snapshot.joints)
                    torso_target = torso_retargeter.update(
                        sample.payload,
                        snapshot.joints,
                        timestamp_ns=sample.timestamp_ns,
                    )
                    active = True
                    if target.calibrated or torso_target.calibrated:
                        print(
                            json.dumps(
                                {
                                    "status": "JOINT_TORSO_CALIBRATED",
                                    "torso_solver": torso_profile["solver"],
                                }
                            )
                        )
                        time.sleep(period)
                        continue
                    monotonic_now = time.monotonic()
                    if (
                        last_gripper_safety is None
                        or monotonic_now >= next_gripper_send_at
                    ):
                        gripper_action = (
                            gripper_retargeter.update(sample.payload)
                            if bool(gripper_profile["enabled"])
                            else current_gripper_hold(snapshot)
                        )
                        last_gripper_safety = evaluate_grippers(
                            gripper_action,
                            snapshot,
                            sample.timestamp_ns,
                        )
                        next_gripper_send_at = monotonic_now + gripper_period
                    decision = evaluate_and_send(
                        target.action,
                        torso_target.action,
                        last_gripper_safety,
                        snapshot,
                        sample.timestamp_ns,
                        "joint_teleop",
                    )
                    if decision.status == "FAULT":
                        driver.stop_on_fault()
                        return 5
                    if lifecycle.consume_step():
                        active = False
                        release_retargeters()
                        last_gripper_safety = None
                        next_gripper_send_at = 0.0
                        print(json.dumps({"status": "STEP_COMPLETE"}))
                    time.sleep(period)
        except KeyboardInterrupt:
            if active:
                print(json.dumps({"status": "OPERATOR_STOP"}))
                driver.stop_on_fault()
        finally:
            if active:
                driver.stop_on_fault()
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="Astribot S1 robot-owned runtime")
    result.add_argument("--profile", default=str(DEFAULT_PROFILE))
    result.add_argument("--qnexo-port", default="/dev/qnbot")
    result.add_argument(
        "--footswitch-device",
        default="/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd",
    )
    result.add_argument("--footswitch-key-code", type=int, default=45)
    result.add_argument(
        "--save-footswitch-key-code",
        type=int,
        default=16,
        help="left pedal key code used to confirm/save a finished episode",
    )
    result.add_argument("--button-deadman", action="store_true")
    result.add_argument("--exo-urdf")
    result.add_argument("--acknowledge-sdk-connect", action="store_true")
    result.add_argument("--acknowledge-workspace-clear", action="store_true")
    result.add_argument("--enable-write", action="store_true")
    result.add_argument("--high-control-rights", action="store_true")
    subparsers = result.add_subparsers(dest="command", required=True)
    subparsers.add_parser("doctor")
    qnexo_parser = subparsers.add_parser("qnexo-observe")
    qnexo_parser.add_argument("--duration", type=float, default=10.0)
    qnexo_parser.add_argument(
        "--footswitch-toggle-on-press",
        action="store_true",
        help="observe DAgger click-toggle takeover instead of hold-to-enable",
    )
    observe_parser = subparsers.add_parser("observe")
    observe_parser.add_argument("--samples", type=int, default=3)
    observe_parser.add_argument("--interval", type=float, default=0.1)
    camera_parser = subparsers.add_parser("camera-observe")
    camera_parser.add_argument("--samples", type=int, default=3)
    camera_parser.add_argument("--interval", type=float, default=0.1)
    camera_parser.add_argument("--timeout", type=float, default=10.0)
    camera_parser.add_argument("--robot-id", default="astribot-s1")
    camera_parser.add_argument("--activate-cameras", action="store_true")
    owner_parser = subparsers.add_parser("owner-shadow")
    owner_parser.add_argument("--samples", type=int, default=5)
    owner_parser.add_argument("--chunk-steps", type=int)
    owner_parser.add_argument("--timeout", type=float, default=10.0)
    owner_parser.add_argument("--robot-id", default="astribot-s1")
    owner_parser.add_argument("--instruction", default="")
    owner_parser.add_argument("--activate-cameras", action="store_true")
    owner_parser.add_argument("--report-root")
    owner_parser.add_argument("--rollout-id")
    subparsers.add_parser("hold-current")
    subparsers.add_parser("hold-current-joints")
    full_body_parser = subparsers.add_parser("full-body-smoke")
    full_body_parser.add_argument("--delta-rad", type=float, default=0.0)
    full_body_parser.add_argument("--phase-seconds", type=float, default=0.3)
    teleop_parser = subparsers.add_parser("teleop")
    teleop_parser.add_argument("--duration", type=float, default=0.0)
    return result


def main() -> None:
    args = parser().parse_args()
    handlers = {
        "doctor": doctor,
        "qnexo-observe": qnexo_observe,
        "observe": observe,
        "camera-observe": camera_observe,
        "owner-shadow": owner_shadow,
        "hold-current": hold_current,
        "hold-current-joints": hold_current_joints,
        "full-body-smoke": full_body_smoke,
        "teleop": teleop,
    }
    raise SystemExit(handlers[args.command](args))


if __name__ == "__main__":
    main()
