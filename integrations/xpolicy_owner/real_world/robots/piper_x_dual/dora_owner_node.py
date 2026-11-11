from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ...teleop import TeleopControlTransition

from .mapping import (
    ACTION_NAMES,
    LEFT_JOINT_NAMES,
    RIGHT_JOINT_NAMES,
    action_dict,
)
from .profile import (
    load_device_instance,
    load_profile,
    validate_device_instance,
)
from .inventory import InventoryError
from .gripper_control import gripper_open_action
from .leader_follower import (
    LeaderFollowerConfig,
    build_leader_follower_action,
    parse_expert_source,
)
from .output import PiperModelLowPassFilter
from .motion_trace import MotionTrace
from .sdk import DualPiperSdkSession
from .safety import (
    DualPiperJointSafetyGuard,
    normalize_model_action_boundaries,
)
from .teleop import PiperShadowController, PiperShadowControlStep
from .transition_poses import (
    TransitionPathGuard,
    TransitionPose,
    TransitionPoseLibrary,
)

DEFAULT_PROFILE = Path(__file__).with_name("profile.yml")
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
MAX_SUPERVISOR_ROLLOUT_STEPS = 10_000


def dispatch_manual_gripper_open(
    snapshot: Any, *, controller: PiperShadowController, sdk_session: Any,
    pause_model: Callable[[Any], None], profile: dict, gripper_scope: str = "both", **guards: Any,
) -> dict[str, float]:
    """Owner-only, one-shot gripper write; invalidate Prepare before dispatch."""
    target = gripper_open_action(
        snapshot, profile=profile, lifecycle_state=controller.lifecycle_state,
        generation=controller.generation, gripper_scope=gripper_scope, **guards,
    )
    _, control = controller.handle_button("stop")
    if control is not None:
        pause_model(control)
    if gripper_scope == "left":
        sdk_session.move_single_gripper_for_live_control(
            "left_arm", target["left_gripper"],
            force_n=float(profile["shadow_teleop"]["gripper_force_n"]),
        )
    else:
        sdk_session.move_dual_grippers_for_live_control(
            target["left_gripper"], target["right_gripper"],
            force_n=float(profile["shadow_teleop"]["gripper_force_n"]),
        )
    return target


def owner_readiness_status(
    *, camera_ready: bool, write_enabled: bool,
    feedback_error: str | None = None, startup_feedback_error: str | None = None,
) -> str:
    """Do not advertise camera readiness as complete robot readiness."""
    if feedback_error:
        return "HOLD_FEEDBACK_UNAVAILABLE"
    if write_enabled and startup_feedback_error:
        return "HOLD_RESTART_REQUIRED"
    suffix = "LIVE_TAKEOVER" if write_enabled else "NO_WRITE"
    return f"READY_{suffix}" if camera_ready else f"WAITING_CAMERAS_{suffix}"


@dataclass(slots=True)
class BoundedModelRollout:
    """One-shot supervisor limit enforced at the sole hardware-owner boundary."""

    pending_max_control_steps: int = 0
    active_max_control_steps: int = 0
    model_steps: int = 0
    rollout_id: int = 0
    active_rollout_id: int = 0
    last: dict[str, Any] = field(default_factory=dict)

    def configure(self, max_control_steps: Any, *, lifecycle_state: str) -> int:
        if str(lifecycle_state) in {"running", "finishing"}:
            raise ValueError("stop the current episode before configuring a rollout")
        try:
            steps = int(max_control_steps)
        except (TypeError, ValueError) as exc:
            raise ValueError("rollout max_control_steps must be an integer") from exc
        if not 1 <= steps <= MAX_SUPERVISOR_ROLLOUT_STEPS:
            raise ValueError(
                "rollout max_control_steps must be in "
                f"[1, {MAX_SUPERVISOR_ROLLOUT_STEPS}]"
            )
        self.rollout_id += 1
        self.pending_max_control_steps = steps
        return steps

    def start(self) -> None:
        self.active_rollout_id = (
            self.rollout_id if self.pending_max_control_steps else 0
        )
        self.active_max_control_steps = self.pending_max_control_steps
        self.pending_max_control_steps = 0
        self.model_steps = 0

    def record_model_step(self) -> bool:
        self.model_steps += 1
        return bool(
            self.active_max_control_steps
            and self.model_steps >= self.active_max_control_steps
        )

    def stop(self, termination_reason: str) -> None:
        if self.active_max_control_steps or self.model_steps:
            self.last = {
                "rollout_id": self.active_rollout_id,
                "max_control_steps": self.active_max_control_steps,
                "model_steps": self.model_steps,
                "termination_reason": str(termination_reason),
                "completed_timestamp_ns": time.time_ns(),
            }
        self.active_max_control_steps = 0
        self.active_rollout_id = 0

    def status(self) -> dict[str, Any]:
        return {
            "rollout_id": self.rollout_id,
            "active_rollout_id": self.active_rollout_id,
            "pending_max_control_steps": self.pending_max_control_steps,
            "active_max_control_steps": self.active_max_control_steps,
            "model_steps": self.model_steps,
            "last": dict(self.last),
        }


def _scalar(value: Any) -> Any:
    item = value[0]
    return item.as_py() if hasattr(item, "as_py") else item


def _truthy(name: str, default: str = "0") -> bool:
    return os.getenv(name, default).strip().lower() in {"1", "true", "yes", "on"}


def build_piper_qnexo_payload(
    raw: Mapping[str, Any],
    parse_joystick_data: Callable[[int, int, int, int], Mapping[str, Any]],
) -> dict[str, Any]:
    """Normalize the shared raw QNexo transport without assigning arm joints."""
    left = parse_joystick_data(*map(int, raw["joystick_left"]))
    right = parse_joystick_data(*map(int, raw["joystick_right"]))
    left_arm = list(map(float, raw["arm_joint_left_rad"][:7]))
    right_arm = list(map(float, raw["arm_joint_right_rad"][:7]))
    if len(left_arm) != 7 or len(right_arm) != 7:
        raise ValueError(
            "QNexo raw arm payload must contain at least 7 joints per side"
        )
    return {
        "left_arm_joints": left_arm,
        "right_arm_joints": right_arm,
        "left_trigger": float(left["trigger"]),
        "right_trigger": float(right["trigger"]),
        "left_joystick_x": float(left["x"]),
        "left_joystick_y": float(left["y"]),
        "right_joystick_x": float(right["x"]),
        "right_joystick_y": float(right["y"]),
    }


def _named_vector_array(pa: Any, action: Mapping[str, Any]) -> Any:
    values = action_dict(action)
    return pa.StructArray.from_arrays(
        [
            pa.array(ACTION_NAMES),
            pa.array([values[name] for name in ACTION_NAMES], type=pa.float64()),
        ],
        names=["name", "value"],
    )


def _named_vector(value: Any) -> dict[str, float]:
    named = {str(item["name"].as_py()): float(item["value"].as_py()) for item in value}
    return action_dict(named)


def feedback_unavailable_boundary_step(
    controller: PiperShadowController,
    event: str,
    *,
    feedback_error: str = "robot feedback unavailable",
) -> PiperShadowControlStep | None:
    """Reject motion explicitly; allow cancellation without a measured pose."""
    if event in {"prepare", "start"}:
        # A rejected click must not advance the lifecycle or leave a pending
        # motion request to execute automatically when feedback recovers.
        return PiperShadowControlStep(
            transition=TeleopControlTransition(
                event=event,
                accepted=False,
                state=controller.lifecycle_state,
                generation=controller.generation,
                reason=f"feedback_unavailable: {feedback_error}",
            ),
        )
    if event not in {"stop", "finish", "discard"}:
        return None
    transition, control = controller.handle_button(event)
    return PiperShadowControlStep(transition=transition, control=control)


def _cartesian_target(
    value: Any,
) -> tuple[tuple[float, ...], tuple[float, ...], float, float]:
    target = _scalar(value)
    if not isinstance(target, Mapping):
        raise ValueError("Piper Cartesian target must be a struct")
    try:
        left = tuple(float(item) for item in target["left_pose"])
        right = tuple(float(item) for item in target["right_pose"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Piper Cartesian target is malformed") from exc
    if len(left) != 6 or len(right) != 6:
        raise ValueError("Piper Cartesian target requires two 6D poses")
    try:
        left_gripper = float(target["left_gripper_m"])
        right_gripper = float(target["right_gripper_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Piper gripper target is malformed") from exc
    if not 0.0 <= left_gripper <= 0.1 or not 0.0 <= right_gripper <= 0.1:
        raise ValueError("Piper gripper targets must be in [0.0, 0.1] m")
    return left, right, left_gripper, right_gripper


def _cartesian_target_array(pa: Any, left_pose: Any, right_pose: Any) -> Any:
    return pa.array(
        [
            {
                "left_pose": list(map(float, left_pose)),
                "right_pose": list(map(float, right_pose)),
            }
        ]
    )


def main() -> None:
    precommit_fault_invalidation = False
    if os.environ.get("PIPER_NATIVE_PRECOMMIT_PROFILE"):
        from XPolicyLab.real_world.precommit_limits import fault_invalidation_enabled
        precommit_fault_invalidation = fault_invalidation_enabled(os.environ["PIPER_NATIVE_PRECOMMIT_PROFILE"])
    try:
        import pyarrow as pa
        from dora import Node
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Piper Dora owner requires dora-rs, pyarrow and the pinned Piper extra"
        ) from exc

    profile_path = Path(os.getenv("PIPER_PROFILE", str(DEFAULT_PROFILE))).resolve()
    profile = load_profile(profile_path)
    configured_device = os.getenv("PIPER_DEVICE_INSTANCE")
    device_path = (
        Path(configured_device).resolve()
        if configured_device
        else REPOSITORY_ROOT / str(profile["device_instance"]["default_path"])
    )
    device = load_device_instance(device_path)
    validate_device_instance(profile, device)
    write_enabled = _truthy("ENABLE_WRITE")
    release_stop_arm = os.getenv("PIPER_RELEASE_SOFTWARE_STOP_ARM", "").strip()
    calibrate_gripper_arm = os.getenv("PIPER_CALIBRATE_CLOSED_GRIPPER", "").strip()
    if calibrate_gripper_arm and (
        not write_enabled or calibrate_gripper_arm not in {"left_arm", "right_arm"}
        or release_stop_arm or os.getenv("PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M", "").strip()
    ):
        raise ValueError("closed-gripper calibration requires a dedicated authorized maintenance run")
    if release_stop_arm and (
        not write_enabled or release_stop_arm not in {"left_arm", "right_arm"}
    ):
        raise ValueError("software-stop release requires --enable-write and one valid arm")
    bounded_review = os.getenv("PIPER_BOUNDED_REVIEW_RELEASE") == "1"
    review_profile = os.getenv("PIPER_REVIEW_PROFILE", "legacy")
    if review_profile not in {"legacy", "policy_prefix"}:
        raise ValueError("unknown runtime review profile")
    if review_profile == "policy_prefix" and not bounded_review:
        raise ValueError("policy_prefix requires explicit bounded review")
    review_gate = receipt_journal = None
    if bounded_review:
        from .review_release import BoundedReviewGate, ReceiptJournal
        review_gate = BoundedReviewGate(
            allow_trajectory=os.getenv("PIPER_GPT_TRAJECTORY_RELEASE") == "1",
            profile=review_profile,
        )
        receipt_journal = ReceiptJournal(os.environ["PIPER_REVIEW_RECEIPT_JOURNAL"], volatile_root=os.getenv("RAX_VOLATILE_CONTROL_ROOT"))
    from .motion_lease import MotionLease
    rax_lease = None
    if os.getenv("RAX_MOTION_LEASE"):
        if not bounded_review or os.getenv("PIPER_GPT_TRAJECTORY_RELEASE") != "1":
            raise ValueError("RAX motion requires formal bounded trajectory review")
        rax_lease = MotionLease(os.environ["RAX_MOTION_LEASE"], os.environ["RAX_ARMING_FILE"])
    cached_diagnostic_path = None
    cached_diagnostic_output = os.getenv("PIPER_READONLY_STATUS_OUTPUT", "").strip()
    if cached_diagnostic_output:
        from .cached_diagnostics import diagnostic_output_path

        cached_diagnostic_path = diagnostic_output_path(
            cached_diagnostic_output, write_enabled=write_enabled
        )
    collection_mode = os.getenv("RECAP_COLLECTION_MODE", "MIXED").strip().upper()
    if collection_mode not in {"OFF", "DATA_COLLECTION", "MIXED"}:
        raise ValueError(
            "RECAP_COLLECTION_MODE must be OFF, DATA_COLLECTION, or MIXED"
        )
    expert_only = collection_mode == "DATA_COLLECTION"
    expert_source = parse_expert_source(
        os.getenv("PIPER_DAGGER_EXPERT_SOURCE", "qnexo")
    )
    lf_defaults = dict(profile.get("leader_follower", {}))
    for env_name, config_name in (
        ("PIPER_LF_LEADER_ROLE", "leader_role"),
        ("PIPER_LF_FOLLOWER_ROLE", "follower_role"),
        ("PIPER_LF_FEEDBACK_ROLE", "feedback_role"),
    ):
        value = os.getenv(env_name, "").strip()
        if value:
            lf_defaults[config_name] = value
    if os.getenv("PIPER_LF_NATIVE_LINKAGE", "").strip():
        lf_defaults["native_linkage"] = _truthy("PIPER_LF_NATIVE_LINKAGE")
    leader_follower = LeaderFollowerConfig.from_mapping(lf_defaults)
    if leader_follower.native_linkage:
        raise RuntimeError(
            "native leader/follower linkage is not enabled by the dual-action "
            "owner yet; set native_linkage=false and use the SDK feedback bridge"
        )
    initialize_left_gripper_width = os.getenv(
        "PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M", ""
    ).strip()
    if initialize_left_gripper_width and not write_enabled:
        raise RuntimeError(
            "PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M requires ENABLE_WRITE=1"
        )
    live = profile["live_takeover"]
    prepare = profile["prepare"]
    transition_config = profile["transition_pose"]
    leader_follower_max_age_ms = float(
        os.getenv("PIPER_LF_MAX_AGE_MS", str(live["max_state_age_ms"]))
    )
    if leader_follower_max_age_ms <= 0.0:
        raise ValueError("PIPER_LF_MAX_AGE_MS must be positive")
    joint_limits = {
        name: tuple(profile["body"]["joint_limits_rad"][name])
        for name in (*LEFT_JOINT_NAMES, *RIGHT_JOINT_NAMES)
    }
    gripper_limits = tuple(
        profile["body"]["grippers"]["left_gripper"]["limits"]
    )
    transition_library_path = os.getenv(
        "PIPER_TRANSITION_POSE_LIBRARY_PATH", ""
    ).strip()
    transition_library = (
        TransitionPoseLibrary(
            transition_library_path,
            expected_sha256=os.getenv(
                "PIPER_TRANSITION_POSE_LIBRARY_SHA256", ""
            ),
            joint_limits_rad=joint_limits,
            joint_names=(*LEFT_JOINT_NAMES, *RIGHT_JOINT_NAMES),
        )
        if transition_library_path
        else None
    )
    transition_path_guard = (
        TransitionPathGuard(profile, profile_path)
        if transition_library is not None
        else None
    )
    model_boundary = live["model_boundary_normalization"]
    safety_guard = DualPiperJointSafetyGuard(
        max_state_age_ms=float(live["max_state_age_ms"]),
        max_command_age_ms=float(live["max_command_age_ms"]),
        measured_state_limit_tolerance_rad=float(
            live["measured_state_limit_tolerance_rad"]
        ),
        joint_limits_rad=joint_limits,
        gripper_limits_m=gripper_limits,
        measured_gripper_limit_tolerance_m=float(
            profile["read_only_probe"]["gripper_feedback_tolerance_m"]
        ),
    )
    model_filter = PiperModelLowPassFilter(
        enabled=_truthy("PIPER_MODEL_FILTER_ENABLED", "1"),
        alpha=float(os.getenv("PIPER_MODEL_FILTER_ALPHA", "0.6")),
    )
    motion_trace = MotionTrace.from_env()
    node = Node(os.getenv("DORA_NODE_ID", "piper_x_dual_owner"))
    controller = PiperShadowController(
        prepare_requires_confirmation=write_enabled,
        expert_only=expert_only,
    )
    latest_expert_action: dict[str, float] | None = None
    latest_source_ns = 0
    last_lf_feedback_ns = 0
    takeover = expert_only
    episode_index = -1
    recording = False
    last_status = ""
    next_status_at = 0.0
    observations = 0
    expert_candidates = 0
    model_candidates = 0
    model_boundary_normalization_count = 0
    model_filter_generation: int | None = None
    hardware_write_count = 0
    next_control_tick_id = 0
    last_dispatch_skew_ms = 0.0
    last_owner_dispatch = "NOT_STARTED"
    last_selected_source = "HOLD"
    last_label_valid = False
    trajectory_samples: deque[dict[str, Any]] = deque(maxlen=6000)
    trajectory_started_ns = 0
    trajectory_capture_index = 0
    reported_feedback_holds: set[tuple[int, str]] = set()
    last_action_hold_signature = ""
    prepare_started_at: float | None = None
    gripper_control_status: dict[str, Any] = {}
    transition_started_at: float | None = None
    active_transition_pose: TransitionPose | None = None
    transition_waypoints: tuple[tuple[float, ...], ...] = ()
    transition_waypoint_index = 0
    transition_status: dict[str, Any] = {
        "phase": "disabled" if transition_library is None else "ready"
    }
    camera_ready = False
    camera_status: dict[str, Any] = {"status": "WAITING_CAMERAS"}
    bounded_rollout = BoundedModelRollout()
    sdk_session = DualPiperSdkSession(
        profile,
        device,
        supervised_write_authorized=write_enabled,
    )

    def reset_model_filter(reason: str) -> None:
        nonlocal model_filter_generation
        if model_filter.reset():
            print(
                "PIPER_X_DUAL_MODEL_FILTER_RESET "
                + json.dumps(
                    {
                        "generation": controller.generation,
                        "reason": reason,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
        model_filter_generation = None

    def report_feedback_hold(exc: InventoryError, *, context: str) -> None:
        """Print each feedback failure once per lifecycle generation."""
        reset_model_filter(f"feedback_hold:{context}")
        signature = (controller.generation, str(exc))
        if signature in reported_feedback_holds:
            return
        reported_feedback_holds.add(signature)
        print(
            "PIPER_X_DUAL_FEEDBACK_HOLD "
            + json.dumps(
                {
                    "context": context,
                    "generation": controller.generation,
                    "reason": str(exc),
                    "takeover": takeover,
                },
                sort_keys=True,
            ),
            flush=True,
        )

    def write_trajectory_capture(reason: str) -> Path | None:
        nonlocal trajectory_started_ns, trajectory_capture_index
        if not trajectory_samples:
            return None
        samples = list(trajectory_samples)
        output_dir = Path(
            os.getenv(
                "PIPER_TRAJECTORY_DIR",
                str(REPOSITORY_ROOT / "out/piper_x_dual/trajectory_diagnostics"),
            )
        ).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        stem = f"trajectory-{stamp}-{trajectory_capture_index:03d}"
        trajectory_capture_index += 1
        jsonl_path = output_dir / f"{stem}.jsonl"
        summary_path = output_dir / f"{stem}.summary.json"
        with jsonl_path.open("w", encoding="utf-8") as stream:
            for sample in samples:
                stream.write(json.dumps(sample, sort_keys=True) + "\n")

        joint_names = [*LEFT_JOINT_NAMES, *RIGHT_JOINT_NAMES]
        duration_s = max(
            0.0,
            (samples[-1]["timestamp_ns"] - samples[0]["timestamp_ns"]) / 1e9,
        )
        per_joint = {}
        for index, name in enumerate(joint_names):
            errors = [
                sample["mink_target_rad"][index]
                - sample["measured_joint_rad"][index]
                for sample in samples
            ]
            target_steps = [
                samples[item]["mink_target_rad"][index]
                - samples[item - 1]["mink_target_rad"][index]
                for item in range(1, len(samples))
            ]
            measured_steps = [
                samples[item]["measured_joint_rad"][index]
                - samples[item - 1]["measured_joint_rad"][index]
                for item in range(1, len(samples))
            ]
            per_joint[name] = {
                "rmse_rad": (sum(value * value for value in errors) / len(errors))
                ** 0.5,
                "max_abs_error_rad": max(abs(value) for value in errors),
                "max_abs_target_step_rad": max(
                    (abs(value) for value in target_steps), default=0.0
                ),
                "max_abs_measured_step_rad": max(
                    (abs(value) for value in measured_steps), default=0.0
                ),
            }
        summary = {
            "reason": reason,
            "samples": len(samples),
            "duration_s": duration_s,
            "effective_hz": (
                (len(samples) - 1) / duration_s if duration_s > 0.0 else 0.0
            ),
            "joint_names": joint_names,
            "per_joint": per_joint,
            "jsonl_path": str(jsonl_path),
        }
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(
            "PIPER_X_DUAL_TRAJECTORY "
            + json.dumps(
                {
                    "jsonl": str(jsonl_path),
                    "summary": str(summary_path),
                    "samples": len(samples),
                    "duration_s": duration_s,
                    "reason": reason,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        trajectory_samples.clear()
        trajectory_started_ns = 0
        return summary_path

    def metadata(source_ns: int = 0, **detail: Any) -> dict[str, Any]:
        now_ns = time.time_ns()
        return {
            "timestamp_ns": now_ns,
            "source_timestamp_ns": int(source_ns or now_ns),
            "target_execution_timestamp_ns": now_ns,
            "robot_type": "piper_x_dual",
            "robot_id": device["robot_id"],
            "profile_id": profile["profile_id"],
            "schema_version": "piper_x_dual_v1",
            "hardware_write_enabled": write_enabled,
            "hardware_dispatched": bool(detail.get("hardware_dispatched", False)),
            **detail,
        }

    def send_scalar(name: str, value: Any, source_ns: int = 0, **detail: Any) -> None:
        node.send_output(name, pa.array([value]), metadata(source_ns, **detail))

    def send_json(name: str, value: Mapping[str, Any], source_ns: int = 0) -> None:
        if name == "status":
            from .component_health import component_health
            value = {**value, "hardware_write_enabled": write_enabled, "component_health": component_health(
                sdk_session, write_enabled=write_enabled,
                gripper_request=gripper_control_status,
            ), "camera_details": camera_status.get("cameras", {})}
        send_scalar(
            name, json.dumps(dict(value), sort_keys=True, default=str), source_ns
        )

    def send_action(
        name: str,
        action: Mapping[str, Any],
        source_ns: int,
        **detail: Any,
    ) -> None:
        node.send_output(
            name,
            _named_vector_array(pa, action),
            metadata(source_ns, **detail),
        )

    def send_cartesian_observation(source_ns: int) -> None:
        left = sdk_session.read_cartesian_feedback("left_arm")
        right = sdk_session.read_cartesian_feedback("right_arm")
        node.send_output(
            "observation_cartesian_robot",
            _cartesian_target_array(pa, left.flange_pose, right.flange_pose),
            metadata(
                source_ns,
                left_flange_timestamp_ns=int(left.flange_timestamp_s * 1e9),
                right_flange_timestamp_ns=int(right.flange_timestamp_s * 1e9),
                left_flange_hz=left.flange_hz,
                right_flange_hz=right.flange_hz,
            ),
        )

    def invalidate_precommit(reason, source_ns, snapshot=None):
        # Opt-in trial only. Existing owner lifecycle advances generation and
        # sends model_control, clearing the pending inference and prefix history.
        # No retries, no pose replacement, no new writer or automatic restart.
        if not precommit_fault_invalidation or controller.lifecycle_state != "running":
            return
        print("PIPER_NATIVE_PRECOMMIT_INVALIDATED " + json.dumps({
            "generation": controller.generation, "reason": reason,
            "sdk_submission_is_not_pose_arrival": True}), flush=True)
        step = (controller.pause_model_rollout() if snapshot is not None
                else feedback_unavailable_boundary_step(controller, "stop"))
        publish_step(step, source_ns, snapshot=snapshot,
                     termination_reason="native_precommit_fault: " + reason)

    def dispatch_selected_action(
        action: Mapping[str, Any],
        snapshot: Any,
        source_ns: int,
        command_timestamp_ns: int,
        *,
        mode: str,
        generation: int,
        policy_metadata: Mapping[str, Any] | None = None,
        trace_sample: dict[str, Any] | None = None,
    ) -> None:
        nonlocal hardware_write_count, last_dispatch_skew_ms
        nonlocal last_owner_dispatch
        nonlocal trajectory_started_ns
        nonlocal model_boundary_normalization_count
        nonlocal model_filter_generation
        nonlocal last_action_hold_signature
        nonlocal next_control_tick_id
        nonlocal last_selected_source, last_label_valid
        if (
            not write_enabled
            or controller.lifecycle_state != "running"
            or controller.buttons.boundary_active
            or (mode == "EXPERT" and not takeover)
            or (mode == "MODEL" and takeover)
        ):
            if trace_sample is not None:
                trace_sample.update(owner_result="HOLD_DISPATCH_GATE", reasons=["owner_dispatch_gate_closed"],
                    write_enabled=write_enabled, lifecycle_state=controller.lifecycle_state,
                    boundary_active=controller.buttons.boundary_active, takeover=takeover)
            if mode == "MODEL":
                invalidate_precommit("owner_dispatch_gate_closed", source_ns, snapshot)
            return
        if startup_feedback_error is not None:
            # Startup never enabled the SDK after a missing feedback sample.
            # Keep every later action in HOLD until the formal launcher is
            # restarted with both arms healthy; otherwise a recovered model
            # event could reach a session that was deliberately left disabled.
            last_owner_dispatch = "HOLD_FEEDBACK_UNAVAILABLE"
            if trace_sample is not None:
                trace_sample.update(owner_result=last_owner_dispatch, reasons=[startup_feedback_error])
            if mode == "MODEL":
                invalidate_precommit(startup_feedback_error, source_ns)
            return
        now_ns = time.time_ns()
        measured_action = action_dict(snapshot.state_14d)
        state_timestamp_ns = int(
            min(
                snapshot.left.arm_timestamp_s,
                snapshot.left.gripper_timestamp_s,
                snapshot.right.arm_timestamp_s,
                snapshot.right.gripper_timestamp_s,
            )
            * 1_000_000_000
        )
        candidate_action = action_dict(action)
        if trace_sample is not None:
            trace_sample.update(measured_action_14d=[measured_action[name] for name in ACTION_NAMES],
                state_timestamp_ns=state_timestamp_ns, state_age_ms=snapshot.maximum_age_ms,
                dispatch_generation=generation, command_timestamp_ns=command_timestamp_ns)
        boundary_adjustments = ()
        if mode == "MODEL" and bool(model_boundary["enabled"]):
            candidate_action, boundary_adjustments = normalize_model_action_boundaries(
                candidate_action,
                joint_limits_rad=joint_limits,
                gripper_limits_m=gripper_limits,
                joint_tolerance_rad=float(model_boundary["joint_tolerance_rad"]),
            )
        if trace_sample is not None:
            trace_sample.update(boundary_action_14d=[candidate_action[name] for name in ACTION_NAMES],
                boundary_adjustments=[adjustment.as_dict() for adjustment in boundary_adjustments])
        # First reject an invalid model target before smoothing. Otherwise a
        # low-pass stage could make a grossly invalid proposal appear safe.
        pre_filter_safety = safety_guard.evaluate(
            candidate_action,
            measured_action,
            enabled=True,
            write_authorized=write_enabled,
            both_arms_ready=True,
            generation=generation,
            expected_generation=controller.generation,
            state_timestamp_ns=state_timestamp_ns,
            command_timestamp_ns=command_timestamp_ns,
            now_ns=now_ns,
        )
        filter_applied = False
        filter_initialized_from_measured = False
        filter_max_joint_adjustment_rad = 0.0
        safety = pre_filter_safety
        if pre_filter_safety.accepted and pre_filter_safety.action is not None:
            if mode == "MODEL":
                if model_filter_generation != generation:
                    reset_model_filter("model_generation_changed")
                    model_filter_generation = generation
                filter_result = model_filter.process(
                    pre_filter_safety.action,
                    measured_action=measured_action,
                )
                filter_applied = filter_result.applied
                filter_initialized_from_measured = (
                    filter_result.initialized_from_measured
                )
                filter_max_joint_adjustment_rad = (
                    filter_result.max_joint_adjustment_rad
                )
                candidate_action = filter_result.action
                # The actual post-filter command remains subject to the same
                # final all-or-nothing safety boundary immediately before SDK
                # dispatch.
                safety = safety_guard.evaluate(
                    candidate_action,
                    measured_action,
                    enabled=True,
                    write_authorized=write_enabled,
                    both_arms_ready=True,
                    generation=generation,
                    expected_generation=controller.generation,
                    state_timestamp_ns=state_timestamp_ns,
                    command_timestamp_ns=command_timestamp_ns,
                    now_ns=now_ns,
                )
            else:
                reset_model_filter("expert_dispatch")
        if trace_sample is not None:
            trace_sample.update(filter_applied=filter_applied, filter_alpha=model_filter.alpha,
                pre_filter_safety_reasons=list(pre_filter_safety.reasons),
                reasons=list(safety.reasons), owner_result="PASS_SAFETY" if safety.accepted else "HOLD_SAFETY")
        if not safety.accepted or safety.action is None:
            if filter_applied:
                reset_model_filter("post_filter_safety_hold")
            invalid_targets: dict[str, float] = {}
            for name in (*LEFT_JOINT_NAMES, *RIGHT_JOINT_NAMES):
                lower, upper = joint_limits[name]
                value = candidate_action[name]
                if not lower <= value <= upper:
                    invalid_targets[name] = value
            for name in ("left_gripper", "right_gripper"):
                value = candidate_action[name]
                if not gripper_limits[0] <= value <= gripper_limits[1]:
                    invalid_targets[name] = value
            hold_payload = {
                "mode": mode,
                "generation": generation,
                "reasons": safety.reasons,
                "invalid_targets": invalid_targets,
            }
            if trace_sample is not None:
                trace_sample['invalid_target_names'] = list(invalid_targets)
            hold_signature = json.dumps(
                {
                    "mode": mode,
                    "generation": generation,
                    "reasons": safety.reasons,
                },
                sort_keys=True,
            )
            if hold_signature != last_action_hold_signature:
                print(
                    "PIPER_X_DUAL_ACTION_HOLD "
                    + json.dumps(hold_payload, sort_keys=True),
                    flush=True,
                )
                last_action_hold_signature = hold_signature
            if mode == "MODEL" and os.environ.get("PIPER_NATIVE_TRAJECTORY_PROFILE"):
                # A rejected processed target invalidates the commitment. Use
                # the existing owner lifecycle, never a second SDK writer or
                # an automatic stop/restart loop. Operator must resume.
                last_owner_dispatch = "HOLD_NATIVE_TRAJECTORY_SAFETY"
                publish_step(controller.pause_model_rollout(), source_ns,
                             snapshot=snapshot,
                             termination_reason="native_trajectory_safety_hold")
            if mode == "MODEL":
                invalidate_precommit("safety_hold: " + str(safety.reasons), source_ns, snapshot)
            return
        dispatched_action = safety.action
        if mode == "MODEL" and precommit_fault_invalidation:
            from XPolicyLab.real_world.precommit_limits import check_owner_command
            check_owner_command(action_dict(action), dispatched_action, ACTION_NAMES,
                                gripper_limits, policy_metadata or {})
            if not bool(live["gripper_write_enabled"]):
                raise ValueError("precommit requires declared gripper command dispatch")
        accepted_release = None
        if bounded_review and mode == "MODEL":
            try:
                accepted_release = review_gate.reserve(
                    json.loads((policy_metadata or {}).get("review_release", "null")),
                    generation=generation, measured=[measured_action[name] for name in ACTION_NAMES],
                    requested=[action_dict(action)[name] for name in ACTION_NAMES], now_ns=time.time_ns(),
                    step_index=int((policy_metadata or {}).get("model_chunk_step_index", -1)))
                if rax_lease is not None:
                    rax_lease.authorize(accepted_release)
            except (ValueError, TypeError, KeyError) as exc:
                reset_model_filter("review_release_rejected")
                last_owner_dispatch = "HOLD_REVIEW_RELEASE"
                receipt_journal.write({"event": "release_rejected", "reason": str(exc),
                    "generation": generation, "hardware_dispatch_confirmed": False})
                print("PIPER_REVIEW_RELEASE_HOLD " + str(exc), flush=True)
                if trace_sample is not None:
                    trace_sample.update(owner_result=last_owner_dispatch, reasons=[str(exc)])
                return
            journal_release = accepted_release
            if accepted_release.get("schema") == "junqi.piper.trajectory_release.v1" and int((policy_metadata or {}).get("model_chunk_step_index", 0)) > 0:
                journal_release = {k: v for k, v in accepted_release.items() if k != "targets14"}
            receipt_journal.write({"event": "dispatch_attempt", "release": journal_release,
                "requested14": [action_dict(action)[name] for name in ACTION_NAMES],
                "applied14": [dispatched_action[name] for name in ACTION_NAMES],
                "measured_before14": [measured_action[name] for name in ACTION_NAMES],
                "hardware_dispatch_confirmed": False})
        if boundary_adjustments:
            model_boundary_normalization_count += 1
            last_owner_dispatch = (
                "PASS_BOUNDARY_NORMALIZED_FILTERED_MODEL_ACTION"
                if filter_applied
                else "PASS_BOUNDARY_NORMALIZED_MODEL_ACTION"
            )
            if (
                model_boundary_normalization_count == 1
                or model_boundary_normalization_count % 100 == 0
            ):
                print(
                    "PIPER_X_DUAL_MODEL_BOUNDARY_NORMALIZED "
                    + json.dumps(
                        {
                            "count": model_boundary_normalization_count,
                            "generation": generation,
                            "adjustments": [
                                adjustment.as_dict()
                                for adjustment in boundary_adjustments
                            ],
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
        elif filter_applied:
            last_owner_dispatch = "PASS_FILTERED_MODEL_ACTION"
        else:
            last_owner_dispatch = f"PASS_EXACT_{mode}_ACTION"
        joint_names = (*LEFT_JOINT_NAMES, *RIGHT_JOINT_NAMES)
        measured_joints = (
            *snapshot.left.joint_position_rad,
            *snapshot.right.joint_position_rad,
        )
        dispatched_joints = tuple(
            dispatched_action[name] for name in joint_names
        )
        sample_timestamp_ns = time.time_ns()
        if trajectory_started_ns == 0:
            trajectory_started_ns = sample_timestamp_ns
        trajectory_samples.append(
            {
                "timestamp_ns": sample_timestamp_ns,
                "elapsed_s": (
                    sample_timestamp_ns - trajectory_started_ns
                )
                / 1e9,
                "source_timestamp_ns": source_ns,
                "mink_target_rad": list(dispatched_joints),
                "dispatched_target_rad": list(dispatched_joints),
                "measured_joint_rad": list(measured_joints),
                "error_rad": [
                    target - measured
                    for target, measured in zip(
                        dispatched_joints, measured_joints, strict=True
                    )
                ],
                "state_age_ms": snapshot.maximum_age_ms,
                "owner_dispatch": last_owner_dispatch,
                "control_mode": mode,
                "boundary_adjustments": [
                    adjustment.as_dict() for adjustment in boundary_adjustments
                ],
                "model_filter": {
                    "enabled": model_filter.enabled,
                    "alpha": model_filter.alpha,
                    "applied": filter_applied,
                    "initialized_from_measured": filter_initialized_from_measured,
                    "max_joint_adjustment_rad": filter_max_joint_adjustment_rad,
                    "grippers": "bypass",
                },
            }
        )
        leader_follower_expert = (
            mode == "EXPERT" and expert_source == "leader_follower"
        )
        if trace_sample is not None:
            trace_sample.update(sdk_target_action_14d=[dispatched_action[name] for name in ACTION_NAMES],
                hardware_write_attempted=True, dispatch_started_time_ns=time.time_ns(),
                dispatch_started_monotonic_ns=time.monotonic_ns(), owner_result="SDK_WRITE_PENDING")
        if leader_follower_expert:
            follower_names = (
                LEFT_JOINT_NAMES
                if leader_follower.follower_role == "left_arm"
                else RIGHT_JOINT_NAMES
            )
            dispatch = sdk_session.move_single_arm_for_live_control(
                leader_follower.follower_role,
                [dispatched_action[name] for name in follower_names],
                speed_percent=int(live["speed_percent"]),
            )
            if bool(live["gripper_write_enabled"]):
                sdk_session.move_single_gripper_for_live_control(
                    leader_follower.follower_role,
                    dispatched_action[
                        f"{leader_follower.follower_role.removesuffix('_arm')}_gripper"
                    ],
                    force_n=float(profile["shadow_teleop"]["gripper_force_n"]),
                )
        else:
            dispatch = sdk_session.move_dual_for_live_control(
                [dispatched_action[name] for name in LEFT_JOINT_NAMES],
                [dispatched_action[name] for name in RIGHT_JOINT_NAMES],
                speed_percent=int(live["speed_percent"]),
            )
        if os.getenv("RAX_LEFT_GRIPPER_DISPATCH") == "1" and mode == "MODEL":
            if rax_lease is None or rax_lease.active is None:
                raise ValueError("left jaw requires the active formal motion lease")
            sdk_session.move_single_gripper_for_live_control("left_arm",
                dispatched_action["left_gripper"], force_n=float(profile["shadow_teleop"]["gripper_force_n"]))
        hardware_write_count += 1
        last_dispatch_skew_ms = dispatch.skew_ms
        if trace_sample is not None:
            trace_sample.update(arm_write_success=True, arm_dispatch_completed_time_ns=time.time_ns(),
                dispatch_skew_ms=dispatch.skew_ms,
                dispatched_action_14d=[dispatched_action[name] if name in joint_names else None
                                       for name in ACTION_NAMES])
        if bool(live["gripper_write_enabled"]) and not leader_follower_expert:
            if trace_sample is not None:
                trace_sample["gripper_write_attempted"] = True
            sdk_session.move_dual_grippers_for_live_control(
                dispatched_action["left_gripper"],
                dispatched_action["right_gripper"],
                force_n=float(profile["shadow_teleop"]["gripper_force_n"]),
            )
            if trace_sample is not None:
                trace_sample.update(gripper_write_success=True,
                    dispatched_action_14d=[dispatched_action[name] for name in ACTION_NAMES])
        control_tick_id = next_control_tick_id
        next_control_tick_id += 1
        if trace_sample is not None:
            trace_sample.update(hardware_write_success=True, owner_result=last_owner_dispatch,
                control_tick_id=control_tick_id, hardware_write_count=hardware_write_count,
                dispatch_completed_time_ns=time.time_ns(),
                dispatch_duration_ns=time.monotonic_ns() - trace_sample["dispatch_started_monotonic_ns"])
        release_receipt_metadata = {}
        if mode == "MODEL" and precommit_fault_invalidation:
            release_receipt_metadata = {"native_precommit_version": 1,
                "model_chunk_id": int((policy_metadata or {}).get("model_chunk_id", -1)),
                "model_chunk_step_index": int((policy_metadata or {}).get("model_chunk_step_index", -1)),
                "pose_arrival_confirmed": False}
        if accepted_release is not None:
            release_receipt_metadata = {"review_release_id": accepted_release["release_id"],
                "teacher_request_id": accepted_release["teacher_request_id"],
                "review_observation_timestamp_ns": accepted_release["review_observation_timestamp_ns"],
                "execution_observation_timestamp_ns": accepted_release["execution_observation_timestamp_ns"],
                "policy_request_id": str((policy_metadata or {}).get("policy_request_id", "")),
                "model_chunk_id": int((policy_metadata or {}).get("model_chunk_id", -1)),
                "model_chunk_step_index": int((policy_metadata or {}).get("model_chunk_step_index", -1))}
            receipt_journal.write({"event": "sdk_dispatch_completed", **release_receipt_metadata,
                "generation": generation, "control_tick_id": control_tick_id,
                "applied14": [dispatched_action[name] for name in ACTION_NAMES],
                "hardware_dispatch_confirmed": True, "pose_arrival_confirmed": False,
                "object_outcome_confirmed": False})
        last_selected_source = mode
        last_label_valid = mode == "EXPERT"
        if hardware_write_count == 1 or hardware_write_count % 100 == 0:
            print(
                "PIPER_X_DUAL_DISPATCH "
                + json.dumps(
                    {
                        "hardware_write_count": hardware_write_count,
                        "owner_dispatch": last_owner_dispatch,
                        "dispatch_skew_ms": dispatch.skew_ms,
                        "max_raw_joint_error_rad": max(
                            abs(target - measured)
                            for target, measured in zip(
                                dispatched_joints, measured_joints, strict=True
                            )
                        ),
                        "left_controller_mode": snapshot.left.controller_mode,
                        "right_controller_mode": snapshot.right.controller_mode,
                        "control_mode": mode,
                        "model_filter_applied": filter_applied,
                        "model_filter_alpha": model_filter.alpha,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
        if mode == "EXPERT":
            expert_step = controller.update(
                payload=None,
                expert_action=dispatched_action,
                button_event=None,
                takeover_active=takeover,
                snapshot=snapshot,
            )
            publish_decision(expert_step.expert, source_ns)
        send_action(
            "action_applied_robot",
            dispatched_action,
            source_ns,
            hardware_dispatched=True,
            generation=generation,
            control_mode=mode,
            owner_dispatch=last_owner_dispatch,
            dispatch_skew_ms=dispatch.skew_ms,
            action_source=(
                "host_mink_joint_target" if mode == "EXPERT" else "policy_action_chunk"
            ),
            boundary_normalized=bool(boundary_adjustments),
            boundary_adjustments=json.dumps(
                [adjustment.as_dict() for adjustment in boundary_adjustments],
                sort_keys=True,
            ),
            model_filter_enabled=model_filter.enabled,
            model_filter_applied=filter_applied,
            model_filter_alpha=model_filter.alpha,
            model_filter_initialized_from_measured=(
                filter_initialized_from_measured
            ),
            model_filter_max_joint_adjustment_rad=(
                filter_max_joint_adjustment_rad
            ),
            model_filter_grippers="bypass",
            control_tick_id=control_tick_id,
            **release_receipt_metadata,
        )
        owner_common = {
            "control_tick_id": control_tick_id,
            "generation": generation,
            "control_mode": mode,
            "selected_action_source": mode,
            "label_valid": mode == "EXPERT",
            "train_mask": mode == "EXPERT",
            "hardware_dispatched": True,
        }
        if mode == "MODEL":
            send_action("action_model_robot", action_dict(action), source_ns, **owner_common)
        else:
            send_action(
                "action_expert_safe_robot",
                dispatched_action,
                source_ns,
                **owner_common,
            )
        send_scalar("control_mode", mode, source_ns, **owner_common)
        send_scalar("selected_action_source", mode, source_ns, **owner_common)
        send_scalar("label_valid", mode == "EXPERT", source_ns, **owner_common)

    def dispatch_expert_target(
        snapshot: Any, source_ns: int, command_timestamp_ns: int
    ) -> None:
        if latest_expert_action is None:
            return
        dispatch_selected_action(
            latest_expert_action,
            snapshot,
            source_ns,
            command_timestamp_ns,
            mode="EXPERT",
            generation=controller.generation,
        )

    def refresh_leader_follower_expert(snapshot: Any, fallback_source_ns: int) -> None:
        """Turn one fresh SDK leader sample into the normal expert stream."""

        nonlocal latest_expert_action, latest_source_ns, last_lf_feedback_ns
        if expert_source != "leader_follower" or not takeover:
            return
        feedback = sdk_session.read_leader_joint_feedback(
            leader_follower.feedback_role
        )
        if feedback is None:
            report_feedback_hold(
                InventoryError(
                    "leader/follower joint feedback is unavailable on "
                    f"{leader_follower.feedback_role}"
                ),
                context="leader_follower",
            )
            return
        gripper = sdk_session.read_leader_gripper_width(leader_follower.leader_role)
        if gripper is None:
            report_feedback_hold(
                InventoryError(
                    "leader/follower gripper feedback is unavailable on "
                    f"{leader_follower.leader_role}"
                ),
                context="leader_follower",
            )
            return
        now_s = time.time()
        leader_age_ms = (now_s - feedback.timestamp_s) * 1000.0
        gripper_age_ms = (now_s - gripper[1]) * 1000.0
        if (
            leader_age_ms < 0.0
            or gripper_age_ms < 0.0
            or leader_age_ms > leader_follower_max_age_ms
            or gripper_age_ms > leader_follower_max_age_ms
        ):
            report_feedback_hold(
                InventoryError(
                    "leader/follower feedback is stale: "
                    f"joints={leader_age_ms:.3f} ms, "
                    f"gripper={gripper_age_ms:.3f} ms"
                ),
                context="leader_follower",
            )
            return
        leader_timestamp_ns = int(feedback.timestamp_s * 1_000_000_000)
        gripper_timestamp_ns = int(gripper[1] * 1_000_000_000)
        source_ns = max(leader_timestamp_ns, gripper_timestamp_ns)
        if source_ns <= 0:
            source_ns = int(fallback_source_ns or time.time_ns())
        if source_ns <= last_lf_feedback_ns:
            return
        last_lf_feedback_ns = source_ns
        latest_source_ns = source_ns
        latest_expert_action = build_leader_follower_action(
            snapshot.state_14d,
            feedback,
            gripper[0],
            config=leader_follower,
        )
        dispatch_expert_target(snapshot, source_ns, time.time_ns())

    def publish_decision(
        decision: Any,
        source_ns: int,
    ) -> None:
        nonlocal expert_candidates, model_candidates
        if decision is None:
            return
        common = {
            "generation": decision.generation,
            "control_mode": decision.mode.value,
            "selected_action_source": decision.mode.value,
            "intervention_id": decision.intervention_id,
            "label_valid": decision.label_valid,
            "train_mask": decision.label_valid,
            "adjustment_reasons": ",".join(decision.adjustment_reasons),
        }
        if decision.control_tick_id is not None:
            common["decision_control_tick_id"] = decision.control_tick_id
        if decision.model_control:
            send_scalar(
                "model_control",
                json.dumps(
                    {
                        "command": decision.model_control,
                        "generation": decision.generation,
                    },
                    sort_keys=True,
                ),
                source_ns,
                generation=decision.generation,
            )
        if decision.event:
            output = (
                "takeover_event"
                if decision.event.startswith("takeover")
                else "control_event"
            )
            send_scalar(output, decision.event, source_ns, **common)
        send_scalar("control_mode", decision.mode.value, source_ns, **common)
        send_scalar("selected_action_source", decision.mode.value, source_ns, **common)
        send_scalar("label_valid", decision.label_valid, source_ns, **common)
        if decision.expert_proposed_action is not None:
            expert_candidates += 1
            send_action(
                "action_expert_proposed_robot",
                decision.expert_proposed_action,
                source_ns,
                **common,
            )
        if decision.expert_safe_action is not None:
            send_action(
                "action_expert_safe_robot",
                decision.expert_safe_action,
                source_ns,
                **common,
            )
        if decision.action is not None:
            if decision.mode.value == "MODEL":
                model_candidates += 1
                output = "action_model_robot"
            else:
                output = "action_selected_candidate_robot"
            send_action(output, decision.action, source_ns, **common)

    def hold_current(snapshot: Any) -> None:
        reset_model_filter("position_hold")
        if not write_enabled:
            return
        sdk_session.move_dual_for_live_control(
            snapshot.left.joint_position_rad,
            snapshot.right.joint_position_rad,
            speed_percent=int(live["speed_percent"]),
        )

        if os.getenv("RAX_LEFT_GRIPPER_DISPATCH") == "1":
            sdk_session.move_single_gripper_for_live_control("left_arm",
                snapshot.left.gripper_width_m, force_n=float(profile["shadow_teleop"]["gripper_force_n"]))

    def start_prepare(snapshot: Any) -> None:
        nonlocal prepare_started_at
        gripper_control_status.clear()
        if not write_enabled:
            return
        if os.getenv("PIPER_PREPARE_HOLD_CURRENT") == "1":
            if os.getenv("PIPER_GPT_TRAJECTORY_RELEASE") != "1":
                raise ValueError("current-pose preparation requires explicit GPT trajectory mode")
            prepare["initial_pose"] = {
                "left_arm": list(snapshot.left.joint_position_rad),
                "right_arm": list(snapshot.right.joint_position_rad),
                "left_gripper": float(snapshot.left.gripper_width_m),
                "right_gripper": float(snapshot.right.gripper_width_m),
            }
            print("PIPER_PREPARE_CURRENT_POSE no homing requested", flush=True)
        if os.getenv("RAX_PREPARE_LEFT_HOME") == "1":
            if os.getenv("PIPER_PREPARE_HOLD_CURRENT") == "1":
                raise ValueError("left home conflicts with current-pose preparation")
            prepare["initial_pose"]["right_arm"] = list(snapshot.right.joint_position_rad)
            prepare["initial_pose"]["left_gripper"] = float(snapshot.left.gripper_width_m)  # Preserve released jaw during home return.
            prepare["initial_pose"]["right_gripper"] = float(snapshot.right.gripper_width_m)
        pose = prepare["initial_pose"]
        sdk_session.move_dual_for_live_control(
            pose["left_arm"],
            pose["right_arm"],
            speed_percent=int(prepare["speed_percent"]),
        )
        if bool(live["gripper_write_enabled"]):
            sdk_session.move_dual_grippers_for_live_control(
                pose["left_gripper"],
                pose["right_gripper"],
                force_n=float(profile["shadow_teleop"]["gripper_force_n"]),
            )
        elif os.getenv("RAX_PREPARE_LEFT_HOME") == "1" and os.getenv("RAX_LEFT_GRIPPER_DISPATCH") == "1":
            sdk_session.move_single_gripper_for_live_control("left_arm",
                pose["left_gripper"], force_n=float(profile["shadow_teleop"]["gripper_force_n"]))
        prepare_started_at = time.monotonic()

    def current_boundary_event() -> str | None:
        if transition_started_at is not None:
            return "transition_pose"
        return controller.buttons.boundary_event

    def cancel_transition(reason: str, snapshot: Any | None) -> None:
        nonlocal transition_started_at, active_transition_pose
        nonlocal transition_waypoints, transition_waypoint_index, transition_status
        if transition_started_at is None:
            return
        pose_id = active_transition_pose.pose_id if active_transition_pose else None
        transition_started_at = None
        active_transition_pose = None
        transition_waypoints = ()
        transition_waypoint_index = 0
        transition_status = {
            "phase": "cancelled",
            "pose_id": pose_id,
            "reason": str(reason),
            "completed_timestamp_ns": time.time_ns(),
        }
        if snapshot is not None:
            hold_current(snapshot)

    def start_transition(snapshot: Any, request: Mapping[str, Any]) -> None:
        nonlocal transition_started_at, active_transition_pose
        nonlocal transition_waypoints, transition_waypoint_index, transition_status
        nonlocal hardware_write_count
        if transition_library is None:
            raise ValueError("this deployment has no transition-pose library")
        if transition_path_guard is None:  # pragma: no cover - construction invariant
            raise ValueError("transition path guard is unavailable")
        if not write_enabled:
            raise ValueError("transition-pose motion requires ENABLE_WRITE=1")
        if controller.lifecycle_state != "prepared":
            raise ValueError("transition pose requires prepared lifecycle")
        if controller.buttons.boundary_event is not None:
            raise ValueError("transition pose requires a completed lifecycle boundary")
        if prepare_started_at is not None or transition_started_at is not None:
            raise ValueError("another Piper boundary motion is already active")
        reason = str(request.get("reason", "task_transition")).strip().lower()
        if reason not in {"task_transition", "next_object", "recovery"}:
            raise ValueError("unsupported transition-pose reason")
        pose = transition_library.resolve_request(request)
        current = (
            *snapshot.left.joint_position_rad,
            *snapshot.right.joint_position_rad,
        )
        direct_bound = min(
            transition_library.max_joint_delta_rad,
            float(transition_config["max_joint_delta_rad"]),
        )
        segmented_enabled = bool(
            transition_config.get("segmented_transition_enabled", False)
        )
        segmented_total_bound = transition_config.get(
            "segmented_max_joint_delta_rad"
        )
        if segmented_total_bound is not None:
            segmented_total_bound = float(segmented_total_bound)
        home_fallback_enabled = bool(
            transition_config.get("home_fallback_enabled", False)
        )
        home_fallback_max_waypoints = int(
            transition_config.get("home_fallback_max_waypoints", 2)
        )

        def validate_route(
            waypoints: tuple[tuple[float, ...], ...],
        ) -> dict[str, float]:
            if not waypoints:
                raise ValueError("transition route did not produce any waypoints")
            minima = {"left": math.inf, "right": math.inf}
            previous = current
            for waypoint in waypoints:
                segment_heights = transition_path_guard.validate_interpolation(
                    previous,
                    waypoint,
                    samples=int(transition_config["interpolation_samples"]),
                    minimum_flange_height_m=float(
                        transition_config["minimum_interpolated_flange_height_m"]
                    ),
                )
                for side in minima:
                    minima[side] = min(minima[side], segment_heights[side])
                previous = waypoint
            return minima

        # A transition target is always an allowlisted, joint-limit-checked
        # recorded pose.  The measured state may nevertheless be on the wrong
        # side of the kinematic height floor.  In that case we can still reach
        # the target through the profile's clear/home pose, while validating
        # every individual segment.  We never bypass the guard or fabricate a
        # target; if the bounded route is unsafe, this operation fails closed.
        maximum_delta = max(
            abs(target - measured)
            for target, measured in zip(
                pose.arm_joint_position_rad, current, strict=True
            )
        )
        home_fallback_reason: str | None = None
        routing_via_home = False
        direct_error: str | None = None

        def fallback_eligible(reason: str) -> bool:
            return reason.startswith(
                (
                    "transition pose requires ",
                    "transition requires ",
                )
            ) or reason == (
                "transition joint interpolation violates the flange-height floor"
            )

        try:
            maximum_delta = transition_library.validate_move(
                current,
                pose,
                profile_max_joint_delta_rad=float(
                    transition_config["max_joint_delta_rad"]
                ),
                allow_segmented=segmented_enabled,
                profile_segmented_max_joint_delta_rad=segmented_total_bound,
            )
        except ValueError as exc:
            direct_error = str(exc)
            if (
                not home_fallback_enabled
                or not segmented_enabled
                or not fallback_eligible(direct_error)
            ):
                raise

        try:
            if direct_error is not None:
                raise ValueError(direct_error)
            transition_waypoints = transition_library.segmented_waypoints(
                current,
                pose.arm_joint_position_rad,
                max_segment_joint_delta_rad=direct_bound,
                max_waypoints=(
                    int(transition_config.get("max_segmented_waypoints", 1))
                    if maximum_delta > direct_bound
                    else 1
                ),
            )
            path_heights = validate_route(transition_waypoints)
        except ValueError as exc:
            if (
                not home_fallback_enabled
                or not segmented_enabled
                or not fallback_eligible(str(exc))
            ):
                raise
            home_fallback_reason = str(exc)
            initial_pose = prepare["initial_pose"]
            home = (
                *tuple(float(value) for value in initial_pose["left_arm"]),
                *tuple(float(value) for value in initial_pose["right_arm"]),
            )
            transition_waypoints = transition_library.segmented_route(
                current,
                home,
                pose.arm_joint_position_rad,
                max_segment_joint_delta_rad=direct_bound,
                max_waypoints_per_leg=home_fallback_max_waypoints,
            )
            path_heights = validate_route(transition_waypoints)
            routing_via_home = True
        first_waypoint = transition_waypoints[0]
        dispatch = sdk_session.move_dual_for_live_control(
            first_waypoint[:6],
            first_waypoint[6:],
            speed_percent=int(transition_config["speed_percent"]),
        )
        hardware_write_count += 1
        active_transition_pose = pose
        transition_waypoint_index = 0
        transition_started_at = time.monotonic()
        transition_status = {
            "phase": "moving",
            "request_id": str(request.get("request_id", "")),
            "pose_id": pose.pose_id,
            "task_prompt": pose.task_prompt,
            "reason": reason,
            "library_id": transition_library.library_id,
            "library_sha256": transition_library.sha256,
            "maximum_joint_delta_rad": maximum_delta,
            "segmented": len(transition_waypoints) > 1,
            "routing_via_home": routing_via_home,
            "home_fallback_reason": home_fallback_reason,
            "waypoint_count": len(transition_waypoints),
            "waypoint_index": 1,
            "waypoint_max_joint_delta_rad": direct_bound,
            "timeout_seconds": float(transition_config["timeout_seconds"])
            * len(transition_waypoints),
            "minimum_interpolated_flange_height_m": path_heights,
            "gripper_policy": "preserve_measured",
            "owner_dispatch": "ALLOWLISTED_DATASET_TRANSITION_MOVE_J",
            "dispatch_skew_ms": dispatch.skew_ms,
            "started_timestamp_ns": time.time_ns(),
        }
        if routing_via_home:
            print(
                "PIPER_X_DUAL_TRANSITION_HOME_FALLBACK "
                + json.dumps(
                    {
                        "pose_id": pose.pose_id,
                        "reason": reason,
                        "fallback_reason": home_fallback_reason,
                        "waypoint_count": len(transition_waypoints),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
        reset_model_filter(f"transition_pose:{reason}")
        print(
            "PIPER_X_DUAL_TRANSITION_POSE "
            + json.dumps(transition_status, sort_keys=True),
            flush=True,
        )

    def service_transition(snapshot: Any, source_ns: int) -> None:
        nonlocal transition_started_at, active_transition_pose
        nonlocal transition_waypoints, transition_waypoint_index, transition_status
        nonlocal hardware_write_count
        if (
            transition_started_at is None
            or active_transition_pose is None
            or not transition_waypoints
        ):
            return
        pose = active_transition_pose
        waypoint = transition_waypoints[transition_waypoint_index]
        errors = [
            abs(current - target)
            for current, target in zip(
                (
                    *snapshot.left.joint_position_rad,
                    *snapshot.right.joint_position_rad,
                ),
                waypoint,
                strict=True,
            )
        ]
        elapsed = time.monotonic() - transition_started_at
        if max(errors) <= float(transition_config["position_tolerance_rad"]):
            if transition_waypoint_index + 1 < len(transition_waypoints):
                transition_waypoint_index += 1
                next_waypoint = transition_waypoints[transition_waypoint_index]
                dispatch = sdk_session.move_dual_for_live_control(
                    next_waypoint[:6],
                    next_waypoint[6:],
                    speed_percent=int(transition_config["speed_percent"]),
                )
                hardware_write_count += 1
                transition_status = {
                    **transition_status,
                    "phase": "moving",
                    "waypoint_index": transition_waypoint_index + 1,
                    "waypoint_dispatch_skew_ms": dispatch.skew_ms,
                }
                print(
                    "PIPER_X_DUAL_TRANSITION_POSE "
                    + json.dumps(transition_status, sort_keys=True),
                    flush=True,
                )
                return
            completed = {
                **transition_status,
                "phase": "complete",
                "maximum_final_error_rad": max(errors),
                "elapsed_seconds": elapsed,
                "completed_timestamp_ns": time.time_ns(),
            }
            transition_started_at = None
            active_transition_pose = None
            transition_waypoints = ()
            transition_waypoint_index = 0
            transition_status = completed
            send_scalar(
                "control_event",
                "transition_pose_complete",
                source_ns,
                accepted=True,
                lifecycle_state=controller.lifecycle_state,
                pose_id=pose.pose_id,
            )
            print(
                "PIPER_X_DUAL_TRANSITION_POSE "
                + json.dumps(completed, sort_keys=True),
                flush=True,
            )
        elif elapsed >= float(
            transition_status.get("timeout_seconds", transition_config["timeout_seconds"])
        ):
            failed = {
                **transition_status,
                "phase": "failed",
                "reason": "transition_timeout",
                "maximum_final_error_rad": max(errors),
                "elapsed_seconds": elapsed,
                "completed_timestamp_ns": time.time_ns(),
            }
            transition_started_at = None
            active_transition_pose = None
            transition_waypoints = ()
            transition_waypoint_index = 0
            transition_status = failed
            hold_current(snapshot)
            send_scalar(
                "control_event",
                "transition_pose_failed",
                source_ns,
                accepted=False,
                lifecycle_state=controller.lifecycle_state,
                rejection_reason="transition_timeout",
                pose_id=pose.pose_id,
            )
            print(
                "PIPER_X_DUAL_TRANSITION_POSE "
                + json.dumps(failed, sort_keys=True),
                flush=True,
            )

    def publish_step(
        step: PiperShadowControlStep,
        source_ns: int,
        *,
        snapshot: Any | None,
        termination_reason: str | None = None,
    ) -> None:
        nonlocal episode_index, recording, prepare_started_at, next_control_tick_id
        if step.transition is not None:
            transition = step.transition
            print(
                "PIPER_X_DUAL_CONTROL "
                + json.dumps(
                    {
                        "event": transition.event,
                        "accepted": transition.accepted,
                        "state": transition.state,
                        "reason": transition.reason,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            send_scalar(
                "control_event",
                transition.event,
                source_ns,
                accepted=transition.accepted,
                lifecycle_state=transition.state,
                rejection_reason=transition.reason,
            )
            if transition.accepted and transition.event in {
                "prepare",
                "start",
                "stop",
                "finish",
                "discard",
                "pause",
            }:
                reset_model_filter(f"lifecycle:{transition.event}")
            if transition.accepted and transition.event == "start":
                bounded_rollout.start()
                next_control_tick_id = 0
                episode_index += 1
                recording = True
                send_scalar("episode_index", episode_index, source_ns)
            elif transition.accepted and transition.event in {
                "stop",
                "finish",
                "discard",
                "pause",
            }:
                resolved_termination_reason = termination_reason or (
                    "operator_finish_pending_review"
                    if transition.event == "finish"
                    else "operator_discard"
                    if transition.event == "discard"
                    else "bounded_rollout_paused"
                    if transition.event == "pause"
                    else "operator_stop"
                )
                bounded_rollout.stop(resolved_termination_reason)
                outcome = "UNKNOWN"
                send_scalar(
                    "episode_result",
                    json.dumps(
                        {
                            "task_outcome": outcome,
                            "outcome_confirmed_by_operator": False,
                            "termination_reason": resolved_termination_reason,
                            "rollout": bounded_rollout.status(),
                        },
                        sort_keys=True,
                    ),
                    source_ns,
                )
                if transition.event == "discard":
                    send_scalar("episode_discard", episode_index, source_ns)
                send_scalar("episode_index", -1, source_ns)
                recording = False
            if transition.accepted and transition.event == "prepare":
                if snapshot is None:
                    raise RuntimeError("prepare requires fresh Piper feedback")
                start_prepare(snapshot)
            elif transition.accepted and transition.event in {
                "stop",
                "finish",
                "discard",
                "pause",
            }:
                # The lifecycle/model scheduler must always be cancellable,
                # including after feedback health has already failed.  A
                # position hold still requires a trustworthy measured pose;
                # never fabricate one from stale feedback.
                if snapshot is not None:
                    hold_current(snapshot)
                prepare_started_at = None
        publish_decision(step.control, source_ns)
        publish_decision(step.takeover, source_ns)
        publish_decision(step.expert, source_ns)

    def service_prepare(snapshot: Any, source_ns: int) -> None:
        nonlocal prepare_started_at
        if prepare_started_at is None:
            return
        pose = prepare["initial_pose"]
        joint_errors = [
            abs(current - float(target))
            for current, target in zip(
                (
                    *snapshot.left.joint_position_rad,
                    *snapshot.right.joint_position_rad,
                ),
                (*pose["left_arm"], *pose["right_arm"]),
                strict=True,
            )
        ]
        gripper_errors = [
            abs(snapshot.left.gripper_width_m - float(pose["left_gripper"])),
            abs(snapshot.right.gripper_width_m - float(pose["right_gripper"])),
        ]
        elapsed = time.monotonic() - prepare_started_at
        if (
            max(joint_errors) <= float(prepare["position_tolerance_rad"])
            and max(gripper_errors) <= float(prepare["gripper_tolerance_m"])
        ):
            prepare_started_at = None
            transition, control = controller.complete_prepare(success=True)
            send_scalar(
                "control_event",
                "prepare_complete",
                source_ns,
                accepted=True,
                lifecycle_state=controller.lifecycle_state,
            )
            print(
                'PIPER_X_DUAL_CONTROL {"event": "prepare_complete", '
                '"accepted": true, "state": "prepared"}',
                flush=True,
            )
            if transition is not None or control is not None:
                publish_step(
                    PiperShadowControlStep(transition=transition, control=control),
                    source_ns,
                    snapshot=snapshot,
                )
        elif elapsed >= float(prepare["timeout_seconds"]):
            prepare_started_at = None
            controller.complete_prepare(success=False)
            hold_current(snapshot)
            send_scalar(
                "control_event",
                "prepare_failed",
                source_ns,
                accepted=False,
                lifecycle_state=controller.lifecycle_state,
                rejection_reason="prepare_timeout",
            )

    with sdk_session as owner:
        # Establish fresh passive feedback from both physical controllers
        # before the first write. This produces a precise missing-CAN error
        # and avoids racing motor enable against SDK receiver startup. A
        # missing arm must leave the owner alive in HOLD so the operator UI
        # can report the failed readiness gate; an unhandled exception here
        # used to kill the owner before it could publish that reason.
        time.sleep(float(profile["read_only_probe"]["status_timeout_seconds"]))
        if calibrate_gripper_arm:
            from .gripper_zero import calibrate_closed_gripper
            result = calibrate_closed_gripper(sdk_session, calibrate_gripper_arm)
            print("PIPER_GRIPPER_ZERO " + json.dumps(result, sort_keys=True), flush=True)
            return
        if release_stop_arm:
            # Exit before normal startup enable and before processing any
            # operator/model event. The formal graph retains the sole owner.
            result = sdk_session.release_software_stop(release_stop_arm)
            payload = {"status": "SOFTWARE_STOP_RELEASE_SENT", **result}
            send_json("status", payload)
            print("PIPER_SOFTWARE_STOP_RELEASE " + json.dumps(payload, sort_keys=True), flush=True)
            return
        startup_feedback_error: str | None = None
        try:
            owner.snapshot()
        except InventoryError as exc:
            startup_feedback_error = str(exc)
            report_feedback_hold(exc, context="startup")
            startup_hold_payload = {
                "status": "HOLD_FEEDBACK_UNAVAILABLE",
                "lifecycle_state": controller.lifecycle_state,
                "hardware_write_count": hardware_write_count,
                "reason": startup_feedback_error,
                "camera_status": camera_status,
            }
            send_json("status", startup_hold_payload)
            print(
                "PIPER_X_DUAL_OWNER "
                + json.dumps(startup_hold_payload, sort_keys=True),
                flush=True,
            )
        if cached_diagnostic_path is not None:
            from .cached_diagnostics import write_cached_status

            write_cached_status(
                cached_diagnostic_path, sdk_session._arms, write_enabled=write_enabled
            )
            print(
                "PIPER_READONLY_STATUS_SAVED " + str(cached_diagnostic_path),
                flush=True,
            )
        if write_enabled and startup_feedback_error is None:
            if initialize_left_gripper_width:
                initialization = sdk_session.initialize_gripper_width_mode(
                    "left_arm",
                    initialize_left_gripper_width,
                    force_n=float(profile["shadow_teleop"]["gripper_force_n"]),
                    timeout_s=float(prepare["timeout_seconds"]),
                    tolerance_m=float(prepare["gripper_tolerance_m"]),
                )
                hardware_write_count += 1
                print(
                    "PIPER_X_DUAL_GRIPPER_INITIALIZATION "
                    + json.dumps(initialization, sort_keys=True),
                    flush=True,
                )
            try:
                sdk_session.enable_for_live_control()
            except (InventoryError, RuntimeError) as exc:
                startup_feedback_error = str(exc)
                report_feedback_hold(exc, context="startup_enable")
        elif write_enabled:
            # Never enable motors or attempt live dispatch after an
            # incomplete startup snapshot. Restart after both arms report
            # fresh feedback; keeping the owner alive preserves a clear HOLD
            # status and lets the operator stop the run cleanly.
            print(
                "PIPER_X_DUAL_WRITE_HOLD "
                + json.dumps(
                    {"reason": startup_feedback_error}, sort_keys=True
                ),
                flush=True,
            )
        readiness_suffix = "LIVE_TAKEOVER" if write_enabled else "NO_WRITE"
        mapping_suffix = "live" if write_enabled else "no_send"
        waiting_payload = {
            "status": (
                "HOLD_FEEDBACK_UNAVAILABLE"
                if startup_feedback_error is not None
                else f"WAITING_CAMERAS_{readiness_suffix}"
            ),
            "hardware_write_count": hardware_write_count,
            "arm_mapping": f"host_mink_joint_stream_{mapping_suffix}",
            "camera_status": camera_status,
            "feedback_error": startup_feedback_error,
            "collection_mode": collection_mode,
            "recorder_enabled": _truthy("ENABLE_DAGGER_RECORDING"),
            "record_dir": os.getenv("DAGGER_RECORD_DIR", ""),
            "dagger_expert_source": expert_source,
            "model_filter": model_filter.status(),
            "motion_trace": motion_trace.status(),
            "leader_follower": {
                "leader_role": leader_follower.leader_role,
                "follower_role": leader_follower.follower_role,
                "feedback_role": leader_follower.feedback_role,
                "native_linkage": leader_follower.native_linkage,
                "max_age_ms": leader_follower_max_age_ms,
            },
        }
        send_json("status", waiting_payload)
        print(
            "PIPER_X_DUAL_OWNER " + json.dumps(waiting_payload, sort_keys=True),
            flush=True,
        )

        try:
            for event in node:
                if event is None or event.get("type") == "STOP":
                    break
                if event.get("type") != "INPUT":
                    continue
                event_id = str(event["id"])
                takeover_released = False
                event_metadata = event.get("metadata", {})
                source_ns = int(
                    event_metadata.get("source_timestamp_ns")
                    or event_metadata.get("timestamp_ns")
                    or time.time_ns()
                )
                if (
                    event_id == "button_command"
                    and str(_scalar(event["value"])) == "discard"
                    and not expert_only
                ):
                    send_scalar(
                        "control_event",
                        "discard",
                        source_ns,
                        accepted=False,
                        lifecycle_state=controller.lifecycle_state,
                        rejection_reason="expert_only_required",
                    )
                    continue
                if event_id == "expert_action":
                    if expert_source != "qnexo":
                        # The leader/follower source is sampled by this owner
                        # from the SDK so the retarget node cannot race it.
                        continue
                    latest_expert_action = _named_vector(event["value"])
                    latest_source_ns = source_ns
                    # questVR_ws uses queue_size=1 and dispatches each newest IK
                    # result directly from its callback. Do the same here: a
                    # second free-running timer otherwise repeats old targets
                    # and then bunches the next step, producing stop/go motion.
                    if (
                        write_enabled
                        and takeover
                        and camera_ready
                        and controller.lifecycle_state == "running"
                        and not controller.buttons.boundary_active
                    ):
                        try:
                            snapshot = owner.snapshot()
                        except InventoryError as exc:
                            report_feedback_hold(exc, context="expert_action")
                        else:
                            dispatch_expert_target(
                                snapshot,
                                latest_source_ns,
                                int(
                                    event_metadata.get("timestamp_ns")
                                    or time.time_ns()
                                ),
                            )
                    continue
                if event_id == "camera_status":
                    parsed_status = json.loads(str(_scalar(event["value"])))
                    if not isinstance(parsed_status, dict):
                        raise ValueError("camera status must be a JSON object")
                    camera_status = parsed_status
                    was_camera_ready = camera_ready
                    became_ready = (
                        not camera_ready
                        and camera_status.get("status") == "READY_STREAMING"
                    )
                    camera_ready = camera_status.get("status") == "READY_STREAMING"
                    if was_camera_ready and not camera_ready and write_enabled:
                        try:
                            hold_current(owner.snapshot())
                        except InventoryError as exc:
                            report_feedback_hold(exc, context="camera_status")
                    if became_ready or was_camera_ready != camera_ready:
                        readiness_feedback_error = None
                        try:
                            owner.snapshot()
                        except InventoryError as exc:
                            readiness_feedback_error = str(exc)
                            report_feedback_hold(exc, context="camera_readiness")
                        ready_payload = {
                            "status": owner_readiness_status(
                                camera_ready=camera_ready, write_enabled=write_enabled,
                                feedback_error=readiness_feedback_error,
                                startup_feedback_error=startup_feedback_error,
                            ),
                            "feedback_error": readiness_feedback_error,
                            "startup_feedback_error": startup_feedback_error,
                            "hardware_write_count": hardware_write_count,
                            "arm_mapping": (
                                f"host_mink_joint_stream_{mapping_suffix}"
                            ),
                            "camera_status": camera_status,
                            "collection_mode": collection_mode,
                            "recorder_enabled": _truthy("ENABLE_DAGGER_RECORDING"),
                            "record_dir": os.getenv("DAGGER_RECORD_DIR", ""),
                            "dagger_expert_source": expert_source,
                            "model_filter": model_filter.status(),
                            "leader_follower": {
                                "leader_role": leader_follower.leader_role,
                                "follower_role": leader_follower.follower_role,
                                "feedback_role": leader_follower.feedback_role,
                                "native_linkage": leader_follower.native_linkage,
                                "max_age_ms": leader_follower_max_age_ms,
                            },
                        }
                        send_json("status", ready_payload, source_ns)
                        print(
                            "PIPER_X_DUAL_OWNER "
                            + json.dumps(ready_payload, sort_keys=True),
                            flush=True,
                        )
                    continue
                if event_id == "rollout_config":
                    configuration = json.loads(str(_scalar(event["value"])))
                    if not isinstance(configuration, Mapping):
                        raise ValueError("rollout configuration must be a JSON object")
                    steps = bounded_rollout.configure(
                        configuration.get("max_control_steps"),
                        lifecycle_state=controller.lifecycle_state,
                    )
                    print(
                        "PIPER_X_DUAL_ROLLOUT_CONFIG "
                        + json.dumps(
                            {
                                "max_control_steps": steps,
                                "operator_source": event_metadata.get(
                                    "operator_source", "unknown"
                                ),
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                    continue
                if event_id == "expert_config":
                    try:
                        configuration = json.loads(str(_scalar(event["value"])))
                        if not isinstance(configuration, Mapping):
                            raise ValueError("expert configuration must be a JSON object")
                        if controller.lifecycle_state in {"running", "finishing"}:
                            raise ValueError(
                                "stop or finish the current episode before changing expert control"
                            )
                        selected_source = parse_expert_source(
                            configuration.get("source", "qnexo")
                        )
                        selected_mapping = LeaderFollowerConfig.from_mapping(
                            configuration
                        )
                        if selected_mapping.native_linkage:
                            raise ValueError(
                                "firmware-native leader/follower linkage is not enabled; use the SDK feedback bridge"
                            )
                        selected_age = float(
                            configuration.get(
                                "max_age_ms", leader_follower_max_age_ms
                            )
                        )
                        if not 1.0 <= selected_age <= 1000.0:
                            raise ValueError(
                                "leader/follower feedback age must be in [1, 1000] ms"
                            )
                    except (TypeError, ValueError, json.JSONDecodeError) as exc:
                        send_json(
                            "status",
                            {
                                "status": "EXPERT_CONFIG_REJECTED",
                                "dagger_expert_source": expert_source,
                                "reason": str(exc),
                            },
                            source_ns,
                        )
                        print(
                            "PIPER_X_DUAL_EXPERT_CONFIG_REJECTED "
                            + json.dumps({"reason": str(exc)}, sort_keys=True),
                            flush=True,
                        )
                        continue
                    expert_source = selected_source
                    leader_follower = selected_mapping
                    leader_follower_max_age_ms = selected_age
                    latest_expert_action = None
                    latest_source_ns = 0
                    last_lf_feedback_ns = 0
                    applied = {
                        "source": expert_source,
                        "leader_role": leader_follower.leader_role,
                        "follower_role": leader_follower.follower_role,
                        "feedback_role": leader_follower.feedback_role,
                        "native_linkage": leader_follower.native_linkage,
                        "max_age_ms": leader_follower_max_age_ms,
                    }
                    send_json(
                        "status",
                        {
                            "status": "EXPERT_CONFIG_APPLIED",
                            "dagger_expert_source": expert_source,
                            "leader_follower": {
                                **applied,
                            },
                        },
                        source_ns,
                    )
                    print(
                        "PIPER_X_DUAL_EXPERT_CONFIG "
                        + json.dumps(applied, sort_keys=True),
                        flush=True,
                    )
                    continue
                if event_id == "takeover":
                    previous_takeover = takeover
                    takeover = expert_only or bool(_scalar(event["value"]))
                    takeover_released = previous_takeover and not takeover
                    if takeover != previous_takeover:
                        reset_model_filter(
                            "takeover_started" if takeover else "takeover_released"
                        )
                    if not takeover:
                        latest_expert_action = None
                        last_lf_feedback_ns = 0
                    if takeover_released:
                        write_trajectory_capture("takeover_released")
                elif event_id == "model_action":
                    incoming_action = _named_vector(event["value"])
                    with motion_trace.candidate(incoming_action, metadata=event_metadata,
                                                source_ns=source_ns) as trace_sample:
                        decision = controller.update_model(
                            incoming_action,
                            int(event_metadata.get("generation", 0)),
                        )
                        publish_decision(decision, source_ns)
                        if decision is not None and decision.action is not None:
                            limit_reached = (
                                controller.lifecycle_state == "running"
                                and bounded_rollout.record_model_step()
                            )
                            model_snapshot = None
                            try:
                                model_snapshot = owner.snapshot()
                            except InventoryError as exc:
                                report_feedback_hold(exc, context="model_action")
                                invalidate_precommit("feedback_unavailable: " + str(exc), source_ns)
                                if trace_sample is not None:
                                    trace_sample.update(owner_result="HOLD_FEEDBACK_UNAVAILABLE", reasons=[str(exc)])
                            else:
                                def model_dispatch():
                                    dispatch_selected_action(
                                    decision.action,
                                    model_snapshot,
                                    source_ns,
                                    int(
                                        event_metadata.get("timestamp_ns")
                                        or time.time_ns()
                                    ),
                                    mode="MODEL",
                                    generation=decision.generation,
                                    policy_metadata=event_metadata,
                                    trace_sample=trace_sample,
                                )
                                if precommit_fault_invalidation:
                                    from XPolicyLab.real_world.precommit_limits import guarded_dispatch
                                    guarded_dispatch(model_dispatch, lambda reason:
                                        invalidate_precommit("dispatch_failure: " + reason, source_ns, model_snapshot))
                                else:
                                    model_dispatch()
                            if limit_reached:
                                print(
                                    "PIPER_X_DUAL_ROLLOUT_LIMIT "
                                    + json.dumps(bounded_rollout.status(), sort_keys=True),
                                    flush=True,
                                )
                                if model_snapshot is None:
                                    bounded_stop = feedback_unavailable_boundary_step(
                                        controller, "stop"
                                    )
                                    if bounded_stop is None:  # pragma: no cover
                                        raise RuntimeError("bounded stop was not accepted")
                                else:
                                    bounded_stop = controller.pause_model_rollout()
                                publish_step(
                                    bounded_stop,
                                    source_ns,
                                    snapshot=model_snapshot,
                                    termination_reason="pigey_rollout_limit",
                                )
                    continue
                elif event_id == "model_status":
                    status = json.loads(str(_scalar(event["value"])))
                    if (status.get("status") == "error"
                            and int(status["generation"]) == controller.generation):
                        invalidate_precommit("policy_error: " + str(status.get("error", "unknown")), source_ns)
                    publish_decision(
                        controller.update_model_status(
                            str(status["status"]), int(status["generation"])
                        ),
                        source_ns,
                    )
                    continue
                elif event_id not in {
                    "button_command",
                    "transition_command",
                    "tick",
                }:
                    continue

                if event_id == "button_command" and str(_scalar(event["value"])) in {"open_grippers", "open_left_gripper"}:
                    gripper_command = str(_scalar(event["value"]))
                    gripper_scope = "left" if gripper_command == "open_left_gripper" else "both"
                    gripper_control_status = {
                        "accepted": False, "requested_timestamp_ns": source_ns,
                    }
                    try:
                        if os.getenv("PIPER_PREPARE_HOLD_CURRENT") == "1" or collection_mode != "OFF":
                            raise ValueError("manual grippers require ordinary initial-pose deployment")
                        gripper_snapshot = owner.snapshot()
                        target = dispatch_manual_gripper_open(
                            gripper_snapshot, controller=controller, sdk_session=sdk_session,
                            pause_model=lambda control: publish_decision(control, source_ns),
                            profile=profile, gripper_scope=gripper_scope, safety_guard=safety_guard,
                            write_enabled=write_enabled, camera_ready=camera_ready,
                            startup_feedback_error=startup_feedback_error,
                            boundary_active=current_boundary_event() is not None,
                            takeover=takeover, recording=recording,
                            operator_source=str(event_metadata.get("operator_source", "")),
                            command_timestamp_ns=source_ns,
                            now_ns=time.time_ns(),
                        )
                        reset_model_filter("manual_grippers")
                        hardware_write_count += 1
                        gripper_control_status.update(
                            accepted=True, reason="", left_target_m=target["left_gripper"],
                            right_target_m=target["right_gripper"], prepare_required=True,
                            scope=gripper_scope, sdk_dispatch_confirmed=True, pose_arrival_confirmed=False,
                        )
                    except (InventoryError, ValueError, RuntimeError) as exc:
                        gripper_control_status["reason"] = str(exc)
                        if isinstance(exc, InventoryError):
                            gripper_control_status["feedback_error"] = str(exc)
                    send_scalar(
                        "control_event", gripper_command, source_ns,
                        accepted=gripper_control_status["accepted"],
                        lifecycle_state=controller.lifecycle_state,
                        rejection_reason=gripper_control_status.get("reason", ""),
                    )
                    send_json("status", {
                        "status": owner_readiness_status(
                            camera_ready=camera_ready, write_enabled=write_enabled,
                            startup_feedback_error=startup_feedback_error,
                            feedback_error=gripper_control_status.get("feedback_error"),
                        ),
                        "gripper_control": dict(gripper_control_status),
                        "lifecycle_state": controller.lifecycle_state,
                        "hardware_write_count": hardware_write_count,
                    }, source_ns)
                    print("PIPER_GRIPPER_CONTROL " + json.dumps(gripper_control_status, sort_keys=True), flush=True)
                    continue

                try:
                    snapshot = owner.snapshot()
                except InventoryError as exc:
                    report_feedback_hold(exc, context=event_id)
                    now_monotonic = time.monotonic()
                    if now_monotonic >= next_status_at:
                        send_json("status", {
                            "status": "HOLD_FEEDBACK_UNAVAILABLE",
                            "lifecycle_state": controller.lifecycle_state,
                            "hardware_write_enabled": write_enabled,
                            "hardware_write_count": hardware_write_count,
                            "feedback_error": str(exc),
                            "camera_status": camera_status,
                            "gripper_control": dict(gripper_control_status),
                        }, source_ns)
                        next_status_at = now_monotonic + 1.0
                    # Stop/finish are fail-closed control boundaries, not
                    # motion requests.  Process them even when feedback is
                    # unavailable so the model scheduler is cancelled and X
                    # returns the lifecycle to a retryable idle state.  We do
                    # not issue a position hold without a fresh measured pose.
                    if event_id == "button_command":
                        button_event = str(_scalar(event["value"]))
                        boundary_step = feedback_unavailable_boundary_step(
                            controller,
                            button_event,
                            feedback_error=str(exc),
                        )
                        if boundary_step is not None:
                            publish_step(
                                boundary_step,
                                source_ns,
                                snapshot=None,
                            )
                            send_json(
                                "status",
                                {
                                    "status": "HOLD_FEEDBACK_UNAVAILABLE",
                                    "lifecycle_state": controller.lifecycle_state,
                                    "boundary_event": current_boundary_event(),
                                    "generation": controller.generation,
                                    "takeover": takeover,
                                    "recording": recording,
                                    "collection_mode": collection_mode,
                                    "hardware_write_enabled": write_enabled,
                                    "feedback_error": str(exc),
                                },
                                source_ns,
                            )
                    continue
                if rax_lease is not None:
                    lease_stop = rax_lease.stop_reason(snapshot.state_14d)
                    if lease_stop is not None:
                        step = controller.update(payload=None, expert_action=None, button_event="stop",
                            takeover_active=takeover, snapshot=snapshot)
                        publish_step(step, source_ns, snapshot=snapshot, termination_reason="rax_lease: " + lease_stop)
                        print("RAX_MOTION_LEASE_STOP " + lease_stop, flush=True)
                        continue
                if takeover_released:
                    hold_current(snapshot)
                if not camera_ready:
                    if event_id == "tick":
                        observations += 1
                        send_action(
                            "observation_state_robot",
                            action_dict(snapshot.state_14d),
                            source_ns,
                            maximum_age_ms=snapshot.maximum_age_ms,
                            inter_arm_skew_ms=snapshot.sample_skew_ms,
                            readiness=f"WAITING_CAMERAS_{readiness_suffix}",
                        )
                        send_cartesian_observation(source_ns)
                    continue
                if event_id == "transition_command":
                    request: Mapping[str, Any] | None = None
                    try:
                        if str(event_metadata.get("operator_source", "")).lower() != "pigey":
                            raise ValueError(
                                "transition-pose requests are reserved for Pigey"
                            )
                        request = json.loads(str(_scalar(event["value"])))
                        if not isinstance(request, Mapping):
                            raise ValueError(
                                "transition-pose request must be a JSON object"
                            )
                        start_transition(snapshot, request)
                    except (TypeError, ValueError, json.JSONDecodeError) as exc:
                        request_id = ""
                        pose_id = ""
                        if request is not None:
                            request_id = str(request.get("request_id", ""))
                            pose_id = str(request.get("pose_id", ""))
                        transition_status = {
                            "phase": "rejected",
                            "request_id": request_id,
                            "pose_id": pose_id,
                            "reason": str(exc),
                            "completed_timestamp_ns": time.time_ns(),
                        }
                        send_scalar(
                            "control_event",
                            "transition_pose_rejected",
                            source_ns,
                            accepted=False,
                            lifecycle_state=controller.lifecycle_state,
                            rejection_reason=str(exc),
                            request_id=request_id,
                            pose_id=pose_id,
                        )
                        print(
                            "PIPER_X_DUAL_TRANSITION_POSE_REJECTED "
                            + json.dumps(transition_status, sort_keys=True),
                            flush=True,
                        )
                    continue
                if event_id == "tick":
                    service_prepare(snapshot, latest_source_ns or source_ns)
                    service_transition(snapshot, latest_source_ns or source_ns)
                    if (
                        expert_source == "leader_follower"
                        and takeover
                        and controller.lifecycle_state == "running"
                        and not controller.buttons.boundary_active
                        and transition_started_at is None
                    ):
                        try:
                            refresh_leader_follower_expert(snapshot, source_ns)
                        except InventoryError as exc:
                            report_feedback_hold(exc, context="leader_follower")
                button_event = (
                    str(_scalar(event["value"]))
                    if event_id == "button_command"
                    else None
                )
                if button_event in {"stop", "finish", "discard", "prepare"}:
                    cancel_transition(f"operator_{button_event}", snapshot)
                if button_event == "pause":
                    step = controller.pause_model_rollout()
                else:
                    step = controller.update(
                        payload=None,
                        expert_action=None,
                        button_event=button_event,
                        takeover_active=takeover,
                        snapshot=snapshot,
                    )
                publish_step(
                    step,
                    latest_source_ns or source_ns,
                    snapshot=snapshot,
                    termination_reason=(
                        "pigey_task_transition"
                        if button_event == "pause"
                        else None
                    ),
                )
                if event_id != "tick":
                    continue

                observations += 1
                send_action(
                    "observation_state_robot",
                    action_dict(snapshot.state_14d),
                    source_ns,
                    maximum_age_ms=snapshot.maximum_age_ms,
                    inter_arm_skew_ms=snapshot.sample_skew_ms,
                )
                send_cartesian_observation(source_ns)
                status_payload = {
                    "status": owner_readiness_status(
                        camera_ready=camera_ready, write_enabled=write_enabled,
                        startup_feedback_error=startup_feedback_error,
                    ),
                    "startup_feedback_error": startup_feedback_error,
                    "lifecycle_state": controller.lifecycle_state,
                    "boundary_event": current_boundary_event(),
                    "generation": controller.generation,
                    "takeover": takeover,
                    "recording": recording,
                    "collection_mode": collection_mode,
                    "recorder_enabled": _truthy("ENABLE_DAGGER_RECORDING"),
                    "record_dir": os.getenv("DAGGER_RECORD_DIR", ""),
                    "dagger_expert_source": expert_source,
                    "model_filter": model_filter.status(),
                    "motion_trace": motion_trace.status(),
                    "leader_follower": {
                        "leader_role": leader_follower.leader_role,
                        "follower_role": leader_follower.follower_role,
                        "feedback_role": leader_follower.feedback_role,
                        "native_linkage": leader_follower.native_linkage,
                        "max_age_ms": leader_follower_max_age_ms,
                    },
                    "observations": observations,
                    "expert_candidates": expert_candidates,
                    "model_candidates": model_candidates,
                    "model_boundary_normalizations": model_boundary_normalization_count,
                    "hardware_write_count": hardware_write_count,
                    "hardware_write_enabled": write_enabled,
                    "review_profile": review_profile,
                    "rax_motion": {"enabled": False} if rax_lease is None else {**rax_lease.status(), "left_gripper_dispatch": os.getenv("RAX_LEFT_GRIPPER_DISPATCH") == "1"},
                    "last_dispatch_skew_ms": last_dispatch_skew_ms,
                    "last_owner_dispatch": last_owner_dispatch,
                    "selected_action_source": last_selected_source,
                    "label_valid": last_label_valid,
                    "arm_mapping": f"host_mink_joint_stream_{mapping_suffix}",
                    "camera_status": camera_status.get("status", "UNKNOWN"),
                    "rollout": bounded_rollout.status(),
                    "transition_pose": dict(transition_status),
                    "gripper_control": dict(gripper_control_status),
                    "transition_pose_library": (
                        transition_library.status()
                        if transition_library is not None
                        else {"enabled": False}
                    ),
                    "waist_pitch": "disabled",
                    "waist_yaw": "disabled",
                }
                signature = json.dumps(
                    {
                        key: status_payload[key]
                        for key in (
                            "status",
                            "lifecycle_state",
                            "generation",
                            "takeover",
                            "recording",
                        )
                    },
                    sort_keys=True,
                )
                now_monotonic = time.monotonic()
                if signature != last_status or now_monotonic >= next_status_at:
                    send_json("status", status_payload, latest_source_ns or source_ns)
                    last_status = signature
                    next_status_at = now_monotonic + 1.0
        finally:
            write_trajectory_capture("owner_stopped")
            controller.fail_closed()
            if write_enabled:
                try:
                    final_snapshot = owner.snapshot()
                    hold_current(final_snapshot)
                except Exception as exc:
                    print(
                        "PIPER_X_DUAL_FINAL_HOLD_SKIPPED "
                        + json.dumps(
                            {
                                "reason": str(exc),
                                "dispatch": "already_paused",
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
            print(
                "PIPER_X_DUAL_OWNER "
                + json.dumps(
                    {
                        "status": (
                            "STOPPED_LIVE_TAKEOVER"
                            if write_enabled
                            else "STOPPED_NO_WRITE"
                        ),
                        "observations": observations,
                        "expert_candidates": expert_candidates,
                        "model_candidates": model_candidates,
                        "hardware_write_count": hardware_write_count,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            if receipt_journal is not None:
                receipt_journal.close()
            motion_trace.close()


if __name__ == "__main__":
    main()
