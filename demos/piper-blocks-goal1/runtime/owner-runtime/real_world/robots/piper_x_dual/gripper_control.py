"""Validate manual gripper requests; only the existing owner dispatches them."""
from __future__ import annotations

from typing import Any

from .mapping import action_dict


def gripper_open_action(
    snapshot: Any, *, profile: dict, safety_guard: Any,
    write_enabled: bool, camera_ready: bool, startup_feedback_error: str | None,
    lifecycle_state: str, boundary_active: bool, takeover: bool, recording: bool,
    operator_source: str, generation: int, command_timestamp_ns: int, now_ns: int,
    gripper_scope: str = "both",
) -> dict[str, float]:
    """Return a complete, PASS-checked target without sending any SDK command."""
    if gripper_scope not in {"both", "left"}:
        raise ValueError("unsupported manual gripper scope")
    if operator_source != "keyboard_ui":
        raise ValueError("manual grippers require the operator Web")
    if not write_enabled or not profile['live_takeover']['gripper_write_enabled']:
        raise ValueError("gripper writes are disabled")
    if not camera_ready or startup_feedback_error:
        raise ValueError("robot is not ready; restore feedback and restart if required")
    if lifecycle_state not in {"idle", "prepared"} or boundary_active or takeover or recording:
        raise ValueError("Hold the robot before opening the grippers")
    if profile['prepare']['initial_pose']['left_gripper'] != 0.0 or profile['prepare']['initial_pose']['right_gripper'] != 0.0:
        raise ValueError("manual opening requires a closed-gripper Prepare pose")
    measured = action_dict(snapshot.state_14d)
    target = dict(measured)
    names = ("left_gripper",) if gripper_scope == "left" else ("left_gripper", "right_gripper")
    for name in names:
        target[name] = float(profile['body']['grippers'][name]['limits'][1])
    state_timestamp_ns = int(min(
        snapshot.left.arm_timestamp_s, snapshot.left.gripper_timestamp_s,
        snapshot.right.arm_timestamp_s, snapshot.right.gripper_timestamp_s,
    ) * 1e9)
    decision = safety_guard.evaluate(
        target, measured, enabled=True, write_authorized=write_enabled,
        both_arms_ready=True, generation=generation, expected_generation=generation,
        state_timestamp_ns=state_timestamp_ns, command_timestamp_ns=command_timestamp_ns,
        now_ns=now_ns,
    )
    if not decision.accepted or decision.action is None:
        raise ValueError("gripper request held: " + ", ".join(decision.reasons))
    return decision.action
