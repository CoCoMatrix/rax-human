"""Explicit, one-shot closed-gripper calibration inside the formal owner."""
import math
import time

from .inventory import InventoryError


def calibrate_closed_gripper(session, role):
    # The operator must have confirmed empty, fully closed jaws. No position
    # target, arm enable, joint calibration, or other-gripper command is issued.
    if role not in {"left_arm", "right_arm"} or not session._supervised_write_authorized:
        raise InventoryError("closed-gripper calibration requires explicit write authorization and one arm")
    if session._live_control_enabled or session._enabled_canary_arms:
        raise InventoryError("closed-gripper calibration requires a maintenance session")
    gripper = session._grippers[role]

    def feedback():
        value = gripper.get_gripper_status()
        if value is None:
            raise InventoryError("missing gripper feedback")
        age = time.time() - float(value.timestamp)
        width = float(value.msg.value)
        if not math.isfinite(age) or not 0 <= age <= .1 or not math.isfinite(width):
            raise InventoryError("stale or nonfinite gripper feedback")
        if value.msg.mode != "width":
            raise InventoryError("gripper calibration requires width mode")
        return value

    before = feedback()
    initial_width = float(before.msg.value)
    initial_timestamp = float(before.timestamp)
    gripper.disable_gripper()
    deadline = time.monotonic() + 2
    while True:
        disabled = feedback()
        if (float(disabled.timestamp) > initial_timestamp
                and not disabled.msg.foc_status.driver_enable_status):
            break
        if time.monotonic() >= deadline:
            raise InventoryError("gripper disable feedback not confirmed; zero not changed")
        time.sleep(.02)
    # Ensure disabling did not let the operator-confirmed closed jaws move.
    baseline = float(disabled.msg.value)
    if abs(baseline - initial_width) > .001:
        raise InventoryError("gripper moved while disabling; zero not changed")
    time.sleep(.2)
    stable = feedback()
    if stable.msg.foc_status.driver_enable_status or abs(float(stable.msg.value)-baseline) > .001:
        raise InventoryError("gripper not stable and disabled; zero not changed")
    sent_at = time.time()
    if not gripper.calibrate_gripper(timeout=1.0):
        raise InventoryError("vendor zero acknowledgement absent; no automatic retry")
    deadline = time.monotonic() + 2
    while True:
        after = feedback()
        if float(after.timestamp) > sent_at and abs(float(after.msg.value)) <= .001:
            return {"role": role, "before_width_m": initial_width,
                    "after_width_m": float(after.msg.value), "vendor_ack": True,
                    "calibration_api_calls": 1, "arm_enable_calls": 0,
                    "motion_targets": 0, "other_gripper_commands": 0}
        if time.monotonic() >= deadline:
            raise InventoryError("zero acknowledged but fresh zero feedback not confirmed")
        time.sleep(.02)
