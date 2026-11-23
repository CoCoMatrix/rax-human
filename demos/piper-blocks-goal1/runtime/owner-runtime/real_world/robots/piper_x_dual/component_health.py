"""Per-device diagnostics using only the formal owner's existing SDK cache."""
from __future__ import annotations

import math
import time


FAULTS = {
    "voltage_too_low": "电压过低", "motor_overheating": "电机过热",
    "driver_overcurrent": "驱动过流", "driver_overheating": "驱动过热",
    "collision_status": "碰撞状态", "driver_error_status": "驱动故障",
    "stall_status": "堵转",
}


def _fresh(message, now):
    if message is None:
        raise ValueError("未收到反馈：检查电源、急停和连接线")
    age = now - float(message.timestamp)
    if not math.isfinite(age) or not -.02 <= age <= .2:
        raise ValueError("反馈已过期：检查电源和连接线")
    return message.msg


def _faults(flags):
    return [label for name, label in FAULTS.items() if getattr(flags, name, False)]


def component_health(session, *, write_enabled, gripper_request=None, now=None):
    """No connections or commands. Failure on one device preserves the others."""
    now = time.time() if now is None else now
    result = {}
    request = gripper_request or {}
    for side in ("left", "right"):
        role = side + "_arm"
        row = {"ok": False, "message": "等待反馈"}
        result[role] = row
        try:
            arm = session._arms[role]
            status = _fresh(arm.get_arm_status(), now)
            mode = getattr(status.ctrl_mode, "name", str(status.ctrl_mode))
            state = getattr(status.arm_status, "name", str(status.arm_status))
            errors = []
            if state != "NORMAL":
                errors.append("控制器状态：" + state)
            for joint in range(1, 7):
                flags = getattr(status, "err_status", None)
                if getattr(flags, f"joint_{joint}_angle_limit", False):
                    errors.append(f"J{joint} 角度超限")
                if getattr(flags, f"communication_status_joint_{joint}", False):
                    errors.append(f"J{joint} 通信异常")
            disabled = []
            for joint in range(1, 7):
                driver = _fresh(arm.get_driver_states(joint), now)
                flags = driver.foc_status
                errors.extend(f"J{joint} {fault}" for fault in _faults(flags))
                if not flags.driver_enable_status:
                    disabled.append(joint)
            row.update(controller_mode=mode, disabled_joints=disabled)
            if disabled and write_enabled:
                errors.append("未使能关节：" + ", ".join(f"J{n}" for n in disabled))
            row.update(ok=not errors, message="；".join(errors) or (
                "反馈正常 · 只读，关节未使能" if disabled else "反馈正常 · 关节已使能"))
        except Exception as exc:
            row["message"] = str(exc)
        row = {"ok": False, "message": "等待夹爪反馈"}
        result[side + "_gripper"] = row
        try:
            status = _fresh(session._grippers[role].get_gripper_status(), now)
            width = float(status.value)
            if status.mode != "width" or not math.isfinite(width):
                raise ValueError("夹爪反馈模式或开度无效")
            flags = status.foc_status
            errors = _faults(flags)
            enabled = bool(flags.driver_enable_status)
            row.update(width_m=width, enabled=enabled)
            if not enabled:
                errors.append("夹爪未使能")
            target = request.get(side + "_target_m")
            if request.get("accepted") and target is not None:
                reached = abs(width - float(target)) <= .003
                row.update(target_m=target, reached=reached)
                if not reached and now - request.get("requested_timestamp_ns", 0) / 1e9 > 5:
                    errors.append(f"张开未到位：目标 {float(target)*1000:.1f} mm，实际 {width*1000:.1f} mm")
                elif not reached:
                    errors.append("正在等待张开到位")
            row.update(ok=not errors, message="；".join(errors) or "反馈正常")
        except Exception as exc:
            row["message"] = str(exc)
    return result
