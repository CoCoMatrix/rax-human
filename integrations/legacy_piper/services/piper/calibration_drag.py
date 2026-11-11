"""Supervised left drag/Hold state machine inside the sole formal owner.

Constructing/observing this controller never writes or enables hardware. Only
explicit operator commands can change modes. A disable is session-latched.
Vendor mode changes are not atomic: physical handoff remains unqualified until
the operator performs a supported, supervised drag/Hold canary.
"""
import math
import time

from .calibration_control import CalibrationControl


class CalibrationDragControl(CalibrationControl):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.mode = None
        self.mode_stamp = 0
        self.switch_started_ns = 0
        self.switch_deadline = 0.
        self.transition = None
        self.fault = ''
        self.stable = False
        self.hold_verified = False

    def controller_feedback(self):
        row = self.sdk._connected_arm('left_arm').get_arm_status()
        if row is None:
            raise ValueError('左臂控制模式反馈缺失')
        age = time.time() - float(row.timestamp)
        if not math.isfinite(age) or not 0 <= age <= .1:
            raise ValueError('左臂控制模式反馈过期')
        if int(row.msg.arm_status) != 0:
            raise ValueError('左臂控制器异常；请继续承托并检查状态')
        self.mode = int(row.msg.ctrl_mode)
        self.mode_stamp = int(row.timestamp * 1e9)
        return self.mode, int(row.msg.mode_feedback)

    def feedback_failed(self, reason):
        self.history.clear()
        self.stable = self.hold_verified = False
        self.mode = None
        self.enable_bits = None
        self.fault = str(reason)
        self.phase = 'DISABLE_UNCONFIRMED' if self.disabled else 'DRAG_FAULT'

    def observe(self, snapshot):
        previous = self.phase
        self.stable = super().observe(snapshot)
        try:
            mode, motion = self.controller_feedback()
            if self.enable_bits == [False]*6:
                self.disabled = True
            if self.disabled:
                self.hold_verified = False
                if self.enable_bits == [False]*6:
                    self.phase = 'DISABLED'
                    self.fault = ''
                return self.stable
            if self.enable_bits != [True]*6:
                raise ValueError('六关节使能未全部确认；不会自动使能')
            if self.fault:
                self.phase = 'DRAG_FAULT'
                return self.stable
            if previous == 'DRAG_ENTERING':
                self.phase = 'DRAG_ENTERING'
                if mode == 6 and self.mode_stamp > self.switch_started_ns:
                    self.phase = 'DRAGGING'
                    self.transition['confirmed_ns'] = time.time_ns()
            elif previous == 'DRAGGING':
                if mode != 6:
                    raise ValueError('拖动期间控制模式变化；请继续承托')
                self.phase = 'DRAGGING'
            elif previous in {'SETTLING', 'HOLD'}:
                arrived = (self.target is not None and
                    max(abs(a-b) for a,b in zip(snapshot.left.joint_position_rad,self.target)) <= .003)
                confirmed = mode == 1 and motion == 1 and self.mode_stamp > self.switch_started_ns
                self.hold_verified = bool(confirmed and arrived and self.stable)
                self.phase = 'HOLD' if self.hold_verified else 'SETTLING'
                if self.hold_verified and self.transition:
                    self.transition.setdefault('confirmed_ns', time.time_ns())
                if previous == 'HOLD' and not self.hold_verified:
                    raise ValueError('原位保持状态已变化；重新托稳并点击 Hold')
            if self.phase in {'DRAG_ENTERING', 'SETTLING'} and time.monotonic() > self.switch_deadline:
                raise ValueError('模式切换或停稳超时；未确认成功，不会自动重试')
        except (ValueError, RuntimeError, TypeError, AttributeError) as exc:
            self.feedback_failed(exc)
        return self.stable

    def command(self, command, *, timestamp_ns, source, camera_ready):
        if command not in {'stop', 'cal_disable_left', 'cal_drag_left', 'cal_restore_feedback'}:
            raise ValueError('拖动采集会话只允许拖动、Hold 和失能')
        if source != 'keyboard_ui' or not 0 <= time.time_ns()-timestamp_ns <= 500_000_000:
            raise ValueError('需新鲜的本地人工指令；不会自动重试')
        if not self.writable:
            raise ValueError('当前是只读模式')
        self.last_attempt = dict(command=command, timestamp_ns=timestamp_ns)
        if command == 'cal_restore_feedback':
            # Supported operator recovery only: never select a pose or enable.
            # Disable is latched before either write, even if CAN delivery fails.
            self.disabled = True
            self.target = None
            self.hold_verified = False
            self.generation += 1
            self.phase = 'DISABLE_UNCONFIRMED'
            self.last_command = dict(command=command,timestamp_ns=timestamp_ns)
            arm = self.sdk._canary_arm('left_arm')
            arm.disable()
            self.writes += 1
            arm.set_follower_mode()
            self.writes += 1
            return
        if command == 'cal_disable_left':
            self.hold_verified = False
            return super().command(command, timestamp_ns=timestamp_ns, source=source, camera_ready=camera_ready)
        if self.disabled:
            raise ValueError('失能已锁存；Hold 不会恢复使能')
        mode, motion = self.controller_feedback()
        if self.driver_bits() != [True]*6:
            raise ValueError('六关节未全部使能；不会自动使能')
        snapshot = self.owner.snapshot(now_s=time.time())
        target = list(snapshot.left.joint_position_rad)
        # Validate measured pose BEFORE any mode/config write. This is also the
        # final PASS/HOLD guard for the only position target: current feedback.
        self.cartesian.vector(target)
        self.validate_target(snapshot, target, timestamp_ns)
        if command == 'cal_drag_left' and (self.phase != 'HOLD' or not self.hold_verified
                or self.fault or mode != 1 or motion != 1 or not camera_ready):
            raise ValueError('先 Hold 锁定并停稳，确认相机就绪，再进入拖动')
        if command == 'cal_drag_left':
            raise ValueError('此固件的 Leader 拖动已实测丢失 CAN 反馈，暂时禁用，等待修复')
        if command == 'stop' and mode not in (1, 6):
            raise ValueError('控制模式未知，不能自动切回位置控制；请继续承托')
        if command == 'stop' and mode == 6 and not self.stable:
            raise ValueError('请先托稳约 1 秒，再点击 Hold 锁定当前位置')
        arm = self.sdk._canary_arm('left_arm')
        self.generation += 1
        self.target = None
        self.hold_verified = False
        self.fault = ''
        self.history.clear()
        self.switch_started_ns = time.time_ns()
        self.switch_deadline = time.monotonic() + 4.
        self.last_command = dict(command=command, timestamp_ns=timestamp_ns)
        self.transition = dict(command=command, requested_ns=timestamp_ns,
            started_ns=self.switch_started_ns, from_mode=mode, measured_joints_rad=target,
            measured_timestamp_ns=int(snapshot.left.arm_timestamp_s*1e9), generation=self.generation)
        try:
            if command == 'cal_drag_left':
                self.phase = 'DRAG_ENTERING'
                # Installation was already configured by the formal Prepare
                # owner. Do not assume/rewrite payload or motor tuning here.
                arm.set_leader_mode()
                self.writes += 1
            else:
                self.phase = 'HOLD_UNCONFIRMED'
                # Seed the measured target BEFORE restoring CAN position mode.
                # Suppress move_j's automatic mode write for that preload only.
                # This ordered vendor sequence is NOT a physical atomicity claim.
                automatic = arm.get_auto_set_motion_mode_enabled()
                arm.set_auto_set_motion_mode_enabled(False)
                try:
                    arm.move_j(target)
                    self.writes += 1
                finally:
                    arm.set_auto_set_motion_mode_enabled(automatic)
                if mode == 6:
                    arm.set_follower_mode()
                    self.writes += 1
                arm.set_speed_percent(1)
                arm.set_motion_mode('j')
                arm.move_j(target)
                self.writes += 3
                self.target = target
                self.phase = 'SETTLING'
        except Exception as exc:
            self.feedback_failed('切换未确认：'+str(exc))
            raise

    def status(self):
        value = super().status()
        value.update(drag_supported=False, feedback_recovery_supported=True, cartesian_jog_supported=False,
            workflow='drag_hold_capture', controller_mode=self.mode,
            controller_mode_timestamp_ns=self.mode_stamp, drag_error=self.fault,
            hold_verified=self.hold_verified, stationary=self.stable,
            capture_ready=self.phase == 'HOLD' and self.hold_verified and not self.fault,
            transition=self.transition, physical_handoff_validated=False)
        return value
