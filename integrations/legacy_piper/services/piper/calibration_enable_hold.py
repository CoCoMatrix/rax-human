"""Explicit supported-operator enable at current pose, in the sole owner.

No startup/reconnect auto-enable. Recovery requests disable + normal feedback;
an explicit request expires within 6 seconds. Targets are preloaded with torque
off; only fresh, stationary feedback can authorize the single enable command.
Stop/disable cancels the transaction, including its remaining tick operations.
"""
import time

from .calibration_control import CalibrationControl
from .calibration_drag import CalibrationDragControl


class CalibrationEnableHoldControl(CalibrationDragControl):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.enable_stage = None
        self.enable_deadline = 0.
        self.enable_sent = False
        self.enable_sent_ns = 0
        self.preload_ns = 0
        self.enable_receipt = None

    def fail_enable(self, reason):
        sent = self.enable_sent
        self.enable_stage = None
        self.enable_sent = False
        self.disabled = True
        self.hold_verified = False
        self.target = None
        self.fault = str(reason)
        self.error = str(reason)
        self.phase = 'DISABLE_UNCONFIRMED'
        if self.enable_receipt:
            self.enable_receipt.update(result='failed',error=str(reason),finished_ns=time.time_ns())
        # Operator must keep supporting throughout the transaction. On an
        # uncertain enable, disable once; never restore torque automatically.
        if sent:
            try:
                self.sdk._canary_arm('left_arm').disable()
                self.writes += 1
            except Exception as exc:
                self.fault += '; 失能未确认：'+str(exc)

    def feedback_failed(self, reason):
        if self.enable_stage:
            if self.enable_stage=='ENABLE_HOLD_WAIT' and time.monotonic()<self.enable_deadline:
                self.history.clear()
                self.stable = False
                self.enable_bits = None
                self.mode = None
                self.phase = self.enable_stage
                self.fault = str(reason)
                return
            self.fail_enable('恢复使能期间反馈失效：'+str(reason))
        else:
            super().feedback_failed(reason)

    def command(self, command, *, timestamp_ns, source, camera_ready):
        if source!='keyboard_ui' or not 0<=time.time_ns()-timestamp_ns<=500_000_000:
            raise ValueError('恢复使能需要新鲜的人工指令')
        if not self.writable:
            raise ValueError('只读会话不能恢复使能')
        if self.enable_stage and command in {'stop','cal_disable_left','cal_restore_feedback'}:
            self.fail_enable('操作者取消恢复使能')
            self.generation += 1
            self.last_attempt = self.last_command = dict(command=command,timestamp_ns=timestamp_ns)
            if command=='stop':return
        if command!='cal_enable_hold':
            return super().command(command,timestamp_ns=timestamp_ns,source=source,camera_ready=camera_ready)
        if self.enable_stage:
            raise ValueError('正在恢复使能，不会重复提交或排队')
        self.last_attempt = dict(command=command,timestamp_ns=timestamp_ns)
        # Unknown feedback is recovered while disabled. A partial/known enabled
        # state is never silently treated as six disabled joints.
        try:
            mode,_=self.controller_feedback()
            bits=self.driver_bits()
        except (ValueError,RuntimeError):
            mode,bits=None,None
        if bits is not None and bits!=[False]*6:
            raise ValueError('六关节并非全部失能；使用普通 Hold 或先承托失能')
        if mode is not None and mode not in (0,1):
            raise ValueError('当前控制模式不适合恢复使能；先恢复普通反馈')
        if bits is None:
            super().command('cal_restore_feedback',timestamp_ns=timestamp_ns,source=source,camera_ready=camera_ready)
        self.generation += 1
        self.disabled = True
        self.target = None
        self.fault = self.error = ''
        self.hold_verified = False
        self.history.clear()
        self.enable_sent = False
        self.enable_stage = self.phase = 'ENABLE_HOLD_WAIT'
        self.enable_deadline = time.monotonic()+6.
        self.last_command = dict(command=command,timestamp_ns=timestamp_ns)
        self.enable_receipt = dict(requested_ns=timestamp_ns,generation=self.generation,result='pending')

    def observe(self, snapshot):
        if not self.enable_stage:
            return super().observe(snapshot)
        self.stable = CalibrationControl.observe(self,snapshot)
        self.phase = self.enable_stage
        try:
            if time.monotonic()>self.enable_deadline:
                raise ValueError('恢复使能超时；请继续承托，不会自动重试')
            mode,motion=self.controller_feedback()
            if self.enable_bits is None:
                raise ValueError(self.enable_error or '驱动状态未知')
            q=list(snapshot.left.joint_position_rad)
            self.cartesian.vector(q)
            self.check_target_guard(snapshot,q,time.time_ns())
            arm=self.sdk._canary_arm('left_arm')
            if self.enable_stage=='ENABLE_HOLD_WAIT':
                if self.enable_bits!=[False]*6 or mode not in (0,1):
                    raise ValueError('恢复期间使能或控制模式发生变化')
                if not self.stable:return self.stable
                self.target=q
                # All six motors are confirmed OFF. Establish mode and measured
                # target before enable; no old Prepare target is used.
                arm.set_speed_percent(1)
                arm.set_motion_mode('j')
                arm.move_j(q)
                self.writes+=3
                self.preload_ns=time.time_ns()
                self.enable_receipt.update(target_joints_rad=q,preloaded_ns=self.preload_ns,
                    measured_timestamp_ns=int(snapshot.left.arm_timestamp_s*1e9))
                self.enable_stage=self.phase='ENABLE_HOLD_PRELOADED'
                return self.stable
            drift=max(abs(a-b) for a,b in zip(q,self.target))
            if self.enable_stage=='ENABLE_HOLD_PRELOADED':
                if self.enable_bits!=[False]*6 or drift>.001 or not self.stable:
                    raise ValueError('预置目标后姿态或使能状态变化；未执行使能，请重新托稳')
                if self.mode_stamp<=self.preload_ns or int(snapshot.left.arm_timestamp_s*1e9)<=self.preload_ns:
                    return self.stable
                if mode!=1 or motion!=1:
                    raise ValueError('普通关节控制模式未确认；不使能')
                # Latch BEFORE SDK call: even a send exception needs disable.
                self.enable_sent=True
                self.enable_sent_ns=time.time_ns()
                self.enable_receipt['enable_requested_ns']=self.enable_sent_ns
                arm.enable()
                self.writes+=1
                self.history.clear()
                self.enable_stage=self.phase='ENABLE_HOLD_CONFIRM'
                return False
            if mode!=1 or motion!=1 or drift>.003:
                raise ValueError('使能后未保持预置当前位置；已请求失能，请继续承托')
            if self.enable_bits==[True]*6 and self.stable and self.mode_stamp>self.enable_sent_ns:
                self.enable_stage=None
                self.enable_sent=False
                self.disabled=False
                self.hold_verified=True
                self.phase='HOLD'
                self.switch_started_ns=self.enable_sent_ns
                self.fault=self.error=''
                self.enable_receipt.update(result='held',confirmed_ns=time.time_ns(),measured_joints_rad=q)
        except Exception as exc:
            self.fail_enable(exc)
        return self.stable

    def status(self):
        value=super().status()
        value.update(enable_hold_supported=True,enable_hold_stage=self.enable_stage,
            enable_hold_receipt=self.enable_receipt,
            enable_hold_requires_operator_support=True)
        return value
