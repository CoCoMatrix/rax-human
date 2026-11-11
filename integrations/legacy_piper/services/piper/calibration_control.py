"""Operator calibration controls, imported ONLY by the formal hardware owner.

No device connections here. A session-latched disable never auto-enables.
The existing dual-arm PASS/HOLD guard remains the last target check.
"""
import math
import time
from .calibration_cartesian import CartesianJog, DIRECTIONS, STEP_MM

MOVE_COMMANDS = {f'cal_move_{direction}': (direction, 2) for direction in DIRECTIONS}
MOVE_COMMANDS.update({f'cal_move_{direction}_{step}mm': (direction, step)
                     for direction in DIRECTIONS for step in STEP_MM})
ROTATION_DEG = (.5, 2, 5, 10)
ROTATE_COMMANDS = {f'cal_jog_{j}_{sign}': (j, 1 if sign=='plus' else -1, .5)
                   for j in range(1,7) for sign in ('plus','minus')}
ROTATE_COMMANDS.update({'cal_wrist_left': (6,1,.5), 'cal_wrist_right': (6,-1,.5)})
ROTATE_COMMANDS.update({f'{command}_{step:g}deg': (joint,sign,step)
                       for command,(joint,sign,_) in list(ROTATE_COMMANDS.items())
                       for step in ROTATION_DEG})

COMMANDS = frozenset({'stop', 'cal_disable_left', 'cal_drag_left', 'cal_restore_feedback', 'cal_enable_hold'} | set(MOVE_COMMANDS) | set(ROTATE_COMMANDS))


class CalibrationControl:
    def __init__(self, sdk, owner, guard, action_dict, *, writable, manual_only=False):
        self.sdk, self.owner, self.guard, self.action_dict = sdk, owner, guard, action_dict
        self.writable = writable
        self.manual_only = manual_only
        self.disabled = False
        self.generation = 0
        self.phase = 'MANUAL_WAIT_DISABLE' if manual_only else 'WAITING_HOLD'
        self.last_command = None
        self.last_attempt = None
        self.target = None
        self.history = []
        self.next_jog_at = 0.
        self.enable_bits = None
        self.enable_error = ''
        self.writes = 0
        self.error = ''
        self.cartesian = CartesianJog()
        self.last_plan = None

    def driver_bits(self):
        arm = self.sdk._connected_arm('left_arm')
        result = []
        for index in range(1, 7):
            row = arm.get_driver_states(index)
            age = float('inf') if row is None else time.time() - float(row.timestamp)
            if not math.isfinite(age) or not 0 <= age <= .1:
                raise ValueError('左臂驱动反馈过期；使能状态未知')
            result.append(bool(row.msg.foc_status.driver_enable_status))
        return result

    def observe(self, snapshot):
        now = time.monotonic()
        q = list(snapshot.left.joint_position_rad)
        self.history.append((now, q))
        self.history = [(t, v) for t, v in self.history if now-t <= 1.]
        try:
            self.enable_bits = self.driver_bits()
            self.enable_error = ''
        except (ValueError, RuntimeError) as exc:
            self.enable_bits = None
            self.enable_error = str(exc)
        stable = (len(self.history) >= 10 and now-self.history[0][0] >= .7
                  and max(max(v[j] for _, v in self.history)-min(v[j] for _, v in self.history)
                          for j in range(6)) <= .001)
        if self.manual_only and self.enable_bits == [False]*6:
            self.disabled = True
        if self.disabled:
            self.phase = 'DISABLED' if self.enable_bits == [False]*6 else 'DISABLE_UNCONFIRMED'
        elif self.target is not None and self.phase in {'MOVING', 'SETTLING'}:
            if stable and max(abs(a-b) for a, b in zip(q, self.target)) <= .003:
                self.phase = 'HOLD'
        return stable

    def validate_target(self, snapshot, target, timestamp_ns):
        if self.manual_only or self.disabled or not self.writable:
            raise ValueError('失能锁存或只读模式：禁止位置指令')
        if self.driver_bits() != [True]*6:
            raise ValueError('左臂六个关节未全部使能；不会自动使能')
        self.check_target_guard(snapshot, target, timestamp_ns)

    def check_target_guard(self, snapshot, target, timestamp_ns):
        """Pose/age/generation preflight; caller separately checks torque state."""
        measured = self.action_dict(snapshot.state_14d)
        desired = dict(measured)
        for j, q in enumerate(target, 1):
            desired[f'left_joint{j}'] = q
        stamp = int(min(snapshot.left.arm_timestamp_s, snapshot.left.gripper_timestamp_s,
                        snapshot.right.arm_timestamp_s, snapshot.right.gripper_timestamp_s)*1e9)
        decision = self.guard.evaluate(desired, measured, enabled=True,
            write_authorized=self.writable, both_arms_ready=True,
            generation=self.generation, expected_generation=self.generation,
            state_timestamp_ns=stamp, command_timestamp_ns=timestamp_ns, now_ns=time.time_ns())
        if not decision.accepted:
            raise ValueError('PASS/HOLD 拒绝：' + ', '.join(decision.reasons))

    def send_target(self, snapshot, target, timestamp_ns):
        self.validate_target(snapshot, target, timestamp_ns)
        # Existing SDK canary path writes the left arm only, unlike the dual
        # live helper which also configures speed/mode on the other arm.
        self.sdk.move_arm_for_canary('left_arm', target, speed_percent=1)
        self.writes += 1
        self.target = list(target)
        self.phase = 'SETTLING'
        self.history.clear()

    def command(self, command, *, timestamp_ns, source, camera_ready):
        if command not in COMMANDS or source != 'keyboard_ui':
            raise ValueError('只允许标定 Web 的人工指令')
        age = time.time_ns()-timestamp_ns
        if not 0 <= age <= 500_000_000:
            raise ValueError('指令已过期；不会重试')
        if not self.writable:
            raise ValueError('当前是只读模式')
        if command in {'cal_drag_left','cal_restore_feedback','cal_enable_hold'}:
            raise ValueError('当前 owner 未接入拖动切换；需要拖动采集会话')
        self.last_attempt = {'command':command,'timestamp_ns':timestamp_ns}
        if self.manual_only and command not in {'stop','cal_disable_left'}:
            raise ValueError('人工摆姿态模式不发送位置目标')
        if command == 'cal_disable_left':
            # Latch BEFORE SDK dispatch; ambiguous failure must not permit
            # later ticks, Hold, or shutdown to send another target.
            self.disabled = True
            self.target = None
            self.phase = 'DISABLE_UNCONFIRMED'
            self.generation += 1
            self.last_command = {'command': command, 'timestamp_ns': timestamp_ns}
            self.sdk._canary_arm('left_arm').disable()
            self.writes += 1
            return
        if command == 'stop':
            self.generation += 1
            self.next_jog_at = time.monotonic()+2.
            self.last_command = {'command': command, 'timestamp_ns': timestamp_ns}
            if self.disabled or self.manual_only:
                return  # HOLD is never enable, reset, or torque restoration.
            self.phase = 'HOLD_UNCONFIRMED'
            self.target = None
            snapshot = self.owner.snapshot(now_s=time.time())
            self.send_target(snapshot, snapshot.left.joint_position_rad, time.time_ns())
            return
        if self.disabled or not camera_ready or self.phase != 'HOLD':
            raise ValueError('需相机就绪、左臂使能且 Hold 停稳')
        if time.monotonic() < self.next_jog_at:
            raise ValueError('等待当前小步完成；不排队连续运动')
        snapshot = self.owner.snapshot(now_s=time.time())
        if self.target is None or max(abs(a-b) for a,b in zip(snapshot.left.joint_position_rad,self.target)) > .003:
            raise ValueError('实测姿态已变化；先重新 Hold')
        if command.startswith('cal_move_'):
            direction,step_mm=MOVE_COMMANDS[command]
            target,plan = self.cartesian.solve(snapshot.left.joint_position_rad, direction, step_mm)
            # IK runs without hardware writes. Re-read CAN after solving;
            # dispatch from a fresh snapshot only if the arm has not drifted.
            fresh=self.owner.snapshot(now_s=time.time())
            if max(abs(a-b) for a,b in zip(fresh.left.joint_position_rad,snapshot.left.joint_position_rad))>.0005:
                raise ValueError('求解期间机械臂姿态变化；先重新 Hold')
            snapshot=fresh
            self.last_plan=plan
        else:
            joint,sign,step_deg=ROTATE_COMMANDS[command]
            target = list(snapshot.left.joint_position_rad)
            target[joint-1] += math.radians(step_deg)*sign
            self.cartesian.vector(target)  # intersect nominal and owner limits (including J6)
            self.last_plan=dict(joint=joint,delta_deg=step_deg*sign)
        self.generation += 1
        self.phase = 'HOLD_UNCONFIRMED'
        self.last_command = {'command': command, 'timestamp_ns': timestamp_ns}
        self.send_target(snapshot, target, timestamp_ns)
        self.next_jog_at = time.monotonic()+2.

    def status(self):
        return dict(schema='rax.piper.calibration_control.v1', phase=self.phase,
            generation=self.generation, disable_latched=self.disabled,
            manual_only=self.manual_only,
            left_enable_bits=self.enable_bits, enable_error=self.enable_error,
            last_command=self.last_command, error=self.error, left_only=True,
            last_attempt=self.last_attempt,
            joint_step_deg=.5, speed_percent=1, automatic_reenable=False,
            translation_step_m=.002, direction_frame='left_base_link',
            translation_steps_mm=list(STEP_MM),
            translation_orientation_modes=['fixed'],
            rotation_steps_deg=list(ROTATION_DEG),
            cartesian_jog_supported=True, last_plan=self.last_plan,
            physical_stop_verified=False, writes=self.writes)
