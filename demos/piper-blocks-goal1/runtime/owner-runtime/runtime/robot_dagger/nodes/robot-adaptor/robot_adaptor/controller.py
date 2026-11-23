from typing import Protocol

import numpy as np


class Controller(Protocol):
    def update(self, target: float, current: float, dt: float) -> float: ...


class PDController:
    def __init__(
        self,
        kp: float,
        kd: float,
        deadzone: float = 0.0,
        scale: float = 1.0,
        target_offset: int | None = None,
        current_offset: int | None = None,
    ):
        self.kp = float(kp)
        self.kd = float(kd)
        self.deadzone = float(deadzone)
        self._prev_error = None  # for finite-difference derivative
        self.current_offset = current_offset
        self.target_offset = target_offset
        self.scale = scale

    @staticmethod
    def _apply_deadzone(e: float, dz: float) -> float:
        return e if abs(e) > dz else 0.0

    def reset(self):
        self._prev_error = None

    def update(
        self,
        target: float,
        current: float,
        dt: float,
        *,
        target_rate: float | None = None,  # optional d(target)/dt
        current_rate: float | None = None,  # optional d(current)/dt (measured velocity)
    ) -> float:
        if not self.current_offset:
            self.current_offset = current
        if not self.target_offset:
            self.target_offset = target

        target -= self.target_offset
        current -= self.current_offset

        # proportional error with deadzone
        error = self._apply_deadzone(target * self.scale - current, self.deadzone)

        # derivative of error: prefer measured rates if provided
        if (target_rate is not None) or (current_rate is not None):
            dr_target = target_rate or 0.0
            dr_current = current_rate or 0.0
            de_dt = dr_target - dr_current
        else:
            if dt <= 0.0 or self._prev_error is None:
                de_dt = 0.0
            else:
                de_dt = (error - self._prev_error) / dt

        u = self.kp * error + self.kd * de_dt
        self._prev_error = error
        return u


class ConstantController:
    def __init__(self, velocity: float, deadzone: float = 0.0, lower: float = 0.0, upper: float = 0.0):
        self.velocity = velocity
        self.deadzone = deadzone
        self._cmd = None
        self.lower = lower
        self.upper = upper

    @property
    def cmd(self):
        return self._cmd

    @cmd.setter
    def cmd(self, value: float):
        self._cmd = np.clip(value, self.lower, self.upper)

    def reset(self, current: float):
        self.cmd = current

    def update(self, target: float, current: float, dt: float) -> float:
        if not self.cmd:
            self.cmd = current
        if abs(target - current) < self.deadzone:
            return self.cmd
        vel = float(np.sign(target - current) * self.velocity * dt)
        self.cmd += vel

        # print(f"{target=} {current=}  {self.cmd=}")
        return self.cmd


class RobotBasePoseController:
    def __init__(self, frequency: float, limits: dict[str, list[float]]):
        self.frequency = frequency
        self.controllers: dict[str, Controller | None] = {
            "base_height": PDController(1.2, 0.1, deadzone=0.01, target_offset=0.0),
            "base_pitch": PDController(0.3, 0.1, scale=1.0, deadzone=np.deg2rad(10), target_offset=0.0),
            "base_yaw": None,
            "vel_x": PDController(0.5, 0.1, deadzone=0.5),
            "vel_y": PDController(0.5, 0.1, deadzone=0.3),
            "vel_yaw": PDController(0.5, 0.1, deadzone=np.deg2rad(15)),
        }

    def update(self, name: str, target: float, current: float, dt: float) -> float:
        # print(f"{name}: {target=}; {current=}")
        if name not in self.controllers:
            raise ValueError(f"Unknown controller: {name}")
        if self.controllers[name] is None:
            return 0.0
        return self.controllers[name].update(target, current, dt)
