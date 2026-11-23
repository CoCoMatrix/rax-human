from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from XPolicyLab.real_world.contracts import ActionChunk, ObservationFrame
from XPolicyLab.real_world.inference import PendingChunk
from XPolicyLab.real_world.robots.astribot_s1.camera import (
    AstribotS1CameraSource,
    AstribotS1ObservationAdapter,
    CameraSyncDecision,
)
from XPolicyLab.real_world.robots.astribot_s1.driver import (
    AstribotS1FullBodyDriver,
    DriverDecision,
)
from XPolicyLab.real_world.robots.astribot_s1.mapping import (
    ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
    current_full_body_hold_action,
    full_body_action_vector,
)
from XPolicyLab.real_world.robots.astribot_s1.runtime import (
    AstribotS1FullBodyRuntime,
    AstribotS1Runtime,
    FullBodyRuntimeDecision,
    RuntimeDecision,
)
from XPolicyLab.real_world.robots.astribot_s1.sdk import (
    AstribotS1SdkSession,
    RobotSnapshot,
)


@dataclass(frozen=True, slots=True)
class OwnerTickDecision:
    """One owner-loop result spanning observation and action safety."""

    status: str
    reasons: tuple[str, ...]
    snapshot: RobotSnapshot
    camera_sync: CameraSyncDecision
    action: RuntimeDecision | FullBodyRuntimeDecision
    observation: ObservationFrame | None = None

    @property
    def wrote(self) -> bool:
        return self.action.wrote


def build_hold_action_chunk(
    snapshot: RobotSnapshot,
    *,
    steps: int,
    frequency_hz: float,
    generation: int,
) -> ActionChunk:
    """Build a finite 25D zero-base-velocity hold from measured state."""
    if steps < 1:
        raise ValueError("dummy hold chunk must contain at least one step")
    action = current_full_body_hold_action(snapshot.joints)
    row = full_body_action_vector(action)
    return ActionChunk(
        schema=ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA,
        values=np.repeat(row[None, :], int(steps), axis=0),
        frequency_hz=float(frequency_hz),
        generation=int(generation),
        metadata={"source": "dummy_full_body_hold_current"},
    )


class AstribotS1OwnerRuntime:
    """Single-session owner for synchronized observations and safe actions."""

    def __init__(
        self,
        session: AstribotS1SdkSession,
        camera_source: AstribotS1CameraSource,
        action_runtime: AstribotS1Runtime | AstribotS1FullBodyRuntime,
        *,
        observation_frequency_hz: float = 30.0,
        robot_id: str = "astribot-s1",
        instruction: str = "",
    ) -> None:
        observation_frequency_hz = float(observation_frequency_hz)
        if not math.isfinite(observation_frequency_hz) or observation_frequency_hz <= 0:
            raise ValueError("observation_frequency_hz must be positive")
        if camera_source.session is not session:
            raise ValueError("camera source must use the owner SDK session")
        if action_runtime.driver.session is not session:
            raise ValueError("action runtime must use the owner SDK session")
        self.session = session
        self.camera_source = camera_source
        self.observation_adapter = AstribotS1ObservationAdapter(camera_source)
        self.action_runtime = action_runtime
        self.observation_frequency_hz = observation_frequency_hz
        self.observation_period_ns = max(
            1, round(1_000_000_000 / observation_frequency_hz)
        )
        self.robot_id = str(robot_id)
        self.instruction = str(instruction)
        self.sequence = 0
        self._next_observation_ns = 0
        self._last_camera_sync = CameraSyncDecision(
            "HOLD",
            ("camera_source_not_started",),
            {},
            0,
        )
        self._started = False

    @classmethod
    def from_profile(
        cls,
        path: str | Path,
        session: AstribotS1SdkSession,
        *,
        enable_write: bool = False,
        robot_id: str = "astribot-s1",
        instruction: str = "",
    ) -> AstribotS1OwnerRuntime:
        profile = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        driver = AstribotS1FullBodyDriver(
            session,
            enabled=enable_write,
            control_way=profile["sdk"]["control_way"],
            torso_control_way=profile["torso_teleop"]["torso_control_way"],
            # Full25 uses the existing joint-space SDK path; WBC is only a
            # Cartesian-driver option and must not alter this owner contract.
            use_wbc=False,
            include_grippers=True,
        )
        return cls(
            session,
            AstribotS1CameraSource.from_profile(path, session),
            AstribotS1FullBodyRuntime.from_profile(path, driver),
            observation_frequency_hz=profile["owner"]["observation_frequency_hz"],
            robot_id=robot_id,
            instruction=instruction,
        )

    def start(self, *, activate_cameras: bool = False) -> dict[str, Any]:
        if self._started:
            raise RuntimeError("Astribot owner runtime is already started")
        camera_info = self.camera_source.start(activate=activate_cameras)
        self._started = True
        return camera_info

    def close(self) -> None:
        self.camera_source.close()
        self._started = False

    def control(self, command: str, generation: int) -> DriverDecision | None:
        if command == "stop":
            self.action_runtime.control("pause", generation)
            return self.action_runtime.driver.stop_on_fault()
        self.action_runtime.control(command, generation)
        return None

    def begin_inference(self) -> PendingChunk:
        return self.action_runtime.begin_inference()

    def accept(self, chunk: ActionChunk, pending: PendingChunk) -> int | None:
        return self.action_runtime.accept(chunk, pending)

    def fail_inference(self, pending: PendingChunk) -> bool:
        return self.action_runtime.fail_inference(pending)

    @property
    def should_infer(self) -> bool:
        return self.action_runtime.should_infer

    @property
    def camera_ready(self) -> bool:
        return self._last_camera_sync.ready

    def _observation_due(self, now_ns: int) -> bool:
        if self._next_observation_ns and now_ns < self._next_observation_ns:
            return False
        if not self._next_observation_ns:
            self._next_observation_ns = now_ns
        while self._next_observation_ns <= now_ns:
            self._next_observation_ns += self.observation_period_ns
        return True

    @staticmethod
    def _unique_reasons(*groups: tuple[str, ...]) -> tuple[str, ...]:
        return tuple(dict.fromkeys(reason for group in groups for reason in group))

    def tick(
        self,
        *,
        operator_enabled: bool,
        now_ns: int | None = None,
        snapshot: RobotSnapshot | None = None,
    ) -> OwnerTickDecision:
        if not self._started:
            raise RuntimeError("Astribot owner runtime is not started")
        snapshot = self.session.snapshot() if snapshot is None else snapshot
        now_ns = time.time_ns() if now_ns is None else int(now_ns)

        observation = None
        if self._observation_due(now_ns):
            observation, self._last_camera_sync = self.observation_adapter.build(
                snapshot,
                self.instruction,
                sequence=self.sequence,
                robot_id=self.robot_id,
                now_ns=now_ns,
            )
            if observation is not None:
                self.sequence += 1

        camera_ready = self._last_camera_sync.ready
        action = self.action_runtime.tick(
            snapshot,
            self.session.joint_limits,
            operator_enabled=bool(operator_enabled and camera_ready),
            now_ns=now_ns,
        )

        if action.driver.status == "FAULT" or (
            action.stop is not None and action.stop.status == "FAULT"
        ):
            status = "FAULT"
        elif not camera_ready:
            status = "HOLD"
        elif action.wrote:
            status = "SENT"
        elif action.driver.reasons == ("hardware_write_disabled",):
            status = "SHADOW"
        elif not self.action_runtime.enabled:
            status = "PAUSED"
        elif action.arm_safety.reasons == ("action_queue_empty",):
            status = "STARVED"
        else:
            status = "HOLD"

        camera_reasons = () if camera_ready else self._last_camera_sync.reasons
        action_reasons = self._unique_reasons(
            getattr(action, "chassis_safety", action.arm_safety).reasons,
            getattr(action, "auxiliary_safety", action.arm_safety).reasons,
            action.arm_safety.reasons,
            action.gripper_safety.reasons,
            action.driver.reasons,
            action.stop.reasons if action.stop is not None else (),
        )
        return OwnerTickDecision(
            status=status,
            reasons=self._unique_reasons(camera_reasons, action_reasons),
            snapshot=snapshot,
            camera_sync=self._last_camera_sync,
            action=action,
            observation=observation,
        )
