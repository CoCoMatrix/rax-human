from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Mapping, Protocol


ARM_ORDER = ("left_arm", "right_arm")
JOINT_COUNT = 6
GRIPPER_LIMITS_M = (0.0, 0.1)


class InventoryError(RuntimeError):
    """Read-only dual-arm state cannot satisfy the inventory contract."""


@dataclass(frozen=True)
class ArmSample:
    role: str
    joint_position_rad: tuple[float, ...]
    gripper_width_m: float
    arm_timestamp_s: float
    gripper_timestamp_s: float
    arm_status_hz: float
    gripper_status_hz: float
    controller_mode: str
    controller_status: str

    def validate(self, *, gripper_feedback_tolerance_m: float = 0.0) -> None:
        if (
            not math.isfinite(gripper_feedback_tolerance_m)
            or gripper_feedback_tolerance_m < 0.0
        ):
            raise InventoryError("gripper feedback tolerance must be non-negative")
        if self.role not in ARM_ORDER:
            raise InventoryError(f"unknown arm role: {self.role}")
        if len(self.joint_position_rad) != JOINT_COUNT:
            raise InventoryError(f"{self.role} must provide exactly six joints")
        values = (*self.joint_position_rad, self.gripper_width_m)
        if any(not math.isfinite(value) for value in values):
            raise InventoryError(f"{self.role} contains NaN or Inf")
        lower = GRIPPER_LIMITS_M[0] - gripper_feedback_tolerance_m
        upper = GRIPPER_LIMITS_M[1] + gripper_feedback_tolerance_m
        if not lower <= self.gripper_width_m <= upper:
            raise InventoryError(
                f"{self.role} gripper width is outside the measured feedback "
                f"range [{lower:.4f}, {upper:.4f}] m; "
                f"measured={self.gripper_width_m:.6f} m "
                f"({self.gripper_width_m * 1000.0:.3f} mm)"
            )
        rates = (self.arm_status_hz, self.gripper_status_hz)
        if any(not math.isfinite(rate) or rate < 0.0 for rate in rates):
            raise InventoryError(f"{self.role} feedback rate is invalid")

    @property
    def latest_timestamp_s(self) -> float:
        return max(self.arm_timestamp_s, self.gripper_timestamp_s)


class ArmReader(Protocol):
    """The complete interface accepted by the inventory owner: read one sample."""

    def read_sample(self) -> ArmSample: ...


@dataclass(frozen=True)
class DualPiperSnapshot:
    left: ArmSample
    right: ArmSample
    state_14d: tuple[float, ...]
    sample_skew_ms: float
    maximum_age_ms: float


class ReadOnlyDualPiperOwner:
    """Assemble and gate dual-arm feedback without exposing a write method."""

    def __init__(
        self,
        readers: Mapping[str, ArmReader],
        *,
        max_state_age_ms: float,
        max_inter_arm_skew_ms: float,
        gripper_feedback_tolerance_m: float = 0.0,
    ) -> None:
        if tuple(readers) != ARM_ORDER:
            raise InventoryError("readers must be ordered left_arm, right_arm")
        if max_state_age_ms <= 0.0 or max_inter_arm_skew_ms < 0.0:
            raise InventoryError("freshness thresholds must be non-negative")
        if (
            not math.isfinite(gripper_feedback_tolerance_m)
            or gripper_feedback_tolerance_m < 0.0
        ):
            raise InventoryError("gripper feedback tolerance must be non-negative")
        self._readers = dict(readers)
        self._max_state_age_ms = float(max_state_age_ms)
        self._max_inter_arm_skew_ms = float(max_inter_arm_skew_ms)
        self._gripper_feedback_tolerance_m = float(gripper_feedback_tolerance_m)

    def snapshot(self, *, now_s: float | None = None) -> DualPiperSnapshot:
        samples = {role: self._readers[role].read_sample() for role in ARM_ORDER}
        for role, sample in samples.items():
            if sample.role != role:
                raise InventoryError(f"reader role mismatch: expected {role}, got {sample.role}")
            sample.validate(
                gripper_feedback_tolerance_m=self._gripper_feedback_tolerance_m
            )

        left = samples["left_arm"]
        right = samples["right_arm"]
        timestamps = (
            left.arm_timestamp_s,
            left.gripper_timestamp_s,
            right.arm_timestamp_s,
            right.gripper_timestamp_s,
        )
        # Live callers sample the clock AFTER reads: a receiver can update while
        # the two arms are read. Explicit now_s remains for offline replay/tests.
        if now_s is None:
            now_s = time.time()
        ages_ms = tuple((now_s - timestamp) * 1000.0 for timestamp in timestamps)
        if any(age < 0.0 for age in ages_ms):
            raise InventoryError("feedback timestamp is in the future")
        maximum_age_ms = max(ages_ms)
        if maximum_age_ms > self._max_state_age_ms:
            raise InventoryError(
                f"dual-arm state is stale: {maximum_age_ms:.3f} ms > {self._max_state_age_ms:.3f} ms"
            )

        sample_skew_ms = abs(left.latest_timestamp_s - right.latest_timestamp_s) * 1000.0
        if sample_skew_ms > self._max_inter_arm_skew_ms:
            raise InventoryError(
                f"inter-arm skew is too large: {sample_skew_ms:.3f} ms > "
                f"{self._max_inter_arm_skew_ms:.3f} ms"
            )

        state_14d = (
            *left.joint_position_rad,
            left.gripper_width_m,
            *right.joint_position_rad,
            right.gripper_width_m,
        )
        return DualPiperSnapshot(
            left=left,
            right=right,
            state_14d=state_14d,
            sample_skew_ms=sample_skew_ms,
            maximum_age_ms=maximum_age_ms,
        )
