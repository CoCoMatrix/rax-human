from __future__ import annotations

from farther_core.gr3_dagger import BASE_ACTION_NAMES, JOINT_NAMES

from XPolicyLab.real_world.dagger import (
    DaggerActionProfile,
    DaggerRouter as SharedDaggerRouter,
    Decision,
    Mode,
)


WAIST_JOINTS = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
HAND_JOINTS = frozenset(JOINT_NAMES[19:31])


class DaggerRouter(SharedDaggerRouter):
    """GR3-compatible facade over the robot-independent DAgger router."""

    def __init__(
        self,
        max_joint_step: float = 0.15,
        freeze_waist: bool = False,
        model_lowpass_alpha: float | None = None,
        execute_base: bool = False,
        frozen_waist_joints: tuple[str, ...] = WAIST_JOINTS,
    ) -> None:
        frozen_waist_joints = tuple(dict.fromkeys(frozen_waist_joints))
        unknown = set(frozen_waist_joints) - set(WAIST_JOINTS)
        if unknown:
            raise ValueError(f"unknown frozen waist joints: {sorted(unknown)}")
        if freeze_waist and not frozen_waist_joints:
            raise ValueError(
                "frozen_waist_joints must not be empty when freeze_waist is enabled"
            )
        self.max_joint_step = float(max_joint_step)
        self.freeze_waist = bool(freeze_waist)
        self.execute_base = bool(execute_base)
        self.frozen_waist_joints = frozen_waist_joints
        super().__init__(
            DaggerActionProfile(
                default_max_step=self.max_joint_step,
                direct_expert_fields=HAND_JOINTS,
                frozen_fields=(self.frozen_waist_joints if self.freeze_waist else ()),
                disabled_fields=(
                    {} if self.execute_base else dict.fromkeys(BASE_ACTION_NAMES, 0.0)
                ),
                frozen_reason="waist_frozen",
                disabled_reason="base_disabled",
            ),
            model_lowpass_alpha=model_lowpass_alpha,
        )

    @property
    def waist_reference(self) -> dict[str, float] | None:
        """Legacy GR3 name retained for existing callers and probes."""
        return self.frozen_reference


__all__ = [
    "DaggerActionProfile",
    "DaggerRouter",
    "Decision",
    "HAND_JOINTS",
    "Mode",
    "WAIST_JOINTS",
]
