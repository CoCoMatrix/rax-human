from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np
from XPolicyLab.real_world.contracts import ObservationFrame, VectorSchema

PART_DOFS = {
    "astribot_chassis": 3,
    "astribot_torso": 4,
    "astribot_arm_left": 7,
    "astribot_gripper_left": 1,
    "astribot_arm_right": 7,
    "astribot_gripper_right": 1,
    "astribot_head": 2,
}
PART_NAMES = tuple(PART_DOFS)
ARM_NAMES = ("astribot_arm_left", "astribot_arm_right")
GRIPPER_NAMES = ("astribot_gripper_left", "astribot_gripper_right")

JOINT_NAMES = tuple(
    f"{part}.joint_{index}" for part, dof in PART_DOFS.items() for index in range(dof)
)

_POSE_COMPONENTS = ("x", "y", "z", "qx", "qy", "qz", "qw")
ACTION_NAMES = tuple(
    component
    for side in ("left", "right")
    for component in (
        *(f"{side}.{name}" for name in _POSE_COMPONENTS),
        f"{side}.gripper",
    )
)
ARM_JOINT_ACTION_NAMES = tuple(
    f"{arm}.joint_{index}" for arm in ARM_NAMES for index in range(PART_DOFS[arm])
)
GRIPPER_ACTION_NAMES = tuple(f"{gripper}.joint_0" for gripper in GRIPPER_NAMES)
CHASSIS_VELOCITY_NAMES = (
    "astribot_chassis.velocity_x",
    "astribot_chassis.velocity_y",
    "astribot_chassis.angular_velocity_z",
)
AUXILIARY_NAMES = ("astribot_torso", "astribot_head")
AUXILIARY_ACTION_NAMES = tuple(
    f"{part}.joint_{index}"
    for part in AUXILIARY_NAMES
    for index in range(PART_DOFS[part])
)
JOINT_GRIPPER_ACTION_NAMES = tuple(
    name
    for arm, gripper in zip(ARM_NAMES, GRIPPER_NAMES, strict=True)
    for name in (
        *(f"{arm}.joint_{index}" for index in range(PART_DOFS[arm])),
        f"{gripper}.joint_0",
    )
)

ASTRIBOT_S1_STATE_SCHEMA = VectorSchema(
    schema_id="astribot_s1.full25_joint_position.v1",
    fields=JOINT_NAMES,
    units=("m", "m", "rad")
    + ("m", "rad", "rad", "rad")
    + ("rad",) * 7
    + ("percent",)
    + ("rad",) * 7
    + ("percent",)
    + ("rad",) * 2,
    semantics=("absolute_position",) * len(JOINT_NAMES),
)

FULL_BODY_ACTION_NAMES = (
    *CHASSIS_VELOCITY_NAMES,
    *AUXILIARY_ACTION_NAMES[:4],
    *JOINT_GRIPPER_ACTION_NAMES[:8],
    *JOINT_GRIPPER_ACTION_NAMES[8:],
    *AUXILIARY_ACTION_NAMES[4:],
)

ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA = VectorSchema(
    schema_id="astribot_s1.full_body_joint_position_chassis_velocity.v1",
    fields=FULL_BODY_ACTION_NAMES,
    units=("m/s", "m/s", "rad/s")
    + ("m", "rad", "rad", "rad")
    + ("rad",) * 7
    + ("percent",)
    + ("rad",) * 7
    + ("percent",)
    + ("rad",) * 2,
    semantics=("linear_velocity", "linear_velocity", "angular_velocity")
    + ("absolute_position",) * 22,
)

ASTRIBOT_S1_CARTESIAN_ACTION_SCHEMA = VectorSchema(
    schema_id="astribot_s1.dual_arm_cartesian_gripper.v1",
    fields=ACTION_NAMES,
    units=tuple(
        unit
        for _side in ("left", "right")
        for unit in (
            "m",
            "m",
            "m",
            "unitless",
            "unitless",
            "unitless",
            "unitless",
            "percent",
        )
    ),
    semantics=tuple(
        semantic
        for _side in ("left", "right")
        for semantic in ("absolute_position",) * 8
    ),
)

ASTRIBOT_S1_ARM_JOINT_ACTION_SCHEMA = VectorSchema(
    schema_id="astribot_s1.dual_arm_joint_position.v1",
    fields=ARM_JOINT_ACTION_NAMES,
    units=("rad",) * len(ARM_JOINT_ACTION_NAMES),
    semantics=("absolute_position",) * len(ARM_JOINT_ACTION_NAMES),
)

ASTRIBOT_S1_JOINT_GRIPPER_ACTION_SCHEMA = VectorSchema(
    schema_id="astribot_s1.dual_arm_gripper_joint_position.v1",
    fields=JOINT_GRIPPER_ACTION_NAMES,
    units=("rad",) * 7 + ("percent",) + ("rad",) * 7 + ("percent",),
    semantics=("absolute_position",) * len(JOINT_GRIPPER_ACTION_NAMES),
)


def flatten_joint_state(joints: Mapping[str, Sequence[float]]) -> np.ndarray:
    if set(joints) != set(PART_NAMES):
        missing = sorted(set(PART_NAMES) - set(joints))
        extra = sorted(set(joints) - set(PART_NAMES))
        raise ValueError(
            f"invalid Astribot joint parts: missing={missing}, extra={extra}"
        )
    values: list[float] = []
    for part, dof in PART_DOFS.items():
        part_values = tuple(joints[part])
        if len(part_values) != dof:
            raise ValueError(
                f"{part} must contain {dof} joints, got {len(part_values)}"
            )
        values.extend(part_values)
    return ASTRIBOT_S1_STATE_SCHEMA.coerce(values, name="Astribot S1 state", ndim=1)


def action_vector(action: Mapping[str, Any]) -> np.ndarray:
    if set(action) != set(ACTION_NAMES):
        missing = sorted(set(ACTION_NAMES) - set(action))
        extra = sorted(set(action) - set(ACTION_NAMES))
        raise ValueError(f"invalid Astribot action: missing={missing}, extra={extra}")
    return ASTRIBOT_S1_CARTESIAN_ACTION_SCHEMA.coerce(
        [action[name] for name in ACTION_NAMES],
        name="Astribot S1 Cartesian action",
        ndim=1,
    )


def action_dict(action: Any) -> dict[str, float]:
    vector = ASTRIBOT_S1_CARTESIAN_ACTION_SCHEMA.coerce(
        action, name="Astribot S1 Cartesian action", ndim=1
    )
    return dict(zip(ACTION_NAMES, map(float, vector), strict=True))


def arm_joint_action_dict(action: Mapping[str, Any] | Any) -> dict[str, float]:
    if isinstance(action, Mapping):
        if set(action) != set(ARM_JOINT_ACTION_NAMES):
            raise ValueError("invalid Astribot dual-arm joint action")
        values = [action[name] for name in ARM_JOINT_ACTION_NAMES]
    else:
        values = action
    vector = ASTRIBOT_S1_ARM_JOINT_ACTION_SCHEMA.coerce(
        values,
        name="Astribot S1 dual-arm joint action",
        ndim=1,
    )
    return dict(zip(ARM_JOINT_ACTION_NAMES, map(float, vector), strict=True))


def gripper_action_dict(
    action: Mapping[str, Any] | Sequence[float],
) -> dict[str, float]:
    if isinstance(action, Mapping):
        if set(action) != set(GRIPPER_ACTION_NAMES):
            raise ValueError("invalid Astribot dual-gripper action")
        values = [action[name] for name in GRIPPER_ACTION_NAMES]
    else:
        values = list(action)
    array = np.asarray(values, dtype=float)
    if array.shape != (2,) or not np.isfinite(array).all():
        raise ValueError("Astribot dual-gripper action must contain 2 finite values")
    return dict(zip(GRIPPER_ACTION_NAMES, map(float, array), strict=True))


def joint_gripper_action_dict(
    action: Mapping[str, Any] | Any,
) -> dict[str, float]:
    """Coerce the 16D arm/gripper teleop action into fixed field order."""
    if isinstance(action, Mapping):
        if set(action) != set(JOINT_GRIPPER_ACTION_NAMES):
            missing = sorted(set(JOINT_GRIPPER_ACTION_NAMES) - set(action))
            extra = sorted(set(action) - set(JOINT_GRIPPER_ACTION_NAMES))
            raise ValueError(
                "invalid Astribot arm/gripper joint action: "
                f"missing={missing}, extra={extra}"
            )
        values = [action[name] for name in JOINT_GRIPPER_ACTION_NAMES]
    else:
        values = action
    vector = ASTRIBOT_S1_JOINT_GRIPPER_ACTION_SCHEMA.coerce(
        values,
        name="Astribot S1 arm/gripper joint action",
        ndim=1,
    )
    return dict(zip(JOINT_GRIPPER_ACTION_NAMES, map(float, vector), strict=True))


def joint_gripper_action_vector(action: Mapping[str, Any]) -> np.ndarray:
    values = joint_gripper_action_dict(action)
    return ASTRIBOT_S1_JOINT_GRIPPER_ACTION_SCHEMA.coerce(
        [values[name] for name in JOINT_GRIPPER_ACTION_NAMES],
        name="Astribot S1 arm/gripper joint action",
        ndim=1,
    )


def chassis_velocity_action_dict(
    action: Mapping[str, Any] | Sequence[float],
) -> dict[str, float]:
    if isinstance(action, Mapping):
        if set(action) != set(CHASSIS_VELOCITY_NAMES):
            raise ValueError("invalid Astribot chassis velocity action")
        values = [action[name] for name in CHASSIS_VELOCITY_NAMES]
    else:
        values = list(action)
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError(
            "Astribot chassis velocity action must contain 3 finite values"
        )
    return dict(zip(CHASSIS_VELOCITY_NAMES, map(float, array), strict=True))


def auxiliary_action_dict(action: Mapping[str, Any] | Any) -> dict[str, float]:
    if isinstance(action, Mapping):
        if set(action) != set(AUXILIARY_ACTION_NAMES):
            raise ValueError("invalid Astribot torso/head action")
        values = [action[name] for name in AUXILIARY_ACTION_NAMES]
    else:
        values = action
    array = np.asarray(values, dtype=float)
    if array.shape != (len(AUXILIARY_ACTION_NAMES),) or not np.isfinite(array).all():
        raise ValueError("Astribot torso/head action must contain 6 finite values")
    return dict(zip(AUXILIARY_ACTION_NAMES, map(float, array), strict=True))


def full_body_action_dict(action: Mapping[str, Any] | Any) -> dict[str, float]:
    if isinstance(action, Mapping):
        if set(action) != set(FULL_BODY_ACTION_NAMES):
            missing = sorted(set(FULL_BODY_ACTION_NAMES) - set(action))
            extra = sorted(set(action) - set(FULL_BODY_ACTION_NAMES))
            raise ValueError(
                f"invalid Astribot full-body action: missing={missing}, extra={extra}"
            )
        values = [action[name] for name in FULL_BODY_ACTION_NAMES]
    else:
        values = action
    vector = ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.coerce(
        values,
        name="Astribot S1 full-body action",
        ndim=1,
    )
    return dict(zip(FULL_BODY_ACTION_NAMES, map(float, vector), strict=True))


def full_body_action_vector(action: Mapping[str, Any]) -> np.ndarray:
    values = full_body_action_dict(action)
    return ASTRIBOT_S1_FULL_BODY_ACTION_SCHEMA.coerce(
        [values[name] for name in FULL_BODY_ACTION_NAMES],
        name="Astribot S1 full-body action",
        ndim=1,
    )


def combine_full_body_action(
    chassis_velocity: Mapping[str, Any],
    auxiliary: Mapping[str, Any],
    arm_gripper: Mapping[str, Any],
) -> dict[str, float]:
    return full_body_action_dict(
        {
            **chassis_velocity_action_dict(chassis_velocity),
            **auxiliary_action_dict(auxiliary),
            **joint_gripper_action_dict(arm_gripper),
        }
    )


def split_full_body_action(
    action: Mapping[str, Any] | Any,
) -> tuple[dict[str, float], dict[str, float], dict[str, float], dict[str, float]]:
    values = full_body_action_dict(action)
    arm_gripper = {name: values[name] for name in JOINT_GRIPPER_ACTION_NAMES}
    arms, grippers = split_arm_gripper_action(arm_gripper)
    return (
        chassis_velocity_action_dict(
            {name: values[name] for name in CHASSIS_VELOCITY_NAMES}
        ),
        auxiliary_action_dict({name: values[name] for name in AUXILIARY_ACTION_NAMES}),
        arms,
        grippers,
    )


def current_auxiliary_action(
    joints: Mapping[str, Sequence[float]],
) -> dict[str, float]:
    return auxiliary_action_dict(
        {
            f"{part}.joint_{index}": joints[part][index]
            for part in AUXILIARY_NAMES
            for index in range(PART_DOFS[part])
        }
    )


def current_full_body_hold_action(
    joints: Mapping[str, Sequence[float]],
) -> dict[str, float]:
    arms = current_arm_joint_action(joints)
    grippers = gripper_action_dict([joints[name][0] for name in GRIPPER_NAMES])
    return combine_full_body_action(
        chassis_velocity_action_dict((0.0, 0.0, 0.0)),
        current_auxiliary_action(joints),
        combine_arm_gripper_action(arms, grippers),
    )


def combine_arm_gripper_action(
    arm_action: Mapping[str, Any],
    gripper_action: Mapping[str, Any],
) -> dict[str, float]:
    arms = arm_joint_action_dict(arm_action)
    grippers = gripper_action_dict(gripper_action)
    return joint_gripper_action_dict({**arms, **grippers})


def split_arm_gripper_action(
    action: Mapping[str, Any] | Any,
) -> tuple[dict[str, float], dict[str, float]]:
    values = joint_gripper_action_dict(action)
    return (
        arm_joint_action_dict({name: values[name] for name in ARM_JOINT_ACTION_NAMES}),
        gripper_action_dict({name: values[name] for name in GRIPPER_ACTION_NAMES}),
    )


def current_arm_joint_action(
    joints: Mapping[str, Sequence[float]],
) -> dict[str, float]:
    return arm_joint_action_dict(
        {
            f"{arm}.joint_{index}": joints[arm][index]
            for arm in ARM_NAMES
            for index in range(PART_DOFS[arm])
        }
    )


def arm_joint_command_lists(
    action: Mapping[str, Any],
) -> tuple[list[str], list[list[float]]]:
    values = arm_joint_action_dict(action)
    return (
        list(ARM_NAMES),
        [
            [values[f"{arm}.joint_{index}"] for index in range(PART_DOFS[arm])]
            for arm in ARM_NAMES
        ],
    )


def gripper_command_lists(
    action: Mapping[str, Any],
) -> tuple[list[str], list[list[float]]]:
    values = gripper_action_dict(action)
    return (
        list(GRIPPER_NAMES),
        [[values[f"{gripper}.joint_0"]] for gripper in GRIPPER_NAMES],
    )


def arm_joint_gripper_command_lists(
    arm_action: Mapping[str, Any],
    gripper_action: Mapping[str, Any],
) -> tuple[list[str], list[list[float]]]:
    """Build the fixed SDK schema used by joint-delta teleoperation."""
    combined = combine_arm_gripper_action(arm_action, gripper_action)
    arm_values, gripper_values = split_arm_gripper_action(combined)
    names = [ARM_NAMES[0], GRIPPER_NAMES[0], ARM_NAMES[1], GRIPPER_NAMES[1]]
    commands = [
        [
            arm_values[f"{ARM_NAMES[0]}.joint_{index}"]
            for index in range(PART_DOFS[ARM_NAMES[0]])
        ],
        [gripper_values[f"{GRIPPER_NAMES[0]}.joint_0"]],
        [
            arm_values[f"{ARM_NAMES[1]}.joint_{index}"]
            for index in range(PART_DOFS[ARM_NAMES[1]])
        ],
        [gripper_values[f"{GRIPPER_NAMES[1]}.joint_0"]],
    ]
    return names, commands


def command_lists(
    action: Mapping[str, Any],
) -> tuple[list[str], list[str], list[list[float]]]:
    values = action_dict(action_vector(action))
    names = [ARM_NAMES[0], GRIPPER_NAMES[0], ARM_NAMES[1], GRIPPER_NAMES[1]]
    types = ["cartesian", "joints", "cartesian", "joints"]
    commands = [
        [values[f"left.{component}"] for component in _POSE_COMPONENTS],
        [values["left.gripper"]],
        [values[f"right.{component}"] for component in _POSE_COMPONENTS],
        [values["right.gripper"]],
    ]
    return names, types, commands


def arm_command_lists(
    action: Mapping[str, Any],
) -> tuple[list[str], list[str], list[list[float]]]:
    values = action_dict(action_vector(action))
    return (
        list(ARM_NAMES),
        ["cartesian", "cartesian"],
        [
            [values[f"left.{component}"] for component in _POSE_COMPONENTS],
            [values[f"right.{component}"] for component in _POSE_COMPONENTS],
        ],
    )


def current_pose_action(
    arm_poses: Mapping[str, Sequence[float]],
    joints: Mapping[str, Sequence[float]],
    gripper_positions: Mapping[str, Sequence[float]] | None = None,
) -> dict[str, float]:
    """Build a no-displacement command from one SDK state snapshot."""
    values: dict[str, float] = {}
    for side, arm, gripper in zip(
        ("left", "right"),
        ARM_NAMES,
        GRIPPER_NAMES,
        strict=True,
    ):
        pose = tuple(arm_poses[arm])
        grip = tuple((gripper_positions or joints)[gripper])
        if len(pose) != 7 or len(grip) != 1:
            raise ValueError("invalid Astribot current-pose state")
        for component, value in zip(_POSE_COMPONENTS, pose, strict=True):
            values[f"{side}.{component}"] = float(value)
        values[f"{side}.gripper"] = float(grip[0])
    return action_dict(action_vector(values))


def build_observation(
    joints: Mapping[str, Sequence[float]],
    images: Mapping[str, np.ndarray],
    instruction: str,
    *,
    timestamp_ns: int,
    sequence: int,
    robot_id: str,
    metadata: Mapping[str, Any] | None = None,
) -> ObservationFrame:
    return ObservationFrame(
        state_schema=ASTRIBOT_S1_STATE_SCHEMA,
        state=flatten_joint_state(joints),
        images=images,
        instruction=instruction,
        timestamp_ns=timestamp_ns,
        sequence=sequence,
        robot_type="astribot_s1",
        robot_id=robot_id,
        metadata=dict(metadata or {}),
    )


def matrix_to_pose(matrix: Any) -> np.ndarray:
    transform = np.asarray(matrix, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("end-effector transform must be a finite 4x4 matrix")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-5):
        raise ValueError("invalid homogeneous transform")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4):
        raise ValueError("end-effector rotation is not orthonormal")
    quaternion = rotation_matrix_to_quaternion(rotation)
    return np.concatenate((transform[:3, 3], quaternion))


def pose_to_matrix(pose: Any) -> np.ndarray:
    values = np.asarray(pose, dtype=float)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError("Cartesian pose must contain seven finite values")
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_rotation_matrix(values[3:])
    matrix[:3, 3] = values[:3]
    return matrix


def quaternion_to_rotation_matrix(quaternion: Any) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_matrix_to_quaternion(rotation: Any) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    # Stable branch-based conversion; output order matches the SDK: qx, qy, qz, qw.
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            quaternion = np.array(
                [
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            quaternion = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ]
            )
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            quaternion = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ]
            )
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0:
        quaternion = -quaternion
    return quaternion
