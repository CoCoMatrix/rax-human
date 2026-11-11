"""Validated, model-derived transition poses for supervised dual-PiPER runs.

The library contains identifiers and recorded follower states.  External
supervisors may request an identifier, but only the sole robot owner resolves
that identifier to joint targets and applies the robot-owned safety contract.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET


SCHEMA = "xpolicylab.piper-transition-pose-library/v1"


def _matrix_multiply(
    left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]
) -> list[list[float]]:
    return [
        [sum(left[row][inner] * right[inner][column] for inner in range(4)) for column in range(4)]
        for row in range(4)
    ]


def _origin_matrix(xyz: Sequence[float], rpy: Sequence[float]) -> list[list[float]]:
    roll, pitch, yaw = map(float, rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    result = [
        [rotation[row][column] if column < 3 else float(xyz[row]) for column in range(4)]
        for row in range(3)
    ]
    result.append([0.0, 0.0, 0.0, 1.0])
    return result


def _axis_rotation(axis: Sequence[float], angle: float) -> list[list[float]]:
    x, y, z = map(float, axis)
    cosine, sine, complement = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    rotation = (
        (
            x * x * complement + cosine,
            x * y * complement - z * sine,
            x * z * complement + y * sine,
        ),
        (
            y * x * complement + z * sine,
            y * y * complement + cosine,
            y * z * complement - x * sine,
        ),
        (
            z * x * complement - y * sine,
            z * y * complement + x * sine,
            z * z * complement + cosine,
        ),
    )
    return [
        [rotation[row][column] if column < 3 else 0.0 for column in range(4)]
        for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def _vector(value: str) -> tuple[float, float, float]:
    parsed = tuple(float(item) for item in value.split())
    if len(parsed) != 3:
        raise ValueError("URDF vector must contain three values")
    return parsed


class TransitionPathGuard:
    """Check sampled joint interpolation against the pinned URDF height floor."""

    def __init__(self, profile: Mapping[str, Any], profile_path: str | Path) -> None:
        geometry = profile["geometry"]
        configured = Path(str(geometry["urdf_path"]))
        self.urdf_path = (
            configured.resolve()
            if configured.is_absolute()
            else (Path(profile_path).resolve().parent / configured).resolve()
        )
        actual_sha256 = hashlib.sha256(self.urdf_path.read_bytes()).hexdigest()
        if actual_sha256 != str(geometry["urdf_source_sha256"]):
            raise ValueError("transition path URDF identity does not match the profile")
        root = ET.parse(self.urdf_path).getroot()
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        self._origins: dict[str, tuple[tuple[float, ...], ...]] = {}
        self._axes: dict[str, tuple[tuple[float, ...], ...]] = {}
        base_xyz = {
            "left": tuple(map(float, geometry["left_base_xyz_m"])),
            "right": tuple(map(float, geometry["right_base_xyz_m"])),
        }
        base_rpy = tuple(map(float, geometry["base_rpy_rad"]))
        for side in ("left", "right"):
            origins: list[tuple[float, ...]] = []
            axes: list[tuple[float, ...]] = []
            for index in range(1, 7):
                joint = joints[f"{side}_joint{index}"]
                origin = joint.find("origin")
                axis = joint.find("axis")
                if origin is None or axis is None:
                    raise ValueError("transition path URDF joint is incomplete")
                origins.append(
                    (
                        *_vector(origin.attrib.get("xyz", "0 0 0")),
                        *_vector(origin.attrib.get("rpy", "0 0 0")),
                    )
                )
                axes.append(_vector(axis.attrib["xyz"]))
            self._origins[side] = tuple(origins)
            self._axes[side] = tuple(axes)
        self._base_xyz = base_xyz
        self._base_rpy = base_rpy

    def flange_height(self, side: str, joints_rad: Sequence[float]) -> float:
        values = tuple(map(float, joints_rad))
        if len(values) != 6:
            raise ValueError("flange-height check requires six joints")
        transform = _origin_matrix(self._base_xyz[side], self._base_rpy)
        for origin, axis, position in zip(
            self._origins[side], self._axes[side], values, strict=True
        ):
            transform = _matrix_multiply(
                transform, _origin_matrix(origin[:3], origin[3:])
            )
            transform = _matrix_multiply(
                transform, _axis_rotation(axis, position)
            )
        return float(transform[2][3])

    def validate_interpolation(
        self,
        current_arm_joint_position_rad: Sequence[float],
        target_arm_joint_position_rad: Sequence[float],
        *,
        samples: int,
        minimum_flange_height_m: float,
    ) -> dict[str, float]:
        current = tuple(map(float, current_arm_joint_position_rad))
        target = tuple(map(float, target_arm_joint_position_rad))
        if len(current) != 12 or len(target) != 12 or int(samples) < 2:
            raise ValueError("transition interpolation contract is invalid")
        minima = {"left": math.inf, "right": math.inf}
        for index in range(int(samples)):
            weight = index / (int(samples) - 1)
            joints = tuple(
                start + weight * (finish - start)
                for start, finish in zip(current, target, strict=True)
            )
            for side, arm in (("left", joints[:6]), ("right", joints[6:])):
                minima[side] = min(minima[side], self.flange_height(side, arm))
        required = float(minimum_flange_height_m)
        if min(minima.values()) < required:
            raise ValueError(
                "transition joint interpolation violates the flange-height floor"
            )
        return minima


@dataclass(frozen=True, slots=True)
class TransitionPose:
    pose_id: str
    task_prompt: str
    arm_joint_position_rad: tuple[float, ...]
    source: Mapping[str, Any]
    pre_action_evidence: Mapping[str, Any]

    @property
    def left_arm(self) -> tuple[float, ...]:
        return self.arm_joint_position_rad[:6]

    @property
    def right_arm(self) -> tuple[float, ...]:
        return self.arm_joint_position_rad[6:]


class TransitionPoseLibrary:
    """Load an immutable allowlist and reject terminal-like candidates."""

    def __init__(
        self,
        path: str | Path,
        *,
        expected_sha256: str = "",
        joint_limits_rad: Mapping[str, Sequence[float]] | None = None,
        joint_names: Sequence[str] = (),
    ) -> None:
        self.path = Path(path).expanduser().resolve()
        raw = self.path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        expected = str(expected_sha256).strip().lower()
        if expected and self.sha256 != expected:
            raise ValueError(
                "transition-pose library SHA-256 does not match the deployment"
            )
        value = json.loads(raw)
        if not isinstance(value, dict) or value.get("schema") != SCHEMA:
            raise ValueError("unsupported Piper transition-pose library schema")
        if value.get("robot_type") != "piper_x_dual":
            raise ValueError("transition-pose library targets the wrong robot type")
        contract = value.get("candidate_contract")
        selection = value.get("selection")
        rows = value.get("entries")
        if not isinstance(contract, dict) or not isinstance(selection, dict):
            raise ValueError("transition-pose library is missing its safety contract")
        if contract.get("terminal_or_post_completion_frames_allowed") is not False:
            raise ValueError("transition poses must exclude terminal/completed frames")
        if contract.get("pose_is_recorded_state_not_average") is not True:
            raise ValueError("transition poses must be actual recorded follower states")
        if not isinstance(rows, list) or not rows:
            raise ValueError("transition-pose library must contain entries")
        try:
            self.max_joint_delta_rad = float(
                selection["runtime_max_joint_delta_rad"]
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid transition-pose runtime joint bound") from exc
        if not 0.0 < self.max_joint_delta_rad <= math.pi:
            raise ValueError("transition-pose runtime joint bound is unsafe")
        segmented_limit = selection.get("segmented_transition_max_joint_delta_rad")
        if segmented_limit is None:
            self.segmented_max_joint_delta_rad: float | None = None
        else:
            try:
                segmented_limit = float(segmented_limit)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "invalid segmented transition joint bound"
                ) from exc
            if not self.max_joint_delta_rad < segmented_limit <= math.pi:
                raise ValueError(
                    "segmented transition joint bound must exceed the direct "
                    "bound and remain within pi"
                )
            self.segmented_max_joint_delta_rad = segmented_limit

        names = tuple(joint_names)
        if joint_limits_rad is not None and len(names) != 12:
            raise ValueError("transition-pose validation requires twelve joint names")
        poses: dict[str, TransitionPose] = {}
        for row in rows:
            pose = self._parse_pose(row, contract)
            if pose.pose_id in poses:
                raise ValueError(f"duplicate transition pose id: {pose.pose_id}")
            if joint_limits_rad is not None:
                for name, position in zip(
                    names, pose.arm_joint_position_rad, strict=True
                ):
                    lower, upper = map(float, joint_limits_rad[name])
                    if not lower <= position <= upper:
                        raise ValueError(
                            f"transition pose {pose.pose_id} exceeds {name} limits"
                        )
            poses[pose.pose_id] = pose
        self.library_id = str(value.get("library_id", "")).strip()
        if not self.library_id:
            raise ValueError("transition-pose library requires a library_id")
        self.dataset = dict(value.get("dataset") or {})
        self.candidate_contract = dict(contract)
        self.selection = dict(selection)
        self.poses = poses

    @staticmethod
    def _parse_pose(row: Any, contract: Mapping[str, Any]) -> TransitionPose:
        if not isinstance(row, dict):
            raise ValueError("transition-pose entry must be an object")
        pose_id = str(row.get("pose_id", "")).strip()
        task = str(row.get("task_prompt", "")).strip()
        if not pose_id or len(pose_id) > 128 or not task:
            raise ValueError("transition-pose entry has an invalid id or task")
        joints = row.get("arm_joint_position_rad")
        if not isinstance(joints, list) or len(joints) != 12:
            raise ValueError(f"transition pose {pose_id} must contain twelve joints")
        try:
            positions = tuple(float(item) for item in joints)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"transition pose {pose_id} has invalid joints") from exc
        if not all(math.isfinite(item) for item in positions):
            raise ValueError(f"transition pose {pose_id} contains non-finite joints")
        if row.get("gripper_policy") != "preserve_measured":
            raise ValueError("transition poses must preserve measured gripper widths")
        recorded = row.get("recorded_arm_joint_position_rad")
        adjustments = row.get("boundary_adjustments")
        if not isinstance(recorded, list) or len(recorded) != 12:
            raise ValueError(
                f"transition pose {pose_id} lacks its recorded follower state"
            )
        try:
            recorded_positions = tuple(float(item) for item in recorded)
            maximum_adjustment = float(
                contract["maximum_boundary_normalization_rad"]
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"transition pose {pose_id} has an invalid boundary contract"
            ) from exc
        if not isinstance(adjustments, list):
            raise ValueError(f"transition pose {pose_id} has invalid adjustments")
        declared: dict[int, Mapping[str, Any]] = {}
        for adjustment in adjustments:
            if not isinstance(adjustment, dict):
                raise ValueError(
                    f"transition pose {pose_id} has malformed adjustment evidence"
                )
            try:
                index = int(adjustment["joint_index"])
                source_value = float(adjustment["recorded_rad"])
                target_value = float(adjustment["target_rad"])
                absolute = float(adjustment["absolute_adjustment_rad"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"transition pose {pose_id} has malformed adjustment evidence"
                ) from exc
            if not 0 <= index < 12 or index in declared:
                raise ValueError(f"transition pose {pose_id} adjustment is ambiguous")
            if (
                source_value != recorded_positions[index]
                or target_value != positions[index]
                or not math.isclose(
                    absolute, abs(target_value - source_value), abs_tol=1e-9
                )
                or absolute > maximum_adjustment
            ):
                raise ValueError(
                    f"transition pose {pose_id} adjustment does not match its source"
                )
            declared[index] = adjustment
        for index, (source_value, target_value) in enumerate(
            zip(recorded_positions, positions, strict=True)
        ):
            changed = not math.isclose(source_value, target_value, abs_tol=1e-12)
            if changed != (index in declared):
                raise ValueError(
                    f"transition pose {pose_id} hides a boundary adjustment"
                )
        source = row.get("source")
        evidence = row.get("pre_action_evidence")
        if not isinstance(source, dict) or not str(source.get("uri", "")).startswith(
            "oss://"
        ):
            raise ValueError(f"transition pose {pose_id} lacks dataset provenance")
        if not isinstance(evidence, dict):
            raise ValueError(f"transition pose {pose_id} lacks pre-action evidence")
        checks = (
            ("frame_fraction", "episode_fraction_min", "episode_fraction_max"),
            ("future_frames", "minimum_future_frames_30hz", None),
            ("future_motion_linf_rad", "minimum_future_motion_linf_rad", None),
            ("terminal_distance_l2_rad", "minimum_terminal_distance_l2_rad", None),
            ("initial_distance_l2_rad", "minimum_initial_distance_l2_rad", None),
        )
        for value_key, lower_key, upper_key in checks:
            try:
                observed = float(evidence[value_key])
                lower = float(contract[lower_key])
                upper = float(contract[upper_key]) if upper_key else None
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"transition pose {pose_id} has incomplete {value_key} evidence"
                ) from exc
            if observed < lower or (upper is not None and observed > upper):
                raise ValueError(
                    f"transition pose {pose_id} violates {value_key} contract"
                )
        # Newer libraries anchor the target to a detected movement onset and
        # require a lead of stationary/pre-action frames.  Keep these checks
        # optional so older deployment-bound libraries remain readable, while
        # refusing a library that claims the contract but has inconsistent
        # source/frame evidence.
        minimum_lead = contract.get("minimum_pre_action_lead_frames_30hz")
        if minimum_lead is not None:
            try:
                source_frame = int(source["resampled_frame_index_30hz"])
                onset_frame = int(evidence["movement_onset_frame_30hz"])
                observed_lead = int(evidence["pre_action_lead_frames_30hz"])
                required_lead = int(minimum_lead)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"transition pose {pose_id} lacks movement-onset evidence"
                ) from exc
            if (
                required_lead < 1
                or observed_lead < required_lead
                or onset_frame - source_frame != observed_lead
            ):
                raise ValueError(
                    f"transition pose {pose_id} violates pre-action lead contract"
                )
        maximum_initial = contract.get("maximum_initial_distance_l2_rad")
        if maximum_initial is not None:
            try:
                initial_distance = float(evidence["initial_distance_l2_rad"])
                initial_bound = float(maximum_initial)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"transition pose {pose_id} lacks initial-distance evidence"
                ) from exc
            if initial_distance > initial_bound:
                raise ValueError(
                    f"transition pose {pose_id} is too far from its source start"
                )
        try:
            speed = float(evidence["local_speed_p75_rad_s"])
            max_speed = float(contract["maximum_local_speed_p75_rad_s"])
            heights = evidence["flange_height_m"]
            min_height = min(float(heights["left"]), float(heights["right"]))
            required_height = float(contract["minimum_flange_height_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"transition pose {pose_id} has invalid speed/height evidence"
            ) from exc
        if speed > max_speed or min_height < required_height:
            raise ValueError(f"transition pose {pose_id} violates speed/height contract")
        return TransitionPose(pose_id, task, positions, dict(source), dict(evidence))

    def resolve_request(self, request: Mapping[str, Any]) -> TransitionPose:
        if str(request.get("library_sha256", "")).lower() != self.sha256:
            raise ValueError("transition request used the wrong library identity")
        pose_id = str(request.get("pose_id", "")).strip()
        try:
            pose = self.poses[pose_id]
        except KeyError as exc:
            raise ValueError("transition request pose id is not allowlisted") from exc
        if str(request.get("task_prompt", "")).strip() != pose.task_prompt:
            raise ValueError("transition request task does not match the pose")
        return pose

    def validate_move(
        self,
        current_arm_joint_position_rad: Sequence[float],
        pose: TransitionPose,
        *,
        profile_max_joint_delta_rad: float,
        allow_segmented: bool = False,
        profile_segmented_max_joint_delta_rad: float | None = None,
    ) -> float:
        current = tuple(float(item) for item in current_arm_joint_position_rad)
        if len(current) != 12 or not all(math.isfinite(item) for item in current):
            raise ValueError("transition move requires a finite measured 12D arm state")
        maximum = max(
            abs(target - measured)
            for target, measured in zip(
                pose.arm_joint_position_rad, current, strict=True
            )
        )
        bound = min(self.max_joint_delta_rad, float(profile_max_joint_delta_rad))
        if maximum <= bound:
            return maximum
        if not allow_segmented or self.segmented_max_joint_delta_rad is None:
            raise ValueError(
                f"transition pose requires {maximum:.3f} rad joint change; "
                f"limit is {bound:.3f} rad"
            )
        if profile_segmented_max_joint_delta_rad is None:
            raise ValueError(
                "segmented transition is enabled without a profile total bound"
            )
        segmented_bound = min(
            self.segmented_max_joint_delta_rad,
            float(profile_segmented_max_joint_delta_rad),
        )
        if not bound < segmented_bound <= math.pi or maximum > segmented_bound:
            raise ValueError(
                f"transition pose requires {maximum:.3f} rad joint change; "
                f"segmented limit is {segmented_bound:.3f} rad"
            )
        return maximum

    @staticmethod
    def segmented_waypoints(
        current_arm_joint_position_rad: Sequence[float],
        target_arm_joint_position_rad: Sequence[float],
        *,
        max_segment_joint_delta_rad: float,
        max_waypoints: int,
    ) -> tuple[tuple[float, ...], ...]:
        """Build bounded joint-space waypoints ending exactly at the target.

        The owner validates each segment's interpolated flange height before
        dispatch. This helper only splits a validated allowlisted target; it
        never invents a target pose or changes joint units.
        """
        current = tuple(float(item) for item in current_arm_joint_position_rad)
        target = tuple(float(item) for item in target_arm_joint_position_rad)
        if (
            len(current) != 12
            or len(target) != 12
            or not all(math.isfinite(item) for item in (*current, *target))
        ):
            raise ValueError("segmented transition requires finite 12D arm states")
        segment_bound = float(max_segment_joint_delta_rad)
        waypoint_limit = int(max_waypoints)
        if not 0.0 < segment_bound <= math.pi or waypoint_limit < 1:
            raise ValueError("segmented transition waypoint contract is invalid")
        maximum = max(
            abs(finish - start)
            for start, finish in zip(current, target, strict=True)
        )
        count = max(1, math.ceil(maximum / segment_bound))
        if count > waypoint_limit:
            raise ValueError(
                f"transition requires {count} waypoints; limit is {waypoint_limit}"
            )
        waypoints: list[tuple[float, ...]] = []
        for index in range(1, count + 1):
            if index == count:
                waypoints.append(target)
            else:
                waypoints.append(
                    tuple(
                        start + (finish - start) * (index / count)
                        for start, finish in zip(current, target, strict=True)
                    )
                )
        return tuple(waypoints)

    @staticmethod
    def segmented_route(
        current_arm_joint_position_rad: Sequence[float],
        via_arm_joint_position_rad: Sequence[float],
        target_arm_joint_position_rad: Sequence[float],
        *,
        max_segment_joint_delta_rad: float,
        max_waypoints_per_leg: int,
    ) -> tuple[tuple[float, ...], ...]:
        """Build a bounded two-leg route through an explicit safe waypoint.

        The intermediate and final targets remain caller-supplied poses.  This
        helper only splits each leg into direct-bound joint-space segments; a
        robot owner must still validate every segment against its kinematic and
        hardware safety guards before dispatch.
        """
        first = TransitionPoseLibrary.segmented_waypoints(
            current_arm_joint_position_rad,
            via_arm_joint_position_rad,
            max_segment_joint_delta_rad=max_segment_joint_delta_rad,
            max_waypoints=max_waypoints_per_leg,
        )
        second = TransitionPoseLibrary.segmented_waypoints(
            via_arm_joint_position_rad,
            target_arm_joint_position_rad,
            max_segment_joint_delta_rad=max_segment_joint_delta_rad,
            max_waypoints=max_waypoints_per_leg,
        )
        return (*first, *second)

    def status(self) -> dict[str, Any]:
        return {
            "library_id": self.library_id,
            "sha256": self.sha256,
            "path": str(self.path),
            "pose_count": len(self.poses),
            "dataset_artifact_id": self.dataset.get("artifact_id"),
            "runtime_max_joint_delta_rad": self.max_joint_delta_rad,
            "segmented_transition_max_joint_delta_rad": self.segmented_max_joint_delta_rad,
        }
