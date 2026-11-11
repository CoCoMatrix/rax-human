"""Offline v10/owner geometry audit; no SDK, graph or device access."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .contracts import FIELDS, PiperContract, require
from .description import CAMERA_FRAMES, MAST_JOINT, PiperDescription
from .kinematics import PiperKinematics


def audit(contract: PiperContract, asset_root: Path, *, samples: int = 512) -> dict:
    require(type(samples) is int and 1 <= samples <= 4096, "audit samples must be 1..4096")
    description, kin = PiperDescription(contract, asset_root), PiperKinematics(contract)
    rng = np.random.default_rng(20260918)
    states = [np.zeros(14), *rng.uniform(contract.lower14, contract.upper14, size=(samples, 14))]
    position_error = orientation_error = matrix_error = 0.0
    outside_v10 = 0
    for state in states:
        frames = description.frames(state, mast_joint_rad=None)
        outside_v10 += bool(frames["description_limit_warnings"])
        owner = kin.forward(state)
        for side, target in (("left", owner.left_transform), ("right", owner.right_transform)):
            actual = np.asarray(frames["flanges"][side]["transform"])
            position_error = max(position_error, float(np.linalg.norm(actual[:3, 3] - target[:3, 3])))
            orientation_error = max(orientation_error, float(np.linalg.norm(
                Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).as_rotvec())))
            matrix_error = max(matrix_error, float(np.max(np.abs(actual - target))))
    require(position_error <= 1e-10 and orientation_error <= 1e-10,
            "description and authoritative owner FK are not equivalent")
    nominal_zero = description.frames(np.zeros(14), mast_joint_rad=0.0)
    user_estimated = description.frames(np.zeros(14), mast_joint_rad=0.0)
    model = description._model
    fixed = {}
    for side in ("left", "right"):
        for child in (f"{side}_flange_link", f"{side}_gripper_base",
                      f"{side}_d435_link", f"{side}_d435_color_optical_frame",
                      f"{side}_d435_depth_optical_frame"):
            fixed[f"{side}_link6 -> {child}"] = model.get_transform(child, f"{side}_link6").tolist()
    return {"schema": "junqi.piper_v10_offline_audit.v1", "identity": description.identity,
            "arm_kinematic_comparison": description.comparison,
            "fk_equivalence": {"method": "yourdfpy v10 versus authoritative MuJoCo/Mink owner FK",
                               "seed": 20260918, "random_samples": samples,
                               "total_dual_arm_states": len(states), "frame_comparisons": len(states) * 2,
                               "sampling_domain": "full authoritative owner command limits",
                               "states_outside_v10_limits_used_only_for_geometry": outside_v10,
                               "max_translation_residual_m": position_error,
                               "max_rotation_residual_rad": orientation_error,
                               "max_matrix_element_residual": matrix_error,
                               "root_transform_required": np.eye(4).tolist(), "passed": True},
            "intersection_command_limits": {name: [float(a), float(b)] for name, a, b in
                                            zip(FIELDS, description.lower14, description.upper14, strict=True)},
            "fixed_nominal_transforms": fixed, "camera_role_to_rgb_optical_link": CAMERA_FRAMES,
            "mast_joint": description.joints[MAST_JOINT],
            "mast_user_estimate": {"total_forward_tilt_degrees": 75, "joint_degrees": 0,
                                   "approximate": True, "source": "user_estimate_not_encoder_measurement"},
            "zero_pose_nominal_mast_zero": nominal_zero,
            "zero_pose_fixed_mast_75deg": user_estimated,
            "gripper_mimics": description.mimics,
            "tcp": {"defined": False, "reason": "no TCP link exists; gripper_link is a virtual command joint",
                    "finger_link_origin_midpoint_offset_from_link6_z_m": 0.1425,
                    "midpoint_is_not_a_calibrated_contact_tip": True},
            "limitations": ["URDF geometry is not camera hand-eye calibration or intrinsic calibration",
                            "global RGB pose uses the fixed 75-degree v10 mast",
                            "no camera/arm device bindings or encoders were accessed by this audit",
                            "meshes/collision elements do not constitute collision certification",
                            "owner profile and URDF are unchanged; description limits additionally restrict candidates"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--asset-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=512)
    args = parser.parse_args()
    result = audit(PiperContract.from_runtime(args.runtime), args.asset_root, samples=args.samples)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result["fk_equivalence"], sort_keys=True))


if __name__ == "__main__":
    main()
