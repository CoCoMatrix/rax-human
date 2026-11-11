"""Checked, nominal extra frames from Junqi's PiPER v12 description asset.

This is observation/geometry code. It neither changes the authoritative owner
URDF nor opens hardware, adds a controller DOF, or certifies collision/TCP/
camera calibration. The mast is fixed at 90 degrees from vertical in the URDF.
"""
from __future__ import annotations

from collections.abc import Mapping
import json
from pathlib import Path, PurePosixPath
import threading
import xml.etree.ElementTree as ET

import numpy as np

from .contracts import (FIELDS, JOINT_FIELDS, ContractError, PiperContract,
                        _sha256, require)

ARCHIVE_SHA256 = "bc80ea36f4b9036296fa3c6db171bf69e22c1462368d5d05d21dbb2aedd4994e"
URDF_SHA256 = "53cb1afc01523b03f073f9462c57a314ca0ab6176d384a006af1cd100563112f"
ACCEPTED_URDF_REVISIONS = {URDF_SHA256: "v12", "12e4e447bffcbbfebca677fb96fd6062d122b1c2987da65187aaa3caa1bb330d": "v13_32cm"}
URDF_RELATIVE_PATH = "dual_piper_x_description/urdf/dual_piper_x.urdf"
MAST_JOINT = "global_camera_mast_upper_joint"
CAMERA_FRAMES = {"camera_head": "global_d435_color_optical_frame",
                 "camera_left": "left_d435_color_optical_frame",
                 "camera_right": "right_d435_color_optical_frame"}


def _numbers(element, key: str, default: str) -> list[float]:
    return [float(value) for value in (default if element is None else element.get(key, default)).split()]


def _joint_info(joint) -> dict:
    origin, axis, limit = joint.find("origin"), joint.find("axis"), joint.find("limit")
    result = {"name": joint.attrib["name"], "type": joint.attrib["type"],
              "parent": joint.find("parent").attrib["link"],
              "child": joint.find("child").attrib["link"],
              "xyz_m": _numbers(origin, "xyz", "0 0 0"),
              "rpy_rad": _numbers(origin, "rpy", "0 0 0"),
              "axis": _numbers(axis, "xyz", "1 0 0")}
    if limit is not None:
        result["limits"] = {key: float(value) for key, value in limit.attrib.items()}
    mimic = joint.find("mimic")
    if mimic is not None:
        result["mimic"] = {"joint": mimic.attrib["joint"],
                           "multiplier": float(mimic.get("multiplier", "1")),
                           "offset": float(mimic.get("offset", "0"))}
    return result


def _joint_map(root) -> dict[str, dict]:
    joints = [_joint_info(joint) for joint in root.findall("joint")]
    require(len({joint["name"] for joint in joints}) == len(joints), "duplicate description joint names")
    return {joint["name"]: joint for joint in joints}


class PiperDescription:
    def __init__(self, contract: PiperContract, asset_root: str | Path):
        self.contract = contract
        contract.verify_identity()
        self.asset_root = Path(asset_root).resolve(strict=True)
        self.manifest_path = self.asset_root / "asset-manifest.json"
        manifest = json.loads(self.manifest_path.read_text())
        require(manifest.get("schema") == "junqi.robot_description.v1", "description manifest schema mismatch")
        require(manifest.get("archive_sha256") == ARCHIVE_SHA256, "description archive identity mismatch")
        files = manifest.get("files")
        require(isinstance(files, list) and files, "description manifest file list missing")
        seen = set()
        for record in files:
            require(isinstance(record, Mapping), "invalid description manifest file record")
            name = record.get("path")
            require(isinstance(name, str), "invalid description asset path")
            relative = PurePosixPath(name)
            require(not relative.is_absolute() and ".." not in relative.parts
                    and "\\" not in name and name not in seen, "unsafe or duplicate description asset path")
            seen.add(name)
            path = self.asset_root.joinpath(*relative.parts)
            require(path.resolve(strict=True).is_relative_to(self.asset_root) and not path.is_symlink(),
                    "description asset escapes selected directory")
            require(path.is_file() and path.stat().st_size == record.get("bytes")
                    and _sha256(path) == record.get("sha256"), f"description asset hash mismatch: {name}")
        require(URDF_RELATIVE_PATH in seen, "description URDF missing from manifest")
        self.urdf_path = self.asset_root / URDF_RELATIVE_PATH
        selected_sha = _sha256(self.urdf_path)
        require(selected_sha in ACCEPTED_URDF_REVISIONS, "description URDF identity mismatch")
        revision = ACCEPTED_URDF_REVISIONS[selected_sha]
        root, owner_root = ET.parse(self.urdf_path).getroot(), ET.parse(contract.urdf_path).getroot()
        require(root.attrib.get("name") == owner_root.attrib.get("name") == "dual_piper_x",
                "description robot name mismatch")
        self.joints, owner_joints = _joint_map(root), _joint_map(owner_root)
        require(self.joints[MAST_JOINT]["type"] == "fixed", "v12 mast must be fixed")
        self.comparison = []
        for name in (*JOINT_FIELDS, "left_mount_joint", "right_mount_joint"):
            require(name in self.joints and name in owner_joints, f"missing owner chain joint: {name}")
            supplied, owner = self.joints[name], owner_joints[name]
            for key in ("type", "parent", "child", "xyz_m", "rpy_rad", "axis"):
                require(supplied[key] == owner[key], f"v12/owner kinematic mismatch: {name}.{key}")
            row = {"joint": name, "origin_axis_parent_child_identical": True}
            if "limits" in owner:
                row.update(owner_limits_rad=[owner["limits"]["lower"], owner["limits"]["upper"]],
                           description_limits_rad=[supplied["limits"]["lower"], supplied["limits"]["upper"]])
                row["max_limit_difference_rad"] = max(abs(a - b) for a, b in
                    zip(row["owner_limits_rad"], row["description_limits_rad"], strict=True))
            self.comparison.append(row)
        expected_mimic = {f"{side}_gripper_joint{index}": {
            "joint": f"{side}_gripper", "multiplier": 0.5 if index == 1 else -0.5, "offset": 0.0}
            for side in ("left", "right") for index in (1, 2)}
        self.mimics = {name: joint["mimic"] for name, joint in self.joints.items() if "mimic" in joint}
        require(self.mimics == expected_mimic, "description gripper mimic contract mismatch")
        description_limits = np.asarray([[self.joints[name]["limits"]["lower"],
                                          self.joints[name]["limits"]["upper"]] for name in FIELDS])
        self.lower14 = np.maximum(contract.lower14, description_limits[:, 0])
        self.upper14 = np.minimum(contract.upper14, description_limits[:, 1])
        require(bool(np.all(self.lower14 < self.upper14)), "owner/description limit intersection is empty")
        self.lower14.setflags(write=False)
        self.upper14.setflags(write=False)
        self.identity = {"asset_schema": "junqi.piper_description." + revision,
                         "archive_sha256": ARCHIVE_SHA256, "description_urdf_sha256": selected_sha,
                         "asset_manifest_sha256": _sha256(self.manifest_path),
                         "verified_asset_file_count": len(files),
                         "owner_profile_sha256": contract.identity["profile_sha256"],
                         "owner_urdf_sha256": contract.identity["urdf_sha256"],
                         "robot_id": contract.robot_id, "coordinate_frame": "dual_base_link",
                         "owner_to_description_root_transform": np.eye(4).tolist(),
                         "camera_calibration_status": "nominal_urdf_not_calibrated",
                         "tcp_calibrated": False, "collision_certified": False}
        from yourdfpy import URDF

        self._model = URDF.load(str(self.urdf_path), load_meshes=False, load_collision_meshes=False,
                                build_scene_graph=True, build_collision_scene_graph=False)
        require(self._model.base_link == "dual_base_link", "description root frame mismatch")
        require(set(self._model.actuated_joint_names) == set(FIELDS),
                "description independent joint set differs from 14D with a fixed camera mast")
        self._lock = threading.RLock()

    def verify_identity(self) -> None:
        self.contract.verify_identity()
        require(_sha256(self.manifest_path) == self.identity["asset_manifest_sha256"],
                "description manifest changed since initialization")
        require(_sha256(self.urdf_path) == self.identity["description_urdf_sha256"],
                "description URDF changed since initialization")

    def _mast_value(self, mast_joint_rad: float | None) -> float | None:
        if mast_joint_rad is None:
            return 0.0
        require(isinstance(mast_joint_rad, (int, float, np.floating)) and not isinstance(mast_joint_rad, bool)
                and np.isfinite(mast_joint_rad), "mast_joint_rad must be a finite explicit angle or None")
        value = float(mast_joint_rad)
        require(self.joints[MAST_JOINT]["type"] == "fixed" and value == 0.0,
                "v12 mast is fixed; additional mast angle must be zero")
        return value

    def joint_configuration(self, state14: np.ndarray, *, mast_joint_rad: float | None) -> dict:
        state = self.contract.validate_measured_state(state14)
        mast = self._mast_value(mast_joint_rad)
        actuated = {name: float(value) for name, value in zip(FIELDS, state, strict=True)}
        mimic = {name: actuated[item["joint"]] * item["multiplier"] + item["offset"]
                 for name, item in self.mimics.items()}
        return {"actuated": actuated, "mimic": mimic}

    def _limit_violations(self, values: np.ndarray) -> list[dict]:
        bad = np.argwhere((values < self.lower14) | (values > self.upper14))
        return [{"step": int(row), "field": FIELDS[col], "value": float(values[row, col]),
                 "intersection_lower": float(self.lower14[col]),
                 "intersection_upper": float(self.upper14[col])} for row, col in bad]

    def validate_candidate_limits(self, actions: np.ndarray) -> dict:
        """Check the supplied values against owner ∩ v12; never normalize or cast them."""
        self.verify_identity()
        require(isinstance(actions, np.ndarray) and actions.dtype.kind == "f" and actions.ndim == 2
                and actions.shape[1] == 14 and len(actions) > 0 and bool(np.isfinite(actions).all()),
                "description candidate must be finite floating ndarray[N,14]")
        violations = self._limit_violations(actions)
        if violations:
            first = violations[0]
            raise ContractError("candidate exceeds owner/v12 intersection limits: " + json.dumps(first))
        return {"valid": True, "checked_rows": len(actions), "limit_source": "owner_intersection_" + self.identity["asset_schema"].split(".")[-1],
                "description_urdf_sha256": self.identity["description_urdf_sha256"]}

    def frames(self, state14: np.ndarray, *, mast_joint_rad: float | None) -> dict:
        self.verify_identity()
        configuration = self.joint_configuration(state14, mast_joint_rad=mast_joint_rad)
        mast = self._mast_value(mast_joint_rad)
        state = self.contract.validate_measured_state(state14)
        with self._lock:
            self._model.update_cfg(configuration["actuated"])
            def pose(link: str, *, nominal_camera: bool = False) -> dict:
                from scipy.spatial.transform import Rotation

                transform = np.array(self._model.get_transform(link, "dual_base_link"), copy=True)
                require(transform.shape == (4, 4) and bool(np.isfinite(transform).all()),
                        "description FK returned invalid transform")
                xyzw = Rotation.from_matrix(transform[:3, :3]).as_quat()
                return {"link": link, "coordinate_frame": "dual_base_link", "transform": transform.tolist(),
                        "position_m": transform[:3, 3].tolist(), "quaternion_wxyz": xyzw[[3, 0, 1, 2]].tolist(),
                        "calibration_status": "nominal_urdf_not_calibrated" if nominal_camera else "urdf_geometry",
                        "is_tcp": False}

            flanges = {side: pose(f"{side}_link6") for side in ("left", "right")}
            gripper_bases = {side: pose(f"{side}_gripper_base") for side in ("left", "right")}
            fingers = {side: [pose(f"{side}_gripper_link{i}") for i in (1, 2)] for side in ("left", "right")}
            cameras = {role: pose(link, nominal_camera=True) for role, link in CAMERA_FRAMES.items()}
        return {"schema": "junqi.piper_description_frames.v1", "identity": dict(self.identity),
                "coordinate_frame": "dual_base_link", "state_fields": list(FIELDS),
                "input_state14": state.tolist(), "joint_configuration": configuration,
                "description_limit_warnings": self._limit_violations(state[None, :]),
                "state_was_clipped": False, "flanges": flanges, "gripper_bases": gripper_bases,
                "finger_link_origins": fingers, "rgb_optical_frames": cameras,
                "camera_frame_semantics": "optical_x_right_y_down_z_forward",
                "mast": {"joint_name": MAST_JOINT, "joint_rad": mast,
                         "total_forward_tilt_rad": None if mast is None else mast + self.joints[MAST_JOINT]["rpy_rad"][1],
                         "joint_type": "fixed", "source": "fixed_" + self.identity["asset_schema"].split(".")[-1] + "_urdf_nominal"},
                "tcp_defined": False, "camera_extrinsics_calibrated": False, "collision_certified": False}

# Compatibility for external callers; loads and verifies only v12 assets.
PiperV9Description = PiperDescription
