"""Metric geometry for a rectified IR1-left / IR2-right stereo pair.

No hardware, model, or robot action is opened here. All transformations use
column-vector semantics: ``T_target_from_source @ [x, y, z, 1]``. Lengths are
metres, disparity is ``u_left - u_right`` in pixels at the supplied resolution,
and every camera frame uses optical x-right / y-down / z-forward coordinates.
The collector is responsible for serial/profile identity and frame freshness.
Validity masks are geometric validity, never model confidence or accuracy.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

import numpy as np

from .contracts import ContractError, require


def _finite_scalar(value, name):
    require(isinstance(value, (int, float, np.number)) and not isinstance(value, (bool, np.bool_))
            and np.isfinite(value), f"{name} must be a finite numeric scalar")
    return float(value)


def validate_se3(transform, name="transform"):
    """Validate and copy a proper rigid transform; never repair its rotation."""
    array = np.asarray(transform)
    require(array.shape == (4, 4) and array.dtype.kind in "fi"
            and bool(np.isfinite(array).all()), f"{name} must be a finite numeric 4x4 SE3")
    array = np.array(array, dtype=np.float64, copy=True)
    rotation = array[:3, :3]
    require(np.allclose(array[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            and abs(np.linalg.det(rotation) - 1) <= 1e-6, f"{name} is not a proper SE3")
    return array


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str = "none"
    coeffs: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def __post_init__(self):
        for name in ("width", "height"):
            value = getattr(self, name)
            require(isinstance(value, (int, np.integer)) and not isinstance(value, (bool, np.bool_))
                    and value > 0, f"{name} must be a positive integer")
            object.__setattr__(self, name, int(value))
        for name in ("fx", "fy", "cx", "cy"):
            object.__setattr__(self, name, _finite_scalar(getattr(self, name), name))
        require(self.fx > 0 and self.fy > 0, "focal lengths must be positive")
        model = self.distortion_model
        require(isinstance(model, str), "distortion model must be an explicit string")
        model = model.removeprefix("distortion.").lower()
        require(model in ("none", "brown_conrady", "modified_brown_conrady", "inverse_brown_conrady"),
                f"unsupported distortion model {model!r}; provide an explicitly supported projection model")
        object.__setattr__(self, "distortion_model", model)
        coeffs = np.asarray(self.coeffs)
        require(coeffs.ndim == 1 and coeffs.dtype.kind in "fi" and len(coeffs) in (0, 5)
                and bool(np.isfinite(coeffs).all()), "coeffs must be five finite [k1,k2,p1,p2,k3] values")
        if len(coeffs) == 0:
            require(model == "none", "Brown-Conrady requires five explicit coefficients")
            coeffs = np.zeros(5)
        require(model != "none" or bool(np.all(coeffs == 0)), "none distortion cannot contain nonzero coefficients")
        object.__setattr__(self, "coeffs", tuple(float(x) for x in coeffs))

    @classmethod
    def from_dict(cls, item: Mapping):
        """Consume exported SDK fields, requiring explicit distortion metadata."""
        require(isinstance(item, Mapping), "intrinsics must be a mapping")
        try:
            if "K" in item:
                matrix = np.asarray(item["K"])
                require(matrix.shape == (3, 3) and matrix.dtype.kind in "fi" and np.isfinite(matrix).all(),
                        "K must be finite numeric 3x3")
                require(np.allclose(matrix[2], [0, 0, 1], atol=1e-12, rtol=0)
                        and abs(matrix[0, 1]) <= 1e-12 and abs(matrix[1, 0]) <= 1e-12,
                        "K requires zero skew and canonical bottom row")
                fx, fy, cx, cy = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
                for key, expected in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy), ("ppx", cx), ("ppy", cy)):
                    require(key not in item or abs(_finite_scalar(item[key], key) - expected) <= 1e-9,
                            f"{key} conflicts with exported K")
            else:
                fx, fy = item["fx"], item["fy"]
                cx = item["cx"] if "cx" in item else item["ppx"]
                cy = item["cy"] if "cy" in item else item["ppy"]
            model = item["distortion_model"] if "distortion_model" in item else item["model"]
            coeffs = tuple(item["distortion_coeffs"] if "distortion_coeffs" in item else item["coeffs"])
            require("distortion_coeffs" not in item or "coeffs" not in item or tuple(item["coeffs"]) == coeffs,
                    "conflicting distortion coefficients")
            return cls(item["width"], item["height"], fx, fy, cx, cy, model, coeffs)
        except (KeyError, TypeError) as error:
            raise ContractError(f"incomplete intrinsics: {error}") from error

    @property
    def matrix(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64)

    def resized(self, width: int, height: int):
        """Intrinsics after a half-pixel-centred resize (e.g. cv2.resize).

        This does not resize images/disparity. Disparity must also be multiplied
        by the horizontal scale. Crops/padding need their own explicit offsets.
        Do not use this for a different physical stream profile; export that
        profile's actual factory intrinsics instead.
        """
        require(isinstance(width, (int, np.integer)) and not isinstance(width, (bool, np.bool_))
                and isinstance(height, (int, np.integer)) and not isinstance(height, (bool, np.bool_))
                and width > 0 and height > 0, "resize dimensions must be positive integers")
        sx, sy = width / self.width, height / self.height
        return CameraIntrinsics(width, height, self.fx * sx, self.fy * sy,
                                (self.cx + 0.5) * sx - 0.5, (self.cy + 0.5) * sy - 0.5,
                                self.distortion_model, self.coeffs)


@dataclass(frozen=True)
class RectifiedStereo:
    """Calibrated rectified pair; baseline is derived, never guessed.

    IR1 is left and IR2 is right. A point in the left frame transforms to the
    right frame with x translation ``-baseline_m``. Arbitrary raw stereo
    extrinsics must first be rectified together with the images/intrinsics.
    """
    left: CameraIntrinsics
    right: CameraIntrinsics
    T_right_from_left: np.ndarray

    def __post_init__(self):
        require(isinstance(self.left, CameraIntrinsics) and isinstance(self.right, CameraIntrinsics),
                "left and right intrinsics must be CameraIntrinsics")
        a, b = self.left, self.right
        require((a.width, a.height) == (b.width, b.height), "rectified IR resolutions must agree")
        require(a.distortion_model in ("none", "brown_conrady") and b.distortion_model in ("none", "brown_conrady"),
                "rectified IR requires explicit none/Brown-Conrady calibration")
        require(all(c == 0 for c in a.coeffs + b.coeffs), "rectified IR images must have zero residual distortion")
        require(np.allclose([a.fx, a.fy, a.cy], [b.fx, b.fy, b.cy], atol=1e-5, rtol=1e-7),
                "rectified pair must share fx, fy and cy; cx may differ")
        transform = validate_se3(self.T_right_from_left, "T_right_from_left")
        require(np.allclose(transform[:3, :3], np.eye(3), atol=1e-5, rtol=0)
                and np.allclose(transform[1:3, 3], 0, atol=1e-6, rtol=0),
                "pair is not rectified; explicitly rectify both images and calibration first")
        require(transform[0, 3] < 0, "IR1-left to IR2-right translation must be negative x; baseline cannot be guessed")
        transform.setflags(write=False)
        object.__setattr__(self, "T_right_from_left", transform)

    @property
    def baseline_m(self):
        return float(-self.T_right_from_left[0, 3])

    def resized(self, width: int, height: int):
        return RectifiedStereo(self.left.resized(width, height), self.right.resized(width, height),
                               self.T_right_from_left)


class StereoRectifier:
    """Explicit OpenCV rectification of a factory-calibrated horizontal pair.

    Output resolution equals input resolution. alpha=1 retains the available
    field of view, including invalid borders; no crop or downstream resize is
    hidden. Intrinsics can change during rectification and are returned in
    ``stereo``. Use ``disparity_valid_mask`` to exclude invalid support before
    metric triangulation. Raw and rectified optical frames are distinct.
    """
    def __init__(self, left: CameraIntrinsics, right: CameraIntrinsics,
                 T_right_from_left, T_rgb_from_left):
        require(isinstance(left, CameraIntrinsics) and isinstance(right, CameraIntrinsics),
                "left and right must be CameraIntrinsics")
        require((left.width, left.height) == (right.width, right.height),
                "raw stereo resolutions must agree")
        require(left.distortion_model in ("none", "brown_conrady")
                and right.distortion_model in ("none", "brown_conrady"),
                "OpenCV IR rectification supports only none/Brown-Conrady; "
                "modified/inverse Brown-Conrady cannot be silently treated as ordinary Brown")
        self.raw_left, self.raw_right = left, right
        raw_stereo = validate_se3(T_right_from_left, "T_right_from_left")
        raw_rgb = validate_se3(T_rgb_from_left, "T_rgb_from_left")
        require(raw_stereo[0, 3] < 0 and abs(raw_stereo[0, 3]) > abs(raw_stereo[1, 3]),
                "IR1-left / IR2-right must be a horizontal negative-x-baseline pair")
        try:
            import cv2
        except ImportError as error:
            raise ContractError("stereo rectification requires opencv-python") from error
        size = (left.width, left.height)
        r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
            left.matrix, np.asarray(left.coeffs), right.matrix, np.asarray(right.coeffs), size,
            np.ascontiguousarray(raw_stereo[:3, :3]),
            np.ascontiguousarray(raw_stereo[:3, 3]).reshape(3, 1),
            flags=0, alpha=1, newImageSize=size)
        self.R_leftrect_from_left = r1
        self.R_rightrect_from_right = r2
        rect_stereo = np.eye(4)
        rect_stereo[:3, :3] = r2 @ raw_stereo[:3, :3] @ r1.T
        rect_stereo[:3, 3] = r2 @ raw_stereo[:3, 3]
        rect_left = CameraIntrinsics(*size, p1[0, 0], p1[1, 1], p1[0, 2], p1[1, 2])
        rect_right = CameraIntrinsics(*size, p2[0, 0], p2[1, 1], p2[0, 2], p2[1, 2])
        self.stereo = RectifiedStereo(rect_left, rect_right, rect_stereo)
        require(abs(-p2[0, 3] / p2[0, 0] - self.stereo.baseline_m) < 1e-8,
                "rectified projection and extrinsic baseline disagree")
        left_from_leftrect = np.eye(4)
        left_from_leftrect[:3, :3] = r1.T
        self.T_rgb_from_leftrect = validate_se3(raw_rgb @ left_from_leftrect,
                                               "T_rgb_from_leftrect")
        self._left_maps = cv2.initUndistortRectifyMap(left.matrix, np.asarray(left.coeffs), r1,
                                                     p1[:3, :3], size, cv2.CV_32FC1)
        self._right_maps = cv2.initUndistortRectifyMap(right.matrix, np.asarray(right.coeffs), r2,
                                                      p2[:3, :3], size, cv2.CV_32FC1)

        def support(maps):
            x, y = maps
            return np.isfinite(x) & np.isfinite(y) & (x >= 0) & (x <= left.width - 1) & (y >= 0) & (y <= left.height - 1)

        self.left_valid_mask = support(self._left_maps)
        self.right_valid_mask = support(self._right_maps)
        for array in (r1, r2, self.T_rgb_from_leftrect, self.left_valid_mask, self.right_valid_mask):
            array.setflags(write=False)

    def rectify_pair(self, left_u8, right_u8):
        import cv2

        shape = (self.raw_left.height, self.raw_left.width)
        images = [np.asarray(left_u8), np.asarray(right_u8)]
        require(all(image.shape == shape and image.dtype == np.uint8 for image in images),
                "IR input must be two uint8 grayscale HxW images at the calibrated profile resolution")
        result = [cv2.remap(image, *maps, interpolation=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                  for image, maps in zip(images, (self._left_maps, self._right_maps), strict=True)]
        return {"left_rectified": result[0], "right_rectified": result[1],
                "left_valid_mask": self.left_valid_mask.copy(), "right_valid_mask": self.right_valid_mask.copy()}

    def disparity_valid_mask(self, disparity):
        """Reject borders and right-image match points without valid support."""
        disparity = np.asarray(disparity)
        shape = (self.raw_left.height, self.raw_left.width)
        require(disparity.shape == shape and disparity.dtype.kind == "f",
                "disparity must be floating HxW at rectified resolution")
        v, u = np.indices(shape)
        right_u = u - disparity
        valid = self.left_valid_mask & np.isfinite(disparity) & (right_u >= 0) & (right_u <= shape[1] - 1)
        safe_u = np.where(valid, right_u, 0)
        lo, hi = np.floor(safe_u).astype(np.int64), np.ceil(safe_u).astype(np.int64)
        return valid & self.right_valid_mask[v, lo] & self.right_valid_mask[v, hi]


def disparity_to_left_xyz(disparity, stereo: RectifiedStereo, *, min_depth_m=0.0, max_depth_m=None):
    """Triangulate with Z=fx*baseline/(disp-(cx_left-cx_right)).

    Nonfinite/nonpositive corrected disparity and depth outside explicit bounds
    become holes (NaN + False). Negative *raw* disparity is possible with a cx
    offset and is not confused with negative corrected disparity.
    """
    require(isinstance(stereo, RectifiedStereo), "stereo must be RectifiedStereo")
    disparity = np.asarray(disparity)
    intr = stereo.left
    require(disparity.shape == (intr.height, intr.width) and disparity.dtype.kind == "f",
            "disparity must be a floating HxW array at the calibrated resolution in pixels")
    near = _finite_scalar(min_depth_m, "min_depth_m")
    require(near >= 0, "min_depth_m must be nonnegative")
    far = math.inf if max_depth_m is None else _finite_scalar(max_depth_m, "max_depth_m")
    require(far > near, "max_depth_m must exceed min_depth_m")
    corrected = np.asarray(disparity, dtype=np.float64) - (intr.cx - stereo.right.cx)
    valid = np.isfinite(corrected) & (corrected > 0)
    depth = np.full(disparity.shape, np.nan, dtype=np.float64)
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        np.divide(intr.fx * stereo.baseline_m, corrected, out=depth, where=valid)
    valid &= np.isfinite(depth) & (depth > near) & (depth <= far)
    v, u = np.indices(depth.shape, dtype=np.float64)
    xyz = np.stack(((u - intr.cx) * depth / intr.fx, (v - intr.cy) * depth / intr.fy, depth), axis=-1)
    valid &= np.isfinite(xyz).all(axis=-1)
    depth[~valid] = np.nan
    xyz[~valid] = np.nan
    return {"xyz_left_m": xyz, "depth_left_m": depth, "valid_mask": valid,
            "coordinate_frame": "left_rectified_optical", "depth_semantics": "optical_z_m",
            "mask_semantics": "geometric_validity_not_confidence"}


def _xyz_and_mask(xyz, valid_mask):
    xyz, mask = np.asarray(xyz), np.asarray(valid_mask)
    require(xyz.ndim == 3 and xyz.shape[2] == 3 and xyz.dtype.kind == "f"
            and min(xyz.shape[:2]) > 0, "xyz must be floating HxWx3")
    require(mask.shape == xyz.shape[:2] and mask.dtype == np.bool_, "valid_mask must be bool HxW matching xyz")
    return np.asarray(xyz, dtype=np.float64), mask & np.isfinite(xyz).all(axis=-1)


def transform_xyz(xyz, valid_mask, T_target_from_source):
    """Transform a point image, including into dual_base_link using URDF RGB pose.

    Base-frame z can be negative and is not a depth-validity condition. No TCP
    or motion target is inferred. Callers must retain nominal-URDF provenance.
    """
    xyz, valid = _xyz_and_mask(xyz, valid_mask)
    transform = validate_se3(T_target_from_source)
    transformed = np.full(xyz.shape, np.nan, dtype=np.float64)
    transformed[valid] = xyz[valid] @ transform[:3, :3].T + transform[:3, 3]
    valid &= np.isfinite(transformed).all(axis=-1)
    transformed[~valid] = np.nan
    return {"xyz_m": transformed, "valid_mask": valid}


def align_left_xyz_to_rgb(xyz_left_m, valid_mask, T_rgb_from_left, rgb_intrinsics: CameraIntrinsics):
    """Forward project through full extrinsics, RGB distortion and a z-buffer.

    Each source point splats to its nearest RGB pixel. Overlaps keep the
    smallest positive RGB-optical z; ties keep source raster order. Holes are
    NaN/False, with no interpolation or invented confidence. xyz_rgb_m stores
    the original 3D point that won the z-buffer (not a pixel-centre ray).
    For rectified input XYZ, T_rgb_from_left must be the rectifier's
    T_rgb_from_leftrect, not the unrectified SDK extrinsic.
    """
    require(isinstance(rgb_intrinsics, CameraIntrinsics), "rgb_intrinsics must be CameraIntrinsics")
    xyz, valid = _xyz_and_mask(xyz_left_m, valid_mask)
    valid &= xyz[..., 2] > 0
    transformed = transform_xyz(xyz, valid, validate_se3(T_rgb_from_left, "T_rgb_from_left"))
    target = transformed["xyz_m"]
    valid = transformed["valid_mask"] & (target[..., 2] > 0)
    flat_indices = np.flatnonzero(valid.ravel())
    points = target.reshape(-1, 3)[flat_indices]
    intr = rgb_intrinsics
    output = np.full((intr.height, intr.width, 3), np.nan, dtype=np.float64)
    source_index = np.full((intr.height, intr.width), -1, dtype=np.int64)
    if len(points):
        if intr.distortion_model == "brown_conrady":
            try:
                import cv2
            except ImportError as error:
                raise ContractError("RGB Brown-Conrady projection requires opencv-python") from error
            projected, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), intr.matrix,
                                             np.asarray(intr.coeffs))
            uv = projected.reshape(-1, 2)
        elif intr.distortion_model in ("modified_brown_conrady", "inverse_brown_conrady"):
            # Match librealsense rs2_project_point_to_pixel, rs.cpp lines 3938+
            # (official source checked 2026-09-18): radial scaling happens
            # before tangential products, unlike OpenCV's ordinary Brown.
            # https://github.com/realsenseai/librealsense/blob/master/src/rs.cpp
            with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
                x, y = points[:, 0] / points[:, 2], points[:, 1] / points[:, 2]
                r2 = x * x + y * y
                k1, k2, p1, p2, k3 = intr.coeffs
                radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
                x, y = x * radial, y * radial
                xd = x + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
                yd = y + 2 * p2 * x * y + p1 * (r2 + 2 * y * y)
                uv = np.column_stack((xd * intr.fx + intr.cx, yd * intr.fy + intr.cy))
        else:
            with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
                uv = points[:, :2] / points[:, 2:] * [intr.fx, intr.fy] + [intr.cx, intr.cy]
        # Pixels cover half-open cells [i-.5, i+.5). Bounds are checked before
        # integer casting, so NaN/inf/huge out-of-view values are never cast.
        visible = (np.isfinite(uv).all(axis=1) & (uv[:, 0] >= -0.5) & (uv[:, 0] < intr.width - 0.5)
                   & (uv[:, 1] >= -0.5) & (uv[:, 1] < intr.height - 0.5))
        points, flat_indices, uv = points[visible], flat_indices[visible], uv[visible]
        pixels = np.floor(uv + 0.5).astype(np.int64)
        pixel_ids = pixels[:, 1] * intr.width + pixels[:, 0]
        order = np.argsort(points[:, 2], kind="stable")
        _, first = np.unique(pixel_ids[order], return_index=True)
        winners = order[first]
        output.reshape(-1, 3)[pixel_ids[winners]] = points[winners]
        source_index.ravel()[pixel_ids[winners]] = flat_indices[winners]
    mask = source_index >= 0
    return {"xyz_rgb_m": output, "depth_rgb_m": output[..., 2].copy(), "valid_mask": mask,
            "source_flat_index": source_index, "coordinate_frame": "rgb_optical",
            "depth_semantics": "rgb_optical_z_m", "mask_semantics": "geometric_validity_not_confidence",
            "alignment": "nearest_pixel_zbuffer_no_hole_filling"}
