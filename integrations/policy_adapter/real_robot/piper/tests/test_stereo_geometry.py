"""Independent synthetic geometry checks; no depth-model/physical accuracy claim."""
import numpy as np
import pytest

from real_robot.piper.contracts import ContractError
from real_robot.piper.stereo_geometry import (
    CameraIntrinsics, RectifiedStereo, StereoRectifier, align_left_xyz_to_rgb,
    disparity_to_left_xyz, transform_xyz, validate_se3,
)


def intr(width=8, height=6, fx=100.0, fy=110.0, cx=3.0, cy=2.0, **kwargs):
    return CameraIntrinsics(width, height, fx, fy, cx, cy, **kwargs)


def translation(x=0, y=0, z=0):
    result = np.eye(4)
    result[:3, 3] = [x, y, z]
    return result


def y_rotation(angle):
    result = np.eye(4)
    c, s = np.cos(angle), np.sin(angle)
    result[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return result


def stereo(cx_left=3, cx_right=1):
    return RectifiedStereo(intr(cx=cx_left), intr(cx=cx_right), translation(-0.05))


def test_metric_triangulation_accounts_for_principal_point_difference():
    pair = stereo()
    disparity = np.full((6, 8), 7.0)  # corrected disparity=5px -> z=1m
    result = disparity_to_left_xyz(disparity, pair)
    assert result["valid_mask"].all()
    np.testing.assert_allclose(result["depth_left_m"], 1)
    np.testing.assert_allclose(result["xyz_left_m"][2, 3], [0, 0, 1])
    np.testing.assert_allclose(result["xyz_left_m"][3, 4], [0.01, 1 / 110, 1])
    assert result["mask_semantics"] == "geometric_validity_not_confidence"


def test_negative_raw_disparity_can_be_valid_with_cx_offset():
    result = disparity_to_left_xyz(np.full((6, 8), -1.0), stereo(cx_left=0, cx_right=6))
    assert result["valid_mask"].all()
    np.testing.assert_allclose(result["depth_left_m"], 1)


def test_invalid_disparity_or_depth_becomes_nan_holes():
    disparity = np.full((6, 8), 7.0)
    disparity[0, :6] = [2, 1, -5, np.nan, np.inf, 2 + 1e-12]
    disparity[1, 0] = 1002
    result = disparity_to_left_xyz(disparity, stereo(), min_depth_m=0.1, max_depth_m=5)
    assert not result["valid_mask"][0, :6].any()
    assert not result["valid_mask"][1, 0]
    assert np.isnan(result["depth_left_m"][~result["valid_mask"]]).all()
    assert np.isnan(result["xyz_left_m"][~result["valid_mask"]]).all()


def test_resize_uses_half_pixel_principal_points_and_pixel_disparity_scale():
    pair = stereo()
    resized = pair.resized(16, 18)  # anisotropic image resize
    assert resized.left.fx == 200
    assert resized.left.fy == 330
    assert resized.left.cx == 6.5
    assert resized.left.cy == 7
    assert resized.right.cx == 2.5
    assert resized.baseline_m == pair.baseline_m
    full = disparity_to_left_xyz(np.full((6, 8), 7.0), pair)
    changed = disparity_to_left_xyz(np.full((18, 16), 14.0), resized)
    np.testing.assert_allclose(changed["depth_left_m"], 1)
    # Old x=.25 continuous pixel -> resized x=1, old y=1 -> resized y=4.
    np.testing.assert_allclose(changed["xyz_left_m"][4, 1], [(0.25 - 3) / 100, (1 - 2) / 110, 1])
    np.testing.assert_allclose(full["depth_left_m"].mean(), changed["depth_left_m"].mean())


def test_nonidentity_rgb_extrinsics_and_zbuffer_store_rgb_optical_z():
    xyz = np.array([[[0.1, 0, 1], [0.15, 0, 2], [0, 0, -1], [np.nan, 0, 1]]])
    transform = translation(z=1)
    transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    result = align_left_xyz_to_rgb(xyz, np.ones((1, 4), dtype=bool), transform,
                                   intr(width=7, height=7, fx=10, fy=10, cx=3, cy=3))
    assert result["valid_mask"].sum() == 1
    np.testing.assert_allclose(result["xyz_rgb_m"][4, 3], [0, 0.1, 2])
    assert result["depth_rgb_m"][4, 3] == 2  # not source depth 1
    assert result["source_flat_index"][4, 3] == 0
    assert np.isnan(result["depth_rgb_m"][~result["valid_mask"]]).all()
    assert (result["source_flat_index"][~result["valid_mask"]] == -1).all()


def test_zbuffer_ties_keep_source_raster_order():
    xyz = np.array([[[0, 0, 1], [0.0001, 0, 1], [0, 0, 2]]], dtype=float)
    result = align_left_xyz_to_rgb(xyz, np.ones((1, 3), bool), np.eye(4), intr())
    assert result["source_flat_index"][2, 3] == 0


def test_rgb_projection_uses_brown_conrady_distortion():
    coeffs = (0.2, 0.1, 0.03, -0.02, 0.005)
    camera = intr(width=220, height=220, fx=100, fy=100, cx=100, cy=100,
                  distortion_model="brown_conrady", coeffs=coeffs)
    xyz = np.array([[[0.5, 0.25, 1]]])
    result = align_left_xyz_to_rgb(xyz, np.ones((1, 1), bool), np.eye(4), camera)
    x, y = 0.5, 0.25
    k1, k2, p1, p2, k3 = coeffs
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
    u = 100 * (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)) + 100
    v = 100 * (y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y) + 100
    assert result["valid_mask"][int(np.floor(v + 0.5)), int(np.floor(u + 0.5))]
    assert not result["valid_mask"][125, 150]  # undistorted result would land here


def test_negative_target_z_out_of_fov_and_nan_rejected_before_integer_cast():
    xyz = np.array([[[0, 0, 1], [1e200, 0, 1], [0, 0, 0.1], [0, np.inf, 1]]])
    result = align_left_xyz_to_rgb(xyz, np.ones((1, 4), bool), translation(z=-0.2), intr())
    assert result["valid_mask"].sum() == 1
    assert result["depth_rgb_m"][2, 3] == pytest.approx(0.8)


def test_transform_rgb_into_nominal_base_preserves_negative_base_z():
    xyz = np.array([[[0.1, 0.2, 1], [0.2, 0.3, 2]]])
    transform = translation(x=0.3, y=-0.2, z=0.5)
    transform[:3, :3] = np.diag([1, -1, -1])
    result = transform_xyz(xyz, np.array([[True, False]]), transform)
    np.testing.assert_allclose(result["xyz_m"][0, 0], [0.4, -0.4, -0.5])
    assert result["valid_mask"][0, 0]
    assert np.isnan(result["xyz_m"][0, 1]).all()


@pytest.mark.parametrize("model", ["kannala_brandt4", "ftheta", "unknown"])
def test_unsupported_distortion_is_never_silently_ignored(model):
    with pytest.raises(ContractError, match="unsupported distortion"):
        intr(distortion_model=model)


@pytest.mark.parametrize("model", ["modified_brown_conrady", "inverse_brown_conrady"])
def test_rgb_modified_and_inverse_use_librealsense_projection_not_opencv_brown(model):
    # Reference scalar rs2_project_point_to_pixel equations for this fixture
    # produce (196.500722453, 167.544201533), vs Brown (198.2423752,168.6327345).
    camera = intr(width=250, height=200, fx=100, fy=100, cx=100, cy=100,
                  distortion_model=model, coeffs=(0.3, 0.05, 0.05, -0.05, 0.01))
    result = align_left_xyz_to_rgb(np.array([[[0.8, 0.5, 1]]]), np.ones((1, 1), bool), np.eye(4), camera)
    assert result["valid_mask"].sum() == 1
    assert result["valid_mask"][168, 197]
    assert not result["valid_mask"][169, 198]
    with pytest.raises(ContractError, match="OpenCV IR rectification"):
        StereoRectifier(camera, camera, translation(-0.05), np.eye(4))


def test_intrinsics_from_sdk_dict_requires_distortion_metadata():
    item = dict(width=8, height=6, fx=100, fy=110, ppx=3, ppy=2,
                model="distortion.brown_conrady", coeffs=[0, 0, 0, 0, 0])
    assert CameraIntrinsics.from_dict(item).cx == 3
    item.pop("model")
    with pytest.raises(ContractError, match="incomplete"):
        CameraIntrinsics.from_dict(item)


def test_intrinsics_from_exporter_K_and_distortion_coeffs():
    item = dict(width=8, height=6, K=[[100, 0, 3], [0, 110, 2], [0, 0, 1]],
                distortion_model="distortion.none", distortion_coeffs=[0, 0, 0, 0, 0])
    actual = CameraIntrinsics.from_dict(item)
    assert (actual.fx, actual.fy, actual.cx, actual.cy) == (100, 110, 3, 2)
    item["fx"] = 50
    with pytest.raises(ContractError, match="conflicts"):
        CameraIntrinsics.from_dict(item)


@pytest.mark.parametrize("damage", ["skew", "lower_skew", "last_row", "shape", "nonfinite"])
def test_exporter_K_must_not_silently_discard_entries(damage):
    matrix = np.array([[100, 0, 3], [0, 110, 2], [0, 0, 1]], dtype=float)
    if damage == "skew":
        matrix[0, 1] = 1
    elif damage == "lower_skew":
        matrix[1, 0] = 1
    elif damage == "last_row":
        matrix[2, 2] = 2
    elif damage == "shape":
        matrix = matrix.reshape(9)
    else:
        matrix[0, 0] = np.nan
    with pytest.raises(ContractError):
        CameraIntrinsics.from_dict(dict(width=8, height=6, K=matrix,
            distortion_model="none", distortion_coeffs=[0, 0, 0, 0, 0]))


@pytest.mark.parametrize("damage", ["reflection", "scale", "lastrow", "nan", "shape", "string"])
def test_bad_se3_is_rejected(damage):
    t = np.eye(4)
    if damage == "reflection":
        t[0, 0] = -1
    elif damage == "scale":
        t[0, 0] = 2
    elif damage == "lastrow":
        t[3, 0] = 0.1
    elif damage == "nan":
        t[1, 3] = np.nan
    elif damage == "shape":
        t = t.reshape(16)
    else:
        t = t.astype(str)
    with pytest.raises(ContractError):
        validate_se3(t)


@pytest.mark.parametrize("damage", ["positive_baseline", "zero_baseline", "rotation", "vertical", "distorted", "cy", "fx", "resolution"])
def test_unrectified_or_reversed_pair_is_rejected(damage):
    left, right, t = intr(), intr(), translation(-0.05)
    if damage == "positive_baseline":
        t[0, 3] *= -1
    elif damage == "zero_baseline":
        t[0, 3] = 0
    elif damage == "rotation":
        t[:3, :3] = y_rotation(0.01)[:3, :3]
    elif damage == "vertical":
        t[1, 3] = 0.001
    elif damage == "distorted":
        left = intr(distortion_model="brown_conrady", coeffs=(0.1, 0, 0, 0, 0))
    elif damage == "cy":
        right = intr(cy=2.1)
    elif damage == "fx":
        right = intr(fx=101)
    else:
        right = intr(width=9)
    with pytest.raises(ContractError):
        RectifiedStereo(left, right, t)


@pytest.mark.parametrize("value", [np.ones((5, 8)), np.ones((6, 8), dtype=np.int16), np.ones((6, 8, 1))])
def test_disparity_resolution_or_pixel_units_must_be_explicit(value):
    with pytest.raises(ContractError):
        disparity_to_left_xyz(value, stereo())


def test_rectification_geometry_keeps_full_raw_to_rgb_transform():
    left = intr(width=80, height=60, fx=75, fy=76, cx=38, cy=29,
                distortion_model="brown_conrady", coeffs=(0.03, -0.01, 0.001, 0.002, 0))
    right = intr(width=80, height=60, fx=74, fy=75, cx=39, cy=28,
                 distortion_model="brown_conrady", coeffs=(0.025, -0.012, 0.002, 0.001, 0))
    raw_stereo = y_rotation(0.015)
    raw_stereo[:3, 3] = [-0.05, 0.0001, 0.0002]
    raw_rgb = y_rotation(-0.03)
    raw_rgb[:3, 3] = [0.012, 0.0002, -0.0005]
    rectifier = StereoRectifier(left, right, raw_stereo, raw_rgb)
    a, b = rectifier.stereo.left, rectifier.stereo.right
    assert (a.width, a.height) == (80, 60)
    assert (b.width, b.height) == (80, 60)
    np.testing.assert_allclose(rectifier.stereo.baseline_m, np.linalg.norm(raw_stereo[:3, 3]), atol=1e-10)
    point_rect_left = np.array([0.12, 0.04, 1.2])
    point_raw_left = rectifier.R_leftrect_from_left.T @ point_rect_left
    point_raw_right = raw_stereo[:3, :3] @ point_raw_left + raw_stereo[:3, 3]
    point_rect_right = rectifier.R_rightrect_from_right @ point_raw_right
    uv_left = (point_rect_left[:2] / point_rect_left[2]) * [a.fx, a.fy] + [a.cx, a.cy]
    uv_right = (point_rect_right[:2] / point_rect_right[2]) * [b.fx, b.fy] + [b.cx, b.cy]
    assert uv_left[1] == pytest.approx(uv_right[1], abs=1e-10)
    disparity = uv_left[0] - uv_right[0]
    z = a.fx * rectifier.stereo.baseline_m / (disparity - (a.cx - b.cx))
    assert z == pytest.approx(1.2, abs=1e-10)
    rgb_via_rect = rectifier.T_rgb_from_leftrect[:3, :3] @ point_rect_left + rectifier.T_rgb_from_leftrect[:3, 3]
    rgb_via_raw = raw_rgb[:3, :3] @ point_raw_left + raw_rgb[:3, 3]
    np.testing.assert_allclose(rgb_via_rect, rgb_via_raw, atol=1e-12)
    assert not np.allclose(rectifier.T_rgb_from_leftrect[:3, :3], raw_rgb[:3, :3])
    image = np.arange(80 * 60, dtype=np.uint8).reshape(60, 80)
    remapped = rectifier.rectify_pair(image, image)
    assert remapped["left_rectified"].shape == image.shape
    assert remapped["left_rectified"].dtype == np.uint8
    assert remapped["left_valid_mask"].any()
    assert not remapped["left_valid_mask"].all()


def test_rectification_disparity_mask_checks_both_ir_support_regions():
    pair = StereoRectifier(intr(), intr(), translation(-0.05), np.eye(4))
    disparity = np.full((6, 8), 2.0)
    disparity[2, 4] = np.nan
    mask = pair.disparity_valid_mask(disparity)
    assert not mask[:, :2].any()
    assert not mask[2, 4]
    assert mask[2, 5]
    assert np.all(mask <= pair.left_valid_mask)


def test_rectifier_rejects_color_and_wrong_profile_ir_images():
    pair = StereoRectifier(intr(), intr(), translation(-0.05), np.eye(4))
    with pytest.raises(ContractError, match="uint8 grayscale"):
        pair.rectify_pair(np.zeros((6, 8, 3), np.uint8), np.zeros((6, 8), np.uint8))


@pytest.mark.parametrize("damage", ["width_bool", "focal_zero", "nan_coeff", "none_nonzero", "coeff_count"])
def test_invalid_intrinsics_rejected(damage):
    kwargs = {}
    if damage == "width_bool":
        kwargs["width"] = True
    elif damage == "focal_zero":
        kwargs["fx"] = 0
    elif damage == "nan_coeff":
        kwargs.update(distortion_model="brown_conrady", coeffs=(np.nan, 0, 0, 0, 0))
    elif damage == "none_nonzero":
        kwargs["coeffs"] = (1, 0, 0, 0, 0)
    else:
        kwargs["coeffs"] = (0, 0, 0)
    with pytest.raises(ContractError):
        intr(**kwargs)
