import numpy as np
import pytest
from real_robot.piper.scene_geometry import scene_agreement, fit_table


def test_scene_comparison_rejects_displaced_local_object():
    original = np.zeros((100, 100, 3), np.uint8)
    original[20:40, 30:50] = 200
    assert scene_agreement(original, original)['fraction_over_30_levels'] == 0
    moved = np.roll(original, 12, axis=1)
    with pytest.raises(ValueError, match='does not agree'):
        scene_agreement(original, moved)


def test_plane_robust_to_object_points_retains_nominal_status():
    x, y = np.meshgrid(np.linspace(-.3, .3, 70), np.linspace(-.4, .4, 70))
    z = .012 + x * .02 - y * .01
    z[:15, :20] += .12
    points = np.stack([x, y, z], axis=-1)
    result = fit_table(points, np.ones(z.shape, bool))
    n = np.array([-.02, .01, 1]); n /= np.linalg.norm(n)
    np.testing.assert_allclose(result['normal'], n, atol=1e-6)
    assert result['inlier_fraction'] > .9
    assert result['collision_clearance_certified'] is False


def test_too_few_points_fail_closed():
    with pytest.raises(ValueError, match='too few'):
        fit_table(np.zeros((10, 10, 3)), np.ones((10, 10), bool))
