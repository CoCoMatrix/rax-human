import math
import pytest

from real_robot.piper.contracts import ContractError
from real_robot.piper.description_config import mast_configuration


def test_fixed_75_degrees_maps_to_joint_zero_and_stays_nominal():
    value = mast_configuration()
    assert value['total_forward_tilt_deg'] == 75
    assert value['joint_position_rad'] == 0.0
    assert value['approximate'] is True and value['include_in_action14'] is False
    assert value['source'] == 'fixed_v11_photo_urdf_geometry'
    assert value['calibration_status'] == 'nominal_urdf_not_calibrated'


def test_explicit_zero_is_fixed_total_75():
    value = mast_configuration(0.0)
    assert value['total_forward_tilt_deg'] == 75
    assert value['joint_position_rad'] == 0
    assert value['approximate'] is True


@pytest.mark.parametrize('value', [True, float('nan'), float('inf'), math.radians(25), math.radians(36), math.radians(-51)])
def test_invalid_mast_values_rejected(value):
    with pytest.raises(ContractError):
        mast_configuration(value)
