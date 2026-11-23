import numpy as np
import pyarrow as pa


def wrap_joints_and_values(
    joints: list[str] | pa.Array | np.ndarray,
    values: list[float] | pa.Array | np.ndarray,
) -> pa.StructArray:
    """Wrap joints and their corresponding values into a structured array.

    Args:
        joints: A list, numpy array, or pyarrow array of joint names.
        values: A list, numpy array, or pyarrow array of integer values.

    Returns:
        A structured array with two fields:
        - "joints": A string field containing the names of the joints.
        - "values": An Float32Array containing the values corresponding to the joints.

    Raises:
        ValueError: If lengths of joints and values do not match.

    Example:
        joints = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
        values = [100, 200, 300]
        struct_array = wrap_joints_and_values(joints, values)

        This example wraps the given joints and their corresponding values into a structured array.

    """
    if len(joints) != len(values):
        raise ValueError("joints and values must have the same length")

    mask = pa.array([False] * len(values), type=pa.bool_())

    if isinstance(values, list) or isinstance(values, np.ndarray):
        mask = pa.array([value is None for value in values])

    if isinstance(values, pa.Array):
        mask = values.is_null()

    return pa.StructArray.from_arrays(
        arrays=[joints, values],
        names=["name", "value"],
        mask=mask,
    ).drop_null()


def pa_to_q_dict(struct_array: pa.StructArray) -> dict[str, float]:
    return {j["name"].as_py(): j["value"].as_py() for j in struct_array}


class JointStateSerde:
    @staticmethod
    def serialize(q_dict: dict[str, float]) -> pa.StructArray:
        return wrap_joints_and_values(list(q_dict.keys()), list(q_dict.values()))

    @staticmethod
    def serialize_from_array(
        names: list[str] | np.ndarray | pa.Array, values: list[float] | np.ndarray | pa.Array
    ) -> pa.StructArray:
        return wrap_joints_and_values(names, values)

    @staticmethod
    def deserialize(struct_array: pa.StructArray) -> dict[str, float]:
        return pa_to_q_dict(struct_array)
