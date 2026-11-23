import dataclasses

import einops
import numpy as np

from openpi import transforms
from openpi.models import model as _model

# Dimension cropping for the anygrasp (gr3qnexo humanoid) dataset.
# See xsy_utils/drop_joints.md for the full rationale.
#   - action 37 -> 29:   drop [0, 3, 4, 31, 33, 34, 35, 36]
#   - state  33 -> 28:   drop [0, 3, 4, 31, 32]
# The first 28 kept action dims are the SAME joints (same order) as the 28 kept state
# dims, so a positional delta transform (make_bool_mask(28)) is aligned. The last kept
# action dim (vel_pitch) is a base velocity and stays absolute (not covered by the mask).
# vel_height/vel_x/vel_yaw are dropped: they are ~constant (range 0 but tiny non-zero std),
# which makes quantile normalization blow up and diverges training.
ACTION_KEEP = [1, 2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 32]  # noqa: E501  (29)
STATE_KEEP = [1, 2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]  # (28)


def make_gr3_example() -> dict:
    """Creates a random input example for the gr3 policy (raw, pre-crop dims)."""
    return {
        "observation/state": np.random.rand(33),
        "observation/image": np.random.randint(256, size=(480, 832, 3), dtype=np.uint8),
        "prompt": "do something",
    }


def _parse_image(image) -> np.ndarray:
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.shape[0] == 3:
        image = einops.rearrange(image, "c h w -> h w c")
    return image


@dataclasses.dataclass(frozen=True)
class Gr3Inputs(transforms.DataTransformFn):
    """Inputs for the gr3qnexo humanoid (anygrasp) policy. Used for both training and inference.

    Crops state to 28 dims and actions to 32 dims (see ACTION_KEEP / STATE_KEEP), maps the single
    top camera to base_0_rgb, and pads the two wrist views with masked-out zero images.
    """

    model_type: _model.ModelType

    def __call__(self, data: dict) -> dict:
        # Crop state 33 -> 28 (delta anchor, aligned with the first 28 action dims).
        state = np.asarray(data["observation/state"])[..., STATE_KEEP]

        base_image = _parse_image(data["observation/image"])

        inputs = {
            "state": state,
            "image": {
                "base_0_rgb": base_image,
                # Dataset only has a single top camera; pad the wrist views with zeros.
                "left_wrist_0_rgb": np.zeros_like(base_image),
                "right_wrist_0_rgb": np.zeros_like(base_image),
            },
            "image_mask": {
                "base_0_rgb": np.True_,
                # Mask padding images for pi0 (not pi0-FAST).
                "left_wrist_0_rgb": np.True_ if self.model_type == _model.ModelType.PI0_FAST else np.False_,
                "right_wrist_0_rgb": np.True_ if self.model_type == _model.ModelType.PI0_FAST else np.False_,
            },
        }

        # Actions are only available during training. Crop 37 -> 29 (first 28 = joints, last 1 = vel_pitch).
        if "actions" in data:
            inputs["actions"] = np.asarray(data["actions"])[..., ACTION_KEEP]

        if "prompt" in data:
            inputs["prompt"] = data["prompt"]

        return inputs


@dataclasses.dataclass(frozen=True)
class Gr3Outputs(transforms.DataTransformFn):
    """Outputs for the gr3 policy. Inference only.

    Returns the 29 cropped action dims (first 28 joints + vel_pitch). Deployment on the real
    robot needs to scatter these back into the robot's 37-dim action layout (see ACTION_KEEP); that
    un-drop mapping is intentionally left out here since it is not needed for training.
    """

    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][..., : len(ACTION_KEEP)])}
