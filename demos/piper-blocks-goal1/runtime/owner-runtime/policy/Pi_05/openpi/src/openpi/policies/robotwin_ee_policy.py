import dataclasses
from typing import ClassVar

import numpy as np

from openpi import transforms
from openpi.policies import aloha_policy


def make_robotwin_ee_example() -> dict:
    """Creates a random input example for the RoboTwin end-effector policy."""
    return {
        "state": np.ones((14,)),
        "images": {
            "cam_high": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
            "cam_left_wrist": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
            "cam_right_wrist": np.random.randint(256, size=(3, 224, 224), dtype=np.uint8),
        },
        "prompt": "do something",
    }


@dataclasses.dataclass(frozen=True)
class RobotwinEEInputs(transforms.DataTransformFn):
    """Inputs for pi0.5 on RoboTwin with end-effector (ee) actions.

    Difference from the joint baseline (``aloha_policy.AlohaInputs``): the action is a
    16-dim *absolute* end-effector pose, not a 14-dim joint command. State stays 14-dim
    joint and is decoded identically to the joint baseline (via ``_decode_aloha``), so the
    only change vs. the joint baseline is the action space.

    Expected inputs:
    - images: dict[name, img] with img [channel, height, width]; name in EXPECTED_CAMERAS.
    - state: [14] joint (left arm 6 + left gripper 1 + right arm 6 + right gripper 1)
    - actions: [action_horizon, 16] absolute ee:
        [L_pos(3), L_quat(4, wxyz), L_grip(1), R_pos(3), R_quat(4, wxyz), R_grip(1)]
    """

    # Applies the aloha->pi transform to the 14-dim joint STATE only (matches the joint
    # baseline's state handling). Never touches the ee actions.
    adapt_to_pi: bool = True

    EXPECTED_CAMERAS: ClassVar[tuple[str, ...]] = ("cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist")

    def __call__(self, data: dict) -> dict:
        # Reuse the aloha decode for images (c h w -> h w c) and the 14-dim joint state.
        # This does NOT touch data["actions"], so the ee actions pass through untouched.
        data = aloha_policy._decode_aloha(data, adapt_to_pi=self.adapt_to_pi)  # noqa: SLF001

        in_images = data["images"]
        if set(in_images) - set(self.EXPECTED_CAMERAS):
            raise ValueError(f"Expected images to contain {self.EXPECTED_CAMERAS}, got {tuple(in_images)}")

        base_image = in_images["cam_high"]
        images = {"base_0_rgb": base_image}
        image_masks = {"base_0_rgb": np.True_}

        extra_image_names = {
            "left_wrist_0_rgb": "cam_left_wrist",
            "right_wrist_0_rgb": "cam_right_wrist",
        }
        for dest, source in extra_image_names.items():
            if source in in_images:
                images[dest] = in_images[source]
                image_masks[dest] = np.True_
            else:
                images[dest] = np.zeros_like(base_image)
                image_masks[dest] = np.False_

        inputs = {
            "image": images,
            "image_mask": image_masks,
            "state": data["state"],
        }

        # Actions are only available during training. Absolute ee, no adapt, no delta.
        if "actions" in data:
            inputs["actions"] = np.asarray(data["actions"])

        if "prompt" in data:
            inputs["prompt"] = data["prompt"]

        return inputs


@dataclasses.dataclass(frozen=True)
class RobotwinEEOutputs(transforms.DataTransformFn):
    """Outputs for the RoboTwin ee policy. Returns the first 16 (ee) action dims."""

    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][:, :16])}
