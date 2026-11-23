import dataclasses

import einops
import numpy as np

from openpi import transforms


PIPER_CAMERAS = ("head", "left_wrist", "right_wrist")


def _camera_hwc_uint8(value: object, *, name: str) -> np.ndarray:
    image = np.asarray(value)
    if image.ndim != 3:
        raise ValueError(f"Piper camera {name} must be rank 3, got {image.shape}")
    if image.shape[0] == 3 and image.shape[-1] != 3:
        image = einops.rearrange(image, "c h w -> h w c")
    if image.shape[-1] != 3:
        raise ValueError(f"Piper camera {name} must have three channels, got {image.shape}")
    if image.dtype != np.uint8:
        raise ValueError(f"Piper camera {name} must be uint8, got {image.dtype}")
    return image


@dataclasses.dataclass(frozen=True)
class PiperTriCameraJointInputs(transforms.DataTransformFn):
    prompt: str

    def __call__(self, data: dict) -> dict:
        state = np.asarray(data["state"], dtype=np.float32)
        if state.shape != (14,) or not np.isfinite(state).all():
            raise ValueError(f"Piper state must be finite [14], got {state.shape}")
        images = data.get("images", {})
        if set(images) != set(PIPER_CAMERAS):
            raise ValueError(f"Piper requires exactly {PIPER_CAMERAS}, got {tuple(images)}")
        result = {
            "image": {
                "base_0_rgb": _camera_hwc_uint8(images["head"], name="head"),
                "left_wrist_0_rgb": _camera_hwc_uint8(images["left_wrist"], name="left_wrist"),
                "right_wrist_0_rgb": _camera_hwc_uint8(images["right_wrist"], name="right_wrist"),
            },
            "image_mask": {
                "base_0_rgb": np.True_,
                "left_wrist_0_rgb": np.True_,
                "right_wrist_0_rgb": np.True_,
            },
            "state": state,
            "prompt": str(data.get("prompt") or self.prompt),
        }
        if "actions" in data:
            actions = np.asarray(data["actions"], dtype=np.float32)
            if actions.ndim != 2 or actions.shape[1] != 14 or not np.isfinite(actions).all():
                raise ValueError(f"Piper actions must be finite [T,14], got {actions.shape}")
            result["actions"] = actions
        return result


@dataclasses.dataclass(frozen=True)
class PiperJointOutputs(transforms.DataTransformFn):
    def __call__(self, data: dict) -> dict:
        actions = np.asarray(data["actions"])
        if actions.ndim != 2 or actions.shape[1] < 14:
            raise ValueError(f"Piper model actions must be [T,>=14], got {actions.shape}")
        actions = np.asarray(actions[:, :14], dtype=np.float32)
        if not np.isfinite(actions).all():
            raise ValueError("Piper model actions contain NaN or Inf")
        return {"actions": actions}
