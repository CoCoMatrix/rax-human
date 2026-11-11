"""Explicitly released, pixel-preserving image preparation on the WS worker.

The resize operations match the immutable OpenPI image_tools.py (SHA below).
Camera/recorder buffers and observation timestamps are never rewritten.
"""
import json
import hashlib
from pathlib import Path
import tarfile

import numpy as np

CONTRACT = {
    "schema": "xpolicy.image-transport/v1",
    "algorithm": "openpi_pil_bilinear_pad",
    "height": 224,
    "width": 224,
    "pillow_version": "11.2.1",
    "source_sha256": "d48b4bd7f44e79fe6db8a8e07c9161144fa250be686e1245014a8b47e6171977",
    "model_manifest_sha256": "dfc8b48c37019f89278b454ec8b5f2bca1ef98173239d6b3fc9e042aeea370f0",
}
CAMERAS = {"camera_head", "camera_left", "camera_right"}


class ImageTransport:
    def __init__(self, contract):
        identity = contract.get("model_manifest_sha256", "")
        if (set(contract) != set(CONTRACT)
                or any(contract[k] != v for k, v in CONTRACT.items() if k != "model_manifest_sha256")
                or not isinstance(identity, str) or len(identity) != 64
                or any(c not in "0123456789abcdef" for c in identity)):
            raise ValueError("Image transport differs from the verified resize contract")
        import PIL
        from PIL import Image
        if PIL.__version__ != CONTRACT["pillow_version"]:
            raise ValueError("Image transport requires the model's Pillow 11.2.1")
        self.Image = Image
        self.status = {"enabled": True, **contract,
                       "placement": "inference_worker_before_websocket",
                       "timestamps": "unchanged", "frame_reuse": False}

    def prepare(self, observation):
        vision = observation["vision"]
        if set(vision) != CAMERAS:
            raise ValueError("Image transport requires all three declared cameras")
        resized = {}
        for name, item in vision.items():
            raw = item["color"]
            if (not isinstance(raw, np.ndarray) or raw.dtype != np.uint8
                    or raw.ndim != 3 or raw.shape[2] != 3
                    or min(raw.shape[:2]) < 1 or item.get("encoding", "rgb8") != "rgb8"):
                raise ValueError("Image transport requires uint8 RGB HWC")
            if raw.shape[:2] == (224, 224):
                image = raw
            else:
                # Same PIL operations and integer rounding as the frozen model.
                source = self.Image.fromarray(raw)
                cur_width, cur_height = source.size
                ratio = max(cur_width / 224, cur_height / 224)
                resized_height = int(cur_height / ratio)
                resized_width = int(cur_width / ratio)
                scaled = source.resize((resized_width, resized_height), resample=self.Image.BILINEAR)
                padded = self.Image.new(scaled.mode, (224, 224), 0)
                pad_height = max(0, int((224 - resized_height) / 2))
                pad_width = max(0, int((224 - resized_width) / 2))
                padded.paste(scaled, (pad_width, pad_height))
                image = np.asarray(padded)
            resized[name] = {**item, "color": image}
        # Preserve every non-image field, including prefix, frame identity and
        # timestamps; keep the original full-resolution observation available.
        return {**observation, "vision": resized}

    def wrap(self, client):
        return ImageTransportClient(client, self)


class ImageTransportClient:
    def __init__(self, client, transport):
        self.client, self.transport = client, transport

    def infer(self, observation, **kwargs):
        # AsyncPolicyInference calls this on its worker, inside its timer and
        # exception handling. No image work runs on the 30 Hz policy tick.
        return self.client.infer(self.transport.prepare(observation), **kwargs)

    def close(self):
        return self.client.close()


def for_deployment(name):
    if not name:
        return None
    from XPolicyLab.policy.Pi_05.piper_deployments import REPO_ROOT, load_deployment
    deployment = load_deployment(name)
    release = deployment.get("release", {})
    if release.get("status") != "contract_complete":
        return None
    evidence = json.loads((REPO_ROOT / release["evidence"]).read_text())
    contract = evidence.get("image_transport")
    if contract is None:
        return None
    bundle = Path(deployment["bundle"]["default_path"])
    manifest_sha = hashlib.sha256((bundle / "package_manifest.json").read_bytes()).hexdigest()
    if (manifest_sha != contract.get("model_manifest_sha256")
            or manifest_sha != evidence.get("source", {}).get("manifest_sha256")):
        raise ValueError("Image transport model identity differs from the selected release")
    # A new model may use this exact resize only when its own immutable source
    # contains the same implementation; matching dimensions alone is insufficient.
    with tarfile.open(bundle / "runtime/source.tar.gz") as archive:
        source = archive.extractfile("packages/openpi-client/src/openpi_client/image_tools.py").read()
    if hashlib.sha256(source).hexdigest() != contract.get("source_sha256"):
        raise ValueError("Image transport resize source differs from the selected model")
    return ImageTransport(contract)
