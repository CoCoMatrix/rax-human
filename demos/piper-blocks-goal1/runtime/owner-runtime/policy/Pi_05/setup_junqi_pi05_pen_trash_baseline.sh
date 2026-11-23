#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XPL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source /home/ubuntu/code_junqi/env.sh
JUNQI_BUNDLE="${PI05_BUNDLE_ROOT:-/home/ubuntu/code_junqi/model-store/models/pi05/pen-trash-dual-piper-real-h32-v1-ema-v1/step-9999}"
JUNQI_MODEL_ENV="${XPL_ROOT}/.xpolicy/envs/junqi_pi05_pen_trash_py312"
JUNQI_OVERLAY="${XPL_ROOT}/.xpolicy/overlays/junqi_pi05_torch213_cu130"
[[ -x "${JUNQI_MODEL_ENV}/bin/python" && -d "${JUNQI_OVERLAY}/torch" ]] || {
  echo "Prepare the dedicated Junqi runtime from the recorded migration recipe first" >&2; exit 1;
}
PYTHONPATH="${XPL_ROOT}" /usr/bin/python3 \
  "${SCRIPT_DIR}/verify_piper_pen_trash_h32_v1_ema9999_bundle.py" \
  "${JUNQI_BUNDLE}" --full-manifest
PYTHONPATH="${JUNQI_OVERLAY}:${XPL_ROOT}" "${JUNQI_MODEL_ENV}/bin/python" - <<'PY'
import importlib.metadata
import json
from pathlib import Path
import sys
import torch
import torchvision

assert sys.version_info[:2] == (3, 12)
assert torch.__version__ == "2.13.0+cu130"
assert torchvision.__version__ == "0.28.0+cu130"
assert "sm_120" in torch.cuda.get_arch_list()
for package, version in {"jax": "0.5.3", "transformers": "4.53.2", "safetensors": "0.5.3", "websockets": "15.0.1"}.items():
    assert importlib.metadata.version(package) == version, package
for name, path in {"python": sys.executable, "torch": torch.__file__, "torchvision": torchvision.__file__}.items():
    assert Path(path).resolve().is_relative_to('/home/ubuntu/code_junqi'), (name, path)
print(json.dumps({"ok": True, "runtime": "junqi_pi05_pen_trash_py312", "python": sys.executable,
                  "torch": torch.__version__, "torch_path": torch.__file__, "torchvision_path": torchvision.__file__}))
PY
