"""Exact RGB/FFS observation identity and expiration; no synthetic live fallback."""
import hashlib
import io
import json
from pathlib import Path
import time
import numpy as np

MODEL_SHA = '98b5a9acf39fbfa795025de8cea95ce123daa40f6b6234d719167751024cf692'


def contained(root, relative):
    p = Path(relative)
    if p.is_absolute() or '..' in p.parts or not p.parts:
        raise ValueError('Expected contained relative result path')
    root = Path(root).resolve()
    path = (root / p).resolve(strict=True)
    if not path.is_relative_to(root) or path == root:
        raise ValueError('Result escaped configured root')
    return path


class DepthExpiredError(ValueError):
    """An original frame expired; callers may wait for a new original frame."""


class DepthNotRecentError(ValueError):
    """Original frame is outside a stricter consumer budget; wait for a new one."""


class DepthReader:
    def __init__(self, root, calibration_sha):
        self.root, self.calibration_sha = Path(root), calibration_sha

    def read(self, *, now_ns=None, max_age_s=None):
        if max_age_s is not None and (type(max_age_s) not in (int,float) or not 0 < max_age_s <= 1):
            raise ValueError("Invalid depth freshness budget")
        clock = time.time_ns if now_ns is None else lambda: now_ns
        pointer = json.loads((self.root/'latest.json').read_text())
        directory = contained(self.root, pointer['result_dir'])
        report = json.loads(contained(directory, 'report.json').read_text())
        if report.get('schema') != 'rax.piper.ffs.v1' or report.get('mode') != 'live':
            raise ValueError('Live depth requires an actual live result; replay cannot substitute')
        if (report.get('source') != 'fast_foundation_stereo' or report.get('inference_kind') != 'real_model'
                or report.get('serial') != '262522073779' or report.get('calibration_payload_sha256') != self.calibration_sha
                or report.get('checkpoint_sha256') != MODEL_SHA):
            raise ValueError('Depth provenance mismatch')
        for key in ('sample_id', 'owner_generation', 'host_received_unix_ns', 'expires_unix_ns'):
            if pointer.get(key) != report.get(key):
                raise ValueError('Depth pointer/report identity mismatch')
        received, expires = report['host_received_unix_ns'], report['expires_unix_ns']
        if type(received) is not int or type(expires) is not int or expires != received + 1_000_000_000:
            raise ValueError('Depth expiry must be original camera receipt + 1 second')
        def fresh():
            now = clock()
            if now < received:
                raise ValueError('Depth is future; acquisition timestamp cannot be refreshed')
            if now >= expires:
                raise DepthExpiredError('Depth is stale; acquisition timestamp cannot be refreshed')
            if max_age_s is not None and now - received > max_age_s * 1e9:
                raise DepthNotRecentError('Depth exceeds consumer freshness budget; await next original frame')
        fresh()
        path = contained(directory, 'observation.npz')
        payload = path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != report['arrays_sha256']:
            raise ValueError('Depth arrays SHA mismatch')
        with np.load(io.BytesIO(payload), allow_pickle=False) as z:
            arrays = {k: z[k] for k in z.files}
        rgb, xyz, valid = (arrays[k] for k in ('rgb', 'xyz_rgb_m', 'valid'))
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[-1] != 3 or xyz.shape != rgb.shape or valid.shape != rgb.shape[:2] or valid.dtype != bool:
            raise ValueError('Depth/RGB shapes or dtypes mismatch')
        if not np.isfinite(xyz[valid]).all() or np.any(xyz[..., 2][valid] <= 0):
            raise ValueError('Invalid finite depth')
        if report.get('native_geometry'):
            validate_native(arrays,report)
        fresh()
        return arrays, report

    def pixel(self, u, v):
        arrays, report = self.read()
        h, w = arrays['valid'].shape
        if type(u) is not int or type(v) is not int or not 0 <= u < w or not 0 <= v < h:
            raise ValueError('Pixel outside image')
        if not arrays['valid'][v, u]:
            raise ValueError('Depth hole at pixel; no filling or nearest-point substitution')
        return dict(position=arrays['xyz_rgb_m'][v, u].tolist(), frame='global_d435_color_optical_frame',
                    units='m', sample_id=report['sample_id'], confidence_available=False,
                    robot_position=None, reason='Camera-to-robot calibration not accepted',
                    expires_unix_ns=report['expires_unix_ns'])


def validate_native(arrays,report):
    """Reject malformed source/coverage association while preserving live expiry."""
    meta=report['native_geometry'];h,w=meta['shape_hw']
    if meta.get('schema')!='rax.native_geometry.v1' or meta.get('coordinate_frame')!='left_rectified_optical':
        raise ValueError('Native geometry schema/frame mismatch')
    z=arrays['native_depth_m'];uv=arrays['native_uv_rgb'];visible=arrays['native_visible']
    ids=arrays['surface_source_index'];surface=arrays['surface_depth_rgb_m'];resampled=arrays['surface_resampled']
    if (z.shape!=(h,w) or z.dtype!=np.float32 or uv.shape!=(h,w,2) or uv.dtype!=np.float32
        or visible.shape!=(h,w) or visible.dtype!=bool or ids.shape!=arrays['valid'].shape or ids.dtype!=np.int32
        or surface.shape!=ids.shape or resampled.shape!=ids.shape or resampled.dtype!=bool):
        raise ValueError('Native geometry shapes/dtypes mismatch')
    if np.any((ids< -1)|(ids>=h*w)):raise ValueError('Native source index invalid')
    sm=ids>=0
    if (not np.isfinite(surface[sm]).all() or np.any(surface[sm]<=0)
        or not np.isnan(surface[~sm]).all() or np.any(resampled&~sm)
        or not np.isfinite(z.ravel()[ids[sm]]).all() or np.any(z.ravel()[ids[sm]]<=0)):
        raise ValueError('Native surface support invalid')
    if not np.isfinite(z[visible]).all() or np.any(z[visible]<=0) or not np.isfinite(uv[visible]).all():
        raise ValueError('Native visibility invalid')
    pixels=uv[visible];H,W=ids.shape
    if np.any((pixels[:,0]<-.5)|(pixels[:,0]>=W-.5)|(pixels[:,1]<-.5)|(pixels[:,1]>=H-.5)):
        raise ValueError('Native visibility outside RGB')
