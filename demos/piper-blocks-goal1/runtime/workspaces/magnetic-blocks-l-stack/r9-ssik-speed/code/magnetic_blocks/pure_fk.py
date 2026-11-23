"""Same pinned native FK, evaluated locally; never motion or collision approval."""
import numpy as np
import hashlib
from pathlib import Path
from scipy.spatial.transform import Rotation


def local_kinematics(ctx, joints):
    if not getattr(ctx, '_blocks_local_fk', False):
        return None
    if hasattr(ctx, '_blocks_validated_fk'):
        return ctx._blocks_validated_fk
    # Ship the exact pinned backend implementation/assets inside the task git
    # archive: the production sandbox intentionally cannot import host services.
    try:
        from native_fk.geometry import Kinematics, URDF_SHA
    except ImportError as exc:
        raise RuntimeError('Required local FK bundle missing; repair candidate package before planning') from exc
    import native_fk.geometry as implementation
    from native_fk import IMPLEMENTATION_SHA256
    if hashlib.sha256(Path(implementation.__file__).read_bytes()).hexdigest() != IMPLEMENTATION_SHA256:
        raise ValueError('Packaged native FK implementation identity changed')
    kin = Kinematics()  # Verifies actual asset SHA and the same joint limits.
    probes = [list(map(float, joints[:6])), [0.] * 6]
    for fraction in (.25, .5, .75):
        probes.append([lo + fraction * (hi-lo) for _, _, lo, hi in kin.chains['left']])
    reference = ctx.value('piper_fk_batch', joints_batch=probes)
    if (reference.get('executed') is not False or reference.get('frame') != 'left_base_link'
            or reference.get('controlled_link') != 'left_link6'
            or reference.get('urdf_sha256') != URDF_SHA):
        raise ValueError('Local FK model/frame identity differs from native service')
    matrices = np.asarray(reference.get('T_base_flange'), dtype=float)
    calculated = kin.fk_batch(probes)
    if matrices.shape != calculated.shape or not np.isfinite(matrices).all() or not np.allclose(matrices, calculated, rtol=0, atol=1e-10):
        raise ValueError('Local FK differs from native service; do not use local geometry')
    ctx._blocks_validated_fk = kin
    ctx._blocks_fk_identity = {'mode': 'packaged_native_local', 'urdf_sha256': URDF_SHA,
                              'implementation_sha256': IMPLEMENTATION_SHA256,
                              'native_validation_samples': len(probes)}
    return kin


def row(matrix):
    t = np.asarray(matrix, dtype=float)
    return {'T_base_flange': t.tolist(), 'position': t[:3, 3].tolist(),
            'quaternion_xyzw': Rotation.from_matrix(t[:3, :3]).as_quat().tolist()}


def batch_rows(kin, joints):
    out = []
    for i in range(0, len(joints), 512):
        # Path geometry only needs transforms. Compute quaternions lazily for
        # individual route endpoints rather than once per sampled matrix.
        out.extend({'T_base_flange': t.tolist(), 'position': t[:3, 3].tolist()}
                   for t in kin.fk_batch(joints[i:i+512]))
    return out
