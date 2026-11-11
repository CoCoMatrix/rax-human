"""Agent-side evidence fusion: no SDK, owner HTTP, arming or motion calls."""
import hashlib
import json
from pathlib import Path
import time

CHECKPOINTS = {
 'approach': {'head': ['target_visible', 'path_clear', 'hands_clear'], 'left': ['fingers_visible', 'nearby_clearance']},
 'precontact': {'head': ['path_clear', 'hands_clear'], 'left': ['fingers_aligned', 'contact_region_visible', 'cables_clear']},
 'grasp': {'head': ['object_upright'], 'left': ['fingers_engaged', 'no_slip']},
 'lift': {'head': ['object_lifted', 'path_clear'], 'left': ['object_retained', 'no_slip']},
 'place': {'head': ['on_target', 'object_upright'], 'left': ['supported', 'release_clear']},
 'finish': {'head': ['on_target', 'object_upright', 'gripper_separated'], 'left': ['released', 'retreat_clear']},
}


class ObservationNotReady(ValueError):
    """Temporal readiness only; integrity/identity failures must not be retried."""


def observe(ctx, root='runtime/stream/multiview', max_age_s=2.0, not_before_ns=None):
    root = Path(root).resolve()
    pointer = json.loads((root / 'latest.json').read_text())
    folder = (root / pointer['id']).resolve()
    if folder.parent != root:
        raise ValueError('观察包路径错误')
    bundle = json.loads((folder / 'bundle.json').read_text())
    age = (time.time_ns() - bundle['oldest_source_ns']) / 1e9
    if (not 0 <= age <= max_age_s or not bundle['stationary']
            or (not_before_ns is not None and bundle['oldest_source_ns'] < not_before_ns)):
        raise ObservationNotReady('三视角观察过期、仍在运动或早于本次检查点；等待新包')
    status = ctx.value('piper_status')
    q = status['measured_state']['values']
    if status['owner_generation'] != bundle['owner_generation'] or max(abs(a-b) for a,b in zip(q,bundle['measured14'])) > .003:
        raise ValueError('观察后机械臂状态改变；重新观察')
    files = {}
    for camera, image in bundle['images'].items():
        file = folder / image['file']
        if file.resolve().parent != folder or hashlib.sha256(file.read_bytes()).hexdigest() != image['sha256']:
            raise ValueError('观察包图像完整性错误')
        files[camera] = str(file)
    # Status RPC and integrity I/O can take time. Validate age again at return,
    # so a packet cannot silently expire while this helper is checking it.
    age = (time.time_ns() - bundle['oldest_source_ns']) / 1e9
    if not 0 <= age <= max_age_s:
        raise ObservationNotReady('三视角观察在检查期间过期；重新采集并审阅')
    return {'bundle_id': bundle['id'], 'images': files, 'age_s': age, 'span_s': bundle['span_s'],
            'source_ns': bundle['oldest_source_ns'],
            'remaining_live_s': max_age_s - age,
            'measured14': q, 'questions': CHECKPOINTS, 'motion_permit': False,
            'geometry_note': 'Use a NEW piper_observe(require_depth=True) head frame for pixel_to_position. Wrist pixels have no metric mapping.'}


def observe_when_ready(ctx, root='runtime/stream/multiview', timeout_s=2.5):
    """Wait only for NEW stationary evidence, never repeat a motion or age old pixels."""
    if not 0 < timeout_s <= 2.5:
        raise ValueError('Observation wait must be bounded to at most 2.5 seconds')
    after_ns = time.time_ns()
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            result = observe(ctx, root, not_before_ns=after_ns)
            if time.monotonic() > deadline:
                raise ObservationNotReady('观察等待超过时限；未下发任何动作')
            return result
        except ObservationNotReady:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise
            time.sleep(min(.1, remaining))


def fuse(phase, evidence):
    """Require specific, attributed evidence; disagreement never becomes a majority vote."""
    required = CHECKPOINTS[phase]
    missing = []; negative = []; conflicts = []
    for camera, facts in required.items():
        row = evidence.get(camera, {})
        if not row.get('frame'):
            missing.append(camera + ': frame missing')
        for fact in facts:
            value = row.get('facts', {}).get(fact, 'unsure')
            if value == 'no': negative.append(camera + ':' + fact)
            elif value != 'yes': missing.append(camera + ':' + fact)
    names = set().union(*(set(v.get('facts', {})) for v in evidence.values() if isinstance(v, dict)))
    for fact in names:
        values = {v.get('facts', {}).get(fact) for v in evidence.values() if isinstance(v, dict)}
        if 'yes' in values and 'no' in values: conflicts.append(fact)
    return {'ok': not (missing or negative or conflicts), 'phase': phase, 'missing': missing,
            'negative': negative, 'conflicts': conflicts, 'evidence': evidence,
            'next_action': 'fresh measured-state and scene check' if not (missing or negative or conflicts) else 'inspect the named view/fact; reobserve or request help; no dispatch',
            'motion_permit': False}


def latest_segments(root='runtime/stream/sam3'):
    """Optional SAM3 snapshot; explicitly historical, never a motion prerequisite."""
    root = Path(root).resolve()
    if not (root / 'latest.json').is_file():
        return {'available': False, 'reason': 'No SAM3 snapshot requested; inspect original views directly'}
    pointer = json.loads((root / 'latest.json').read_text())
    folder = (root / pointer['id']).resolve()
    if folder.parent != root: raise ValueError('SAM3 result path escaped root')
    result = json.loads((folder / 'result.json').read_text())
    result['age_s'] = (time.time_ns()-result['source_created_ns'])/1e9
    result['historical_snapshot'] = True
    return result
