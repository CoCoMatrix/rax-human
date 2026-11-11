"""One immutable GPT-reviewed student prefix via the original XPolicy owner.

Fresh execution observation is distinct from the historical teacher observation.
No SDK, device, operator command or automatic second prefix is implemented here.
"""
import argparse
import asyncio
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
import uuid

import numpy as np

from .backend import write_json, array_sha256
from .contracts import (PiperContract, PiperObservation, CandidateEnvelope, CAMERA_ROLES,
                        vector_schema, require, _thaw)
from .description import PiperDescription
from .kinematics import PiperKinematics
from .review import PiperReviewer, ReviewBinding, ReviewEnvelope
from .scene_geometry import scene_agreement
from .teacher_client import MODEL, STUDENT_TASK, BOTTLE_PLACE_TASK

PARENT = Path('/home/ubuntu/code_junqi/model-store/models/pi05/pen-trash-dual-piper-real-h32-v1-ema-v1/step-9999')
ENVELOPE = CandidateEnvelope(max_joint_delta_from_measured_rad=.10, max_joint_step_rad=.03,
                             max_gripper_delta_m=.02)
SERIALS = dict(camera_head='262522073779', camera_left='262422071560', camera_right='262522070284')
EVIDENCE_FILES = ('teacher-result.json', 'teacher-context.json', 'actions.npz', 'observation.npz',
                  'observation.json', 'reviewed_candidate_preview.npy', 'student-hello.json')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_review(source, runtime, gap):
    source = Path(source).resolve(strict=True)
    result = json.loads((source / 'teacher-result.json').read_text())
    context = json.loads((source / 'teacher-context.json').read_text())
    observation_meta = json.loads((source / 'observation.json').read_text())
    require(result['api_report']['returned_model'] == MODEL
            and result['api_report']['verification_scope'] == 'bounded_transport_canary'
            and result['api_report']['user_goal'] == BOTTLE_PLACE_TASK,
            'exact model, goal and bounded canary review required')
    require(result['decision']['mode'] == 'student'
            and result['review_validation']['status'] == 'accepted_offline_review'
            and result['review_validation']['candidate_present'] is True,
            'student prefix must have passed teacher and pure reviewer')
    require(result['decision']['binding'] == context['binding'], 'review binding changed')
    contract = PiperContract.from_runtime(runtime)
    description = PiperDescription(contract, gap / 'assets/robots/piper/global-d435-fixed-v12')
    with np.load(source / 'observation.npz', allow_pickle=False) as data:
        anchor = data['state14'].copy()
        images = {key: data[key].copy() for key in CAMERA_ROLES}
    require(observation_meta['task'] == STUDENT_TASK if 'task' in observation_meta
            else observation_meta['student_conditioning'] == STUDENT_TASK, 'observation task changed')
    obs = PiperObservation(anchor, images, STUDENT_TASK, observation_meta['timestamp_ns'],
        observation_meta['sequence'], contract.robot_id, contract.identity, 0., {})
    with np.load(source / 'actions.npz', allow_pickle=False) as data:
        effective = data['owner_boundary_preview'].copy()
    proposal = contract.validate_candidate(effective, schema=vector_schema(action=True),
        frequency_hz=30, measured_state14=anchor, prefix_steps=16)
    reviewer = PiperReviewer(contract, PiperKinematics(contract),
        gate_module_path=gap / 'hybrid_rollout/robodojo/robodojo_server/gate_assessment.py', description=description)
    checked = reviewer.review(obs, proposal, result['decision'],
        envelope=ReviewEnvelope(ReviewBinding(**context['binding']), result['completed_ns'], 60000., frozenset()),
        candidate_envelope=ENVELOPE)
    selected = np.load(source / 'reviewed_candidate_preview.npy', allow_pickle=False)
    require(np.array_equal(selected, checked.candidate.actions), 'reviewed prefix bytes changed')
    actions = selected.astype(np.float32)
    contract.validate_candidate(actions, schema=vector_schema(action=True), frequency_hz=30,
        measured_state14=anchor, prefix_steps=len(actions), envelope=ENVELOPE)
    description.validate_candidate_limits(actions)
    return result, obs, actions, contract, description


def verify(bundle, runtime, gap):
    bundle = Path(bundle).resolve(strict=True)
    manifest = json.loads((bundle / 'manifest.json').read_text())
    require(manifest['schema'] == 'junqi.reviewed_pi05_prefix.v1', 'wrong reviewed prefix manifest')
    for name, sha in manifest['source_evidence_sha256'].items():
        require(name in EVIDENCE_FILES and digest(Path(manifest['review_source']) / name) == sha,
                'source review evidence changed')
    require(set(manifest['source_evidence_sha256']) == set(EVIDENCE_FILES), 'incomplete review evidence')
    require(digest(bundle / 'dataset-info.json') == 'fa1099ed392f3c981d39c43e08dd83819b4ec53ebcffb75b43012625bc347d97',
            'training dataset metadata hash mismatch')
    info = json.loads((bundle / 'dataset-info.json').read_text())
    names = [*(f'left_j{i}' for i in range(1,7)), 'left_gripper_width',
             *(f'right_j{i}' for i in range(1,7)), 'right_gripper_width']
    require(info['fps'] == 30 and info['features']['action']['names'] == names
            and info['features']['observation.state']['names'] == names, 'dataset state/action order mismatch')
    sys.path.insert(0, str(runtime))
    from XPolicyLab.policy.Pi_05.piper_pen_trash_h32_v1_bundle import verify_piper_pen_trash_h32_v1_bundle
    parent = verify_piper_pen_trash_h32_v1_bundle(PARENT)
    result, obs, actions, contract, description = load_review(manifest['review_source'], runtime, gap)
    require(manifest['steps'] == len(actions) and manifest['actions_sha256'] == array_sha256(actions),
            'selected action prefix identity mismatch')
    require(manifest['robot_identity'] == dict(contract.identity), 'robot contract changed')
    require(manifest['parent_bundle'] == str(PARENT), 'parent checkpoint changed')
    require(manifest['verification_scope'] == 'bounded_transport_canary', 'not a task rollout release')
    return manifest, result, obs, actions, contract, description


def freeze(source, bundle, dataset_info, runtime, gap):
    result, obs, actions, contract, _ = load_review(source, runtime, gap)
    bundle.mkdir(parents=True, exist_ok=False)
    (bundle / 'dataset-info.json').write_bytes(dataset_info.read_bytes())
    manifest = {'schema': 'junqi.reviewed_pi05_prefix.v1', 'review_source': str(source.resolve()),
        'source_evidence_sha256': {name: digest(source / name) for name in EVIDENCE_FILES},
        'steps': len(actions), 'actions_sha256': array_sha256(actions),
        'parent_bundle': str(PARENT), 'robot_identity': dict(contract.identity),
        'task': STUDENT_TASK, 'user_goal': BOTTLE_PLACE_TASK,
        'verification_scope': 'bounded_transport_canary', 'teacher_response_id': result['api_report']['response_id'],
        'created_ns': time.time_ns(), 'hardware_canary_passed': False, 'task_success': False,
        'fresh_execution_binding_required': True, 'one_shot': True}
    write_json(bundle / 'manifest.json', manifest)
    verify(bundle, runtime, gap)
    return manifest


class ReviewedPrefixModel:
    def __init__(self, bundle, runtime, gap, *, receipt_root, skip_image_comparison=False):
        self.manifest, self.result, self.reference, self.actions, self.contract, self.description = verify(bundle, runtime, gap)
        self.root = Path(receipt_root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.attempted = False
        self.skip_image_comparison = skip_image_comparison

    def get_capabilities(self):
        return {'mode': 'bounded_reviewed_pi05_prefix', 'one_shot': True,
                'max_targets': len(self.actions), 'hardware_owner': 'formal_xpolicy_only'}

    async def reset(self, **kwargs):
        return {'physical_reset': False, 'one_shot_rearmed': False}

    async def infer(self, wire):
        require(not self.attempted, 'reviewed prefix already consumed')
        self.attempted = True
        # Process-level and durable one-shot reservation before any response.
        with (self.root / 'attempt.json').open('x') as f:
            json.dump({'time_ns': time.time_ns(), 'hardware_dispatch_confirmed': False}, f)
        try:
            obs = self.contract.validate_observation(wire, max_age_ms=300.)
            require(obs.instruction == STUDENT_TASK, 'execution task differs from reviewed task')
            require(time.time_ns() - self.result['completed_ns'] < 600_000_000_000,
                    'review older than ten-minute canary budget; obtain a new review')
            require(np.max(np.abs(obs.state14 - self.reference.state14)) <= .001,
                    'robot moved since teacher review')
            metadata = _thaw(obs.metadata)
            np.savez(self.root / 'execution-observation.npz', state14=obs.state14,
                     **{role: obs.images[role] for role in CAMERA_ROLES})
            camera_frames = metadata.get('camera_frames', {})
            now = time.time_ns()
            for role, serial in SERIALS.items():
                frame = camera_frames.get(role, {})
                stamp = frame.get('source_timestamp_ns')
                require(frame.get('camera_serial') == serial and type(stamp) is int
                        and 0 <= now - stamp <= 300_000_000, 'individual camera stale or wrong identity')
            agreement = {}
            for role in (() if self.skip_image_comparison else CAMERA_ROLES):
                try:
                    agreement[role] = scene_agreement(obs.images[role], self.reference.images[role])
                except Exception as exc:
                    raise ValueError('scene changed in ' + role + ': ' + str(exc)) from exc
            generation = metadata.get('owner_generation')
            session = metadata.get('policy_session_id')
            require(type(generation) is int and generation >= 0 and isinstance(session, str) and len(session) == 32,
                    'fresh policy generation/session required')
            self.contract.validate_candidate(self.actions, schema=vector_schema(action=True), frequency_hz=30,
                measured_state14=obs.state14, prefix_steps=len(self.actions), envelope=ENVELOPE)
            self.description.validate_candidate_limits(self.actions)
            release = {'schema': 'junqi.piper.bounded_review_release.v1', 'release_id': uuid.uuid4().hex,
                'teacher_request_id': self.result['request_id'], 'generation': generation,
                'execution_session_id': session, 'steps': len(self.actions),
                'review_observation_timestamp_ns': self.reference.timestamp_ns,
                'execution_observation_timestamp_ns': obs.timestamp_ns,
                'expires_ns': obs.timestamp_ns + 300_000_000 + int((len(self.actions)-1)*1e9/30),
                'anchor14': obs.state14.tolist(), 'targets14': self.actions.tolist(),
                'scene_check': ('fresh_three_cameras_and_state_match_image_comparison_disabled_by_operator'
                    if self.skip_image_comparison else 'three_camera_photometric_and_state_match')}
            require(time.time_ns() - obs.timestamp_ns <= 250_000_000, 'fresh release processing budget exceeded')
            write_json(self.root / 'execution-release.json', {'release': release, 'scene_agreement': agreement,
                'image_comparison_disabled_by_explicit_user_request': self.skip_image_comparison,
                'camera_frames': camera_frames, 'original_review_preserved': True,
                'hardware_dispatch_confirmed': False, 'task_success': False})
            np.savez(self.root / 'execution-observation.npz', state14=obs.state14,
                                **{role: obs.images[role] for role in CAMERA_ROLES})
            # Recheck after evidence IO; never restamp the original observation.
            require(time.time_ns() - obs.timestamp_ns <= 280_000_000, 'release expired during evidence persistence')
            return {'actions': self.actions.copy(), 'action_schema': vector_schema(action=True),
                'action_frequency_hz': 30, 'policy_metadata': {'execute_prefix_steps': len(self.actions),
                'review_release': release, 'hardware_execution_confirmed': False}}
        except Exception as exc:
            write_json(self.root / 'release-rejected.json', {'error_type': type(exc).__name__,
                'error': str(exc), 'time_ns': time.time_ns(), 'hardware_dispatch_confirmed': False})
            raise


async def serve(args):
    sys.path.insert(0, str(args.runtime))
    from client_server.ws.model_server import PolicyServer, PolicyServerConfig
    model = ReviewedPrefixModel(args.bundle, args.runtime, args.gap, receipt_root=args.receipts,
                                skip_image_comparison=args.skip_image_comparison)
    server = PolicyServer(model, PolicyServerConfig(host='127.0.0.1', port=args.port, compression=None,
        deployment_identity={'mode': 'bounded_reviewed_pi05_prefix', 'manifest_sha256': digest(args.bundle / 'manifest.json')}))
    await server.start()
    print('READY reviewed-prefix ' + server.url, flush=True)
    stop=asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, stop.set)
    try:
        await asyncio.wait_for(stop.wait(), args.duration)
    except asyncio.TimeoutError:
        pass
    finally:
        await server.stop()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('command', choices=['freeze','verify','serve'])
    p.add_argument('--bundle', type=Path, required=True)
    p.add_argument('--runtime', type=Path, default=Path('/home/ubuntu/code_junqi/xpolicylab-robot-runtime'))
    p.add_argument('--gap', type=Path, default=Path('/home/ubuntu/code_junqi/GPT-as-Policy'))
    p.add_argument('--source', type=Path)
    p.add_argument('--dataset-info', type=Path)
    p.add_argument('--receipts', type=Path)
    p.add_argument('--port', type=int, default=19015)
    p.add_argument('--duration', type=int, default=300)
    p.add_argument('--skip-image-comparison', action='store_true')
    a=p.parse_args()
    if a.command == 'freeze':
        print(json.dumps(freeze(a.source,a.bundle,a.dataset_info,a.runtime,a.gap)))
    elif a.command == 'verify':
        manifest,*_=verify(a.bundle,a.runtime,a.gap)
        print(json.dumps({'verified':True,'steps':manifest['steps'],'hardware_canary_passed':False}))
    else:
        require(a.port==19015 and 1<=a.duration<=600, 'bounded isolated service required')
        asyncio.run(serve(a))


if __name__ == '__main__':
    main()
