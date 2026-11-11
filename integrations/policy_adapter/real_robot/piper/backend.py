"""PiPER observation, Pi0.5 proposal and formal XPolicy submission adapter.

This module never opens robot devices. A submission is not a hardware receipt.
The current runner requires the formal owner to remain in READY_NO_WRITE;
learned-policy hardware qualification is a separate deployment stage.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time
from typing import Any
import urllib.request
import uuid

import numpy as np
from PIL import Image

from .contracts import (CAMERA_ROLES, FIELDS, ContractError, PiperContract,
                        require, runtime_module, vector_schema)
from .kinematics import PiperKinematics


def array_sha256(value: np.ndarray) -> str:
    header = json.dumps({'dtype': value.dtype.str, 'shape': value.shape}, sort_keys=True)
    return hashlib.sha256(header.encode() + b'\0' + value.tobytes(order='C')).hexdigest()


def write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + '.partial')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def pose_json(pose) -> dict:
    return {'coordinate_frame': pose.coordinate_frame,
            'endpoint_links': list(pose.endpoint_links),
            'collision_certified': False,
            **{arm: {'position_m': pose.positions_m[i].tolist(),
                     'quaternion_wxyz': pose.quaternions_wxyz[i].tolist()}
               for i, arm in enumerate(('left', 'right'))}}


class OwnerObserver:
    """Read-only view of the existing operator API; no command methods."""
    def __init__(self, url: str, task: str):
        require(url.startswith('http://127.0.0.1:'), 'owner API must be loopback')
        self.url, self.task = url.rstrip('/'), task

    def snapshot(self) -> dict:
        with urllib.request.urlopen(self.url + '/api/state', timeout=2) as response:
            value = json.load(response)
        self.validate(value, self.task)
        return value

    @staticmethod
    def validate(value: dict, task: str) -> None:
        robot = value.get('robot_status', {})
        require(value.get('task') == task, 'owner task differs from model task')
        require(value.get('control_source') == 'keyboard', 'unexpected control source')
        require(value.get('write_enabled') is False, 'shadow requires UI write gate off')
        require(robot.get('hardware_write_enabled') is False, 'shadow requires owner write gate off')
        require(robot.get('hardware_write_count') == 0, 'shadow observed a hardware write')
        require(robot.get('status') == 'READY_NO_WRITE', 'owner is not ready for no-write shadow')
        require(type(robot.get('generation')) is int and robot['generation'] >= 0,
                'missing or invalid owner generation')
        require(not value.get('pigey', {}).get('active'), 'unexpected Pigey takeover')
        require(robot.get('expert_candidates', 0) == 0, 'unexpected expert candidate')


class PiperXPolicyBackend:
    """Replace simulator observation/proposal/FK/IK interfaces with PiPER ones.

`observe` accepts one real ObservationFrame, without reset or homing.
`propose` uses the pinned PiPER student. `execute` packages its raw actions for
the existing formal graph. Only that graph's owner can accept/write a target;
the returned metadata deliberately says hardware_execution_confirmed=False.
"""
    def __init__(self, contract: PiperContract, student, output: Path, *, task: str,
                 identity: dict, max_observation_age_ms: float = 300.0,
                 description=None, mast_joint_rad: float | None = None,
                 mode: str = 'pi05_only_shadow', mast_metadata: dict | None = None):
        require(mode in ('pi05_only_shadow', 'offline_replay'), 'unsupported backend mode')
        self.mode = mode
        self.contract, self.student = contract, student
        self.output, self.task, self.identity = output, task, identity
        self.max_observation_age_ms = max_observation_age_ms
        self.kinematics = PiperKinematics(contract)
        self.description, self.mast_joint_rad = description, mast_joint_rad
        self.mast_metadata = dict(mast_metadata or {})
        if description is not None:
            require(description.contract is contract, 'description must use the same robot contract')
        self.previous_sequence = None
        self.request_count = 0
        self.used_request_ids = set()
        self.submitted_sequences = set()
        self.output.mkdir(parents=True, exist_ok=False)
        write_json(self.output / 'run.json', {
            'schema': 'junqi.gap.piper.shadow.v1', 'started_ns': time.time_ns(),
            'mode': mode, 'teacher_called': False,
            'student_identity': identity, 'robot_identity': dict(contract.identity),
            'task': task, 'hardware_writer': 'existing formal xpolicy owner only',
            'policy_frequency_hz': 30, 'action_horizon': 32, 'execute_prefix_steps': 16,
            'observation_max_age_ms': max_observation_age_ms,
            'task_success_evaluated': False,
            'description_identity': dict(description.identity) if description is not None else None,
            'mast_joint_rad': mast_joint_rad,
            'mast_angle_source': self.mast_metadata.get('source',
                'unspecified' if mast_joint_rad is None else 'operator_configured'),
            'mast_configuration': self.mast_metadata,
        })

    def observe(self, wire: dict):
        obs = self.contract.validate_observation(
            wire, max_age_ms=self.max_observation_age_ms,
            previous_sequence=self.previous_sequence)
        require(obs.instruction == self.task, 'observation task changed')
        require(all(image.shape == (720, 1280, 3) for image in obs.images.values()),
                'checkpoint camera profile requires original 1280x720 RGB')
        self.previous_sequence = obs.sequence
        return obs

    async def propose(self, observation):
        require(self.mode == 'pi05_only_shadow', 'offline replay cannot request model inference')
        # Preserve every original PiPER field; no ARX reshaping, gripper scaling,
        # image compression or simulation state enters the student's input.
        response = await self.student.infer(observation.to_policy_observation())
        payload = dict(response.payload)
        actions = payload.get('actions')
        require(isinstance(actions, np.ndarray) and actions.shape == (32, 14)
                and actions.dtype == np.float32 and np.isfinite(actions).all(),
                'student must return finite float32[32,14] actions')
        require(payload.get('action_schema') == vector_schema(action=True),
                'student action schema mismatch')
        require(payload.get('action_frequency_hz') == 30, 'student frequency mismatch')
        payload['_gap_binding'] = {'request_id': uuid.uuid4().hex,
            'observation_sequence': observation.sequence,
            'observation_timestamp_ns': observation.timestamp_ns,
            'task': observation.instruction,
            'state_sha256': array_sha256(observation.state14),
            'action_sha256': array_sha256(actions)}
        return payload

    def diagnostics(self, obs, actions: np.ndarray) -> tuple[dict, np.ndarray]:
        """Keep raw predictions and the owner's explicit boundary preview separate.

        Normalization below is preview-only. Raw predictions go to the formal
        owner unchanged. A failed check is evidence, not permission to weaken it.
        """
        result = {'raw_sha256': array_sha256(actions), 'raw_shape': list(actions.shape),
                  'raw_finite': True, 'raw_gripper_range_m': {
                      'left': [float(actions[:, 6].min()), float(actions[:, 6].max())],
                      'right': [float(actions[:, 13].min()), float(actions[:, 13].max())]},
                  'normalization_is_preview_only': True, 'hardware_execution_confirmed': False}
        try:
            checked = self.contract.validate_candidate(actions, schema=vector_schema(action=True),
                frequency_hz=30, measured_state14=obs.state14, prefix_steps=16)
            result.update(raw_command_valid=True, raw_metrics=dict(checked.metrics))
        except ContractError as exc:
            result.update(raw_command_valid=False, raw_rejection=str(exc))
        safety = runtime_module(self.contract.repo_root, 'real_world.robots.piper_x_dual.safety')
        mapping = runtime_module(self.contract.repo_root, 'real_world.robots.piper_x_dual.mapping')
        boundary = self.contract.profile['live_takeover']['model_boundary_normalization']
        preview, adjustments = [], []
        for index, row in enumerate(actions):
            target, changes = safety.normalize_model_action_boundaries(mapping.action_dict(row),
                joint_limits_rad=self.contract.profile['body']['joint_limits_rad'],
                gripper_limits_m=tuple(self.contract.profile['body']['grippers']['left_gripper']['limits']),
                joint_tolerance_rad=float(boundary['joint_tolerance_rad'])) if boundary['enabled'] else (mapping.action_dict(row), ())
            preview.append([target[name] for name in FIELDS])
            adjustments.extend({'step': index, **change.as_dict()} for change in changes)
        effective = np.asarray(preview, dtype=np.float64)
        result['owner_boundary_adjustments'] = adjustments
        result['effective_preview_sha256'] = array_sha256(effective)
        result['effective_preview_exact_limits_valid'] = bool(
            np.all(effective >= self.contract.lower14) and np.all(effective <= self.contract.upper14))
        try:
            result['current_flange'] = pose_json(self.kinematics.forward(obs.state14))
        except ContractError as exc:
            result['current_fk_rejection'] = str(exc)
        trajectory = []
        for index, row in enumerate(effective):
            try:
                trajectory.append({'step': index, **pose_json(self.kinematics.forward(row))})
            except ContractError as exc:
                trajectory.append({'step': index, 'rejection': str(exc)})
        result['student_flange_trajectory'] = trajectory
        result['candidate_collision_certified'] = False
        if self.description is not None:
            result['description_context'] = self.description.frames(
                obs.state14, mast_joint_rad=self.mast_joint_rad)
            from .camera_geometry import apply_head_geometry
            result['description_context'] = apply_head_geometry(result['description_context'])
            result['description_context']['mast_configuration'] = self.mast_metadata
            try:
                self.description.validate_candidate_limits(effective)
                result['v9_intersection_preview_valid'] = True
            except ContractError as exc:
                result.update(v9_intersection_preview_valid=False, v9_preview_rejection=str(exc))
        return result, effective

    def execute(self, obs, payload: dict, *, owner_before: dict, owner_after: dict):
        """Return one single-use transport submission; never report it as motion."""
        require(self.mode == 'pi05_only_shadow', 'offline replay cannot submit actions')
        OwnerObserver.validate(owner_before, self.task)
        OwnerObserver.validate(owner_after, self.task)
        require(owner_before['robot_status']['generation'] == owner_after['robot_status']['generation'],
                'owner generation changed during inference')
        require(obs.sequence not in self.submitted_sequences, 'observation already submitted')
        require(payload.get('action_schema') == vector_schema(action=True)
                and payload.get('action_frequency_hz') == self.contract.frequency_hz,
                'proposal schema or frequency changed')
        binding = payload.get('_gap_binding', {})
        request_id = binding.get('request_id')
        require(isinstance(request_id, str) and len(request_id) == 32, 'missing proposal binding')
        require(binding.get('observation_sequence') == obs.sequence
                and binding.get('observation_timestamp_ns') == obs.timestamp_ns
                and binding.get('task') == obs.instruction
                and binding.get('state_sha256') == array_sha256(obs.state14)
                and binding.get('action_sha256') == array_sha256(payload['actions']),
                'proposal does not belong to this observation or has changed')
        require(request_id not in self.used_request_ids, 'proposal replay')
        self.used_request_ids.add(request_id)
        self.submitted_sequences.add(obs.sequence)
        self.request_count += 1
        directory = self.output / f'proposal_{self.request_count:04d}'
        directory.mkdir()
        actions = payload['actions']
        diagnostics, effective = self.diagnostics(obs, actions)
        np.savez_compressed(directory / 'observation.npz', state14=obs.state14,
                            **{key: obs.images[key] for key in CAMERA_ROLES})
        np.savez_compressed(directory / 'actions.npz', raw_actions=actions,
                            owner_boundary_preview=effective)
        for role, value in obs.images.items():
            Image.fromarray(value).save(directory / f'{role}.jpg', quality=90)
        record = {'request_id': request_id, 'index': self.request_count,
                  'observation_sequence': obs.sequence, 'observation_timestamp_ns': obs.timestamp_ns,
                  'observation_age_at_receive_ms': obs.age_ms,
                  'age_at_submission_ms': (time.time_ns() - obs.timestamp_ns) / 1e6,
                  'task': obs.instruction, 'measured_state14': obs.state14.tolist(),
                  'camera_sha256': {key: array_sha256(value) for key, value in obs.images.items()},
                  'camera_shapes': {key: list(value.shape) for key, value in obs.images.items()},
                  'student_identity': self.identity, 'student_latency_ms': payload.get('latency_ms'),
                  'owner_before': owner_before, 'owner_after': owner_after,
                  'submission_status': 'returned_to_policy_transport',
                  'hardware_execution_confirmed': False, **diagnostics}
        write_json(directory / 'proposal.json', record)
        result = dict(payload)
        result.pop('_gap_binding')
        result['policy_metadata'] = dict(payload.get('policy_metadata') or {},
            piper_backend_identity=dict(self.contract.identity), gap_request_id=request_id,
            gap_observation_sequence=obs.sequence, gap_raw_action_sha256=array_sha256(actions),
            gap_mode='pi05_only_shadow', execute_prefix_steps=16,
            hardware_execution_confirmed=False)
        if self.description is not None:
            result['policy_metadata']['robot_description_identity'] = dict(self.description.identity)
        write_json(self.output / 'progress.json', {'proposals_returned': self.request_count,
            'latest_request_id': request_id, 'latest_sequence': obs.sequence,
            'completed_ns': time.time_ns(), 'hardware_execution_confirmed': False})
        return result
