"""One fresh observation, one asynchronous teacher review, always no_action.

The formal owner's transport is released immediately. Long API review is an
explicit historical, stationary shadow window, never executable freshness.
This module has no SDK, CAN, operator-command or nonempty-action output path.
"""
from __future__ import annotations

import argparse
import asyncio
import json
from dataclasses import asdict
from pathlib import Path
import signal
import sys
import time
import uuid

import numpy as np
from PIL import Image

from .backend import OwnerObserver, PiperXPolicyBackend, array_sha256, write_json
from .contracts import (CAMERA_ROLES, FIELDS, UNITS, STATE_SCHEMA_ID,
                        CandidateEnvelope, PiperContract, _thaw, require, vector_schema)
from .description import PiperDescription
from .description_config import mast_configuration
from .review import PiperReviewer, ReviewEnvelope, make_binding, proposal_sha256
from .teacher_client import PiperTeacherClient, TeacherClientError, BOTTLE_TASK, STUDENT_TASK, SUPPORTED_GOALS

REVIEW_WINDOW_MS = 60000.0
STATE_TOLERANCE = 0.001
EXECUTION_FRESHNESS_MS = 300.0
PREVIEW_ENVELOPE = CandidateEnvelope(max_joint_delta_from_measured_rad=0.10,
    max_joint_step_rad=0.03, max_gripper_delta_m=0.02)


def owner_state14(snapshot, task):
    OwnerObserver.validate(snapshot, task)
    state = snapshot.get('robot_state', {})
    require(state.get('available') is True and state.get('names') == list(FIELDS)
            and state.get('units') == list(UNITS) and state.get('schema_id') == STATE_SCHEMA_ID,
            'owner API state schema unavailable or changed')
    values = np.asarray(state.get('values'))
    require(values.shape == (14,) and values.dtype.kind in 'fi' and np.isfinite(values).all(),
            'owner API state must be finite state14')
    count = snapshot['robot_status'].get('observations')
    require(type(count) is int and count >= 0, 'owner observation counter missing')
    return values.astype(np.float64)


def stationary_window(before, after, observation, task):
    first, last = owner_state14(before, task), owner_state14(after, task)
    require(before['robot_status']['generation'] == after['robot_status']['generation'],
            'owner generation changed during historical review')
    require(after['robot_status']['observations'] > before['robot_status']['observations'],
            'owner feedback counter did not advance during review')
    delta = max(float(np.max(np.abs(first - observation.state14))),
                float(np.max(np.abs(last - observation.state14))),
                float(np.max(np.abs(last - first))))
    require(delta <= STATE_TOLERANCE, 'owner state changed during historical review')
    return {'owner_feedback_counter_advanced': True,
            'max_state_change_rad_or_m': delta, 'limit_rad_or_m': STATE_TOLERANCE,
            'continuous_stationarity_proven': False,
            'sampled_before_after_stationarity_only': True}


def no_action_payload(*, request_id, status, inference_executed=False):
    return {'result_kind': 'no_action', 'actions': np.empty((0, 14), dtype=np.float32),
            'action_schema': vector_schema(action=True), 'action_frequency_hz': 30,
            'policy_metadata': {'inference_executed': bool(inference_executed),
                'transport_receipt': {'mode': 'hybrid_one_shot_shadow', 'request_id': request_id,
                    'status': status, 'dispatch_authorized': False,
                    'hardware_execution_confirmed': False},
                'execute_prefix_steps': 0, 'hardware_execution_confirmed': False}}


class HybridShadowModel:
    def __init__(self, backend, owner, reviewer, teacher, *, output, user_goal=BOTTLE_TASK, geometry=None,
                 verification_scope='task_review', previous_result=None):
        require(backend.task == STUDENT_TASK and user_goal in SUPPORTED_GOALS,
                'this reviewed integration pins separate canonical conditioning and user goal')
        self.backend, self.owner, self.reviewer, self.teacher = backend, owner, reviewer, teacher
        self.output, self.user_goal = Path(output), user_goal
        self.geometry = geometry
        require(verification_scope in ('task_review', 'bounded_transport_canary'), 'invalid verification scope')
        self.verification_scope = verification_scope
        self.previous_result = previous_result
        self.step_id = 1 if previous_result is not None else 0
        self.attempted = False
        self.request_id = None
        self.pending = None
        self.status = 'awaiting_first_fresh_observation'
        self.consumed = set()
        write_json(self.output / 'run.json', {'mode': 'hybrid_one_shot_shadow',
            'student_conditioning': backend.task, 'user_goal': user_goal,
            'verification_scope': verification_scope,
            'identical_task_to_original_method': False,
            'student_identity': backend.identity, 'teacher_max_calls': 1,
            'hardware_execution_confirmed': False, 'dispatch_authorized': False,
            'fresh_input_max_age_ms': EXECUTION_FRESHNESS_MS,
            'historical_stationary_review_window_ms': REVIEW_WINDOW_MS,
            'candidate_preview_envelope': asdict(PREVIEW_ENVELOPE),
            'started_ns': time.time_ns()})

    def get_capabilities(self):
        return {'robot_type': 'piper_x_dual', 'mode': 'hybrid_one_shot_shadow',
                'no_action': True, 'teacher_review_only': True,
                'hardware_write_allowed': False, 'teacher_max_calls': 1}

    async def reset(self):
        # A logical owner reset never rearms the one-shot API budget.
        if not self.attempted:
            await self.backend.student.reset(trial_id='junqi-hybrid-shadow')
        return {'physical_reset': False, 'one_shot_rearmed': False}

    async def infer(self, wire):
        if self.attempted:
            return no_action_payload(request_id=self.request_id, status=self.status)
        self.attempted = True  # Reserve before any await, including simultaneous callers.
        self.request_id = uuid.uuid4().hex
        received_ns = time.time_ns()
        try:
            observation = self.backend.observe(wire)
            self.status = 'background_review_scheduled'
            self.pending = asyncio.create_task(self._review(observation, received_ns))
        except Exception as exc:
            self.status = 'HOLD_input_rejected'
            write_json(self.output / 'teacher-error.json', {'status': self.status,
                'completed': True, 'error_type': type(exc).__name__, 'error': str(exc),
                'teacher_called': False, 'received_ns': received_ns,
                'dispatch_authorized': False, 'hardware_execution_confirmed': False})
        return no_action_payload(request_id=self.request_id, status=self.status)

    async def _snapshot(self, name):
        started_ns = time.time_ns()
        value = await asyncio.to_thread(self.owner.snapshot)
        completed_ns = time.time_ns()
        write_json(self.output / (name + '.json'), {'request_started_ns': started_ns,
            'response_received_ns': completed_ns, 'state': value,
            'source': 'fresh read of existing operator API; not device timestamp'})
        return value

    async def _advanced_snapshot(self, name, before, *, timeout_s=2.0):
        """Await a new owner publication; HTTP reads do not refresh cached status.

        Do not relax execution freshness. This only bounds the historical
        shadow stationarity check, and rejects intervening faults or movement.
        """
        initial = owner_state14(before, self.backend.task)
        count = before['robot_status']['observations']
        deadline = time.monotonic() + timeout_s
        attempts = []
        while True:
            current = await self._snapshot(name)
            state = owner_state14(current, self.backend.task)
            status = current['robot_status']
            require(status['generation'] == before['robot_status']['generation'],
                    'owner generation changed while waiting for publication')
            require(status['observations'] >= count, 'owner observation counter regressed')
            require(float(np.max(np.abs(state - initial))) <= STATE_TOLERANCE,
                    'owner state changed while waiting for publication')
            attempts.append({'received_ns': time.time_ns(),
                             'observations': status['observations']})
            write_json(self.output / (name + '-publication-wait.json'),
                       {'timeout_s': timeout_s, 'attempts': attempts})
            if status['observations'] > count:
                return current
            require(time.monotonic() < deadline,
                    'owner feedback counter did not advance before publication deadline')
            await asyncio.sleep(min(.05, max(0, deadline - time.monotonic())))

    async def _review(self, obs, received_ns):
        teacher_called = False
        student_executed = False
        phase = 'owner_before'
        try:
            before = await self._snapshot('owner-before')
            before_state = owner_state14(before, self.backend.task)
            require(float(np.max(np.abs(before_state - obs.state14))) <= STATE_TOLERANCE,
                    'initial owner state differs from captured observation')
            phase = 'student'
            payload = await self.backend.propose(obs)
            student_executed = True
            raw = payload['actions']
            diagnostics, effective = await asyncio.to_thread(self.backend.diagnostics, obs, raw)
            np.savez_compressed(self.output / 'observation.npz', state14=obs.state14,
                                **{key: obs.images[key] for key in CAMERA_ROLES})
            np.savez_compressed(self.output / 'actions.npz', raw_actions=raw,
                                owner_boundary_preview=effective)
            for role, pixels in obs.images.items():
                Image.fromarray(pixels).save(self.output / (role + '.png'))
            write_json(self.output / 'student-diagnostics.json', diagnostics)
            write_json(self.output / 'observation.json', {'sequence': obs.sequence,
                'timestamp_ns': obs.timestamp_ns, 'received_ns': received_ns,
                'age_at_acceptance_ms': obs.age_ms, 'state14': obs.state14.tolist(),
                'camera_sha256': {key: array_sha256(obs.images[key]) for key in CAMERA_ROLES},
                'student_conditioning': self.backend.task, 'user_goal': self.user_goal,
                'student_request_binding': payload.get('_gap_binding'),
                'student_latency_ms': payload.get('latency_ms')})
            phase = 'proposal_validation'
            proposal = self.backend.contract.validate_candidate(effective,
                schema=vector_schema(action=True), frequency_hz=30,
                measured_state14=obs.state14, prefix_steps=16)
            # All v9 joints are checked before asking the teacher as well as by reviewer.
            if self.backend.description is not None:
                self.backend.description.validate_candidate_limits(proposal.actions)
                self.backend.description.validate_candidate_limits(proposal.actions.astype(np.float32))
            binding = make_binding(self.request_id, obs, proposal, step_id=self.step_id)
            normalization = {'raw_actions_sha256': array_sha256(raw),
                'effective_preview_sha256': array_sha256(effective),
                'review_proposal_sha256': proposal_sha256(proposal),
                'owner_boundary_adjustments': diagnostics['owner_boundary_adjustments'],
                'source': 'existing owner normalize_model_action_boundaries, preview only',
                'model_output_preserved': True, 'hardware_execution_confirmed': False}
            write_json(self.output / 'normalization-receipt.json', normalization)
            phase = 'owner_pre_teacher'
            pre_teacher = await self._advanced_snapshot('owner-pre-teacher', before)
            stationary_window(before, pre_teacher, obs, self.backend.task)
            context = {'task': self.backend.task, 'user_goal': self.user_goal, 'step_id': self.step_id,
                'verification_scope': self.verification_scope,
                'measured_state14': obs.state14.tolist(), 'student_actions': proposal.actions.tolist(),
                'current_flange': diagnostics['current_flange'],
                'student_flange_trajectory': diagnostics['student_flange_trajectory'],
                'description_context': diagnostics.get('description_context'),
                'previous_result': self.previous_result,
                'executed_steps': 0 if self.previous_result is None else self.previous_result['sdk_dispatch_completed_count'],
                'normalization_receipt': normalization,
                'candidate_preview_envelope': asdict(PREVIEW_ENVELOPE),
                'observation_timestamp_ns': obs.timestamp_ns,
                'scene_age_before_api_ms': (time.time_ns() - obs.timestamp_ns) / 1e6,
                'stationary_shadow_reviewwindow_ms': REVIEW_WINDOW_MS,
                'owner_generation': before['robot_status']['generation'],
                'both_hardware_write_gates_off': True, 'hardware_write_count': 0,
                'depth_status': 'not_bound_to_this_RGB_observation',
                'tcp_status': 'flange_only_no_calibrated_grasp_tcp',
                'dispatch_authorized': False}
            write_json(self.output / 'teacher-context.json', {'binding': binding.as_dict(),
                                                             'context': context})
            if self.geometry is not None:
                phase = 'shadow_scene_geometry'
                evidence = await asyncio.to_thread(self.geometry, obs)
                context['scene_geometry'] = evidence
                context['depth_status'] = 'separately_timestamped_recent_stereo_with_photometric_agreement'
                context['tcp_status'] = 'explicit_CAD_nominal_preview_not_physically_calibrated'
                write_json(self.output / 'scene-geometry.json', evidence)
                write_json(self.output / 'teacher-context.json', {'binding': binding.as_dict(),
                                                                 'context': context})
            phase = 'teacher_api'
            teacher_called = True
            api_started_ns = time.time_ns()
            api_result = await asyncio.to_thread(self.teacher.review, binding=binding.as_dict(),
                                                images=obs.images, context=context)
            api_completed_ns = time.time_ns()
            write_json(self.output / 'teacher-api-output.json', api_result)
            phase = 'historical_review_validation'
            decision = api_result['decision']
            validation = {'status': 'HOLD', 'dispatch_authorized': False,
                          'candidate_preview_envelope': asdict(PREVIEW_ENVELOPE)}
            age_ms = (time.time_ns() - obs.timestamp_ns) / 1e6
            try:
                after = await self._advanced_snapshot('owner-after', pre_teacher)
                stationary = stationary_window(before, after, obs, self.backend.task)
                now_ns = time.time_ns()
                age_ms = (now_ns - obs.timestamp_ns) / 1e6
                if decision.get('mode') in ('edit', 'eef'):
                    require(self.verification_scope != 'bounded_transport_canary',
                            'transport canary permits only unchanged student or stop')
                    require(type(decision.get('steps')) is int and decision['steps'] == 1,
                            'initial teacher correction preview requires exactly one step')
                result = self.reviewer.review(obs, proposal, decision,
                    envelope=ReviewEnvelope(binding, now_ns, REVIEW_WINDOW_MS,
                                            frozenset(self.consumed)),
                    candidate_envelope=PREVIEW_ENVELOPE)
                validation.update(status='accepted_offline_review', mode=result.mode,
                    assessment_gate=result.assessment_gate, metadata=_thaw(result.metadata),
                    stationary_window=stationary, candidate_present=result.candidate is not None)
                if result.candidate is not None:
                    np.save(self.output / 'reviewed_candidate_preview.npy', result.candidate.actions)
                    validation['candidate_metrics'] = dict(result.candidate.metrics)
                    validation['candidate_prefix_steps'] = result.candidate.prefix_steps
            except Exception as exc:
                validation.update(error_type=type(exc).__name__, error=str(exc))
            finally:
                self.consumed.add(binding.request_id)
            self.status = 'review_completed_no_action'
            write_json(self.output / 'teacher-result.json', {'completed': True,
                'status': self.status, 'request_id': self.request_id,
                'decision': decision, 'api_report': api_result['report'],
                'review_validation': validation, 'teacher_called': True,
                'student_inference_executed': True, 'received_ns': received_ns,
                'observation_timestamp_ns': obs.timestamp_ns,
                'api_started_ns': api_started_ns, 'api_completed_ns': api_completed_ns,
                'completed_ns': time.time_ns(), 'observation_age_at_review_ms': age_ms,
                'review_scope': 'historical stationary shadow review only',
                'review_window_ms': REVIEW_WINDOW_MS,
                'execution_freshness_budget_ms': EXECUTION_FRESHNESS_MS,
                'execution_observation_stale': age_ms > EXECUTION_FRESHNESS_MS,
                'transport_result': 'no_action_float32_0x14',
                'hardware_execution_confirmed': False, 'dispatch_authorized': False,
                'task_success_evaluated': False})
        except asyncio.CancelledError:
            self.status = 'HOLD_review_cancelled'
            write_json(self.output / 'teacher-error.json', {'completed': True, 'status': self.status,
                'phase': phase, 'teacher_called': teacher_called,
                'student_inference_executed': student_executed,
                'dispatch_authorized': False, 'hardware_execution_confirmed': False})
            raise
        except Exception as exc:
            self.status = 'HOLD_review_error'
            write_json(self.output / 'teacher-error.json', {'completed': True, 'status': self.status,
                'phase': phase, 'error_type': type(exc).__name__, 'error': str(exc),
                'teacher_report': exc.report if isinstance(exc, TeacherClientError) else None,
                'teacher_called': teacher_called, 'student_inference_executed': student_executed,
                'observation_timestamp_ns': obs.timestamp_ns, 'completed_ns': time.time_ns(),
                'dispatch_authorized': False, 'hardware_execution_confirmed': False})

    async def close(self):
        if self.pending is not None and not self.pending.done():
            self.pending.cancel()
            try:
                await self.pending
            except asyncio.CancelledError:
                pass


async def run(args):
    import yaml
    runtime = args.runtime.resolve(strict=True)
    sys.path.insert(0, str(runtime))
    from client_server.ws.protocol.client import PolicyEvalClient, PolicyEvalClientConfig
    from client_server.ws.model_server import PolicyServer, PolicyServerConfig, deployment_identity_from_config
    require(args.task == STUDENT_TASK and args.user_goal in SUPPORTED_GOALS, 'pinned task/goal required')
    require(args.student_url == 'ws://127.0.0.1:19013' and args.port == 19014,
            'hybrid shadow requires isolated loopback ports 19013/19014')
    expected = deployment_identity_from_config(yaml.safe_load(args.student_config.read_text()))
    student = PolicyEvalClient(PolicyEvalClientConfig(url=args.student_url,
        evaluation_id='junqi-hybrid-one-shot-shadow', max_connect_attempts=1,
        connect_timeout_s=5, request_timeout_s=15, compression=None, proxy=None))
    server = model = None
    try:
        hello = await student.connect(handshake=True)
        require(hello is not None and hello.payload.get('deployment') == expected,
                'PiPER student HELLO differs from pinned deployment identity')
        args.output.mkdir(parents=True, exist_ok=False)
        write_json(args.output / 'student-hello.json', hello.payload)
        contract = PiperContract.from_runtime(runtime)
        description = PiperDescription(contract, args.description_assets)
        mast = mast_configuration(args.mast_joint_rad)
        backend = PiperXPolicyBackend(contract, student, args.output / 'student-adapter',
            task=args.task, identity=expected, max_observation_age_ms=EXECUTION_FRESHNESS_MS,
            description=description, mast_joint_rad=mast['joint_position_rad'], mast_metadata=mast)
        reviewer = PiperReviewer(contract, backend.kinematics, gate_module_path=args.gate_module,
                                 description=description)
        geometry = None
        if args.depth_output is not None or args.packet_root is not None or args.nominal_tcp is not None:
            require(all(path is not None for path in (args.depth_output, args.packet_root, args.nominal_tcp)),
                    'depth output, packet root and explicit nominal TCP must be supplied together')
            from functools import partial
            from .scene_geometry import geometry_context
            geometry = partial(geometry_context, depth_output=args.depth_output,
                               packet_root=args.packet_root, tcp_path=args.nominal_tcp,
                               regions_path=args.scene_regions)
        model = HybridShadowModel(backend, OwnerObserver(args.owner_url, args.task), reviewer,
                                  PiperTeacherClient(), output=args.output, user_goal=args.user_goal,
                                  geometry=geometry, verification_scope=args.verification_scope,
                                  previous_result=json.loads(args.previous_execution_report.read_text())
                                      if args.previous_execution_report is not None else None)
        server = PolicyServer(model, PolicyServerConfig(host='127.0.0.1', port=args.port,
            compression=None, deployment_identity=expected))
        await server.start()
        write_json(args.output / 'ready.json', {'status': 'ready', 'url': server.url,
            'mode': 'hybrid_one_shot_shadow', 'no_action': True, 'teacher_called': False})
        print('READY hybrid_one_shot_shadow ws://127.0.0.1:19014', flush=True)
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stop.set)
        try:
            await asyncio.wait_for(stop.wait(), timeout=args.duration)
        except asyncio.TimeoutError:
            pass
    finally:
        if server is not None:
            await server.stop()
        if model is not None:
            await model.close()
        await student.close()


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--student-config', type=Path, required=True)
    parser.add_argument('--student-url', default='ws://127.0.0.1:19013')
    parser.add_argument('--owner-url', default='http://127.0.0.1:18091')
    parser.add_argument('--port', type=int, default=19014)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--task', required=True)
    parser.add_argument('--user-goal', required=True)
    parser.add_argument('--duration', type=int, default=180)
    parser.add_argument('--depth-output', type=Path)
    parser.add_argument('--packet-root', type=Path)
    parser.add_argument('--nominal-tcp', type=Path)
    parser.add_argument('--scene-regions', type=Path)
    parser.add_argument('--previous-execution-report', type=Path)
    parser.add_argument('--verification-scope', choices=['task_review', 'bounded_transport_canary'],
                        default='task_review')
    parser.add_argument('--description-assets', type=Path,
        default=root / 'assets/robots/piper/global-d435-fixed-v12')
    parser.add_argument('--gate-module', type=Path,
        default=root / 'hybrid_rollout/robodojo/robodojo_server/gate_assessment.py')
    parser.add_argument('--mast-joint-rad', type=float, default=None)
    args = parser.parse_args()
    require(1 <= args.duration <= 180, 'duration must be 1..180 seconds')
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
