"""Observe/propose/review/execute/ACK loop; only the existing owner writes hardware."""
import argparse, asyncio, json, signal, sys, time, uuid
from dataclasses import asdict
from pathlib import Path
import numpy as np
from types import SimpleNamespace
from .backend import PiperXPolicyBackend, write_json
from .contracts import PiperContract, CAMERA_ROLES, require, vector_schema, _thaw
from .description import PiperDescription
from .description_config import mast_configuration
from .hybrid_shadow import no_action_payload, PREVIEW_ENVELOPE
from .review import PiperReviewer, ReviewEnvelope, make_binding
from .reviewed_prefix import SERIALS
from .teacher_client import PiperTeacherClient, STUDENT_TASK, BOTTLE_PLACE_TASK
from .scene_geometry import nominal_tcp


def completed_receipts(path, release):
    rows=[]
    if Path(path).exists():
        for line in Path(path).read_text().splitlines():
            try:r=json.loads(line)
            except json.JSONDecodeError:continue  # active writer's incomplete final line
            if r.get('event')=='release_rejected':raise ValueError('owner rejected a release')
            if r.get('event')=='sdk_dispatch_completed' and r.get('review_release_id')==release['release_id']:rows.append(r)
    require(len(rows)<=release['steps'], 'extra dispatch receipt')
    require([r['model_chunk_step_index'] for r in rows]==list(range(len(rows))), 'dispatch receipt order mismatch')
    require(all(r['teacher_request_id']==release['teacher_request_id'] and r['hardware_dispatch_confirmed'] is True for r in rows), 'wrong dispatch receipt identity')
    return rows


class ContinuousReviewModel:
    def __init__(self, backend, reviewer, teacher, root, journal, *, max_reviews=24, prior_canary=None, tcp=None, geometry=None):
        self.backend,self.reviewer,self.teacher=backend,reviewer,teacher
        self.root,self.journal=Path(root),Path(journal)
        self.max_reviews,self.prior_canary,self.tcp=max_reviews,prior_canary,tcp
        self.geometry=geometry;self.scene=None;self.geometry_wait_start=None
        self.phase='observing';self.pending=None;self.ready=None;self.active=None
        self.session=None;self.generation=None;self.reference=None
        self.reviews=0;self.executed_steps=0;self.consumed=set();self.previous=None
        self.correction=None
        self.next_review_time=time.monotonic()+2.;self.status()

    def status(self, **extra):
        write_json(self.root/'status.json', {'phase':self.phase,'reviews':self.reviews,
            'executed_steps':self.executed_steps,'time_ns':time.time_ns(),
            'student_prefix_max':15,'teacher_correction_protocol_max':5,
            'image_comparison_enabled':False,'task_success':False,**extra})

    def stop(self, reason, *, error=False):
        self.phase='error' if error else 'stopped';self.ready=None
        self.status(reason=reason);write_json(self.root/'stop.json',{'reason':reason,'error':error,
            'time_ns':time.time_ns(),'executed_steps':self.executed_steps,'task_success':False})

    def get_capabilities(self):
        return {'mode':'continuous_gpt_reviewed_pi05','student_prefix_max':15,
                'no_action_while_reviewing':True,'hardware_owner':'original_xpolicy_only'}

    async def reset(self, **kwargs):
        return {'physical_reset':False,'does_not_rearm_a_stopped_episode':True}

    def hold(self):
        result=no_action_payload(request_id='continuous-review',status=self.phase)
        result['policy_metadata']['transport_receipt']['mode']='continuous_review_wait'
        return result

    async def infer(self, wire):
        if self.phase in ('stopped','error'):return self.hold()
        try:
            obs=self.backend.observe(wire);meta=_thaw(obs.metadata);now=time.time_ns()
            for role,serial in SERIALS.items():
                f=meta.get('camera_frames',{}).get(role,{})
                require(f.get('camera_serial')==serial and type(f.get('source_timestamp_ns')) is int
                    and 0<=now-f['source_timestamp_ns']<=300_000_000,'camera stale or wrong identity')
            if self.session is None:self.session,self.generation=meta['policy_session_id'],meta['owner_generation']
            require(meta['policy_session_id']==self.session and meta['owner_generation']==self.generation,
                    'episode generation changed')
            if self.active is not None:
                rows=completed_receipts(self.journal,self.active)
                if len(rows)<self.active['steps']:
                    require(now<self.active['expires_ns']+1_000_000_000,'missing hardware ACK')
                    return self.hold()
                self.executed_steps+=len(rows)
                self.previous={'sdk_dispatch_completed_count':len(rows),
                    'teacher_request_id':self.active['teacher_request_id'],
                    'actual_applied_targets14':[r['applied14'] for r in rows],
                    'anchor_state14':self.active['anchor14'],'observed_state14':obs.state14.tolist(),
                    'max_observed_state_delta':float(np.max(np.abs(obs.state14-self.active['anchor14']))),
                    'object_success_verified':False,'pose_arrival_confirmed':False}
                write_json(self.round_dir/f'execution-feedback-{self.executed_steps:04d}.json',self.previous)
                self.active=None;self.phase='observing';self.next_review_time=time.monotonic()+.3
                if self.correction is not None:
                    c=self.correction
                    if c['next_index']<c['decision']['steps']:
                        index=c['next_index']
                        proposal_row=SimpleNamespace(actions=c['proposal'].actions[index:index+1])
                        action=self.reviewer._correct(obs,proposal_row,c['decision'],1).astype(np.float32)
                        self.backend.contract.validate_candidate(action,schema=vector_schema(action=True),
                            frequency_hz=30,measured_state14=obs.state14,prefix_steps=1,envelope=PREVIEW_ENVELOPE)
                        self.backend.description.validate_candidate_limits(action)
                        write_json(self.round_dir/f'correction-after-ack-{index}.json',{
                            'original_teacher_binding':c['binding'].as_dict(),'measured_state14':obs.state14.tolist(),
                            'after_actual_sdk_ack':True,'target14':action[0].tolist()})
                        self.ready=(obs,action,c['binding']);self.phase='ready';c['next_index']+=1
                    else:self.correction=None
                self.status()
            if self.pending is not None and not self.pending.done():return self.hold()
            if self.ready is not None:
                ref,actions,binding=self.ready
                require(np.max(np.abs(obs.state14-ref.state14))<=.001,'state changed during GPT review')
                self.backend.contract.validate_candidate(actions,schema=vector_schema(action=True),frequency_hz=30,
                    measured_state14=obs.state14,prefix_steps=len(actions),envelope=PREVIEW_ENVELOPE)
                release={'schema':'junqi.piper.bounded_review_release.v1','release_id':uuid.uuid4().hex,
                    'teacher_request_id':binding.request_id,'generation':self.generation,
                    'execution_session_id':self.session,'steps':len(actions),
                    'review_observation_timestamp_ns':binding.observation_timestamp_ns,
                    'execution_observation_timestamp_ns':obs.timestamp_ns,
                    'expires_ns':obs.timestamp_ns+300_000_000+int((len(actions)-1)*1e9/30),
                    'anchor14':obs.state14.tolist(),'targets14':actions.tolist(),
                    'scene_check':'fresh_three_cameras_and_state_match_image_comparison_disabled_by_operator'}
                write_json(self.round_dir/f'execution-release-{self.executed_steps:04d}.json',release)
                require(time.time_ns()-obs.timestamp_ns<=280_000_000,'execution observation expired')
                self.ready=None;self.active=release;self.phase='executing';self.status()
                return {'actions':actions.copy(),'action_schema':vector_schema(action=True),
                    'action_frequency_hz':30,'policy_metadata':{'execute_prefix_steps':len(actions),
                    'review_release':release,'hardware_execution_confirmed':False}}
            if self.phase=='observing' and time.monotonic()>=self.next_review_time:
                if self.reviews>=self.max_reviews:self.stop('review budget reached');return self.hold()
                if self.geometry is not None:
                    try:self.scene=await asyncio.to_thread(self.geometry,obs);self.geometry_wait_start=None
                    except (ValueError,OSError) as exc:
                        if self.geometry_wait_start is None:self.geometry_wait_start=time.monotonic()
                        require(time.monotonic()-self.geometry_wait_start<35,'fresh FFS depth unavailable: '+str(exc))
                        self.status(waiting_for='fresh FFS depth');return self.hold()
                self.reviews+=1;self.round_dir=self.root/f'round-{self.reviews:03d}';self.round_dir.mkdir()
                self.phase='reviewing';self.status();self.pending=asyncio.create_task(self.review(obs,self.round_dir))
            return self.hold()
        except Exception as exc:
            self.stop(type(exc).__name__+': '+str(exc),error=True);return self.hold()

    async def review(self, obs, out):
        try:
            payload=await self.backend.propose(obs)
            diagnostics,effective=await asyncio.to_thread(self.backend.diagnostics,obs,payload['actions'])
            np.savez(out/'observation.npz',state14=obs.state14,**obs.images)
            np.savez(out/'actions.npz',raw_actions=payload['actions'],owner_boundary_preview=effective)
            proposal=self.backend.contract.validate_candidate(effective,schema=vector_schema(action=True),
                frequency_hz=30,measured_state14=obs.state14,prefix_steps=15)
            binding=make_binding(uuid.uuid4().hex,obs,proposal,step_id=self.executed_steps)
            context={'task':STUDENT_TASK,'user_goal':BOTTLE_PLACE_TASK,'step_id':self.executed_steps,
                'verification_scope':'task_review','continuous_execution':True,
                'measured_state14':obs.state14.tolist(),'student_actions':proposal.actions.tolist(),
                'current_flange':diagnostics['current_flange'],
                'student_flange_trajectory':diagnostics['student_flange_trajectory'],
                'description_context':diagnostics.get('description_context'),
                'previous_result':self.previous,'prior_transport_canary':self.prior_canary,
                'executed_steps':self.executed_steps,'candidate_preview_envelope':asdict(PREVIEW_ENVELOPE),
                'nominal_grasp_tcp':self.tcp,'operator_geometry_choice':'Use the supplied nominal URDF; no calibration-board prerequisite.',
                'operator_accepted_nominal_geometry':True,'scene_geometry':self.scene,
                'known_nominal_geometry_disagreement':('The previous v9 geometry was inconsistent with observed table depth. The corrected v10 geometry is nominal and has not been physically calibrated.' if self.scene is not None else None),
                'camera_extrinsic_status':'nominal_urdf_unverified_mount_pose; do not treat image pixels as measured base-frame object coordinates',
                'perception_mode':'rgb_plus_ffs' if self.scene is not None else 'rgb_only',
                'requested_execution_scope':('one task-directed segment, then pause for operator observation' if self.max_reviews == 1 else 'bounded continuous task execution'),
                'segment_lengths':{'student':[1,15],'correction':[1,5]},
                'image_comparison_disabled_by_operator':True,
                'depth_status':'separately_timestamped_recent_FFS' if self.scene is not None else 'disabled_by_operator_no_depth_input',
                'dispatch_authorized':False,'owner_holds_during_review':True}
            write_json(out/'teacher-context.json',{'binding':binding.as_dict(),'context':context})
            api=await asyncio.to_thread(self.teacher.review,binding=binding.as_dict(),images=obs.images,context=context)
            write_json(out/'teacher-result.json',api)
            decision=api['decision']
            if decision['mode']=='student':require(1<=decision['steps']<=15,'original method student segment is 1..15')
            checked=self.reviewer.review(obs,proposal,decision,
                envelope=ReviewEnvelope(binding,time.time_ns(),(getattr(self.teacher,'timeout',180.)+10.)*1000 if getattr(self.teacher,'reasoning_effort','low')=='xhigh' else 60000.,frozenset(self.consumed)),
                candidate_envelope=PREVIEW_ENVELOPE)
            self.consumed.add(binding.request_id)
            if checked.candidate is None:self.stop(decision['reason']);return
            actions=np.asarray(checked.candidate.actions,dtype=np.float32)
            if decision['mode'] in ('edit','eef'):
                self.correction={'decision':decision,'proposal':proposal,'binding':binding,'next_index':1}
                actions=actions[:1]  # Remaining IK steps are recomputed from fresh feedback after each ACK.
            self.backend.description.validate_candidate_limits(actions)
            self.ready=(obs,actions,binding);self.phase='ready';self.status(decision_mode=decision['mode'],steps=len(actions))
        except Exception as exc:
            if hasattr(exc,'report'):write_json(out/'teacher-error.json',exc.report)
            self.stop(type(exc).__name__+': '+str(exc),error=True)


async def run(a):
    import yaml
    sys.path.insert(0,str(a.runtime))
    from client_server.ws.protocol.client import PolicyEvalClient,PolicyEvalClientConfig
    from client_server.ws.model_server import PolicyServer,PolicyServerConfig,deployment_identity_from_config
    expected=deployment_identity_from_config(yaml.safe_load((a.runtime/'policy/Pi_05/deploy_junqi_pi05_pen_trash_baseline.yml').read_text()))
    student=PolicyEvalClient(PolicyEvalClientConfig(url='ws://127.0.0.1:19013',evaluation_id='junqi-gap-continuous',
        max_connect_attempts=1,connect_timeout_s=5,request_timeout_s=15,compression=None,proxy=None))
    server=model=None
    try:
        hello=await student.connect(handshake=True);require(hello.payload.get('deployment')==expected,'student checkpoint identity mismatch')
        a.output.mkdir(parents=True,exist_ok=False);write_json(a.output/'student-hello.json',hello.payload)
        contract=PiperContract.from_runtime(a.runtime);gap=Path(__file__).resolve().parents[2]
        description=PiperDescription(contract,gap/'assets/robots/piper/global-d435-fixed-v12');mast=mast_configuration(None)
        backend=PiperXPolicyBackend(contract,student,a.output/'backend',task=STUDENT_TASK,identity=expected,
            description=description,mast_joint_rad=mast['joint_position_rad'],mast_metadata=mast)
        reviewer=PiperReviewer(contract,backend.kinematics,gate_module_path=gap/'hybrid_rollout/robodojo/robodojo_server/gate_assessment.py',description=description)
        prior=json.loads(a.prior_canary.read_text()) if a.prior_canary else None
        tcp=nominal_tcp(a.nominal_tcp) if a.nominal_tcp else None
        geometry=None
        if a.depth_output is not None:
            from functools import partial
            from .scene_geometry import geometry_context
            geometry=partial(geometry_context,depth_output=a.depth_output,packet_root=a.packet_root,
                tcp_path=a.nominal_tcp,regions_path=a.scene_regions,skip_image_comparison=True)
        model=ContinuousReviewModel(backend,reviewer,PiperTeacherClient(timeout=300,max_output_tokens=16384,reasoning_effort='xhigh'),
            a.output,a.journal,max_reviews=a.max_reviews,prior_canary=prior,tcp=tcp,geometry=geometry)
        server=PolicyServer(model,PolicyServerConfig(host='127.0.0.1',port=19016,compression=None,
            deployment_identity={**expected,'adapter':'junqi.continuous_gpt_reviewed_pi05.v1'}))
        await server.start();print('READY continuous review ws://127.0.0.1:19016',flush=True)
        stop=asyncio.Event()
        for sig in (signal.SIGINT,signal.SIGTERM):asyncio.get_running_loop().add_signal_handler(sig,stop.set)
        try:await asyncio.wait_for(stop.wait(),a.duration)
        except asyncio.TimeoutError:pass
    finally:
        if server:await server.stop()
        if model and model.pending and not model.pending.done():model.pending.cancel()
        await student.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('--runtime',type=Path,default=Path('/home/ubuntu/code_junqi/xpolicylab-robot-runtime'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--journal',type=Path,required=True)
    p.add_argument('--prior-canary',type=Path);p.add_argument('--nominal-tcp',type=Path)
    p.add_argument('--depth-output',type=Path);p.add_argument('--packet-root',type=Path);p.add_argument('--scene-regions',type=Path)
    p.add_argument('--max-reviews',type=int,default=24);p.add_argument('--duration',type=int,default=600)
    a=p.parse_args();require(1<=a.max_reviews<=64 and 1<=a.duration<=1200,'bounded session required');asyncio.run(run(a))
if __name__=='__main__':main()
